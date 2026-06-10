"""Concrete DashboardApplier: folder-per-project inside Grafana org 1.

All projects share the default org. Each project owns a folder (the manifest's
``grafana.folder``, defaulting to the project name) and the dashboards inside
it; the TimescaleDB datasource is shared across projects — normally the one
the server stack provisions (name ``Timescale``) — and is only created here
if it doesn't exist yet. Auth is the Grafana admin user (same credentials the
stack already has via ``GF_SECURITY_ADMIN_*``).

Idempotent throughout: the folder is created once and found by stable uid
thereafter; dashboards are pushed with ``overwrite: true`` keyed on their
stable uid, so re-applying never duplicates.
"""

import re
from urllib.parse import urlparse

import requests

from ..interfaces import DashboardApplier, GrafanaState, DatasourceState, DashboardState
from ..errors import RefusedDestructive, StateReadError
from . import dashboard_library as lib

HTTP_TIMEOUT = 15

# Everything lives in the default org. Pinned via the X-Grafana-Org-Id header
# so it holds regardless of which org the admin user last switched to.
ORG_ID = 1


def _datasource_uid(datasource_name):
    """Stable uid for a datasource the applier has to create itself. (The
    server-provisioned one keeps its own uid, e.g. ``timescale-main``.)"""
    slug = re.sub(r"[^a-zA-Z0-9-]", "-", datasource_name.lower()).strip("-")
    return f"ts-{slug}"[:40]


def _folder_uid(folder_name):
    """Stable Grafana folder uid derived from the project's folder name."""
    slug = re.sub(r"[^a-zA-Z0-9-]", "-", folder_name).strip("-")
    return f"folder-{slug}"[:40] or "folder"


def _parse_db_url(url):
    """postgresql://user:pass@host:port/db -> dict for a Grafana datasource."""
    p = urlparse(url)
    return {
        "host_port": f"{p.hostname}:{p.port or 5432}",
        "user": p.username or "",
        "password": p.password or "",
        "database": (p.path or "/").lstrip("/"),
    }


class GrafanaDashboardApplier(DashboardApplier):
    def __init__(self, base_url, admin_user, admin_password, db_url_template):
        self.base_url = base_url.rstrip("/")
        self.admin_user = admin_user
        self.admin_password = admin_password
        self.db = _parse_db_url(db_url_template)

    def _session(self):
        s = requests.Session()
        s.auth = (self.admin_user, self.admin_password)
        s.headers["Content-Type"] = "application/json"
        s.headers["X-Grafana-Org-Id"] = str(ORG_ID)
        return s

    def _url(self, path):
        return f"{self.base_url}{path}"

    # ------------------------------------------------------------------ read
    def read_state(self, folder_name, datasource_name, dashboard_uids):
        state = GrafanaState()
        s = self._session()

        # folder (blank = General, which always exists with uid "")
        folder_uid = _folder_uid(folder_name) if folder_name else ""
        if not folder_name:
            state.folder_exists = True
            state.folder_uid = ""
        else:
            try:
                r = s.get(self._url(f"/api/folders/{folder_uid}"), timeout=HTTP_TIMEOUT)
            except requests.RequestException as e:
                raise StateReadError(f"cannot reach Grafana: {e}") from e
            if r.status_code == 200:
                state.folder_exists = True
                state.folder_uid = folder_uid
            elif r.status_code not in (403, 404):
                raise StateReadError(
                    f"Grafana folder lookup failed ({r.status_code}): {r.text}"
                )

        # shared datasource, by name
        try:
            r = s.get(self._url(f"/api/datasources/name/{requests.utils.quote(datasource_name)}"),
                      timeout=HTTP_TIMEOUT)
        except requests.RequestException as e:
            raise StateReadError(f"cannot reach Grafana: {e}") from e
        if r.status_code == 200:
            ds = r.json()
            json_data = ds.get("jsonData") or {}
            state.datasource = DatasourceState(
                exists=True, uid=ds.get("uid"), name=ds.get("name"),
                database=json_data.get("database") or ds.get("database"),
            )

        # dashboards referenced by the manifest
        for uid in dashboard_uids:
            r = s.get(self._url(f"/api/dashboards/uid/{uid}"), timeout=HTTP_TIMEOUT)
            if r.status_code == 200:
                body = r.json().get("dashboard", {})
                state.dashboards[uid] = DashboardState(
                    uid=uid, exists=True, fingerprint=lib.fingerprint(body),
                )

        # every croptopus dashboard in the project's FOLDER, for orphan
        # detection. Scoped to the folder so one project's plan never lists
        # another project's dashboards.
        r = s.get(self._url("/api/search"), params={"tag": "croptopus", "type": "dash-db"},
                  timeout=HTTP_TIMEOUT)
        if r.status_code == 200:
            for item in r.json():
                if item.get("uid") and item.get("folderUid", "") == folder_uid:
                    state.all_dashboards[item["uid"]] = item.get("title", "")
        return state

    # ----------------------------------------------------------------- write
    def ensure_folder(self, folder_name):
        """Ensure the project's folder exists and return its uid. A blank
        folder name means the General folder (uid ``""``). Idempotent:
        created once, found by uid thereafter."""
        if not folder_name:
            return ""
        s = self._session()
        uid = _folder_uid(folder_name)
        r = s.get(self._url(f"/api/folders/{uid}"), timeout=HTTP_TIMEOUT)
        if r.status_code == 200:
            return uid
        r = s.post(self._url("/api/folders"),
                   json={"uid": uid, "title": folder_name}, timeout=HTTP_TIMEOUT)
        if r.status_code in (200, 201):
            return uid
        # created concurrently / name clash — re-check by uid before giving up
        r = s.get(self._url(f"/api/folders/{uid}"), timeout=HTTP_TIMEOUT)
        if r.status_code == 200:
            return uid
        raise StateReadError(f"folder provisioning failed ({r.status_code}): {r.text}")

    def ensure_datasource(self, datasource_name):
        """Reuse the shared datasource if present; create it only if missing.
        An existing datasource (e.g. the server-provisioned ``Timescale``) is
        never modified — it's owned by the stack's provisioning."""
        s = self._session()
        r = s.get(self._url(f"/api/datasources/name/{requests.utils.quote(datasource_name)}"),
                  timeout=HTTP_TIMEOUT)
        if r.status_code == 200:
            return r.json().get("uid")

        payload = {
            "uid": _datasource_uid(datasource_name),
            "name": datasource_name,
            "type": "postgres",
            "access": "proxy",
            "url": self.db["host_port"],
            "user": self.db["user"],
            "isDefault": True,
            "jsonData": {
                "database": self.db["database"],
                "sslmode": "disable",
                "timescaledb": True,
                "postgresVersion": 1600,
                "connMaxLifetime": 14400,
            },
            "secureJsonData": {"password": self.db["password"]},
        }
        r = s.post(self._url("/api/datasources"), json=payload, timeout=HTTP_TIMEOUT)
        if r.status_code not in (200, 201):
            raise StateReadError(f"datasource provisioning failed ({r.status_code}): {r.text}")
        return r.json().get("datasource", {}).get("uid") or payload["uid"]

    def ensure_dashboard(self, uid, dashboard_json, folder=None):
        s = self._session()
        # overwrite + folderUid enforces folder placement on every apply, so a
        # dashboard manually moved out of its project folder is moved back.
        folder_uid = self.ensure_folder(folder)
        payload = {"dashboard": dashboard_json, "overwrite": True, "folderUid": folder_uid}
        r = s.post(self._url("/api/dashboards/db"), json=payload, timeout=HTTP_TIMEOUT)
        if r.status_code not in (200, 201):
            raise StateReadError(f"dashboard push failed ({r.status_code}): {r.text}")
        return f"dashboard {uid} pushed"

    # ----------------------------------------------------------- destructive
    def delete_dashboard(self, uid):
        raise RefusedDestructive()

    def delete_datasource(self, uid):
        raise RefusedDestructive()
