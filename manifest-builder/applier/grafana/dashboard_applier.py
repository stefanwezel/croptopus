"""Concrete DashboardApplier: provisions the datasource + dashboards inside a
project's Grafana org, using the org-scoped service-account token.

Idempotent throughout: the datasource is created with a stable uid and updated
via PUT-by-uid; dashboards are pushed with ``overwrite: true`` keyed on their
stable uid, so re-applying never duplicates.
"""

import re
from urllib.parse import urlparse

import requests

from ..interfaces import DashboardApplier, GrafanaState, DatasourceState, DashboardState
from ..errors import RefusedDestructive, StateReadError
from . import dashboard_library as lib

HTTP_TIMEOUT = 15


def _datasource_uid(project_id):
    return f"ts-{project_id}"[:40]


def _folder_uid(folder_name):
    """Stable Grafana folder uid derived from the manifest's folder name."""
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
    def __init__(self, org_manager, project_id, project_db_url_template):
        self.org = org_manager
        self.project_id = project_id
        self.db = _parse_db_url(project_db_url_template)

    def _url(self, path):
        return f"{self.org.base_url}{path}"

    # ------------------------------------------------------------------ read
    def read_state(self, org_name, datasource_name, dashboard_uids):
        state = GrafanaState()
        org_id = self.org.find_org(org_name)
        if org_id is None:
            return state
        state.org_exists = True
        state.org_id = org_id

        s = self.org.org_session(self.project_id, org_id)
        # datasource
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
                schema=json_data.get("croptopusSchema"),
            )
        # dashboards referenced by the manifest
        for uid in dashboard_uids:
            r = s.get(self._url(f"/api/dashboards/uid/{uid}"), timeout=HTTP_TIMEOUT)
            if r.status_code == 200:
                body = r.json().get("dashboard", {})
                state.dashboards[uid] = DashboardState(
                    uid=uid, exists=True, fingerprint=lib.fingerprint(body),
                )
        # every croptopus dashboard in the org, for orphan detection
        r = s.get(self._url("/api/search"), params={"tag": "croptopus", "type": "dash-db"},
                  timeout=HTTP_TIMEOUT)
        if r.status_code == 200:
            for item in r.json():
                if item.get("uid"):
                    state.all_dashboards[item["uid"]] = item.get("title", "")
        return state

    # ----------------------------------------------------------------- write
    def ensure_org(self, org_name):
        return self.org.ensure_org(org_name)

    def ensure_datasource(self, org_name, datasource_name, schema):
        org_id = self.org.ensure_org(org_name)
        s = self.org.org_session(self.project_id, org_id)
        uid = _datasource_uid(self.project_id)
        payload = {
            "uid": uid,
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
                # the project schema is enforced by schema-qualified SQL in the
                # dashboard templates; recorded here for transparency/drift.
                "connMaxLifetime": 14400,
                "croptopusSchema": schema,
            },
            "secureJsonData": {"password": self.db["password"]},
        }
        # create, or update-by-uid if it already exists
        r = s.get(self._url(f"/api/datasources/uid/{uid}"), timeout=HTTP_TIMEOUT)
        if r.status_code == 200:
            r = s.put(self._url(f"/api/datasources/uid/{uid}"), json=payload,
                      timeout=HTTP_TIMEOUT)
        else:
            r = s.post(self._url("/api/datasources"), json=payload, timeout=HTTP_TIMEOUT)
        if r.status_code not in (200, 201):
            raise StateReadError(f"datasource provisioning failed ({r.status_code}): {r.text}")
        return uid

    def ensure_folder(self, s, folder_name):
        """Ensure the manifest's Grafana folder exists in the current org and
        return its uid. An empty/blank folder name means the org's General
        folder (uid ``""``). Idempotent: created once, found by uid thereafter."""
        if not folder_name:
            return ""
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

    def ensure_dashboard(self, org_name, uid, dashboard_json, folder=None):
        org_id = self.org.ensure_org(org_name)
        s = self.org.org_session(self.project_id, org_id)
        # overwrite + folderUid enforces folder placement on every apply, so a
        # dashboard manually moved out of its manifest folder is moved back.
        folder_uid = self.ensure_folder(s, folder)
        payload = {"dashboard": dashboard_json, "overwrite": True, "folderUid": folder_uid}
        r = s.post(self._url("/api/dashboards/db"), json=payload, timeout=HTTP_TIMEOUT)
        if r.status_code not in (200, 201):
            raise StateReadError(f"dashboard push failed ({r.status_code}): {r.text}")
        return f"dashboard {uid} pushed"

    # ----------------------------------------------------------- destructive
    def delete_dashboard(self, org_name, uid):
        raise RefusedDestructive()

    def delete_datasource(self, org_name, uid):
        raise RefusedDestructive()
