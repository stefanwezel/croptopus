"""Grafana org + per-project service-account token management.

This is the "admin to bootstrap, scoped tokens for steady state" pattern:

1. The shared admin credentials create (or find) the org
   ``{customer.name} / {project.name}`` (multi-tenancy mechanism #2).
2. On first Apply for a project, a service account + token is created inside
   that org and stored in ``instance/secrets/{project_id}.json`` (gitignored).
3. Subsequent datasource/dashboard calls reuse that org-scoped token instead
   of admin auth.

Isolated here so the org/token model can be swapped (e.g. one Grafana per
project) without touching dashboard_applier.
"""

import os
import json
import datetime

import requests

from ..errors import StateReadError

SA_NAME = "croptopus-applier"
HTTP_TIMEOUT = 15


class GrafanaOrgManager:
    def __init__(self, base_url, admin_user, admin_password, secrets_dir):
        self.base_url = base_url.rstrip("/")
        self.admin_user = admin_user
        self.admin_password = admin_password
        self.secrets_dir = secrets_dir

    # -- admin session -----------------------------------------------------
    def _admin(self):
        s = requests.Session()
        s.auth = (self.admin_user, self.admin_password)
        s.headers["Content-Type"] = "application/json"
        return s

    def _url(self, path):
        return f"{self.base_url}{path}"

    # -- org ---------------------------------------------------------------
    def find_org(self, org_name):
        """Return org id or None. Read-only (used by state.py)."""
        s = self._admin()
        try:
            r = s.get(self._url(f"/api/orgs/name/{requests.utils.quote(org_name)}"),
                      timeout=HTTP_TIMEOUT)
        except requests.RequestException as e:
            raise StateReadError(f"cannot reach Grafana: {e}") from e
        if r.status_code == 404:
            return None
        if r.status_code == 200:
            return r.json().get("id")
        raise StateReadError(f"Grafana org lookup failed ({r.status_code}): {r.text}")

    def ensure_org(self, org_name):
        org_id = self.find_org(org_name)
        if org_id is not None:
            return org_id
        s = self._admin()
        r = s.post(self._url("/api/orgs"), json={"name": org_name}, timeout=HTTP_TIMEOUT)
        if r.status_code in (200, 201):
            return r.json().get("orgId")
        # someone created it concurrently
        org_id = self.find_org(org_name)
        if org_id is not None:
            return org_id
        raise StateReadError(f"could not create Grafana org ({r.status_code}): {r.text}")

    # -- per-project token -------------------------------------------------
    def _secrets_path(self, project_id):
        return os.path.join(self.secrets_dir, f"{project_id}.json")

    def _load_secret(self, project_id):
        path = self._secrets_path(project_id)
        if os.path.exists(path):
            with open(path) as f:
                return json.load(f)
        return None

    def _store_secret(self, project_id, data):
        os.makedirs(self.secrets_dir, exist_ok=True)
        path = self._secrets_path(project_id)
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        os.chmod(path, 0o600)

    def get_org_token(self, project_id, org_id):
        """Return an org-scoped service-account token, creating it on first
        use. Reused across applies via the gitignored secrets file."""
        secret = self._load_secret(project_id)
        if secret and secret.get("token") and secret.get("org_id") == org_id:
            return secret["token"]

        s = self._admin()
        # switch the admin user's active org so SA creation lands in it
        r = s.post(self._url(f"/api/user/using/{org_id}"), timeout=HTTP_TIMEOUT)
        if r.status_code != 200:
            raise StateReadError(f"could not switch to org {org_id}: {r.text}")

        sa_id = self._find_or_create_sa(s)
        # token keys can't be re-read; mint a fresh, uniquely-named one
        token_name = f"applier-{datetime.datetime.utcnow().strftime('%Y%m%d%H%M%S')}"
        r = s.post(self._url(f"/api/serviceaccounts/{sa_id}/tokens"),
                   json={"name": token_name}, timeout=HTTP_TIMEOUT)
        if r.status_code not in (200, 201):
            raise StateReadError(f"could not create SA token: {r.text}")
        token = r.json()["key"]
        self._store_secret(project_id, {
            "org_id": org_id,
            "sa_id": sa_id,
            "token_name": token_name,
            "token": token,
        })
        return token

    def _find_or_create_sa(self, s):
        r = s.get(self._url("/api/serviceaccounts/search"),
                  params={"query": SA_NAME}, timeout=HTTP_TIMEOUT)
        if r.status_code == 200:
            for sa in r.json().get("serviceAccounts", []):
                if sa.get("name") == SA_NAME:
                    return sa["id"]
        r = s.post(self._url("/api/serviceaccounts"),
                   json={"name": SA_NAME, "role": "Admin", "isDisabled": False},
                   timeout=HTTP_TIMEOUT)
        if r.status_code not in (200, 201):
            raise StateReadError(f"could not create service account: {r.text}")
        return r.json()["id"]

    # -- org-scoped session ------------------------------------------------
    def org_session(self, project_id, org_id):
        """A requests.Session authenticated as the project's service account."""
        token = self.get_org_token(project_id, org_id)
        s = requests.Session()
        s.headers["Authorization"] = f"Bearer {token}"
        s.headers["Content-Type"] = "application/json"
        return s
