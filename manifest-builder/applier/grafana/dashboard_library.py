"""The v1 hand-written dashboard template library.

Manifests reference dashboards by id (``overview``, ``per_station``,
``device_health``); each id maps to a JSON file in ``templates/``. At apply
time the applier substitutes the datasource uid and the project's Postgres
schema. If a manifest references an id that isn't in the library, the plan
surfaces a warning and apply skips it (never a hard error).
"""

import os
import re
import json
import copy
import hashlib

TEMPLATE_DIR = os.path.join(os.path.dirname(__file__), "templates")

# All projects share Grafana org 1, and dashboard uids are scoped per-org, so
# the stable "<project>-<id>" prefix is what keeps projects from colliding —
# and keeps re-applies idempotent (PUT by uid).
UID_MAX = 40


def available_ids():
    ids = []
    for fn in sorted(os.listdir(TEMPLATE_DIR)):
        if fn.endswith(".json"):
            ids.append(fn[:-len(".json")])
    return ids


def has_template(dashboard_id):
    return os.path.exists(os.path.join(TEMPLATE_DIR, dashboard_id + ".json"))


def _load_raw(dashboard_id):
    with open(os.path.join(TEMPLATE_DIR, dashboard_id + ".json")) as f:
        return json.load(f)


def dashboard_uid(project_id, dashboard_id):
    raw = f"{project_id}-{dashboard_id}"
    uid = re.sub(r"[^a-zA-Z0-9-]", "-", raw)
    return uid[:UID_MAX]


def render(dashboard_id, project_id, datasource_uid, schema, title=None):
    """Return a concrete dashboard JSON ready to PUT to Grafana.

    ``datasource_uid`` may be a placeholder when planning against infra where
    the datasource does not exist yet; apply re-renders with the real uid.
    """
    dash = _load_raw(dashboard_id)
    uid = dashboard_uid(project_id, dashboard_id)
    # Templates carry the literal placeholder as their title, so defaulting to
    # dash["title"] would leave "__TITLE__" in Grafana. Mirror the server's
    # naming style ("Croptopus Overview") with the project id prefixed.
    if not title:
        template_title = dash.get("title", "")
        if template_title and template_title != "__TITLE__":
            title = template_title
        else:
            title = f"{project_id} {dashboard_id}".replace("_", " ").title()

    serialized = json.dumps(dash)
    serialized = serialized.replace("__UID__", uid)
    serialized = serialized.replace("__TITLE__", title)
    serialized = serialized.replace("__DS_UID__", datasource_uid or "")
    serialized = serialized.replace("__SCHEMA__", schema)
    return json.loads(serialized)


# --------------------------------------------------------------------------
# Drift fingerprint
# --------------------------------------------------------------------------
# Grafana rewrites a saved dashboard: it stamps ``id``, ``version`` and
# ``updated`` and may add panel defaults. Comparing raw JSON would report drift
# on every apply. Instead we hash a small, stable projection of the fields we
# actually control: title + per-panel (type, title, datasource uid, queries).
def fingerprint(dashboard_json):
    dash = dashboard_json or {}
    panels = []
    for p in sorted(dash.get("panels", []), key=lambda x: x.get("id", 0)):
        ds = p.get("datasource") or {}
        targets = []
        for t in p.get("targets", []):
            targets.append({
                "refId": t.get("refId"),
                "rawSql": t.get("rawSql"),
                "format": t.get("format"),
            })
        panels.append({
            "type": p.get("type"),
            "title": p.get("title"),
            "ds_uid": ds.get("uid") if isinstance(ds, dict) else ds,
            "targets": targets,
        })
    projection = {
        "title": dash.get("title"),
        "refresh": dash.get("refresh"),
        "panels": panels,
    }
    blob = json.dumps(projection, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode()).hexdigest()[:16]
