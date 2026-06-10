"""Pure diff: manifest + live state -> Plan.

No writes, no I/O of its own (state is passed in). Each resource is classified
CREATE / UPDATE / NO_CHANGE / ORPHAN. In v1 ORPHANs are surfaced but never
scheduled for deletion. The Plan carries everything apply.py needs so apply
never recomputes the diff — it just enacts what the user saw.
"""

import re
import json
import hashlib

from .model import Plan, ResourceChange, Action
from . import state as state_mod
from .grafana import dashboard_library as lib


# --------------------------------------------------------------------------
# interval helpers for retention/compression drift
# --------------------------------------------------------------------------
_INTERVAL_UNIT_DAYS = {
    "year": 365, "years": 365, "mon": 30, "mons": 30, "month": 30, "months": 30,
    "week": 7, "weeks": 7, "day": 1, "days": 1,
}


def _interval_to_days(text):
    """Approx days from a Postgres interval string like '365 days' / '1 year'."""
    if not text:
        return None
    total = 0.0
    for amount, unit in re.findall(r"(\d+)\s*([a-z]+)", str(text).lower()):
        total += int(amount) * _INTERVAL_UNIT_DAYS.get(unit, 0)
    return total or None


def _same_days(manifest_days, live_interval):
    ld = _interval_to_days(live_interval)
    if manifest_days is None or ld is None:
        return False
    return abs(float(manifest_days) - ld) < 0.5


# --------------------------------------------------------------------------
# section diffs
# --------------------------------------------------------------------------
def _timescale_changes(manifest, live):
    db = manifest.get("database") or {}
    schema = db.get("schema")
    changes = []
    s = live.schema

    # schema
    if s.schema_exists:
        changes.append(ResourceChange(
            "timescale", "schema", schema, Action.NO_CHANGE.value,
            detail=f"schema {schema} already exists",
        ))
    else:
        changes.append(ResourceChange(
            "timescale", "schema", schema, Action.CREATE.value,
            detail=f"create per-project schema {schema}",
            sql=f'CREATE SCHEMA IF NOT EXISTS "{schema}";',
            op="create_schema", params={"schema": schema},
        ))

    # measurements hypertable (+ devices table + indexes, mirroring the server)
    qualified = f"{schema}.measurements"
    if s.measurements_table_exists and s.is_hypertable:
        changes.append(ResourceChange(
            "timescale", "table", qualified, Action.NO_CHANGE.value,
            detail=f"hypertable {qualified} already exists",
        ))
    else:
        changes.append(ResourceChange(
            "timescale", "table", qualified, Action.CREATE.value,
            detail=f"create measurements + devices tables in {schema}",
            sql=(f'CREATE TABLE IF NOT EXISTS "{schema}".measurements (\n'
                 "    time TIMESTAMPTZ NOT NULL, device_id TEXT NOT NULL,\n"
                 "    sensor_type TEXT NOT NULL, value DOUBLE PRECISION, unit TEXT, meta JSONB);\n"
                 f"SELECT create_hypertable('{qualified}', 'time', if_not_exists => TRUE);"),
            op="create_measurements_table", params={"schema": schema},
        ))

    # retention policy (manifest stores days)
    retention_days = db.get("retention_days")
    if retention_days:
        if _same_days(retention_days, s.retention_interval):
            changes.append(ResourceChange(
                "timescale", "retention_policy", qualified, Action.NO_CHANGE.value,
                detail=f"retention already {s.retention_interval}",
            ))
        else:
            action = Action.UPDATE if s.retention_interval else Action.CREATE
            changes.append(ResourceChange(
                "timescale", "retention_policy", qualified, action.value,
                detail=(f"set retention to {retention_days} days"
                        + (f" (was {s.retention_interval})" if s.retention_interval else "")),
                sql=f"SELECT add_retention_policy('{qualified}', INTERVAL '{retention_days} days', if_not_exists => TRUE);",
                op="set_retention_policy",
                params={"schema": schema, "days": retention_days},
            ))

    # compression policy (manifest stores days)
    compression_days = db.get("compression_after_days")
    if compression_days:
        if s.compression_enabled and _same_days(compression_days, s.compression_interval):
            changes.append(ResourceChange(
                "timescale", "compression_policy", qualified, Action.NO_CHANGE.value,
                detail=f"compression already {s.compression_interval}",
            ))
        else:
            action = Action.UPDATE if s.compression_interval else Action.CREATE
            changes.append(ResourceChange(
                "timescale", "compression_policy", qualified, action.value,
                detail=(f"set compression to {compression_days} days"
                        + (f" (was {s.compression_interval})" if s.compression_interval else "")),
                sql=(f'ALTER TABLE "{schema}".measurements SET (timescaledb.compress, '
                     "timescaledb.compress_segmentby = 'device_id, sensor_type');\n"
                     f"SELECT add_compression_policy('{qualified}', INTERVAL '{compression_days} days', if_not_exists => TRUE);"),
                op="set_compression_policy",
                params={"schema": schema, "days": compression_days},
            ))

    return changes


def _grafana_changes(manifest, live, warnings):
    g = manifest.get("grafana") or {}
    project_id = (manifest.get("project") or {}).get("id", "")
    schema = (manifest.get("database") or {}).get("schema")
    folder_name = state_mod.manifest_folder(manifest)
    ds_name = state_mod.manifest_datasource_name(manifest)
    gs = live.grafana
    changes = []

    # per-project folder in the shared org
    if gs.folder_exists:
        changes.append(ResourceChange(
            "grafana", "folder", folder_name, Action.NO_CHANGE.value,
            detail=f"folder '{folder_name}' already exists",
        ))
    else:
        changes.append(ResourceChange(
            "grafana", "folder", folder_name, Action.CREATE.value,
            detail=f"create Grafana folder '{folder_name}'",
            op="ensure_folder", params={"folder_name": folder_name},
        ))

    # shared datasource: reused if present (normally the server-provisioned
    # 'Timescale'); only created when missing, never modified
    if gs.datasource.exists:
        changes.append(ResourceChange(
            "grafana", "datasource", ds_name, Action.NO_CHANGE.value,
            detail=f"datasource '{ds_name}' present (uid {gs.datasource.uid})",
        ))
    else:
        changes.append(ResourceChange(
            "grafana", "datasource", ds_name, Action.CREATE.value,
            detail=f"create shared TimescaleDB datasource '{ds_name}'",
            op="ensure_datasource",
            params={"datasource_name": ds_name},
        ))

    # dashboards
    ds_uid_live = gs.datasource.uid if gs.datasource.exists else None
    referenced_uids = set()
    for entry in g.get("dashboards", []):
        did = entry.get("id")
        enabled = entry.get("enabled", False)
        uid = lib.dashboard_uid(project_id, did)

        if not enabled:
            changes.append(ResourceChange(
                "grafana", "dashboard", did, Action.SKIP.value,
                detail="disabled in manifest",
            ))
            continue

        if not lib.has_template(did):
            w = f"dashboard '{did}' has no template in the library; it will be skipped"
            warnings.append(w)
            changes.append(ResourceChange(
                "grafana", "dashboard", did, Action.SKIP.value,
                detail="no template in library", warning=w,
            ))
            continue

        referenced_uids.add(uid)
        params = {
            "uid": uid, "dashboard_id": did,
            "project_id": project_id, "schema": schema,
            "datasource_uid": ds_uid_live, "folder": folder_name,
        }
        live_dash = gs.dashboards.get(uid)
        if live_dash is None:
            changes.append(ResourceChange(
                "grafana", "dashboard", did, Action.CREATE.value,
                detail=f"create dashboard '{did}' (uid {uid})",
                op="ensure_dashboard", params=params,
            ))
        else:
            desired = lib.render(did, project_id, ds_uid_live, schema)
            if lib.fingerprint(desired) == live_dash.fingerprint:
                changes.append(ResourceChange(
                    "grafana", "dashboard", did, Action.NO_CHANGE.value,
                    detail=f"dashboard '{did}' up to date",
                ))
            else:
                changes.append(ResourceChange(
                    "grafana", "dashboard", did, Action.UPDATE.value,
                    detail=f"dashboard '{did}' content drifted; will be updated",
                    op="ensure_dashboard", params=params,
                ))

    # orphan dashboards: live croptopus dashboards in the project's folder
    # that the manifest no longer covers
    for uid, title in (gs.all_dashboards or {}).items():
        if uid not in referenced_uids:
            changes.append(ResourceChange(
                "grafana", "dashboard", title or uid, Action.ORPHAN.value,
                detail=f"live dashboard '{title or uid}' not in manifest (not deleted in v1)",
            ))

    return changes


# --------------------------------------------------------------------------
# entry point
# --------------------------------------------------------------------------
def diff(manifest, live, manifest_id, manifest_hash):
    """Build a Plan from a manifest and a LiveState (see state.read_live_state)."""
    warnings = []
    changes = []
    changes += _timescale_changes(manifest, live)
    changes += _grafana_changes(manifest, live, warnings)

    plan = Plan(
        manifest_id=manifest_id,
        manifest_hash=manifest_hash,
        customer_id=(manifest.get("customer") or {}).get("id", ""),
        project_id=(manifest.get("project") or {}).get("id", ""),
        schema=(manifest.get("database") or {}).get("schema", ""),
        folder_name=state_mod.manifest_folder(manifest),
        changes=changes,
        warnings=warnings,
        stubs=["nodes / sensors (firmware loop is separate)",
               "alerts (after dashboards are reliable)"],
    )
    plan.plan_hash = compute_plan_hash(plan)
    return plan


def compute_plan_hash(plan):
    """Stable hash of the plan's intent, bound to the manifest hash. Used to
    detect a stale plan at apply time (manifest changed after Plan)."""
    payload = {
        "manifest_hash": plan.manifest_hash,
        "changes": [
            {"section": c.section, "kind": c.kind, "name": c.name,
             "action": c.action, "op": c.op, "params": c.params}
            for c in plan.changes
        ],
    }
    blob = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(blob.encode()).hexdigest()[:16]
