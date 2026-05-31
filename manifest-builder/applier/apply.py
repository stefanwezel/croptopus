"""Execute a Plan idempotently.

Apply never recomputes the diff — it enacts exactly the changes the user saw on
the plan page. Each resource op is wrapped in try/except so a failure on
dashboard 3 of 5 doesn't stop 4 and 5; per-resource results are collected and
returned. All destructive ops are refused (none should appear in a v1 plan).
"""

from .model import Action, ACTIONABLE, ResourceResult, ApplyResult
from .errors import RefusedDestructive
from .grafana import dashboard_library as lib


def apply_plan(plan, schema_applier, dashboard_applier):
    results = []
    # the datasource uid, captured if we (re)provision it this run, so freshly
    # created dashboards can reference it without recomputing the plan
    created_ds_uid = None

    for change in plan.changes:
        action = Action(change.action)
        if action not in ACTIONABLE:
            results.append(ResourceResult(
                change.name, change.kind, change.action, "skipped", change.detail,
            ))
            continue

        if change.destructive:
            results.append(ResourceResult(
                change.name, change.kind, change.action, "error",
                "refused: destructive operations are not supported in v1",
            ))
            continue

        try:
            msg, created_ds_uid = _execute(
                change, schema_applier, dashboard_applier, created_ds_uid,
            )
            results.append(ResourceResult(
                change.name, change.kind, change.action, "ok", msg,
            ))
        except RefusedDestructive as e:
            results.append(ResourceResult(
                change.name, change.kind, change.action, "error", f"refused: {e}",
            ))
        except Exception as e:  # noqa: BLE001 - collect, don't abort the run
            results.append(ResourceResult(
                change.name, change.kind, change.action, "error", str(e),
            ))

    return ApplyResult(
        manifest_id=plan.manifest_id,
        manifest_hash=plan.manifest_hash,
        results=results,
    )


def _execute(change, schema_applier, dashboard_applier, created_ds_uid):
    """Run one op. Returns (message, possibly-updated created_ds_uid)."""
    op = change.op
    p = change.params

    if op == "create_schema":
        return schema_applier.create_schema(p["schema"]), created_ds_uid
    if op == "create_measurements_table":
        return schema_applier.create_measurements_table(p["schema"]), created_ds_uid
    if op == "set_retention_policy":
        return schema_applier.set_retention_policy(p["schema"], p["days"]), created_ds_uid
    if op == "set_compression_policy":
        return schema_applier.set_compression_policy(p["schema"], p["days"]), created_ds_uid

    if op == "ensure_org":
        org_id = dashboard_applier.ensure_org(p["org_name"])
        return f"org ensured (id {org_id})", created_ds_uid
    if op == "ensure_datasource":
        uid = dashboard_applier.ensure_datasource(
            p["org_name"], p["datasource_name"], p["schema"],
        )
        return f"datasource ensured (uid {uid})", uid
    if op == "ensure_dashboard":
        ds_uid = created_ds_uid or p.get("datasource_uid")
        if not ds_uid:
            raise RuntimeError("no datasource uid available for dashboard")
        body = lib.render(p["dashboard_id"], p["project_id"], ds_uid, p["schema"])
        msg = dashboard_applier.ensure_dashboard(p["org_name"], p["uid"], body)
        return msg, created_ds_uid

    raise RuntimeError(f"unknown op: {op!r}")
