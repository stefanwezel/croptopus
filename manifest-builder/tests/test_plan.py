"""Unit tests for the pure plan logic (applier.plan.diff) with mocked state.

Covers CREATE / UPDATE / NO_CHANGE / ORPHAN / SKIP classification for the
database (Timescale) and grafana sections. No real infra is touched.
"""

import copy

from applier import plan as plan_mod
from applier import state as state_mod
from applier.model import Action
from applier.interfaces import (
    SchemaState, GrafanaState, DatasourceState, DashboardState,
)
from applier.grafana import dashboard_library as lib


DEMO = {
    "customer": {"id": "demo_farm", "name": "Demo Farm"},
    "project": {"id": "croptopus_dev", "name": "Croptopus Dev", "environment": "dev"},
    "database": {"schema": "croptopus", "retention_days": 730, "compression_after_days": 30},
    "grafana": {
        "folder": "Croptopus",
        "datasource_name": "croptopus_tsdb",
        "dashboards": [
            {"id": "overview", "enabled": True},
            {"id": "per_station", "enabled": True},
        ],
    },
}

ORG = "Demo Farm / Croptopus Dev"
PROJECT_ID = "croptopus_dev"
SCHEMA = "croptopus"


def _actions(plan, kind):
    return {c.name: c.action for c in plan.changes if c.kind == kind}


def _by_kind(plan, kind):
    return [c for c in plan.changes if c.kind == kind]


# --------------------------------------------------------------------------
# helpers to build live state
# --------------------------------------------------------------------------
def empty_live():
    return state_mod.LiveState(schema=SchemaState(), grafana=GrafanaState())


def in_sync_live(manifest=DEMO):
    ds_uid = "ts-" + PROJECT_ID
    schema = SchemaState(
        schema_exists=True, measurements_table_exists=True, is_hypertable=True,
        retention_interval="730 days",
        compression_enabled=True, compression_interval="30 days",
    )
    dashboards = {}
    all_dash = {}
    for entry in manifest["grafana"]["dashboards"]:
        if not entry["enabled"] or not lib.has_template(entry["id"]):
            continue
        uid = lib.dashboard_uid(PROJECT_ID, entry["id"])
        fp = lib.fingerprint(lib.render(entry["id"], PROJECT_ID, ds_uid, SCHEMA))
        dashboards[uid] = DashboardState(uid=uid, exists=True, fingerprint=fp)
        all_dash[uid] = entry["id"]
    grafana = GrafanaState(
        org_exists=True, org_id=2,
        datasource=DatasourceState(exists=True, uid=ds_uid, name="croptopus_tsdb",
                                   database="croptopus", schema=SCHEMA),
        dashboards=dashboards, all_dashboards=all_dash,
    )
    return state_mod.LiveState(schema=schema, grafana=grafana)


def make_plan(manifest, live):
    return plan_mod.diff(manifest, live, "demo", "hash123")


# --------------------------------------------------------------------------
# empty infra -> everything CREATE
# --------------------------------------------------------------------------
def test_empty_infra_all_create():
    plan = make_plan(DEMO, empty_live())

    ts = _actions(plan, "schema")
    assert ts["croptopus"] == Action.CREATE.value
    assert _actions(plan, "table")["croptopus.measurements"] == Action.CREATE.value
    assert _actions(plan, "retention_policy")["croptopus.measurements"] == Action.CREATE.value
    assert _actions(plan, "compression_policy")["croptopus.measurements"] == Action.CREATE.value

    assert _actions(plan, "org")[ORG] == Action.CREATE.value
    assert _actions(plan, "datasource")["croptopus_tsdb"] == Action.CREATE.value
    dash = _actions(plan, "dashboard")
    assert dash["overview"] == Action.CREATE.value
    assert dash["per_station"] == Action.CREATE.value

    # SQL is rendered for transparency
    schema_change = _by_kind(plan, "schema")[0]
    assert "CREATE SCHEMA IF NOT EXISTS" in schema_change.sql
    assert not plan.has_destructive
    # 4 timescale (schema, table, retention, compression) + org + datasource + 2 dashboards
    assert plan.actionable_count == 8


def test_empty_infra_plan_hash_stable():
    p1 = make_plan(DEMO, empty_live())
    p2 = make_plan(copy.deepcopy(DEMO), empty_live())
    assert p1.plan_hash == p2.plan_hash


# --------------------------------------------------------------------------
# in-sync infra -> everything NO_CHANGE (the idempotency invariant)
# --------------------------------------------------------------------------
def test_in_sync_all_no_change():
    plan = make_plan(DEMO, in_sync_live())
    actionable = [c for c in plan.changes
                  if c.action in (Action.CREATE.value, Action.UPDATE.value)]
    assert actionable == [], f"expected no changes, got {[(c.kind, c.name) for c in actionable]}"
    assert plan.actionable_count == 0

    assert _actions(plan, "schema")["croptopus"] == Action.NO_CHANGE.value
    assert _actions(plan, "dashboard")["overview"] == Action.NO_CHANGE.value
    assert _actions(plan, "dashboard")["per_station"] == Action.NO_CHANGE.value


# --------------------------------------------------------------------------
# UPDATE: retention days changed + a dashboard drifted
# --------------------------------------------------------------------------
def test_retention_change_is_update():
    live = in_sync_live()
    live.schema.retention_interval = "180 days"   # manifest says 730
    plan = make_plan(DEMO, live)
    assert _actions(plan, "retention_policy")["croptopus.measurements"] == Action.UPDATE.value
    rc = [c for c in plan.changes if c.kind == "retention_policy"][0]
    assert rc.params["days"] == 730


def test_dashboard_content_drift_is_update():
    live = in_sync_live()
    overview_uid = lib.dashboard_uid(PROJECT_ID, "overview")
    live.grafana.dashboards[overview_uid].fingerprint = "deadbeefdeadbeef"
    plan = make_plan(DEMO, live)
    assert _actions(plan, "dashboard")["overview"] == Action.UPDATE.value
    # the other dashboard is still in sync
    assert _actions(plan, "dashboard")["per_station"] == Action.NO_CHANGE.value


# --------------------------------------------------------------------------
# datasource re-scoped to a different schema -> UPDATE
# --------------------------------------------------------------------------
def test_datasource_schema_drift_is_update():
    live = in_sync_live()
    live.grafana.datasource.schema = "some_other_schema"
    plan = make_plan(DEMO, live)
    assert _actions(plan, "datasource")["croptopus_tsdb"] == Action.UPDATE.value


# --------------------------------------------------------------------------
# ORPHAN: a live dashboard the manifest no longer references
# --------------------------------------------------------------------------
def test_orphan_dashboard_surfaced_not_deleted():
    live = in_sync_live()
    live.grafana.all_dashboards["croptopus_dev-legacy"] = "Legacy board"
    plan = make_plan(DEMO, live)
    orphans = [c for c in plan.changes if c.action == Action.ORPHAN.value]
    assert any("Legacy board" in c.name for c in orphans)
    # orphans never become apply ops
    assert all(c.op is None for c in orphans)


# --------------------------------------------------------------------------
# SKIP: disabled dashboard, and unknown-template dashboard (warning, no error)
# --------------------------------------------------------------------------
def test_disabled_and_unknown_dashboards_skipped():
    manifest = copy.deepcopy(DEMO)
    manifest["grafana"]["dashboards"] = [
        {"id": "overview", "enabled": True},
        {"id": "per_station", "enabled": False},          # disabled
        {"id": "does_not_exist", "enabled": True},        # no template
    ]
    plan = make_plan(manifest, empty_live())
    dash = _actions(plan, "dashboard")
    assert dash["overview"] == Action.CREATE.value
    assert dash["per_station"] == Action.SKIP.value
    assert dash["does_not_exist"] == Action.SKIP.value
    assert any("does_not_exist" in w for w in plan.warnings)


# --------------------------------------------------------------------------
# fingerprint ignores volatile fields Grafana adds at save time
# --------------------------------------------------------------------------
def test_fingerprint_ignores_version_and_id():
    base = lib.render("overview", PROJECT_ID, "ts-x", SCHEMA)
    stamped = copy.deepcopy(base)
    stamped["id"] = 42
    stamped["version"] = 7
    stamped["updated"] = "2026-01-01T00:00:00Z"
    assert lib.fingerprint(base) == lib.fingerprint(stamped)
