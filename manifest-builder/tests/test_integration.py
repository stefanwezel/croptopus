"""Round-trip integration test against a real Timescale + Grafana.

Skipped automatically unless the applier env vars are set (see config.from_env).
Run locally with docker compose up the server stack, or wire into CI later:

    TIMESCALE_ADMIN_URL=postgresql://postgres:...@localhost:5432/croptopus \
    GRAFANA_URL=http://localhost:3000 \
    GRAFANA_ADMIN_USER=admin GRAFANA_ADMIN_PASSWORD=... \
    PROJECT_DATABASE_URL_TEMPLATE=postgresql://croptopus:...@timescaledb:5432/croptopus \
    pytest -m integration

The invariant under test: Plan -> Apply -> Plan must end with all NO_CHANGE.
"""

import os
import tempfile

import pytest
import yaml

from applier import build_appliers, plan as plan_mod, apply as apply_mod
from applier import state as state_mod, config as config_mod
from applier.model import Action

DEMO_PATH = os.path.join(os.path.dirname(__file__), "..", "examples", "example_manifest.yaml")

pytestmark = pytest.mark.integration


def _configured():
    cfg = config_mod.from_env(tempfile.mkdtemp())
    return cfg if cfg.configured else None


@pytest.fixture
def cfg():
    c = _configured()
    if c is None:
        pytest.skip("applier env vars not set; skipping integration round-trip")
    return c


def _load_demo():
    with open(DEMO_PATH) as f:
        return yaml.safe_load(f)


def _plan(cfg, manifest):
    schema_app, dash_app = build_appliers(cfg, manifest)
    live = state_mod.read_live_state(manifest, schema_app, dash_app)
    return plan_mod.diff(manifest, live, "integration", "hash"), schema_app, dash_app


def test_plan_apply_replan_is_idempotent(cfg):
    manifest = _load_demo()

    # 1. Plan (may be all-create against empty infra, or partial against existing)
    plan1, schema_app, dash_app = _plan(cfg, manifest)

    # 2. Apply
    result = apply_mod.apply_plan(plan1, schema_app, dash_app)
    assert result.success, [r.to_dict() for r in result.results if r.status == "error"]

    # 3. Re-plan — the acceptance test: everything must be NO_CHANGE
    plan2, _, _ = _plan(cfg, manifest)
    drift = [c for c in plan2.changes
             if c.action in (Action.CREATE.value, Action.UPDATE.value)]
    assert drift == [], f"re-plan not idempotent: {[(c.kind, c.name, c.action) for c in drift]}"
