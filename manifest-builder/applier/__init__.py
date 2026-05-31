"""The executable infrastructure layer for the croptopus manifest builder.

Importable and usable standalone (no Flask context required) — this is the
seam for extracting a ``projectctl`` CLI later. Typical use:

    from applier import build_appliers, plan as plan_mod, apply as apply_mod
    from applier import state as state_mod, config as config_mod

    cfg = config_mod.from_env(instance_dir)
    schema_app, dash_app = build_appliers(cfg, manifest)
    live = state_mod.read_live_state(manifest, schema_app, dash_app)
    the_plan = plan_mod.diff(manifest, live, manifest_id, manifest_hash)
    result = apply_mod.apply_plan(the_plan, schema_app, dash_app)
"""

from . import plan, apply, state, config, apply_log  # noqa: F401
from .model import Plan, ResourceChange, Action, ApplyResult, ResourceResult  # noqa: F401
from .errors import ApplierError, RefusedDestructive, StateReadError  # noqa: F401


def build_appliers(cfg, manifest):
    """Construct the concrete Timescale + Grafana appliers for a manifest.

    The only place the concrete backends are named — swap them here to change
    the multi-tenancy model without touching routes, plan or apply.
    """
    from .timescale.schema_applier import TimescaleSchemaApplier
    from .grafana.org_applier import GrafanaOrgManager
    from .grafana.dashboard_applier import GrafanaDashboardApplier

    project_id = (manifest.get("project") or {}).get("id", "")
    schema_applier = TimescaleSchemaApplier(cfg.timescale_admin_url)
    org_manager = GrafanaOrgManager(
        cfg.grafana_url, cfg.grafana_admin_user, cfg.grafana_admin_password,
        cfg.secrets_dir,
    )
    dashboard_applier = GrafanaDashboardApplier(
        org_manager, project_id, cfg.project_database_url_template,
    )
    return schema_applier, dashboard_applier
