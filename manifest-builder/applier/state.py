"""Read live state for a project from Timescale + Grafana.

Pure reads — no mutation. Returns value objects that plan.py diffs against the
manifest. All access goes through the SchemaApplier / DashboardApplier
interfaces so the backends stay pluggable.
"""

from dataclasses import dataclass

from .interfaces import SchemaState, GrafanaState
from .grafana import dashboard_library as lib


@dataclass
class LiveState:
    schema: SchemaState
    grafana: GrafanaState


def manifest_schema(manifest):
    return (manifest.get("database") or {}).get("schema")


def manifest_org_name(manifest):
    """Per-project Grafana org: '{customer.name} / {project.name}'
    (multi-tenancy mechanism #2)."""
    g = manifest.get("grafana") or {}
    if g.get("org"):
        return g["org"]
    customer = (manifest.get("customer") or {}).get("name", "")
    project = (manifest.get("project") or {}).get("name", "")
    return f"{customer} / {project}"


def manifest_datasource_name(manifest):
    g = manifest.get("grafana") or {}
    return g.get("datasource_name") or "croptopus_tsdb"


def enabled_dashboards(manifest):
    """Return [(id, has_template)] for enabled dashboard entries."""
    out = []
    for d in (manifest.get("grafana") or {}).get("dashboards", []):
        if not d.get("enabled", False):
            continue
        out.append((d["id"], lib.has_template(d["id"])))
    return out


def read_live_state(manifest, schema_applier, dashboard_applier):
    schema = manifest_schema(manifest)
    project_id = (manifest.get("project") or {}).get("id", "")

    schema_state = schema_applier.read_state(schema) if schema else SchemaState()

    uids = [
        lib.dashboard_uid(project_id, did)
        for did, has_tpl in enabled_dashboards(manifest)
        if has_tpl
    ]
    grafana_state = dashboard_applier.read_state(
        manifest_org_name(manifest),
        manifest_datasource_name(manifest),
        uids,
    )
    return LiveState(schema=schema_state, grafana=grafana_state)
