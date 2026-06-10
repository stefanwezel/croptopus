"""Configuration for the applier, read from environment variables.

The manifest NEVER holds credentials. The Flask app builds an AppConfig at
startup and hands it to the applier factory. For v1 a single shared admin
connection is fine (superuser creates schemas; Grafana admin creates orgs and
per-org service accounts).

Each setting reads its own applier-specific variable first, then falls back to
the name the Croptopus *server* stack already uses (`DATABASE_URL`,
`GF_SECURITY_ADMIN_*`). That way a manifest-builder co-deployed in the same
Coolify project needs ZERO extra env vars — it reuses the stack's existing
ones — while the explicit applier names still win when you want to point the
applier at a different admin role/URL.
"""

import os
from dataclasses import dataclass


def _first_env(*names, default=""):
    """First non-empty value among `names`, else `default`. Lets an explicit
    applier var override a shared server-stack var of a different name."""
    for name in names:
        value = os.environ.get(name)
        if value:
            return value
    return default


@dataclass
class AppConfig:
    timescale_admin_url: str
    grafana_url: str
    grafana_admin_user: str
    grafana_admin_password: str
    project_database_url_template: str
    instance_dir: str

    @property
    def secrets_dir(self):
        return os.path.join(self.instance_dir, "secrets")

    @property
    def apply_log_db(self):
        return os.path.join(self.instance_dir, "apply_log.db")

    @property
    def configured(self):
        """True once the infra-targeting env vars are present."""
        return bool(self.timescale_admin_url and self.grafana_url
                    and self.grafana_admin_password)


def from_env(instance_dir):
    # The admin DB URL and the per-project datasource template both default to
    # the server stack's DATABASE_URL: in a co-deployed setup the runtime role
    # (POSTGRES_USER) is the image superuser, so it can also create schemas.
    return AppConfig(
        timescale_admin_url=_first_env("TIMESCALE_ADMIN_URL", "DATABASE_URL"),
        grafana_url=_first_env("GRAFANA_URL", default="http://grafana:3000"),
        grafana_admin_user=_first_env(
            "GRAFANA_ADMIN_USER", "GF_SECURITY_ADMIN_USER", default="admin"
        ),
        grafana_admin_password=_first_env(
            "GRAFANA_ADMIN_PASSWORD", "GF_SECURITY_ADMIN_PASSWORD"
        ),
        project_database_url_template=_first_env(
            "PROJECT_DATABASE_URL_TEMPLATE", "DATABASE_URL",
            default="postgresql://croptopus:croptopus@timescaledb:5432/croptopus",
        ),
        instance_dir=instance_dir,
    )
