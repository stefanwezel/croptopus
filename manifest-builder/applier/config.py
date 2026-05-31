"""Configuration for the applier, read from environment variables.

The manifest NEVER holds credentials. The Flask app builds an AppConfig at
startup and hands it to the applier factory. For v1 a single shared admin
connection is fine (superuser creates schemas; Grafana admin creates orgs and
per-org service accounts).
"""

import os
from dataclasses import dataclass


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
    return AppConfig(
        timescale_admin_url=os.environ.get("TIMESCALE_ADMIN_URL", ""),
        grafana_url=os.environ.get("GRAFANA_URL", "http://grafana:3000"),
        grafana_admin_user=os.environ.get("GRAFANA_ADMIN_USER", "admin"),
        grafana_admin_password=os.environ.get("GRAFANA_ADMIN_PASSWORD", ""),
        project_database_url_template=os.environ.get(
            "PROJECT_DATABASE_URL_TEMPLATE",
            "postgresql://croptopus:croptopus@timescaledb:5432/croptopus",
        ),
        instance_dir=instance_dir,
    )
