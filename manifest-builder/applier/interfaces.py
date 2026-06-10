"""Abstract interfaces for every side effect the applier can have.

All "do things to infrastructure" calls funnel through these two abstract
bases. The concrete Timescale / Grafana implementations are pluggable, so the
"shared Grafana with orgs" model can later be swapped for "one Grafana per
project" without touching the routes, plan.py or apply.py.

Read methods (``read_*``) are used by state.py and must never mutate.
Write methods are used by apply.py and MUST be idempotent.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional


# --------------------------------------------------------------------------
# Live-state value objects
# --------------------------------------------------------------------------
@dataclass
class SchemaState:
    """Snapshot of what exists in Timescale for one project schema."""

    schema_exists: bool = False
    measurements_table_exists: bool = False
    is_hypertable: bool = False
    retention_interval: Optional[str] = None     # e.g. "365 days" or None
    compression_enabled: bool = False
    compression_interval: Optional[str] = None


@dataclass
class DatasourceState:
    exists: bool = False
    uid: Optional[str] = None
    name: Optional[str] = None
    database: Optional[str] = None
    schema: Optional[str] = None      # the project schema the datasource is scoped to


@dataclass
class DashboardState:
    uid: str
    exists: bool = True
    fingerprint: Optional[str] = None   # stable content hash of live JSON


@dataclass
class GrafanaState:
    """Snapshot of what exists in Grafana for one project org."""

    org_exists: bool = False
    org_id: Optional[int] = None
    datasource: DatasourceState = field(default_factory=DatasourceState)
    # uid -> DashboardState (only the manifest-referenced uids that exist live)
    dashboards: dict = field(default_factory=dict)
    # every croptopus-tagged dashboard live in the org (uid -> title); used to
    # surface ORPHANs that the manifest no longer references
    all_dashboards: dict = field(default_factory=dict)


# --------------------------------------------------------------------------
# Abstract appliers
# --------------------------------------------------------------------------
class SchemaApplier(ABC):
    """Owns the per-project Postgres schema (multi-tenancy mechanism #1)."""

    # ---- read path (state.py) ----
    @abstractmethod
    def read_state(self, schema: str) -> SchemaState:
        ...

    # ---- write path (apply.py); all idempotent ----
    @abstractmethod
    def create_schema(self, schema: str) -> str:
        """CREATE SCHEMA IF NOT EXISTS. Returns a human message."""

    @abstractmethod
    def create_measurements_table(self, schema: str) -> str:
        """CREATE TABLE IF NOT EXISTS <schema>.measurements + hypertable."""

    @abstractmethod
    def set_retention_policy(self, schema: str, days: int) -> str:
        ...

    @abstractmethod
    def set_compression_policy(self, schema: str, days: int) -> str:
        ...

    # ---- destructive (refused in v1) ----
    @abstractmethod
    def drop_schema(self, schema: str):
        """Must raise RefusedDestructive in v1."""


class DashboardApplier(ABC):
    """Owns the per-project Grafana org, datasource and dashboards
    (multi-tenancy mechanism #2)."""

    # ---- read path (state.py) ----
    @abstractmethod
    def read_state(self, org_name: str, datasource_name: str,
                   dashboard_uids: list) -> GrafanaState:
        ...

    # ---- write path (apply.py); all idempotent ----
    @abstractmethod
    def ensure_org(self, org_name: str) -> int:
        """Create or find the org. Returns its org id."""

    @abstractmethod
    def ensure_datasource(self, org_name: str, datasource_name: str,
                          schema: str) -> str:
        """Provision the TimescaleDB datasource inside the org, scoped to
        ``schema``. Returns the datasource uid."""

    @abstractmethod
    def ensure_dashboard(self, org_name: str, uid: str, dashboard_json: dict,
                         folder: str = None) -> str:
        """PUT a dashboard by uid (create-or-update) into ``folder`` (the
        manifest's grafana.folder; blank means the General folder). Returns a
        message."""

    # ---- destructive (refused in v1) ----
    @abstractmethod
    def delete_dashboard(self, org_name: str, uid: str):
        """Must raise RefusedDestructive in v1."""

    @abstractmethod
    def delete_datasource(self, org_name: str, uid: str):
        """Must raise RefusedDestructive in v1."""
