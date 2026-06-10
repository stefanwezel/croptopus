"""Abstract interfaces for every side effect the applier can have.

All "do things to infrastructure" calls funnel through these two abstract
bases. The concrete Timescale / Grafana implementations are pluggable, so the
"shared Grafana org with a folder per project" model can later be swapped for
"one Grafana per project" without touching the routes, plan.py or apply.py.

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
class IngestTokenState:
    """Snapshot of a project's row in registry.projects (ingest routing)."""

    exists: bool = False
    schema_name: Optional[str] = None   # the schema the token routes to


@dataclass
class DatasourceState:
    exists: bool = False
    uid: Optional[str] = None
    name: Optional[str] = None
    database: Optional[str] = None


@dataclass
class DashboardState:
    uid: str
    exists: bool = True
    fingerprint: Optional[str] = None   # stable content hash of live JSON


@dataclass
class GrafanaState:
    """Snapshot of what exists in Grafana (org 1) for one project."""

    folder_exists: bool = False
    folder_uid: Optional[str] = None
    datasource: DatasourceState = field(default_factory=DatasourceState)
    # uid -> DashboardState (only the manifest-referenced uids that exist live)
    dashboards: dict = field(default_factory=dict)
    # every croptopus-tagged dashboard live in the project's folder
    # (uid -> title); used to surface ORPHANs the manifest no longer references
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

    # ---- ingest-token registry (token -> schema routing for the ingester) ----
    @abstractmethod
    def read_ingest_token(self, project_id: str) -> IngestTokenState:
        """Read the project's registry.projects row (read path; never mutates).
        A missing registry table reads as 'not registered'."""

    @abstractmethod
    def ensure_ingest_token(self, project_id: str, schema: str) -> str:
        """Register the project in registry.projects, generating its ingest
        token on first registration (the token is included in the returned
        message ONLY then). Repoints schema_name if it drifted. Idempotent."""

    # ---- destructive (refused in v1) ----
    @abstractmethod
    def drop_schema(self, schema: str):
        """Must raise RefusedDestructive in v1."""


class DashboardApplier(ABC):
    """Owns the per-project Grafana folder and dashboards in the shared org
    (multi-tenancy mechanism #2), plus the shared TimescaleDB datasource."""

    # ---- read path (state.py) ----
    @abstractmethod
    def read_state(self, folder_name: str, datasource_name: str,
                   dashboard_uids: list) -> GrafanaState:
        ...

    # ---- write path (apply.py); all idempotent ----
    @abstractmethod
    def ensure_folder(self, folder_name: str) -> str:
        """Create or find the project's folder. Returns its uid (blank name
        means the General folder, uid ``""``)."""

    @abstractmethod
    def ensure_datasource(self, datasource_name: str) -> str:
        """Find the shared TimescaleDB datasource by name (e.g. the one the
        server stack provisions), creating it only if missing. Returns its
        uid. Never modifies an existing datasource."""

    @abstractmethod
    def ensure_dashboard(self, uid: str, dashboard_json: dict,
                         folder: str = None) -> str:
        """PUT a dashboard by uid (create-or-update) into ``folder`` (the
        project's folder; blank means the General folder). Returns a
        message."""

    # ---- destructive (refused in v1) ----
    @abstractmethod
    def delete_dashboard(self, uid: str):
        """Must raise RefusedDestructive in v1."""

    @abstractmethod
    def delete_datasource(self, uid: str):
        """Must raise RefusedDestructive in v1."""
