"""Concrete SchemaApplier against the shared Timescale cluster.

Each project gets its own Postgres schema (multi-tenancy mechanism #1). The
shape mirrors server/db/init/01_schema.sql so a project schema provisioned by
the applier matches the hand-written `croptopus` schema the running stack uses:
``measurements`` hypertable (long/narrow), a ``devices`` table, the two query
indexes, and retention + compression policies.

All writes are idempotent (``CREATE ... IF NOT EXISTS`` / ``if_not_exists``),
so re-applying an in-sync manifest is a no-op. Raw SQL via psycopg2 — clearer
for an IaC-style tool and consistent with the ingester.
"""

import psycopg2
from psycopg2 import sql

from ..interfaces import SchemaApplier, SchemaState
from ..errors import RefusedDestructive, StateReadError


class TimescaleSchemaApplier(SchemaApplier):
    def __init__(self, admin_url):
        self.admin_url = admin_url

    # -- connection helper -------------------------------------------------
    def _connect(self):
        try:
            conn = psycopg2.connect(self.admin_url)
            conn.autocommit = True
            return conn
        except psycopg2.OperationalError as e:
            raise StateReadError(f"cannot connect to Timescale: {e}") from e

    # ------------------------------------------------------------------ read
    def read_state(self, schema):
        state = SchemaState()
        conn = self._connect()
        try:
            with conn.cursor() as cur:
                cur.execute(
                    "SELECT 1 FROM information_schema.schemata WHERE schema_name = %s",
                    (schema,),
                )
                state.schema_exists = cur.fetchone() is not None
                if not state.schema_exists:
                    return state

                cur.execute(
                    """SELECT 1 FROM information_schema.tables
                       WHERE table_schema = %s AND table_name = 'measurements'""",
                    (schema,),
                )
                state.measurements_table_exists = cur.fetchone() is not None
                if not state.measurements_table_exists:
                    return state

                cur.execute(
                    """SELECT compression_enabled
                       FROM timescaledb_information.hypertables
                       WHERE hypertable_schema = %s
                         AND hypertable_name = 'measurements'""",
                    (schema,),
                )
                row = cur.fetchone()
                if row is not None:
                    state.is_hypertable = True
                    state.compression_enabled = bool(row[0])

                # retention + compression policies live in the jobs catalog
                cur.execute(
                    """SELECT proc_name, config
                       FROM timescaledb_information.jobs
                       WHERE hypertable_schema = %s
                         AND hypertable_name = 'measurements'""",
                    (schema,),
                )
                for proc_name, config in cur.fetchall():
                    config = config or {}
                    if proc_name == "policy_retention":
                        state.retention_interval = config.get("drop_after")
                    elif proc_name == "policy_compression":
                        state.compression_interval = config.get("compress_after")
        finally:
            conn.close()
        return state

    # ----------------------------------------------------------------- write
    def create_schema(self, schema):
        with self._connect() as conn:
            with conn.cursor() as cur:
                cur.execute(
                    sql.SQL("CREATE SCHEMA IF NOT EXISTS {}").format(sql.Identifier(schema))
                )
        return f"schema {schema} ensured"

    def create_measurements_table(self, schema):
        ident = sql.Identifier(schema)
        qualified = f"{schema}.measurements"
        measurements_ddl = sql.SQL(
            """CREATE TABLE IF NOT EXISTS {}.measurements (
                   time        TIMESTAMPTZ NOT NULL,
                   device_id   TEXT        NOT NULL,
                   sensor_type TEXT        NOT NULL,
                   value       DOUBLE PRECISION,
                   unit        TEXT,
                   meta        JSONB
               )"""
        ).format(ident)
        devices_ddl = sql.SQL(
            """CREATE TABLE IF NOT EXISTS {}.devices (
                   device_id  TEXT PRIMARY KEY,
                   first_seen TIMESTAMPTZ NOT NULL DEFAULT now(),
                   last_seen  TIMESTAMPTZ NOT NULL DEFAULT now(),
                   last_rssi  REAL,
                   last_snr   REAL,
                   meta       JSONB
               )"""
        ).format(ident)
        idx_device = sql.SQL(
            "CREATE INDEX IF NOT EXISTS measurements_device_time_idx "
            "ON {}.measurements (device_id, time DESC)"
        ).format(ident)
        idx_sensor = sql.SQL(
            "CREATE INDEX IF NOT EXISTS measurements_sensor_time_idx "
            "ON {}.measurements (sensor_type, time DESC)"
        ).format(ident)
        with self._connect() as conn:
            with conn.cursor() as cur:
                cur.execute("CREATE EXTENSION IF NOT EXISTS timescaledb")
                cur.execute(measurements_ddl)
                cur.execute(
                    "SELECT create_hypertable(%s, 'time', if_not_exists => TRUE)",
                    (qualified,),
                )
                cur.execute(devices_ddl)
                cur.execute(idx_device)
                cur.execute(idx_sensor)
        return f"tables {schema}.measurements (+devices, indexes) ensured as hypertable"

    def set_retention_policy(self, schema, days):
        qualified = f"{schema}.measurements"
        interval = f"{days} days"
        with self._connect() as conn:
            with conn.cursor() as cur:
                # remove-then-add makes interval changes idempotent; only ever
                # called for CREATE/UPDATE, so an in-sync re-apply never hits it
                cur.execute(
                    "SELECT remove_retention_policy(%s, if_exists => TRUE)",
                    (qualified,),
                )
                cur.execute(
                    "SELECT add_retention_policy(%s, INTERVAL %s, if_not_exists => TRUE)",
                    (qualified, interval),
                )
        return f"retention policy on {qualified} set to {interval}"

    def set_compression_policy(self, schema, days):
        qualified = f"{schema}.measurements"
        interval = f"{days} days"
        alter = sql.SQL(
            "ALTER TABLE {}.measurements SET "
            "(timescaledb.compress, timescaledb.compress_segmentby = 'device_id, sensor_type')"
        ).format(sql.Identifier(schema))
        with self._connect() as conn:
            with conn.cursor() as cur:
                cur.execute(alter)
                cur.execute(
                    "SELECT remove_compression_policy(%s, if_exists => TRUE)",
                    (qualified,),
                )
                cur.execute(
                    "SELECT add_compression_policy(%s, INTERVAL %s, if_not_exists => TRUE)",
                    (qualified, interval),
                )
        return f"compression policy on {qualified} set to {interval}"

    # ----------------------------------------------------------- destructive
    def drop_schema(self, schema):
        raise RefusedDestructive()
