"""Async Postgres/TimescaleDB access: connection pool, token routing + writes.

DATABASE_URL comes from the environment. The pool is created on app startup and
closed on shutdown (see main.py lifespan).

Writes are schema-per-project: the target schema is resolved from the request's
bearer token via registry.projects (written by the manifest-builder applier) and
threaded into the SQL. Schema names come from our own registry, never from the
client, but are still validated as plain identifiers before interpolation —
identifiers can't be bound as query parameters.
"""

from __future__ import annotations

import json
import os
import re
import time

import asyncpg

from .models import IngestPayload

_pool: asyncpg.Pool | None = None

_IDENT_RE = re.compile(r"^[a-z_][a-z0-9_]*$")

_INSERT_MEASUREMENT = """
    INSERT INTO {schema}.measurements (time, device_id, sensor_type, value, unit, meta)
    VALUES ($1, $2, $3, $4, $5, $6::jsonb)
"""

# Upsert "last seen" / link quality. first_seen sticks; the rest refresh.
_UPSERT_DEVICE = """
    INSERT INTO {schema}.devices (device_id, first_seen, last_seen, last_rssi, last_snr, meta)
    VALUES ($1, $2, $2, $3, $4, $5::jsonb)
    ON CONFLICT (device_id) DO UPDATE SET
        last_seen = EXCLUDED.last_seen,
        last_rssi = EXCLUDED.last_rssi,
        last_snr  = EXCLUDED.last_snr,
        meta      = EXCLUDED.meta
"""


def _qualify(sql_template: str, schema: str) -> str:
    if not _IDENT_RE.match(schema):
        raise ValueError(f"invalid schema name: {schema!r}")
    return sql_template.format(schema=schema)


async def connect() -> None:
    """Create the connection pool. Call once on startup."""
    global _pool
    if _pool is None:
        _pool = await asyncpg.create_pool(
            dsn=os.environ["DATABASE_URL"],
            min_size=1,
            max_size=10,
            command_timeout=10,
        )


async def disconnect() -> None:
    """Close the pool. Call once on shutdown."""
    global _pool
    if _pool is not None:
        await _pool.close()
        _pool = None


# Token lookups hit every request, so positive results are cached briefly. A
# rotated/revoked token therefore keeps working for at most the TTL.
_TOKEN_CACHE_TTL = 60.0
_token_cache: dict[str, tuple[str, float]] = {}


async def resolve_token(token: str) -> str | None:
    """Schema name for a registry ingest token, or None if unknown.

    Tolerates a missing registry table (returns None) so the ingester runs
    against a database the applier hasn't touched yet — only the legacy env
    token works in that case (see auth.require_token).
    """
    if _pool is None:
        return None
    cached = _token_cache.get(token)
    if cached and cached[1] > time.monotonic():
        return cached[0]
    try:
        async with _pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT schema_name FROM registry.projects WHERE ingest_token = $1",
                token,
            )
    except (asyncpg.UndefinedTableError, asyncpg.InvalidSchemaNameError):
        return None
    if row is None:
        return None
    schema = row["schema_name"]
    _token_cache[token] = (schema, time.monotonic() + _TOKEN_CACHE_TTL)
    return schema


async def healthcheck() -> bool:
    """True if the DB answers a trivial query."""
    if _pool is None:
        return False
    try:
        async with _pool.acquire() as conn:
            await conn.fetchval("SELECT 1")
        return True
    except Exception:
        return False


async def ingest(payload: IngestPayload, schema: str) -> int:
    """Persist one decoded uplink into ``schema``: a measurement row per
    reading plus a device upsert, all in one transaction. Returns the number
    of measurement rows written."""
    if _pool is None:
        raise RuntimeError("db pool not initialised")

    insert_sql = _qualify(_INSERT_MEASUREMENT, schema)
    upsert_sql = _qualify(_UPSERT_DEVICE, schema)

    meta_dict = payload.meta.model_dump() if payload.meta is not None else None
    meta_json = json.dumps(meta_dict) if meta_dict is not None else None
    rssi = payload.meta.rssi if payload.meta is not None else None
    snr = payload.meta.snr if payload.meta is not None else None

    async with _pool.acquire() as conn:
        async with conn.transaction():
            for r in payload.readings:
                await conn.execute(
                    insert_sql,
                    payload.timestamp,
                    payload.device_id,
                    r.sensor_type,
                    r.value,
                    r.unit,
                    meta_json,
                )
            await conn.execute(
                upsert_sql,
                payload.device_id,
                payload.timestamp,
                rssi,
                snr,
                meta_json,
            )

    return len(payload.readings)
