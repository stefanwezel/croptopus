"""Async Postgres/TimescaleDB access: connection pool + writes.

DATABASE_URL comes from the environment. The pool is created on app startup and
closed on shutdown (see main.py lifespan).
"""

from __future__ import annotations

import json
import os
from datetime import datetime

import asyncpg

from .models import IngestPayload

_pool: asyncpg.Pool | None = None

_INSERT_MEASUREMENT = """
    INSERT INTO croptopus.measurements (time, device_id, sensor_type, value, unit, meta)
    VALUES ($1, $2, $3, $4, $5, $6::jsonb)
"""

# Upsert "last seen" / link quality. first_seen sticks; the rest refresh.
_UPSERT_DEVICE = """
    INSERT INTO croptopus.devices (device_id, first_seen, last_seen, last_rssi, last_snr, meta)
    VALUES ($1, $2, $2, $3, $4, $5::jsonb)
    ON CONFLICT (device_id) DO UPDATE SET
        last_seen = EXCLUDED.last_seen,
        last_rssi = EXCLUDED.last_rssi,
        last_snr  = EXCLUDED.last_snr,
        meta      = EXCLUDED.meta
"""


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


async def write_reading(
    conn: asyncpg.Connection,
    *,
    time: datetime,
    device_id: str,
    sensor_type: str,
    value: float | None,
    unit: str | None,
    meta: dict | None,
) -> None:
    """Insert a single measurement row using the given connection."""
    await conn.execute(
        _INSERT_MEASUREMENT,
        time,
        device_id,
        sensor_type,
        value,
        unit,
        json.dumps(meta) if meta is not None else None,
    )


async def ingest(payload: IngestPayload) -> int:
    """Persist one decoded uplink: a measurement row per reading plus a device
    upsert, all in one transaction. Returns the number of measurement rows written."""
    if _pool is None:
        raise RuntimeError("db pool not initialised")

    meta_dict = payload.meta.model_dump() if payload.meta is not None else None
    meta_json = json.dumps(meta_dict) if meta_dict is not None else None
    rssi = payload.meta.rssi if payload.meta is not None else None
    snr = payload.meta.snr if payload.meta is not None else None

    async with _pool.acquire() as conn:
        async with conn.transaction():
            for r in payload.readings:
                await write_reading(
                    conn,
                    time=payload.timestamp,
                    device_id=payload.device_id,
                    sensor_type=r.sensor_type,
                    value=r.value,
                    unit=r.unit,
                    meta=meta_dict,
                )
            await conn.execute(
                _UPSERT_DEVICE,
                payload.device_id,
                payload.timestamp,
                rssi,
                snr,
                meta_json,
            )

    return len(payload.readings)
