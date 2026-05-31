"""Croptopus ingester — FastAPI app.

Accepts already-decoded sensor uplinks from the BBB listener and persists them to
TimescaleDB. No LoRa/device decoding happens here: decoding stays on the BBB; this
service is a validate -> SQL insert.

Endpoints:
    GET  /healthz  -> 200 {"status": "ok", "db": "ok"|"down"}
    POST /ingest   -> 202 Accepted  (bearer-token protected)
"""

from __future__ import annotations

import logging
import sys
import time
from contextlib import asynccontextmanager

from fastapi import Depends, FastAPI, status
from fastapi.responses import JSONResponse

from . import db
from .auth import require_token
from .models import IngestPayload

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s %(message)s",
    stream=sys.stdout,  # Coolify captures container stdout
)
log = logging.getLogger("ingester")


@asynccontextmanager
async def lifespan(_app: FastAPI):
    await db.connect()
    log.info("ingester started; db pool ready")
    try:
        yield
    finally:
        await db.disconnect()
        log.info("ingester stopped; db pool closed")


app = FastAPI(title="Croptopus Ingester", version="0.1.0", lifespan=lifespan)


@app.get("/healthz")
async def healthz() -> JSONResponse:
    db_ok = await db.healthcheck()
    body = {"status": "ok", "db": "ok" if db_ok else "down"}
    code = status.HTTP_200_OK if db_ok else status.HTTP_503_SERVICE_UNAVAILABLE
    return JSONResponse(content=body, status_code=code)


@app.post("/ingest", status_code=status.HTTP_202_ACCEPTED, dependencies=[Depends(require_token)])
async def ingest(payload: IngestPayload) -> JSONResponse:
    started = time.perf_counter()
    n = await db.ingest(payload)
    latency_ms = round((time.perf_counter() - started) * 1000, 1)

    log.info(
        "ingest ok device_id=%s num_readings=%d latency_ms=%s",
        payload.device_id,
        n,
        latency_ms,
    )
    return JSONResponse(
        content={"status": "accepted", "device_id": payload.device_id, "rows": n},
        status_code=status.HTTP_202_ACCEPTED,
    )
