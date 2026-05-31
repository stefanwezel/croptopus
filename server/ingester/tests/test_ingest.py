"""Tests for the ingester HTTP surface.

These exercise auth, validation, and the success path WITHOUT a real database:
the db module's pool/connect/ingest/healthcheck functions are monkeypatched, so
`pytest` runs with no TimescaleDB and no network.

Run from server/ingester/:
    uv pip install --system ".[test]"   # or: pip install ".[test]"
    pytest
"""

from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from app import db, main

TOKEN = "test-token"

VALID_PAYLOAD = {
    "device_id": "node_4f2a",
    "timestamp": "2026-05-31T19:14:08Z",
    "readings": [
        {"sensor_type": "temperature", "value": 23.13, "unit": "C"},
        {"sensor_type": "humidity", "value": 58.61, "unit": "%"},
        {"sensor_type": "soil_moisture", "value": 13, "unit": "%"},
    ],
    "meta": {
        "gateway_id": "bbb_gw01",
        "rssi": -48,
        "snr": 13.2,
        "freq_mhz": 868.1,
        "datarate": "SF7BW125",
    },
}


@pytest.fixture
def client(monkeypatch):
    """TestClient with the DB stubbed out and a known INGEST_TOKEN."""
    monkeypatch.setenv("INGEST_TOKEN", TOKEN)

    ingested: list = []

    async def fake_connect() -> None:
        return None

    async def fake_disconnect() -> None:
        return None

    async def fake_healthcheck() -> bool:
        return True

    async def fake_ingest(payload) -> int:
        ingested.append(payload)
        return len(payload.readings)

    monkeypatch.setattr(db, "connect", fake_connect)
    monkeypatch.setattr(db, "disconnect", fake_disconnect)
    monkeypatch.setattr(db, "healthcheck", fake_healthcheck)
    monkeypatch.setattr(db, "ingest", fake_ingest)

    with TestClient(main.app) as c:
        c.ingested = ingested  # expose for assertions
        yield c


def _auth(token: str = TOKEN) -> dict:
    return {"Authorization": f"Bearer {token}"}


def test_healthz_ok(client):
    resp = client.get("/healthz")
    assert resp.status_code == 200
    assert resp.json() == {"status": "ok", "db": "ok"}


def test_ingest_happy_path(client):
    resp = client.post("/ingest", json=VALID_PAYLOAD, headers=_auth())
    assert resp.status_code == 202
    body = resp.json()
    assert body["rows"] == 3
    assert body["device_id"] == "node_4f2a"
    # the payload reached the (stubbed) DB layer
    assert len(client.ingested) == 1
    assert client.ingested[0].device_id == "node_4f2a"


def test_ingest_accepts_null_value(client):
    """A failed sensor read forwards value=null and must be accepted/stored."""
    payload = {
        "device_id": "node_4f2a",
        "timestamp": "2026-05-31T19:15:00Z",
        "readings": [
            {"sensor_type": "temperature", "value": None, "unit": "C"},
            {"sensor_type": "soil_moisture", "value": 13, "unit": "%"},
        ],
    }
    resp = client.post("/ingest", json=payload, headers=_auth())
    assert resp.status_code == 202
    assert client.ingested[0].readings[0].value is None


def test_ingest_requires_token(client):
    resp = client.post("/ingest", json=VALID_PAYLOAD)  # no Authorization header
    assert resp.status_code == 401


def test_ingest_rejects_wrong_token(client):
    resp = client.post("/ingest", json=VALID_PAYLOAD, headers=_auth("nope"))
    assert resp.status_code == 401


def test_ingest_rejects_malformed_body(client):
    resp = client.post("/ingest", json={"device_id": "x"}, headers=_auth())
    assert resp.status_code == 422  # missing timestamp/readings
