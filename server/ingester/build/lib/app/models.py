"""Pydantic request models for the ingester.

Field names match exactly what the BBB forwarder will POST (see server/README.md),
so handling a request is a straight validate -> SQL insert with no renaming.
"""

from __future__ import annotations

from datetime import datetime

from pydantic import BaseModel, Field


class Reading(BaseModel):
    """One sensor reading. `value` is nullable: a failed sensor read (sentinel on
    the LoRa wire) is forwarded as null and stored as NULL."""

    sensor_type: str = Field(..., examples=["temperature", "humidity", "soil_moisture"])
    value: float | None = None
    unit: str | None = None


class Meta(BaseModel):
    """Per-uplink RF / gateway metadata. All optional — the BBB may omit fields."""

    gateway_id: str | None = None
    rssi: float | None = None
    snr: float | None = None
    freq_mhz: float | None = None
    datarate: str | None = None


class IngestPayload(BaseModel):
    """A decoded uplink as forwarded by the BBB listener."""

    device_id: str
    timestamp: datetime  # ISO8601 UTC; when the gateway received the uplink
    readings: list[Reading]
    meta: Meta | None = None
