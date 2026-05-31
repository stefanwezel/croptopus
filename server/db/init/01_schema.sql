-- Croptopus measurement schema for TimescaleDB.
--
-- This file runs ONCE, automatically, the first time the timescaledb container
-- starts against an empty data volume (via /docker-entrypoint-initdb.d/). It does
-- not re-run on plain container restarts. We still write it idempotently
-- (IF NOT EXISTS / guarded policy calls) so that re-applying it by hand — e.g.
-- `psql ... -f 01_schema.sql` — is safe.
--
-- Shape and names mirror the project manifest's `database` section so this hand
-- written schema matches what projectctl will eventually generate:
--   schema = croptopus, table = measurements, long/narrow columns.
--
-- Tunables (defaults chosen here; change in one place):
--   retention  : 730 days   (manifest field: retention_days)
--   compress   : after 30 days (manifest field: compression_after_days)

CREATE EXTENSION IF NOT EXISTS timescaledb;

CREATE SCHEMA IF NOT EXISTS croptopus;

-- ---------------------------------------------------------------------------
-- measurements: one row per (time, device, sensor_type) reading.
-- value is nullable so a failed sensor read (sentinel on the wire) stores NULL.
-- ---------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS croptopus.measurements (
    time         timestamptz      NOT NULL,
    device_id    text             NOT NULL,
    sensor_type  text             NOT NULL,
    value        double precision,            -- nullable: sensor read failure -> NULL
    unit         text,
    meta         jsonb
);

-- Convert to a hypertable. if_not_exists keeps this safe on re-run.
SELECT create_hypertable(
    'croptopus.measurements',
    'time',
    if_not_exists => TRUE
);

-- "latest readings per device" and "per sensor type" query paths.
CREATE INDEX IF NOT EXISTS measurements_device_time_idx
    ON croptopus.measurements (device_id, time DESC);
CREATE INDEX IF NOT EXISTS measurements_sensor_time_idx
    ON croptopus.measurements (sensor_type, time DESC);

-- ---------------------------------------------------------------------------
-- devices: "last seen" / link-quality tracking, upserted by the ingester on
-- every POST. Not a hypertable — one row per device.
-- ---------------------------------------------------------------------------
CREATE TABLE IF NOT EXISTS croptopus.devices (
    device_id  text PRIMARY KEY,
    first_seen timestamptz NOT NULL DEFAULT now(),
    last_seen  timestamptz NOT NULL DEFAULT now(),
    last_rssi  real,
    last_snr   real,
    meta       jsonb
);

-- ---------------------------------------------------------------------------
-- Retention + compression policies. add_*_policy already supports if_not_exists,
-- but we wrap in a DO block so the whole file survives older TimescaleDB builds
-- and repeated manual application without erroring.
-- ---------------------------------------------------------------------------

-- Retention: drop chunks older than 730 days.
DO $$
BEGIN
    PERFORM add_retention_policy('croptopus.measurements', INTERVAL '730 days', if_not_exists => TRUE);
EXCEPTION
    WHEN duplicate_object THEN NULL;   -- policy already exists
END
$$;

-- Compression: enable, segment by the natural query keys.
ALTER TABLE croptopus.measurements
    SET (
        timescaledb.compress,
        timescaledb.compress_segmentby = 'device_id, sensor_type'
    );

-- Compress chunks older than 30 days.
DO $$
BEGIN
    PERFORM add_compression_policy('croptopus.measurements', INTERVAL '30 days', if_not_exists => TRUE);
EXCEPTION
    WHEN duplicate_object THEN NULL;   -- policy already exists
END
$$;
