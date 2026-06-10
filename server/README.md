# Croptopus server stack

Server-side ingestion + visualization for the Croptopus LoRa sensor network.
The BBB-side listener decodes raw-LoRa uplinks and (in a later step) POSTs them as
JSON to the **ingester**, which validates and writes them into **TimescaleDB**;
**Grafana** reads that DB for dashboards. This stack is deployed on a VPS managed
by Coolify, which provides the reverse proxy and TLS — so there is no Caddy/Traefik/
nginx in this compose file.

Three services: `timescaledb`, `ingester` (FastAPI), `grafana`.

## What the ingester accepts

Decoding stays on the BBB. The ingester takes already-decoded JSON:

```json
{
  "device_id": "node_4f2a",
  "timestamp": "2026-05-31T19:14:08Z",
  "readings": [
    {"sensor_type": "temperature",   "value": 23.13, "unit": "C"},
    {"sensor_type": "humidity",      "value": 58.61, "unit": "%"},
    {"sensor_type": "soil_moisture", "value": 13,    "unit": "%"}
  ],
  "meta": {"gateway_id": "bbb_gw01", "rssi": -48, "snr": 13.2, "freq_mhz": 868.1, "datarate": "SF7BW125"}
}
```

A failed sensor read may send `"value": null`; it is accepted and stored as NULL.
These field names match what the BBB forwarder will send, so the BBB→server step is
mechanical.

| Endpoint        | Method | Auth          | Result                                  |
| --------------- | ------ | ------------- | --------------------------------------- |
| `/healthz`      | GET    | none          | `200 {"status":"ok","db":"ok"\|"down"}` |
| `/ingest`       | POST   | Bearer token  | `202 Accepted`                          |

Auth: `Authorization: Bearer <INGEST_TOKEN>`; missing/wrong → `401`.

## Run locally

```bash
cd server
cp .env.example .env        # 1. then edit secrets (passwords, INGEST_TOKEN)
docker compose up -d --build  # 2. start the stack

# 3. ingester health (also checks the DB)
curl http://127.0.0.1:8000/healthz
# -> {"status":"ok","db":"ok"}

# 4. Grafana UI: http://127.0.0.1:3000  (admin / the GF_SECURITY_ADMIN_PASSWORD
#    you set; change it after first login). The "Croptopus Overview" dashboard
#    and the Timescale datasource are provisioned automatically.
```

### End-to-end smoke test

POST one uplink, then read it back from the DB. Uses the `INGEST_TOKEN` from `.env`:

```bash
# load INGEST_TOKEN (and the rest) into the shell
set -a; source .env; set +a

# 5a. POST a real-looking uplink
curl -i -X POST http://127.0.0.1:8000/ingest \
  -H "Authorization: Bearer ${INGEST_TOKEN}" \
  -H "Content-Type: application/json" \
  -d '{
    "device_id": "node_4f2a",
    "timestamp": "2026-05-31T19:14:08Z",
    "readings": [
      {"sensor_type": "temperature",   "value": 23.13, "unit": "C"},
      {"sensor_type": "humidity",      "value": 58.61, "unit": "%"},
      {"sensor_type": "soil_moisture", "value": 13,    "unit": "%"}
    ],
    "meta": {"gateway_id": "bbb_gw01", "rssi": -48, "snr": 13.2, "freq_mhz": 868.1, "datarate": "SF7BW125"}
  }'
# -> HTTP/1.1 202 Accepted ... {"status":"accepted","device_id":"node_4f2a","rows":3}

# 5b. confirm the rows landed
docker compose exec timescaledb \
  psql -U "${POSTGRES_USER}" -d "${POSTGRES_DB}" \
  -c "SELECT time, device_id, sensor_type, value, unit FROM croptopus.measurements ORDER BY time DESC LIMIT 5;"

# and the device "last seen" upsert
docker compose exec timescaledb \
  psql -U "${POSTGRES_USER}" -d "${POSTGRES_DB}" \
  -c "SELECT device_id, last_seen, last_rssi, last_snr FROM croptopus.devices;"
```

### Run the tests

The ingester has pytest tests that stub the DB (no TimescaleDB needed):

```bash
cd server/ingester
uv pip install --system ".[test]"   # or: pip install ".[test]"
pytest
```

## Deploy on Coolify

- This compose runs as a single **Coolify-managed application** (point Coolify at
  this `server/` directory / its `docker-compose.yml`).
- Each service you tag **Public** in Coolify gets a Traefik route + Let's Encrypt
  cert automatically — that is the reverse proxy / TLS layer, which is why it is
  intentionally absent from this compose file.
- **Ingester** must be Public so the BBB can POST to it over HTTPS.
- **Grafana** can be Public too if you want the UI on the internet, or kept private
  and reached via an SSH tunnel / Coolify's private networking.
- **TimescaleDB** must stay private — do **not** add a public toggle. Other services
  reach it over the internal compose network by service name (`timescaledb`).
- Set the env vars (`POSTGRES_*`, `DATABASE_URL`, `INGEST_TOKEN`, `GF_*`) in
  Coolify's UI; those override `.env`. Keep `DATABASE_URL`'s host as `timescaledb`.
- The localhost-only port mappings (`127.0.0.1:8000`, `127.0.0.1:3000`,
  `127.0.0.1:5432`) are for local dev; on Coolify, traffic comes via Traefik, not
  those bindings.

## Schema notes

`db/init/01_schema.sql` creates schema `croptopus`, the `measurements` hypertable
(long/narrow: `time, device_id, sensor_type, value, unit, meta`), and a `devices`
table for last-seen/link-quality tracking. It runs once, automatically, on the first
start against an empty DB volume. Defaults: 730-day retention, compress after 30 days
— change them in that one file.

## Future work

This stack will eventually be **(re)generated from the project manifest by
`projectctl`**. For now the schema, datasource, and dashboard are hand-written using
defaults that match the manifest's `database` section. Names here (schema=`croptopus`,
table=`measurements`, the column names, datasource UID `timescale-main`) match what a
generator would produce, so swapping the hand-written version for the generated one is
mechanical.

> Note: the manifest's `database.retention_days` (`730`) now matches this
> hand-written schema's retention default, so the two are in sync. When
> `projectctl` generates the schema it will follow the manifest; if you change
> `retention_days` there, mirror it in `01_schema.sql` until then.
