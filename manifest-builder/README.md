# manifest-builder

A small Flask app for the **Croptopus manifest** — the single source of truth
for a LoRa sensor network — with two jobs:

1. **Author** the manifest. Visually configure the customer, project, gateways,
   sensor stations (with their sensors and sampling config), database, Grafana
   dashboards, and alerts; export/import clean YAML; round-trip losslessly.
2. **Apply** the manifest. Turn a saved manifest into real infrastructure: a
   per-project TimescaleDB schema and a per-project Grafana folder (in the
   shared org) with its dashboards. Plan first, review the diff, then apply —
   with an append-only audit log of every apply.

The manifest itself never holds credentials. Infra targets and admin secrets
come from the environment (see [Configuration](#configuration)).

## Run it

From inside `manifest-builder/`:

```bash
uv run flask --app app run --debug
```

`uv` creates a virtualenv and installs deps on first run. Open
<http://127.0.0.1:5000>.

> The applier needs `psycopg2-binary` and `requests` (see `requirements.txt`).
> Authoring/round-trip works without live infra; Plan/Apply stay disabled until
> the infra env vars are set.

## Authoring (manifest CRUD)

- **Landing page** offers: start a new manifest, load the bundled example,
  upload an existing YAML, or open a previously **saved** configuration.
- **Editor** has forms for every section. Gateways, nodes (with their
  sensors), Grafana dashboards, and alerts can be added/removed/duplicated.
- **Save / Load named** — persist the current manifest under a name into
  `manifest-builder/saved/<name>.yaml`. The landing page lists every saved
  manifest with project name, environment, and gateway/node counts. You can
  also drop a YAML file into `saved/` by hand and it shows up there.
- **Export** downloads the current manifest as a clean, stably-ordered YAML;
  **View raw YAML** shows it inline.
- **Upload** parses a YAML file and loads it into the editor.
- **Round-trips**: export → re-upload reproduces the same logical config.
- **Validation**: surfaces friendly errors (e.g. a node references an unknown
  `gateway_id`, required field missing, numeric field not numeric) without
  crashing.

The bundled example (`examples/example_manifest.yaml`) reflects what we
currently run: one BBB+WM1302 gateway and two XIAO ESP32S3 sensor stations,
each with SHT35 (temperature/humidity) and a soil-moisture probe.

## Applying (manifest → infrastructure)

The `applier/` package is the executable layer. The Flask routes stay thin:
they load a *saved* manifest, call into `applier/`, and render the result.
Everything that mutates infra is idempotent and refuses destructive operations.

| Route            | What it does                                                        |
| ---------------- | ------------------------------------------------------------------ |
| `/plan/<name>`   | Read live state, compute a diff, show every CREATE/UPDATE/NO_CHANGE |
| `/apply/<name>`  | Enact exactly the plan you reviewed (stale plans are refused)       |
| `/status/<name>` | Read live state and show drift without changing anything           |
| `/history`       | Append-only audit log of past applies                              |

**What an apply provisions, per project:**

- **TimescaleDB** (`applier/timescale/schema_applier.py`): a Postgres schema
  named from `database.schema`, the `measurements` hypertable (long/narrow),
  a `devices` table, the two query indexes, and retention + compression
  policies from `database.retention_days` / `compression_after_days`. The shape
  mirrors `server/db/init/01_schema.sql` exactly. Plus the project's row in
  **`registry.projects`**: on first apply a per-project **ingest token** is
  generated (shown once, in the apply result — configure the gateway forwarder
  with it). The ingester resolves token → schema through this table, so
  uplinks land in the right project schema with no ingester redeploy.
- **Grafana** (`applier/grafana/`): a folder in org 1 named from
  `grafana.folder` (blank means the project name), and each enabled dashboard
  from `grafana.dashboards` inside it. The Postgres datasource (named from
  `grafana.datasource_name`, default `Timescale`) is **shared** across
  projects: the one the server stack provisions is reused as-is, and it is
  only created here when missing — never modified.

**Dashboards** are a hand-written template library
(`applier/grafana/templates/`). A manifest references them by id; available ids
are the file stems there — currently `overview`, `per_station`,
`device_health`. At apply time the datasource UID and project schema are
substituted into the template. An id with no matching template is a plan
warning, not a hard error (apply skips it).

### Multi-tenancy

Two isolation mechanisms, both keyed off the manifest:

1. **Postgres schema per project** — each project's data lives in its own
   schema in the shared Timescale cluster. The ingester routes uplinks by
   bearer token via `registry.projects` (token → schema), maintained by Apply.
2. **Grafana folder per project** — all projects share org 1 (where the server
   stack provisions the `Timescale` datasource); each project owns a folder
   and the dashboards in it. Dashboard uids are prefixed `{project_id}-` so
   projects can't collide. All Grafana calls authenticate as the admin user
   (`GF_SECURITY_ADMIN_*`) — no per-project tokens.

### Safety model

- **Idempotent**: all writes are `CREATE ... IF NOT EXISTS` / `if_not_exists`
  / PUT-by-uid, so re-applying an in-sync manifest is a no-op.
- **No destructive ops**: dropping a schema, datasource, or dashboard raises
  `RefusedDestructive` — the applier never deletes.
- **Plan/apply integrity**: `/apply` enacts the exact plan you saw; if the
  manifest changed since planning (hash mismatch) the stale plan is refused and
  you must re-plan.
- **Audit log**: every apply is appended to `instance/apply_log.db` and shown
  at `/history`.

## Configuration

Authoring needs nothing. The applier reads its infra targets and admin secrets
from the **process environment** (never from the manifest, and note the app does
*not* load a `.env` file — set real env vars, e.g. in Coolify's UI).

Each setting reads its own applier-specific variable first, then falls back to
the name the Croptopus **server stack** already uses. So a manifest-builder
co-deployed in the same Coolify project needs **zero extra env vars** — it
reuses the stack's `DATABASE_URL` and `GF_SECURITY_ADMIN_*` — while the explicit
applier names still win when set (e.g. to point at a separate admin role).

| Setting                | Reads (in order)                              | Default               |
| ---------------------- | --------------------------------------------- | --------------------- |
| Timescale admin URL    | `TIMESCALE_ADMIN_URL` → `DATABASE_URL`        | — (required)          |
| Datasource DB template | `PROJECT_DATABASE_URL_TEMPLATE` → `DATABASE_URL` | local compose default |
| Grafana base URL       | `GRAFANA_URL`                                 | `http://grafana:3000` |
| Grafana admin user     | `GRAFANA_ADMIN_USER` → `GF_SECURITY_ADMIN_USER` | `admin`             |
| Grafana admin password | `GRAFANA_ADMIN_PASSWORD` → `GF_SECURITY_ADMIN_PASSWORD` | — (required) |

`TIMESCALE_ADMIN_URL` falling back to `DATABASE_URL` works because the stack's
`POSTGRES_USER` is the image superuser, so the runtime role can also create
schemas. Plan/Apply/Status stay disabled (with a flash explaining why) until the
admin URL, Grafana URL, and Grafana admin password all resolve. Reaching the
**private** TimescaleDB requires running inside the stack's network (so
`timescaledb:5432` / `grafana:3000` resolve). Runtime state — the apply audit
DB — lives under `instance/` (gitignored).

## Deploy on Coolify

manifest-builder is the **control plane**, so deploy it as its **own** Coolify
application (not merged into the `server/` data-plane stack), attached to that
stack's network. The included `Dockerfile` serves the app with gunicorn on port
`8000`.

1. **Build**: point Coolify at this `manifest-builder/` directory using the
   Dockerfile build pack.
2. **Network**: connect the app to the **same Docker network** as the server
   stack so `timescaledb:5432` and `grafana:3000` resolve. Required —
   TimescaleDB has no public route.
3. **Keep it private**: it has **no authentication** (single-user tool). Do not
   give it a public domain; reach it via Coolify private networking or an SSH
   tunnel.
4. **Persistent storage**: mount a volume at **`/app/instance`** so the apply
   audit DB survives redeploys.
5. **Environment** — reuse the stack's values via Coolify shared variables; the
   names line up thanks to the fallbacks above:

   | Variable | Value |
   | -------- | ----- |
   | `DATABASE_URL` | the stack's (covers both the admin URL and datasource template) |
   | `GF_SECURITY_ADMIN_USER` / `GF_SECURITY_ADMIN_PASSWORD` | the stack's |
   | `GRAFANA_URL` | `http://grafana:3000` (internal) |
   | `SECRET_KEY` | a fresh random string (signs the session cookie) |

   Running inside the network, the single `DATABASE_URL` serves both DB roles —
   no split hosts, no tunnel (that split is only needed when running off-network,
   e.g. from a laptop).

## Manifest schema

```
customer:   { id, name }
project:    { id, name, environment (prod/staging/dev), timezone }
gateways:   list of { id, ip?, lorawan_region, site_name, location: { lat, lon } }
nodes:      list of {
              device_id, hw_id?, gateway_id, role,
              location: { lat, lon, description },
              sensors: list of { type, channel },
              sampling: { interval_sec, transmit_every_n_samples },
            }
database:   { schema, retention_days, compression_after_days }
grafana:    { folder, datasource_name, dashboards: list of { id, enabled } }
alerts:     map of name -> { enabled, threshold?, duration_min }
```

Key ordering on export follows that top-level order. Nested keys are emitted
in dict-insertion order — stable across round trips.

## Storage seams

- **Working manifest**: a single in-memory dict in `app.py`
  (`get_working` / `set_working`, marked **STORAGE SEAM**). Fine for an
  internal single-user tool; swap for Postgres/SQLite/S3 when you need
  multi-user editing or persistence across restarts.
- **Saved manifests**: flat YAML files in `manifest-builder/saved/`,
  intentionally simple to inspect or swap out.
- **No auth.** Internal tool, single user assumed.

## Testing

```bash
uv run pytest                 # unit tests (plan diff logic; no infra)
```

`tests/test_plan.py` exercises the plan/diff engine with stubbed live state.
`tests/test_integration.py` is a round-trip against a **real** Timescale +
Grafana and is marked `integration` — skipped unless the infra env vars are
set.

## Layout

```
manifest-builder/
├── app.py                          # Flask app and routes (thin; dispatches to applier/)
├── manifest.py                     # YAML I/O, validation, form parsing
├── Dockerfile  .dockerignore       # production image (gunicorn) for Coolify deploy
├── applier/                        # executable layer: plan / apply / state / audit
│   ├── config.py                   # AppConfig from env (infra targets, secrets)
│   ├── plan.py  apply.py  state.py # diff, enact, read-live-state
│   ├── model.py  interfaces.py  errors.py
│   ├── apply_log.py                # append-only SQLite audit log
│   ├── timescale/schema_applier.py # per-project Postgres schema + policies
│   └── grafana/                    # folder, shared datasource + dashboards (org 1)
│       ├── dashboard_applier.py  dashboard_library.py
│       └── templates/              # overview / per_station / device_health JSON
├── examples/example_manifest.yaml  # starter manifest (matches the running stack)
├── saved/                          # persisted named manifests
├── instance/                       # runtime state: apply_log.db (gitignored)
├── static/style.css
└── templates/                      # base, index, editor, plan, status, apply_result, history
```
