# manifest-builder

A small Flask web app for authoring the **Croptopus manifest** — the single
source of truth for the LoRa sensor network. You visually configure the
customer, project, gateways, sensor stations (with their sensors and sampling
config), database, Grafana dashboards, and alerts; then export a clean YAML
file. You can also upload an existing manifest and edit it back.

The manifest is **just the description**. Downstream artifacts (firmware
config, database schema, dashboard JSON) are not generated here yet — there's
a clearly-marked future seam in `app.py` for that.

## Run it

From inside `manifest-builder/`:

```bash
uv run flask --app app run --debug
```

`uv` will create a virtualenv and install Flask + PyYAML on first run.
Open <http://127.0.0.1:5000>.

## What v1 does

- **Landing page** offers: start a new manifest, load the bundled example,
  upload an existing YAML, or open a previously **saved** configuration.
- **Editor** has forms for every section. Gateways, nodes (with their
  sensors), Grafana dashboards, and alerts can be added/removed.
- **Save / Load named** — persist the current manifest under a name into
  `manifest-builder/saved/<name>.yaml`. The landing page lists every saved
  manifest with project name, environment, and gateway/node counts. You can
  also drop a YAML file into `saved/` by hand and it shows up there.
- **Export** downloads the current manifest as a clean, stably-ordered YAML.
- **Upload** parses a YAML file and loads it into the editor.
- **Round-trips**: export → re-upload reproduces the same logical config.
- **Validation**: surfaces friendly errors (e.g. a node references an unknown
  `gateway_id`, required field missing, numeric field not numeric) without
  crashing.

The bundled example (`examples/example_manifest.yaml`) reflects what we
currently run: one BBB+WM1302 gateway and two XIAO ESP32S3 sensor stations,
each with SHT35 (temperature/humidity) and a soil-moisture probe.

## What v1 deliberately doesn't do

- **No database.** The working manifest lives in a single in-memory dict in
  `app.py` (see `get_working` / `set_working` — marked **STORAGE SEAM** —
  this is where Postgres/SQLite/S3 would plug in). Saved manifests are flat
  YAML files in `manifest-builder/saved/`, intentionally simple and easy to
  inspect or swap out.
- **No artifact generation.** No firmware config, no DB schema, no dashboards
  emitted from the manifest. There's a comment in `app.py` marking where that
  hook will live.
- **No auth.** Internal tool, single user assumed.

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

## Layout

```
manifest-builder/
├── app.py                          # Flask app and routes
├── manifest.py                     # YAML I/O, validation, form parsing
├── pyproject.toml                  # uv-managed deps (Flask, PyYAML)
├── README.md
├── examples/
│   └── example_manifest.yaml       # dummy starter manifest
├── saved/                          # persisted named manifests (created on first save)
├── static/
│   └── style.css
└── templates/
    ├── base.html
    ├── index.html                  # landing
    └── editor.html                 # the editor UI
```
