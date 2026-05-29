"""Croptopus manifest builder — Flask app.

Run with:
    uv run flask --app app run --debug

This exposes a single `app` object at module level so the Flask CLI command
above works without any extra config.
"""

from __future__ import annotations

import io
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from flask import (
    Flask,
    abort,
    flash,
    redirect,
    render_template,
    request,
    send_file,
    url_for,
)

import manifest as M

app = Flask(__name__)
app.secret_key = "dev-only-not-for-prod"  # flashes only; replace if exposing

EXAMPLE_PATH = Path(__file__).parent / "examples" / "example_manifest.yaml"
SAVED_DIR = Path(__file__).parent / "saved"
SAVED_DIR.mkdir(exist_ok=True)


# --- Working-manifest storage ----------------------------------------------
# STORAGE SEAM: v1 keeps a single in-memory working manifest. This is fine
# because the manifest builder is an internal, single-user tool. When we want
# multi-user editing or persistence across restarts, replace these two
# functions with a real backend (Postgres, SQLite, S3, whatever) — every call
# site goes through get_working() / set_working() so nothing else needs to
# change.

_working: dict[str, Any] = M.empty_manifest()


def get_working() -> dict[str, Any]:
    return _working


def set_working(manifest: dict[str, Any]) -> None:
    global _working
    _working = manifest


# --- Saved manifests -------------------------------------------------------
# STORAGE SEAM (saved): named manifests live as YAML files in SAVED_DIR. The
# directory is the source of truth — you can also drop files in by hand. When
# we move to a real backend, replace list_saved/load_saved/save_named/
# delete_saved with the equivalent DB operations.

_NAME_OK = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,63}$")


def _sanitize_name(raw: str) -> str:
    """Coerce a user-supplied name into something safe to use as a filename."""
    name = (raw or "").strip().lower()
    name = re.sub(r"[^a-z0-9_.-]+", "-", name).strip("-.")
    return name[:64]


def _saved_path(name: str) -> Path:
    """Resolve a saved-manifest path, refusing anything path-traversal-y."""
    if not _NAME_OK.match(name):
        abort(400, f"Invalid manifest name: {name!r}")
    p = (SAVED_DIR / f"{name}.yaml").resolve()
    if SAVED_DIR.resolve() not in p.parents:
        abort(400, "Invalid path")
    return p


def list_saved() -> list[dict[str, Any]]:
    """Summarize every saved manifest for the landing page."""
    out = []
    for p in sorted(SAVED_DIR.glob("*.yaml")):
        try:
            data = M.load_yaml(p.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            # Don't crash the listing page if one file is corrupted.
            out.append({
                "name": p.stem,
                "project_name": "(unreadable)",
                "environment": "?",
                "gateways": 0,
                "nodes": 0,
                "mtime": datetime.fromtimestamp(p.stat().st_mtime, tz=timezone.utc),
                "broken": True,
            })
            continue
        proj = data.get("project") or {}
        out.append({
            "name": p.stem,
            "project_name": proj.get("name") or proj.get("id") or "—",
            "environment": proj.get("environment") or "—",
            "gateways": len(data.get("gateways") or []),
            "nodes": len(data.get("nodes") or []),
            "mtime": datetime.fromtimestamp(p.stat().st_mtime, tz=timezone.utc),
            "broken": False,
        })
    out.sort(key=lambda d: d["mtime"], reverse=True)
    return out


# --- Routes -----------------------------------------------------------------

@app.route("/")
def index():
    return render_template("index.html", saved=list_saved())


@app.post("/new")
def new_manifest():
    set_working(M.empty_manifest())
    flash("Started a new empty manifest.", "ok")
    return redirect(url_for("editor"))


@app.post("/load-example")
def load_example():
    try:
        text = EXAMPLE_PATH.read_text(encoding="utf-8")
        set_working(M.load_yaml(text))
    except (OSError, ValueError) as e:
        flash(f"Could not load example manifest: {e}", "err")
        return redirect(url_for("index"))
    flash("Loaded example manifest.", "ok")
    return redirect(url_for("editor"))


@app.post("/upload")
def upload():
    f = request.files.get("manifest")
    if not f or not f.filename:
        flash("No file selected.", "err")
        return redirect(url_for("index"))
    try:
        text = f.read().decode("utf-8")
        loaded = M.load_yaml(text)
    except (UnicodeDecodeError, ValueError) as e:
        flash(f"Could not parse uploaded manifest: {e}", "err")
        return redirect(url_for("index"))
    set_working(loaded)
    flash(f"Loaded manifest from {f.filename}.", "ok")
    return redirect(url_for("editor"))


@app.route("/editor", methods=["GET", "POST"])
def editor():
    if request.method == "POST":
        try:
            updated = M.parse_form(request.form)
        except ValueError as e:
            flash(f"Could not parse form: {e}", "err")
            return redirect(url_for("editor"))
        set_working(updated)
        errs = M.validate(updated)
        if errs:
            for err in errs:
                flash(err, "err")
        else:
            flash("Manifest updated.", "ok")
        return redirect(url_for("editor"))

    # Validation errors surface only after a save attempt (via flash), not on
    # plain GET — otherwise a fresh blank manifest greets you with errors.
    return render_template("editor.html", m=get_working())


@app.get("/export")
def export():
    m = get_working()
    errs = M.validate(m)
    if errs:
        # We still let the user export an invalid manifest — they may be
        # mid-edit — but warn loudly.
        for err in errs:
            flash(err, "warn")
    text = M.dump_yaml(m)
    buf = io.BytesIO(text.encode("utf-8"))
    filename = f"{(m.get('project') or {}).get('id') or 'manifest'}.yaml"
    return send_file(
        buf,
        as_attachment=True,
        download_name=filename,
        mimetype="application/x-yaml",
    )


@app.post("/save")
def save_named():
    """Persist the current working manifest under a name in SAVED_DIR."""
    raw_name = request.form.get("name", "").strip()
    if not raw_name:
        proj_id = (get_working().get("project") or {}).get("id") or ""
        raw_name = proj_id
    name = _sanitize_name(raw_name)
    if not name or not _NAME_OK.match(name):
        flash("Please provide a name (letters, digits, _ . -).", "err")
        return redirect(url_for("editor"))
    p = _saved_path(name)
    existed = p.exists()
    p.write_text(M.dump_yaml(get_working()), encoding="utf-8")
    flash(f"{'Overwrote' if existed else 'Saved'} configuration '{name}'.", "ok")
    return redirect(url_for("editor"))


@app.post("/load-saved/<name>")
def load_saved(name: str):
    p = _saved_path(name)
    if not p.exists():
        flash(f"No saved configuration named '{name}'.", "err")
        return redirect(url_for("index"))
    try:
        set_working(M.load_yaml(p.read_text(encoding="utf-8")))
    except ValueError as e:
        flash(f"Could not load '{name}': {e}", "err")
        return redirect(url_for("index"))
    flash(f"Loaded saved configuration '{name}'.", "ok")
    return redirect(url_for("editor"))


@app.post("/delete-saved/<name>")
def delete_saved(name: str):
    p = _saved_path(name)
    if p.exists():
        p.unlink()
        flash(f"Deleted '{name}'.", "ok")
    else:
        flash(f"No saved configuration named '{name}'.", "err")
    return redirect(url_for("index"))


@app.get("/raw")
def raw_yaml():
    """Plain-text view of the current manifest as YAML — handy for debugging."""
    return (M.dump_yaml(get_working()), 200, {"Content-Type": "text/plain; charset=utf-8"})


# --- Future seam ------------------------------------------------------------
# A future "generate artifacts" step (firmware config, DB schema, dashboards)
# would hook in here as another route that reads get_working() and emits
# whatever is needed. v1 deliberately does NOT do this — the manifest builder
# only authors and round-trips the manifest.
