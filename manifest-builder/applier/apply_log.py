"""Append-only audit log of applies, in a small SQLite file.

Standalone (no Flask). The web layer points it at
``instance/apply_log.db`` and renders /history from list_entries().
"""

import os
import json
import sqlite3
import datetime

_SCHEMA = """
CREATE TABLE IF NOT EXISTS apply_log (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp     TEXT NOT NULL,
    manifest_id   TEXT NOT NULL,
    manifest_hash TEXT NOT NULL,
    customer_id   TEXT,
    project_id    TEXT,
    plan_summary  TEXT,
    results_json  TEXT,
    success       INTEGER
);
"""


def _connect(db_path):
    os.makedirs(os.path.dirname(db_path), exist_ok=True)
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    return conn


def init_db(db_path):
    with _connect(db_path) as conn:
        conn.execute(_SCHEMA)


def record(db_path, *, manifest_id, manifest_hash, customer_id, project_id,
           plan_summary, results, success):
    init_db(db_path)
    with _connect(db_path) as conn:
        conn.execute(
            """INSERT INTO apply_log
               (timestamp, manifest_id, manifest_hash, customer_id, project_id,
                plan_summary, results_json, success)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
            (
                datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z",
                manifest_id, manifest_hash, customer_id, project_id,
                plan_summary, json.dumps(results), 1 if success else 0,
            ),
        )


def list_entries(db_path, limit=100):
    if not os.path.exists(db_path):
        return []
    with _connect(db_path) as conn:
        rows = conn.execute(
            "SELECT * FROM apply_log ORDER BY id DESC LIMIT ?", (limit,),
        ).fetchall()
    out = []
    for r in rows:
        d = dict(r)
        try:
            d["results"] = json.loads(d.get("results_json") or "null")
        except (ValueError, TypeError):
            d["results"] = None
        out.append(d)
    return out
