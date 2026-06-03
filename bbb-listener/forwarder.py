#!/usr/bin/env python3
"""
forwarder.py — durable SQLite-backed outbox + HTTP forwarder for lora_listener.

The receive loop decodes an uplink and calls ``Outbox.enqueue(payload)``: a single
local SQLite INSERT, no network. A background worker thread (``ForwarderWorker``)
drains the outbox, POSTing each row to the ingester with bearer-token auth over
HTTPS and retrying with backoff.

A reading is durable the instant it is enqueued: it survives ingester outages, BBB
reboots and listener restarts. If the server is down for a day the outbox grows and
drains when the server comes back.

Only the standard library is used for the network path (urllib.request), whose
default SSL context verifies TLS certificates. There is deliberately no flag to
disable verification.
"""

from __future__ import annotations

import json
import logging
import sqlite3
import threading
import time
import urllib.error
import urllib.request
from contextlib import contextmanager
from datetime import datetime, timezone

log = logging.getLogger("lora_listener.forwarder")

DEFAULT_QUEUE_DB_PATH = "/home/debian/lora_listener_queue.sqlite3"

_CREATE_TABLE = """
CREATE TABLE IF NOT EXISTS outbox (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    queued_at   TEXT    NOT NULL,  -- ISO8601 UTC, for ordering and debug
    payload     TEXT    NOT NULL,  -- the JSON document, already serialized
    attempts    INTEGER NOT NULL DEFAULT 0,
    last_error  TEXT
);
"""
_CREATE_INDEX = "CREATE INDEX IF NOT EXISTS idx_outbox_queued_at ON outbox(queued_at);"


def _utc_now_iso() -> str:
    """ISO8601 UTC with an explicit Z suffix, second precision. Never naive."""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ============================================================================
# Outbox — the durable SQLite queue
# ============================================================================


class Outbox:
    """Thin SQLite wrapper around the ``outbox`` table.

    Every operation opens a short-lived connection so the receive thread (the
    writer) and the worker thread (the reader) never share a handle. WAL mode lets
    a writer and a reader proceed concurrently without blocking each other.
    """

    def __init__(self, db_path: str):
        self.db_path = db_path

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.db_path, timeout=30.0)
        # WAL: writer (receive loop) doesn't block reader (forwarder worker).
        conn.execute("PRAGMA journal_mode=WAL;")
        conn.execute("PRAGMA busy_timeout=30000;")
        return conn

    @contextmanager
    def _txn(self):
        conn = self._connect()
        try:
            with conn:  # commit on success, rollback on exception
                yield conn
        finally:
            conn.close()

    def init_schema(self) -> None:
        with self._txn() as conn:
            conn.execute(_CREATE_TABLE)
            conn.execute(_CREATE_INDEX)

    def enqueue(self, payload: dict) -> int:
        """Serialize and INSERT one payload. ``allow_nan=False`` guarantees a failed
        sensor read is encoded as JSON ``null`` and never as ``NaN``."""
        body = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        with self._txn() as conn:
            cur = conn.execute(
                "INSERT INTO outbox (queued_at, payload) VALUES (?, ?)",
                (_utc_now_iso(), body),
            )
            return cur.lastrowid

    def fetch_batch(self, batch_size: int, max_attempts: int):
        """Oldest-first batch of rows still eligible for forwarding (i.e. not yet
        stuck). Returns a list of (id, payload, attempts) tuples."""
        with self._txn() as conn:
            return conn.execute(
                "SELECT id, payload, attempts FROM outbox "
                "WHERE attempts < ? ORDER BY queued_at ASC, id ASC LIMIT ?",
                (max_attempts, batch_size),
            ).fetchall()

    def delete(self, row_id: int) -> None:
        with self._txn() as conn:
            conn.execute("DELETE FROM outbox WHERE id = ?", (row_id,))

    def mark_failure(self, row_id: int, error: str) -> None:
        with self._txn() as conn:
            conn.execute(
                "UPDATE outbox SET attempts = attempts + 1, last_error = ? WHERE id = ?",
                (error[:500], row_id),
            )

    def counts(self, max_attempts: int) -> tuple[int, int]:
        """(pending, stuck). ``stuck`` rows have hit max_attempts and are no longer
        retried; ``pending`` is everything else still in the outbox."""
        with self._txn() as conn:
            total = conn.execute("SELECT COUNT(*) FROM outbox").fetchone()[0]
            stuck = conn.execute(
                "SELECT COUNT(*) FROM outbox WHERE attempts >= ?", (max_attempts,)
            ).fetchone()[0]
        return total - stuck, stuck


# ============================================================================
# ForwarderWorker — drains the outbox to the ingester
# ============================================================================

STATUS_INTERVAL_S = 60.0
_AUTH_PAUSE_S = 60.0
_EMPTY_POLL_S = 5.0
_MAX_BACKOFF_S = 60.0


class ForwarderWorker(threading.Thread):
    """Daemon thread that continuously drains the outbox.

    Per row:
      * 2xx               -> DELETE the row, bump the forwarded counter.
      * 401/403           -> auth failure: pause 60s, leave rows untouched.
      * other 4xx         -> bump attempts/last_error; at max_attempts the row is
                             logged as STUCK and left in the outbox for a human.
      * 5xx / net / timeout -> bump attempts/last_error, exponential backoff to 60s.
    """

    def __init__(
        self,
        outbox: Outbox,
        url: str,
        token: str,
        *,
        timeout: float,
        batch_size: int,
        max_attempts: int,
    ):
        super().__init__(name="forwarder", daemon=True)
        self.outbox = outbox
        self.url = url
        self.token = token
        self.timeout = timeout
        self.batch_size = batch_size
        self.max_attempts = max_attempts

        self._stop_event = threading.Event()
        self._counter_lock = threading.Lock()
        self._forwarded = 0
        self._last_forward_monotonic: float | None = None

    # --- counters ----------------------------------------------------------

    def _record_forward(self) -> None:
        with self._counter_lock:
            self._forwarded += 1
            self._last_forward_monotonic = time.monotonic()

    @property
    def forwarded(self) -> int:
        with self._counter_lock:
            return self._forwarded

    # --- HTTP --------------------------------------------------------------

    def _post(self, payload_json: str) -> int:
        """POST one already-serialized payload. Returns the HTTP status on 2xx;
        raises urllib.error.HTTPError on 4xx/5xx and URLError/OSError on network
        problems. TLS is verified by urllib's default SSL context."""
        req = urllib.request.Request(
            self.url,
            data=payload_json.encode("utf-8"),
            method="POST",
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.token}",
            },
        )
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            return resp.status

    def _send_row(self, row_id: int, payload_json: str, attempts: int) -> str:
        """Forward one row. Returns one of: ok | auth | client_error | transient."""
        try:
            self._post(payload_json)
            self.outbox.delete(row_id)
            self._record_forward()
            return "ok"
        except urllib.error.HTTPError as e:
            code = e.code
            if code in (401, 403):
                log.error(
                    "forward auth failed (HTTP %d) — check INGEST_TOKEN; pausing %ds",
                    code, int(_AUTH_PAUSE_S),
                )
                return "auth"
            if 400 <= code < 500:
                self.outbox.mark_failure(row_id, f"HTTP {code}")
                if attempts + 1 >= self.max_attempts:
                    log.error(
                        "row %d STUCK after %d attempts (HTTP %d) — left in outbox "
                        "for human review, will not be retried",
                        row_id, attempts + 1, code,
                    )
                return "client_error"
            # 5xx — server-side, retry with backoff.
            self.outbox.mark_failure(row_id, f"HTTP {code}")
            return "transient"
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            self.outbox.mark_failure(row_id, str(e))
            return "transient"

    # --- main loop ---------------------------------------------------------

    def run(self) -> None:
        backoff = 1.0
        auth_pause_until = 0.0
        next_status = time.monotonic() + STATUS_INTERVAL_S

        while not self._stop_event.is_set():
            now = time.monotonic()
            if now >= next_status:
                self.log_status()
                next_status = now + STATUS_INTERVAL_S

            if now < auth_pause_until:
                self._stop_event.wait(min(_EMPTY_POLL_S, auth_pause_until - now))
                continue

            rows = self.outbox.fetch_batch(self.batch_size, self.max_attempts)
            if not rows:
                backoff = 1.0
                self._stop_event.wait(_EMPTY_POLL_S)
                continue

            transient = False
            for row_id, payload_json, attempts in rows:
                if self._stop_event.is_set():
                    break
                outcome = self._send_row(row_id, payload_json, attempts)
                if outcome == "ok":
                    backoff = 1.0
                elif outcome == "auth":
                    auth_pause_until = time.monotonic() + _AUTH_PAUSE_S
                    break
                elif outcome == "transient":
                    transient = True
                    break
                # client_error: row stays put, move on to the next one

            if transient:
                self._stop_event.wait(backoff)
                backoff = min(backoff * 2, _MAX_BACKOFF_S)

        log.debug("forwarder worker loop exited")

    # --- status / shutdown -------------------------------------------------

    def log_status(self) -> None:
        pending, stuck = self.outbox.counts(self.max_attempts)
        with self._counter_lock:
            forwarded = self._forwarded
            last = self._last_forward_monotonic
        last_str = "never" if last is None else f"{int(time.monotonic() - last)}s ago"
        log.info(
            "queue: %d pending, %d forwarded, %d stuck, last forward %s",
            pending, forwarded, stuck, last_str,
        )

    def stop(self) -> None:
        self._stop_event.set()

    def final_drain(self, deadline_seconds: float = 10.0) -> None:
        """Best-effort final flush, bounded by a deadline. Call after stop() and
        join() so it runs with no concurrent worker. Stops on an empty outbox, the
        deadline, or the first auth/transient failure (the server is unreachable —
        no point hammering during shutdown). Remaining rows replay on next start."""
        end = time.monotonic() + deadline_seconds
        while time.monotonic() < end:
            rows = self.outbox.fetch_batch(self.batch_size, self.max_attempts)
            if not rows:
                return
            for row_id, payload_json, attempts in rows:
                if time.monotonic() >= end:
                    return
                outcome = self._send_row(row_id, payload_json, attempts)
                if outcome in ("auth", "transient"):
                    return
