#!/usr/bin/env python3
"""
lora_listener.py — local receiver for raw-LoRa sensor frames

Listens on UDP/1700 (127.0.0.1) for traffic from sx1302_hal's lora_pkt_fwd.
Decodes our XIAO sensor stations' 7-byte payload format and prints one
human-readable line per uplink, plus RF metadata (RSSI, SNR, frequency).

Wire format expected (matches xiao_lora_sensor.ino):
    [0..1] uint16  short station ID, big-endian  (MAC-derived FNV-1a-16,
                                                  or pinned via --force-id
                                                  at flash time)
    [2..3] int16   temperature × 100, big-endian (0x8000 = sentinel "NaN")
    [4..5] uint16  humidity × 100,    big-endian (0xFFFF = sentinel "NaN")
    [6]    uint8   soil moisture %                (0..100)

The short ID is a 16-bit hash of the chip's factory MAC, so it's stable per
device but tells you nothing on its own about which physical station it
belongs to. Cross-reference against stations.csv (written by
flash_xiao_station.sh) to map a short ID back to a labelled device.

Anything that isn't 7 bytes is printed as hex with metadata so unknown
traffic is still visible (other LoRa devices in the area, stations still
running the old 6-byte firmware, etc).

Forwarding (NEW): each successfully-decoded uplink is also written to a durable
SQLite outbox and forwarded to the server-side ingester by a background worker
thread (see forwarder.py). The UDP receive loop's job ends at the outbox INSERT;
it never blocks on network I/O. Readings survive ingester outages, BBB reboots and
listener restarts. Without INGEST_URL + INGEST_TOKEN (or with --no-forward) the
listener runs in degraded mode: decode + log only, nothing queued or forwarded.

Stops cleanly on SIGINT (Ctrl+C) and SIGTERM (systemd stop).
"""

import argparse
import base64
import json
import logging
import os
import signal
import socket
import struct
import sys
from datetime import datetime, timezone

import forwarder

try:
    from dotenv import load_dotenv
except ImportError:  # python-dotenv is optional; env can be set the usual way
    def load_dotenv(*_args, **_kwargs):
        return False

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ============================================================================

LISTEN_HOST = "127.0.0.1"
LISTEN_PORT = 1700

# Semtech UDP protocol packet identifiers (PROTOCOL.TXT in the packet_forwarder repo)
PUSH_DATA = 0x00
PUSH_ACK  = 0x01
PULL_DATA = 0x02
PULL_RESP = 0x03
PULL_ACK  = 0x04

# Expected sensor payload size (in bytes)
SENSOR_PAYLOAD_LEN = 7

# ============================================================================

log = logging.getLogger("lora_listener")

# Forwarding state, wired up in main(). Both stay None in degraded / --no-forward
# mode, in which case the receive loop decodes + logs but queues nothing.
_outbox: "forwarder.Outbox | None" = None
_worker: "forwarder.ForwarderWorker | None" = None
# Optional friendly gateway name (GATEWAY_ID); when unset we use the EUI from the
# PUSH_DATA frame header.
_gateway_id_override = ""


def format_device_id(short_id: int) -> str:
    """Map a numeric station short ID to the dashboards' "node_<4-hex>" key.

    Station short ID 0x0001 -> "node_0001", 0x4F2A -> "node_4f2a". Accepts a wider
    int unchanged, so the future MAC-derived 2-byte short ID needs no further work.
    """
    return f"node_{short_id:04x}"


def rxpk_timestamp(rxpk: dict) -> str:
    """When the gateway received the uplink, as ISO8601 UTC with an explicit Z.

    Prefer the gateway's own "time" field when present (GPS/host UTC); otherwise
    fall back to now(). Always UTC, never naive."""
    raw = rxpk.get("time")
    if raw:
        try:
            dt = datetime.fromisoformat(raw.replace("Z", "+00:00"))
            if dt.tzinfo is None:
                dt = dt.replace(tzinfo=timezone.utc)
            return dt.astimezone(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        except (ValueError, AttributeError):
            pass
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def build_payload(decoded: dict, rxpk: dict, gateway_id: str) -> dict:
    """Assemble the ingester JSON document from a decoded frame + RF metadata.

    Shape matches server/ingester/app/models.py exactly. A failed sensor read is
    carried as None here and serialized to JSON null by the outbox."""
    return {
        "device_id": format_device_id(decoded["short_id"]),
        "timestamp": rxpk_timestamp(rxpk),
        "readings": [
            {"sensor_type": "temperature",   "value": decoded["temperature"],   "unit": "C"},
            {"sensor_type": "humidity",      "value": decoded["humidity"],      "unit": "%"},
            {"sensor_type": "soil_moisture", "value": decoded["soil_moisture"], "unit": "%"},
        ],
        "meta": {
            "gateway_id": gateway_id,
            "rssi": rxpk.get("rssi"),
            "snr": rxpk.get("lsnr"),
            "freq_mhz": rxpk.get("freq"),
            "datarate": rxpk.get("datr"),
        },
    }


def decode_sensor_payload(data: bytes):
    """Decode a 7-byte sensor frame. Returns None if length doesn't match."""
    if len(data) != SENSOR_PAYLOAD_LEN:
        return None

    short_id, t_raw, h_raw, soil = struct.unpack(">HhHB", data)
    temperature = None if t_raw == -32768 else t_raw / 100.0
    humidity    = None if h_raw == 0xFFFF else h_raw / 100.0
    return {
        "short_id": short_id,
        "temperature": temperature,
        "humidity": humidity,
        "soil_moisture": soil,
    }


def format_value(v, fmt, na="  n/a"):
    return na if v is None else fmt.format(v)


def handle_rxpk(rxpk: dict, gateway_id: str = "unknown"):
    """One received-packet entry from the gateway's PUSH_DATA JSON."""
    try:
        raw = base64.b64decode(rxpk["data"])
    except (KeyError, ValueError) as e:
        log.warning("rxpk without decodable data: %s", e)
        return

    ts = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ")
    freq = rxpk.get("freq", "?")
    rssi = rxpk.get("rssi", "?")
    snr  = rxpk.get("lsnr", "?")
    dr   = rxpk.get("datr", "?")

    decoded = decode_sensor_payload(raw)
    if decoded is not None:
        # Persist to the outbox BEFORE logging — the receive loop's job ends here;
        # forwarding is exclusively the worker thread's. Only decoded frames are
        # queued (the "unknown payload" branch below is never forwarded).
        if _outbox is not None:
            try:
                _outbox.enqueue(build_payload(decoded, rxpk, gateway_id))
            except Exception as e:  # never let a queue hiccup break the receive path
                log.error("failed to enqueue uplink to outbox: %s", e)
        log.info(
            "[%s] station 0x%04X  T=%s°C  RH=%s%%  soil=%3d%%   "
            "(%s MHz, RSSI %s dBm, SNR %s dB, %s)",
            ts,
            decoded["short_id"],
            format_value(decoded["temperature"], "{:5.2f}"),
            format_value(decoded["humidity"],    "{:5.2f}"),
            decoded["soil_moisture"],
            freq, rssi, snr, dr,
        )
    else:
        log.info(
            "[%s] unknown payload (%d bytes): %s   "
            "(%s MHz, RSSI %s dBm, SNR %s dB, %s)",
            ts, len(raw), raw.hex(" "),
            freq, rssi, snr, dr,
        )


def handle_push_data(sock: socket.socket, addr, pkt: bytes):
    """PUSH_DATA: gateway delivering received packets or stats. Ack + parse."""
    if len(pkt) < 12:
        return

    # First, ack so the gateway is happy (token from bytes 1..2)
    ack = bytes([pkt[0], pkt[1], pkt[2], PUSH_ACK])
    sock.sendto(ack, addr)

    # Bytes 4..11 of a PUSH_DATA frame are the gateway's unique EUI. Prefer an
    # explicit GATEWAY_ID override (a friendly name) when set; else use the EUI.
    gateway_id = _gateway_id_override or pkt[4:12].hex() or "unknown"

    # JSON payload follows the 12-byte header
    try:
        payload = json.loads(pkt[12:].decode("utf-8", errors="replace"))
    except json.JSONDecodeError as e:
        log.warning("malformed PUSH_DATA JSON: %s", e)
        return

    for rxpk in payload.get("rxpk", []):
        handle_rxpk(rxpk, gateway_id)

    # Gateway stat block — optionally log periodically
    stat = payload.get("stat")
    if stat is not None:
        log.debug("gateway stat: %s", stat)


def handle_pull_data(sock: socket.socket, addr, pkt: bytes):
    """PULL_DATA: gateway heartbeat. Ack so it knows we're alive."""
    if len(pkt) < 12:
        return
    ack = bytes([pkt[0], pkt[1], pkt[2], PULL_ACK])
    sock.sendto(ack, addr)


# ============================================================================

_running = True


def _stop(signum, _frame):
    global _running
    log.info("received signal %d, shutting down", signum)
    _running = False


def _setup_forwarding(no_forward: bool):
    """Build the outbox + worker from the environment.

    Returns (outbox, worker), or (None, None) for degraded / --no-forward mode.
    Missing INGEST_URL or INGEST_TOKEN is a WARNING, not a crash: decode + log
    still happen, but nothing is queued (draining is impossible without a URL).
    """
    if no_forward:
        log.warning("forwarding disabled via --no-forward: decode + log only, nothing queued")
        return None, None

    url = os.environ.get("INGEST_URL")
    token = os.environ.get("INGEST_TOKEN")
    if not url or not token:
        log.warning(
            "INGEST_URL and/or INGEST_TOKEN not set — running in DEGRADED mode "
            "(decode + log only, no forwarding, nothing queued)"
        )
        return None, None

    db_path = os.environ.get("QUEUE_DB_PATH", forwarder.DEFAULT_QUEUE_DB_PATH)
    timeout = float(os.environ.get("INGEST_TIMEOUT_SECONDS", "10"))
    batch_size = int(os.environ.get("INGEST_BATCH_SIZE", "50"))
    max_attempts = int(os.environ.get("INGEST_MAX_ATTEMPTS", "10"))

    outbox = forwarder.Outbox(db_path)
    outbox.init_schema()
    worker = forwarder.ForwarderWorker(
        outbox, url, token,
        timeout=timeout, batch_size=batch_size, max_attempts=max_attempts,
    )
    log.info(
        "forwarding enabled -> %s  (queue=%s, batch=%d, max_attempts=%d, timeout=%ss)",
        url, db_path, batch_size, max_attempts, timeout,
    )
    return outbox, worker


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1] if __doc__ else None)
    parser.add_argument(
        "--no-forward",
        action="store_true",
        help="disable forwarding entirely (decode + log only; nothing is queued)",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )

    # Load bbb-listener/.env if present (no-op when python-dotenv isn't installed
    # or the file is absent); real env vars still win over .env values.
    load_dotenv(os.path.join(SCRIPT_DIR, ".env"))

    global _outbox, _worker, _gateway_id_override
    _gateway_id_override = os.environ.get("GATEWAY_ID", "").strip()
    _outbox, _worker = _setup_forwarding(args.no_forward)
    if _worker is not None:
        _worker.start()

    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((LISTEN_HOST, LISTEN_PORT))
    sock.settimeout(1.0)  # so signals are processed promptly
    log.info("listening on %s:%d/udp", LISTEN_HOST, LISTEN_PORT)

    while _running:
        try:
            pkt, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        except OSError as e:
            log.error("socket error: %s", e)
            break

        if len(pkt) < 4:
            continue

        pkt_type = pkt[3]
        if pkt_type == PUSH_DATA:
            handle_push_data(sock, addr, pkt)
        elif pkt_type == PULL_DATA:
            handle_pull_data(sock, addr, pkt)
        # other packet types (PUSH_ACK, PULL_ACK, PULL_RESP, TX_ACK) ignored —
        # we are acting as the server, so we send those, we don't receive them.

    sock.close()  # stop accepting new UDP packets

    # Signal the worker to stop, let it finish its current op, then do one final
    # best-effort flush (10s) in this thread. Anything left replays on next start.
    if _worker is not None:
        log.info("draining outbox before exit (best-effort, 10s)...")
        _worker.stop()
        _worker.join(timeout=2.0)
        _worker.final_drain(10.0)
        pending, stuck = _outbox.counts(_worker.max_attempts)
        log.info(
            "shutting down. %d pending in queue (will replay on next start). %d stuck.",
            pending, stuck,
        )

    log.info("stopped")


if __name__ == "__main__":
    main()
