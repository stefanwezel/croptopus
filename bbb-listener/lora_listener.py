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

Stops cleanly on SIGINT (Ctrl+C) and SIGTERM (systemd stop).
"""

import base64
import json
import logging
import signal
import socket
import struct
import sys
from datetime import datetime, timezone

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


def handle_rxpk(rxpk: dict):
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

    # JSON payload follows the 12-byte header
    try:
        payload = json.loads(pkt[12:].decode("utf-8", errors="replace"))
    except json.JSONDecodeError as e:
        log.warning("malformed PUSH_DATA JSON: %s", e)
        return

    for rxpk in payload.get("rxpk", []):
        handle_rxpk(rxpk)

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


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stdout,
    )

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

    sock.close()
    log.info("stopped")


if __name__ == "__main__":
    main()
