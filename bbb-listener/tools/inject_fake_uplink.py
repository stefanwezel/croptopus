#!/usr/bin/env python3
"""
inject_fake_uplink.py — queue a synthetic uplink into the listener's SQLite outbox.

Lets you exercise the forward path end-to-end without LoRa hardware or a transmitting
XIAO. It writes a properly-formed ingester payload straight into the same outbox the
listener uses; the running listener's worker thread picks it up on its next forward
cycle and POSTs it to the server exactly as if it had arrived over the air.

Examples:
    python3 tools/inject_fake_uplink.py
    python3 tools/inject_fake_uplink.py --device-id node_4f2a --temperature 23.1 \
        --humidity 58.6 --soil 13
"""

import argparse
import os
import sys
from datetime import datetime, timezone

# Import the sibling forwarder module (one directory up).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import forwarder  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device-id", default="node_0001",
                        help='device id, "node_<4-hex>" form (default: node_0001)')
    parser.add_argument("--temperature", type=float, default=24.85,
                        help="temperature in C (default: 24.85)")
    parser.add_argument("--humidity", type=float, default=55.93,
                        help="relative humidity in %% (default: 55.93)")
    parser.add_argument("--soil", type=float, default=13,
                        help="soil moisture in %% (default: 13)")
    parser.add_argument("--gateway-id", default="inject-tool",
                        help="meta.gateway_id to stamp (default: inject-tool)")
    parser.add_argument(
        "--db",
        default=os.environ.get("QUEUE_DB_PATH", forwarder.DEFAULT_QUEUE_DB_PATH),
        help="outbox SQLite path (default: $QUEUE_DB_PATH or the listener default)",
    )
    args = parser.parse_args()

    payload = {
        "device_id": args.device_id,
        "timestamp": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "readings": [
            {"sensor_type": "temperature",   "value": args.temperature, "unit": "C"},
            {"sensor_type": "humidity",      "value": args.humidity,    "unit": "%"},
            {"sensor_type": "soil_moisture", "value": args.soil,        "unit": "%"},
        ],
        "meta": {
            "gateway_id": args.gateway_id,
            "rssi": -44,
            "snr": 13.5,
            "freq_mhz": 868.1,
            "datarate": "SF7BW125",
        },
    }

    outbox = forwarder.Outbox(args.db)
    outbox.init_schema()
    outbox.enqueue(payload)
    print("queued. The listener will pick it up on next forward cycle.")


if __name__ == "__main__":
    main()
