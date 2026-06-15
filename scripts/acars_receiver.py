#!/usr/bin/env python3
# UDP receiver for Iridium ACARS push frames from the P4-NANO board.
#
# Each datagram is one JSON line from acars_push.c — same shape as
# GET /messages entries. Appends to daily NDJSON files under DATA_DIR
# and deletes files older than KEEP_DAYS on every day rollover.
#
# Usage:
#   scripts/acars_receiver.py
#   PORT=6700 DATA_DIR=~/data/acars scripts/acars_receiver.py
#
# Defaults: PORT=5005, DATA_DIR=~/data/iridium_acars, KEEP_DAYS=30

import datetime
import glob
import os
import socket
import sys

PORT = int(os.environ.get("PORT", 5005))
DATA_DIR = os.path.expanduser(os.environ.get("DATA_DIR", "~/data/iridium_acars"))
KEEP_DAYS = int(os.environ.get("KEEP_DAYS", 30))


def _rotate(data_dir: str, keep_days: int) -> None:
    cutoff = datetime.datetime.utcnow() - datetime.timedelta(days=keep_days)
    for path in glob.glob(os.path.join(data_dir, "acars_????????.ndjson")):
        name = os.path.basename(path)
        try:
            # acars_2026-06-15.ndjson → date part is chars [6:16]
            file_date = datetime.datetime.strptime(name[6:16], "%Y-%m-%d")
            if file_date < cutoff:
                os.remove(path)
                print(f"[rotate] removed {path}", flush=True)
        except (ValueError, OSError):
            pass


def main() -> None:
    os.makedirs(DATA_DIR, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", PORT))
    print(f"[acars_receiver] UDP :{PORT} → {DATA_DIR}", flush=True)

    current_day: str | None = None
    out_file = None

    while True:
        data, addr = sock.recvfrom(4096)
        now = datetime.datetime.utcnow()
        day = now.strftime("%Y-%m-%d")

        if day != current_day:
            if out_file is not None:
                out_file.close()
            _rotate(DATA_DIR, KEEP_DAYS)
            log_path = os.path.join(DATA_DIR, f"acars_{day}.ndjson")
            out_file = open(log_path, "a", buffering=1)  # line-buffered
            current_day = day
            print(f"[acars_receiver] logging to {log_path}", flush=True)

        line = data.decode("utf-8", errors="replace").rstrip("\n") + "\n"
        out_file.write(line)
        # stdout for journald — one line per frame
        print(f"[acars_receiver] {addr[0]}: {line.rstrip()}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
