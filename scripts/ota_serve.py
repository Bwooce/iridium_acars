#!/usr/bin/env python3
# Range-capable HTTP server for P4 OTA.
#
# The device's OTA client (ota_runner partial_http_download) fetches the app in
# 32 KB `Range: bytes=` chunks and REQUIRES 206 Partial Content responses. The
# stock `python3 -m http.server` ignores Range and returns the whole file with
# 200 OK, which corrupts the download — so use THIS instead.
#
# Usage:
#   scripts/ota_serve.py [PORT] [DIR]
#     PORT default 8000; DIR default <repo>/p4-usb-host/build
#
# Then point the device's ota_url at http://<this-host>:PORT/p4-usb-host.bin
# (POST /config or it's already set) and trigger with `POST /ota`.
import http.server, socketserver, os, re, sys

_here = os.path.dirname(os.path.abspath(__file__))
_default_dir = os.path.join(_here, os.pardir, "p4-usb-host", "build")
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
DIRECTORY = os.path.abspath(sys.argv[2] if len(sys.argv) > 2 else _default_dir)


class RangeHandler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def do_HEAD(self):
        self._serve(head=True)

    def do_GET(self):
        self._serve(head=False)

    def _serve(self, head):
        path = os.path.normpath(os.path.join(DIRECTORY, self.path.lstrip("/")))
        if not path.startswith(DIRECTORY) or not os.path.isfile(path):
            self.send_error(404)
            return
        size = os.path.getsize(path)
        start, end, code = 0, size - 1, 200
        rng = self.headers.get("Range")
        if rng:
            m = re.match(r"bytes=(\d+)-(\d*)", rng)
            if m:
                start = int(m.group(1))
                end = int(m.group(2)) if m.group(2) else size - 1
                end = min(end, size - 1)
                code = 206
        length = end - start + 1
        self.send_response(code)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(length))
        if code == 206:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.end_headers()
        if head:
            return
        with open(path, "rb") as f:
            f.seek(start)
            rem = length
            while rem > 0:
                chunk = f.read(min(65536, rem))
                if not chunk:
                    break
                try:
                    self.wfile.write(chunk)
                except (BrokenPipeError, ConnectionResetError):
                    return
                rem -= len(chunk)

    def log_message(self, fmt, *args):
        # one concise line per request (Range visible) — handy during an OTA
        sys.stderr.write("  %s - %s\n" % (self.address_string(), fmt % args))


class ThreadingServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    if not os.path.isdir(DIRECTORY):
        sys.exit(f"error: dir not found: {DIRECTORY}")
    with ThreadingServer(("0.0.0.0", PORT), RangeHandler) as httpd:
        print(f"OTA range server on 0.0.0.0:{PORT} serving {DIRECTORY}", flush=True)
        print(f"  point device ota_url at http://<this-host>:{PORT}/p4-usb-host.bin, then POST /ota",
              flush=True)
        httpd.serve_forever()
