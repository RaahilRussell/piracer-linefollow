#!/usr/bin/env python3
"""One Mac console: Pi video/controls over HTTP, local recording analysis over SSH.

python3 -B pi/review_dashboard.py --pi CURRENT_PI_IP
No Pi writes or motion commands occur at startup. Browser actions control the car.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]
import webui
from recording_review import RecordingReview, pi_host


class ConsoleHandler(webui.Handler):
    def do_GET(self):
        if self.path.split("?")[0] in ("/", "/index.html"):
            return self._send(200, (HERE / "dashboard.html").read_bytes(), "text/html; charset=utf-8")
        if self.path.split("?")[0] == "/api/review":
            return self._json(dict(self.server.review.status(), available=True, default_host=self.server.pi_host,
                                   connected_console=True))
        return self.proxy("GET")

    def do_POST(self):
        if self.path.split("?")[0] != "/api/review":
            return self.proxy("POST")
        try:
            body = self._body()
            if body.get("action") == "cancel":
                return self._json(self.server.review.cancel())
            if body.get("action") != "start":
                raise ValueError("Unknown analysis action")
            with urlopen(self.server.pi_url + "/api/state", timeout=8) as response:
                state = json.load(response)
            knobs = {k["id"]: k["value"] for k in state["knobs"]}
            return self._json(self.server.review.start(self.server.pi_host, knobs, body.get("deep", True)))
        except Exception as e:
            return self._json({"error": str(e)}, 400)

    def proxy(self, method):
        # The destination is fixed by the local launcher, never by a request URL.
        try:
            data = None
            if method == "POST":
                length = int(self.headers.get("Content-Length") or 0)
                if length > 100_000:
                    raise ValueError("Request too large")
                data = self.rfile.read(length)
            request = Request(self.server.pi_url + self.path, data=data, method=method,
                              headers={"Content-Type": "application/json"})
            with urlopen(request, timeout=8) as response:
                if self.path.startswith(("/video", "/events")):
                    self.send_response(response.status)
                    self.send_header("Content-Type", response.headers.get("Content-Type", "application/octet-stream"))
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Connection", "close")
                    self.end_headers()
                    while True:
                        block = response.read1(16384)
                        if not block:
                            break
                        self.wfile.write(block); self.wfile.flush()
                    return
                return self._send(response.status, response.read(), response.headers.get("Content-Type", "application/json"))
        except HTTPError as e:
            return self._send(e.code, e.read(), e.headers.get("Content-Type", "application/json"))
        except (BrokenPipeError, ConnectionResetError):
            return
        except (URLError, TimeoutError, ValueError) as e:
            return self._json({"error": f"Pi connection unavailable: {e}"}, 502)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi", required=True, help="Current Pi IP")
    parser.add_argument("--port", type=int, default=8899)
    args = parser.parse_args()
    host = pi_host(args.pi)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), ConsoleHandler)
    server.pi_host = host
    server.pi_url = f"http://[{host}]:8890" if ":" in host else f"http://{host}:8890"
    server.review = RecordingReview()
    print(f"Driving console: http://localhost:{args.port} · connected to Pi {host}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
