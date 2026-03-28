"""Minimal HTTP health endpoint for planner runtime observability."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
from typing import Callable


class _HealthHandler(BaseHTTPRequestHandler):
    snapshot_provider: Callable[[], dict[str, object]] | None = None

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path != "/healthz":
            self.send_response(404)
            self.end_headers()
            return

        provider = self.snapshot_provider
        snapshot = {} if provider is None else provider()
        payload = json.dumps(snapshot, separators=(",", ":"), sort_keys=True).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, fmt: str, *args: object) -> None:
        # Suppress default request logs
        return


def start_health_server(
    host: str,
    port: int,
    snapshot_provider: Callable[[], dict[str, object]],
) -> ThreadingHTTPServer:
    _HealthHandler.snapshot_provider = snapshot_provider
    server = ThreadingHTTPServer((host, port), _HealthHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server

