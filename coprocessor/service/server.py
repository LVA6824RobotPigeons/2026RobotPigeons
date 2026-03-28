"""TCP runtime."""

from __future__ import annotations

from collections import deque
import json
import socket
import time
from typing import Callable, Deque

from coprocessor.protocol import bcnp
from coprocessor.service.session import PlannerSession, SessionConfig, SessionState


class PlannerTcpServer:
    """Single-client TCP server for BCNP planner."""

    def __init__(
        self,
        host: str,
        port: int,
        session_config: SessionConfig,
        wire_sizes: dict[int, int],
        *,
        tick_ms: int = 20,
        logger: Callable[[dict], None] | None = None,
    ) -> None:
        self._host = host
        self._port = port
        self._session_config = session_config
        self._wire_sizes = wire_sizes
        self._tick_ms = tick_ms
        self._logger = logger or _default_logger

        self._server_socket: socket.socket | None = None
        self._client_socket: socket.socket | None = None
        self._session: PlannerSession | None = None
        self._tx_queue: Deque[memoryview] = deque()
        self._running = False

    def run_forever(self) -> None:
        self._start_server_socket()
        self._running = True
        self._log("planner_server_started", host=self._host, port=self._port)

        try:
            while self._running:
                now_ms = _now_ms()
                self._accept_if_available()
                self._read_if_available(now_ms)
                self._poll_session(now_ms)
                self._flush_tx()
                time.sleep(self._tick_ms / 1000.0)
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        self._running = False
        self._close_client("shutdown")
        if self._server_socket is not None:
            self._server_socket.close()
            self._server_socket = None
        self._log("planner_server_stopped")

    def health_snapshot(self) -> dict[str, object]:
        now_ms = _now_ms()
        if self._session is None:
            return {
                "server_running": self._running,
                "client_connected": False,
                "state": "IDLE",
                "last_fault": "none",
            }
        snapshot = self._session.health_snapshot(now_ms)
        snapshot["server_running"] = self._running
        snapshot["client_connected"] = self._client_socket is not None
        return snapshot

    def _start_server_socket(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setblocking(False)
        sock.bind((self._host, self._port))
        sock.listen(1)
        self._server_socket = sock

    def _accept_if_available(self) -> None:
        if self._server_socket is None or self._client_socket is not None:
            return
        try:
            client, addr = self._server_socket.accept()
        except BlockingIOError:
            return

        client.setblocking(False)
        self._client_socket = client
        self._session = PlannerSession(self._session_config, self._wire_sizes)
        self._tx_queue.clear()
        self._enqueue_tx(bcnp.build_handshake(self._session_config.expected_schema_hash))
        self._log("planner_client_connected", peer=f"{addr[0]}:{addr[1]}")

    def _read_if_available(self, now_ms: int) -> None:
        if self._client_socket is None or self._session is None:
            return
        try:
            chunk = self._client_socket.recv(4096)
        except BlockingIOError:
            return
        except OSError as exc:
            self._close_client(f"socket_error:{exc}")
            return

        if not chunk:
            self._close_client("client_closed")
            return

        for packet in self._session.feed(chunk, now_ms):
            self._enqueue_tx(packet)
        if self._session.state == SessionState.CLOSED:
            self._close_client(self._session.last_fault)

    def _poll_session(self, now_ms: int) -> None:
        if self._session is None:
            return
        for packet in self._session.poll(now_ms):
            self._enqueue_tx(packet)
        if self._session.state == SessionState.CLOSED:
            self._close_client(self._session.last_fault)

    def _enqueue_tx(self, packet: bytes) -> None:
        self._tx_queue.append(memoryview(packet))

    def _flush_tx(self) -> None:
        if self._client_socket is None:
            return
        while self._tx_queue:
            view = self._tx_queue[0]
            try:
                written = self._client_socket.send(view)
            except BlockingIOError:
                return
            except OSError as exc:
                self._close_client(f"send_error:{exc}")
                return
            if written <= 0:
                return
            if written == len(view):
                self._tx_queue.popleft()
            else:
                self._tx_queue[0] = view[written:]
                return

    def _close_client(self, reason: str) -> None:
        if self._client_socket is not None:
            try:
                self._client_socket.close()
            except OSError:
                pass
        self._client_socket = None
        self._tx_queue.clear()
        if self._session is not None:
            self._log("planner_client_disconnected", reason=reason, fault=self._session.last_fault)
        else:
            self._log("planner_client_disconnected", reason=reason, fault="none")
        self._session = None

    def _log(self, event: str, **fields: object) -> None:
        payload = {"event": event, "ts_ms": _now_ms(), **fields}
        self._logger(payload)


def _now_ms() -> int:
    return int(time.time() * 1000)


def _default_logger(payload: dict) -> None:
    print(json.dumps(payload, separators=(",", ":"), sort_keys=True), flush=True)
