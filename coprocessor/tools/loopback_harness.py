"""Local loopback harness for validating planner handshake and plan response."""

from __future__ import annotations

import argparse
import socket
import struct
import time
from pathlib import Path

from coprocessor.protocol import bcnp, schema


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="BCNP planner loopback harness")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5812)
    parser.add_argument("--schema-path", type=Path, default=Path("src/main/deploy/bcnp/messages.json"))
    parser.add_argument("--timeout-s", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    contract = schema.load_contract(args.schema_path)
    schema_hash = schema.schema_crc32(args.schema_path.read_bytes())

    with socket.create_connection((args.host, args.port), timeout=args.timeout_s) as sock:
        sock.settimeout(args.timeout_s)

        remote_handshake = sock.recv(bcnp.HANDSHAKE_SIZE)
        if not bcnp.is_handshake_magic(remote_handshake):
            raise RuntimeError("remote did not send BCNP handshake magic")
        remote_hash = bcnp.read_handshake_schema_hash(remote_handshake)
        if remote_hash != schema_hash:
            raise RuntimeError(f"schema hash mismatch: expected 0x{schema_hash:08x} got 0x{remote_hash:08x}")

        sock.sendall(bcnp.build_handshake(schema_hash))

        plan_request_payload = struct.pack(">HBBiii", 1, 0, 0, 0, 0, 0)
        sock.sendall(bcnp.encode_packet(bcnp.MSG_AUTO_PLAN_REQUEST, 0, 1, plan_request_payload))

        deadline = time.time() + args.timeout_s
        buffer = bytearray()
        seen_response = False
        seen_heartbeat = False
        while time.time() < deadline:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buffer.extend(chunk)
            while len(buffer) >= bcnp.HEADER_SIZE + bcnp.CRC_SIZE:
                decoded = bcnp.decode_packet(buffer, wire_sizes=contract.wire_sizes_by_id())
                if decoded.error == bcnp.DecodeError.INCOMPLETE:
                    break
                if not decoded.is_ok:
                    raise RuntimeError(f"decode error: {decoded.error.value}")
                del buffer[: decoded.consumed_bytes]
                if decoded.message_type == bcnp.MSG_AUTO_PLAN_RESPONSE:
                    seen_response = True
                if decoded.message_type == bcnp.MSG_AUTO_HEARTBEAT:
                    seen_heartbeat = True
            if seen_response and seen_heartbeat:
                break

    if not seen_response:
        raise RuntimeError("did not observe AutoPlanResponse")
    if not seen_heartbeat:
        raise RuntimeError("did not observe AutoHeartbeat")
    print("loopback_ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

