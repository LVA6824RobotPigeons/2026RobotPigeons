"""CLI entrypoint for the off-device BCNP planner server."""

from __future__ import annotations

import argparse
from pathlib import Path
import random

from coprocessor.protocol import schema
from coprocessor.service.health import start_health_server
from coprocessor.service.server import PlannerTcpServer
from coprocessor.service.session import SessionConfig


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="BCNP autonomous planner service scaffold")
    parser.add_argument("--host", default="0.0.0.0", help="TCP bind host")
    parser.add_argument("--port", default=5812, type=int, help="TCP bind port")
    parser.add_argument(
        "--schema-path",
        default="src/main/deploy/bcnp/messages.json",
        type=Path,
        help="Path to canonical BCNP schema",
    )
    parser.add_argument("--heartbeat-period-ms", default=250, type=int)
    parser.add_argument("--heartbeat-timeout-ms", default=1200, type=int)
    parser.add_argument("--phase-count-hint", default=5, type=int)
    parser.add_argument("--objective-id-hint", default=101, type=int)
    parser.add_argument("--policy-source-hint", default=1, type=int)
    parser.add_argument("--global-confidence-permille-hint", default=800, type=int)
    parser.add_argument(
        "--health-port",
        default=5813,
        type=int,
        help="Health endpoint port; set <0 to disable",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    raw_schema = args.schema_path.read_bytes()
    contract = schema.load_contract(args.schema_path)
    expected_hash = schema.schema_crc32(raw_schema)

    session_config = SessionConfig(
        expected_schema_hash=expected_hash,
        session_id=random.randint(1, 0x7FFFFFFF),
        heartbeat_period_ms=args.heartbeat_period_ms,
        heartbeat_timeout_ms=args.heartbeat_timeout_ms,
        phase_count_hint=args.phase_count_hint,
        objective_id_hint=args.objective_id_hint,
        policy_source_hint=args.policy_source_hint,
        global_confidence_permille_hint=args.global_confidence_permille_hint,
    )
    server = PlannerTcpServer(
        args.host,
        args.port,
        session_config,
        contract.wire_sizes_by_id(),
    )

    health_server = None
    if args.health_port >= 0:
        health_server = start_health_server(args.host, args.health_port, server.health_snapshot)

    try:
        server.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if health_server is not None:
            health_server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
