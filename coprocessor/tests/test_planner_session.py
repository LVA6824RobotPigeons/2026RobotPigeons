from __future__ import annotations

from pathlib import Path
import struct
import unittest

from coprocessor.protocol import bcnp, schema
from coprocessor.service.session import PlannerSession, SessionConfig, SessionState


class PlannerSessionTest(unittest.TestCase):
    def setUp(self) -> None:
        schema_path = Path("src/main/deploy/bcnp/messages.json")
        contract = schema.load_contract(schema_path)
        self._wire_sizes = contract.wire_sizes_by_id()
        self._schema_hash = schema.schema_crc32(schema_path.read_bytes())

    def test_rejects_handshake_with_wrong_schema_hash(self) -> None:
        session = PlannerSession(
            SessionConfig(expected_schema_hash=self._schema_hash),
            self._wire_sizes,
        )
        bad_handshake = bcnp.build_handshake(self._schema_hash ^ 0x00FF00FF)
        outgoing = session.feed(bad_handshake, now_ms=1_000)
        self.assertEqual([], outgoing)
        self.assertEqual(SessionState.CLOSED, session.state)
        self.assertEqual("SCHEMA_HASH_MISMATCH", session.last_fault)

    def test_plan_request_produces_plan_response(self) -> None:
        session = PlannerSession(
            SessionConfig(expected_schema_hash=self._schema_hash),
            self._wire_sizes,
        )
        session.feed(bcnp.build_handshake(self._schema_hash), now_ms=1_000)
        self.assertEqual(SessionState.HEALTHY, session.state)

        plan_request_payload = struct.pack(">HBBiii", 2, 0, 0, 100, 200, 300)
        request_packet = bcnp.encode_packet(bcnp.MSG_AUTO_PLAN_REQUEST, 0, 1, plan_request_payload)
        outgoing = session.feed(request_packet, now_ms=1_050)
        self.assertEqual(6, len(outgoing))

        decoded = bcnp.decode_packet(outgoing[0], wire_sizes=self._wire_sizes)
        self.assertTrue(decoded.is_ok)
        self.assertEqual(bcnp.MSG_AUTO_PLAN_RESPONSE, decoded.message_type)
        (
            plan_id,
            flags,
            phase_count,
            checksum,
            objective_id,
            policy_source,
            reserved0,
            global_confidence_permille,
            reserved1,
        ) = struct.unpack(">IHHIHBBHH", decoded.payload)
        self.assertEqual(1, plan_id)
        self.assertEqual(0, flags)
        self.assertEqual(5, phase_count)
        self.assertNotEqual(0, checksum)
        self.assertEqual(102, objective_id)
        self.assertEqual(1, policy_source)
        self.assertEqual(0, reserved0)
        self.assertEqual(790, global_confidence_permille)
        self.assertEqual(0, reserved1)

        phase_packets = outgoing[1:]
        for sequence, packet in enumerate(phase_packets, start=1):
            decoded_phase = bcnp.decode_packet(packet, wire_sizes=self._wire_sizes)
            self.assertTrue(decoded_phase.is_ok)
            self.assertEqual(bcnp.MSG_AUTO_PHASE_COMMAND, decoded_phase.message_type)
            self.assertEqual(1, decoded_phase.message_count)
            (
                phase_plan_id,
                phase_seq,
                phase_type,
                timeout_ms,
                phase_flags,
                arg0,
                arg1,
                intent_type,
                safety_class,
                phase_policy_source,
                fallback_hint,
                phase_objective_id,
                path_id,
                confidence_permille,
                deadline_ms,
            ) = struct.unpack(">IHHHHiiBBBBHHHH", decoded_phase.payload)
            self.assertEqual(1, phase_plan_id)
            self.assertEqual(sequence, phase_seq)
            self.assertGreater(phase_type, 0)
            self.assertGreater(timeout_ms, 0)
            self.assertEqual(0, phase_flags)
            self.assertEqual(0, arg0)
            self.assertEqual(0, arg1)
            self.assertGreater(intent_type, 0)
            self.assertGreater(safety_class, 0)
            self.assertEqual(1, phase_policy_source)
            self.assertEqual(1, fallback_hint)
            self.assertEqual(102, phase_objective_id)
            self.assertGreaterEqual(path_id, 0)
            self.assertGreater(confidence_permille, 0)
            self.assertGreaterEqual(deadline_ms, timeout_ms)

    def test_conservative_profile_emits_two_phase_program(self) -> None:
        session = PlannerSession(
            SessionConfig(expected_schema_hash=self._schema_hash),
            self._wire_sizes,
        )
        session.feed(bcnp.build_handshake(self._schema_hash), now_ms=1_000)

        plan_request_payload = struct.pack(">HBBiii", 13, 0, 0, 6000, 1000, 0)
        request_packet = bcnp.encode_packet(bcnp.MSG_AUTO_PLAN_REQUEST, 0, 1, plan_request_payload)
        outgoing = session.feed(request_packet, now_ms=1_050)
        self.assertEqual(3, len(outgoing))

        decoded_plan = bcnp.decode_packet(outgoing[0], wire_sizes=self._wire_sizes)
        self.assertTrue(decoded_plan.is_ok)
        (
            _plan_id,
            _flags,
            phase_count,
            _checksum,
            objective_id,
            _policy_source,
            _reserved0,
            confidence,
            _reserved1,
        ) = struct.unpack(">IHHIHBBHH", decoded_plan.payload)
        self.assertEqual(2, phase_count)
        self.assertEqual(203, objective_id)
        self.assertEqual(900, confidence)

    def test_unknown_profile_near_centerline_falls_back_to_conservative_routine(self) -> None:
        session = PlannerSession(
            SessionConfig(expected_schema_hash=self._schema_hash),
            self._wire_sizes,
        )
        session.feed(bcnp.build_handshake(self._schema_hash), now_ms=1_000)

        # Unknown profile near centerline should force conservative mobility.
        plan_request_payload = struct.pack(">HBBiii", 777, 0, 0, 8270, 2500, 0)
        request_packet = bcnp.encode_packet(bcnp.MSG_AUTO_PLAN_REQUEST, 0, 1, plan_request_payload)
        outgoing = session.feed(request_packet, now_ms=1_050)
        decoded_plan = bcnp.decode_packet(outgoing[0], wire_sizes=self._wire_sizes)
        self.assertTrue(decoded_plan.is_ok)
        _, _, phase_count, _, objective_id, _, _, _, _ = struct.unpack(">IHHIHBBHH", decoded_plan.payload)
        self.assertEqual(2, phase_count)
        self.assertEqual(203, objective_id)

    def test_poll_sends_heartbeat_after_period(self) -> None:
        session = PlannerSession(
            SessionConfig(expected_schema_hash=self._schema_hash, session_id=42, heartbeat_period_ms=100),
            self._wire_sizes,
        )
        session.feed(bcnp.build_handshake(self._schema_hash), now_ms=1_000)

        packets = session.poll(now_ms=1_050)
        self.assertEqual(1, len(packets))
        decoded = bcnp.decode_packet(packets[0], wire_sizes=self._wire_sizes)
        self.assertTrue(decoded.is_ok)
        self.assertEqual(bcnp.MSG_AUTO_HEARTBEAT, decoded.message_type)
        session_id, sequence, timestamp = struct.unpack(">III", decoded.payload)
        self.assertEqual(42, session_id)
        self.assertEqual(0, sequence)
        self.assertEqual(1_050, timestamp)

        self.assertEqual([], session.poll(now_ms=1_100))
        next_packets = session.poll(now_ms=1_151)
        self.assertEqual(1, len(next_packets))
        next_decoded = bcnp.decode_packet(next_packets[0], wire_sizes=self._wire_sizes)
        _, next_sequence, next_timestamp = struct.unpack(">III", next_decoded.payload)
        self.assertEqual(1, next_sequence)
        self.assertEqual(1_151, next_timestamp)


if __name__ == "__main__":
    unittest.main()
