"""Connection-level session state machine for BCNP traffic."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import struct
from typing import Mapping
import zlib

from coprocessor.protocol import bcnp
from coprocessor.service.classical_routines import ClassicalRoutine, PhaseProgram, select_routine
from coprocessor.service.tactical_planner import TacticalPlanner, TacticalPlan
from coprocessor.service.world_model import WorldModel, ShiftState, OpponentTrack, Pose2D


class SessionState(str, Enum):
    CONNECTING = "CONNECTING"
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    CLOSED = "CLOSED"


@dataclass(frozen=True)
class SessionConfig:
    expected_schema_hash: int
    session_id: int = 1
    heartbeat_period_ms: int = 250
    heartbeat_timeout_ms: int = 1200
    phase_count_hint: int = 5
    objective_id_hint: int = 101
    policy_source_hint: int = 1
    global_confidence_permille_hint: int = 800
    use_tactical_planner: bool = True


class PlannerSession:
    """State machine for validating stream rules and emitting packets."""

    def __init__(self, config: SessionConfig, wire_sizes: Mapping[int, int]) -> None:
        self._config = config
        self._wire_sizes = dict(wire_sizes)

        self.state = SessionState.CONNECTING
        self.last_fault = "none"

        self._handshake = bytearray()
        self._pending = bytearray()
        self._last_rx_ms: int | None = None
        self._last_tx_heartbeat_ms: int = 0
        self._heartbeat_seq = 0
        self._next_plan_id = 1

        # Dynamic planning infra
        self._world_model = WorldModel()
        self._opponent_accumulator: list[OpponentTrack] = []
        self._tactical_planner = TacticalPlanner()
        self._last_tactical_plan: TacticalPlan | None = None

    @property
    def handshake_validated(self) -> bool:
        return self.state in (SessionState.HEALTHY, SessionState.DEGRADED)

    @property
    def world_model(self) -> WorldModel:
        """Expose world model for external updates."""
        return self._world_model

    def feed(self, data: bytes, now_ms: int) -> list[bytes]:
        if self.state == SessionState.CLOSED:
            return []
        if data:
            self._last_rx_ms = now_ms

        outgoing: list[bytes] = []
        cursor = 0
        if not self.handshake_validated:
            need = bcnp.HANDSHAKE_SIZE - len(self._handshake)
            take = min(need, len(data))
            if take > 0:
                self._handshake.extend(data[:take])
                cursor = take
            if len(self._handshake) < bcnp.HANDSHAKE_SIZE:
                return outgoing

            if not bcnp.is_handshake_magic(self._handshake):
                self._fault_close("BAD_HANDSHAKE_MAGIC")
                return outgoing

            remote_hash = bcnp.read_handshake_schema_hash(self._handshake)
            if remote_hash != self._config.expected_schema_hash:
                self._fault_close("SCHEMA_HASH_MISMATCH")
                return outgoing

            self.state = SessionState.HEALTHY
            self._last_rx_ms = now_ms

        if cursor < len(data):
            self._pending.extend(data[cursor:])

        while len(self._pending) >= bcnp.HEADER_SIZE + bcnp.CRC_SIZE:
            decoded = bcnp.decode_packet(self._pending, wire_sizes=self._wire_sizes)
            if decoded.error == bcnp.DecodeError.INCOMPLETE:
                break
            if not decoded.is_ok:
                self.last_fault = f"DECODE_{decoded.error.value}"
                if decoded.consumed_bytes > 0:
                    del self._pending[: decoded.consumed_bytes]
                    continue
                self._fault_close(self.last_fault)
                break

            del self._pending[: decoded.consumed_bytes]
            outgoing.extend(self._handle_packet(decoded, now_ms))
            self._last_rx_ms = now_ms

        return outgoing

    def poll(self, now_ms: int) -> list[bytes]:
        if self.state == SessionState.CLOSED or not self.handshake_validated:
            return []

        self._world_model.tick(now_ms)

        outgoing: list[bytes] = []
        if now_ms - self._last_tx_heartbeat_ms >= self._config.heartbeat_period_ms:
            payload = struct.pack(">III", self._config.session_id, self._heartbeat_seq, now_ms & 0xFFFFFFFF)
            outgoing.append(
                bcnp.encode_packet(
                    bcnp.MSG_AUTO_HEARTBEAT,
                    0,
                    1,
                    payload,
                )
            )
            self._heartbeat_seq += 1
            self._last_tx_heartbeat_ms = now_ms

        if self._last_rx_ms is not None and now_ms - self._last_rx_ms > self._config.heartbeat_timeout_ms:
            self._fault_close("HEARTBEAT_TIMEOUT")
        return outgoing

    def health_snapshot(self, now_ms: int) -> dict[str, int | str | bool]:
        age_ms = -1 if self._last_rx_ms is None else now_ms - self._last_rx_ms
        snapshot: dict[str, int | str | bool] = {
            "state": self.state.value,
            "last_fault": self.last_fault,
            "handshake_validated": self.handshake_validated,
            "rx_age_ms": age_ms,
            "next_plan_id": self._next_plan_id,
            "heartbeat_sequence": self._heartbeat_seq,
        }
        if self._last_tactical_plan is not None:
            snapshot["last_plan_type"] = self._last_tactical_plan.candidate_type.name
            snapshot["last_plan_score"] = int(self._last_tactical_plan.score.total * 100)
            snapshot["last_plan_desc"] = self._last_tactical_plan.description
        return snapshot

    def _handle_packet(self, decoded: bcnp.DecodedPacket, now_ms: int) -> list[bytes]:
        if decoded.message_type == bcnp.MSG_AUTO_PLAN_REQUEST:
            if len(decoded.payload) != self._wire_sizes[bcnp.MSG_AUTO_PLAN_REQUEST]:
                self.last_fault = "PLAN_REQUEST_BAD_SIZE"
                return []
            return self._build_plan_packets(decoded.payload, now_ms)
        if decoded.message_type == bcnp.MSG_AUTO_WORLD_UPDATE:
            self._handle_world_update(decoded.payload, now_ms)
            return []
        if decoded.message_type == bcnp.MSG_AUTO_OPPONENT_UPDATE:
            self._handle_opponent_update(decoded.payload, now_ms)
            return []
        if decoded.message_type == bcnp.MSG_AUTO_ABORT:
            self.last_fault = "ROBOT_ABORT"
            self.state = SessionState.DEGRADED
            return []
        return []

    def _handle_world_update(self, payload: bytes, now_ms: int) -> None:
        """Process world update (fuel state, velocity, pose)."""
        expected_size = self._wire_sizes.get(bcnp.MSG_AUTO_WORLD_UPDATE, 0)
        if len(payload) != expected_size:
            self.last_fault = "WORLD_UPDATE_BAD_SIZE"
            return

        (
            fuel_held, last_shot_success, phase_seq_completed, event_flags,
            robot_vx, robot_vy, pose_x, pose_y, heading_mrad, _reserved,
        ) = struct.unpack(">BBHHhhiihi", payload)

        wm = self._world_model
        wm.update_fuel_held(fuel_held)
        wm.update_robot_pose(pose_x, pose_y, heading_mrad)
        wm.update_robot_velocity(robot_vx, robot_vy)
        if last_shot_success:
            wm.record_shot_result(True)

    def _handle_opponent_update(self, payload: bytes, now_ms: int) -> None:
        """Process opponent track update."""
        expected_size = self._wire_sizes.get(bcnp.MSG_AUTO_OPPONENT_UPDATE, 0)
        if len(payload) != expected_size:
            self.last_fault = "OPPONENT_UPDATE_BAD_SIZE"
            return

        (
            track_index, track_count, pose_x, pose_y,
            vx, vy, confidence, _reserved,
        ) = struct.unpack(">BBiihhHH", payload)

        if track_index == 0:
            self._opponent_accumulator = []

        if track_count > 0:
            self._opponent_accumulator.append(
                OpponentTrack(
                    pose=Pose2D(x_mm=pose_x, y_mm=pose_y, heading_mrad=0),
                    vx_mm_s=vx,
                    vy_mm_s=vy,
                    confidence_permille=confidence,
                    last_seen_ms=now_ms,
                )
            )

        if len(self._opponent_accumulator) >= track_count:
            self._world_model.update_opponents(self._opponent_accumulator)

    def _build_plan_packets(self, plan_request_payload: bytes, now_ms: int) -> list[bytes]:
        requested_profile, alliance, _reserved, pose_x_mm, pose_y_mm, heading_millirad = struct.unpack(
            ">HBBiii",
            plan_request_payload,
        )

        self._world_model.update_robot_pose(pose_x_mm, pose_y_mm, heading_millirad)
        if self._world_model.state.alliance != alliance:
            self._world_model.reset(alliance)
            self._world_model.update_robot_pose(pose_x_mm, pose_y_mm, heading_millirad)

        if self._config.use_tactical_planner:
            self._world_model.tick(now_ms)
            tactical_plan = self._tactical_planner.plan(self._world_model.state)
            self._last_tactical_plan = tactical_plan
            return self._emit_tactical_plan(plan_request_payload, tactical_plan)

        # Fallback: static routines
        routine = select_routine(
            requested_profile_id=requested_profile,
            alliance=alliance,
            pose_x_mm=pose_x_mm,
            heading_millirad=heading_millirad,
        )
        return self._emit_legacy_plan(plan_request_payload, routine)

    def _emit_tactical_plan(self, plan_request_payload: bytes, plan: TacticalPlan) -> list[bytes]:
        """Emit plan response + phase commands + waypoint deltas."""
        routine = plan.routine
        plan_id = self._next_plan_id
        phase_count = len(plan.phases)
        checksum = self._plan_checksum(plan_request_payload, routine)

        plan_response_payload = struct.pack(
            ">IHHIHBBHH",
            plan_id,
            0,
            phase_count,
            checksum,
            routine.objective_id & 0xFFFF,
            routine.policy_source & 0xFF,
            0,
            routine.global_confidence_permille & 0xFFFF,
            0,
        )
        packets = [
            bcnp.encode_packet(bcnp.MSG_AUTO_PLAN_RESPONSE, 0, 1, plan_response_payload)
        ]

        for phase_seq, tactical_phase in enumerate(plan.phases, start=1):
            packets.append(
                self._build_phase_command_packet(
                    plan_id=plan_id,
                    phase_seq=phase_seq,
                    routine=routine,
                    phase=tactical_phase.phase_program,
                )
            )
            for wp_idx, wp in enumerate(tactical_phase.waypoints):
                wp_payload = struct.pack(
                    ">IHBBiihH",
                    plan_id & 0xFFFFFFFF,
                    phase_seq & 0xFFFF,
                    wp_idx & 0xFF,
                    len(tactical_phase.waypoints) & 0xFF,
                    wp.x_mm,
                    wp.y_mm,
                    wp.heading_mrad,
                    wp.max_velocity_mm_s & 0xFFFF,
                )
                packets.append(
                    bcnp.encode_packet(bcnp.MSG_AUTO_WAYPOINT_DELTA, 0, 1, wp_payload)
                )

        self._next_plan_id += 1
        return packets

    def _emit_legacy_plan(self, plan_request_payload: bytes, routine: ClassicalRoutine) -> list[bytes]:
        """Emit plan response + phase commands using legacy static."""
        plan_id = self._next_plan_id
        phase_count = len(routine.phases)
        checksum = self._plan_checksum(plan_request_payload, routine)

        plan_response_payload = struct.pack(
            ">IHHIHBBHH",
            plan_id,
            0,
            phase_count,
            checksum,
            routine.objective_id & 0xFFFF,
            routine.policy_source & 0xFF,
            0,
            routine.global_confidence_permille & 0xFFFF,
            0,
        )
        packets = [
            bcnp.encode_packet(bcnp.MSG_AUTO_PLAN_RESPONSE, 0, 1, plan_response_payload)
        ]
        for phase_seq, phase in enumerate(routine.phases, start=1):
            packets.append(
                self._build_phase_command_packet(
                    plan_id=plan_id,
                    phase_seq=phase_seq,
                    routine=routine,
                    phase=phase,
                )
            )
        self._next_plan_id += 1
        return packets

    def _build_phase_command_packet(
        self,
        *,
        plan_id: int,
        phase_seq: int,
        routine: ClassicalRoutine,
        phase: PhaseProgram,
    ) -> bytes:
        confidence = phase.confidence_permille if phase.confidence_permille > 0 else routine.global_confidence_permille
        timeout_ms = phase.timeout_ms & 0xFFFF
        deadline_ms = phase.deadline_ms if phase.deadline_ms > 0 else min(0xFFFF, timeout_ms + 300)
        payload = struct.pack(
            ">IHHHHiiBBBBHHHH",
            plan_id & 0xFFFFFFFF,
            phase_seq & 0xFFFF,
            phase.phase_type & 0xFFFF,
            timeout_ms,
            phase.flags & 0xFFFF,
            phase.arg0,
            phase.arg1,
            phase.intent_type & 0xFF,
            phase.safety_class & 0xFF,
            routine.policy_source & 0xFF,
            phase.fallback_hint & 0xFF,
            routine.objective_id & 0xFFFF,
            phase.path_id & 0xFFFF,
            confidence & 0xFFFF,
            deadline_ms & 0xFFFF,
        )
        packet_flags = bcnp.FLAG_CLEAR_QUEUE if phase_seq == 1 else 0
        return bcnp.encode_packet(
            bcnp.MSG_AUTO_PHASE_COMMAND,
            packet_flags,
            1,
            payload,
        )

    def _plan_checksum(self, plan_request_payload: bytes, routine: ClassicalRoutine) -> int:
        checksum = zlib.crc32(plan_request_payload)
        checksum = zlib.crc32(
            struct.pack(
                ">HHHH",
                routine.profile_id & 0xFFFF,
                routine.objective_id & 0xFFFF,
                routine.policy_source & 0xFFFF,
                routine.global_confidence_permille & 0xFFFF,
            ),
            checksum,
        )
        for index, phase in enumerate(routine.phases, start=1):
            confidence = phase.confidence_permille if phase.confidence_permille > 0 else routine.global_confidence_permille
            deadline_ms = phase.deadline_ms if phase.deadline_ms > 0 else min(0xFFFF, phase.timeout_ms + 300)
            checksum = zlib.crc32(
                struct.pack(
                    ">HHHHiiBBBHHHH",
                    index & 0xFFFF,
                    phase.phase_type & 0xFFFF,
                    phase.timeout_ms & 0xFFFF,
                    phase.flags & 0xFFFF,
                    phase.arg0,
                    phase.arg1,
                    phase.intent_type & 0xFF,
                    phase.safety_class & 0xFF,
                    phase.fallback_hint & 0xFF,
                    routine.objective_id & 0xFFFF,
                    phase.path_id & 0xFFFF,
                    confidence & 0xFFFF,
                    deadline_ms & 0xFFFF,
                ),
                checksum,
            )
        return checksum & 0xFFFFFFFF

    def _fault_close(self, fault_code: str) -> None:
        self.last_fault = fault_code
        self.state = SessionState.CLOSED
