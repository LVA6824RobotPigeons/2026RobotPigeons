"""Tactical auto planner with candidate-scored decision engine.

Evaluates candidate action sequences (collect fuel, score, exit)
and selects the best based on weighted scoring that considers expected 
points, time budget, opponent risk, shift alignment, and safety.

Generates waypoints for the selected plan that the robot executes via
WaypointFollowerCommand.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum

from coprocessor.service.world_model import (
    ALLIANCE_BLUE,
    ALLIANCE_RED,
    CENTERLINE_X_MM,
    FIELD_LENGTH_MM,
    FIELD_WIDTH_MM,
    FuelZone,
    Pose2D,
    WorldState,
    ZoneId,
)
from coprocessor.service.classical_routines import (
    FALLBACK_SAFE_EXIT,
    INTENT_COLLECT,
    INTENT_DRIVE,
    INTENT_SAFE_EXIT,
    INTENT_SCORE,
    PHASE_AIM_AND_SHOOT,
    PHASE_COLLECT_DEPOT_FUEL,
    PHASE_SAFE_EXIT,
    PHASE_SEED_AND_DEPART,
    PHASE_TRANSIT_AND_PRESPIN,
    POLICY_SOURCE_CLASSICAL,
    SAFETY_HIGH_RISK,
    SAFETY_SCORING,
    SAFETY_STANDARD,
    ClassicalRoutine,
    PhaseProgram,
)


# tune

WEIGHT_EXPECTED_POINTS = 5.0
WEIGHT_TIME_EFFICIENCY = 2.0
WEIGHT_SAFETY_MARGIN = 3.0
WEIGHT_OPPONENT_RISK = 4.0
WEIGHT_SHIFT_ALIGNMENT = 2.5


def _score_to_confidence_permille(score: "CandidateScore") -> int:
    """Map the weighted tactical score into a bounded 0-1000 confidence value."""
    return max(0, min(1000, int(score.total * 100)))


class CandidateType(IntEnum):
    COLLECT_AND_SCORE = 1
    COLLECT_AND_SCORE_ON_MOVE = 2
    SCORE_IMMEDIATE = 3
    SAFE_EXIT = 4


@dataclass
class CandidateScore:
    expected_points: float = 0.0
    time_efficiency: float = 0.0
    safety_margin: float = 0.0
    opponent_risk: float = 0.0
    shift_alignment: float = 0.0

    @property
    def total(self) -> float:
        return (
            self.expected_points * WEIGHT_EXPECTED_POINTS
            + self.time_efficiency * WEIGHT_TIME_EFFICIENCY
            + self.safety_margin * WEIGHT_SAFETY_MARGIN
            + self.opponent_risk * WEIGHT_OPPONENT_RISK
            + self.shift_alignment * WEIGHT_SHIFT_ALIGNMENT
        )


@dataclass
class Candidate:
    candidate_type: CandidateType
    target_zone: FuelZone | None = None
    description: str = ""


@dataclass
class Waypoint:
    """Single waypoint for the robot to follow."""
    x_mm: int = 0
    y_mm: int = 0
    heading_mrad: int = 0
    max_velocity_mm_s: int = 4000  # tune to def max rob speed


@dataclass
class TacticalPhase:
    """One phase of the tactical plan with waypoints."""
    phase_program: PhaseProgram
    waypoints: list[Waypoint] = field(default_factory=list)


@dataclass
class TacticalPlan:
    """Complete plan selected by the tactical planner."""
    candidate_type: CandidateType
    score: CandidateScore
    routine: ClassicalRoutine
    phases: list[TacticalPhase] = field(default_factory=list)
    description: str = ""


# Opponent avoidance helpers, tune

_OPPONENT_AVOIDANCE_RADIUS_MM = 1500  # How far to deviate from opponents
_WAYPOINT_SPACING_MM = 1500  # Max distance between consecutive waypoints


def _heading_to_target(from_pose: Pose2D, to_pose: Pose2D) -> int:
    """Compute heading in milliradians from one pose to another."""
    dx = to_pose.x_mm - from_pose.x_mm
    dy = to_pose.y_mm - from_pose.y_mm
    return int(math.atan2(dy, dx) * 1000)


def _interpolate_waypoints(start: Pose2D, end: Pose2D, max_spacing_mm: int = _WAYPOINT_SPACING_MM) -> list[Waypoint]:
    """Generate intermediate waypoints between start and end with heading interpolation."""
    dist = start.distance_to(end)
    if dist < max_spacing_mm:
        heading = _heading_to_target(start, end)
        return [Waypoint(x_mm=end.x_mm, y_mm=end.y_mm, heading_mrad=heading)]

    num_segments = max(2, int(dist / max_spacing_mm))
    waypoints = []
    for i in range(1, num_segments + 1):
        t = i / num_segments
        x = int(start.x_mm + (end.x_mm - start.x_mm) * t)
        y = int(start.y_mm + (end.y_mm - start.y_mm) * t)
        heading = _heading_to_target(
            Pose2D(x_mm=x, y_mm=y),
            end if i < num_segments else end,
        )
        waypoints.append(Waypoint(x_mm=x, y_mm=y, heading_mrad=heading))

    # Final waypoint uses the end heading
    if waypoints:
        waypoints[-1].heading_mrad = end.heading_mrad if end.heading_mrad != 0 else waypoints[-1].heading_mrad

    return waypoints


def _apply_opponent_avoidance(
    waypoints: list[Waypoint],
    opponents: list,
    avoidance_radius_mm: int = _OPPONENT_AVOIDANCE_RADIUS_MM,
) -> list[Waypoint]:
    """Offset waypoints away from nearby opponents using a repulsive field. (orb)"""
    if not opponents:
        return waypoints

    adjusted = []
    for wp in waypoints:
        offset_x = 0
        offset_y = 0
        for opp in opponents:
            dx = wp.x_mm - opp.pose.x_mm
            dy = wp.y_mm - opp.pose.y_mm
            dist = math.hypot(dx, dy)
            if dist < avoidance_radius_mm and dist > 0:
                # Repulsive force: push waypoint away from opponent
                force = (avoidance_radius_mm - dist) / avoidance_radius_mm
                confidence_factor = opp.confidence_permille / 1000.0
                push = force * confidence_factor * avoidance_radius_mm * 0.5
                offset_x += int(push * dx / dist)
                offset_y += int(push * dy / dist)

        # Clamp to field boundaries
        new_x = max(500, min(FIELD_LENGTH_MM - 500, wp.x_mm + offset_x))
        new_y = max(500, min(FIELD_WIDTH_MM - 500, wp.y_mm + offset_y))
        adjusted.append(Waypoint(
            x_mm=new_x, y_mm=new_y,
            heading_mrad=wp.heading_mrad,
            max_velocity_mm_s=wp.max_velocity_mm_s,
        ))

    return adjusted


# Tactical planner

# Min time budget to attempt actions (ms), tune
_MIN_TIME_FOR_COLLECT_AND_SCORE_MS = 8_000
_MIN_TIME_FOR_SCORE_IMMEDIATE_MS = 4_000
_MIN_TIME_FOR_SAFE_EXIT_MS = 1_500

# Est dur for action segments (ms), tune
_COLLECT_DURATION_MS = 2_000
_SCORE_DURATION_MS = 3_000
_SAFE_EXIT_DURATION_MS = 1_000

# Points per fuel (approximate, depends on hub active state)
_POINTS_PER_FUEL_ACTIVE = 1
_POINTS_PER_FUEL_INACTIVE = 0

# Centerline safety margin, tune
_CENTERLINE_DANGER_MM = 800
_ALLIANCE_ZONE_MARGIN_MM = 500


class TacticalPlanner:
    """Evaluates candidate action plans and selects the best one."""

    def __init__(self) -> None:
        self._last_plan: TacticalPlan | None = None
        self._plan_count = 0

    @property
    def last_plan(self) -> TacticalPlan | None:
        return self._last_plan

    def plan(self, state: WorldState) -> TacticalPlan:
        """Evaluate all viable candidates and select the highest-scoring plan."""
        candidates = self._generate_candidates(state)
        scored = [(c, self._score(c, state)) for c in candidates]
        best_candidate, best_score = max(scored, key=lambda x: x[1].total)
        plan = self._build_plan(best_candidate, best_score, state)
        self._last_plan = plan
        self._plan_count += 1
        return plan

    # Candidate generation 

    def _generate_candidates(self, state: WorldState) -> list[Candidate]:
        candidates: list[Candidate] = []

        # For each reachable fuel zone with fuel, consider collect→score
        for zone in state.fuel_zones:
            if zone.estimated_fuel_count <= 0:
                continue

            # Filter zones by alliance ownership/accessibility
            if not self._is_zone_accessible(zone, state):
                continue

            estimated_total_ms = zone.travel_cost_ms + _COLLECT_DURATION_MS + _SCORE_DURATION_MS
            if estimated_total_ms < state.remaining_auto_ms and state.remaining_auto_ms >= _MIN_TIME_FOR_COLLECT_AND_SCORE_MS:
                candidates.append(Candidate(
                    candidate_type=CandidateType.COLLECT_AND_SCORE,
                    target_zone=zone,
                    description=f"Collect from {zone.zone_id.name} ({zone.estimated_fuel_count} fuel), then score",
                ))

                # Also consider shot-on-move variant if we'll pass near hub
                if self._shot_on_move_viable(state, zone):
                    candidates.append(Candidate(
                        candidate_type=CandidateType.COLLECT_AND_SCORE_ON_MOVE,
                        target_zone=zone,
                        description=f"Collect from {zone.zone_id.name}, score on the move",
                    ))

        # Score preloaded/held fuel immediately (skip collection)
        if state.fuel_held > 0 and state.remaining_auto_ms >= _MIN_TIME_FOR_SCORE_IMMEDIATE_MS:
            candidates.append(Candidate(
                candidate_type=CandidateType.SCORE_IMMEDIATE,
                description=f"Score {state.fuel_held} held fuel immediately",
            ))

        # Safe exit (always available as a fallback)
        candidates.append(Candidate(
            candidate_type=CandidateType.SAFE_EXIT,
            description="Safe exit to alliance zone",
        ))

        return candidates

    # Scoring

    def _score(self, candidate: Candidate, state: WorldState) -> CandidateScore:
        return CandidateScore(
            expected_points=self._expected_points(candidate, state),
            time_efficiency=self._time_efficiency(candidate, state),
            safety_margin=self._safety_margin(candidate, state),
            opponent_risk=self._opponent_risk(candidate, state),
            shift_alignment=self._shift_alignment(candidate, state),
        )

    def _expected_points(self, candidate: Candidate, state: WorldState) -> float:
        points_per_fuel = _POINTS_PER_FUEL_ACTIVE if state.hub_active else _POINTS_PER_FUEL_INACTIVE

        if candidate.candidate_type == CandidateType.SCORE_IMMEDIATE:
            return state.fuel_held * points_per_fuel * 1.0

        if candidate.candidate_type in (CandidateType.COLLECT_AND_SCORE, CandidateType.COLLECT_AND_SCORE_ON_MOVE):
            zone = candidate.target_zone
            if zone is None:
                return 0.0
            # Estimate, TUNE TUNE TUNE TUNE TUNE TUNE TUNE 
            collectible = min(zone.estimated_fuel_count, 4)
            total_fuel = state.fuel_held + collectible
            shot_success_rate = 0.7 if candidate.candidate_type == CandidateType.COLLECT_AND_SCORE_ON_MOVE else 0.85
            return total_fuel * points_per_fuel * shot_success_rate

        return 0.0  # SAFE_EXIT

    def _time_efficiency(self, candidate: Candidate, state: WorldState) -> float:
        """Ratio of time budget vs estimated cost. Higher = more time margin."""
        if state.remaining_auto_ms <= 0:
            return 0.0

        if candidate.candidate_type in (CandidateType.COLLECT_AND_SCORE, CandidateType.COLLECT_AND_SCORE_ON_MOVE):
            zone = candidate.target_zone
            if zone is None:
                return 0.0
            cost = zone.travel_cost_ms + _COLLECT_DURATION_MS + _SCORE_DURATION_MS
            return min(1.0, state.remaining_auto_ms / max(cost, 1))

        if candidate.candidate_type == CandidateType.SCORE_IMMEDIATE:
            cost = _SCORE_DURATION_MS
            return min(1.0, state.remaining_auto_ms / max(cost, 1))

        if candidate.candidate_type == CandidateType.SCORE_IMMEDIATE:
            cost = _SCORE_DURATION_MS
            return min(1.0, state.remaining_auto_ms / max(cost, 1))

        return 1.0  # SAFE_EXIT is always time-efficient

    def _safety_margin(self, candidate: Candidate, state: WorldState) -> float:
        """Penalize plans that approach center line or wrong alliance zone."""
        score = 1.0

        if candidate.candidate_type in (CandidateType.COLLECT_AND_SCORE, CandidateType.COLLECT_AND_SCORE_ON_MOVE):
            zone = candidate.target_zone
            if zone is None:
                return 0.5

            # Penalize neutral zones (near center line)
            zone_x = zone.center.x_mm
            distance_from_centerline = abs(zone_x - CENTERLINE_X_MM)
            if distance_from_centerline < _CENTERLINE_DANGER_MM:
                score *= 0.5

            # Heavy penalty for opponent alliance zones
            if state.alliance == ALLIANCE_BLUE and zone_x > CENTERLINE_X_MM + _ALLIANCE_ZONE_MARGIN_MM:
                score *= 0.2
            elif state.alliance == ALLIANCE_RED and zone_x < CENTERLINE_X_MM - _ALLIANCE_ZONE_MARGIN_MM:
                score *= 0.2

        return score

    def _opponent_risk(self, candidate: Candidate, state: WorldState) -> float:
        """Penalize plans where opponents are near the target zone. Returns 0-1 (1 = no risk)."""
        if not state.opponents:
            return 1.0

        if candidate.candidate_type in (CandidateType.COLLECT_AND_SCORE, CandidateType.COLLECT_AND_SCORE_ON_MOVE):
            zone = candidate.target_zone
            if zone is None:
                return 1.0
            # Risk based on zone's computed risk score
            return 1.0 - (zone.risk_score / 1000.0)

        if candidate.candidate_type == CandidateType.SCORE_IMMEDIATE:
            # Check if opponents are near the hub
            hub = state.hub_pose
            risk = 0.0
            for opp in state.opponents:
                dist = opp.pose.distance_to(hub)
                if dist < 2000:
                    risk += (1.0 - dist / 2000.0) * (opp.confidence_permille / 1000.0)
            return max(0.0, 1.0 - risk)

        return 1.0  # SAFE_EXIT doesn't care about opponents much

    def _shift_alignment(self, candidate: Candidate, state: WorldState) -> float:
        """Bonus for scoring when hub is active, penalty when inactive."""
        if candidate.candidate_type in (
            CandidateType.SCORE_IMMEDIATE,
            CandidateType.COLLECT_AND_SCORE,
            CandidateType.COLLECT_AND_SCORE_ON_MOVE,
        ):
            return 1.0 if state.hub_active else 0.2

        return 0.5  # Neutral for non-scoring actions

    # Plan building

    def _build_plan(self, candidate: Candidate, score: CandidateScore, state: WorldState) -> TacticalPlan:
        """Convert the best candidate into a concrete TacticalPlan with phases and waypoints."""
        if candidate.candidate_type == CandidateType.COLLECT_AND_SCORE:
            return self._build_collect_and_score(candidate, score, state, shot_on_move=False)
        if candidate.candidate_type == CandidateType.COLLECT_AND_SCORE_ON_MOVE:
            return self._build_collect_and_score(candidate, score, state, shot_on_move=True)
        if candidate.candidate_type == CandidateType.SCORE_IMMEDIATE:
            return self._build_score_immediate(candidate, score, state)
        return self._build_safe_exit(candidate, score, state)

    def _build_collect_and_score(
        self, candidate: Candidate, score: CandidateScore, state: WorldState, *, shot_on_move: bool,
    ) -> TacticalPlan:
        zone = candidate.target_zone
        assert zone is not None

        hub = state.hub_pose
        robot = state.robot_pose

        # Phase 1: Drive to fuel zone
        transit_to_zone_wps = _interpolate_waypoints(robot, zone.center)
        transit_to_zone_wps = _apply_opponent_avoidance(transit_to_zone_wps, state.opponents)

        # Phase 2: Collect fuel (stay at zone)
        collect_wps = [Waypoint(
            x_mm=zone.center.x_mm, y_mm=zone.center.y_mm,
            heading_mrad=zone.center.heading_mrad,
            max_velocity_mm_s=500,  # Slow for collection
        )]

        # Phase 3: Transit to hub (with optional shot-on-move)
        hub_approach = self._compute_hub_approach_pose(state)
        transit_to_hub_wps = _interpolate_waypoints(zone.center, hub_approach)
        transit_to_hub_wps = _apply_opponent_avoidance(transit_to_hub_wps, state.opponents)

        # Set heading toward hub during transit for pre-aiming
        for wp in transit_to_hub_wps:
            wp.heading_mrad = _heading_to_target(Pose2D(x_mm=wp.x_mm, y_mm=wp.y_mm), hub)

        phases: list[TacticalPhase] = []

        # P1: Drive to zone
        phases.append(TacticalPhase(
            phase_program=PhaseProgram(
                PHASE_SEED_AND_DEPART, 5000, INTENT_DRIVE, SAFETY_STANDARD,
                confidence_permille=_score_to_confidence_permille(score),
            ),
            waypoints=transit_to_zone_wps,
        ))

        # P2: Collect
        phases.append(TacticalPhase(
            phase_program=PhaseProgram(
                PHASE_COLLECT_DEPOT_FUEL, 3000, INTENT_COLLECT, SAFETY_STANDARD,
                confidence_permille=_score_to_confidence_permille(score),
            ),
            waypoints=collect_wps,
        ))

        # P3: Transit + prespin
        transit_intent = INTENT_SCORE if shot_on_move else INTENT_DRIVE
        phases.append(TacticalPhase(
            phase_program=PhaseProgram(
                PHASE_TRANSIT_AND_PRESPIN, 4000, transit_intent, SAFETY_STANDARD,
                confidence_permille=_score_to_confidence_permille(score),
            ),
            waypoints=transit_to_hub_wps,
        ))

        if not shot_on_move:
            # P4: Stationary aim and shoot
            phases.append(TacticalPhase(
                phase_program=PhaseProgram(
                    PHASE_AIM_AND_SHOOT, 4000, INTENT_SCORE, SAFETY_SCORING,
                    confidence_permille=_score_to_confidence_permille(score),
                ),
                waypoints=[Waypoint(
                    x_mm=hub_approach.x_mm, y_mm=hub_approach.y_mm,
                    heading_mrad=_heading_to_target(hub_approach, hub),
                    max_velocity_mm_s=0,
                )],
            ))

        # P_last: Safe exit
        phases.append(TacticalPhase(
            phase_program=PhaseProgram(
                PHASE_SAFE_EXIT, 1500, INTENT_SAFE_EXIT, SAFETY_STANDARD,
                confidence_permille=_score_to_confidence_permille(score),
            ),
            waypoints=self._safe_exit_waypoints(state),
        ))

        routine = ClassicalRoutine(
            profile_id=100 + self._plan_count,
            objective_id=300 + candidate.candidate_type,
            global_confidence_permille=_score_to_confidence_permille(score),
            phases=tuple(p.phase_program for p in phases),
            policy_source=POLICY_SOURCE_CLASSICAL,
        )

        return TacticalPlan(
            candidate_type=candidate.candidate_type,
            score=score,
            routine=routine,
            phases=phases,
            description=candidate.description,
        )

    def _build_score_immediate(self, candidate: Candidate, score: CandidateScore, state: WorldState) -> TacticalPlan:
        hub = state.hub_pose
        hub_approach = self._compute_hub_approach_pose(state)

        transit_wps = _interpolate_waypoints(state.robot_pose, hub_approach)
        transit_wps = _apply_opponent_avoidance(transit_wps, state.opponents)
        for wp in transit_wps:
            wp.heading_mrad = _heading_to_target(Pose2D(x_mm=wp.x_mm, y_mm=wp.y_mm), hub)

        phases = [
            TacticalPhase(
                phase_program=PhaseProgram(
                    PHASE_TRANSIT_AND_PRESPIN, 4000, INTENT_DRIVE, SAFETY_STANDARD,
                    confidence_permille=_score_to_confidence_permille(score),
                ),
                waypoints=transit_wps,
            ),
            TacticalPhase(
                phase_program=PhaseProgram(
                    PHASE_AIM_AND_SHOOT, 4000, INTENT_SCORE, SAFETY_SCORING,
                    confidence_permille=_score_to_confidence_permille(score),
                ),
                waypoints=[Waypoint(
                    x_mm=hub_approach.x_mm, y_mm=hub_approach.y_mm,
                    heading_mrad=_heading_to_target(hub_approach, hub),
                    max_velocity_mm_s=0,
                )],
            ),
            TacticalPhase(
                phase_program=PhaseProgram(
                    PHASE_SAFE_EXIT, 1500, INTENT_SAFE_EXIT, SAFETY_STANDARD,
                    confidence_permille=_score_to_confidence_permille(score),
                ),
                waypoints=self._safe_exit_waypoints(state),
            ),
        ]

        routine = ClassicalRoutine(
            profile_id=100 + self._plan_count,
            objective_id=310,
            global_confidence_permille=_score_to_confidence_permille(score),
            phases=tuple(p.phase_program for p in phases),
        )

        return TacticalPlan(
            candidate_type=candidate.candidate_type,
            score=score,
            routine=routine,
            phases=phases,
            description=candidate.description,
        )

    def _build_safe_exit(self, candidate: Candidate, score: CandidateScore, state: WorldState) -> TacticalPlan:
        phases = [
            TacticalPhase(
                phase_program=PhaseProgram(
                    PHASE_SAFE_EXIT, 3000, INTENT_SAFE_EXIT, SAFETY_STANDARD,
                    confidence_permille=900,
                ),
                waypoints=self._safe_exit_waypoints(state),
            ),
        ]

        routine = ClassicalRoutine(
            profile_id=100 + self._plan_count,
            objective_id=330,
            global_confidence_permille=900,
            phases=tuple(p.phase_program for p in phases),
        )

        return TacticalPlan(
            candidate_type=candidate.candidate_type,
            score=score,
            routine=routine,
            phases=phases,
            description=candidate.description,
        )

    # Helpers

    def _is_zone_accessible(self, zone: FuelZone, state: WorldState) -> bool:
        """Check if a fuel zone is legally accessible for this alliance."""
        # Neutral zones are always accessible
        if zone.zone_id in (ZoneId.NEUTRAL_A, ZoneId.NEUTRAL_B):
            return True
        # Alliance-specific zones
        if state.alliance == ALLIANCE_BLUE:
            return zone.zone_id in (ZoneId.DEPOT_BLUE, ZoneId.OUTPOST_BLUE)
        return zone.zone_id in (ZoneId.DEPOT_RED, ZoneId.OUTPOST_RED)

    def _shot_on_move_viable(self, state: WorldState, zone: FuelZone) -> bool:
        """Check if the transit from zone to hub passes close enough for shot-on-move."""
        hub = state.hub_pose

        dist_zone_to_hub = zone.center.distance_to(hub)
        # Viable if hub is within 5m of the zone (not too far for accuracy)
        return dist_zone_to_hub < 5000

    def _compute_hub_approach_pose(self, state: WorldState) -> Pose2D:
        """Compute ideal approach position for scoring (offset from hub)."""
        hub = state.hub_pose
        robot = state.robot_pose

        # Approach from current side at ~2.5m distance
        dx = robot.x_mm - hub.x_mm
        dy = robot.y_mm - hub.y_mm
        dist = math.hypot(dx, dy)
        if dist < 100:
            # Edge case: robot on top of hub, approach from behind
            approach_x = hub.x_mm - (2500 if state.alliance == ALLIANCE_BLUE else -2500)
            approach_y = hub.y_mm
        else:
            # Approach from the robot's current direction at 2.5m radius
            scale = 2500.0 / dist
            approach_x = int(hub.x_mm + dx * scale)
            approach_y = int(hub.y_mm + dy * scale)

        # Clamp to field and alliance zone
        approach_x = max(500, min(FIELD_LENGTH_MM - 500, approach_x))
        approach_y = max(500, min(FIELD_WIDTH_MM - 500, approach_y))

        return Pose2D(
            x_mm=approach_x,
            y_mm=approach_y,
            heading_mrad=_heading_to_target(Pose2D(x_mm=approach_x, y_mm=approach_y), hub),
        )

    def _safe_exit_waypoints(self, state: WorldState) -> list[Waypoint]:
        """Generate waypoints to nearest safe alliance zone."""
        if state.alliance == ALLIANCE_BLUE:
            safe = Pose2D(x_mm=2000, y_mm=4100, heading_mrad=0)
        else:
            safe = Pose2D(x_mm=FIELD_LENGTH_MM - 2000, y_mm=4100, heading_mrad=3142)

        wps = _interpolate_waypoints(state.robot_pose, safe)
        return _apply_opponent_avoidance(wps, state.opponents)
