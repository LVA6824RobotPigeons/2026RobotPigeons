"""Fused field state model for tactical autonomous planning."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import IntEnum


# Field consts

FIELD_LENGTH_MM = 16_541
FIELD_WIDTH_MM = 8_211
CENTERLINE_X_MM = FIELD_LENGTH_MM // 2

# Hub locations in blue-origin frame (mm)
HUB_BLUE_X_MM = 4_625
HUB_BLUE_Y_MM = 4_035
HUB_RED_X_MM = FIELD_LENGTH_MM - HUB_BLUE_X_MM
HUB_RED_Y_MM = HUB_BLUE_Y_MM

AUTO_DURATION_MS = 20_000

ALLIANCE_BLUE = 0
ALLIANCE_RED = 1


class ShiftState(IntEnum):
    UNKNOWN = 0
    RED_INACTIVE_SHIFT1 = 1
    BLUE_INACTIVE_SHIFT1 = 2


class ZoneId(IntEnum):
    DEPOT_BLUE = 1
    DEPOT_RED = 2
    NEUTRAL_A = 3
    NEUTRAL_B = 4
    OUTPOST_BLUE = 5
    OUTPOST_RED = 6


# Data structures

@dataclass
class Pose2D:
    """Position and heading in the blue-origin WPILib field frame (millimeters/milliradians)."""
    x_mm: int = 0
    y_mm: int = 0
    heading_mrad: int = 0

    def distance_to(self, other: Pose2D) -> float:
        dx = self.x_mm - other.x_mm
        dy = self.y_mm - other.y_mm
        return math.hypot(dx, dy)


@dataclass
class Velocity2D:
    """Translational and rotational velocity (mm/s, mrad/s)."""
    vx_mm_s: int = 0
    vy_mm_s: int = 0
    omega_mrad_s: int = 0

    @property
    def speed_mm_s(self) -> float:
        return math.hypot(self.vx_mm_s, self.vy_mm_s)


@dataclass
class OpponentTrack:
    """Tracked opponent robot position from vision detection."""
    pose: Pose2D
    vx_mm_s: int = 0
    vy_mm_s: int = 0
    confidence_permille: int = 500  # 0-1000
    last_seen_ms: int = 0

    def is_stale(self, now_ms: int, max_age_ms: int = 2000) -> bool:
        return now_ms - self.last_seen_ms > max_age_ms

    def predicted_pose(self, horizon_ms: int) -> Pose2D:
        """Linear extrapolation of opponent position."""
        dt_s = horizon_ms / 1000.0
        return Pose2D(
            x_mm=int(self.pose.x_mm + self.vx_mm_s * dt_s),
            y_mm=int(self.pose.y_mm + self.vy_mm_s * dt_s),
            heading_mrad=self.pose.heading_mrad,
        )


@dataclass
class FuelZone:
    """Known fuel collection area with estimated density."""
    zone_id: ZoneId
    center: Pose2D
    estimated_fuel_count: int = 8  # Start from known field layout
    risk_score: int = 0  # 0-1000, computed from opponent proximity
    travel_cost_ms: int = 0  # Estimated transit from robot, updated each cycle

    @property
    def value_density(self) -> float:
        """Fuel per unit risk+cost. Higher is better."""
        cost = max(self.travel_cost_ms, 100)  # Avoid div by 0
        risk = max(self.risk_score, 1)
        return (self.estimated_fuel_count * 1000.0) / (cost * risk)


@dataclass
class WorldState:
    """Complete snapshot of game state for tactical planning."""

    # Robot state
    robot_pose: Pose2D = field(default_factory=Pose2D)
    robot_velocity: Velocity2D = field(default_factory=Velocity2D)
    alliance: int = ALLIANCE_BLUE

    # Game state
    shift_data: ShiftState = ShiftState.UNKNOWN
    elapsed_auto_ms: int = 0
    remaining_auto_ms: int = AUTO_DURATION_MS

    # Fuel state
    fuel_held: int = 8  # Preloaded count
    fuel_scored_estimate: int = 0
    fuel_zones: list[FuelZone] = field(default_factory=list)

    # Opponents
    opponents: list[OpponentTrack] = field(default_factory=list)

    # Derived
    hub_pose: Pose2D = field(default_factory=Pose2D)
    hub_active: bool = True
    last_shot_success: bool = False

    @property
    def distance_to_hub_mm(self) -> float:
        return self.robot_pose.distance_to(self.hub_pose)


# World model manager

# Est robot max speed for travel cost estimation (mm/s), tune
_ROBOT_MAX_SPEED_MM_S = 4000  # ~4 m/s

# Opponent repulsive field parameters, tune
_OPPONENT_THREAT_RADIUS_MM = 2000  # Within 2m, opponent contributes to risk
_OPPONENT_MAX_RISK = 500  # Max risk contribution per opponent


def _default_fuel_zones() -> list[FuelZone]:
    """Known fuel zone positions in blue-origin frame."""
    return [
        FuelZone(
            zone_id=ZoneId.DEPOT_BLUE,
            center=Pose2D(x_mm=2_300, y_mm=1_200, heading_mrad=0),
            estimated_fuel_count=8,
        ),
        FuelZone(
            zone_id=ZoneId.DEPOT_RED,
            center=Pose2D(x_mm=FIELD_LENGTH_MM - 2_300, y_mm=1_200, heading_mrad=3142),
            estimated_fuel_count=8,
        ),
        FuelZone(
            zone_id=ZoneId.NEUTRAL_A,
            center=Pose2D(x_mm=CENTERLINE_X_MM, y_mm=2_500, heading_mrad=0),
            estimated_fuel_count=4,
        ),
        FuelZone(
            zone_id=ZoneId.NEUTRAL_B,
            center=Pose2D(x_mm=CENTERLINE_X_MM, y_mm=5_700, heading_mrad=0),
            estimated_fuel_count=4,
        ),
        FuelZone(
            zone_id=ZoneId.OUTPOST_BLUE,
            center=Pose2D(x_mm=1_200, y_mm=6_500, heading_mrad=0),
            estimated_fuel_count=6,
        ),
        FuelZone(
            zone_id=ZoneId.OUTPOST_RED,
            center=Pose2D(x_mm=FIELD_LENGTH_MM - 1_200, y_mm=6_500, heading_mrad=3142),
            estimated_fuel_count=6,
        ),
    ]


class WorldModel:
    """Maintains and updates the fused field state each planning cycle."""

    def __init__(self, alliance: int = ALLIANCE_BLUE) -> None:
        self._alliance = alliance
        self._auto_start_ms: int | None = None
        self._state = WorldState(
            alliance=alliance,
            fuel_zones=_default_fuel_zones(),
        )
        self._update_hub_pose()

    @property
    def state(self) -> WorldState:
        return self._state

    def reset(self, alliance: int = ALLIANCE_BLUE) -> None:
        """Reset state for a new match."""
        self._alliance = alliance
        self._auto_start_ms = None
        self._state = WorldState(
            alliance=alliance,
            fuel_zones=_default_fuel_zones(),
        )
        self._update_hub_pose()

    # Incremental updates

    def update_robot_pose(self, x_mm: int, y_mm: int, heading_mrad: int) -> None:
        self._state.robot_pose = Pose2D(x_mm=x_mm, y_mm=y_mm, heading_mrad=heading_mrad)
        self._update_travel_costs()

    def update_robot_velocity(self, vx_mm_s: int, vy_mm_s: int, omega_mrad_s: int = 0) -> None:
        self._state.robot_velocity = Velocity2D(
            vx_mm_s=vx_mm_s, vy_mm_s=vy_mm_s, omega_mrad_s=omega_mrad_s,
        )

    def update_shift_data(self, shift: ShiftState) -> None:
        self._state.shift_data = shift
        self._update_hub_active()

    def update_fuel_held(self, count: int) -> None:
        self._state.fuel_held = max(0, count)

    def record_fuel_acquired(self) -> None:
        self._state.fuel_held += 1

    def record_shot_result(self, success: bool) -> None:
        self._state.last_shot_success = success
        if success and self._state.fuel_held > 0:
            self._state.fuel_held -= 1
            self._state.fuel_scored_estimate += 1

    def update_opponents(self, tracks: list[OpponentTrack]) -> None:
        self._state.opponents = tracks
        self._update_zone_risks()

    def mark_auto_start(self, now_ms: int) -> None:
        self._auto_start_ms = now_ms

    def tick(self, now_ms: int) -> None:
        """Called each planning cycle to update time-derived fields."""
        if self._auto_start_ms is not None:
            self._state.elapsed_auto_ms = now_ms - self._auto_start_ms
            self._state.remaining_auto_ms = max(0, AUTO_DURATION_MS - self._state.elapsed_auto_ms)
        else:
            self._state.remaining_auto_ms = AUTO_DURATION_MS

        # Prune stale opponent tracks
        self._state.opponents = [
            t for t in self._state.opponents if not t.is_stale(now_ms)
        ]
        self._update_zone_risks()
        self._update_travel_costs()

    # Private helpers

    def _update_hub_pose(self) -> None:
        if self._alliance == ALLIANCE_BLUE:
            self._state.hub_pose = Pose2D(x_mm=HUB_BLUE_X_MM, y_mm=HUB_BLUE_Y_MM)
        else:
            self._state.hub_pose = Pose2D(x_mm=HUB_RED_X_MM, y_mm=HUB_RED_Y_MM)
        self._update_hub_active()

    def _update_hub_active(self) -> None:
        shift = self._state.shift_data
        if shift == ShiftState.UNKNOWN:
            # Default: assume our hub is active during auto
            self._state.hub_active = True
        elif shift == ShiftState.RED_INACTIVE_SHIFT1:
            self._state.hub_active = self._alliance == ALLIANCE_BLUE
        elif shift == ShiftState.BLUE_INACTIVE_SHIFT1:
            self._state.hub_active = self._alliance == ALLIANCE_RED

    def _update_travel_costs(self) -> None:
        """Estimate travel time from robot to each fuel zone."""
        robot = self._state.robot_pose
        for zone in self._state.fuel_zones:
            dist = robot.distance_to(zone.center)
            # Simple estimate: distance / max_speed with 20% overhead for decel/accel
            self._state.fuel_zones[self._state.fuel_zones.index(zone)] = FuelZone(
                zone_id=zone.zone_id,
                center=zone.center,
                estimated_fuel_count=zone.estimated_fuel_count,
                risk_score=zone.risk_score,
                travel_cost_ms=int(dist / _ROBOT_MAX_SPEED_MM_S * 1000 * 1.2),
            )

    def _update_zone_risks(self) -> None:
        """Compute risk score for each zone based on opponent proximity."""
        for i, zone in enumerate(self._state.fuel_zones):
            risk = 0
            for opp in self._state.opponents:
                dist = opp.pose.distance_to(zone.center)
                if dist < _OPPONENT_THREAT_RADIUS_MM:
                    # Linear falloff: full risk at 0 distance, zero at threat radius
                    proximity_factor = 1.0 - (dist / _OPPONENT_THREAT_RADIUS_MM)
                    confidence_factor = opp.confidence_permille / 1000.0
                    risk += int(_OPPONENT_MAX_RISK * proximity_factor * confidence_factor)

            self._state.fuel_zones[i] = FuelZone(
                zone_id=zone.zone_id,
                center=zone.center,
                estimated_fuel_count=zone.estimated_fuel_count,
                risk_score=min(1000, risk),
                travel_cost_ms=zone.travel_cost_ms,
            )
