"""Velocity-compensated shot parameter solver."""

from __future__ import annotations

import math
from dataclasses import dataclass

from coprocessor.service.world_model import Pose2D, Velocity2D, WorldState


@dataclass(frozen=True)
class ShotSolution:
    """Computed shot parameters."""
    aim_offset_mrad: int       # Heading correction for robot velocity compensation
    shooter_rpm: int           # Velocity-compensated RPM
    hood_position_permille: int  # Hood angle as 0-1000
    fire_window_mrad: int      # Angular tolerance where shot is viable
    confidence_permille: int   # Solution confidence (0-1000)
    distance_to_hub_mm: int    # Current distance to hub

# Format: (distance_mm, rpm, hood_permille)
# Hood position is 0-1000.
_SHOT_TABLE = [
    (1321,  2800,  190),   # 52.0 inches
    (2906,  3275,  400),   # 114.4 inches
    (4204,  3650,  480),   # 165.5 inches
    # fine tune more todo
    (5000,  4000,  550),
    (6000,  4400,  650),
    (7000,  4800,  750),
]

# todo: tune these
# tof_ms ≈ base_tof_ms + distance_factor * distance_mm
_TOF_BASE_MS = 100
_TOF_DISTANCE_FACTOR = 0.08  # ms per mm of distance

# RPM compensation factor for robot velocity
# Higher velocity = more RPM needed to maintain trajectory
_RPM_VELOCITY_FACTOR = 0.15  # RPM boost per mm/s of robot speed

# Fire window narrows with distance
_FIRE_WINDOW_BASE_MRAD = 200
_FIRE_WINDOW_DISTANCE_DECAY = 0.02  # mrad reduction per mm

# Minimum conf for a solution to be considered valid
_MIN_DISTANCE_MM = 1000
_MAX_DISTANCE_MM = 8000
_MAX_SPEED_FOR_SHOT_MM_S = 3500  # Beyond this, don't attempt shot-on-move


def _interpolate_shot_table(distance_mm: float) -> tuple[int, int]:
    """Linearly interpolate RPM and hood position from shot table."""
    if distance_mm <= _SHOT_TABLE[0][0]:
        return _SHOT_TABLE[0][1], _SHOT_TABLE[0][2]
    if distance_mm >= _SHOT_TABLE[-1][0]:
        return _SHOT_TABLE[-1][1], _SHOT_TABLE[-1][2]

    for i in range(len(_SHOT_TABLE) - 1):
        d0, rpm0, hood0 = _SHOT_TABLE[i]
        d1, rpm1, hood1 = _SHOT_TABLE[i + 1]
        if d0 <= distance_mm <= d1:
            t = (distance_mm - d0) / (d1 - d0)
            rpm = int(rpm0 + t * (rpm1 - rpm0))
            hood = int(hood0 + t * (hood1 - hood0))
            return rpm, hood

    return _SHOT_TABLE[-1][1], _SHOT_TABLE[-1][2]


class ShotSolver:
    """Computes velocity-compensated shot parameters for a moving robot."""

    def solve(self, state: WorldState) -> ShotSolution | None:
        """Compute optimal shot parameters given current world state.
        Returns None if shot not viable.
        """
        hub = state.hub_pose
        robot = state.robot_pose
        vel = state.robot_velocity

        dx = hub.x_mm - robot.x_mm
        dy = hub.y_mm - robot.y_mm
        dist_mm = math.hypot(dx, dy)

        # Bail out if out of range
        if dist_mm < _MIN_DISTANCE_MM or dist_mm > _MAX_DISTANCE_MM:
            return None

        # Bail out if moving too fast for shot-on-move
        robot_speed = vel.speed_mm_s
        if robot_speed > _MAX_SPEED_FOR_SHOT_MM_S:
            return None

        base_rpm, base_hood = _interpolate_shot_table(dist_mm)

        # Time of flight est
        tof_ms = _TOF_BASE_MS + _TOF_DISTANCE_FACTOR * dist_mm

        # Compensate for robot velocity during projectile flight
        tof_s = tof_ms / 1000.0
        lead_x_mm = vel.vx_mm_s * tof_s
        lead_y_mm = vel.vy_mm_s * tof_s

        # Corrected aim angle (where to point to hit the hub)
        corrected_dx = dx - lead_x_mm
        corrected_dy = dy - lead_y_mm
        aim_rad = math.atan2(corrected_dy, corrected_dx)

        robot_heading_rad = robot.heading_mrad / 1000.0

        # Aim offset = how much to rotate from current heading
        aim_offset_rad = aim_rad - robot_heading_rad
        # Normalize to [-pi, pi]
        aim_offset_rad = math.atan2(math.sin(aim_offset_rad), math.cos(aim_offset_rad))
        aim_offset_mrad = int(aim_offset_rad * 1000)

        # RPM compensation for robot vel
        rpm_boost = int(robot_speed * _RPM_VELOCITY_FACTOR)
        compensated_rpm = base_rpm + rpm_boost

        # Fire window (narrows with distance and robot speed)
        fire_window = max(
            50,  # Minimum 50 mrad window
            int(_FIRE_WINDOW_BASE_MRAD - _FIRE_WINDOW_DISTANCE_DECAY * dist_mm - robot_speed * 0.02),
        )

        # Confidence based on distance and speed
        dist_factor = 1.0 - abs(dist_mm - 3000) / 5000  # Peak confidence at ~3m
        speed_factor = 1.0 - (robot_speed / _MAX_SPEED_FOR_SHOT_MM_S)
        hub_factor = 1.0 if state.hub_active else 0.3
        confidence = int(max(0, min(1000, dist_factor * speed_factor * hub_factor * 1000)))

        return ShotSolution(
            aim_offset_mrad=aim_offset_mrad,
            shooter_rpm=compensated_rpm,
            hood_position_permille=base_hood,
            fire_window_mrad=fire_window,
            confidence_permille=confidence,
            distance_to_hub_mm=int(dist_mm),
        )
