"""Classical autonomous routine catalog and selector for off-device planning."""

from __future__ import annotations

from dataclasses import dataclass


FIELD_LENGTH_MM = 16_541
CENTERLINE_X_MM = FIELD_LENGTH_MM // 2
CENTERLINE_MARGIN_MM = 450
FAST_START_BAND_MM = 4_500
HEADING_RISK_THRESHOLD_MRAD = 2_500

ALLIANCE_BLUE = 0
ALLIANCE_RED = 1

POLICY_SOURCE_CLASSICAL = 1

PHASE_SEED_AND_DEPART = 1
PHASE_COLLECT_DEPOT_FUEL = 2
PHASE_TRANSIT_AND_PRESPIN = 3
PHASE_AIM_AND_SHOOT = 4
PHASE_SAFE_EXIT = 6
PHASE_ADVANCE_DEPOT_NO_COLLECT = 7

INTENT_DRIVE = 1
INTENT_COLLECT = 2
INTENT_SCORE = 3
INTENT_SAFE_EXIT = 5

SAFETY_STANDARD = 1
SAFETY_SCORING = 2
SAFETY_HIGH_RISK = 3

FALLBACK_SAFE_EXIT = 1

PATH_OUTPOST_SEGMENT = 1
PATH_DEPOT_SEGMENT = 2
PATH_SHOOTING_SEGMENT = 3


@dataclass(frozen=True)
class PhaseProgram:
    phase_type: int
    timeout_ms: int
    intent_type: int
    safety_class: int
    path_id: int = 0
    arg0: int = 0
    arg1: int = 0
    fallback_hint: int = FALLBACK_SAFE_EXIT
    confidence_permille: int = 0
    deadline_ms: int = 0
    flags: int = 0


@dataclass(frozen=True)
class ClassicalRoutine:
    profile_id: int
    objective_id: int
    global_confidence_permille: int
    phases: tuple[PhaseProgram, ...]
    policy_source: int = POLICY_SOURCE_CLASSICAL


def _make_catalog() -> dict[int, ClassicalRoutine]:
    max_fuel = (
        PhaseProgram(PHASE_SEED_AND_DEPART, 4500, INTENT_DRIVE, SAFETY_STANDARD, path_id=PATH_OUTPOST_SEGMENT, confidence_permille=760),
        PhaseProgram(PHASE_COLLECT_DEPOT_FUEL, 4000, INTENT_COLLECT, SAFETY_STANDARD, path_id=PATH_DEPOT_SEGMENT, confidence_permille=740),
        PhaseProgram(PHASE_TRANSIT_AND_PRESPIN, 4000, INTENT_DRIVE, SAFETY_STANDARD, path_id=PATH_SHOOTING_SEGMENT, confidence_permille=730),
        PhaseProgram(PHASE_AIM_AND_SHOOT, 4000, INTENT_SCORE, SAFETY_SCORING, confidence_permille=720),
    )
    fast_cycle = (
        PhaseProgram(PHASE_SEED_AND_DEPART, 4500, INTENT_DRIVE, SAFETY_STANDARD, path_id=PATH_OUTPOST_SEGMENT, confidence_permille=790),
        PhaseProgram(PHASE_COLLECT_DEPOT_FUEL, 4000, INTENT_COLLECT, SAFETY_STANDARD, path_id=PATH_DEPOT_SEGMENT, confidence_permille=780),
        PhaseProgram(PHASE_TRANSIT_AND_PRESPIN, 4000, INTENT_DRIVE, SAFETY_STANDARD, path_id=PATH_SHOOTING_SEGMENT, confidence_permille=760),
        PhaseProgram(PHASE_AIM_AND_SHOOT, 4000, INTENT_SCORE, SAFETY_SCORING, confidence_permille=750),
        PhaseProgram(PHASE_SAFE_EXIT, 1200, INTENT_SAFE_EXIT, SAFETY_STANDARD, confidence_permille=820),
    )
    safe_score_exit = (
        PhaseProgram(PHASE_SEED_AND_DEPART, 4500, INTENT_DRIVE, SAFETY_STANDARD, path_id=PATH_OUTPOST_SEGMENT, confidence_permille=840),
        PhaseProgram(PHASE_ADVANCE_DEPOT_NO_COLLECT, 3000, INTENT_DRIVE, SAFETY_STANDARD, path_id=PATH_DEPOT_SEGMENT, confidence_permille=840),
        PhaseProgram(PHASE_TRANSIT_AND_PRESPIN, 4000, INTENT_DRIVE, SAFETY_STANDARD, path_id=PATH_SHOOTING_SEGMENT, confidence_permille=820),
        PhaseProgram(PHASE_AIM_AND_SHOOT, 4000, INTENT_SCORE, SAFETY_SCORING, confidence_permille=800),
        PhaseProgram(PHASE_SAFE_EXIT, 1200, INTENT_SAFE_EXIT, SAFETY_STANDARD, confidence_permille=850),
    )
    conservative = (
        PhaseProgram(PHASE_SEED_AND_DEPART, 4500, INTENT_DRIVE, SAFETY_STANDARD, path_id=PATH_OUTPOST_SEGMENT, confidence_permille=900),
        PhaseProgram(PHASE_SAFE_EXIT, 1200, INTENT_SAFE_EXIT, SAFETY_STANDARD, confidence_permille=900),
    )
    return {
        1: ClassicalRoutine(profile_id=1, objective_id=101, global_confidence_permille=760, phases=max_fuel),
        2: ClassicalRoutine(profile_id=2, objective_id=102, global_confidence_permille=790, phases=fast_cycle),
        3: ClassicalRoutine(profile_id=3, objective_id=103, global_confidence_permille=840, phases=safe_score_exit),
        11: ClassicalRoutine(profile_id=11, objective_id=201, global_confidence_permille=840, phases=safe_score_exit),
        12: ClassicalRoutine(profile_id=12, objective_id=202, global_confidence_permille=790, phases=fast_cycle),
        13: ClassicalRoutine(profile_id=13, objective_id=203, global_confidence_permille=900, phases=conservative),
    }


CATALOG = _make_catalog()


def select_routine(
    requested_profile_id: int,
    alliance: int,
    pose_x_mm: int,
    heading_millirad: int,
) -> ClassicalRoutine:
    selected = CATALOG.get(requested_profile_id)
    if selected is not None:
        return selected

    near_centerline = abs(pose_x_mm - CENTERLINE_X_MM) <= CENTERLINE_MARGIN_MM
    unstable_heading = abs(heading_millirad) >= HEADING_RISK_THRESHOLD_MRAD
    if near_centerline or unstable_heading:
        return CATALOG[13]

    if alliance == ALLIANCE_RED:
        if pose_x_mm >= FIELD_LENGTH_MM - FAST_START_BAND_MM:
            return CATALOG[12]
        return CATALOG[11]

    if pose_x_mm <= FAST_START_BAND_MM:
        return CATALOG[12]
    return CATALOG[11]
