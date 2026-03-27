from __future__ import annotations

import unittest

from coprocessor.service.tactical_planner import CandidateScore, _score_to_confidence_permille


class TacticalPlannerConfidenceTest(unittest.TestCase):
    def test_confidence_permille_is_clamped_to_wire_range(self) -> None:
        self.assertEqual(1000, _score_to_confidence_permille(CandidateScore(expected_points=20.0)))
        self.assertEqual(0, _score_to_confidence_permille(CandidateScore(expected_points=-1.0)))


if __name__ == "__main__":
    unittest.main()
