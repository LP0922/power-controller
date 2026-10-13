import unittest

from powder_sampling_control.dispensing_algorithm.calibration import (
    ContinuousTaperPlanner,
    ContinuousTaperStage,
)


class ContinuousTaperPlannerTests(unittest.TestCase):
    def setUp(self):
        self.planner = ContinuousTaperPlanner()

    def test_keeps_fixed_coarse_action_below_420_mg(self):
        decision = self.planner.decide(
            raw_mass_mg=300.0,
            filtered_mass_mg=302.0,
            rate_mg_s=20.0,
            current_window_position_units=200,
            current_frequency_hz=80,
            current_duty_permyriad=2000,
        )

        self.assertEqual(decision.stage, ContinuousTaperStage.COARSE)
        self.assertEqual(decision.command, "KEEP")
        self.assertEqual(decision.action.window_position_units, 200)
        self.assertEqual(decision.action.frequency_hz, 80)
        self.assertEqual(decision.action.duty_permyriad, 2000)

    def test_switches_once_to_fixed_taper_action_after_420_mg(self):
        decision = self.planner.decide(
            raw_mass_mg=421.0,
            filtered_mass_mg=420.0,
            rate_mg_s=10.0,
            current_window_position_units=200,
            current_frequency_hz=80,
            current_duty_permyriad=2000,
        )

        self.assertEqual(decision.stage, ContinuousTaperStage.TAPER)
        self.assertEqual(decision.command, "UPDATE")
        self.assertEqual(decision.action.window_position_units, 100)
        self.assertEqual(decision.action.frequency_hz, 65)
        self.assertEqual(decision.action.duty_permyriad, 1600)

    def test_predictive_stop_requires_at_least_485_mg_raw_mass(self):
        before_gate = self.planner.decide(
            raw_mass_mg=484.0,
            filtered_mass_mg=490.0,
            rate_mg_s=20.0,
        )
        at_gate = self.planner.decide(
            raw_mass_mg=485.0,
            filtered_mass_mg=490.0,
            rate_mg_s=20.0,
        )

        self.assertNotEqual(before_gate.command, "STOP")
        self.assertEqual(at_gate.command, "STOP")
        self.assertEqual(at_gate.stage, ContinuousTaperStage.SETTLE)

    def test_hard_overweight_enters_review_and_stops(self):
        decision = self.planner.decide(raw_mass_mg=510.0)

        self.assertEqual(decision.command, "STOP")
        self.assertEqual(decision.stage, ContinuousTaperStage.REVIEW)


if __name__ == "__main__":
    unittest.main()
