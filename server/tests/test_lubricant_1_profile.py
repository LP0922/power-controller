"""Offline checks for the lubricant-1 candidate; never open hardware ports."""

import json
import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from device_control_server import (
    DeviceControlService,
    resolve_powder_identity,
    resolve_tail_pulse_config,
)
from powder_sampling_control.dispensing_algorithm.core import (
    load_control_profile,
    resolve_default_target_rate_band,
)
from powder_sampling_control.dispensing_algorithm.core import interpolate_profile
from powder_sampling_control.dispensing_algorithm.core import (
    ContinuousFeedbackController,
    FeedbackControllerSettings,
    FeedbackInitialParameters,
    FeedbackObservation,
)
from powder_sampling_control.dispensing_algorithm.calibration import load_fingerprint


POWDER_ID = "powder-20260817-100751-4e849e"


class Lubricant1ProfileTests(unittest.TestCase):
    def setUp(self):
        self.service = DeviceControlService.__new__(DeviceControlService)
        self.service._powder_session = None

    def config(self, target=1000, **extra):
        return self.service.resolve_dispense_configuration({
            "powder_id": POWDER_ID, "target_mg": target, **extra,
        })

    def test_large_precision_reserve_matches_supervised_mode_policy(self):
        # Below 5 g the reserve is a fixed 500 mg.  From 5 g up it is 10% of the
        # target, which keeps the policy continuous at exactly 5 g.
        self.assertEqual(DeviceControlService._large_precision_reserve_mg(1000), 500.0)
        self.assertEqual(DeviceControlService._large_precision_reserve_mg(4999), 500.0)
        self.assertEqual(DeviceControlService._large_precision_reserve_mg(5000), 500.0)
        self.assertEqual(DeviceControlService._large_precision_reserve_mg(6500), 650.0)
        self.assertEqual(DeviceControlService._large_precision_reserve_mg(8000), 800.0)
        self.assertEqual(DeviceControlService._large_precision_reserve_mg(10000), 1000.0)

    def test_name_is_canonical_without_creating_a_second_identity(self):
        fingerprint = load_fingerprint(POWDER_ID)
        self.assertEqual(fingerprint.powder_name, "润滑剂1")
        self.assertEqual(fingerprint.powder_id, POWDER_ID)
        self.assertEqual(resolve_powder_identity({"powder_id": POWDER_ID}), {
            "powder_id": POWDER_ID, "powder_name": "润滑剂1",
        })
        entries = [
            item for item in self.service.list_dispense_powders()
            if item["powder_id"] == POWDER_ID
        ]
        self.assertEqual(len(entries), 1)
        self.assertEqual(entries[0]["powder_name"], "润滑剂1")
        self.assertEqual(entries[0]["config_source"], "dedicated_profile")
        self.assertTrue(entries[0]["selectable"])
        # The probe's auto-classification is historical data, not the new seed.
        self.assertEqual(fingerprint.applied_profile["preset_id"], "steady-70hz-p100")

    def test_candidate_has_explicit_seed_and_is_not_validated(self):
        config = self.config()
        self.assertEqual(config["initial"], {
            "frequency_hz": 70, "duty_permyriad": 2000, "window_position_units": 400,
        })
        self.assertEqual(config["preset_id"], "lubricant-1-70hz-p400")
        self.assertEqual(config["control_profile_status"], "draft")
        self.assertEqual(config["target_resolution"]["resolution"], "exact")
        self.assertEqual(config["acceptance_min_mg"], 990)
        self.assertEqual(config["acceptance_max_mg"], 1010)
        self.assertTrue(config["control_profile_warnings"])
        self.assertNotIn("validation_evidence", load_control_profile(POWDER_ID))
        self.assertEqual(config["controller"]["fixed_tail_mass_mg"], 5)
        self.assertEqual(config["controller"]["settle_confirmations_delta"], 1)
        self.assertEqual(config["controller"]["dead_zone_multiplier"], 1.5)
        self.assertEqual(config["controller"]["max_duty_permyriad"], 3500)

    def test_all_901_targets_keep_legacy_timing_and_product_rates_with_no_window_control(self):
        for target in range(100, 1001):
            with self.subTest(target=target):
                config = self.config(target)
                baseline = interpolate_profile(target)
                rates = resolve_default_target_rate_band(target)
                for key in ("coarse_rate_mg_s", "fine_rate_mg_s", "precision_rate_mg_s"):
                    baseline[key] = rates[key]
                baseline["maximum_flow_rate_mg_s"] = 80.0
                for key, value in baseline.items():
                    self.assertAlmostEqual(config["profile"][key], value)
                controller = config["controller"]
                for key in ("position_feedback_enabled", "stall_recovery_enabled",
                            "tail_pulse_enabled", "fast_predictive_stop_enabled",
                            "projected_safety_stop_enabled", "flow_hold_enabled"):
                    self.assertFalse(controller[key])
                self.assertFalse(resolve_tail_pulse_config(controller, target)["enabled"])
                self.assertEqual(controller["max_duty_permyriad"], 3500)

    def test_dedicated_profile_ignores_old_or_other_powder_sessions(self):
        expected = self.config()
        for session_id in (POWDER_ID, "powder-20260817-143530-182017"):
            self.service._powder_session = {
                "powder_id": session_id,
                "preset_id": "80hz-22pct-p400-plugging-agent-2",
                "initial": {"frequency_hz": 80, "duty_permyriad": 2200,
                            "window_position_units": 400},
                "fixed_tail_mass_mg": 15,
                "stall_recovery_enabled": True,
                "tail_pulse_enabled": True,
            }
            self.assertEqual(self.config(), expected)

    def test_manual_seed_does_not_replace_lubricant_controller(self):
        expected = self.config()
        config = self.config(
            preset_id="80hz-22pct-p400-plugging-agent-2",
            initial={"frequency_hz": 70, "duty_permyriad": 2000,
                     "window_position_units": 400},
        )
        self.assertEqual(config["controller"], expected["controller"])
        self.assertEqual(config["profile"], expected["profile"])
        self.assertEqual(config["initial"], expected["initial"])
        self.assertEqual(config["powder_name"], "润滑剂1")

    def test_stale_browser_name_is_rejected_until_refreshed(self):
        with self.assertRaisesRegex(ValueError, "粉末名称与指纹不一致"):
            self.config(powder_name="润滑剂-text")

    def test_single_run_submission_and_worker_receive_canonical_identity(self):
        service = self.service
        service._test_lock = threading.Lock()
        service._cancel_event = threading.Event()
        service.snapshot = lambda: {"test": {"status": "idle"}}
        service._set_test = lambda _value: None
        with patch("device_control_server.threading.Thread") as worker:
            response = service.start_dispense({"powder_id": POWDER_ID, "target_mg": 1000})
        self.assertEqual(response["submitted"]["powder_name"], "润滑剂1")
        self.assertEqual(response["submitted"]["control_profile_status"], "draft")
        args = worker.call_args.kwargs["args"]
        self.assertEqual(args[-1], {"powder_id": POWDER_ID, "powder_name": "润滑剂1"})
        self.assertFalse(args[-2]["stall_recovery_enabled"])
        self.assertEqual(args[3]["window_position_units"], 400)

    def test_batch_worker_uses_lubricant_settings_and_records_name(self):
        service = self.service
        service._cancel_event = threading.Event()
        state = {"test": {}}
        service.snapshot = lambda: state
        service._set_test = lambda payload: state.update(test=payload)
        service._persist_result = lambda *_args, **_kwargs: None
        captured = []

        def fake_run(run_id, _target, _preset_id, initial, _profile, controller, powder):
            captured.append((initial, controller, powder))
            state["test"] = {"result": {
                "run_id": run_id, "result": "completed", **powder,
                "final_mass_mg": 1000.0, "actual_duration_s": 1.0,
            }}

        service._run_feedback_dispense = fake_run
        service._run_dispense_batch("disp-batch-lubricant-regression", {
            "powder_id": POWDER_ID, "powder_name": "润滑剂1", "tare_between_sets": True,
            "sets": [{"powder_id": POWDER_ID, "powder_name": "润滑剂1",
                      "target_mg": 1000, "preset_id": "lubricant-1-70hz-p400",
                      "frequency_hz": 70, "duty_permyriad": 2000,
                      "window_position_units": 400, "repeat_count": 1}],
        })
        self.assertEqual(len(captured), 1)
        initial, controller, powder = captured[0]
        self.assertEqual(initial["window_position_units"], 400)
        self.assertEqual(powder, {"powder_id": POWDER_ID, "powder_name": "润滑剂1"})
        self.assertEqual(controller["max_duty_permyriad"], 3500)
        self.assertEqual(controller["fixed_tail_mass_mg"], 5)
        self.assertFalse(controller["stall_recovery_enabled"])
        self.assertFalse(controller["tail_pulse_enabled"])
        self.assertEqual(state["test"]["result"]["powder_name"], "润滑剂1")

    def test_real_worker_accepts_30_and_35_percent_at_p550_and_rejects_above_cap(self):
        for duty in (3000, 3500, 3600):
            with self.subTest(duty=duty):
                service = self.service
                service._cancel_event = threading.Event()
                states = []
                moves = []
                service._set_test = states.append
                service._persist_result = lambda *_args, **_kwargs: None

                def fake_move(target, **_kwargs):
                    moves.append(target)
                    raise RuntimeError("synthetic motion boundary; no hardware")

                service._move_window_to = fake_move
                config = self.config(initial={
                    "frequency_hz": 80, "duty_permyriad": duty,
                    "window_position_units": 550,
                })
                service._run_feedback_dispense(
                    "lubricant-duty-limit-regression", 1000, config["preset_id"],
                    config["initial"], config["profile"], config["controller"],
                    {"powder_id": POWDER_ID, "powder_name": "润滑剂1"},
                )
                self.assertEqual(states[-1]["status"], "failed")
                error = states[-1]["result"]["error"]
                if duty <= 3500:
                    self.assertEqual(moves, [550])
                    self.assertEqual(error, "synthetic motion boundary; no hardware")
                else:
                    self.assertEqual(moves, [])
                    self.assertIn("initial duty is outside controller limits", error)

    def test_feedback_can_rise_above_25_percent_but_still_brakes_and_keeps_window(self):
        config = self.config()
        controller = ContinuousFeedbackController(
            target_mass_mg=1000,
            initial=FeedbackInitialParameters(80, 2500, 550),
            settings=FeedbackControllerSettings(
                coarse_rate_mg_s=config["profile"]["coarse_rate_mg_s"],
                maximum_flow_rate_mg_s=config["profile"]["maximum_flow_rate_mg_s"],
                max_duty_permyriad=config["controller"]["max_duty_permyriad"],
                duty_step_per_update=config["controller"]["duty_step_per_update"],
                emergency_duty_step_per_update=config["controller"]["emergency_duty_step_per_update"],
                position_feedback_enabled=config["controller"]["position_feedback_enabled"],
                stall_recovery_enabled=config["controller"]["stall_recovery_enabled"],
                force_stop_max_mass_mg=1000-config["controller"]["force_stop_offset_mg"],
            ),
        )
        decisions = [controller.step(
            FeedbackObservation(154, 154, 0, 0, 0, True), timestamp_s=index * 0.5,
        ) for index in range(100)]
        self.assertEqual(max(d.duty_after_permyriad for d in decisions), 3500)
        self.assertTrue(all(d.window_after_units == 550 for d in decisions))
        self.assertTrue(all(not d.window_changed for d in decisions))
        overspeed = controller.step(
            FeedbackObservation(154, 154, 100, 100, 0, True), timestamp_s=51,
        )
        self.assertLess(overspeed.duty_after_permyriad, 3500)
        stop = controller.step(
            FeedbackObservation(995, 995, 0, 0, 0, True), timestamp_s=52,
        )
        self.assertTrue(stop.stop_requested)

    def test_other_powders_keep_their_existing_duty_ceilings(self):
        for powder_id, expected in (
            ("powder-20260817-143530-182017", 2500),
            ("water_loss_agent_2", 3000),
            ("bentonite", 5000),
        ):
            config = self.service.resolve_dispense_configuration({
                "powder_id": powder_id, "target_mg": 1000,
            })
            self.assertEqual(config["controller"]["max_duty_permyriad"], expected)

    def test_research_workspace_distinguishes_inferred_history_from_validation(self):
        workspace = json.loads(
            (PROJECT_ROOT / "data" / "powder_library" / "润滑剂1.json").read_text(encoding="utf-8")
        )
        self.assertEqual(workspace["powder_id"], POWDER_ID)
        self.assertEqual(workspace["aliases"], ["润滑剂-text"])
        self.assertEqual(workspace["historical_attribution"]["method"], "probe_time_interval_inference")
        self.assertFalse(workspace["historical_summary"]["validation_of_current_candidate"])
        self.assertEqual(len(workspace["historical_runs"]), 34)
        self.assertEqual({run["target_mass_mg"] for run in workspace["historical_runs"]}, {100})


if __name__ == "__main__":
    unittest.main()
