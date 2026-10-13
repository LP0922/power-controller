import json
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from powder_sampling_control.dispensing_algorithm.core import (
    list_control_profiles,
    load_control_profile,
    normalize_target_keys,
    resolve_control_profile,
)
from powder_sampling_control.dispensing_algorithm.core import (
    build_candidate_control_profile,
)
from powder_sampling_control.dispensing_algorithm.calibration import (
    list_fingerprints,
    load_fingerprint,
)
from powder_sampling_control.dispensing_algorithm.core import (
    acceptance_limits,
    validate_target_mass,
)


class ProductRequirementsTests(unittest.TestCase):
    def test_acceptance_is_symmetric_ten_mg(self):
        self.assertEqual(acceptance_limits(437), (427.0, 447.0))

    def test_every_integer_from_100_through_1000_is_valid(self):
        for target in range(100, 1001):
            self.assertEqual(validate_target_mass(target), target)

    def test_non_integer_and_out_of_range_targets_are_rejected(self):
        for target in (99, 1001, 100.0, "100", True):
            with self.assertRaises(ValueError):
                validate_target_mass(target)


class ControlProfileTests(unittest.TestCase):
    def test_bentonite_fingerprint_uses_canonical_control_profile_id(self):
        fingerprint = load_fingerprint("bentonite")
        self.assertIsNotNone(fingerprint)
        self.assertEqual(fingerprint.powder_id, "bentonite")
        self.assertEqual(fingerprint.powder_name, "膨润土")
        self.assertIsNone(load_fingerprint("powder-20260811-101154-c32e23"))
        self.assertEqual(
            sum(item["powder_id"] == "bentonite" for item in list_fingerprints()),
            1,
        )

    def test_water_loss_agent_2_fingerprint_uses_canonical_control_profile_id(self):
        fingerprint = load_fingerprint("water_loss_agent_2")
        self.assertIsNotNone(fingerprint)
        self.assertEqual(fingerprint.powder_id, "water_loss_agent_2")
        self.assertEqual(fingerprint.powder_name, "失水剂2")
        self.assertIsNone(load_fingerprint("powder-20260817-110326-9f48b6"))
        self.assertEqual(
            sum(
                item["powder_id"] == "water_loss_agent_2"
                for item in list_fingerprints()
            ),
            1,
        )

    def test_runtime_profiles_include_reference_powders_and_plugging_agents(self):
        ids = {profile["powder_id"] for profile in list_control_profiles()}
        self.assertEqual(
            ids,
            {
                "baking_soda",
                "bentonite",
                "slaked_lime",
                "corn_starch",
                "powder-20260814-143554-702da0",
                "powder-20260817-100751-4e849e",
                "powder-20260817-143530-182017",
                "water_loss_agent_2",
            },
        )

    def test_plugging_agent_1_has_a_dedicated_conservative_profile(self):
        profile = load_control_profile("powder-20260814-143554-702da0")
        resolved = resolve_control_profile(profile, 1000)
        self.assertEqual(resolved["powder_name"], "封堵剂1")
        self.assertEqual(resolved["status"], "validated")
        self.assertEqual(resolved["target_rate_band_source"], "powder_profile")
        self.assertEqual(
            (
                resolved["target_profile"]["coarse_rate_mg_s"],
                resolved["target_profile"]["fine_rate_mg_s"],
                resolved["target_profile"]["precision_rate_mg_s"],
            ),
            (15.0, 6.0, 3.0),
        )
        self.assertEqual(profile["recommended_controller"]["max_duty_permyriad"], 3000)
        self.assertTrue(profile["recommended_controller"]["flow_hold_enabled"])
        self.assertEqual(profile["generation_warnings"], [])
        self.assertEqual(profile["validation_evidence"]["passed_runs"], 2)
        self.assertEqual(profile["validation_evidence"]["final_masses_mg"], [996.0, 996.0])

    def test_arbitrary_integer_target_is_interpolated(self):
        resolved = resolve_control_profile(load_control_profile("corn_starch"), 437)
        self.assertEqual(resolved["target_resolution"]["resolution"], "interpolated")
        self.assertEqual(resolved["target_profile"]["coarse_rate_mg_s"], 25.0)
        self.assertEqual(resolved["target_rate_band_source"], "product_default")

    def test_all_powders_use_the_product_default_rates_without_interpolation(self):
        expected_by_target = {
            250: (25.0, 15.0, 10.0, 0.0),
            437: (25.0, 15.0, 10.0, 0.0),
            750: (35.0, 20.0, 14.0, 0.0),
        }
        for powder_id in ("baking_soda", "bentonite", "slaked_lime", "corn_starch"):
            profile = load_control_profile(powder_id)
            for target, expected in expected_by_target.items():
                resolved = resolve_control_profile(profile, target)
                rates = resolved["target_profile"]
                self.assertEqual(
                    (
                        rates["coarse_rate_mg_s"],
                        rates["fine_rate_mg_s"],
                        rates["precision_rate_mg_s"],
                        resolved["target_rate_band"]["settle_rate_mg_s"],
                    ),
                    expected,
                )

    def test_bentonite_1000mg_validated_profile_is_exact_and_uses_dedicated_settings(self):
        profile = load_control_profile("bentonite")
        resolved = resolve_control_profile(profile, 1000)
        self.assertEqual(resolved["status"], "validated")
        self.assertEqual(resolved["target_resolution"]["resolution"], "exact")
        self.assertEqual(resolved["recommended_preset_id"], "80hz-22pct-p300-bentonite")
        self.assertEqual(
            resolved["recommended_controller"],
            {
                "duty_step_per_update": 50,
                "emergency_duty_step_per_update": 200,
                "position_feedback_enabled": False,
                "maximum_flow_rate_mg_s": 50.0,
                "fixed_tail_mass_mg": 5.0,
                "fast_predictive_stop_enabled": True,
                "fast_predictive_stop_rate_mg_s": 20.0,
                "fast_predictive_stop_confirmations": 1,
                "projected_safety_stop_enabled": True,
                "projected_safety_stop_offset_mg": 5.0,
                "force_stop_offset_mg": 10.0,
                "max_duty_permyriad": 5000,
                "flow_hold_enabled": False,
            },
        )
        self.assertEqual(resolved["target_profile"]["precision_start_remaining_mg"], 37.5)
        self.assertEqual(resolved["target_profile"]["tail_taper_start_remaining_mg"], 137.5)
        self.assertEqual(resolved["target_profile"]["tail_taper_end_remaining_mg"], 10.0)
        self.assertTrue(
            any(
                "strict product-window performance was 9 of 10" in warning
                for warning in resolved["generation_warnings"]
            )
        )
        evidence = profile["validation_evidence"]
        self.assertEqual(evidence["passed_runs"], 9)
        self.assertEqual(evidence["operator_accepted_runs"], 10)
        self.assertEqual(evidence["total_runs"], 10)
        self.assertEqual(evidence["algorithm_version"], "predictive-stop-v3-bentonite-fast-tail")
        self.assertEqual(evidence["final_masses_mg"][-1], 1004.0)

    def test_bentonite_uses_the_configured_stage_rate_bands(self):
        profile = load_control_profile("bentonite")
        expected = {
            100: (25.0, 15.0, 10.0, 0.0),
            250: (25.0, 15.0, 10.0, 0.0),
            437: (25.0, 15.0, 10.0, 0.0),
            500: (25.0, 15.0, 10.0, 0.0),
            501: (35.0, 20.0, 14.0, 0.0),
            750: (35.0, 20.0, 14.0, 0.0),
            1000: (35.0, 20.0, 14.0, 0.0),
        }
        for target, rates in expected.items():
            resolved = resolve_control_profile(profile, target)
            target_profile = resolved["target_profile"]
            rate_band = resolved["target_rate_band"]
            self.assertEqual(
                (
                    target_profile["coarse_rate_mg_s"],
                    target_profile["fine_rate_mg_s"],
                    target_profile["precision_rate_mg_s"],
                    rate_band["settle_rate_mg_s"],
                ),
                rates,
            )

    def test_water_loss_agent_2_1000mg_validated_profile_requires_two_stop_confirmations(self):
        profile = load_control_profile("water_loss_agent_2")
        resolved = resolve_control_profile(profile, 1000)

        self.assertEqual(resolved["powder_name"], "失水剂2")
        self.assertEqual(resolved["status"], "validated")
        self.assertEqual(resolved["target_resolution"]["resolution"], "exact")
        self.assertEqual(
            resolved["recommended_preset_id"],
            "80hz-22pct-p350-water-loss-agent-2",
        )
        controller = resolved["recommended_controller"]
        self.assertEqual(controller["settle_confirmations_delta"], 0)
        self.assertEqual(controller["maximum_flow_rate_mg_s"], 30.0)
        self.assertEqual(controller["fixed_tail_mass_mg"], 0.0)
        self.assertFalse(controller["fast_predictive_stop_enabled"])
        self.assertEqual(controller["fast_predictive_stop_rate_mg_s"], 20.0)
        self.assertEqual(controller["fast_predictive_stop_confirmations"], 1)
        self.assertTrue(controller["projected_safety_stop_enabled"])
        self.assertEqual(controller["projected_safety_stop_offset_mg"], 5.0)
        self.assertTrue(controller["stall_recovery_enabled"])
        self.assertEqual(controller["stall_recovery_observation_window_s"], 3.0)
        self.assertEqual(controller["stall_recovery_window_step_units"], 25)
        self.assertEqual(controller["stall_recovery_max_window_position_units"], 400)
        self.assertEqual(controller["stall_recovery_max_attempts"], 2)
        self.assertEqual(controller["stall_recovery_disable_remaining_mg"], 50.0)
        self.assertEqual(controller["max_duty_permyriad"], 3000)
        self.assertTrue(controller["flow_hold_enabled"])
        self.assertEqual(controller["flow_hold_window_s"], 4.0)
        self.assertEqual(controller["flow_hold_min_gain_mg"], 20.0)
        evidence = profile["validation_evidence"]
        self.assertEqual(evidence["passed_runs"], 9)
        self.assertEqual(evidence["operator_accepted_runs"], 10)
        self.assertEqual(evidence["total_runs"], 10)
        self.assertEqual(evidence["successful_stall_recovery_count"], 19)
        self.assertEqual(evidence["failed_stall_recovery_count"], 0)
        self.assertEqual(evidence["mean_final_mass_mg"], 1001.6)
        self.assertEqual(
            (
                resolved["target_profile"]["coarse_rate_mg_s"],
                resolved["target_profile"]["fine_rate_mg_s"],
                resolved["target_profile"]["precision_rate_mg_s"],
            ),
            (35.0, 20.0, 14.0),
        )

    def test_plugging_agent_2_1000mg_candidate_uses_historical_seed_with_duty_guard(self):
        profile = load_control_profile("powder-20260817-143530-182017")
        resolved = resolve_control_profile(profile, 1000)

        self.assertEqual(resolved["powder_name"], "封堵剂2")
        self.assertEqual(resolved["status"], "draft")
        self.assertEqual(
            resolved["recommended_preset_id"],
            "80hz-22pct-p400-plugging-agent-2",
        )
        controller = resolved["recommended_controller"]
        self.assertEqual(controller["maximum_flow_rate_mg_s"], 40.0)
        self.assertEqual(controller["max_duty_permyriad"], 2500)
        self.assertTrue(controller["flow_hold_enabled"])
        self.assertEqual(controller["flow_hold_min_gain_mg"], 20.0)
        self.assertTrue(controller["fast_predictive_stop_enabled"])
        self.assertEqual(controller["fast_predictive_stop_rate_mg_s"], 20.0)
        self.assertEqual(controller["fast_predictive_stop_confirmations"], 1)
        self.assertEqual(controller["fixed_tail_mass_mg"], 15.0)
        self.assertEqual(controller["force_stop_offset_mg"], 40.0)
        self.assertFalse(any(key.startswith("tail_pulse_") for key in controller))
        self.assertFalse(controller["stall_recovery_enabled"])
        self.assertFalse(controller["position_feedback_enabled"])
        self.assertEqual(controller["stall_recovery_observation_window_s"], 3.0)
        self.assertEqual(controller["stall_recovery_window_step_units"], 25)
        self.assertEqual(controller["stall_recovery_max_window_position_units"], 425)
        self.assertEqual(controller["stall_recovery_max_attempts"], 2)
        self.assertEqual(controller["stall_recovery_disable_remaining_mg"], 100.0)
        self.assertEqual(resolved["target_profile"]["precision_start_remaining_mg"], 80.0)
        self.assertEqual(resolved["target_profile"]["tail_taper_start_remaining_mg"], 80.0)
        self.assertEqual(
            (
                resolved["target_profile"]["coarse_rate_mg_s"],
                resolved["target_profile"]["fine_rate_mg_s"],
                resolved["target_profile"]["precision_rate_mg_s"],
            ),
            (35.0, 20.0, 14.0),
        )

        resolved_500 = resolve_control_profile(profile, 500)
        self.assertEqual(resolved_500["recommended_controller"]["fixed_tail_mass_mg"], 2.0)
        self.assertEqual(resolved_500["recommended_controller"]["force_stop_offset_mg"], 10.0)
        self.assertNotIn(
            "tail_pulse_enabled",
            resolved_500["recommended_controller"],
        )
        self.assertEqual(
            resolved_500["target_profile"]["precision_start_remaining_mg"],
            70.0,
        )

    def test_json_string_keys_are_normalized_to_integers(self):
        normalized = normalize_target_keys({"100": {"fine_rate_mg_s": 10}})
        self.assertEqual(normalized, {100: {"fine_rate_mg_s": 10}})

    def test_workspace_generator_keeps_incomplete_data_as_draft_warning(self):
        workspace = json.loads(
            (PROJECT_ROOT / "data" / "powder_library" / "玉米淀粉.json").read_text(
                encoding="utf-8"
            )
        )
        candidate = build_candidate_control_profile(workspace)
        self.assertEqual(candidate["status"], "draft")
        self.assertTrue(candidate["generation_warnings"])
        self.assertIn("100", candidate["target_profiles"])

    def test_workspace_generator_preserves_target_rate_bands(self):
        workspace = json.loads(
            (PROJECT_ROOT / "data" / "powder_library" / "膨润土.json").read_text(
                encoding="utf-8"
            )
        )
        candidate = build_candidate_control_profile(workspace)
        self.assertEqual(candidate["target_rate_bands"], workspace["target_rate_bands"])


if __name__ == "__main__":
    unittest.main()
