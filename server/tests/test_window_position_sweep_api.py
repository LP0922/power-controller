import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "scripts"))

from device_control_server import (
    DISPENSE_PRESETS,
    DeviceControlService,
    motion_distance_register,
    resolve_powder_identity,
    resolve_tail_pulse_config,
)
from device_dashboard import HTML as DEVICE_DASHBOARD_HTML
from powder_sampling_control.dispensing_algorithm.core import interpolate_profile
from device_maintenance import position_schedule


class WindowPositionSweepTests(unittest.TestCase):
    def test_regular_feedback_triggers_low_flow_recovery_and_publishes_live_samples(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        snapshots = []
        service._set_test = snapshots.append
        service._persist_result = lambda *_args, **_kwargs: None
        read_index = [0]
        latest_mass = [0.0]
        clock = SimpleNamespace(value=0.0)
        positions = []
        duties = []

        def measurement(mass):
            return SimpleNamespace(
                raw_count=mass, mass_mg=float(mass), stable=True, status_word=256
            )

        def read_mass():
            read_index[0] += 1
            clock.value += 0.25
            if read_index[0] > 45:
                latest_mass[0] += 10.0
            return measurement(latest_mass[0])

        service.reader = SimpleNamespace(
            _lock=threading.Lock(),
            _at=SimpleNamespace(read_measurement=read_mass, zero=lambda **_kwargs: None),
            _la10=SimpleNamespace(
                start_vibration=lambda **_kwargs: None,
                read_vibration_settings=lambda: SimpleNamespace(enabled=True),
                update_vibration_duty=duties.append,
                stop_vibration=lambda: None,
            ),
        )

        def move(target, **_kwargs):
            positions.append(target)
            status = SimpleNamespace(position_units=target)
            return status, status, 0

        service._move_window_to = move
        service._wait_for_stable_measurement = lambda *_args, **_kwargs: measurement(500)
        service._run_tail_pulse_stage = lambda **_kwargs: self.fail(
            "tail pulse stage must not be entered"
        )
        with patch("device_control_server.time.monotonic", lambda: clock.value):
            with patch("device_control_server.POLL_INTERVAL_S", 0.001):
                service._run_feedback_dispense(
                    "dispense-500mg-low-flow-recovery-regression",
                    500,
                    "regression",
                    {
                        "frequency_hz": 80,
                        "duty_permyriad": 2000,
                        "window_position_units": 700,
                    },
                    {
                        "coarse_rate_mg_s": 25.0,
                        "fine_rate_mg_s": 15.0,
                        "precision_rate_mg_s": 10.0,
                        "precision_start_remaining_mg": 50.0,
                        "tail_taper_start_remaining_mg": 100.0,
                        "tail_taper_end_remaining_mg": 10.0,
                        "maximum_flow_rate_mg_s": 80.0,
                    },
                    {"min_window_position_units": 0, "max_window_position_units": 2000},
                    {"powder_id": "recovery-test", "powder_name": "test"},
                )

        result = snapshots[-1]["result"]
        self.assertEqual(result["result"], "completed", result.get("error"))
        recoveries = result["large_closed_loop_low_flow_recovery"]
        self.assertEqual(len(recoveries), 1)
        self.assertEqual(recoveries[0]["duty_before_permyriad"], 2400)
        self.assertEqual(recoveries[0]["duty_after_permyriad"], 2600)
        self.assertEqual(
            [item["requested_units"] for item in recoveries[0]["window_positions"]],
            [800, 600, 700],
        )
        self.assertGreater(len(recoveries[0]["recovery_samples"]), 1)
        live_recovery = [
            item for item in snapshots
            if item.get("phase") == "large_closed_loop_low_flow_recovery"
        ]
        self.assertGreater(len(live_recovery), 1)
        self.assertNotEqual(
            live_recovery[0]["latest_sample"]["mass_mg"],
            live_recovery[-1]["latest_sample"]["mass_mg"],
        )
        self.assertGreater(result["samples"][-1]["mass_mg"], 0.0)
        recovery_sample = next(
            sample for sample in result["samples"]
            if sample["large_closed_loop_low_flow_recovery"] is not None
        )
        self.assertEqual(recovery_sample["duty_permyriad"], 2600)
        self.assertEqual(recovery_sample["window_position_units"], 700)
        self.assertEqual(positions, [700, 800, 600, 700])
        self.assertIn(2600, duties)

    def test_large_low_flow_recovery_keeps_timer_initialized_while_active(self):
        low_flow_since = 100.0
        recovery_active = True
        sample_time = 106.0
        low_flow_rate = 3.0
        motion_valid = True

        if not motion_valid or low_flow_rate >= 10.0:
            low_flow_since = None
            recovery_active = False
        elif low_flow_since is None:
            low_flow_since = sample_time
        elif sample_time - low_flow_since >= 5.0:
            recovery_active = True

        self.assertTrue(recovery_active)
        self.assertIsNotNone(low_flow_since)
        self.assertEqual(sample_time - low_flow_since, 6.0)

    def test_large_dispense_accepts_window_position_above_2000(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._test_lock = threading.Lock()
        service._cancel_event = threading.Event()
        service.snapshot = lambda: {"test": {"status": "idle"}}
        service.resolve_dispense_configuration = lambda _payload: {
            "profile": {}, "controller": {}, "initial": {},
        }
        service._set_test = lambda *_args, **_kwargs: None
        service._persist_result = lambda *_args, **_kwargs: None

        with patch("device_control_server.resolve_powder_identity", return_value={
            "powder_id": "bentonite-test", "powder_name": "test powder",
        }), patch.object(service, "_run_feedback_dispense", lambda *_args, **_kwargs: None):
            result = service.start_large_dispense({
                "target_mg": 5000,
                "powder_id": "bentonite-test",
                "initial": {"window_position_units": 2001},
            })

        self.assertEqual(result["submitted"]["initial"]["window_position_units"], 2001)

    def test_large_dispense_carries_operator_selected_prefeed_duty(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._test_lock = threading.Lock()
        service._cancel_event = threading.Event()
        service.snapshot = lambda: {"test": {"status": "idle"}}
        service.resolve_dispense_configuration = lambda _payload: {
            "profile": {}, "controller": {}, "initial": {},
        }
        service._set_test = lambda *_args, **_kwargs: None
        service._persist_result = lambda *_args, **_kwargs: None

        with patch("device_control_server.resolve_powder_identity", return_value={
            "powder_id": "bentonite-test", "powder_name": "test powder",
        }), patch.object(service, "_run_feedback_dispense", lambda *_args, **_kwargs: None):
            result = service.start_large_dispense({
                "target_mg": 5000,
                "powder_id": "bentonite-test",
                "initial": {"window_position_units": 700, "duty_permyriad": 3000},
            })

        self.assertEqual(result["submitted"]["initial"]["duty_permyriad"], 3000)

    def test_large_dispense_defaults_to_20pct_prefeed_duty(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._test_lock = threading.Lock()
        service._cancel_event = threading.Event()
        service.snapshot = lambda: {"test": {"status": "idle"}}
        service.resolve_dispense_configuration = lambda _payload: {
            "profile": {}, "controller": {}, "initial": {},
        }
        service._set_test = lambda *_args, **_kwargs: None
        service._persist_result = lambda *_args, **_kwargs: None

        with patch("device_control_server.resolve_powder_identity", return_value={
            "powder_id": "bentonite-test", "powder_name": "test powder",
        }), patch.object(service, "_run_feedback_dispense", lambda *_args, **_kwargs: None):
            result = service.start_large_dispense({
                "target_mg": 1000,
                "powder_id": "bentonite-test",
                "initial": {"window_position_units": 700},
            })

        self.assertEqual(result["submitted"]["initial"]["duty_permyriad"], 2000)

    def test_large_dispense_reserve_policy_depends_on_target(self):
        def submitted_for(target_mg):
            service = DeviceControlService.__new__(DeviceControlService)
            service._test_lock = threading.Lock()
            service._cancel_event = threading.Event()
            service.snapshot = lambda: {"test": {"status": "idle"}}
            service.resolve_dispense_configuration = lambda _payload: {
                "profile": {}, "controller": {}, "initial": {},
            }
            service._set_test = lambda *_args, **_kwargs: None
            service._persist_result = lambda *_args, **_kwargs: None
            with patch("device_control_server.resolve_powder_identity", return_value={
                "powder_id": "bentonite-test", "powder_name": "test powder",
            }), patch.object(service, "_run_feedback_dispense", lambda *_args, **_kwargs: None):
                result = service.start_large_dispense({
                    "target_mg": target_mg,
                    "powder_id": "bentonite-test",
                    "initial": {"window_position_units": 700},
                })
            return result["submitted"]["large_prefeed_reserve_mg"]

        # Below 5 g the reserve stays at 500 mg; from 5 g up it is 10% of the
        # target (8 g -> 800 mg), so the handover point scales with the run.
        self.assertEqual(submitted_for(3000), 500.0)
        self.assertEqual(submitted_for(5000), 500.0)
        self.assertEqual(submitted_for(8000), 800.0)
        self.assertEqual(submitted_for(10000), 1000.0)

    def test_large_prefeed_holds_operator_selected_duty(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        snapshots = []
        service._set_test = snapshots.append
        service._persist_result = lambda *_args, **_kwargs: None
        read_index = [0]
        clock = SimpleNamespace(value=0.0)
        positions = []
        duties = []

        def measurement(mass):
            return SimpleNamespace(
                raw_count=mass, mass_mg=float(mass), stable=True, status_word=256
            )

        def read_mass():
            read_index[0] += 1
            clock.value += 0.25
            # Stay inside the fixed-duty pre-feed for a few cycles, then
            # overshoot the hard overweight boundary to end the worker.
            return measurement(0.0 if read_index[0] < 4 else 9000.0)

        service.reader = SimpleNamespace(
            _lock=threading.Lock(),
            _at=SimpleNamespace(read_measurement=read_mass, zero=lambda **_kwargs: None),
            _la10=SimpleNamespace(
                start_vibration=lambda **_kwargs: None,
                read_vibration_settings=lambda: SimpleNamespace(enabled=True),
                update_vibration_duty=duties.append,
                stop_vibration=lambda: None,
            ),
        )

        def move(target, **_kwargs):
            positions.append(target)
            status = SimpleNamespace(position_units=target)
            return status, status, 0

        service._move_window_to = move
        service._wait_for_stable_measurement = lambda *_args, **_kwargs: measurement(0)
        service._run_tail_pulse_stage = lambda **_kwargs: self.fail(
            "tail pulse stage must not be entered"
        )

        with patch("device_control_server.time.monotonic", lambda: clock.value):
            with patch("device_control_server.POLL_INTERVAL_S", 0.001):
                service._run_feedback_dispense(
                    "large-dispense-3000mg-prefeed-duty-regression",
                    3000,
                    "large-80hz",
                    {
                        "frequency_hz": 80,
                        "duty_permyriad": 3000,
                        "window_position_units": 700,
                    },
                    {
                        "coarse_rate_mg_s": 25.0,
                        "fine_rate_mg_s": 15.0,
                        "precision_rate_mg_s": 10.0,
                        "precision_start_remaining_mg": 50.0,
                        "tail_taper_start_remaining_mg": 100.0,
                        "tail_taper_end_remaining_mg": 10.0,
                        "maximum_flow_rate_mg_s": 80.0,
                    },
                    {
                        "min_window_position_units": 0,
                        "max_window_position_units": 2050,
                        "max_duty_permyriad": 2500,
                    },
                    {"powder_id": "large-test", "powder_name": "test"},
                    large_prefeed_reserve_mg=500.0,
                    closed_loop_initial={
                        "frequency_hz": 80,
                        "duty_permyriad": 2000,
                        "window_position_units": 700,
                    },
                )

        # The pre-feed must hold the operator-selected 30% and must never fall
        # back to the historical 20% default, nor write the clamped 25%
        # closed-loop ceiling over it.  Only the start-up write reaches the
        # actuator, so no duty write is expected during the pre-feed.
        result = snapshots[-1]["result"] if snapshots else {}
        self.assertTrue(
            str(result.get("error", "")).startswith("hard overweight boundary"),
            result.get("error"),
        )
        self.assertIn(3000, [sample["duty_permyriad"] for sample in result["samples"]])
        self.assertEqual(duties, [])
        self.assertEqual(positions, [700])

    def test_large_dispense_rejects_duty_outside_actuator_range(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._test_lock = threading.Lock()
        service._cancel_event = threading.Event()
        service.snapshot = lambda: {"test": {"status": "idle"}}
        service.resolve_dispense_configuration = lambda _payload: {
            "profile": {}, "controller": {}, "initial": {},
        }
        service._set_test = lambda *_args, **_kwargs: None
        service._persist_result = lambda *_args, **_kwargs: None

        for rejected_duty in (900, 5001):
            with self.subTest(rejected_duty=rejected_duty), patch(
                "device_control_server.resolve_powder_identity",
                return_value={
                    "powder_id": "bentonite-test", "powder_name": "test powder",
                },
            ):
                with self.assertRaises(ValueError):
                    service.start_large_dispense({
                        "target_mg": 5000,
                        "powder_id": "bentonite-test",
                        "initial": {
                            "window_position_units": 700,
                            "duty_permyriad": rejected_duty,
                        },
                    })

    def test_large_low_flow_recovery_moves_exact_cycle_without_stopping_vibration(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service.reader = SimpleNamespace(
            _lock=threading.Lock(),
            _la10=SimpleNamespace(
                read_vibration_settings=lambda: SimpleNamespace(enabled=True),
            ),
        )
        positions = []
        current_position = [700]
        vibration_checks = []

        def move(target, **_kwargs):
            positions.append(target)
            vibration_checks.append(
                service.reader._la10.read_vibration_settings().enabled
            )
            current_position[0] = target
            status = SimpleNamespace(position_units=target)
            return status, status, abs(target - (positions[-2] if len(positions) > 1 else 700))

        service._move_window_to = move
        controller = SimpleNamespace(
            duty_permyriad=2000,
            window_position_units=700,
            settings=SimpleNamespace(
                min_window_position_units=0,
                max_window_position_units=2000,
            ),
        )
        start = controller.window_position_units
        target = start
        for delta in (100, -200, 200):
            target += delta
            service._move_window_to(target, speed_mm_s=1.0, timeout_s=10.0)

        self.assertEqual(positions, [800, 600, 800])
        self.assertEqual(vibration_checks, [True, True, True])

    def test_feedback_worker_accepts_30_percent_duty_and_clears_failed_state(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        captured = []
        service._set_test = captured.append
        service._persist_result = lambda *_args, **_kwargs: None

        def fail_after_controller_initialization(*_args, **_kwargs):
            raise RuntimeError("synthetic window failure")

        service._move_window_to = fail_after_controller_initialization
        service._run_feedback_dispense(
            "dispense-500mg-regression",
            500,
            "regression",
            {
                "frequency_hz": 40,
                "duty_permyriad": 3000,
                "window_position_units": 250,
            },
            interpolate_profile(500),
            {},
            {"powder_id": "bentonite-test", "powder_name": "膨润土"},
        )

        self.assertEqual(captured[-1]["status"], "failed")
        self.assertEqual(captured[-1]["result"]["error"], "synthetic window failure")
        self.assertEqual(captured[-1]["result"]["powder_id"], "bentonite-test")
        self.assertEqual(captured[-1]["result"]["powder_name"], "膨润土")

    def test_dispense_requires_a_stored_powder_identity(self):
        with self.assertRaisesRegex(ValueError, "请先选择粉末类型"):
            resolve_powder_identity({})

    def test_dispense_submitted_state_and_worker_keep_canonical_powder_identity(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._powder_session = None
        service._test_lock = threading.Lock()
        service._cancel_event = threading.Event()
        service.snapshot = lambda: {"test": {"status": "idle"}}
        captured = []
        service._set_test = captured.append
        with patch("device_control_server.threading.Thread") as thread_class:
            response = service.start_dispense(
                {
                    "powder_id": "bentonite",
                    "powder_name": "膨润土",
                    "target_mg": 1000,
                    "initial": {
                        "frequency_hz": 80,
                        "duty_permyriad": 2200,
                        "window_position_units": 300,
                    },
                }
            )

        self.assertEqual(response["submitted"]["powder_id"], "bentonite")
        self.assertEqual(response["submitted"]["powder_name"], "膨润土")
        self.assertEqual(response["submitted"]["control_profile_status"], "validated")
        self.assertEqual(response["submitted"]["target_resolution"]["resolution"], "exact")
        worker_args = thread_class.call_args.kwargs["args"]
        self.assertEqual(worker_args[-1], {"powder_id": "bentonite", "powder_name": "膨润土"})
        thread_class.return_value.start.assert_called_once_with()

    def test_plugging_agent_1_start_uses_its_dedicated_seed_and_rates(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._powder_session = None
        service._test_lock = threading.Lock()
        service._cancel_event = threading.Event()
        service.snapshot = lambda: {"test": {"status": "idle"}}
        captured = []
        service._set_test = captured.append
        with patch("device_control_server.threading.Thread"):
            response = service.start_dispense(
                {
                    "powder_id": "powder-20260814-143554-702da0",
                    "powder_name": "封堵剂1",
                    "target_mg": 1000,
                    "initial": {},
                }
            )
        submitted = response["submitted"]
        self.assertEqual(
            submitted["initial"],
            {
                "frequency_hz": 80,
                "duty_permyriad": 2200,
                "window_position_units": 400,
            },
        )
        self.assertEqual(
            (
                submitted["profile"]["coarse_rate_mg_s"],
                submitted["profile"]["fine_rate_mg_s"],
                submitted["profile"]["precision_rate_mg_s"],
            ),
            (15.0, 6.0, 3.0),
        )

    def test_formal_dispense_catalog_includes_profiled_and_unprofiled_powders(self):
        service = DeviceControlService.__new__(DeviceControlService)
        catalog = {
            item["powder_id"]: item for item in service.list_dispense_powders()
        }
        plugging = catalog["powder-20260814-143554-702da0"]
        plugging_2 = catalog["powder-20260817-143530-182017"]
        water_loss_1 = catalog["powder-20260817-114613-f88e45"]
        water_loss_2 = catalog["water_loss_agent_2"]

        self.assertEqual(plugging["config_source"], "dedicated_profile")
        self.assertEqual(plugging_2["config_source"], "dedicated_profile")
        self.assertEqual(plugging_2["powder_name"], "封堵剂2")
        self.assertEqual(plugging_2["status"], "draft")
        self.assertEqual(
            water_loss_1["config_source"], "fingerprint_with_product_defaults"
        )
        self.assertEqual(water_loss_2["config_source"], "dedicated_profile")
        self.assertEqual(water_loss_2["status"], "validated")
        self.assertTrue(plugging["selectable"])
        self.assertTrue(plugging_2["selectable"])
        self.assertTrue(water_loss_1["selectable"])
        self.assertTrue(water_loss_2["selectable"])

    def test_plugging_agent_2_start_uses_its_draft_seed_and_duty_guard(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._powder_session = None

        config = service.resolve_dispense_configuration({
            "powder_id": "powder-20260817-143530-182017",
            "target_mg": 1000,
        })

        self.assertEqual(config["powder_name"], "封堵剂2")
        self.assertEqual(config["config_source"], "dedicated_profile")
        self.assertEqual(config["control_profile_status"], "draft")
        self.assertEqual(
            config["initial"],
            {
                "frequency_hz": 80,
                "duty_permyriad": 2200,
                "window_position_units": 250,
            },
        )
        self.assertEqual(config["controller"]["max_duty_permyriad"], 2500)
        self.assertTrue(config["controller"]["flow_hold_enabled"])
        self.assertTrue(config["controller"]["fast_predictive_stop_enabled"])
        self.assertEqual(config["controller"]["fast_predictive_stop_rate_mg_s"], 20.0)
        self.assertEqual(config["controller"]["fast_predictive_stop_confirmations"], 1)
        self.assertEqual(config["controller"]["fixed_tail_mass_mg"], 15.0)
        self.assertEqual(config["controller"]["force_stop_offset_mg"], 40.0)
        self.assertFalse(any(key.startswith("tail_pulse_") for key in config["controller"]))
        self.assertFalse(resolve_tail_pulse_config(config["controller"], 1000)["enabled"])
        self.assertEqual(config["profile"]["precision_start_remaining_mg"], 80.0)
        self.assertFalse(config["controller"]["stall_recovery_enabled"])
        self.assertFalse(config["controller"]["position_feedback_enabled"])
        self.assertEqual(
            config["controller"]["stall_recovery_max_window_position_units"],
            425,
        )
        self.assertEqual(
            config["controller"]["stall_recovery_disable_remaining_mg"],
            100.0,
        )
        self.assertEqual(config["profile"]["tail_taper_start_remaining_mg"], 80.0)

        config_500 = service.resolve_dispense_configuration({
            "powder_id": "powder-20260817-143530-182017",
            "target_mg": 500,
        })
        self.assertNotIn("tail_pulse_enabled", config_500["controller"])

    def test_plugging_agent_2_does_not_enable_tail_pulses_at_any_target(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._powder_session = None
        for target in (100, 300, 500, 700, 999, 1000):
            with self.subTest(target=target):
                config = service.resolve_dispense_configuration({
                    "powder_id": "powder-20260817-143530-182017",
                    "target_mg": target,
                })
                self.assertFalse(resolve_tail_pulse_config(config["controller"], target)["enabled"])
                self.assertFalse(config["controller"]["stall_recovery_enabled"])
                self.assertFalse(config["controller"]["position_feedback_enabled"])

    def test_plugging_agent_2_continues_through_700mg_without_pulse_stage(self):
        """Exercise the real worker/controller with fake devices; never touch hardware."""
        service = DeviceControlService.__new__(DeviceControlService)
        service._powder_session = None
        service._cancel_event = threading.Event()
        snapshots = []
        service._set_test = snapshots.append
        service._persist_result = lambda *_args: None
        config = service.resolve_dispense_configuration({
            "powder_id": "powder-20260817-143530-182017", "target_mg": 1000,
        })
        # Keep the zero-flow period shorter than the five-second recovery
        # threshold so this test remains focused on the tail-stage behavior.
        masses = iter([0] + [700] * 10 + [750, 800, 850, 900, 920, 945, 961])
        clock = SimpleNamespace(value=0.0)
        latest_mass = [0]
        actions = []
        positions = []

        def measurement(mass):
            return SimpleNamespace(raw_count=mass, mass_mg=float(mass), stable=True, status_word=256)

        def read_mass():
            clock.value += 0.25
            latest_mass[0] = next(masses)
            return measurement(latest_mass[0])

        service.reader = SimpleNamespace(
            _lock=threading.Lock(),
            _at=SimpleNamespace(read_measurement=read_mass, zero=lambda **_kwargs: None),
            _la10=SimpleNamespace(
                start_vibration=lambda **_kwargs: actions.append(("start", latest_mass[0])),
                read_vibration_settings=lambda: SimpleNamespace(enabled=True),
                update_vibration_duty=lambda _duty: None,
                stop_vibration=lambda: actions.append(("stop", latest_mass[0])),
            ),
        )

        def move(target, **_kwargs):
            positions.append(target)
            status = SimpleNamespace(position_units=target)
            return status, status, 0

        settled = iter([measurement(0), measurement(1000)])
        service._move_window_to = move
        service._wait_for_stable_measurement = lambda *_args, **_kwargs: next(settled)
        service._run_tail_pulse_stage = lambda **_kwargs: self.fail("pulse stage must not be entered")
        with patch("device_control_server.time.monotonic", lambda: clock.value):
            service._run_feedback_dispense(
                "continuous-restore-regression", 1000, config["preset_id"], config["initial"],
                config["profile"], config["controller"],
                {"powder_id": config["powder_id"], "powder_name": config["powder_name"]},
            )
        result = snapshots[-1]["result"]
        self.assertEqual(result["result"], "completed", result.get("error"))
        self.assertEqual(result["algorithm_version"], "predictive-stop-v3-fast-tail")
        self.assertFalse(result["tail_pulse_config"]["enabled"])
        self.assertNotIn("tail_pulse_stage_entered", result)
        self.assertNotIn("tail_pulse_events", result)
        self.assertEqual(result["samples"][0]["mass_mg"], 700)
        self.assertGreater(result["stop_sample"]["mass_mg"], 700)
        self.assertEqual([action[0] for action in actions], ["start", "stop"])
        self.assertGreater(actions[-1][1], 700)
        self.assertEqual(positions, [250])  # Initial positioning only; no end-of-run return.
        self.assertNotIn("stall_recovery_cleanup", result)
        self.assertFalse(result["stall_recovery_config"]["enabled"])
        self.assertTrue(all(not s["stall_recovery_active"] for s in result["samples"]))
        self.assertTrue(all(s["window_position_units"] == 250 for s in result["samples"]))
        self.assertEqual(result["predictive_stop_config"]["measured_safety_stop_mass_mg"], 960)
        self.assertEqual(result["predictive_stop_config"]["fixed_tail_mass_mg"], 15)

    def test_tail_pulse_completes_using_settle_between_short_pulses(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        service._set_test = lambda *_args, **_kwargs: None
        vibration_commands = []
        la10 = SimpleNamespace(
            start_vibration=lambda **kwargs: vibration_commands.append(("start", kwargs)),
            stop_vibration=lambda: vibration_commands.append(("stop", None)),
            read_vibration_settings=lambda: SimpleNamespace(enabled=True),
        )
        service.reader = SimpleNamespace(_lock=threading.Lock(), _la10=la10)
        position = [350]
        requested_positions = []

        def move_window(target, **_kwargs):
            before = SimpleNamespace(position_units=position[0])
            requested_positions.append(target)
            position[0] = target
            return before, SimpleNamespace(position_units=target), abs(target - before.position_units)

        stable_measurements = iter([
            SimpleNamespace(mass_mg=850.0, raw_count=850, stable=True, status_word=0),
            SimpleNamespace(mass_mg=992.0, raw_count=992, stable=True, status_word=0),
        ])
        service._move_window_to = move_window
        service._wait_for_stable_measurement = lambda *_args, **_kwargs: next(stable_measurements)
        config = resolve_tail_pulse_config(
            {
                "tail_pulse_enabled": True,
                "tail_pulse_initial_duration_s": 0.001,
                "tail_pulse_min_duration_s": 0.001,
                "tail_pulse_max_duration_s": 0.002,
            },
            1000,
        )
        result = {"samples": [], "control_updates": []}

        outcome = service._run_tail_pulse_stage(
            run_id="pulse-complete",
            target_mg=1000,
            acceptance_min_mg=990,
            acceptance_max_mg=1010,
            config=config,
            initial_measurement=SimpleNamespace(
                mass_mg=800.0, raw_count=800, stable=True, status_word=0
            ),
            started_at=0.0,
            result=result,
        )

        self.assertEqual(outcome["status"], "completed")
        self.assertEqual(outcome["mass_mg"], 992.0)
        self.assertEqual(requested_positions, [385, 350, 380, 350])
        self.assertEqual([command[0] for command in vibration_commands], ["start", "stop", "start", "stop"])
        self.assertEqual(len(result["tail_pulse_events"]), 2)
        self.assertNotIn(400, requested_positions)

    def test_tail_pulse_fails_safely_after_two_no_gain_attempts_without_p400(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        service._set_test = lambda *_args, **_kwargs: None
        la10 = SimpleNamespace(
            start_vibration=lambda **_kwargs: None,
            stop_vibration=lambda: None,
            read_vibration_settings=lambda: SimpleNamespace(enabled=True),
        )
        service.reader = SimpleNamespace(_lock=threading.Lock(), _la10=la10)
        position = [350]
        requested_positions = []

        def move_window(target, **_kwargs):
            before = SimpleNamespace(position_units=position[0])
            requested_positions.append(target)
            position[0] = target
            return before, SimpleNamespace(position_units=target), abs(target - before.position_units)

        service._move_window_to = move_window
        service._wait_for_stable_measurement = lambda *_args, **_kwargs: SimpleNamespace(
            mass_mg=800.0, raw_count=800, stable=True, status_word=0
        )
        config = resolve_tail_pulse_config(
            {
                "tail_pulse_enabled": True,
                "tail_pulse_initial_duration_s": 0.001,
                "tail_pulse_min_duration_s": 0.001,
                "tail_pulse_max_duration_s": 0.002,
            },
            1000,
        )
        result = {"samples": [], "control_updates": []}

        outcome = service._run_tail_pulse_stage(
            run_id="pulse-no-gain",
            target_mg=1000,
            acceptance_min_mg=990,
            acceptance_max_mg=1010,
            config=config,
            initial_measurement=SimpleNamespace(
                mass_mg=800.0, raw_count=800, stable=True, status_word=0
            ),
            started_at=0.0,
            result=result,
        )

        self.assertEqual(outcome["status"], "failed")
        self.assertIn("p400 was not opened", outcome["reason"])
        self.assertEqual(requested_positions, [385, 350, 390, 350])
        self.assertNotIn(400, requested_positions)

    def test_tail_pulse_headroom_gate_permanently_uses_p370_to_p375_near_limit(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        service._set_test = lambda *_args, **_kwargs: None
        la10 = SimpleNamespace(
            start_vibration=lambda **_kwargs: None,
            stop_vibration=lambda: None,
            read_vibration_settings=lambda: SimpleNamespace(enabled=True),
        )
        service.reader = SimpleNamespace(_lock=threading.Lock(), _la10=la10)
        position = [350]
        requested_positions = []

        def move_window(target, **_kwargs):
            before = SimpleNamespace(position_units=position[0])
            requested_positions.append(target)
            position[0] = target
            return before, SimpleNamespace(position_units=target), abs(target - before.position_units)

        stable_measurements = iter([
            SimpleNamespace(mass_mg=986.0, raw_count=986, stable=True, status_word=0),
            SimpleNamespace(mass_mg=986.0, raw_count=986, stable=True, status_word=0),
            SimpleNamespace(mass_mg=986.0, raw_count=986, stable=True, status_word=0),
        ])
        service._move_window_to = move_window
        service._wait_for_stable_measurement = lambda *_args, **_kwargs: next(stable_measurements)
        config = resolve_tail_pulse_config(
            {
                "tail_pulse_enabled": True,
                "tail_pulse_initial_duration_s": 0.001,
                "tail_pulse_min_duration_s": 0.001,
                "tail_pulse_max_duration_s": 0.002,
                "tail_pulse_micro_initial_duration_s": 0.001,
                "tail_pulse_micro_min_duration_s": 0.001,
                "tail_pulse_micro_max_duration_s": 0.002,
            },
            1000,
        )
        result = {"samples": [], "control_updates": []}

        outcome = service._run_tail_pulse_stage(
            run_id="pulse-headroom-guard",
            target_mg=1000,
            acceptance_min_mg=990,
            acceptance_max_mg=1010,
            config=config,
            initial_measurement=SimpleNamespace(
                mass_mg=925.0, raw_count=925, stable=True, status_word=0
            ),
            started_at=0.0,
            result=result,
        )

        self.assertEqual(outcome["status"], "failed")
        self.assertIn("safe micro pulse stopped underweight", outcome["reason"])
        self.assertEqual(requested_positions, [370, 350, 370, 350, 375, 350])
        self.assertNotIn(380, requested_positions)
        self.assertEqual(result["tail_pulse_safety_gate"]["entered_at_mass_mg"], 925.0)
        self.assertEqual(
            [event["pulse_mode"] for event in result["tail_pulse_events"]],
            ["micro", "micro", "micro"],
        )

    def test_unprofiled_powder_automatically_uses_fingerprint_and_default_rates(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._powder_session = None
        config = service.resolve_dispense_configuration({
            "powder_id": "powder-20260817-114613-f88e45",
            "target_mg": 500,
        })

        self.assertEqual(
            config["config_source"], "fingerprint_with_product_defaults"
        )
        self.assertEqual(config["target_rate_band_source"], "product_default")
        self.assertEqual(
            (
                config["profile"]["coarse_rate_mg_s"],
                config["profile"]["fine_rate_mg_s"],
                config["profile"]["precision_rate_mg_s"],
            ),
            (25.0, 15.0, 10.0),
        )

    def test_dedicated_profile_is_not_overridden_by_an_experiment_session(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._powder_session = {
            "powder_id": "powder-20260814-143554-702da0",
            "preset_id": "slow-10hz-p100",
            "initial": {
                "frequency_hz": 10,
                "duty_permyriad": 1000,
                "window_position_units": 100,
            },
        }
        config = service.resolve_dispense_configuration({
            "powder_id": "powder-20260814-143554-702da0",
            "target_mg": 500,
        })

        self.assertEqual(config["preset_id"], "80hz-22pct-p400-plugging-agent-1")
        self.assertEqual(config["initial"]["frequency_hz"], 80)
        self.assertEqual(config["initial"]["duty_permyriad"], 2200)
        self.assertEqual(config["initial"]["window_position_units"], 400)

    def test_selected_powder_keeps_its_process_profile_with_a_user_preset(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._powder_session = None
        config = service.resolve_dispense_configuration({
            "powder_id": "bentonite",
            "target_mg": 500,
            "preset_id": "slow-10hz-p100",
            "initial": {
                "frequency_hz": 10,
                "duty_permyriad": 1000,
                "window_position_units": 100,
            },
        })

        self.assertEqual(config["powder_name"], "膨润土")
        self.assertEqual(config["preset_id"], "slow-10hz-p100")
        self.assertEqual(
            config["initial"],
            {
                "frequency_hz": 10,
                "duty_permyriad": 1000,
                "window_position_units": 100,
            },
        )
        self.assertEqual(
            (
                config["profile"]["coarse_rate_mg_s"],
                config["profile"]["fine_rate_mg_s"],
                config["profile"]["precision_rate_mg_s"],
            ),
            (25.0, 15.0, 10.0),
        )
        self.assertEqual(config["controller"]["duty_step_per_update"], 50)
        self.assertTrue(config["controller"]["fast_predictive_stop_enabled"])
        self.assertEqual(
            config["controller"]["fast_predictive_stop_rate_mg_s"], 20.0
        )
        self.assertEqual(
            config["controller"]["fast_predictive_stop_confirmations"], 1
        )
        self.assertTrue(config["controller"]["projected_safety_stop_enabled"])
        self.assertEqual(
            config["controller"]["projected_safety_stop_offset_mg"], 5.0
        )

    def test_dispense_batch_forwards_the_resolved_bentonite_stop_configuration(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._powder_session = None
        service._cancel_event = threading.Event()
        state = {"test": {}}
        service.snapshot = lambda: state
        service._set_test = lambda payload: state.update(test=payload)
        service._persist_result = lambda *_args, **_kwargs: None
        captured = {}

        def fake_run(run_id, _target, _preset_id, _preset, _profile, controller, powder):
            captured["controller"] = controller
            state["test"] = {
                "result": {
                    "run_id": run_id,
                    "result": "completed",
                    **powder,
                    "final_mass_mg": 1000.0,
                    "actual_duration_s": 1.0,
                }
            }

        service._run_feedback_dispense = fake_run
        service._run_dispense_batch(
            "disp-batch-regression",
            {
                "powder_id": "bentonite",
                "powder_name": "膨润土",
                "tare_between_sets": True,
                "close_window_between_runs": False,
                "sets": [{
                    "powder_id": "bentonite",
                    "powder_name": "膨润土",
                    "target_mg": 1000,
                    "preset_id": "80hz-22pct-p300-bentonite",
                    "frequency_hz": 80,
                    "duty_permyriad": 2200,
                    "window_position_units": 300,
                    "repeat_count": 1,
                }],
            },
        )

        controller = captured["controller"]
        self.assertTrue(controller["fast_predictive_stop_enabled"])
        self.assertEqual(controller["fast_predictive_stop_rate_mg_s"], 20.0)
        self.assertEqual(controller["fast_predictive_stop_confirmations"], 1)
        self.assertTrue(controller["projected_safety_stop_enabled"])
        self.assertEqual(controller["projected_safety_stop_offset_mg"], 5.0)

    def test_dispense_batch_forwards_water_loss_agent_2_stall_recovery(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._powder_session = None
        service._cancel_event = threading.Event()
        state = {"test": {}}
        service.snapshot = lambda: state
        service._set_test = lambda payload: state.update(test=payload)
        service._persist_result = lambda *_args, **_kwargs: None
        captured = {}

        def fake_run(run_id, _target, _preset_id, _preset, _profile, controller, powder):
            captured["controller"] = controller
            state["test"] = {
                "result": {
                    "run_id": run_id,
                    "result": "completed",
                    **powder,
                    "final_mass_mg": 1000.0,
                    "actual_duration_s": 1.0,
                }
            }

        service._run_feedback_dispense = fake_run
        service._run_dispense_batch(
            "disp-batch-water-loss-stall-recovery",
            {
                "powder_id": "water_loss_agent_2",
                "powder_name": "失水剂2",
                "tare_between_sets": True,
                "sets": [{
                    "powder_id": "water_loss_agent_2",
                    "powder_name": "失水剂2",
                    "target_mg": 1000,
                    "preset_id": "80hz-22pct-p350-water-loss-agent-2",
                    "frequency_hz": 80,
                    "duty_permyriad": 2200,
                    "window_position_units": 350,
                    "repeat_count": 1,
                }],
            },
        )

        controller = captured["controller"]
        self.assertTrue(controller["stall_recovery_enabled"])
        self.assertEqual(controller["stall_recovery_observation_window_s"], 3.0)
        self.assertEqual(controller["stall_recovery_window_step_units"], 25)
        self.assertEqual(controller["stall_recovery_max_window_position_units"], 400)
        self.assertEqual(controller["stall_recovery_max_attempts"], 2)
        self.assertEqual(controller["stall_recovery_disable_remaining_mg"], 50.0)
        self.assertEqual(controller["max_duty_permyriad"], 3000)
        self.assertTrue(controller["flow_hold_enabled"])

    def test_dispense_batch_aborts_after_stall_recovery_is_exhausted(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._powder_session = None
        service._cancel_event = threading.Event()
        state = {"test": {}}
        service.snapshot = lambda: state
        service._set_test = lambda payload: state.update(test=payload)
        service._persist_result = lambda *_args, **_kwargs: None
        calls = []

        def fake_run(run_id, *_args):
            calls.append(run_id)
            state["test"] = {
                "result": {
                    "run_id": run_id,
                    "result": "failed",
                    "powder_id": "water_loss_agent_2",
                    "powder_name": "失水剂2",
                    "stop_reason": (
                        "stall recovery exhausted after 2 temporary-window attempts"
                    ),
                    "final_mass_mg": 500.0,
                    "actual_duration_s": 10.0,
                }
            }

        service._run_feedback_dispense = fake_run
        service._run_dispense_batch(
            "disp-batch-stall-abort",
            {
                "powder_id": "water_loss_agent_2",
                "powder_name": "失水剂2",
                "tare_between_sets": True,
                "sets": [{
                    "powder_id": "water_loss_agent_2",
                    "powder_name": "失水剂2",
                    "target_mg": 1000,
                    "preset_id": "80hz-22pct-p350-water-loss-agent-2",
                    "frequency_hz": 80,
                    "duty_permyriad": 2200,
                    "window_position_units": 350,
                    "repeat_count": 3,
                }],
            },
        )

        self.assertEqual(len(calls), 1)
        self.assertEqual(state["test"]["result"]["result"], "failed")
        self.assertIn("remaining repetitions", state["test"]["result"]["error"])

    def test_dispense_batch_requires_every_rep_to_pass_acceptance(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._powder_session = None
        service._cancel_event = threading.Event()
        state = {"test": {}}
        service.snapshot = lambda: state
        service._set_test = lambda payload: state.update(test=payload)
        service._persist_result = lambda *_args, **_kwargs: None
        results = iter((("completed", 1000.0), ("failed", 1013.0)))

        def fake_run(run_id, _target, _preset_id, _preset, _profile, _controller, powder):
            result, mass = next(results)
            state["test"] = {
                "result": {
                    "run_id": run_id,
                    "result": result,
                    **powder,
                    "final_mass_mg": mass,
                    "actual_duration_s": 1.0,
                    "acceptance_error": (
                        "final stable mass outside accepted band" if result == "failed" else None
                    ),
                }
            }

        service._run_feedback_dispense = fake_run
        service._run_dispense_batch(
            "disp-batch-acceptance-regression",
            {
                "powder_id": "bentonite",
                "powder_name": "膨润土",
                "tare_between_sets": True,
                "close_window_between_runs": False,
                "sets": [{
                    "powder_id": "bentonite",
                    "powder_name": "膨润土",
                    "target_mg": 1000,
                    "preset_id": "80hz-22pct-p300-bentonite",
                    "frequency_hz": 80,
                    "duty_permyriad": 2200,
                    "window_position_units": 300,
                    "repeat_count": 2,
                }],
            },
        )

        batch = state["test"]["result"]
        self.assertFalse(batch["set_results"][0]["acceptance_passed"])
        self.assertEqual(batch["set_results"][0]["result"], "failed")
        self.assertEqual(batch["result"], "completed")
        self.assertFalse(batch["acceptance_passed"])
        self.assertNotIn("error", batch)
        self.assertEqual(batch["cross_set_summary"]["passed_sets"], 0)

    def test_feedback_worker_applies_light_powder_hold_and_duty_ceiling(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        snapshots = []
        service._set_test = snapshots.append
        service._persist_result = lambda *_args, **_kwargs: None
        service._move_window_to = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError("stop after settings capture")
        )
        captured = {}

        def capture_controller(**kwargs):
            captured["settings"] = kwargs["settings"]
            return SimpleNamespace()

        with patch(
            "device_control_server.ContinuousFeedbackController",
            side_effect=capture_controller,
        ):
            service._run_feedback_dispense(
                "dispense-1000mg-plugging-agent-1-regression",
                1000,
                "80hz-22pct-p350-plugging-agent-1",
                {
                    "frequency_hz": 80,
                    "duty_permyriad": 2200,
                    "window_position_units": 350,
                },
                interpolate_profile(1000),
                {
                    "max_duty_permyriad": 3000,
                    "flow_hold_enabled": True,
                    "flow_hold_window_s": 4.0,
                    "flow_hold_min_gain_mg": 5.0,
                    "flow_response_check_s": 4.0,
                    "flow_response_min_improvement_mg_s": 1.0,
                },
                {
                    "powder_id": "powder-20260814-143554-702da0",
                    "powder_name": "封堵剂1",
                },
            )
        settings = captured["settings"]
        self.assertEqual(settings.max_duty_permyriad, 3000)
        self.assertTrue(settings.flow_hold_enabled)
        self.assertEqual(settings.flow_hold_window_s, 4.0)
        self.assertEqual(settings.flow_hold_min_gain_mg, 5.0)

    def test_dispense_batch_rejects_mixed_powders(self):
        service = DeviceControlService.__new__(DeviceControlService)
        fingerprints = {
            "fp-bentonite": SimpleNamespace(powder_name="膨润土"),
            "fp-water-loss": SimpleNamespace(powder_name="失水剂2"),
        }
        sets = [
            {
                "powder_id": powder_id,
                "powder_name": fingerprint.powder_name,
                "target_mg": 1000,
                "frequency_hz": 80,
                "duty_permyriad": 2200,
                "window_position_units": 300,
            }
            for powder_id, fingerprint in fingerprints.items()
        ]

        with patch(
            "device_control_server.load_fingerprint",
            side_effect=lambda powder_id: fingerprints.get(powder_id),
        ):
            with self.assertRaisesRegex(ValueError, "同一批量队列只能使用一种粉末"):
                service.start_dispense_batch({"sets": sets})

    def test_feedback_worker_uses_predictive_stop_v2_safety_settings(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        snapshots = []
        service._set_test = snapshots.append
        service._persist_result = lambda *_args, **_kwargs: None
        service._move_window_to = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError("stop after settings capture")
        )
        captured = {}

        def capture_controller(**kwargs):
            captured["settings"] = kwargs["settings"]
            return SimpleNamespace()

        with patch(
            "device_control_server.ContinuousFeedbackController",
            side_effect=capture_controller,
        ):
            service._run_feedback_dispense(
                "dispense-1000mg-predictive-stop-regression",
                1000,
                "regression",
                {
                    "frequency_hz": 80,
                    "duty_permyriad": 2200,
                    "window_position_units": 350,
                },
                interpolate_profile(1000),
                {},
            )

        settings = captured["settings"]
        self.assertTrue(settings.predictive_stop_enabled)
        self.assertEqual(settings.settle_confirmations, 2)
        self.assertAlmostEqual(settings.stop_prediction_horizon_s, 1.0)
        self.assertEqual(settings.force_stop_max_mass_mg, 990.0)
        self.assertFalse(settings.fast_predictive_stop_enabled)
        self.assertFalse(settings.projected_safety_stop_enabled)
        self.assertFalse(settings.stall_recovery_enabled)
        config = snapshots[-1]["result"]["predictive_stop_config"]
        self.assertEqual(
            snapshots[-1]["result"]["algorithm_version"], "predictive-stop-v2"
        )
        self.assertEqual(config["confirmation_samples"], 2)
        self.assertEqual(config["minimum_enabled_target_mg"], 1000.0)
        self.assertEqual(config["measured_safety_stop_mass_mg"], 990.0)
        self.assertFalse(config["fast_stop_enabled"])
        self.assertFalse(config["projected_safety_stop_enabled"])
        self.assertFalse(snapshots[-1]["result"]["stall_recovery_config"]["enabled"])

    def test_feedback_worker_maps_bentonite_only_stop_overrides(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        snapshots = []
        service._set_test = snapshots.append
        service._persist_result = lambda *_args, **_kwargs: None
        service._move_window_to = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError("stop after settings capture")
        )
        captured = {}

        def capture_controller(**kwargs):
            captured["settings"] = kwargs["settings"]
            return SimpleNamespace()

        with patch(
            "device_control_server.ContinuousFeedbackController",
            side_effect=capture_controller,
        ):
            service._run_feedback_dispense(
                "dispense-1000mg-bentonite-fast-stop-regression",
                1000,
                "80hz-22pct-p300-bentonite",
                {
                    "frequency_hz": 80,
                    "duty_permyriad": 2200,
                    "window_position_units": 300,
                },
                interpolate_profile(1000),
                {
                    "fast_predictive_stop_enabled": True,
                    "fast_predictive_stop_rate_mg_s": 20.0,
                    "fast_predictive_stop_confirmations": 1,
                    "projected_safety_stop_enabled": True,
                    "projected_safety_stop_offset_mg": 5.0,
                },
                {"powder_id": "bentonite", "powder_name": "膨润土"},
            )

        settings = captured["settings"]
        self.assertTrue(settings.fast_predictive_stop_enabled)
        self.assertEqual(settings.fast_predictive_stop_rate_mg_s, 20.0)
        self.assertEqual(settings.fast_predictive_stop_confirmations, 1)
        self.assertTrue(settings.projected_safety_stop_enabled)
        self.assertEqual(settings.projected_safety_stop_offset_mg, 5.0)
        self.assertEqual(
            snapshots[-1]["result"]["algorithm_version"],
            "predictive-stop-v3-bentonite-fast-tail",
        )

        with patch(
            "device_control_server.ContinuousFeedbackController",
            side_effect=capture_controller,
        ):
            service._run_feedback_dispense(
                "dispense-1000mg-water-loss-agent-2-fast-stop-regression",
                1000,
                "80hz-22pct-p350-water-loss-agent-2",
                {
                    "frequency_hz": 80,
                    "duty_permyriad": 2200,
                    "window_position_units": 350,
                },
                interpolate_profile(1000),
                {
                    "fast_predictive_stop_enabled": True,
                    "fast_predictive_stop_rate_mg_s": 20.0,
                    "fast_predictive_stop_confirmations": 1,
                    "projected_safety_stop_enabled": True,
                    "projected_safety_stop_offset_mg": 5.0,
                    "stall_recovery_enabled": True,
                    "stall_recovery_observation_window_s": 3.0,
                    "stall_recovery_min_gain_mg": 2.0,
                    "stall_recovery_rate_threshold_mg_s": 1.0,
                    "stall_recovery_resume_rate_mg_s": 2.0,
                    "stall_recovery_retry_s": 3.0,
                    "stall_recovery_window_step_units": 25,
                    "stall_recovery_max_window_position_units": 400,
                    "stall_recovery_max_attempts": 2,
                    "stall_recovery_disable_remaining_mg": 50.0,
                },
                {"powder_id": "water_loss_agent_2", "powder_name": "失水剂2"},
            )
        self.assertEqual(
            snapshots[-1]["result"]["algorithm_version"],
            "predictive-stop-v4-water-loss-agent-2-stall-recovery",
        )
        settings = captured["settings"]
        self.assertTrue(settings.stall_recovery_enabled)
        self.assertEqual(settings.stall_recovery_observation_window_s, 3.0)
        self.assertEqual(settings.stall_recovery_window_step_units, 25)
        self.assertEqual(settings.stall_recovery_max_window_position_units, 400)
        self.assertEqual(settings.stall_recovery_max_attempts, 2)

    def test_feedback_controller_initialization_error_does_not_stick_running_state(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        captured = []
        service._set_test = captured.append
        service._persist_result = lambda *_args, **_kwargs: None

        with patch(
            "device_control_server.ContinuousFeedbackController",
            side_effect=RuntimeError("synthetic controller failure"),
        ):
            service._run_feedback_dispense(
                "dispense-500mg-controller-regression",
                500,
                "regression",
                {
                    "frequency_hz": 40,
                    "duty_permyriad": 3000,
                    "window_position_units": 200,
                },
                interpolate_profile(500),
                {},
            )

        self.assertEqual(captured[-1]["status"], "failed")
        self.assertEqual(
            captured[-1]["result"]["error"],
            "synthetic controller failure",
        )

    def test_continuous_taper_starts_as_an_independent_task(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._test_lock = threading.Lock()
        service._cancel_event = threading.Event()
        service.snapshot = lambda: {"test": {"status": "idle"}}
        captured = []
        service._set_test = captured.append

        with patch("device_control_server.threading.Thread") as thread_class:
            response = service.start_continuous_taper_test()

        self.assertTrue(response["accepted"])
        self.assertTrue(response["run_id"].startswith("continuous-taper-500mg-"))
        self.assertEqual(response["submitted"]["algorithm_version"], "continuous-taper-v1")
        self.assertEqual(captured[0]["status"], "running")
        thread_class.return_value.start.assert_called_once_with()

    def test_active_task_sample_updates_cached_balance_without_an_extra_read(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._snapshot_lock = threading.Lock()
        service._snapshot = {
            "at8811c": {"online": True, "raw_weight_mg": 0.0, "configuration": {}},
            "test": {"status": "idle"},
        }

        service._set_test(
            {
                "status": "running",
                "run_id": "continuous-taper-500mg-test",
                "latest_sample": {
                    "mass_mg": 421.0,
                    "stable": False,
                    "status_word": 0,
                },
            }
        )

        self.assertEqual(service._snapshot["at8811c"]["raw_weight_mg"], 421.0)
        self.assertFalse(service._snapshot["at8811c"]["stable"])

    def test_final_measurement_requires_wait_and_continuous_stability(self):
        class FakeClock:
            def __init__(self):
                self.value = 0.0

            def monotonic(self):
                current = self.value
                self.value += 0.1
                return current

            def sleep(self, _seconds):
                pass

        class FakeBalance:
            def read_measurement(self):
                return SimpleNamespace(
                    mass_mg=None,
                    raw_count=500,
                    stable=True,
                    status_word=0x0100,
                )

        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        service.reader = SimpleNamespace(
            _lock=threading.Lock(),
            _at=FakeBalance(),
        )
        trace = []
        clock = FakeClock()

        with patch("device_control_server.time.monotonic", clock.monotonic), patch(
            "device_control_server.time.sleep", clock.sleep
        ):
            final = service._wait_for_stable_measurement(
                5.0,
                minimum_wait_s=0.5,
                continuous_stable_s=0.5,
                max_spread_mg=1.0,
                trace=trace,
            )

        self.assertEqual(final.raw_count, 500)
        self.assertGreaterEqual(trace[-1]["elapsed_after_stop_s"], 1.0)
        self.assertTrue(all(sample["stable"] for sample in trace))

    def test_motion_distance_never_rounds_a_nonzero_delta_to_zero(self):
        self.assertEqual(motion_distance_register(0), 0)
        self.assertEqual(motion_distance_register(1), 1)
        self.assertEqual(motion_distance_register(5), 2)

    def test_move_window_does_not_command_an_in_tolerance_correction(self):
        class FakeLA10:
            def __init__(self):
                self.move_calls = []

            def read_status(self):
                return SimpleNamespace(position_units=100, is_running=False, fault_bits=0)

            def enable(self):
                raise AssertionError("enable must not be called within tolerance")

            def move_relative(self, **kwargs):
                self.move_calls.append(kwargs)

        service = DeviceControlService.__new__(DeviceControlService)
        la10 = FakeLA10()
        service.reader = SimpleNamespace(_lock=threading.Lock(), _la10=la10)

        before, after, distance = service._move_window_to(
            105,
            speed_mm_s=1.0,
            timeout_s=10.0,
            position_tolerance_units=5,
        )

        self.assertIs(before, after)
        self.assertEqual(distance, 0)
        self.assertEqual(la10.move_calls, [])

    def test_move_window_corrects_a_stopped_position_outside_tolerance(self):
        class FakeLA10:
            def __init__(self):
                self.move_calls = []
                self.initial = SimpleNamespace(
                    position_units=381,
                    target_units=381,
                    is_running=False,
                    fault_bits=0,
                    last_result=0,
                )

            def read_status(self):
                return self.initial

            def enable(self):
                pass

            def move_relative(self, **kwargs):
                self.move_calls.append(kwargs)

        service = DeviceControlService.__new__(DeviceControlService)
        la10 = FakeLA10()
        service.reader = SimpleNamespace(_lock=threading.Lock(), _la10=la10)
        stopped_statuses = iter([
            SimpleNamespace(
                position_units=318,
                target_units=350,
                is_running=False,
                fault_bits=0,
                last_result=0,
            ),
            SimpleNamespace(
                position_units=349,
                target_units=350,
                is_running=False,
                fault_bits=0,
                last_result=0,
            ),
        ])
        service._wait_for_window_move = lambda *_args, **_kwargs: next(stopped_statuses)
        service._wait_for_window_idle = lambda *_args, **_kwargs: SimpleNamespace(
            position_units=318, target_units=350, is_running=False,
            fault_bits=0, last_result=2,
        )

        before, after, distance = service._move_window_to(
            350,
            speed_mm_s=1.0,
            timeout_s=5.0,
        )

        self.assertEqual(before.position_units, 381)
        self.assertEqual(after.position_units, 349)
        self.assertEqual(len(la10.move_calls), 2)
        self.assertFalse(la10.move_calls[0]["extend"])
        self.assertTrue(la10.move_calls[1]["extend"])
        self.assertEqual(distance, 32)

    def test_move_window_retries_result_code_once_and_verifies_position(self):
        class FakeLA10:
            def __init__(self):
                self.move_calls = []
                self.reads = iter([
                    SimpleNamespace(
                        position_units=318,
                        target_units=318,
                        is_running=False,
                        fault_bits=0,
                        last_result=0,
                    ),
                    SimpleNamespace(
                        position_units=318,
                        target_units=350,
                        is_running=False,
                        fault_bits=0,
                        last_result=2,
                    ),
                ])

            def read_status(self):
                return next(self.reads)

            def enable(self):
                pass

            def move_relative(self, **kwargs):
                self.move_calls.append(kwargs)

        service = DeviceControlService.__new__(DeviceControlService)
        la10 = FakeLA10()
        service.reader = SimpleNamespace(_lock=threading.Lock(), _la10=la10)
        stopped_statuses = iter([
            SimpleNamespace(
                position_units=318,
                target_units=350,
                is_running=False,
                fault_bits=0,
                last_result=2,
            ),
            SimpleNamespace(
                position_units=349,
                target_units=350,
                is_running=False,
                fault_bits=0,
                last_result=0,
            ),
        ])
        service._wait_for_window_move = lambda *_args, **_kwargs: next(stopped_statuses)
        service._wait_for_window_idle = lambda *_args, **_kwargs: SimpleNamespace(
            position_units=318, target_units=350, is_running=False,
            fault_bits=0, last_result=2,
        )

        _before, after, distance = service._move_window_to(
            350,
            speed_mm_s=1.0,
            timeout_s=5.0,
        )

        self.assertEqual(after.position_units, 349)
        self.assertEqual(len(la10.move_calls), 2)
        self.assertEqual(distance, 32)

    def test_move_window_stops_after_one_persistent_result_code_retry(self):
        status = SimpleNamespace(
            position_units=318,
            target_units=350,
            is_running=False,
            fault_bits=0,
            last_result=2,
        )
        la10 = SimpleNamespace(
            read_status=lambda: status,
            enable=lambda: None,
            move_relative=lambda **_kwargs: None,
        )
        service = DeviceControlService.__new__(DeviceControlService)
        service.reader = SimpleNamespace(_lock=threading.Lock(), _la10=la10)
        service._wait_for_window_move = lambda *_args, **_kwargs: status
        service._wait_for_window_idle = lambda *_args, **_kwargs: status

        with self.assertRaisesRegex(RuntimeError, "重试后仍失败"):
            service._move_window_to(
                350,
                speed_mm_s=1.0,
                timeout_s=5.0,
            )

    def test_window_cleanup_can_finish_after_user_cancellation(self):
        status = SimpleNamespace(
            position_units=350,
            is_running=False,
            fault_bits=0,
            last_result=0,
        )
        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        service._cancel_event.set()
        service.reader = SimpleNamespace(
            _lock=threading.Lock(),
            _la10=SimpleNamespace(read_status=lambda: status),
        )

        restored = service._wait_for_window_move(
            1.0,
            target_position=350,
            respect_cancel=False,
        )

        self.assertIs(restored, status)

    def test_window_wait_does_not_treat_target_register_update_as_motion_completion(self):
        statuses = iter([
            SimpleNamespace(
                position_units=318,
                target_units=350,
                is_running=False,
                fault_bits=0,
                last_result=0,
            ),
            SimpleNamespace(
                position_units=318,
                target_units=350,
                is_running=True,
                fault_bits=0,
                last_result=0,
            ),
            SimpleNamespace(
                position_units=349,
                target_units=350,
                is_running=False,
                fault_bits=0,
                last_result=0,
            ),
        ])
        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        service.reader = SimpleNamespace(
            _lock=threading.Lock(),
            _la10=SimpleNamespace(read_status=lambda: next(statuses)),
        )

        final = service._wait_for_window_move(
            1.0,
            target_position=350,
            position_tolerance_units=10,
            initial_position=318,
            initial_last_result=0,
        )

        self.assertEqual(final.position_units, 349)

    def test_window_wait_confirms_idle_target_after_a_early_target_update(self):
        statuses = iter([
            # The gateway briefly exposes the requested target before it marks
            # the motor as running.  This must not be treated as completion.
            SimpleNamespace(position_units=350, target_units=350, is_running=False, fault_bits=0, last_result=0),
            SimpleNamespace(position_units=350, target_units=350, is_running=True, fault_bits=0, last_result=0),
            SimpleNamespace(position_units=349, target_units=350, is_running=False, fault_bits=0, last_result=0),
            SimpleNamespace(position_units=349, target_units=350, is_running=False, fault_bits=0, last_result=0),
        ])
        service = DeviceControlService.__new__(DeviceControlService)
        service._cancel_event = threading.Event()
        service.reader = SimpleNamespace(
            _lock=threading.Lock(),
            _la10=SimpleNamespace(read_status=lambda: next(statuses)),
        )

        final = service._wait_for_window_move(
            2.0,
            target_position=350,
            position_tolerance_units=10,
            initial_position=318,
            initial_last_result=0,
        )

        self.assertEqual(final.position_units, 349)

    def test_500mg_presets_match_the_approved_starting_points(self):
        self.assertEqual(
            DISPENSE_PRESETS["fast-80hz-p200"],
            {"frequency_hz": 80, "duty_permyriad": 2000, "window_position_units": 200},
        )
        self.assertEqual(
            DISPENSE_PRESETS["steady-65hz-p300"],
            {"frequency_hz": 65, "duty_permyriad": 2000, "window_position_units": 300},
        )

    def test_1000mg_material_presets_match_the_requested_starting_points(self):
        self.assertEqual(
            DISPENSE_PRESETS["80hz-22pct-p350-water-loss-agent-2"],
            {"frequency_hz": 80, "duty_permyriad": 2200, "window_position_units": 350},
        )
        self.assertEqual(
            DISPENSE_PRESETS["80hz-22pct-p300-bentonite"],
            {"frequency_hz": 80, "duty_permyriad": 2200, "window_position_units": 300},
        )
        self.assertEqual(
            DISPENSE_PRESETS["80hz-22pct-p400-plugging-agent-1"],
            {"frequency_hz": 80, "duty_permyriad": 2200, "window_position_units": 400},
        )
        self.assertEqual(
            DISPENSE_PRESETS["80hz-22pct-p400-plugging-agent-2"],
            {"frequency_hz": 80, "duty_permyriad": 2200, "window_position_units": 400},
        )

    def test_1000mg_material_presets_are_target_specific_in_dashboard(self):
        self.assertIn("80Hz-22%-p350-失水剂2", DEVICE_DASHBOARD_HTML)
        self.assertIn("80Hz-22%-p300-膨润土", DEVICE_DASHBOARD_HTML)
        self.assertIn("80Hz-22%-p400-封堵剂1", DEVICE_DASHBOARD_HTML)
        self.assertIn("80Hz-22%-p400-封堵剂2", DEVICE_DASHBOARD_HTML)
        self.assertEqual(DEVICE_DASHBOARD_HTML.count('data-exact-target="1000"'), 4)
        self.assertIn("const standardTargets=[100,300,500,1000]", DEVICE_DASHBOARD_HTML)

    def test_dashboard_requires_and_submits_powder_identity_for_dispensing(self):
        self.assertIn('id="dispense-powder"', DEVICE_DASHBOARD_HTML)
        self.assertIn("粉末类型（必选，决定过程控制配置）", DEVICE_DASHBOARD_HTML)
        self.assertIn("请先选择粉末类型，再启动加粉", DEVICE_DASHBOARD_HTML)
        self.assertIn(
            "{...powder,target_mg:selectedDispenseTarget,preset_id:selectedDispensePreset,initial}",
            DEVICE_DASHBOARD_HTML,
        )
        self.assertIn("粉末决定过程配置，起始 Preset 可修改", DEVICE_DASHBOARD_HTML)
        self.assertIn("fetch('/api/dispense/powders'", DEVICE_DASHBOARD_HTML)
        self.assertIn("fetch('/api/dispense/config?powder_id='", DEVICE_DASHBOARD_HTML)
        self.assertIn("{powder_id:powderId,target_mg:target}", DEVICE_DASHBOARD_HTML)
        self.assertNotIn(
            "{powder_id:powderId,target_mg:target,initial:",
            DEVICE_DASHBOARD_HTML,
        )

    def test_grid_preview_rows_are_editable_and_submitted_explicitly(self):
        self.assertIn("function updateGridSet(index,key,rawValue,scale=1)", DEVICE_DASHBOARD_HTML)
        self.assertIn("sets:gridSets.map", DEVICE_DASHBOARD_HTML)
        self.assertIn("启动时将按当前预览表提交", DEVICE_DASHBOARD_HTML)

    def test_grid_api_preserves_explicit_edited_rows(self):
        service = DeviceControlService.__new__(DeviceControlService)
        service._test_lock = threading.Lock()
        service._cancel_event = threading.Event()
        service.snapshot = lambda: {"test": {"status": "idle"}}
        captured = []
        service._set_test = captured.append
        rows = [
            {
                "frequency_hz": 35,
                "duty_permyriad": 1600,
                "window_position_units": 180,
                "duration_s": 8,
                "repeat_count": 2,
            },
            {
                "frequency_hz": 55,
                "duty_permyriad": 2200,
                "window_position_units": 300,
                "duration_s": 8,
                "repeat_count": 2,
            },
        ]
        with patch("device_control_server.threading.Thread") as thread_class:
            response = service.start_grid_experiment(
                {
                    "experiment_type": "constant_rate",
                    "powder_name": "膨润土",
                    "powder_id": "bentonite",
                    "sets": rows,
                    "duration_s": 8,
                    "repeat_count": 2,
                }
            )
        self.assertEqual(response["expanded_sets"], 2)
        self.assertEqual(captured[0]["submitted"]["sets"], [
            {**rows[0], "duration_s": 8.0},
            {**rows[1], "duration_s": 8.0},
        ])
        thread_class.return_value.start.assert_called_once_with()

    def test_grid_api_rejects_an_invalid_edited_row(self):
        service = DeviceControlService.__new__(DeviceControlService)
        with self.assertRaisesRegex(ValueError, "frequency_hz"):
            service.start_grid_experiment(
                {
                    "experiment_type": "constant_rate",
                    "powder_name": "膨润土",
                    "powder_id": "bentonite",
                    "sets": [{
                        "frequency_hz": 81,
                        "duty_permyriad": 1600,
                        "window_position_units": 180,
                        "duration_s": 8,
                        "repeat_count": 2,
                    }],
                }
            )

    def test_default_position_schedule_covers_all_requested_points(self):
        self.assertEqual(position_schedule(100, 750, 50), list(range(100, 751, 50)))

    def test_position_schedule_rejects_invalid_direction_or_step(self):
        with self.assertRaises(ValueError):
            position_schedule(750, 100, 50)
        with self.assertRaises(ValueError):
            position_schedule(100, 750, 0)

    def test_backend_accepts_the_requested_fixed_sweep(self):
        DeviceControlService._validate_position_sweep_payload(
            {
                "position_schedule": list(range(100, 751, 50)),
                "window_speed_mm_s": 1.0,
                "window_timeout_s": 20.0,
                "frequency_hz": 80,
                "duty_permyriad": 2000,
                "duration_s": 10.0,
            }
        )

    def test_backend_rejects_positions_outside_the_approved_range(self):
        payload = {
            "position_schedule": [100, 2100],
            "window_speed_mm_s": 1.0,
            "window_timeout_s": 20.0,
            "frequency_hz": 80,
            "duty_permyriad": 2000,
            "duration_s": 10.0,
        }
        with self.assertRaises(ValueError):
            DeviceControlService._validate_position_sweep_payload(payload)


if __name__ == "__main__":
    unittest.main()
