"""Own serial devices once and expose read-only telemetry plus a feed-test API."""

from __future__ import annotations

import argparse
import copy
import csv
import json
import sys
import threading
import time
import uuid
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from device_dashboard import DeviceReader, HTML
from powder_sampling_control.device_adapters import load_balance_calibration
from powder_sampling_control.dispensing_algorithm.core import (
    ContinuousFeedbackController,
    FeedbackControllerSettings,
    FeedbackInitialParameters,
    FeedbackObservation,
    MassMotionEstimator,
    MassMotionEstimatorSettings,
    acceptance_limits,
    list_control_profiles,
    load_control_profile,
    resolve_default_target_rate_band,
    resolve_control_profile,
    resolve_partial_target_overrides,
    DISPENSE_ALLOWED_OVERWEIGHT_MG,
    DISPENSE_ALLOWED_UNDERWEIGHT_MG,
    DISPENSE_PRESETS,
    DISPENSE_TARGET_PROFILES,
    interpolate_profile,
    powder_classification_params,
    validate_dispense_target,
)
from powder_sampling_control.dispensing_algorithm.calibration import (
    ContinuousTaperConfig,
    ContinuousTaperPlanner,
    ConstantRateController,
    ConstantRateObservation,
    ConstantRateSettings,
    GridSpec,
    GridDimension,
    GridConstraint,
    expand_grid,
    collect_scan_results,
    REFERENCE_POWDER_SCANS,
    PROBE_DEFAULT_DURATION_S,
    PROBE_DEFAULT_DUTY_PERMYRIAD,
    PROBE_DEFAULT_FREQUENCY_HZ,
    PROBE_DEFAULT_WINDOW_POSITION,
    PowderFingerprint,
    ProbeParameters,
    apply_fingerprint,
    delete_fingerprint,
    execute_probe,
    list_fingerprints,
    load_fingerprint,
    save_fingerprint,
    START_DUTY_PERMYRIAD,
    START_FREQUENCY_HZ,
    START_WINDOW_POSITION,
    TARGET_RATE_RANGE_MG_S,
    apply_rate_search,
    execute_rate_search,
    WindowPidConfig,
    WindowPidPlanner,
    WindowPidState,
)


POSITION_UNITS_PER_MM = 200
MOTION_REGISTER_UNITS_PER_MM = 100
POSITION_TOLERANCE_UNITS = 100
WINDOW_MIN_POSITION_UNITS = 100
# COM9 full measurements are polled at 10 Hz.  The AT8811C's 9600-baud
# Modbus link cannot expose its internal ADC rate directly.
POLL_INTERVAL_S = 0.10
# A single confirmed idle state is sufficient once the encoder is within the
# requested window tolerance. Requiring repeated idle reads caused false
# failures when the LA10 gateway's running flag briefly fluctuated.
WINDOW_IDLE_CONFIRMATIONS = 1
# Immediately after a motion command, one target-position idle read can be a
# stale gateway update. Confirm it once more before declaring that move done.
WINDOW_TARGET_IDLE_CONFIRMATIONS = 2
# Estimate flow and tail mass from every balance sample, but do not churn
# actuator settings on each estimate. Emergency stop decisions bypass this.
ACTUATOR_UPDATE_INTERVAL_S = 0.50
TARE_TIMEOUT_S = 5.0
POST_VIBRATION_SETTLE_S = 3.0
FINAL_STABILITY_WINDOW_S = 1.0
FINAL_STABILITY_MAX_SPREAD_MG = 1.0
DISPENSE_TARGET_MG = 500
DISPENSE_MIN_ACCEPT_MG = 490.0
DISPENSE_HARD_LIMIT_MG = 510.0
DISPENSE_MIN_DUTY_PERMYRIAD = 1000
DISPENSE_MAX_DUTY_PERMYRIAD = 5000
PREDICTIVE_STOP_HORIZON_S = 1.0
PREDICTIVE_STOP_CONFIRMATIONS = 2
PREDICTIVE_STOP_TARGET_OFFSET_MG = 3.0


def resolve_tail_pulse_config(overrides, target_mg):
    """Resolve and validate the optional settle-between-pulses tail stage."""
    source = overrides or {}
    enabled = bool(source.get("tail_pulse_enabled", False))
    config = {
        "enabled": enabled,
        "start_remaining_mg": float(source.get("tail_pulse_start_remaining_mg", 300.0)),
        "hold_window_position_units": int(
            source.get("tail_pulse_hold_window_position_units", 350)
        ),
        "initial_window_position_units": int(
            source.get("tail_pulse_initial_window_position_units", 385)
        ),
        "max_window_position_units": int(
            source.get("tail_pulse_max_window_position_units", 390)
        ),
        "min_window_position_units": int(
            source.get("tail_pulse_min_window_position_units", 380)
        ),
        "window_step_units": int(source.get("tail_pulse_window_step_units", 5)),
        "frequency_hz": int(source.get("tail_pulse_frequency_hz", 80)),
        "duty_permyriad": int(source.get("tail_pulse_duty_permyriad", 1000)),
        "initial_duration_s": float(source.get("tail_pulse_initial_duration_s", 0.2)),
        "min_duration_s": float(source.get("tail_pulse_min_duration_s", 0.08)),
        "max_duration_s": float(source.get("tail_pulse_max_duration_s", 0.3)),
        "settle_timeout_s": float(source.get("tail_pulse_settle_timeout_s", 8.0)),
        "settle_minimum_wait_s": float(
            source.get("tail_pulse_settle_minimum_wait_s", 1.0)
        ),
        "settle_continuous_stable_s": float(
            source.get("tail_pulse_settle_continuous_stable_s", 0.6)
        ),
        "settle_max_spread_mg": float(
            source.get("tail_pulse_settle_max_spread_mg", 2.0)
        ),
        "min_gain_mg": float(source.get("tail_pulse_min_gain_mg", 2.0)),
        "max_attempts": int(source.get("tail_pulse_max_attempts", 12)),
        "max_no_gain_attempts": int(
            source.get("tail_pulse_max_no_gain_attempts", 2)
        ),
        "safety_guard_gain_mg": float(
            source.get("tail_pulse_safety_guard_gain_mg", 140.0)
        ),
        "safety_guard_multiplier": float(
            source.get("tail_pulse_safety_guard_multiplier", 1.3)
        ),
        "micro_initial_window_position_units": int(
            source.get("tail_pulse_micro_initial_window_position_units", 370)
        ),
        "micro_max_window_position_units": int(
            source.get("tail_pulse_micro_max_window_position_units", 375)
        ),
        "micro_window_step_units": int(
            source.get("tail_pulse_micro_window_step_units", 5)
        ),
        "micro_initial_duration_s": float(
            source.get("tail_pulse_micro_initial_duration_s", 0.06)
        ),
        "micro_min_duration_s": float(
            source.get("tail_pulse_micro_min_duration_s", 0.05)
        ),
        "micro_max_duration_s": float(
            source.get("tail_pulse_micro_max_duration_s", 0.08)
        ),
        "micro_max_attempts": int(
            source.get("tail_pulse_micro_max_attempts", 16)
        ),
        "micro_safe_stop_min_mass_mg": float(
            source.get("tail_pulse_micro_safe_stop_min_mass_mg", 985.0)
        ),
        "micro_max_no_gain_attempts": int(
            source.get("tail_pulse_micro_max_no_gain_attempts", 2)
        ),
    }
    config["transition_mass_mg"] = float(target_mg) - config["start_remaining_mg"]
    if not enabled:
        return config
    if not 0 < config["start_remaining_mg"] < float(target_mg):
        raise ValueError("tail pulse start remaining mass must be inside the target")
    positions = (
        config["hold_window_position_units"],
        config["min_window_position_units"],
        config["initial_window_position_units"],
        config["max_window_position_units"],
    )
    if not all(100 <= value <= 2050 for value in positions):
        raise ValueError("tail pulse window positions must be inside 100..2050")
    if not positions[0] < positions[1] <= positions[2] <= positions[3]:
        raise ValueError("tail pulse windows must satisfy hold < min <= initial <= max")
    micro_positions = (
        config["micro_initial_window_position_units"],
        config["micro_max_window_position_units"],
    )
    if not all(100 <= value <= 2050 for value in micro_positions):
        raise ValueError("tail pulse micro windows must be inside 100..2050")
    if not (
        config["hold_window_position_units"]
        < micro_positions[0]
        <= micro_positions[1]
        < config["min_window_position_units"]
    ):
        raise ValueError(
            "tail pulse micro windows must satisfy hold < initial <= max < standard min"
        )
    if config["window_step_units"] < 1:
        raise ValueError("tail pulse window step must be positive")
    if config["micro_window_step_units"] < 1:
        raise ValueError("tail pulse micro window step must be positive")
    if not 10 <= config["frequency_hz"] <= 80:
        raise ValueError("tail pulse frequency must be inside 10..80 Hz")
    if not DISPENSE_MIN_DUTY_PERMYRIAD <= config["duty_permyriad"] <= DISPENSE_MAX_DUTY_PERMYRIAD:
        raise ValueError("tail pulse duty is outside the device range")
    if not 0 < config["min_duration_s"] <= config["initial_duration_s"] <= config["max_duration_s"]:
        raise ValueError("tail pulse durations must satisfy 0 < min <= initial <= max")
    if not (
        0
        < config["micro_min_duration_s"]
        <= config["micro_initial_duration_s"]
        <= config["micro_max_duration_s"]
    ):
        raise ValueError(
            "tail pulse micro durations must satisfy 0 < min <= initial <= max"
        )
    if not 0 < config["micro_safe_stop_min_mass_mg"] < float(target_mg):
        raise ValueError("tail pulse micro safe-stop mass must be below the target")
    if config["safety_guard_multiplier"] < 1.0:
        raise ValueError("tail pulse safety guard multiplier must be at least 1.0")
    if min(
        config["settle_timeout_s"],
        config["settle_minimum_wait_s"],
        config["settle_continuous_stable_s"],
        config["settle_max_spread_mg"],
        config["min_gain_mg"],
        config["max_attempts"],
        config["max_no_gain_attempts"],
        config["safety_guard_gain_mg"],
        config["safety_guard_multiplier"],
        config["micro_max_attempts"],
        config["micro_max_no_gain_attempts"],
    ) <= 0:
        raise ValueError("tail pulse settle and attempt settings must be positive")
    return config


def motion_distance_register(delta_position_units):
    """Convert a non-zero LA10 position delta to a valid motion distance."""
    delta = abs(float(delta_position_units))
    if delta == 0:
        return 0
    return max(
        1,
        round(delta / POSITION_UNITS_PER_MM * MOTION_REGISTER_UNITS_PER_MM),
    )


def validate_dispense_initial(initial):
    """Reject unsafe per-run seeds before a worker can start hardware."""
    limits = {
        "frequency_hz": (10, 80),
        "duty_permyriad": (
            DISPENSE_MIN_DUTY_PERMYRIAD,
            DISPENSE_MAX_DUTY_PERMYRIAD,
        ),
        "window_position_units": (100, 2050),
    }
    for key, (minimum, maximum) in limits.items():
        value = initial[key]
        if not isinstance(value, int) or not minimum <= value <= maximum:
            raise ValueError(f"{key} must be an integer from {minimum} to {maximum}")


def resolve_powder_identity(payload):
    """Resolve a canonical powder identity from a control profile or fingerprint."""
    powder_id = str(payload.get("powder_id", "")).strip()
    if not powder_id:
        raise ValueError("请先选择粉末类型")
    control_profile = load_control_profile(powder_id)
    fingerprint = None if control_profile else load_fingerprint(powder_id)
    if control_profile:
        powder_name = str(control_profile["powder_name"]).strip()
    elif fingerprint is not None:
        powder_name = str(fingerprint.powder_name).strip()
    else:
        raise ValueError(f"粉末控制配置或指纹不存在: {powder_id}")
    if not powder_name:
        raise ValueError(f"粉末名称为空: {powder_id}")
    requested_name = str(payload.get("powder_name", "")).strip()
    if requested_name and requested_name != powder_name:
        raise ValueError(
            f"粉末名称与指纹不一致: {requested_name} != {powder_name}"
        )
    return {"powder_id": powder_id, "powder_name": powder_name}


def json_default(value):
    """Serialize timestamped device records in HTTP responses and JSON logs."""
    if isinstance(value, datetime):
        return value.isoformat()
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


class DeviceControlService:
    """Single owner of COM9/COM8 with a 100 ms telemetry cache."""

    def __init__(
        self,
        at_port,
        la10_port,
        at_address,
        la10_address,
        balance_calibration=None,
    ):
        self.reader = DeviceReader(
            at_port,
            la10_port,
            at_address,
            la10_address,
            balance_calibration=balance_calibration,
        )
        self._snapshot_lock = threading.Lock()
        self._snapshot = {
            "timestamp_utc": None,
            "at8811c": {"online": False, "error": "等待首次采样"},
            "la10": {"online": False, "error": "等待首次采样"},
            "test": {"status": "idle"},
        }
        self._test_lock = threading.Lock()
        self._cancel_event = threading.Event()
        self._stop_event = threading.Event()
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        # Powder fingerprint session state
        self._powder_session: dict | None = None  # current applied fingerprint overrides
        self._probe_result: PowderFingerprint | None = None  # latest probe result cache
        self._rate_search_result = None  # latest rate-search result cache

    def start(self):
        self._poll_thread.start()

    def close(self):
        self._stop_event.set()
        self._poll_thread.join(timeout=1)
        self.reader.close()

    def snapshot(self):
        with self._snapshot_lock:
            return copy.deepcopy(self._snapshot)

    def start_feed_test(self, payload):
        self._validate_test_payload(payload)
        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("已有振动测试正在运行")
            run_id = f"feed-{datetime.now().strftime('%Y%m%d_%H%M%S')}-{uuid.uuid4().hex[:6]}"
            self._cancel_event.clear()
            self._set_test({"status": "running", "run_id": run_id, "submitted": payload})
            worker = threading.Thread(target=self._run_feed_test, args=(run_id, payload), daemon=True)
            worker.start()
            return {"accepted": True, "run_id": run_id, "poll_interval_ms": int(POLL_INTERVAL_S * 1000)}

    def start_window_position_sweep(self, payload):
        """Start a position-only calibration sweep with fixed vibration settings."""
        self._validate_position_sweep_payload(payload)
        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("a device test is already running")
            run_id = f"window-sweep-{datetime.now().strftime('%Y%m%d_%H%M%S')}-{uuid.uuid4().hex[:6]}"
            self._cancel_event.clear()
            self._set_test({"status": "running", "run_id": run_id, "submitted": payload})
            worker = threading.Thread(
                target=self._run_window_position_sweep, args=(run_id, payload), daemon=True
            )
            worker.start()
            return {"accepted": True, "run_id": run_id, "poll_interval_ms": int(POLL_INTERVAL_S * 1000)}

    def tare_balance(self):
        """Tare the balance only while every physical task is idle."""
        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("cannot tare while a device task is running")
            self._cancel_event.clear()
            with self.reader._lock:
                status = self.reader._la10.read_status()
                vibration = self.reader._la10.read_vibration_settings()
                if status.is_running or vibration.enabled:
                    raise RuntimeError("cannot tare while LA10 motion or vibration is active")
                before = self.reader._at.read_measurement()
                self.reader._at.zero(authorized=True)
            after = self._wait_for_stable_measurement(TARE_TIMEOUT_S)
        self._refresh()
        return {
            "accepted": True,
            "before_raw_count": before.raw_count,
            "after_raw_count": after.raw_count,
            "stable": after.stable,
            "in_zero_band": after.in_zero_band,
        }

    def list_dispense_powders(self):
        """Return every powder that can be selected by the formal dispense UI."""
        profiles = {
            item["powder_id"]: item for item in list_control_profiles()
        }
        fingerprints = {
            item["powder_id"]: item for item in list_fingerprints()
            if item.get("powder_id")
        }
        powders = []
        for powder_id in sorted(set(profiles) | set(fingerprints)):
            control_profile = profiles.get(powder_id)
            fingerprint_summary = fingerprints.get(powder_id)
            if control_profile is not None:
                powders.append({
                    **control_profile,
                    "config_source": "dedicated_profile",
                    "config_source_label": "专属控制配置",
                    "selectable": True,
                })
                continue

            fingerprint = load_fingerprint(powder_id)
            applied = apply_fingerprint(fingerprint) if fingerprint else {}
            preset_id = str(applied.get("preset_id", ""))
            selectable = preset_id in DISPENSE_PRESETS
            warnings = [] if selectable else [f"指纹引用了未知 preset: {preset_id or '空'}"]
            powders.append({
                "powder_id": powder_id,
                "powder_name": fingerprint_summary.get("powder_name", ""),
                "status": "fingerprint-default",
                "recommended_preset_id": preset_id,
                "anchor_masses_mg": [],
                "generation_warnings": warnings,
                "config_source": "fingerprint_with_product_defaults",
                "config_source_label": "粉末指纹＋统一默认参数",
                "selectable": selectable,
            })
        return powders

    def resolve_dispense_configuration(self, payload):
        """Resolve the exact automatic settings used for a powder and target."""
        powder = resolve_powder_identity(payload)
        raw_target = payload.get("target_mg", DISPENSE_TARGET_MG)
        validate_dispense_target(raw_target)
        target_mg = raw_target
        profile = interpolate_profile(target_mg)
        control_profile = load_control_profile(powder["powder_id"])
        resolved_control = (
            resolve_control_profile(control_profile, target_mg)
            if control_profile is not None
            else None
        )
        if resolved_control:
            profile.update(resolved_control["target_profile"])

        # A stored control profile takes precedence. Legacy fingerprints remain
        # supported for supervised development runs.
        powder_overrides = {}
        active_powder_session = (
            self._powder_session
            if control_profile is None
            and self._powder_session
            and self._powder_session.get("powder_id") == powder["powder_id"]
            else None
        )
        if control_profile is None and active_powder_session is None:
            fingerprint = load_fingerprint(powder["powder_id"])
            if fingerprint is not None:
                active_powder_session = apply_fingerprint(fingerprint)

        control_settings = (
            resolved_control["recommended_controller"]
            if resolved_control
            else active_powder_session
        )
        if control_settings is None:
            raise ValueError(
                f"粉末 {powder['powder_id']} 没有可用控制配置，不能启动加粉"
            )

        if resolved_control:
            for key in ("maximum_flow_rate_mg_s",):
                if key in control_settings:
                    profile[key] = float(control_settings[key])
        if active_powder_session:
            profile = dict(profile)
            for key in ("maximum_flow_rate_mg_s",):
                if key in active_powder_session:
                    profile[key] = float(active_powder_session[key])
            # Apply legacy fingerprint target overrides after normalizing JSON
            # string keys and interpolating arbitrary integer targets.
            cat_overrides = active_powder_session.get("profile_overrides", {})
            target_overrides = (
                resolve_partial_target_overrides(cat_overrides, target_mg)
                if cat_overrides
                else {}
            )
            for key in ("coarse_rate_mg_s", "fine_rate_mg_s",
                        "precision_start_remaining_mg", "tail_taper_start_remaining_mg",
                        "tail_taper_end_remaining_mg"):
                if key in target_overrides:
                    profile[key] = float(target_overrides[key])
        if not resolved_control:
            default_rate_band = resolve_default_target_rate_band(target_mg)
            for key in (
                "coarse_rate_mg_s",
                "fine_rate_mg_s",
                "precision_rate_mg_s",
            ):
                profile[key] = float(default_rate_band[key])
        powder_overrides = {
            "fixed_tail_mass_mg": control_settings.get("fixed_tail_mass_mg", 3.0),
            "settle_confirmations_delta": control_settings.get("settle_confirmations_delta", 0),
            "dead_zone_multiplier": control_settings.get("dead_zone_multiplier", 1.0),
            "fast_predictive_stop_enabled": control_settings.get(
                "fast_predictive_stop_enabled", False
            ),
            "fast_predictive_stop_rate_mg_s": control_settings.get(
                "fast_predictive_stop_rate_mg_s", 20.0
            ),
            "fast_predictive_stop_confirmations": control_settings.get(
                "fast_predictive_stop_confirmations", 1
            ),
            "projected_safety_stop_enabled": control_settings.get(
                "projected_safety_stop_enabled", False
            ),
            "projected_safety_stop_offset_mg": control_settings.get(
                "projected_safety_stop_offset_mg", 5.0
            ),
            "duty_step_per_update": control_settings.get("duty_step_per_update", 50),
            "emergency_duty_step_per_update": control_settings.get("emergency_duty_step_per_update", 200),
            "position_feedback_enabled": control_settings.get("position_feedback_enabled", False),
            "force_stop_offset_mg": control_settings.get("force_stop_offset_mg", 10.0),
            "max_duty_permyriad": control_settings.get(
                "max_duty_permyriad", DISPENSE_MAX_DUTY_PERMYRIAD
            ),
            "flow_hold_enabled": control_settings.get("flow_hold_enabled", False),
            "flow_hold_window_s": control_settings.get("flow_hold_window_s", 4.0),
            "flow_hold_min_gain_mg": control_settings.get(
                "flow_hold_min_gain_mg", 5.0
            ),
            "flow_response_check_s": control_settings.get(
                "flow_response_check_s", 4.0
            ),
            "flow_response_min_improvement_mg_s": control_settings.get(
                "flow_response_min_improvement_mg_s", 1.0
            ),
            "stall_recovery_enabled": control_settings.get(
                "stall_recovery_enabled", False
            ),
            "stall_recovery_observation_window_s": control_settings.get(
                "stall_recovery_observation_window_s", 3.0
            ),
            "stall_recovery_min_gain_mg": control_settings.get(
                "stall_recovery_min_gain_mg", 2.0
            ),
            "stall_recovery_rate_threshold_mg_s": control_settings.get(
                "stall_recovery_rate_threshold_mg_s", 1.0
            ),
            "stall_recovery_resume_rate_mg_s": control_settings.get(
                "stall_recovery_resume_rate_mg_s", 2.0
            ),
            "stall_recovery_retry_s": control_settings.get(
                "stall_recovery_retry_s", 3.0
            ),
            "stall_recovery_window_step_units": control_settings.get(
                "stall_recovery_window_step_units", 25
            ),
            "stall_recovery_max_window_position_units": control_settings.get(
                "stall_recovery_max_window_position_units", 400
            ),
            "stall_recovery_max_attempts": control_settings.get(
                "stall_recovery_max_attempts", 2
            ),
            "stall_recovery_disable_remaining_mg": control_settings.get(
                "stall_recovery_disable_remaining_mg", 50.0
            ),
        }
        for key in (
            "tail_pulse_enabled",
            "tail_pulse_start_remaining_mg",
            "tail_pulse_hold_window_position_units",
            "tail_pulse_initial_window_position_units",
            "tail_pulse_max_window_position_units",
            "tail_pulse_min_window_position_units",
            "tail_pulse_window_step_units",
            "tail_pulse_frequency_hz",
            "tail_pulse_duty_permyriad",
            "tail_pulse_initial_duration_s",
            "tail_pulse_min_duration_s",
            "tail_pulse_max_duration_s",
            "tail_pulse_settle_timeout_s",
            "tail_pulse_settle_minimum_wait_s",
            "tail_pulse_settle_continuous_stable_s",
            "tail_pulse_settle_max_spread_mg",
            "tail_pulse_min_gain_mg",
            "tail_pulse_max_attempts",
            "tail_pulse_max_no_gain_attempts",
            "tail_pulse_safety_guard_gain_mg",
            "tail_pulse_safety_guard_multiplier",
            "tail_pulse_micro_initial_window_position_units",
            "tail_pulse_micro_max_window_position_units",
            "tail_pulse_micro_window_step_units",
            "tail_pulse_micro_initial_duration_s",
            "tail_pulse_micro_min_duration_s",
            "tail_pulse_micro_max_duration_s",
            "tail_pulse_micro_max_attempts",
            "tail_pulse_micro_safe_stop_min_mass_mg",
            "tail_pulse_micro_max_no_gain_attempts",
        ):
            if key in control_settings:
                powder_overrides[key] = control_settings[key]
        recommended_preset = (
            resolved_control["recommended_preset_id"]
            if resolved_control
            else active_powder_session.get("preset_id", "")
        )
        # Powder identity owns the process/controller configuration. The UI may
        # still choose a different actuator seed for this run without changing
        # stage rates, thresholds, ceilings, or other powder-specific settings.
        preset_id = str(payload.get("preset_id") or recommended_preset)
        if preset_id in DISPENSE_PRESETS:
            initial = dict(DISPENSE_PRESETS[preset_id])
        else:
            raise ValueError(f"粉末控制配置引用了未知 preset: {preset_id}")
        # A rate-search session may provide a measured initial point.
        if active_powder_session and active_powder_session.get("initial"):
            for key in initial:
                if key in active_powder_session["initial"]:
                    initial[key] = int(active_powder_session["initial"][key])
        requested_initial = payload.get("initial", {})
        if requested_initial is not None:
            if not isinstance(requested_initial, dict):
                raise ValueError("initial must be an object when supplied")
            for key in initial:
                if key in requested_initial:
                    initial[key] = int(requested_initial[key])
        validate_dispense_initial(initial)
        acceptance_min_mg, acceptance_max_mg = acceptance_limits(target_mg)
        source = (
            "dedicated_profile" if resolved_control
            else "fingerprint_with_product_defaults"
        )
        return {
            **powder,
            "target_mg": target_mg,
            "preset_id": preset_id,
            "initial": initial,
            "acceptance_min_mg": acceptance_min_mg,
            "acceptance_max_mg": acceptance_max_mg,
            "profile": profile,
            "controller": powder_overrides,
            "config_source": source,
            "config_source_label": (
                "专属控制配置" if resolved_control else "粉末指纹＋统一默认参数"
            ),
            "control_profile_status": (
                resolved_control["status"] if resolved_control else "fingerprint-default"
            ),
            "target_resolution": (
                resolved_control["target_resolution"] if resolved_control else None
            ),
            "target_rate_band_source": (
                resolved_control["target_rate_band_source"]
                if resolved_control else "product_default"
            ),
            "control_profile_warnings": (
                resolved_control["generation_warnings"] if resolved_control else []
            ),
        }

    def start_dispense(self, payload):
        """Start one dispense run with automatically resolved powder settings."""
        configuration = self.resolve_dispense_configuration(payload)
        powder = {
            "powder_id": configuration["powder_id"],
            "powder_name": configuration["powder_name"],
        }
        target_mg = configuration["target_mg"]
        preset_id = configuration["preset_id"]
        initial = configuration["initial"]
        profile = configuration["profile"]
        powder_overrides = configuration["controller"]
        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("a device task is already running")
            run_id = f"dispense-{target_mg}mg-{datetime.now().strftime('%Y%m%d_%H%M%S')}-{uuid.uuid4().hex[:6]}"
            self._cancel_event.clear()
            # A direct operation is a complete dispense cycle: move to the
            # configured feed position, dispense, then close the window again.
            # Batch runs manage the same cleanup between repetitions in their
            # own worker so they can stop the queue safely on a close failure.
            submitted = copy.deepcopy(configuration)
            submitted["close_window_after_run"] = False
            self._set_test({"status": "running", "run_id": run_id, "submitted": submitted})
            worker = threading.Thread(
                target=self._run_feedback_dispense,
                args=(
                    run_id, target_mg, preset_id, initial, profile,
                    powder_overrides, powder,
                ),
                kwargs={"close_window_after_run": False},
                daemon=True,
            )
            worker.start()
            return {"accepted": True, "run_id": run_id, "submitted": submitted}

    def start_500mg_dispense(self, payload):
        """Compatibility entry point for the legacy 500 mg dashboard button."""
        return self.start_dispense({**payload, "target_mg": DISPENSE_TARGET_MG})

    @staticmethod
    def _large_precision_reserve_mg(target_mg: int) -> float:
        """Return the supervised large-run closed-loop reserve in mg.

        Runs below 5 g keep a fixed 500 mg reserve.  From 5 g upwards the
        reserve is 10% of the target, so an 8 g run hands control back to the
        saved closed-loop configuration with 800 mg still to go.  The policy is
        continuous at exactly 5 g, where both rules give 500 mg.
        """
        if target_mg < 5000:
            return 500.0
        return round(target_mg * 0.1, 1)

    def start_large_dispense(self, payload):
        """Start a supervised 1–10 g run with a target-specific precision reserve.

        The fixed-duty pre-feed uses the operator-selected duty (default 20%)
        and the operator-selected window.  Once the reserve is reached, the
        saved 500 mg powder configuration supplies the closed-loop rates and
        controller limits; only the larger-run reserve itself differs.  Below
        5 g the reserve is a fixed 500 mg; from 5 g up it is 10% of the target.
        """
        if not isinstance(payload, dict):
            raise ValueError("payload must be an object")
        target_mg = payload.get("target_mg")
        if isinstance(target_mg, bool) or not isinstance(target_mg, int) or not 1000 <= target_mg <= 10000:
            raise ValueError("large target_mg must be an integer from 1000 to 10000")
        powder = resolve_powder_identity(payload)
        requested_initial = payload.get("initial") or {}
        initial = {
            "frequency_hz": int(requested_initial.get("frequency_hz", 80)),
            "duty_permyriad": int(requested_initial.get("duty_permyriad", 2000)),
            "window_position_units": int(requested_initial.get("window_position_units", 700)),
        }
        if not 10 <= initial["frequency_hz"] <= 80:
            raise ValueError("large frequency must be in 10..80 Hz")
        if not 1000 <= initial["duty_permyriad"] <= 5000:
            raise ValueError("large duty must be in 1000..5000")
        if not 0 <= initial["window_position_units"] <= 2050:
            raise ValueError("large window position must be in 0..2050")
        precision_reserve_mg = self._large_precision_reserve_mg(target_mg)
        saved_configuration = self.resolve_dispense_configuration({
            "powder_id": powder["powder_id"], "target_mg": 500,
        })
        profile = dict(saved_configuration["profile"])
        overrides = dict(saved_configuration["controller"])
        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("a device task is already running")
            run_id = f"large-dispense-{target_mg}mg-{datetime.now().strftime('%Y%m%d_%H%M%S')}-{uuid.uuid4().hex[:6]}"
            self._cancel_event.clear()
            submitted = {"powder_id": powder["powder_id"], "powder_name": powder.get("powder_name"),
                         "target_mg": target_mg, "initial": initial, "profile": profile,
                         "controller": overrides,
                         "closed_loop_config_target_mg": 500,
                         "large_prefeed_reserve_mg": precision_reserve_mg,
                         "closed_loop_initial": saved_configuration["initial"],
                         "mode": "large_fixed_prefeed_then_saved_closed_loop",
                         "close_window_after_run": False}
            self._set_test({"status": "running", "run_id": run_id, "submitted": submitted})
            threading.Thread(target=self._run_feedback_dispense,
                args=(run_id, target_mg, "large-80hz", initial, profile, overrides, powder),
                kwargs={
                    "close_window_after_run": False,
                    "large_prefeed_reserve_mg": precision_reserve_mg,
                    "closed_loop_initial": saved_configuration["initial"],
                }, daemon=True).start()
            return {"accepted": True, "run_id": run_id, "submitted": submitted}

    def start_continuous_taper_test(self):
        """Start the independent fixed-stage continuous-taper controller."""
        config = ContinuousTaperConfig()
        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("a device task is already running")
            run_id = (
                f"continuous-taper-500mg-{datetime.now().strftime('%Y%m%d_%H%M%S')}-"
                f"{uuid.uuid4().hex[:6]}"
            )
            self._cancel_event.clear()
            submitted = {
                "target_mg": config.target_mass_mg,
                "coarse_end_mg": config.coarse_end_mg,
                "coarse": config.coarse.__dict__,
                "taper": config.taper.__dict__,
                "control_strategy": "fixed-stage-continuous-taper",
                "algorithm_version": "continuous-taper-v1",
            }
            self._set_test({"status": "running", "run_id": run_id, "submitted": submitted})
            worker = threading.Thread(
                target=self._run_continuous_taper_dispense,
                args=(run_id,),
                daemon=True,
            )
            worker.start()
            return {"accepted": True, "run_id": run_id, "submitted": submitted}

    def start_window_pid_test(self):
        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("a device task is already running")
            run_id = f"window-pid-500mg-{datetime.now().strftime('%Y%m%d_%H%M%S')}-{uuid.uuid4().hex[:6]}"
            self._cancel_event.clear()
            submitted = {
                "target_mg": DISPENSE_TARGET_MG,
                "frequency_hz": 80,
                "duty_permyriad": 2000,
                "control_variable": "window_position_units",
            }
            self._set_test({"status": "running", "run_id": run_id, "submitted": submitted})
            worker = threading.Thread(
                target=self._run_window_pid_test, args=(run_id,), daemon=True
            )
            worker.start()
            return {"accepted": True, "run_id": run_id, "submitted": submitted}

    def cancel_feed_test(self):
        self._cancel_event.set()
        return {"accepted": True}

    # ── powder fingerprint probe ──────────────────────────────────────

    def start_powder_probe(self, payload):
        """Start a powder fingerprint probe test."""
        powder_name = str(payload.get("powder_name", "")).strip()
        if not powder_name:
            raise ValueError("powder_name must be a non-empty string")

        probe_params = ProbeParameters(
            frequency_hz=int(payload.get("frequency_hz", PROBE_DEFAULT_FREQUENCY_HZ)),
            duty_permyriad=int(payload.get("duty_permyriad", PROBE_DEFAULT_DUTY_PERMYRIAD)),
            window_position_units=int(payload.get("window_position_units", PROBE_DEFAULT_WINDOW_POSITION)),
            duration_s=float(payload.get("duration_s", PROBE_DEFAULT_DURATION_S)),
        )
        # Validate ranges
        if not 10 <= probe_params.frequency_hz <= 80:
            raise ValueError("frequency_hz must be 10–80")
        if not 1000 <= probe_params.duty_permyriad <= 5000:
            raise ValueError("duty_permyriad must be 1000–5000")
        if not 100 <= probe_params.window_position_units <= 2050:
            raise ValueError("window_position_units must be 100–2050")

        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("已有设备任务正在运行")
            run_id = f"probe-{datetime.now().strftime('%Y%m%d_%H%M%S')}-{uuid.uuid4().hex[:6]}"
            self._cancel_event.clear()
            self._probe_result = None
            self._set_test({
                "status": "running", "run_id": run_id,
                "phase": "启动中", "progress_pct": 0.0,
                "submitted": {"powder_name": powder_name, "probe_params": payload},
            })
            worker = threading.Thread(
                target=self._run_powder_probe,
                args=(run_id, powder_name, probe_params),
                daemon=True,
            )
            worker.start()
            return {"accepted": True, "run_id": run_id}

    def _run_powder_probe(self, run_id, powder_name, probe_params):
        """Background thread: execute probe, classify, save, auto-apply."""
        result_data = {"run_id": run_id, "result": "failed"}
        try:
            def progress_callback(phase, pct):
                self._set_test({
                    "status": "running", "run_id": run_id,
                    "phase": phase, "progress_pct": round(pct, 1),
                    "result": result_data,
                })

            fingerprint = execute_probe(
                self.reader, probe_params, self._cancel_event, progress_callback,
            )
            # Generate powder_id from timestamp (ASCII-safe, no Chinese chars)
            fingerprint.powder_id = f"powder-{datetime.now().strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:6]}"
            fingerprint.powder_name = powder_name

            # Save to disk
            save_fingerprint(fingerprint)

            # Auto-apply to session
            self._powder_session = apply_fingerprint(fingerprint)
            self._probe_result = fingerprint

            result_data.update({
                "result": "completed",
                "powder_id": fingerprint.powder_id,
                "powder_name": fingerprint.powder_name,
                "feature_vector": fingerprint.feature_vector.__dict__,
                "classification": fingerprint.classification.__dict__,
                "applied_profile": fingerprint.applied_profile,
            })
            self._set_test({
                "status": "idle", "run_id": run_id,
                "phase": "完成", "progress_pct": 100.0,
                "result": result_data,
            })
        except Exception as error:
            result_data["result"] = "failed"
            result_data["error"] = str(error)
            self._set_test({
                "status": "idle", "run_id": run_id,
                "phase": "失败", "progress_pct": 0.0,
                "result": result_data,
            })

    def get_probe_result(self, probe_id):
        """Return cached probe result if available."""
        if self._probe_result is None:
            raise RuntimeError("没有探针结果缓存")
        return self._probe_result

    def get_powder_status(self):
        """Return current session powder status."""
        if self._powder_session:
            return {"loaded": True, **self._powder_session}
        return {"loaded": False, "message": "未加载粉末指纹，使用默认参数"}

    def apply_powder_fingerprint(self, powder_id):
        """Load a stored fingerprint by powder_id and apply to current session."""
        fingerprint = load_fingerprint(powder_id)
        if fingerprint is None:
            raise ValueError(f"粉末指纹不存在: {powder_id}")
        self._powder_session = apply_fingerprint(fingerprint)
        self._probe_result = fingerprint
        return {"applied": True, "powder_id": powder_id, **self._powder_session}

    def set_powder_category(self, category):
        """Manually change powder classification category (session-only)."""
        if category not in ("Low-flow", "Medium-flow", "High-flow"):
            raise ValueError(f"无效分类: {category}")
        if not self._powder_session:
            raise RuntimeError("未加载粉末指纹")
        tags = self._powder_session.get("tags", [])
        params = powder_classification_params(category, tags)
        self._powder_session["category"] = category
        self._powder_session["preset_id"] = params["preset_id"]
        self._powder_session["maximum_flow_rate_mg_s"] = params["maximum_flow_rate_mg_s"]
        self._powder_session["fixed_tail_mass_mg"] = params["fixed_tail_mass_mg"]
        self._powder_session["profile_overrides"] = params.get("profile_overrides", {})
        self._powder_session["duty_step_per_update"] = params.get("duty_step_per_update", 50)
        self._powder_session["emergency_duty_step_per_update"] = params.get("emergency_duty_step_per_update", 200)
        self._powder_session["position_feedback_enabled"] = params.get("position_feedback_enabled", False)
        self._powder_session["force_stop_offset_mg"] = params.get("force_stop_offset_mg", 10.0)
        if "unstable" in tags:
            self._powder_session["settle_confirmations_delta"] = params.get("settle_confirmations_delta", 0)
            self._powder_session["dead_zone_multiplier"] = params.get("dead_zone_multiplier", 1.0)
        return {"category": category, **params}

    # ── powder rate-band search ───────────────────────────────────────

    def start_rate_search(self, payload):
        """Start a target-rate band search for a powder."""
        powder_name = str(payload.get("powder_name", "")).strip()
        if not powder_name:
            raise ValueError("powder_name must be a non-empty string")

        target_lo = float(payload.get("target_lo", TARGET_RATE_RANGE_MG_S[0]))
        target_hi = float(payload.get("target_hi", TARGET_RATE_RANGE_MG_S[1]))
        if not (1.0 <= target_lo < target_hi <= 100.0):
            raise ValueError("target band must satisfy 1 ≤ lo < hi ≤ 100")

        start_freq = int(payload.get("start_frequency_hz", START_FREQUENCY_HZ))
        start_duty = int(payload.get("start_duty_permyriad", START_DUTY_PERMYRIAD))
        start_window = int(payload.get("start_window_position_units", START_WINDOW_POSITION))
        if not 10 <= start_freq <= 80:
            raise ValueError("start_frequency_hz must be 10–80")
        if not 1000 <= start_duty <= 5000:
            raise ValueError("start_duty_permyriad must be 1000–5000")
        if not 100 <= start_window <= 2050:
            raise ValueError("start_window_position_units must be 100–2050")

        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("已有设备任务正在运行")
            run_id = f"ratesearch-{datetime.now().strftime('%Y%m%d_%H%M%S')}-{uuid.uuid4().hex[:6]}"
            self._cancel_event.clear()
            self._rate_search_result = None
            self._set_test({
                "status": "running", "run_id": run_id,
                "phase": "启动中", "progress_pct": 0.0,
                "submitted": {"powder_name": powder_name, "params": payload},
            })
            worker = threading.Thread(
                target=self._run_rate_search,
                args=(run_id, powder_name, target_lo, target_hi,
                      start_freq, start_duty, start_window),
                daemon=True,
            )
            worker.start()
            return {"accepted": True, "run_id": run_id}

    def _run_rate_search(self, run_id, powder_name, target_lo, target_hi,
                         start_freq, start_duty, start_window):
        """Background thread: search, cache, auto-apply."""
        result_data = {"run_id": run_id, "result": "failed"}
        try:
            def progress_callback(phase, pct):
                self._set_test({
                    "status": "running", "run_id": run_id,
                    "phase": phase, "progress_pct": round(pct, 1),
                    "result": result_data,
                })

            search_result = execute_rate_search(
                self.reader, self._cancel_event, progress_callback,
                target_lo=target_lo, target_hi=target_hi,
                start_freq_hz=start_freq, start_duty_permyriad=start_duty,
                start_window_units=start_window,
            )
            search_result.powder_id = f"powder-{datetime.now().strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:6]}"
            search_result.powder_name = powder_name

            self._rate_search_result = search_result
            self._powder_session = apply_rate_search(search_result)

            result_data.update({
                "result": "completed",
                "powder_id": search_result.powder_id,
                "powder_name": search_result.powder_name,
                "target_rate_range_mg_s": search_result.target_rate_range_mg_s,
                "locked_point": search_result.locked_point,
                "stable_band": search_result.stable_band,
                "applied": self._powder_session,
            })
            self._set_test({
                "status": "idle", "run_id": run_id,
                "phase": "完成", "progress_pct": 100.0,
                "result": result_data,
            })
        except Exception as error:
            result_data["result"] = "failed"
            result_data["error"] = str(error)
            self._set_test({
                "status": "idle", "run_id": run_id,
                "phase": "失败", "progress_pct": 0.0,
                "result": result_data,
            })

    def get_rate_search_result(self):
        """Return cached rate-search result if available."""
        if self._rate_search_result is None:
            raise RuntimeError("没有速率搜索结果缓存")
        return self._rate_search_result.to_dict()

    # ── constant-rate dispensing test ──────────────────────────────────

    @staticmethod
    def _validate_constant_rate_payload(payload):
        """Reject unsafe constant-rate test parameters before starting."""
        limits = {
            "frequency_hz": (10, 80),
            "duty_permyriad": (1000, 5000),
            "window_position_units": (100, 2050),
        }
        for key, (minimum, maximum) in limits.items():
            value = payload.get(key)
            if not isinstance(value, int) or not minimum <= value <= maximum:
                raise ValueError(f"{key} must be an integer from {minimum} to {maximum}")
        duration = payload.get("duration_s", 15)
        if not isinstance(duration, (int, float)) or not 5 <= duration <= 300:
            raise ValueError("duration_s must be a number from 5 to 300")
        repeats = payload.get("repeat_count", 3)
        if not isinstance(repeats, int) or not 1 <= repeats <= 10:
            raise ValueError("repeat_count must be an integer from 1 to 10")

    def start_constant_rate_test(self, payload):
        """Start one constant-rate dispensing run with fixed parameters."""
        payload.setdefault("duration_s", 15)
        payload.setdefault("repeat_count", 3)
        payload.setdefault("target_rate_mg_s", 10.0)
        self._validate_constant_rate_payload(payload)
        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("a device test is already running")
            run_id = (
                f"const-rate-{payload['frequency_hz']}hz-"
                f"{payload['duty_permyriad'] // 100}pct-"
                f"{datetime.now().strftime('%Y%m%d_%H%M%S')}-"
                f"{uuid.uuid4().hex[:6]}"
            )
            self._cancel_event.clear()
            submitted = {
                "test_type": "constant_rate_dispense",
                "target_rate_mg_s": payload["target_rate_mg_s"],
                "frequency_hz": payload["frequency_hz"],
                "duty_permyriad": payload["duty_permyriad"],
                "window_position_units": payload["window_position_units"],
                "duration_s": payload["duration_s"],
                "repeat_count": payload["repeat_count"],
            }
            self._set_test({"status": "running", "run_id": run_id, "submitted": submitted})
            worker = threading.Thread(
                target=self._run_constant_rate_test, args=(run_id, submitted), daemon=True
            )
            worker.start()
            return {"accepted": True, "run_id": run_id, "submitted": submitted}

    def start_constant_rate_batch(self, payload):
        """Start a batch queue of constant-rate tests."""
        sets = payload.get("sets", [])
        if not isinstance(sets, list) or len(sets) < 1:
            raise ValueError("sets must be a non-empty array of parameter objects")
        if len(sets) > 20:
            raise ValueError("batch queue limited to 20 sets")
        for idx, params in enumerate(sets):
            params.setdefault("duration_s", payload.get("duration_s", 15))
            params.setdefault("repeat_count", payload.get("repeat_count", 3))
            try:
                self._validate_constant_rate_payload(params)
            except ValueError as error:
                raise ValueError(f"set [{idx}] invalid: {error}") from error
        tare_between = bool(payload.get("tare_between_sets", True))
        target_rate = float(payload.get("target_rate_mg_s", 10.0))
        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("a device test is already running")
            run_id = f"const-batch-{len(sets)}sets-{datetime.now().strftime('%Y%m%d_%H%M%S')}-{uuid.uuid4().hex[:6]}"
            self._cancel_event.clear()
            submitted = {
                "test_type": "constant_rate_batch",
                "target_rate_mg_s": target_rate,
                "tare_between_sets": tare_between,
                "set_count": len(sets),
                "sets": sets,
            }
            self._set_test({"status": "running", "run_id": run_id, "submitted": submitted})
            worker = threading.Thread(
                target=self._run_constant_rate_batch, args=(run_id, submitted), daemon=True
            )
            worker.start()
            return {"accepted": True, "run_id": run_id, "submitted": submitted}

    def start_dispense_batch(self, payload):
        """Start a batch queue of feedback-dispense tests."""
        raw_sets = payload.get("sets", [])
        if not isinstance(raw_sets, list) or len(raw_sets) < 1:
            raise ValueError("sets must be a non-empty array of parameter objects")
        if len(raw_sets) > 20:
            raise ValueError("batch queue limited to 20 sets")
        sets = []
        powder_ids = set()
        for idx, raw_params in enumerate(raw_sets):
            if not isinstance(raw_params, dict):
                raise ValueError(f"set [{idx}] must be a parameter object")
            params = dict(raw_params)
            powder = resolve_powder_identity(params)
            params.update(powder)
            powder_ids.add(powder["powder_id"])
            target_mg = params.get("target_mg", 500)
            validate_dispense_target(target_mg)
            params["target_mg"] = target_mg
            freq = int(params.get("frequency_hz", 80))
            if not 10 <= freq <= 80:
                raise ValueError(f"set [{idx}]: frequency_hz must be 10-80, got {freq}")
            duty = int(params.get("duty_permyriad", 2000))
            if not 1000 <= duty <= 5000:
                raise ValueError(f"set [{idx}]: duty_permyriad must be 1000-5000, got {duty}")
            win = int(params.get("window_position_units", 200))
            if not 100 <= win <= 2050:
                raise ValueError(f"set [{idx}]: window_position_units must be 100-2050, got {win}")
            params.setdefault("repeat_count", 1)
            sets.append(params)
        if len(powder_ids) != 1:
            raise ValueError("同一批量队列只能使用一种粉末，请清空队列后重新添加")
        powder = {
            "powder_id": sets[0]["powder_id"],
            "powder_name": sets[0]["powder_name"],
        }
        tare_between = bool(payload.get("tare_between_sets", True))
        close_window_between_runs = bool(payload.get("close_window_between_runs", False))
        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("a device test is already running")
            run_id = f"disp-batch-{len(sets)}sets-{datetime.now().strftime('%Y%m%d_%H%M%S')}-{uuid.uuid4().hex[:6]}"
            self._cancel_event.clear()
            submitted = {
                **powder,
                "test_type": "dispense_batch",
                "tare_between_sets": tare_between,
                "close_window_between_runs": close_window_between_runs,
                "set_count": len(sets),
                "sets": sets,
            }
            self._set_test({"status": "running", "run_id": run_id, "submitted": submitted})
            worker = threading.Thread(
                target=self._run_dispense_batch, args=(run_id, submitted), daemon=True
            )
            worker.start()
            return {"accepted": True, "run_id": run_id, "submitted": submitted}

    # ── Experiment Orchestrator ──────────────────────────────────────

    def start_grid_experiment(self, payload):
        """Accept a grid spec, expand it, and delegate to the existing batch runner.

        Request body::

            {
              "experiment_type": "constant_rate",
              "powder_name": "玉米淀粉",
              "powder_id": "corn_starch",
              "dimensions": [
                {"name": "frequency_hz", "values": [50, 55, 60, 65, 70, 75]},
                {"name": "duty_permyriad", "values": [2000]},
                {"name": "window_position_units", "values": [250]}
              ],
              "duration_s": 15,
              "repeat_count": 3,
              "tare_between_sets": true,
              "auto_collect": true
            }
        """
        experiment_type = payload.get("experiment_type", "constant_rate")
        if experiment_type not in ("constant_rate", "dispense"):
            raise ValueError("experiment_type must be constant_rate or dispense")

        explicit_rows = payload.get("sets")
        if explicit_rows is not None and (
            not isinstance(explicit_rows, list) or not explicit_rows
        ):
            raise ValueError("sets must be a non-empty array when supplied")

        # Explicit preview rows preserve per-row edits/removals from the UI.
        # Legacy callers may continue to submit Cartesian dimensions.
        dims = [
            GridDimension(d["name"], d["values"]) for d in payload.get("dimensions", [])
        ]
        if explicit_rows and not dims:
            dims = [
                GridDimension(
                    "frequency_hz", [explicit_rows[0].get("frequency_hz")]
                )
            ]
        constraints = [
            GridConstraint(c["type"], c["max_value"])
            for c in payload.get("constraints", [])
        ]
        grid_target_mg = payload.get("target_mg", 0)
        if experiment_type == "dispense":
            validate_dispense_target(grid_target_mg)
        spec = GridSpec(
            experiment_id=payload.get("experiment_id", f"grid-{uuid.uuid4().hex[:6]}"),
            powder_name=payload.get("powder_name", "unknown"),
            powder_id=payload.get("powder_id", "unknown"),
            experiment_type=experiment_type,
            dimensions=dims,
            constraints=constraints,
            duration_s=float(payload.get("duration_s", 15)),
            repeat_count=int(payload.get("repeat_count", 1)),
            tare_between_sets=bool(payload.get("tare_between_sets", True)),
            target_rate_mg_s=float(payload.get("target_rate_mg_s", 10.0)),
            target_mg=grid_target_mg,
            preset_id=str(payload.get("preset_id", "")),
        )

        if explicit_rows:
            warnings = []
            sets = []
            for index, row in enumerate(explicit_rows):
                if not isinstance(row, dict):
                    raise ValueError(f"set [{index}] must be an object")
                frequency = row.get("frequency_hz")
                duty = row.get("duty_permyriad")
                window = row.get("window_position_units")
                duration = row.get("duration_s", spec.duration_s)
                repeats = row.get("repeat_count", spec.repeat_count)
                if isinstance(frequency, bool) or not isinstance(frequency, int) or not 10 <= frequency <= 80:
                    raise ValueError(f"set [{index}]: frequency_hz must be an integer from 10 to 80")
                if isinstance(duty, bool) or not isinstance(duty, int) or not 1000 <= duty <= 5000:
                    raise ValueError(f"set [{index}]: duty_permyriad must be an integer from 1000 to 5000")
                if isinstance(window, bool) or not isinstance(window, int) or not 100 <= window <= 750:
                    raise ValueError(f"set [{index}]: window_position_units must be an integer from 100 to 750")
                if isinstance(duration, bool) or not isinstance(duration, (int, float)) or not 1 <= duration <= 120:
                    raise ValueError(f"set [{index}]: duration_s must be from 1 to 120")
                if isinstance(repeats, bool) or not isinstance(repeats, int) or not 1 <= repeats <= 20:
                    raise ValueError(f"set [{index}]: repeat_count must be an integer from 1 to 20")
                params = {
                    "frequency_hz": frequency,
                    "duty_permyriad": duty,
                    "window_position_units": window,
                }
                failed_constraint = next(
                    (constraint for constraint in constraints if not constraint.check(params)),
                    None,
                )
                if failed_constraint:
                    raise ValueError(
                        f"set [{index}] violates {failed_constraint.type} "
                        f"(max {failed_constraint.max_value})"
                    )
                entry = {
                    **params,
                    "duration_s": float(duration),
                    "repeat_count": repeats,
                }
                if experiment_type == "dispense":
                    entry["target_mg"] = spec.target_mg
                    entry["preset_id"] = spec.preset_id or "fast-80hz-p200"
                sets.append(entry)
        else:
            sets, warnings = expand_grid(spec)
        if not sets:
            raise ValueError(
                f"No valid parameter sets after expansion. "
                f"Warnings: {'; '.join(warnings) if warnings else 'none'}"
            )

        auto_collect = bool(payload.get("auto_collect", False))
        powder_id = spec.powder_id

        # Delegate to existing batch runner
        batch_limit = 20
        if len(sets) > batch_limit:
            raise ValueError(
                f"Grid expands to {len(sets)} sets (limit {batch_limit}); "
                f"split into smaller sweeps"
            )

        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("a device test is already running")

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            uid = uuid.uuid4().hex[:6]
            run_id = f"grid-{spec.experiment_id}-{timestamp}-{uid}"
            self._cancel_event.clear()

            submitted = {
                "test_type": f"{experiment_type}_batch",
                "target_rate_mg_s": spec.target_rate_mg_s,
                "tare_between_sets": spec.tare_between_sets,
                "set_count": len(sets),
                "sets": sets,
                "orchestrator_meta": {
                    "powder_id": powder_id,
                    "auto_collect": auto_collect,
                    "warnings": warnings,
                },
            }
            self._set_test({"status": "running", "run_id": run_id, "submitted": submitted})

        if experiment_type == "constant_rate":
            worker = threading.Thread(
                target=self._run_grid_constant_rate,
                args=(run_id, submitted, powder_id, auto_collect),
                daemon=True,
            )
        else:
            worker = threading.Thread(
                target=self._run_constant_rate_batch,
                args=(run_id, submitted),
                daemon=True,
            )
        worker.start()
        return {
            "accepted": True,
            "run_id": run_id,
            "expanded_sets": len(sets),
            "warnings": warnings,
        }

    def _run_grid_constant_rate(self, run_id, submitted, powder_id, auto_collect):
        """Run a constant-rate batch then optionally collect results."""
        try:
            self._run_constant_rate_batch(run_id, submitted)
        finally:
            if auto_collect:
                try:
                    snap = self.snapshot()
                    batch_result = snap.get("test", {}).get("result", {})
                    if batch_result and batch_result.get("set_results"):
                        library_path = str(
                            PROJECT_ROOT / "data" / "powder_library"
                        )
                        collect_scan_results(
                            batch_result, powder_id, library_path
                        )
                except Exception:
                    pass  # collection failure must not mask the batch result

    def get_grid_templates(self):
        """Return the pre-defined reference-powder scan templates."""
        templates = {}
        for key, spec in REFERENCE_POWDER_SCANS.items():
            templates[key] = {
                "experiment_id": spec.experiment_id,
                "powder_name": spec.powder_name,
                "powder_id": spec.powder_id,
                "experiment_type": spec.experiment_type,
                "dimensions": [
                    {"name": d.name, "values": d.values} for d in spec.dimensions
                ],
                "constraints": [
                    {"type": c.type, "max_value": c.max_value}
                    for c in spec.constraints
                ],
                "duration_s": spec.duration_s,
                "repeat_count": spec.repeat_count,
                "tare_between_sets": spec.tare_between_sets,
            }
        return {"templates": templates}

    def collect_grid_to_library(self, payload):
        """Manually collect a completed batch result into powder_library."""
        run_id = payload.get("run_id", "")
        powder_id = payload.get("powder_id", "")
        if not run_id or not powder_id:
            raise ValueError("run_id and powder_id are required")
        snap = self.snapshot()
        batch_result = snap.get("test", {}).get("result", {})
        if not batch_result or batch_result.get("run_id") != run_id:
            # Try to load from disk
            log_files = sorted(
                (PROJECT_ROOT / "logs").glob(f"constant_rate_batch_{run_id}*.json"),
                reverse=True,
            )
            if not log_files:
                raise FileNotFoundError(f"no log file found for run_id {run_id}")
            batch_result = json.loads(log_files[0].read_text(encoding="utf-8"))
        library_path = str(PROJECT_ROOT / "data" / "powder_library")
        workspace = collect_scan_results(batch_result, powder_id, library_path)
        entries = len(workspace.get("scan_grid", []))
        return {"collected": True, "powder_id": powder_id, "scan_grid_entries": entries}

    def save_latest_test_result(self):
        """Persist the latest cached result, including recovery after a log error."""
        result = self.snapshot().get("test", {}).get("result")
        if not isinstance(result, dict) or not result.get("run_id"):
            raise RuntimeError("there is no test result to save")
        prefix = (
            "window_position_sweep"
            if result.get("test_type") == "window_position_sweep"
            else "recovered_test"
        )
        log_path = PROJECT_ROOT / "logs" / f"{prefix}_{result['run_id']}_recovered.json"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2, default=json_default),
            encoding="utf-8",
        )
        return {"saved": True, "log_path": str(log_path)}

    def write_la10_register(self, payload):
        """Write one operator-writable LA10 register while no test is running."""
        try:
            address = int(payload["address"])
            value = int(payload["value"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("address and value must be integers") from error
        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("manual register writes are locked while a test is running")
            with self.reader._lock:
                la10 = self.reader._la10
                if address == la10.VIBRATION_ENABLE_REGISTER:
                    if value == 1:
                        settings = la10.read_vibration_settings()
                        la10.start_vibration(
                            frequency_hz=settings.frequency_hz,
                            duty_permyriad=settings.duty_permyriad,
                        )
                    elif value == 0:
                        la10.stop_vibration()
                    else:
                        la10.write_manual_register(address, value)
                else:
                    la10.write_manual_register(address, value)
                raw = la10.read_holding_registers(address, 1)
                applied_value = int.from_bytes(raw, "big")
        # Patch the cached snapshot directly with the confirmed value
        # instead of calling _refresh().  A full 15-register re-read can
        # return stale data from the ESP32 gateway when it fires too soon
        # after a single-register write, causing the dashboard to revert
        # the operator's edit until the write is retried several times.
        self._patch_la10_snapshot(address, applied_value)
        return {"accepted": True, "address": address, "value": applied_value}

    def move_window_to_minimum(self):
        """Move the window to its minimum configured safe position while idle."""
        with self._test_lock:
            if self.snapshot()["test"].get("status") == "running":
                raise RuntimeError("cannot move the window while a device task is running")
            close_result = self._move_window_to_minimum()
        self._refresh()
        return {
            "accepted": True,
            "target_position_units": WINDOW_MIN_POSITION_UNITS,
            **close_result,
        }

    def _move_window_to_minimum(self):
        """Close the window, accepting an encoder value beyond the close limit.

        The mechanical close stop can report slightly below the configured
        minimum coordinate (for example target 100, actual 56).  That is a
        safely closed state, not an accuracy failure.  This helper is safe for
        use between batch repetitions because it does not require the overall
        batch task to be idle.
        """
        with self.reader._lock:
            status = self.reader._la10.read_status()
        if status.is_running:
            # The gateway can retain the running bit briefly after the
            # preceding motion has completed. Match the normal move path:
            # wait for confirmed idle before issuing the close command.
            status = self._wait_for_window_idle(5.0, respect_cancel=False)
        with self.reader._lock:
            vibration = self.reader._la10.read_vibration_settings()
        if vibration.enabled:
            raise RuntimeError("stop vibration before closing the window")
        if status.position_units <= WINDOW_MIN_POSITION_UNITS:
            return {
                "before_position_units": status.position_units,
                "after_position_units": status.position_units,
                "distance_register": 0,
                "already_closed": True,
            }
        before, after, distance_register = self._move_window_to(
            WINDOW_MIN_POSITION_UNITS,
            speed_mm_s=1.0,
            timeout_s=20.0,
            respect_cancel=False,
            accept_at_or_below_target=True,
        )
        return {
            "before_position_units": before.position_units,
            "after_position_units": after.position_units,
            "distance_register": distance_register,
            "already_closed": False,
        }

    def _patch_la10_snapshot(self, address, value):
        """Update one LA10 register value inside the cached snapshot."""
        with self._snapshot_lock:
            la10 = dict(self._snapshot.get("la10", {}))
            if not la10.get("online"):
                return
            registers = la10.get("registers", [])
            for reg in registers:
                if reg["address"] == address:
                    reg["value"] = value
                    reg["display"] = DeviceReader._la10_display(address, value)
                    break
            # Keep derived vibration-enabled flag in sync
            if address == self.reader._la10.VIBRATION_ENABLE_REGISTER:
                la10["vibration_enabled"] = value == 1
            self._snapshot["la10"] = la10

    def list_test_records(self, test_type=None, date_from=None, date_to=None,
                          result_filter=None, limit=200):
        """Return the newest matching log records, capped only after filtering."""
        logs_dir = PROJECT_ROOT / "logs"
        records = []
        parsed_logs = []

        # File names contain test type and other parameters, so lexical order
        # is neither chronological nor useful for a "latest records" view.
        # Parse every JSON first, then order by the recorded finish time with
        # the run-id timestamp as a fallback.
        for f in logs_dir.glob("*.json"):
            try:
                d = json.loads(f.read_text(encoding="utf-8"))
            except Exception:
                continue
            parsed_logs.append((self._record_sort_timestamp(d), f.name, d))

        for _timestamp, fname, d in sorted(parsed_logs, reverse=True):

            # --- infer test_type ---
            fname = f.name
            if "test_type" in d:
                tt = d["test_type"]
            elif fname.startswith("vibration_feed_"):
                tt = "vibration_feed"
            elif fname.startswith("window_position_sweep_"):
                tt = "window_position_sweep"
            elif fname.startswith("feedback_dispense_"):
                tt = "feedback_dispense"
                # Distinguish 100mg / 300mg / 500mg variants from run_id
                run_id_for_type = d.get("run_id", "")
                if "100mg" in run_id_for_type:
                    tt = "feedback_dispense_100mg"
                elif "300mg" in run_id_for_type:
                    tt = "feedback_dispense_300mg"
                elif "500mg" in run_id_for_type:
                    tt = "feedback_dispense_500mg"
            elif fname.startswith("window_pid_dispense_"):
                tt = "window_pid_dispense"
            elif fname.startswith("constant_rate_batch_"):
                tt = "constant_rate_batch"
            elif fname.startswith("constant_rate_dispense_"):
                tt = "constant_rate_dispense"
            elif "drift" in fname:
                tt = "at8811c_drift"
            else:
                tt = "unknown"

            # Refine feedback_dispense into 100mg/300mg/500mg variant
            if tt == "feedback_dispense":
                run_id_for_type = d.get("run_id", "")
                if "100mg" in run_id_for_type:
                    tt = "feedback_dispense_100mg"
                elif "300mg" in run_id_for_type:
                    tt = "feedback_dispense_300mg"
                elif "500mg" in run_id_for_type:
                    tt = "feedback_dispense_500mg"

            if test_type and tt != test_type:
                continue

            # --- extract date ---
            run_id = d.get("run_id", "")
            date_str = self._extract_date_from_run_id(run_id)
            if not date_str:
                finished = d.get("finished_at_utc", "")
                if finished:
                    try:
                        date_str = datetime.fromisoformat(finished).strftime("%Y%m%d")
                    except Exception:
                        pass

            if date_from and date_str and date_str < date_from:
                continue
            if date_to and date_str and date_str > date_to:
                continue

            # --- determine pass/fail ---
            result = d.get("result", "completed")
            error = d.get("error")
            final_mass = d.get("final_mass_mg")
            accept_min = d.get("acceptance_min_mg")
            accept_max = d.get("acceptance_max_mg")

            if result in ("failed", "cancelled") or error:
                passed = False
                status = "cancelled" if result == "cancelled" and not error else "failed"
            elif tt in ("constant_rate_dispense", "constant_rate_batch"):
                # constant-rate tests use summary.overall_passed
                summary = d.get("summary") or {}
                passed = summary.get("overall_passed", False)
                status = "passed" if passed else "failed"
            elif accept_min is not None and accept_max is not None and final_mass is not None:
                passed = accept_min <= final_mass <= accept_max
                status = "passed" if passed else "failed"
            else:
                passed = True
                status = "passed"

            if result_filter == "passed" and not passed:
                continue
            if result_filter == "failed" and passed:
                continue

            # --- build summary row ---
            cr_summary = (d.get("summary") or {}) if tt in ("constant_rate_dispense", "constant_rate_batch") else {}
            records.append({
                "run_id": run_id,
                "test_type": tt,
                "powder_id": d.get("powder_id"),
                "powder_name": d.get("powder_name"),
                "date": date_str or "",
                "status": status,
                "result": result,
                "error": error,
                "final_mass_mg": final_mass,
                "target_mass_mg": d.get("target_mass_mg"),
                "acceptance_min_mg": accept_min,
                "acceptance_max_mg": accept_max,
                "actual_duration_s": d.get("actual_duration_s"),
                "frequency_hz": d.get("frequency_hz")
                    or (d.get("initial_parameters") or {}).get("frequency_hz"),
                "duty_permyriad": d.get("duty_permyriad")
                    or (d.get("initial_parameters") or {}).get("duty_permyriad"),
                "preset_id": d.get("preset_id"),
                "mass_gain_mg": d.get("mass_gain_from_tare_mg") or d.get("final_mass_mg"),
                "segments_count": len(d.get("segments", [])),
                "samples_count": len(d.get("samples", [])),
                "note": d.get("note") or d.get("stop_reason") or "",
                # constant-rate specific
                "overall_mean_rate_mg_s": cr_summary.get("overall_mean_rate_mg_s"),
                "overall_cv_pct": cr_summary.get("overall_cv_pct"),
                "target_rate_mg_s": d.get("target_rate_mg_s"),
                "startup_delay_s": d.get("startup_delay_s"),
                "repeat_count": d.get("repeat_count"),
            })
            if len(records) >= limit:
                break

        # --- compute summary stats ---
        passed_records = [r for r in records if r["status"] == "passed"]
        failed_records = [r for r in records if r["status"] == "failed"]
        masses = [r["final_mass_mg"] for r in records
                  if r["final_mass_mg"] is not None]

        summary = {
            "total": len(records),
            "passed": len(passed_records),
            "failed": len(failed_records),
            "pass_rate_pct": round(len(passed_records) / len(records) * 100, 1)
                if records else 0,
        }
        if masses:
            summary["avg_final_mass_mg"] = round(sum(masses) / len(masses), 1)
            summary["min_final_mass_mg"] = round(min(masses), 1)
            summary["max_final_mass_mg"] = round(max(masses), 1)

        return {"records": records, "summary": summary}

    @staticmethod
    def _record_sort_timestamp(record):
        """Return an aware timestamp from finish time, or the run-id fallback."""
        finished = record.get("finished_at_utc")
        if isinstance(finished, str) and finished:
            try:
                value = datetime.fromisoformat(finished.replace("Z", "+00:00"))
                if value.tzinfo is None:
                    value = value.replace(tzinfo=timezone.utc)
                return value.astimezone(timezone.utc)
            except ValueError:
                pass

        run_id = record.get("run_id", "")
        import re
        match = re.search(r"(\d{8})_(\d{6})", run_id)
        if match:
            try:
                return datetime.strptime(
                    "".join(match.groups()), "%Y%m%d%H%M%S"
                ).replace(tzinfo=timezone.utc)
            except ValueError:
                pass
        return datetime.min.replace(tzinfo=timezone.utc)

    @staticmethod
    def _extract_date_from_run_id(run_id):
        """Extract YYYYMMDD date from run_id like 'dispense-500mg-20260724_100222-721c30'."""
        import re
        m = re.search(r'(\d{8})_\d{6}', run_id)
        return m.group(1) if m else None

    def _poll_loop(self):
        while not self._stop_event.is_set():
            started = time.monotonic()
            # Active tasks own device sampling. Running the full dashboard poll
            # concurrently adds duplicate Modbus traffic and can delay a balance
            # sample by several seconds while diagnostic registers are read.
            with self._snapshot_lock:
                task_running = self._snapshot["test"].get("status") == "running"
            if not task_running:
                self._refresh()
            self._stop_event.wait(max(0, POLL_INTERVAL_S - (time.monotonic() - started)))

    def _refresh(self):
        device_snapshot = self.reader.snapshot()
        with self._snapshot_lock:
            test = self._snapshot["test"]
            self._snapshot = {**device_snapshot, "test": test}

    def _set_test(self, test):
        with self._snapshot_lock:
            self._snapshot["test"] = test
            latest_sample = test.get("latest_sample")
            if isinstance(latest_sample, dict) and latest_sample.get("mass_mg") is not None:
                balance = dict(self._snapshot.get("at8811c", {}))
                balance.update(
                    {
                        "online": True,
                        "error": None,
                        "raw_weight_mg": latest_sample["mass_mg"],
                    }
                )
                if latest_sample.get("stable") is not None:
                    balance["stable"] = latest_sample["stable"]
                if latest_sample.get("status_word") is not None:
                    balance["status_word"] = latest_sample["status_word"]
                self._snapshot["at8811c"] = balance

    def _publish_post_stop_sample(self, run_id, result, sample):
        """Expose each settling read to the live snapshot after vibration stops."""
        live_sample = {
            **sample,
            "stage": "final_settle",
            "reason": "vibration stopped; continuously monitoring balance",
        }
        self._set_test(
            {
                "status": "running",
                "run_id": run_id,
                "phase": "final_settle",
                "latest_sample": live_sample,
                "result": result,
            }
        )

    def _run_feed_test(self, run_id, payload):
        result = {
            "run_id": run_id,
            "target_window_position_units": payload["target_window_position_units"],
            "frequency_hz": payload["frequency_hz"],
            "duty_permyriad": payload["duty_permyriad"],
            "requested_duration_s": payload["duration_s"],
            "duty_schedule": payload.get("duty_schedule", [payload["duty_permyriad"]]),
            "segment_duration_s": payload.get("segment_duration_s", payload["duration_s"]),
            "samples": [],
            "segments": [],
            "duty_changes": [],
        }
        vibration_started = False
        started_at = None
        try:
            with self.reader._lock:
                window_before = self.reader._la10.read_status()
                vibration_before = self.reader._la10.read_vibration_settings()
            if window_before.is_running:
                raise RuntimeError("Cannot tare while the LA10 window is moving")
            if window_before.fault_bits:
                raise RuntimeError(f"LA10 fault bits before motion: 0x{window_before.fault_bits:04X}")
            if vibration_before.enabled:
                with self.reader._lock:
                    self.reader._la10.stop_vibration()
                raise RuntimeError("Vibration was enabled; it has been stopped. Start a new test after inspection")

            self._set_test({"status": "running", "run_id": run_id, "phase": "tare", "result": result})
            with self.reader._lock:
                tare_before = self.reader._at.read_measurement()
                self.reader._at.zero(authorized=True)
            result["tare_before"] = tare_before.__dict__
            tare_after = self._wait_for_stable_measurement(TARE_TIMEOUT_S)
            result["tare_after"] = tare_after.__dict__

            current = window_before.position_units
            delta = abs(payload["target_window_position_units"] - current)
            distance_register = motion_distance_register(delta)
            result.update(
                {
                    "window_before": window_before.__dict__,
                    "window_distance_register": distance_register,
                    "window_motion_write_function": "0x10",
                    "pre_motion_fault_bits": window_before.fault_bits,
                }
            )
            if delta > POSITION_TOLERANCE_UNITS:
                extend = payload["target_window_position_units"] > current
                with self.reader._lock:
                    self.reader._la10.enable()
                    self.reader._la10.move_relative(
                        extend=extend,
                        distance_mm=distance_register / MOTION_REGISTER_UNITS_PER_MM,
                        speed_mm_s=payload["window_speed_mm_s"],
                    )
                self._set_test({"status": "running", "run_id": run_id, "phase": "window_move", "result": result})
                window_after = self._wait_for_window_move(payload["window_timeout_s"])
            else:
                distance_register = 0
                result["window_distance_register"] = 0
                window_after = window_before
                result["window_after"] = window_after.__dict__
                result["post_motion_fault_bits"] = window_after.fault_bits
                result["post_motion_last_result"] = window_after.last_result
                if abs(window_after.position_units - payload["target_window_position_units"]) > POSITION_TOLERANCE_UNITS:
                    raise RuntimeError(
                        f"窗口未到位：实际 {window_after.position_units}，目标 "
                        f"{payload['target_window_position_units']}，容差 {POSITION_TOLERANCE_UNITS}"
                    )
            result["window_after"] = window_after.__dict__
            result["post_motion_fault_bits"] = window_after.fault_bits
            result["post_motion_last_result"] = window_after.last_result
            if abs(window_after.position_units - payload["target_window_position_units"]) > POSITION_TOLERANCE_UNITS:
                raise RuntimeError("window is outside the approved position tolerance")
            result["window_verified"] = True
            duty_schedule = [int(value) for value in result["duty_schedule"]]
            segment_duration_s = float(result["segment_duration_s"])
            motion_estimator = MassMotionEstimator()
            with self.reader._lock:
                self.reader._la10.start_vibration(
                    frequency_hz=payload["frequency_hz"],
                    duty_permyriad=duty_schedule[0],
                )
            # From this point the finally block owns the physical stop command,
            # including when the readback check below fails.
            vibration_started = True
            with self.reader._lock:
                vibration_started_settings = self.reader._la10.read_vibration_settings()
                control_word = int.from_bytes(
                    self.reader._la10.read_holding_registers(0, 1), "big"
                )
            result["vibration_started"] = {
                "frequency_hz": vibration_started_settings.frequency_hz,
                "duty_permyriad": vibration_started_settings.duty_permyriad,
                "enabled": vibration_started_settings.enabled,
                "control_word": control_word,
            }
            if not vibration_started_settings.enabled:
                raise RuntimeError("LA10 did not confirm vibration enable after start")
            started_at = time.monotonic()
            for segment_index, duty_permyriad in enumerate(duty_schedule, start=1):
                segment_started_at = time.monotonic()
                segment_samples = []
                self._set_test(
                    {
                        "status": "running",
                        "run_id": run_id,
                        "phase": "vibrating",
                        "segment_index": segment_index,
                        "segment_count": len(duty_schedule),
                        "duty_permyriad": duty_permyriad,
                        "result": result,
                    }
                )
                while time.monotonic() - segment_started_at < segment_duration_s:
                    if self._cancel_event.is_set():
                        result["result"] = "cancelled"
                        break
                    sample_time = time.monotonic()
                    with self.reader._lock:
                        measurement = self.reader._at.read_measurement()
                    # Some AT8811C firmware revisions expose the calibrated
                    # mass field as None while raw_count remains valid. The
                    # project scale is 1 mg/count, so retain the raw value.
                    measured_mass = (
                        measurement.mass_mg
                        if measurement.mass_mg is not None
                        else measurement.raw_count
                    )
                    mass_mg = max(0.0, float(measured_mass))
                    motion = motion_estimator.update(sample_time, mass_mg)
                    sample = {
                        "elapsed_s": round(sample_time - started_at, 3),
                        "duty_permyriad": duty_permyriad,
                        "frequency_hz": payload["frequency_hz"],
                        "mass_mg": round(mass_mg, 3),
                        "raw_count": measurement.raw_count,
                        "stable": measurement.stable,
                        "status_word": measurement.status_word,
                        "filtered_mass_mg": round(motion.filtered_mass_mg, 3),
                        "rate_valid": motion.valid,
                        "rate_mg_s": round(motion.rate_mg_s, 3),
                        "acceleration_mg_s2": round(motion.acceleration_mg_s2, 3),
                        "jerk_mg_s3": round(motion.jerk_mg_s3, 3),
                        "predicted_mass_mg": round(motion.predicted_mass_mg, 3),
                        "predicted_rate_mg_s": round(motion.predicted_rate_mg_s, 3),
                    }
                    result["samples"].append(sample)
                    segment_samples.append(sample)
                    self._set_test(
                        {
                            "status": "running",
                            "run_id": run_id,
                            "phase": "vibrating",
                            "segment_index": segment_index,
                            "segment_count": len(duty_schedule),
                            "duty_permyriad": duty_permyriad,
                            "latest_sample": sample,
                            "result": result,
                        }
                    )
                    wait_s = max(0.0, POLL_INTERVAL_S - (time.monotonic() - sample_time))
                    if self._cancel_event.wait(wait_s):
                        result["result"] = "cancelled"
                        break
                if result.get("result") == "cancelled":
                    break
                valid_rates = [item["rate_mg_s"] for item in segment_samples if item["rate_valid"]]
                result["segments"].append(
                    {
                        "segment_index": segment_index,
                        "duty_permyriad": duty_permyriad,
                        "started_elapsed_s": round(segment_started_at - started_at, 3),
                        "actual_duration_s": round(time.monotonic() - segment_started_at, 3),
                        "sample_count": len(segment_samples),
                        "mass_before_mg": segment_samples[0]["mass_mg"] if segment_samples else None,
                        "mass_after_mg": segment_samples[-1]["mass_mg"] if segment_samples else None,
                        "mass_gain_mg": round(
                            segment_samples[-1]["mass_mg"] - segment_samples[0]["mass_mg"], 3
                        ) if segment_samples else None,
                        "mean_rate_mg_s": round(sum(valid_rates) / max(1, len(valid_rates)), 3),
                    }
                )
                if segment_index < len(duty_schedule):
                    next_duty = duty_schedule[segment_index]
                    with self.reader._lock:
                        self.reader._la10.update_vibration_parameters(
                            frequency_hz=payload["frequency_hz"],
                            duty_permyriad=next_duty,
                        )
                    result["duty_changes"].append(
                        {
                            "elapsed_s": round(time.monotonic() - started_at, 3),
                            "duty_before": duty_permyriad,
                            "duty_after": next_duty,
                            "frequency_hz": payload["frequency_hz"],
                        }
                    )
            if result.get("result") != "cancelled":
                result["result"] = "completed"
            result["actual_duration_s"] = round(time.monotonic() - started_at, 3)
        except Exception as error:
            result["result"] = "failed"
            result["error"] = str(error)
        finally:
            if vibration_started:
                try:
                    with self.reader._lock:
                        self.reader._la10.stop_vibration()
                        vibration_after = self.reader._la10.read_vibration_settings()
                    result["actual_duration_s"] = round(time.monotonic() - started_at, 3)
                    result["vibration_after"] = vibration_after.__dict__
                    if vibration_after.enabled:
                        result["result"] = "failed"
                        result["stop_error"] = "LA10 vibration enable register remained set after stop command"
                    else:
                        self._set_test(
                            {"status": "running", "run_id": run_id, "phase": "post_vibration_settle", "result": result}
                        )
                        time.sleep(POST_VIBRATION_SETTLE_S)
                        mass_after_vibration = self._wait_for_stable_measurement(TARE_TIMEOUT_S)
                        result["mass_after_vibration"] = mass_after_vibration.__dict__
                        result["mass_gain_from_tare_mg"] = (
                            mass_after_vibration.raw_count - result["tare_after"]["raw_count"]
                        )
                except Exception as error:
                    result["result"] = "failed"
                    result["post_vibration_measurement_error"] = str(error)
            result["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            log_path = PROJECT_ROOT / "logs" / f"vibration_feed_{run_id}.json"
            try:
                log_path.parent.mkdir(parents=True, exist_ok=True)
                log_path.write_text(
                    json.dumps(result, ensure_ascii=False, indent=2, default=json_default),
                    encoding="utf-8",
                )
                result["log_path"] = str(log_path)
            except Exception as error:
                result["log_error"] = str(error)
            self._set_test({"status": result["result"], "run_id": run_id, "result": result})

    def _run_window_pid_test(self, run_id):
        config = WindowPidConfig(
            target_mass_mg=DISPENSE_TARGET_MG,
            allowed_overweight_mg=15.0,
            frequency_hz=80,
            duty_permyriad=2000,
            recovery_duty_permyriad=2000,
        )
        planner = WindowPidPlanner(config)
        pid_state = WindowPidState()
        estimator = MassMotionEstimator()
        result = {
            "run_id": run_id,
            "test_type": "window_pid_dispense",
            "target_mass_mg": DISPENSE_TARGET_MG,
            "acceptance_min_mg": DISPENSE_MIN_ACCEPT_MG,
            "acceptance_max_mg": DISPENSE_HARD_LIMIT_MG,
            "controller": "feedforward_window_plus_pid_trim",
            "config": {
                "frequency_hz": config.frequency_hz,
                "duty_permyriad": config.duty_permyriad,
                "coarse_window_units": config.coarse.base_window_position_units,
                "slow_window_units": config.slow.base_window_position_units,
                "fine_window_units": config.fine.base_window_position_units,
                "tail_seconds": config.tail_seconds,
                "max_tail_mg": config.max_tail_mg,
            },
            "samples": [],
            "window_updates": [],
        }
        started_at = time.monotonic()
        previous_sample_at = None
        last_window_update_at = None
        vibration_started = False
        try:
            self._set_test({"status": "running", "run_id": run_id, "phase": "window_move", "result": result})
            before, after, distance = self._move_window_to(
                config.coarse.base_window_position_units,
                speed_mm_s=1.0,
                timeout_s=20.0,
            )
            result["window_before"] = before.__dict__
            result["window_after"] = after.__dict__
            result["window_distance_register"] = distance

            self._set_test({"status": "running", "run_id": run_id, "phase": "tare", "result": result})
            with self.reader._lock:
                tare_before = self.reader._at.read_measurement()
                self.reader._at.zero(authorized=True)
            result["tare_before"] = tare_before.__dict__
            result["tare_after"] = self._wait_for_stable_measurement(TARE_TIMEOUT_S).__dict__

            self._set_test({"status": "running", "run_id": run_id, "phase": "vibration_start", "result": result})
            vibration_started = True
            with self.reader._lock:
                self.reader._la10.start_vibration(
                    frequency_hz=config.frequency_hz,
                    duty_permyriad=config.duty_permyriad,
                )
                vibration = self.reader._la10.read_vibration_settings()
            if not vibration.enabled:
                raise RuntimeError("LA10 did not confirm vibration enable")
            result["vibration_started"] = vibration.__dict__

            while True:
                if self._cancel_event.is_set():
                    result["result"] = "cancelled"
                    break
                if time.monotonic() - started_at > 180.0:
                    raise TimeoutError("window PID test exceeded 180 seconds")
                sample_time = time.monotonic()
                dt_s = 0.25 if previous_sample_at is None else max(0.05, sample_time - previous_sample_at)
                previous_sample_at = sample_time
                with self.reader._lock:
                    measurement = self.reader._at.read_measurement()
                measured_mass = measurement.mass_mg if measurement.mass_mg is not None else measurement.raw_count
                mass_mg = max(0.0, float(measured_mass))
                if mass_mg > DISPENSE_HARD_LIMIT_MG:
                    raise RuntimeError(f"hard overweight boundary exceeded: {mass_mg:.1f} mg")
                motion = estimator.update(sample_time, mass_mg)
                observed_rate = motion.rate_mg_s if motion.valid else None
                decision = planner.next_decision(
                    current_mass_mg=mass_mg,
                    observed_rate_mg_s=observed_rate,
                    state=pid_state,
                    dt_s=dt_s,
                )
                pid_state = decision.next_state
                actual_window = None
                if decision.action is not None:
                    with self.reader._lock:
                        current_window = self.reader._la10.read_status().position_units
                    if (
                        abs(decision.action.window_position_units - current_window) > 5
                        and (
                            last_window_update_at is None
                            or sample_time - last_window_update_at >= ACTUATOR_UPDATE_INTERVAL_S
                        )
                    ):
                        window_before, window_after, distance = self._move_window_to(
                            decision.action.window_position_units,
                            speed_mm_s=1.0,
                            timeout_s=10.0,
                            position_tolerance_units=5,
                        )
                        result["window_updates"].append(
                            {
                                "elapsed_s": round(sample_time - started_at, 3),
                                "stage": decision.stage.value,
                                "before_units": window_before.position_units,
                                "requested_units": decision.action.window_position_units,
                                "after_units": window_after.position_units,
                                "pid_adjust_units": decision.pid_adjust_units,
                                "reason": decision.reason,
                            }
                        )
                        last_window_update_at = sample_time
                        actual_window = window_after.position_units
                    else:
                        actual_window = current_window
                sample = {
                    "elapsed_s": round(sample_time - started_at, 3),
                    "mass_mg": round(mass_mg, 3),
                    "filtered_mass_mg": round(motion.filtered_mass_mg, 3),
                    "rate_valid": motion.valid,
                    "rate_mg_s": round(motion.rate_mg_s, 3),
                    "acceleration_mg_s2": round(motion.acceleration_mg_s2, 3),
                    "jerk_mg_s3": round(motion.jerk_mg_s3, 3),
                    "stage": decision.stage.value,
                    "target_rate_mg_s": decision.target_rate_mg_s,
                    "predicted_tail_mg": round(decision.predicted_tail_mg, 3),
                    "remaining_mg": round(decision.remaining_mg, 3),
                    "pid_adjust_units": decision.pid_adjust_units,
                    "window_position_units": actual_window,
                    "planned_window_position_units": (
                        decision.action.window_position_units if decision.action is not None else None
                    ),
                    "frequency_hz": config.frequency_hz,
                    "duty_permyriad": config.duty_permyriad,
                    "planned_duration_ms": (
                        decision.action.duration_ms if decision.action is not None else None
                    ),
                    "reason": decision.reason,
                }
                result["samples"].append(sample)
                self._set_test(
                    {
                        "status": "running",
                        "run_id": run_id,
                        "phase": f"pid_{decision.stage.value}",
                        "latest_sample": sample,
                        "result": result,
                    }
                )
                if decision.action is None:
                    result["result"] = "completed"
                    result["stop_sample"] = sample
                    result["stop_reason"] = decision.reason
                    break
                self._cancel_event.wait(max(0, POLL_INTERVAL_S - (time.monotonic() - sample_time)))
        except Exception as error:
            result["result"] = "failed"
            result["error"] = str(error)
        finally:
            if vibration_started:
                try:
                    with self.reader._lock:
                        self.reader._la10.stop_vibration()
                    result["vibration_stopped_at_utc"] = datetime.now(timezone.utc).isoformat()
                except Exception as error:
                    result["result"] = "failed"
                    result["stop_error"] = str(error)
            if result.get("result") == "completed":
                try:
                    result["post_stop_samples"] = []
                    self._set_test({"status": "running", "run_id": run_id, "phase": "final_settle", "result": result})
                    final = self._wait_for_stable_measurement(
                        15.0,
                        minimum_wait_s=POST_VIBRATION_SETTLE_S,
                        continuous_stable_s=FINAL_STABILITY_WINDOW_S,
                        max_spread_mg=FINAL_STABILITY_MAX_SPREAD_MG,
                        trace=result["post_stop_samples"],
                        on_sample=lambda sample: self._publish_post_stop_sample(
                            run_id, result, sample
                        ),
                    )
                    final_mass = final.mass_mg if final.mass_mg is not None else final.raw_count
                    result["final_measurement"] = final.__dict__
                    result["final_mass_mg"] = float(final_mass)
                    result["final_settle_rule"] = {
                        "minimum_wait_s": POST_VIBRATION_SETTLE_S,
                        "continuous_stable_s": FINAL_STABILITY_WINDOW_S,
                        "max_spread_mg": FINAL_STABILITY_MAX_SPREAD_MG,
                    }
                    if not DISPENSE_MIN_ACCEPT_MG <= final_mass <= DISPENSE_HARD_LIMIT_MG:
                        result["result"] = "failed"
                        result["acceptance_error"] = (
                            f"final stable mass outside {DISPENSE_MIN_ACCEPT_MG:.0f}-"
                            f"{DISPENSE_HARD_LIMIT_MG:.0f} mg: {final_mass:.1f} mg"
                        )
                except Exception as error:
                    result["result"] = "failed"
                    result["final_stability_error"] = str(error)
            result["actual_duration_s"] = round(time.monotonic() - started_at, 3)
            result["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            self._persist_result("window_pid_dispense", run_id, result)
            self._set_test({"status": result["result"], "run_id": run_id, "result": result})

    def _run_continuous_taper_dispense(self, run_id):
        config = ContinuousTaperConfig()
        planner = ContinuousTaperPlanner(config)
        result = {
            "run_id": run_id,
            "test_type": "continuous_taper_dispense",
            "algorithm": "fixed-stage-continuous-taper",
            "algorithm_version": "continuous-taper-v1",
            "target_mass_mg": config.target_mass_mg,
            "hard_limit_mg": DISPENSE_HARD_LIMIT_MG,
            "config": {
                "coarse_end_mg": config.coarse_end_mg,
                "stop_raw_mg": config.stop_raw_mg,
                "stop_filtered_mg": config.stop_filtered_mg,
                "stop_prediction_mg": config.stop_prediction_mg,
                "min_raw_for_predictive_stop_mg": config.min_raw_for_predictive_stop_mg,
                "prediction_horizon_s": config.prediction_horizon_s,
                "fixed_tail_mg": config.fixed_tail_mg,
                "coarse": config.coarse.__dict__,
                "taper": config.taper.__dict__,
            },
            "samples": [],
            "control_updates": [],
        }
        started_at = time.monotonic()
        vibration_started = False
        estimator = MassMotionEstimator()
        current_action = config.coarse
        last_actuator_update_at = None
        try:
            self._set_test(
                {"status": "running", "run_id": run_id, "phase": "window_move", "result": result}
            )
            before, after, distance_register = self._move_window_to(
                current_action.window_position_units,
                speed_mm_s=1.0,
                timeout_s=20.0,
            )
            result["window_before"] = before.__dict__
            result["window_after"] = after.__dict__
            result["window_distance_register"] = distance_register

            self._set_test(
                {"status": "running", "run_id": run_id, "phase": "tare", "result": result}
            )
            with self.reader._lock:
                tare_before = self.reader._at.read_measurement()
                self.reader._at.zero(authorized=True)
            result["tare_before"] = tare_before.__dict__
            result["tare_after"] = self._wait_for_stable_measurement(TARE_TIMEOUT_S).__dict__

            self._set_test(
                {
                    "status": "running",
                    "run_id": run_id,
                    "phase": "vibration_start",
                    "result": result,
                }
            )
            vibration_started = True
            with self.reader._lock:
                self.reader._la10.start_vibration(
                    frequency_hz=current_action.frequency_hz,
                    duty_permyriad=current_action.duty_permyriad,
                )
                vibration = self.reader._la10.read_vibration_settings()
            if not vibration.enabled:
                raise RuntimeError("LA10 did not confirm vibration enable")
            result["vibration_started"] = vibration.__dict__

            while True:
                if self._cancel_event.is_set():
                    result["result"] = "cancelled"
                    break
                if time.monotonic() - started_at > 180.0:
                    raise TimeoutError("continuous taper dispense exceeded 180 seconds")

                read_started_at = time.monotonic()
                with self.reader._lock:
                    measurement = self.reader._at.read_measurement()
                sample_time = time.monotonic()
                measurement_read_s = sample_time - read_started_at
                measured_mass = (
                    measurement.mass_mg
                    if measurement.mass_mg is not None
                    else measurement.raw_count
                )
                mass_mg = max(0.0, float(measured_mass))
                if mass_mg > DISPENSE_HARD_LIMIT_MG:
                    raise RuntimeError(f"hard overweight boundary exceeded: {mass_mg:.1f} mg")

                motion = estimator.update(sample_time, mass_mg)
                decision = planner.decide(
                    raw_mass_mg=mass_mg,
                    filtered_mass_mg=motion.filtered_mass_mg,
                    rate_mg_s=motion.rate_mg_s,
                    acceleration_mg_s2=motion.acceleration_mg_s2,
                    predicted_mass_mg=motion.predicted_mass_mg,
                    current_window_position_units=current_action.window_position_units,
                    current_frequency_hz=current_action.frequency_hz,
                    current_duty_permyriad=current_action.duty_permyriad,
                )

                update = None
                if (
                    decision.command == "UPDATE"
                    and decision.action is not None
                    and (
                        last_actuator_update_at is None
                        or sample_time - last_actuator_update_at >= ACTUATOR_UPDATE_INTERVAL_S
                    )
                ):
                    requested_action = decision.action
                    update = {
                        "elapsed_s": round(sample_time - started_at, 3),
                        "mass_mg": round(mass_mg, 3),
                        "stage": decision.stage.value,
                        "before": current_action.__dict__,
                        "after": requested_action.__dict__,
                        "reason": decision.reason,
                    }
                    # Registers 12-13 are updated without touching vibration
                    # enable. The window then moves while vibration stays on.
                    with self.reader._lock:
                        self.reader._la10.update_vibration_parameters(
                            frequency_hz=requested_action.frequency_hz,
                            duty_permyriad=requested_action.duty_permyriad,
                        )
                    if requested_action.window_position_units != current_action.window_position_units:
                        move_before, move_after, move_distance = self._move_window_to(
                            requested_action.window_position_units,
                            speed_mm_s=1.0,
                            timeout_s=10.0,
                        )
                        update["window_motion"] = {
                            "before": move_before.position_units,
                            "after": move_after.position_units,
                            "requested": requested_action.window_position_units,
                            "distance_register": move_distance,
                        }
                    current_action = requested_action
                    last_actuator_update_at = sample_time
                    result["control_updates"].append(update)

                estimated_tail_mg = max(0.0, decision.predicted_mass_mg - mass_mg)
                sample = {
                    "elapsed_s": round(sample_time - started_at, 3),
                    "measurement_read_s": round(measurement_read_s, 4),
                    "mass_mg": round(mass_mg, 3),
                    "filtered_mass_mg": round(motion.filtered_mass_mg, 3),
                    "predicted_mass_mg": round(decision.predicted_mass_mg, 3),
                    "rate_valid": motion.valid,
                    "rate_mg_s": round(motion.rate_mg_s, 3),
                    "predicted_rate_mg_s": round(motion.predicted_rate_mg_s, 3),
                    "control_rate_mg_s": round(motion.rate_mg_s, 3),
                    "acceleration_mg_s2": round(motion.acceleration_mg_s2, 3),
                    "jerk_mg_s3": round(motion.jerk_mg_s3, 3),
                    "estimated_tail_mg": round(estimated_tail_mg, 3),
                    "projected_stop_mass_mg": round(decision.predicted_mass_mg, 3),
                    "remaining_predicted_mg": round(decision.remaining_predicted_mg, 3),
                    "stage": decision.stage.value,
                    "command": decision.command,
                    "frequency_hz": current_action.frequency_hz,
                    "duty_permyriad": current_action.duty_permyriad,
                    "window_position_units": current_action.window_position_units,
                    "stable": measurement.stable,
                    "status_word": measurement.status_word,
                    "reason": decision.reason,
                }
                result["samples"].append(sample)
                self._set_test(
                    {
                        "status": "running",
                        "run_id": run_id,
                        "phase": f"{decision.stage.value}_continuous_taper",
                        "latest_sample": sample,
                        "result": result,
                    }
                )
                if decision.command == "STOP":
                    result["result"] = "completed"
                    result["stop_sample"] = sample
                    result["stop_reason"] = decision.reason
                    break
                self._cancel_event.wait(
                    max(0, POLL_INTERVAL_S - (time.monotonic() - sample_time))
                )
        except Exception as error:
            result["result"] = "failed"
            result["error"] = str(error)
        finally:
            if vibration_started:
                try:
                    with self.reader._lock:
                        self.reader._la10.stop_vibration()
                    result["vibration_stopped_at_utc"] = datetime.now(timezone.utc).isoformat()
                except Exception as error:
                    result["result"] = "failed"
                    result["stop_error"] = str(error)
            if result.get("result") == "completed":
                try:
                    result["post_stop_samples"] = []
                    self._set_test(
                        {
                            "status": "running",
                            "run_id": run_id,
                            "phase": "final_settle",
                            "result": result,
                        }
                    )
                    final_measurement = self._wait_for_stable_measurement(
                        15.0,
                        minimum_wait_s=POST_VIBRATION_SETTLE_S,
                        continuous_stable_s=FINAL_STABILITY_WINDOW_S,
                        max_spread_mg=FINAL_STABILITY_MAX_SPREAD_MG,
                        trace=result["post_stop_samples"],
                        on_sample=lambda sample: self._publish_post_stop_sample(
                            run_id, result, sample
                        ),
                    )
                    result["final_measurement"] = final_measurement.__dict__
                    final_mass_mg = (
                        final_measurement.mass_mg
                        if final_measurement.mass_mg is not None
                        else final_measurement.raw_count
                    )
                    result["final_mass_mg"] = float(final_mass_mg)
                    result["final_settle_rule"] = {
                        "minimum_wait_s": POST_VIBRATION_SETTLE_S,
                        "continuous_stable_s": FINAL_STABILITY_WINDOW_S,
                        "max_spread_mg": FINAL_STABILITY_MAX_SPREAD_MG,
                    }
                    result["acceptance_min_mg"] = DISPENSE_MIN_ACCEPT_MG
                    result["acceptance_max_mg"] = DISPENSE_HARD_LIMIT_MG
                    if not DISPENSE_MIN_ACCEPT_MG <= final_mass_mg <= DISPENSE_HARD_LIMIT_MG:
                        result["result"] = "failed"
                        result["acceptance_error"] = (
                            f"final stable mass outside {DISPENSE_MIN_ACCEPT_MG:.0f}-"
                            f"{DISPENSE_HARD_LIMIT_MG:.0f} mg: {final_mass_mg:.1f} mg"
                        )
                except Exception as error:
                    result["result"] = "failed"
                    result["final_stability_error"] = str(error)
            result["actual_duration_s"] = round(time.monotonic() - started_at, 3)
            result["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            self._persist_result("continuous_taper_dispense", run_id, result)
            self._set_test({"status": result["result"], "run_id": run_id, "result": result})

    @staticmethod
    def _measurement_mass_mg(measurement):
        value = measurement.mass_mg if measurement.mass_mg is not None else measurement.raw_count
        return max(0.0, float(value))

    def _run_tail_pulse_stage(
        self,
        *,
        run_id,
        target_mg,
        acceptance_min_mg,
        acceptance_max_mg,
        config,
        initial_measurement,
        started_at,
        result,
    ):
        """Meter a heterogeneous tail using brief low-energy open/close pulses."""
        current_measurement = initial_measurement
        current_mass_mg = self._measurement_mass_mg(current_measurement)
        pulse_window = config["initial_window_position_units"]
        pulse_duration_s = config["initial_duration_s"]
        no_gain_attempts = 0
        pulse_mode = "standard"
        standard_attempts = 0
        micro_attempts = 0
        attempt = 0
        observed_max_gain_mg = 0.0
        result.setdefault("tail_pulse_events", [])

        def finish(status, reason, measurement=None):
            measurement = measurement or current_measurement
            return {
                "status": status,
                "reason": reason,
                "measurement": measurement,
                "mass_mg": self._measurement_mass_mg(measurement),
            }

        if current_mass_mg > acceptance_max_mg:
            return finish(
                "failed",
                f"tail pulse transition settled above {acceptance_max_mg:.0f} mg: "
                f"{current_mass_mg:.1f} mg",
            )
        if current_mass_mg >= acceptance_min_mg:
            return finish(
                "completed",
                "continuous-stage tail settled inside the acceptance window; no pulse required",
            )

        while True:
            if self._cancel_event.is_set():
                return finish("cancelled", "tail pulse stage cancelled", current_measurement)

            headroom_mg = acceptance_max_mg - current_mass_mg
            guard_gain_mg = max(
                config["safety_guard_gain_mg"],
                observed_max_gain_mg * config["safety_guard_multiplier"],
            )
            if pulse_mode == "standard" and headroom_mg <= guard_gain_mg:
                pulse_mode = "micro"
                pulse_window = config["micro_initial_window_position_units"]
                pulse_duration_s = config["micro_initial_duration_s"]
                no_gain_attempts = 0
                result["tail_pulse_safety_gate"] = {
                    "entered_at_mass_mg": round(current_mass_mg, 3),
                    "headroom_to_upper_limit_mg": round(headroom_mg, 3),
                    "guard_gain_mg": round(guard_gain_mg, 3),
                    "maximum_standard_window_position_units": config[
                        "max_window_position_units"
                    ],
                    "maximum_micro_window_position_units": config[
                        "micro_max_window_position_units"
                    ],
                    "reason": (
                        "standard pulse prohibited because the remaining upper-limit "
                        "headroom cannot safely absorb a recorded breakout pulse"
                    ),
                }

            if pulse_mode == "standard":
                if standard_attempts >= config["max_attempts"]:
                    return finish(
                        "failed",
                        f"standard tail pulse exhausted {config['max_attempts']} attempts "
                        f"below {acceptance_min_mg:.0f} mg",
                    )
                standard_attempts += 1
            else:
                if micro_attempts >= config["micro_max_attempts"]:
                    return finish(
                        "failed",
                        f"safe micro pulse exhausted {config['micro_max_attempts']} attempts "
                        f"at {current_mass_mg:.1f} mg; p380 was prohibited",
                    )
                micro_attempts += 1
            attempt += 1
            mass_before_mg = current_mass_mg
            opened_before = opened_after = closed_before = closed_after = None
            open_distance = close_distance = 0
            pulse_vibration_started = False
            pulse_cancelled = False
            pulse_started_at = time.monotonic()
            try:
                opened_before, opened_after, open_distance = self._move_window_to(
                    pulse_window,
                    speed_mm_s=1.0,
                    timeout_s=5.0,
                )
                with self.reader._lock:
                    self.reader._la10.start_vibration(
                        frequency_hz=config["frequency_hz"],
                        duty_permyriad=config["duty_permyriad"],
                    )
                    pulse_vibration_started = True
                    vibration = self.reader._la10.read_vibration_settings()
                if not vibration.enabled:
                    raise RuntimeError("LA10 did not confirm tail-pulse vibration enable")
                pulse_cancelled = self._cancel_event.wait(pulse_duration_s)
            finally:
                try:
                    if pulse_vibration_started:
                        with self.reader._lock:
                            self.reader._la10.stop_vibration()
                finally:
                    closed_before, closed_after, close_distance = self._move_window_to(
                        config["hold_window_position_units"],
                        speed_mm_s=1.0,
                        timeout_s=5.0,
                        respect_cancel=False,
                    )

            if pulse_cancelled or self._cancel_event.is_set():
                return finish("cancelled", "tail pulse stage cancelled", current_measurement)

            settle_trace = []
            current_measurement = self._wait_for_stable_measurement(
                config["settle_timeout_s"],
                minimum_wait_s=config["settle_minimum_wait_s"],
                continuous_stable_s=config["settle_continuous_stable_s"],
                max_spread_mg=config["settle_max_spread_mg"],
                trace=settle_trace,
            )
            current_mass_mg = self._measurement_mass_mg(current_measurement)
            gain_mg = current_mass_mg - mass_before_mg
            event = {
                "attempt": attempt,
                "pulse_mode": pulse_mode,
                "standard_attempt": standard_attempts,
                "micro_attempt": micro_attempts,
                "elapsed_s": round(time.monotonic() - started_at, 3),
                "headroom_to_upper_limit_before_mg": round(headroom_mg, 3),
                "guard_gain_mg": round(guard_gain_mg, 3),
                "pulse_window_position_units": pulse_window,
                "pulse_duration_s": round(pulse_duration_s, 3),
                "frequency_hz": config["frequency_hz"],
                "duty_permyriad": config["duty_permyriad"],
                "mass_before_mg": round(mass_before_mg, 3),
                "mass_after_mg": round(current_mass_mg, 3),
                "gain_mg": round(gain_mg, 3),
                "open_window_before_units": opened_before.position_units,
                "open_window_after_units": opened_after.position_units,
                "open_distance_register": open_distance,
                "close_window_before_units": closed_before.position_units,
                "close_window_after_units": closed_after.position_units,
                "close_distance_register": close_distance,
                "pulse_command_elapsed_s": round(time.monotonic() - pulse_started_at, 3),
                "settle_samples": settle_trace,
            }
            result["tail_pulse_events"].append(event)
            reason = (
                f"tail pulse {attempt} ({pulse_mode}): {mass_before_mg:.1f} -> "
                f"{current_mass_mg:.1f} mg (gain {gain_mg:.1f} mg)"
            )
            sample = {
                "elapsed_s": round(time.monotonic() - started_at, 3),
                "measurement_read_s": 0.0,
                "mass_mg": round(current_mass_mg, 3),
                "filtered_mass_mg": round(current_mass_mg, 3),
                "predicted_mass_mg": round(current_mass_mg, 3),
                "rate_valid": False,
                "rate_mg_s": 0.0,
                "predicted_rate_mg_s": 0.0,
                "control_rate_mg_s": 0.0,
                "acceleration_mg_s2": 0.0,
                "jerk_mg_s3": 0.0,
                "estimated_tail_mg": 0.0,
                "projected_stop_mass_mg": round(current_mass_mg, 3),
                "predictive_stop_candidate": False,
                "stop_confirmation_count": 0,
                "stop_confirmation_required": 0,
                "stall_recovery_active": False,
                "stall_recovery_attempt": 0,
                "stage": (
                    "tail_pulse_micro" if pulse_mode == "micro" else "tail_pulse"
                ),
                "target_rate_mg_s": 0.0,
                "frequency_hz": config["frequency_hz"],
                "duty_permyriad": 0,
                "window_position_units": config["hold_window_position_units"],
                "stable": current_measurement.stable,
                "status_word": current_measurement.status_word,
                "reason": reason,
            }
            result["samples"].append(sample)
            self._set_test(
                {
                    "status": "running",
                    "run_id": run_id,
                    "phase": (
                        "tail_pulse_micro" if pulse_mode == "micro" else "tail_pulse"
                    ),
                    "latest_sample": sample,
                    "result": result,
                }
            )

            if current_mass_mg > acceptance_max_mg:
                return finish(
                    "failed",
                    f"tail pulse {attempt} exceeded {acceptance_max_mg:.0f} mg: "
                    f"{current_mass_mg:.1f} mg",
                    current_measurement,
                )
            if current_mass_mg >= acceptance_min_mg:
                return finish(
                    "completed",
                    f"tail pulse {attempt} settled inside the acceptance window",
                    current_measurement,
                )

            if gain_mg >= config["min_gain_mg"]:
                observed_max_gain_mg = max(observed_max_gain_mg, gain_mg)

            if gain_mg < config["min_gain_mg"]:
                no_gain_attempts += 1
                if pulse_mode == "micro":
                    pulse_window = min(
                        config["micro_max_window_position_units"],
                        pulse_window + config["micro_window_step_units"],
                    )
                    pulse_duration_s = min(
                        config["micro_max_duration_s"],
                        pulse_duration_s * 1.25,
                    )
                    if (
                        current_mass_mg >= config["micro_safe_stop_min_mass_mg"]
                        and no_gain_attempts
                        >= config["micro_max_no_gain_attempts"]
                    ):
                        return finish(
                            "failed",
                            "safe micro pulse stopped underweight at "
                            f"{current_mass_mg:.1f} mg after {no_gain_attempts} "
                            "no-gain attempts; p380 was prohibited",
                            current_measurement,
                        )
                else:
                    pulse_window = min(
                        config["max_window_position_units"],
                        pulse_window + config["window_step_units"],
                    )
                    pulse_duration_s = min(
                        config["max_duration_s"],
                        pulse_duration_s * 1.25,
                    )
                    if no_gain_attempts >= config["max_no_gain_attempts"]:
                        return finish(
                            "failed",
                            "tail pulse produced no measurable gain after "
                            f"{no_gain_attempts} attempts; p400 was not opened",
                            current_measurement,
                        )
            else:
                no_gain_attempts = 0
                remaining_to_acceptance_mg = acceptance_min_mg - current_mass_mg
                if pulse_mode == "micro":
                    desired_gain_mg = max(
                        2.0, min(8.0, remaining_to_acceptance_mg)
                    )
                    pulse_duration_s = max(
                        config["micro_min_duration_s"],
                        min(
                            config["micro_max_duration_s"],
                            pulse_duration_s
                            * desired_gain_mg
                            / max(gain_mg, 1.0),
                        ),
                    )
                    if gain_mg >= 20.0:
                        pulse_window = config[
                            "micro_initial_window_position_units"
                        ]
                else:
                    desired_gain_mg = max(
                        2.0, min(20.0, remaining_to_acceptance_mg)
                    )
                    pulse_duration_s = max(
                        config["min_duration_s"],
                        min(
                            config["max_duration_s"],
                            pulse_duration_s
                            * desired_gain_mg
                            / max(gain_mg, 1.0),
                        ),
                    )
                    if gain_mg >= 40.0:
                        pulse_window = max(
                            config["min_window_position_units"],
                            pulse_window - config["window_step_units"],
                        )

    def _run_feedback_dispense(
        self,
        run_id,
        target_mg,
        preset_id,
        preset,
        profile,
        powder_overrides=None,
        powder=None,
        close_window_after_run=False,
        large_prefeed_reserve_mg=None,
        closed_loop_initial=None,
    ):
        acceptance_min_mg = target_mg - DISPENSE_ALLOWED_UNDERWEIGHT_MG
        acceptance_max_mg = target_mg + DISPENSE_ALLOWED_OVERWEIGHT_MG
        result = {
            "run_id": run_id,
            "test_type": "feedback_dispense",
            "powder_id": (powder or {}).get("powder_id"),
            "powder_name": (powder or {}).get("powder_name"),
            "target_mass_mg": target_mg,
            "hard_limit_mg": acceptance_max_mg,
            "preset_id": preset_id,
            "initial_parameters": preset,
            "profile": profile,
            "samples": [],
            "control_updates": [],
        }
        started_at = time.monotonic()
        vibration_started = False
        estimator = MassMotionEstimator()
        controller_settings = None
        controller = None
        tail_pulse_config = None
        tail_pulse_stage_entered = False
        large_prefeed_handoff_completed = False
        # Large runs hold the operator-selected duty open-loop until the
        # precision reserve is reached.  The run start already wrote this
        # duty to the actuator, so it doubles as the last-applied reference.
        large_prefeed_duty_permyriad = (
            int(preset.get("duty_permyriad", 2000))
            if large_prefeed_reserve_mg is not None
            else None
        )
        large_prefeed_applied_duty = large_prefeed_duty_permyriad
        large_low_flow_since = None
        large_recovery_active = False
        large_recovery_rounds = 0
        closed_loop_low_flow_since = None
        closed_loop_recovery_rounds = 0
        try:
            configured_stall_recovery = bool(
                (powder_overrides or {}).get("stall_recovery_enabled", False)
            )
            stall_recovery_max_window = int(
                (powder_overrides or {}).get(
                    "stall_recovery_max_window_position_units", 400
                )
            )
            # Do not reject a dispense merely because its chosen feed window
            # lies outside the optional recovery window. Recovery is skipped
            # for that run; normal feedback dispensing remains available.
            stall_recovery_enabled = (
                configured_stall_recovery
                and preset["window_position_units"] < stall_recovery_max_window
            )
            # Keep the controller limit aligned with the dashboard, request
            # validator, and LA10 adapter (10%-50%).  Controller construction
            # belongs inside this guard so a configuration error cannot leave
            # the dashboard task permanently stuck in the running state.
            controller_settings = FeedbackControllerSettings(
                coarse_rate_mg_s=profile["coarse_rate_mg_s"],
                fine_rate_mg_s=profile["fine_rate_mg_s"],
                precision_rate_mg_s=profile["precision_rate_mg_s"],
                maximum_flow_rate_mg_s=profile["maximum_flow_rate_mg_s"],
                min_duty_permyriad=DISPENSE_MIN_DUTY_PERMYRIAD,
                max_duty_permyriad=int(
                    (powder_overrides or {}).get(
                        "max_duty_permyriad", DISPENSE_MAX_DUTY_PERMYRIAD
                    )
                ),
                min_window_position_units=(0 if target_mg >= 1000 and run_id.startswith("large-") else 100),
                max_window_position_units=(2050 if target_mg >= 1000 and run_id.startswith("large-") else 2050),
                duty_step_per_update=int((powder_overrides or {}).get("duty_step_per_update", 50)),
                emergency_duty_step_per_update=int((powder_overrides or {}).get("emergency_duty_step_per_update", 200)),
                emergency_duty_update_interval_s=ACTUATOR_UPDATE_INTERVAL_S,
                position_step_units=25,
                duty_update_interval_s=0.5,
                position_update_interval_s=2.0,
                position_persistence_samples=4,
                position_feedback_enabled=bool((powder_overrides or {}).get("position_feedback_enabled", False)),
                settle_confirmations=(
                    PREDICTIVE_STOP_CONFIRMATIONS
                    + (powder_overrides or {}).get("settle_confirmations_delta", 0)
                ),
                allowed_underweight_mg=DISPENSE_ALLOWED_UNDERWEIGHT_MG,
                allowed_overweight_mg=DISPENSE_ALLOWED_OVERWEIGHT_MG,
                precision_start_remaining_mg=profile["precision_start_remaining_mg"],
                stop_prediction_horizon_s=PREDICTIVE_STOP_HORIZON_S,
                fixed_tail_mass_mg=float((powder_overrides or {}).get("fixed_tail_mass_mg", 3.0)),
                stop_target_offset_mg=PREDICTIVE_STOP_TARGET_OFFSET_MG,
                predictive_stop_enabled=True,
                fast_predictive_stop_enabled=bool(
                    (powder_overrides or {}).get(
                        "fast_predictive_stop_enabled", False
                    )
                ),
                fast_predictive_stop_rate_mg_s=float(
                    (powder_overrides or {}).get(
                        "fast_predictive_stop_rate_mg_s", 20.0
                    )
                ),
                fast_predictive_stop_confirmations=int(
                    (powder_overrides or {}).get(
                        "fast_predictive_stop_confirmations", 1
                    )
                ),
                projected_safety_stop_enabled=bool(
                    (powder_overrides or {}).get(
                        "projected_safety_stop_enabled", False
                    )
                ),
                projected_safety_stop_offset_mg=float(
                    (powder_overrides or {}).get(
                        "projected_safety_stop_offset_mg", 5.0
                    )
                ),
                tail_taper_enabled=True,
                tail_taper_start_remaining_mg=profile["tail_taper_start_remaining_mg"],
                tail_taper_end_remaining_mg=profile["tail_taper_end_remaining_mg"],
                tail_duty_reduction_permyriad=200,
                tail_window_reduction_units=50,
                flow_hold_enabled=bool(
                    (powder_overrides or {}).get("flow_hold_enabled", False)
                ),
                flow_hold_window_s=float(
                    (powder_overrides or {}).get("flow_hold_window_s", 4.0)
                ),
                flow_hold_min_gain_mg=float(
                    (powder_overrides or {}).get("flow_hold_min_gain_mg", 5.0)
                ),
                flow_response_check_s=float(
                    (powder_overrides or {}).get("flow_response_check_s", 4.0)
                ),
                flow_response_min_improvement_mg_s=float(
                    (powder_overrides or {}).get(
                        "flow_response_min_improvement_mg_s", 1.0
                    )
                ),
                stall_recovery_enabled=stall_recovery_enabled,
                stall_recovery_observation_window_s=float(
                    (powder_overrides or {}).get(
                        "stall_recovery_observation_window_s", 3.0
                    )
                ),
                stall_recovery_min_gain_mg=float(
                    (powder_overrides or {}).get("stall_recovery_min_gain_mg", 2.0)
                ),
                stall_recovery_rate_threshold_mg_s=float(
                    (powder_overrides or {}).get(
                        "stall_recovery_rate_threshold_mg_s", 1.0
                    )
                ),
                stall_recovery_resume_rate_mg_s=float(
                    (powder_overrides or {}).get(
                        "stall_recovery_resume_rate_mg_s", 2.0
                    )
                ),
                stall_recovery_retry_s=float(
                    (powder_overrides or {}).get("stall_recovery_retry_s", 3.0)
                ),
                stall_recovery_window_step_units=int(
                    (powder_overrides or {}).get(
                        "stall_recovery_window_step_units", 25
                    )
                ),
                stall_recovery_max_window_position_units=stall_recovery_max_window,
                stall_recovery_max_attempts=int(
                    (powder_overrides or {}).get("stall_recovery_max_attempts", 2)
                ),
                stall_recovery_disable_remaining_mg=float(
                    (powder_overrides or {}).get(
                        "stall_recovery_disable_remaining_mg", 50.0
                    )
                ),
                force_stop_max_mass_mg=float(
                    target_mg - (powder_overrides or {}).get("force_stop_offset_mg", 10.0)
                ),
            )
            # The large-run pre-feed bypasses the rate loop and drives the
            # operator-selected duty straight to the actuator, so keep the
            # controller seed inside its own limits instead of rejecting the
            # run when the pre-feed duty sits above the saved closed-loop
            # ceiling.  The pre-feed duty itself is not clamped.
            controller_seed_duty = int(preset["duty_permyriad"])
            if large_prefeed_reserve_mg is not None:
                controller_seed_duty = max(
                    controller_settings.min_duty_permyriad,
                    min(
                        controller_settings.max_duty_permyriad,
                        controller_seed_duty,
                    ),
                )
            controller = ContinuousFeedbackController(
                target_mass_mg=target_mg,
                initial=FeedbackInitialParameters(
                    frequency_hz=preset["frequency_hz"],
                    duty_permyriad=controller_seed_duty,
                    window_position_units=preset["window_position_units"],
                ),
                settings=controller_settings,
            )
            tail_pulse_config = resolve_tail_pulse_config(powder_overrides, target_mg)
            fast_tail_enabled = (
                controller_settings.fast_predictive_stop_enabled
                or controller_settings.projected_safety_stop_enabled
            )
            powder_id = (powder or {}).get("powder_id", "")
            if tail_pulse_config["enabled"]:
                result["algorithm_version"] = "predictive-stop-v5-tail-pulse"
            elif (
                controller_settings.stall_recovery_enabled
                and powder_id == "water_loss_agent_2"
            ):
                result["algorithm_version"] = (
                    "predictive-stop-v4-water-loss-agent-2-stall-recovery"
                )
            elif fast_tail_enabled and powder_id == "bentonite":
                result["algorithm_version"] = (
                    "predictive-stop-v3-bentonite-fast-tail"
                )
            elif fast_tail_enabled and powder_id == "water_loss_agent_2":
                result["algorithm_version"] = (
                    "predictive-stop-v3-water-loss-agent-2-fast-tail"
                )
            elif fast_tail_enabled:
                result["algorithm_version"] = "predictive-stop-v3-fast-tail"
            else:
                result["algorithm_version"] = "predictive-stop-v2"
            result["predictive_stop_config"] = {
                "enabled": controller_settings.predictive_stop_enabled,
                "prediction_horizon_s": controller_settings.stop_prediction_horizon_s,
                "fixed_tail_mass_mg": controller_settings.fixed_tail_mass_mg,
                "target_offset_mg": controller_settings.stop_target_offset_mg,
                "confirmation_samples": controller_settings.settle_confirmations,
                "fast_stop_enabled": controller_settings.fast_predictive_stop_enabled,
                "fast_stop_rate_mg_s": controller_settings.fast_predictive_stop_rate_mg_s,
                "fast_stop_confirmation_samples": (
                    controller_settings.fast_predictive_stop_confirmations
                ),
                "projected_safety_stop_enabled": (
                    controller_settings.projected_safety_stop_enabled
                ),
                "projected_safety_stop_offset_mg": (
                    controller_settings.projected_safety_stop_offset_mg
                ),
                "prediction_start_mass_mg": (
                    target_mg - controller_settings.precision_start_remaining_mg
                ),
                "measured_safety_stop_mass_mg": controller_settings.force_stop_max_mass_mg,
            }
            result["flow_hold_config"] = {
                "enabled": controller_settings.flow_hold_enabled,
                "duty_ceiling_permyriad": controller_settings.max_duty_permyriad,
                "mass_gain_window_s": controller_settings.flow_hold_window_s,
                "minimum_gain_mg": controller_settings.flow_hold_min_gain_mg,
                "response_check_s": controller_settings.flow_response_check_s,
                "minimum_rate_improvement_mg_s": (
                    controller_settings.flow_response_min_improvement_mg_s
                ),
            }
            result["stall_recovery_config"] = {
                "enabled": controller_settings.stall_recovery_enabled,
                "configured_enabled": configured_stall_recovery,
                "disabled_for_current_window": (
                    configured_stall_recovery
                    and not controller_settings.stall_recovery_enabled
                ),
                "observation_window_s": (
                    controller_settings.stall_recovery_observation_window_s
                ),
                "minimum_gain_mg": controller_settings.stall_recovery_min_gain_mg,
                "rate_threshold_mg_s": (
                    controller_settings.stall_recovery_rate_threshold_mg_s
                ),
                "resume_rate_mg_s": (
                    controller_settings.stall_recovery_resume_rate_mg_s
                ),
                "retry_s": controller_settings.stall_recovery_retry_s,
                "window_step_units": (
                    controller_settings.stall_recovery_window_step_units
                ),
                "maximum_window_position_units": (
                    controller_settings.stall_recovery_max_window_position_units
                ),
                "maximum_attempts": controller_settings.stall_recovery_max_attempts,
                "disable_remaining_mg": (
                    controller_settings.stall_recovery_disable_remaining_mg
                ),
                "configured_window_position_units": preset["window_position_units"],
                "duty_cap_permyriad": preset["duty_permyriad"],
            }
            result["tail_pulse_config"] = dict(tail_pulse_config)
            self._set_test({"status": "running", "run_id": run_id, "phase": "window_move", "result": result})
            before, after, distance_register = self._move_window_to(
                preset["window_position_units"], speed_mm_s=1.0, timeout_s=20.0,
            )
            result["window_before"] = before.__dict__
            result["window_after"] = after.__dict__
            result["window_distance_register"] = distance_register

            self._set_test({"status": "running", "run_id": run_id, "phase": "tare", "result": result})
            with self.reader._lock:
                tare_before = self.reader._at.read_measurement()
                self.reader._at.zero(authorized=True)
            result["tare_before"] = tare_before.__dict__
            result["tare_after"] = self._wait_for_stable_measurement(TARE_TIMEOUT_S).__dict__

            self._set_test({"status": "running", "run_id": run_id, "phase": "vibration_start", "result": result})
            vibration_started = True
            with self.reader._lock:
                self.reader._la10.start_vibration(
                    frequency_hz=preset["frequency_hz"],
                    duty_permyriad=preset["duty_permyriad"],
                )
                vibration = self.reader._la10.read_vibration_settings()
            if not vibration.enabled:
                raise RuntimeError("LA10 did not confirm vibration enable")
            result["vibration_started"] = vibration.__dict__

            while True:
                if self._cancel_event.is_set():
                    result["result"] = "cancelled"
                    break
                read_started_at = time.monotonic()
                with self.reader._lock:
                    measurement = self.reader._at.read_measurement()
                sample_time = time.monotonic()
                measurement_read_s = sample_time - read_started_at
                measured_mass = measurement.mass_mg if measurement.mass_mg is not None else measurement.raw_count
                mass_mg = max(0.0, float(measured_mass))
                if mass_mg > acceptance_max_mg:
                    raise RuntimeError(f"hard overweight boundary exceeded: {mass_mg:.1f} mg")
                if (
                    tail_pulse_config["enabled"]
                    and mass_mg >= tail_pulse_config["transition_mass_mg"]
                ):
                    tail_pulse_stage_entered = True
                    result["tail_pulse_stage_entered"] = True
                    result["tail_pulse_transition"] = {
                        "elapsed_s": round(sample_time - started_at, 3),
                        "mass_mg": round(mass_mg, 3),
                        "configured_transition_mass_mg": tail_pulse_config[
                            "transition_mass_mg"
                        ],
                    }
                    with self.reader._lock:
                        self.reader._la10.stop_vibration()
                    vibration_started = False
                    result["continuous_vibration_stopped_at_utc"] = (
                        datetime.now(timezone.utc).isoformat()
                    )
                    hold_before, hold_after, hold_distance = self._move_window_to(
                        tail_pulse_config["hold_window_position_units"],
                        speed_mm_s=1.0,
                        timeout_s=5.0,
                        respect_cancel=False,
                    )
                    result["tail_pulse_hold_transition"] = {
                        "before": hold_before.position_units,
                        "after": hold_after.position_units,
                        "requested": tail_pulse_config["hold_window_position_units"],
                        "distance_register": hold_distance,
                    }
                    transition_trace = []
                    transition_measurement = self._wait_for_stable_measurement(
                        tail_pulse_config["settle_timeout_s"],
                        minimum_wait_s=tail_pulse_config["settle_minimum_wait_s"],
                        continuous_stable_s=tail_pulse_config[
                            "settle_continuous_stable_s"
                        ],
                        max_spread_mg=tail_pulse_config["settle_max_spread_mg"],
                        trace=transition_trace,
                    )
                    result["tail_pulse_transition_settle_samples"] = transition_trace
                    result["tail_pulse_transition_stable_mass_mg"] = (
                        self._measurement_mass_mg(transition_measurement)
                    )
                    pulse_outcome = self._run_tail_pulse_stage(
                        run_id=run_id,
                        target_mg=target_mg,
                        acceptance_min_mg=acceptance_min_mg,
                        acceptance_max_mg=acceptance_max_mg,
                        config=tail_pulse_config,
                        initial_measurement=transition_measurement,
                        started_at=started_at,
                        result=result,
                    )
                    result["result"] = pulse_outcome["status"]
                    pulse_stop_sample = {
                        "elapsed_s": round(time.monotonic() - started_at, 3),
                        "mass_mg": round(pulse_outcome["mass_mg"], 3),
                        "stage": "tail_pulse",
                        "frequency_hz": tail_pulse_config["frequency_hz"],
                        "duty_permyriad": 0,
                        "window_position_units": tail_pulse_config[
                            "hold_window_position_units"
                        ],
                        "stable": pulse_outcome["measurement"].stable,
                        "status_word": pulse_outcome["measurement"].status_word,
                        "reason": pulse_outcome["reason"],
                    }
                    result["stop_sample"] = pulse_stop_sample
                    result["stop_reason"] = pulse_outcome["reason"]
                    if pulse_outcome["status"] == "failed":
                        result["error"] = pulse_outcome["reason"]
                    break
                if (
                    large_prefeed_reserve_mg is not None
                    and not large_prefeed_handoff_completed
                    and target_mg - mass_mg <= large_prefeed_reserve_mg
                ):
                    # The high-speed phase must end physically, not merely by
                    # changing its duty.  Let its tail settle, then start a
                    # fresh ordinary closed-loop run with the saved powder
                    # parameters and actuator seed.
                    self._set_test({
                        "status": "running", "run_id": run_id,
                        "phase": "large_prefeed_handoff_settle", "result": result,
                    })
                    with self.reader._lock:
                        self.reader._la10.stop_vibration()
                    vibration_started = False
                    handoff_trace = []
                    handoff_measurement = self._wait_for_stable_measurement(
                        15.0,
                        minimum_wait_s=POST_VIBRATION_SETTLE_S,
                        continuous_stable_s=FINAL_STABILITY_WINDOW_S,
                        max_spread_mg=FINAL_STABILITY_MAX_SPREAD_MG,
                        trace=handoff_trace,
                        on_sample=lambda sample: self._publish_post_stop_sample(
                            run_id, result, sample
                        ),
                    )
                    settled_mass_mg = self._measurement_mass_mg(handoff_measurement)
                    if acceptance_min_mg <= settled_mass_mg <= acceptance_max_mg:
                        result["large_prefeed_handoff"] = {
                            "reserve_mg": large_prefeed_reserve_mg,
                            "vibration_stopped_at_mass_mg": round(mass_mg, 3),
                            "settled_mass_mg": round(settled_mass_mg, 3),
                            "settle_samples": handoff_trace,
                            "closed_loop_skipped": True,
                            "vibration_restarted": False,
                        }
                        result["result"] = "completed"
                        result["stop_sample"] = {
                            "elapsed_s": round(time.monotonic() - started_at, 3),
                            "mass_mg": round(settled_mass_mg, 3),
                            "stage": "large_prefeed_handoff_settle",
                            "frequency_hz": 0,
                            "duty_permyriad": 0,
                            "window_position_units": preset["window_position_units"],
                            "stable": handoff_measurement.stable,
                            "status_word": handoff_measurement.status_word,
                            "reason": "settled mass already inside acceptance window",
                        }
                        result["stop_reason"] = result["stop_sample"]["reason"]
                        break
                    closed_loop_seed = dict(closed_loop_initial or preset)
                    saved_closed_loop_window = closed_loop_seed["window_position_units"]
                    # Stored powder profiles use the UI window coordinate.
                    # Apply the same +750 device-coordinate correction used
                    # by normal requests before restarting closed-loop dosing.
                    closed_loop_seed["window_position_units"] = (
                        saved_closed_loop_window + 750
                    )
                    handoff_before, handoff_after, handoff_distance = self._move_window_to(
                        closed_loop_seed["window_position_units"],
                        speed_mm_s=1.0,
                        timeout_s=20.0,
                        respect_cancel=False,
                    )
                    controller = ContinuousFeedbackController(
                        target_mass_mg=target_mg,
                        initial=FeedbackInitialParameters(**closed_loop_seed),
                        settings=controller_settings,
                    )
                    estimator = MassMotionEstimator()
                    with self.reader._lock:
                        self.reader._la10.start_vibration(
                            frequency_hz=closed_loop_seed["frequency_hz"],
                            duty_permyriad=closed_loop_seed["duty_permyriad"],
                        )
                        handoff_vibration = self.reader._la10.read_vibration_settings()
                    if not handoff_vibration.enabled:
                        raise RuntimeError("LA10 did not confirm vibration enable after large prefeed handoff")
                    vibration_started = True
                    large_prefeed_handoff_completed = True
                    result["large_prefeed_handoff"] = {
                        "reserve_mg": large_prefeed_reserve_mg,
                        "vibration_stopped_at_mass_mg": round(mass_mg, 3),
                        "settled_mass_mg": round(
                            self._measurement_mass_mg(handoff_measurement), 3
                        ),
                        "settle_samples": handoff_trace,
                        "closed_loop_initial": closed_loop_seed,
                        "saved_closed_loop_window_units": saved_closed_loop_window,
                        "applied_closed_loop_window_units": (
                            saved_closed_loop_window + 750
                        ),
                        "window_before_units": handoff_before.position_units,
                        "window_after_units": handoff_after.position_units,
                        "window_distance_register": handoff_distance,
                        "vibration_restarted": handoff_vibration.__dict__,
                    }
                    continue
                motion = estimator.update(sample_time, mass_mg)
                # Large runs use a constant high-speed feed. Keep the actuator
                # at the operator-selected pre-feed duty until the measured
                # mass reaches the configured precision reserve, then hand
                # control back to the normal closed-loop duty regulator.
                large_fixed_speed = (
                    large_prefeed_reserve_mg is not None
                    and not large_prefeed_handoff_completed
                    and target_mg - mass_mg > large_prefeed_reserve_mg
                )
                if (
                    not large_fixed_speed
                    and mass_mg >= acceptance_min_mg
                ):
                    with self.reader._lock:
                        self.reader._la10.stop_vibration()
                    vibration_started = False
                    result["result"] = "completed"
                    result["stop_sample"] = {
                        "elapsed_s": round(sample_time - started_at, 3),
                        "mass_mg": round(mass_mg, 3),
                        "stage": "closed_loop_acceptance_guard",
                        "frequency_hz": 0,
                        "duty_permyriad": 0,
                        "window_position_units": controller.window_position_units,
                        "stable": measurement.stable,
                        "status_word": measurement.status_word,
                        "reason": "measured mass reached acceptance lower bound",
                    }
                    result["stop_reason"] = result["stop_sample"]["reason"]
                    break
                decision = controller.step(
                    FeedbackObservation(
                        mass_mg=mass_mg,
                        predicted_mass_mg=motion.predicted_mass_mg,
                        rate_mg_s=motion.rate_mg_s,
                        predicted_rate_mg_s=motion.predicted_rate_mg_s,
                        acceleration_mg_s2=motion.acceleration_mg_s2,
                        jerk_mg_s3=motion.jerk_mg_s3,
                        valid=motion.valid,
                    ),
                    timestamp_s=sample_time,
                    bypass_rate_control=large_fixed_speed,
                )
                update = None
                duty_before_actual = controller.duty_permyriad
                if large_fixed_speed:
                    # Hold the operator-selected pre-feed duty.  Compare it
                    # with the last value written to the actuator instead of
                    # the controller state, so a clamped controller duty
                    # cannot trigger repeated register writes.
                    requested_duty = large_prefeed_duty_permyriad
                    duty_before_actual = large_prefeed_applied_duty
                else:
                    requested_duty = decision.duty_after_permyriad
                if requested_duty != controller.duty_permyriad:
                    controller.duty_permyriad = requested_duty
                if requested_duty != duty_before_actual:
                    with self.reader._lock:
                        self.reader._la10.update_vibration_duty(requested_duty)
                    if large_fixed_speed:
                        large_prefeed_applied_duty = requested_duty
                    update = {
                        "type": "duty",
                        "before": (
                            duty_before_actual
                            if large_fixed_speed
                            else decision.duty_before_permyriad
                        ),
                        "after": requested_duty,
                        "mode": (
                            "large_prefeed_fixed_duty"
                            if large_fixed_speed
                            else "closed_loop"
                        ),
                    }
                if decision.window_changed:
                    window_before, window_after, distance = self._move_window_to(
                        decision.window_after_units, speed_mm_s=1.0, timeout_s=5.0,
                    )
                    window_update = {
                        "type": "window",
                        "before": window_before.position_units,
                        "after": window_after.position_units,
                        "requested": decision.window_after_units,
                        "distance_register": distance,
                    }
                    if update is None:
                        update = window_update
                    else:
                        update = {"type": "duty_and_window", "duty": update, "window": window_update}
                large_recovery = None
                closed_loop_recovery = None
                large_recovery_samples = []
                closed_loop_recovery_samples = []
                if large_fixed_speed and not decision.stop_requested:
                    if not motion.valid or motion.rate_mg_s >= 10.0:
                        large_low_flow_since = None
                        large_recovery_active = False
                    elif large_low_flow_since is None:
                        large_low_flow_since = sample_time
                    elif sample_time - large_low_flow_since >= 5.0:
                        large_recovery_active = True

                    if large_recovery_active:
                        with self.reader._lock:
                            vibration_state = self.reader._la10.read_vibration_settings()
                        if not vibration_state.enabled:
                            raise RuntimeError(
                                "large-run low-flow recovery requires vibration to remain enabled"
                            )
                        duty_before_recovery = controller.duty_permyriad
                        duty_after_recovery = min(
                            5000, duty_before_recovery + 200
                        )
                        if duty_after_recovery != duty_before_recovery:
                            with self.reader._lock:
                                self.reader._la10.update_vibration_duty(
                                    duty_after_recovery
                                )
                            controller.duty_permyriad = duty_after_recovery
                            requested_duty = duty_after_recovery
                            large_prefeed_applied_duty = duty_after_recovery
                            update = {
                                "type": "duty",
                                "before": duty_before_recovery,
                                "after": duty_after_recovery,
                                "mode": "large_prefeed_fixed_duty_low_flow_recovery",
                            }

                        recovery_positions = []
                        if target_mg - mass_mg > 200.0:
                            window_start = controller.window_position_units
                            for offset in (100, -100, 0):
                                recovery_target = window_start + offset
                                move_before, move_after, move_distance = self._move_window_to(
                                    recovery_target,
                                    speed_mm_s=1.0,
                                    timeout_s=10.0,
                                )
                                controller.window_position_units = recovery_target
                                recovery_positions.append({
                                    "requested_units": recovery_target,
                                    "before_units": move_before.position_units,
                                    "after_units": move_after.position_units,
                                    "distance_register": move_distance,
                                })

                        large_recovery_rounds += 1
                        recovery_rate = motion.rate_mg_s
                        recovery_rate_samples = []
                        large_recovery_samples = []
                        recovery_estimator = MassMotionEstimator()
                        for recovery_index in range(60):
                            if self._cancel_event.wait(POLL_INTERVAL_S):
                                break
                            recovery_read_started_at = time.monotonic()
                            with self.reader._lock:
                                recovery_measurement = self.reader._at.read_measurement()
                            recovery_mass_mg = self._measurement_mass_mg(
                                recovery_measurement
                            )
                            recovery_sample_time = time.monotonic()
                            if recovery_mass_mg > acceptance_max_mg:
                                raise RuntimeError(
                                    "hard overweight boundary exceeded during recovery: "
                                    f"{recovery_mass_mg:.1f} mg"
                                )
                            recovery_motion = recovery_estimator.update(
                                recovery_sample_time, recovery_mass_mg
                            )
                            recovery_rate = recovery_motion.rate_mg_s
                            if recovery_motion.valid:
                                recovery_rate_samples.append(recovery_rate)
                            recovery_sample = {
                                "elapsed_s": round(recovery_sample_time - started_at, 3),
                                "measurement_read_s": round(
                                    recovery_sample_time - recovery_read_started_at, 4
                                ),
                                "mass_mg": round(recovery_mass_mg, 3),
                                "rate_valid": recovery_motion.valid,
                                "rate_mg_s": round(recovery_rate, 3),
                                "predicted_rate_mg_s": round(
                                    recovery_motion.predicted_rate_mg_s, 3
                                ),
                                "control_rate_mg_s": round(recovery_rate, 3),
                                "duty_permyriad": controller.duty_permyriad,
                                "window_position_units": controller.window_position_units,
                                "stable": recovery_measurement.stable,
                                "status_word": recovery_measurement.status_word,
                                "stage": "large_low_flow_recovery",
                                "recovery_round": large_recovery_rounds + 1,
                            }
                            large_recovery_samples.append(recovery_sample)
                            self._set_test(
                                {
                                    "status": "running",
                                    "run_id": run_id,
                                    "phase": "large_low_flow_recovery",
                                    "latest_sample": recovery_sample,
                                    "result": result,
                                }
                            )
                            if recovery_rate > 30.0:
                                break
                        large_recovery = {
                            "round": large_recovery_rounds,
                            "trigger_rate_mg_s": round(motion.rate_mg_s, 3),
                            "low_flow_duration_s": round(
                                sample_time - large_low_flow_since, 3
                            ),
                            "duty_before_permyriad": duty_before_recovery,
                            "duty_after_permyriad": duty_after_recovery,
                            "window_positions": recovery_positions,
                            "recovery_samples": large_recovery_samples,
                            "vibration_remained_enabled": True,
                            "post_recovery_rate_mg_s": round(recovery_rate, 3),
                            "post_recovery_rate_samples_mg_s": [
                                round(rate, 3) for rate in recovery_rate_samples
                            ],
                        }
                        result.setdefault("large_low_flow_recovery", []).append(
                            large_recovery
                        )
                        estimator = MassMotionEstimator()
                        if recovery_rate > 30.0:
                            large_low_flow_since = None
                            large_recovery_active = False
                        else:
                            # Keep the low-flow timer initialized while recovery
                            # remains active; resetting it here allows the next
                            # iteration to clear it before computing duration.
                            large_low_flow_since = time.monotonic()
                            large_recovery_active = True
                        if recovery_rate > 30.0:
                            mass_mg = recovery_mass_mg
                            sample_time = time.monotonic()
                        if large_recovery_active:
                            continue
                elif (
                    (large_prefeed_reserve_mg is None or large_prefeed_handoff_completed)
                    and not decision.stop_requested
                ):
                    if motion.rate_mg_s >= 5.0:
                        closed_loop_low_flow_since = None
                    elif closed_loop_low_flow_since is None:
                        closed_loop_low_flow_since = sample_time

                    if (
                        closed_loop_low_flow_since is not None
                        and sample_time - closed_loop_low_flow_since >= 5.0
                    ):
                        with self.reader._lock:
                            vibration_state = self.reader._la10.read_vibration_settings()
                        if not vibration_state.enabled:
                            raise RuntimeError(
                                "large-run closed-loop recovery requires vibration to remain enabled"
                            )

                        duty_before_recovery = controller.duty_permyriad
                        duty_after_recovery = min(5000, duty_before_recovery + 200)
                        if duty_after_recovery != duty_before_recovery:
                            with self.reader._lock:
                                self.reader._la10.update_vibration_duty(
                                    duty_after_recovery
                                )
                            controller.duty_permyriad = duty_after_recovery
                            update = {
                                "type": "duty",
                                "before": duty_before_recovery,
                                "after": duty_after_recovery,
                                "mode": "large_closed_loop_low_flow_recovery",
                            }

                        recovery_positions = []
                        if target_mg - mass_mg > 200.0:
                            window_start = controller.window_position_units
                            for offset in (100, -100, 0):
                                recovery_target = window_start + offset
                                if not 0 <= recovery_target <= 2050:
                                    raise RuntimeError(
                                        "closed-loop low-flow recovery window is outside "
                                        f"the device range: {recovery_target}"
                                    )
                                move_before, move_after, move_distance = self._move_window_to(
                                    recovery_target,
                                    speed_mm_s=1.0,
                                    timeout_s=10.0,
                                )
                                controller.window_position_units = recovery_target
                                recovery_positions.append({
                                    "requested_units": recovery_target,
                                    "before_units": move_before.position_units,
                                    "after_units": move_after.position_units,
                                    "distance_register": move_distance,
                                })

                        closed_loop_recovery_rounds += 1
                        recovery_rate = motion.rate_mg_s
                        recovery_rate_samples = []
                        closed_loop_recovery_samples = []
                        recovery_mass_mg = mass_mg
                        recovery_estimator = MassMotionEstimator()
                        for _ in range(60):
                            if self._cancel_event.wait(POLL_INTERVAL_S):
                                break
                            recovery_read_started_at = time.monotonic()
                            with self.reader._lock:
                                recovery_measurement = self.reader._at.read_measurement()
                            recovery_mass_mg = self._measurement_mass_mg(
                                recovery_measurement
                            )
                            recovery_sample_time = time.monotonic()
                            if recovery_mass_mg > acceptance_max_mg:
                                raise RuntimeError(
                                    "hard overweight boundary exceeded during closed-loop recovery: "
                                    f"{recovery_mass_mg:.1f} mg"
                                )
                            recovery_motion = recovery_estimator.update(
                                recovery_sample_time, recovery_mass_mg
                            )
                            recovery_rate = recovery_motion.rate_mg_s
                            if recovery_motion.valid:
                                recovery_rate_samples.append(recovery_rate)
                            recovery_sample = {
                                "elapsed_s": round(recovery_sample_time - started_at, 3),
                                "measurement_read_s": round(
                                    recovery_sample_time - recovery_read_started_at, 4
                                ),
                                "mass_mg": round(recovery_mass_mg, 3),
                                "rate_valid": recovery_motion.valid,
                                "rate_mg_s": round(recovery_rate, 3),
                                "predicted_rate_mg_s": round(
                                    recovery_motion.predicted_rate_mg_s, 3
                                ),
                                "control_rate_mg_s": round(recovery_rate, 3),
                                "duty_permyriad": controller.duty_permyriad,
                                "window_position_units": controller.window_position_units,
                                "stable": recovery_measurement.stable,
                                "status_word": recovery_measurement.status_word,
                                "stage": "large_closed_loop_low_flow_recovery",
                                "recovery_round": closed_loop_recovery_rounds + 1,
                            }
                            closed_loop_recovery_samples.append(recovery_sample)
                            self._set_test(
                                {
                                    "status": "running",
                                    "run_id": run_id,
                                    "phase": "large_closed_loop_low_flow_recovery",
                                    "latest_sample": recovery_sample,
                                    "result": result,
                                }
                            )
                            if recovery_rate > 30.0:
                                break

                        closed_loop_recovery = {
                            "round": closed_loop_recovery_rounds,
                            "trigger_rate_mg_s": round(motion.rate_mg_s, 3),
                            "low_flow_duration_s": round(
                                sample_time - closed_loop_low_flow_since, 3
                            ),
                            "duty_before_permyriad": duty_before_recovery,
                            "duty_after_permyriad": duty_after_recovery,
                            "window_positions": recovery_positions,
                            "recovery_samples": closed_loop_recovery_samples,
                            "vibration_remained_enabled": True,
                            "post_recovery_rate_mg_s": round(recovery_rate, 3),
                            "post_recovery_rate_samples_mg_s": [
                                round(rate, 3) for rate in recovery_rate_samples
                            ],
                        }
                        result.setdefault(
                            "large_closed_loop_low_flow_recovery", []
                        ).append(closed_loop_recovery)
                        estimator = MassMotionEstimator()
                        if recovery_rate > 30.0:
                            closed_loop_low_flow_since = None
                        else:
                            closed_loop_low_flow_since = time.monotonic()
                        mass_mg = recovery_mass_mg
                        sample_time = time.monotonic()
                if update is not None:
                    result["control_updates"].append(
                        {
                            "elapsed_s": round(sample_time - started_at, 3),
                            "mass_mg": round(mass_mg, 3),
                            "stage": decision.stage.value,
                            "update": update,
                            "reason": decision.reason,
                        }
                    )
                sample_mass_mg = mass_mg
                sample_rate_mg_s = motion.rate_mg_s
                sample_rate_valid = motion.valid
                sample_predicted_rate_mg_s = motion.predicted_rate_mg_s
                sample_control_rate_mg_s = decision.control_rate_mg_s
                sample_duty_permyriad = decision.duty_after_permyriad
                sample_window_position_units = decision.window_after_units
                active_recovery_samples = (
                    closed_loop_recovery_samples
                    if closed_loop_recovery is not None
                    else large_recovery_samples
                )
                if active_recovery_samples:
                    latest_recovery_sample = active_recovery_samples[-1]
                    sample_mass_mg = float(latest_recovery_sample["mass_mg"])
                    sample_rate_mg_s = float(latest_recovery_sample["rate_mg_s"])
                    sample_rate_valid = bool(latest_recovery_sample["rate_valid"])
                    sample_predicted_rate_mg_s = float(
                        latest_recovery_sample["predicted_rate_mg_s"]
                    )
                    sample_control_rate_mg_s = float(
                        latest_recovery_sample["control_rate_mg_s"]
                    )
                    sample_duty_permyriad = int(
                        latest_recovery_sample["duty_permyriad"]
                    )
                    sample_window_position_units = int(
                        latest_recovery_sample["window_position_units"]
                    )
                elif large_recovery is not None and large_recovery_samples:
                    latest_recovery_sample = large_recovery_samples[-1]
                    sample_mass_mg = float(latest_recovery_sample["mass_mg"])
                    sample_rate_mg_s = float(latest_recovery_sample["rate_mg_s"])
                    sample_rate_valid = bool(latest_recovery_sample["rate_valid"])
                    sample_predicted_rate_mg_s = float(
                        latest_recovery_sample["predicted_rate_mg_s"]
                    )
                    sample_control_rate_mg_s = float(
                        latest_recovery_sample["control_rate_mg_s"]
                    )
                    sample_duty_permyriad = int(
                        latest_recovery_sample["duty_permyriad"]
                    )
                    sample_window_position_units = int(
                        latest_recovery_sample["window_position_units"]
                    )
                sample = {
                    "elapsed_s": round(sample_time - started_at, 3),
                    "measurement_read_s": round(measurement_read_s, 4),
                    "mass_mg": round(sample_mass_mg, 3),
                    "filtered_mass_mg": round(motion.filtered_mass_mg, 3),
                    "predicted_mass_mg": round(motion.predicted_mass_mg, 3),
                    "rate_valid": sample_rate_valid,
                    "rate_mg_s": round(sample_rate_mg_s, 3),
                    "predicted_rate_mg_s": round(sample_predicted_rate_mg_s, 3),
                    "control_rate_mg_s": round(sample_control_rate_mg_s, 3),
                    "acceleration_mg_s2": round(motion.acceleration_mg_s2, 3),
                    "jerk_mg_s3": round(motion.jerk_mg_s3, 3),
                    "estimated_tail_mg": round(decision.estimated_tail_mg, 3),
                    "projected_stop_mass_mg": round(decision.projected_stop_mass_mg, 3),
                    "predictive_stop_candidate": decision.predictive_stop_candidate,
                    "stop_confirmation_count": decision.stop_confirmation_count,
                    "stop_confirmation_required": decision.stop_confirmation_required,
                    "stall_recovery_active": decision.stall_recovery_active,
                    "stall_recovery_attempt": decision.stall_recovery_attempt,
                    "stage": decision.stage.value,
                    "target_rate_mg_s": decision.target_rate_mg_s,
                    "frequency_hz": decision.frequency_hz,
                    "duty_permyriad": sample_duty_permyriad,
                    "window_position_units": sample_window_position_units,
                    "stable": measurement.stable,
                    "status_word": measurement.status_word,
                    "reason": decision.reason,
                    "large_low_flow_recovery": large_recovery,
                    "large_closed_loop_low_flow_recovery": closed_loop_recovery,
                }
                if not decision.stop_requested and closed_loop_low_flow_since is not None:
                    sample["low_flow_recovery_monitoring"] = {
                        "active": True,
                        "rate_threshold_mg_s": 5.0,
                        "required_duration_s": 5.0,
                        "elapsed_s": round(sample_time - closed_loop_low_flow_since, 3),
                        "duty_cap_permyriad": 5000,
                    }
                result["samples"].append(sample)
                self._set_test(
                    {
                        "status": "running",
                        "run_id": run_id,
                        "phase": f"{decision.stage.value}_feedback",
                        "latest_sample": sample,
                        "result": result,
                    }
                )
                if decision.stop_requested:
                    result["result"] = "completed"
                    result["stop_sample"] = sample
                    result["stop_reason"] = decision.reason
                    break
                self._cancel_event.wait(max(0, POLL_INTERVAL_S - (time.monotonic() - sample_time)))
        except Exception as error:
            result["result"] = "failed"
            result["error"] = str(error)
        finally:
            if vibration_started:
                try:
                    with self.reader._lock:
                        self.reader._la10.stop_vibration()
                    result["vibration_stopped_at_utc"] = datetime.now(timezone.utc).isoformat()
                except Exception as error:
                    result["result"] = "failed"
                    result["stop_error"] = str(error)
            if (
                controller_settings is not None
                and controller_settings.stall_recovery_enabled
                and not tail_pulse_stage_entered
            ):
                try:
                    restore_before, restore_after, restore_distance = self._move_window_to(
                        preset["window_position_units"],
                        speed_mm_s=1.0,
                        timeout_s=5.0,
                        respect_cancel=False,
                    )
                    result["stall_recovery_cleanup"] = {
                        "before": restore_before.position_units,
                        "after": restore_after.position_units,
                        "requested": preset["window_position_units"],
                        "distance_register": restore_distance,
                    }
                except Exception as error:
                    result["result"] = "failed"
                    result["window_restore_error"] = str(error)
            elif tail_pulse_stage_entered and tail_pulse_config is not None:
                result["window_left_at_safe_hold_units"] = tail_pulse_config["hold_window_position_units"]
            if result.get("result") == "completed":
                try:
                    result["post_stop_samples"] = []
                    self._set_test({"status": "running", "run_id": run_id, "phase": "final_settle", "result": result})
                    final_measurement = self._wait_for_stable_measurement(
                        15.0,
                        minimum_wait_s=POST_VIBRATION_SETTLE_S,
                        continuous_stable_s=FINAL_STABILITY_WINDOW_S,
                        max_spread_mg=FINAL_STABILITY_MAX_SPREAD_MG,
                        trace=result["post_stop_samples"],
                        on_sample=lambda sample: self._publish_post_stop_sample(
                            run_id, result, sample
                        ),
                    )
                    result["final_measurement"] = final_measurement.__dict__
                    final_mass_mg = (
                        final_measurement.mass_mg
                        if final_measurement.mass_mg is not None
                        else final_measurement.raw_count
                    )
                    result["final_mass_mg"] = float(final_mass_mg)
                    result["final_settle_rule"] = {
                        "minimum_wait_s": POST_VIBRATION_SETTLE_S,
                        "continuous_stable_s": FINAL_STABILITY_WINDOW_S,
                        "max_spread_mg": FINAL_STABILITY_MAX_SPREAD_MG,
                    }
                    result["acceptance_min_mg"] = acceptance_min_mg
                    result["acceptance_max_mg"] = acceptance_max_mg
                    if not acceptance_min_mg <= final_mass_mg <= acceptance_max_mg:
                        result["result"] = "failed"
                        result["acceptance_error"] = (
                            f"final stable mass outside {acceptance_min_mg:.0f}-"
                            f"{acceptance_max_mg:.0f} mg: {final_mass_mg:.1f} mg"
                        )
                except Exception as error:
                    result["result"] = "failed"
                    result["final_stability_error"] = str(error)
            if close_window_after_run:
                self._set_test({
                    "status": "running", "run_id": run_id,
                    "phase": "closing_window_after_dispense", "result": result,
                })
                try:
                    result["window_close_after_run"] = self._move_window_to_minimum()
                except Exception as error:
                    result["window_close_error"] = str(error)
                    result["result"] = "failed"
                    result["error"] = f"window close failed: {error}"
            result["actual_duration_s"] = round(time.monotonic() - started_at, 3)
            result["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            self._persist_result("feedback_dispense", run_id, result)
            self._set_test({"status": result["result"], "run_id": run_id, "result": result})

    # ── constant-rate dispensing worker ────────────────────────────────

    def _run_constant_rate_test(self, run_id, submitted):
        """Execute one constant-rate dispensing run with repeat_count segments."""
        target_rate_mg_s = submitted["target_rate_mg_s"]
        frequency_hz = submitted["frequency_hz"]
        duty_permyriad = submitted["duty_permyriad"]
        window_units = submitted["window_position_units"]
        segment_duration_s = submitted["duration_s"]
        repeat_count = submitted["repeat_count"]
        rate_tolerance_mg_s = 2.0  # ±2 mg/s
        cv_pct_limit = 10.0

        result = {
            "run_id": run_id,
            "test_type": "constant_rate_dispense",
            "target_rate_mg_s": target_rate_mg_s,
            "rate_tolerance_mg_s": rate_tolerance_mg_s,
            "cv_pct_limit": cv_pct_limit,
            "frequency_hz": frequency_hz,
            "duty_permyriad": duty_permyriad,
            "window_position_units": window_units,
            "segment_duration_s": segment_duration_s,
            "repeat_count": repeat_count,
            "samples": [],
            "segments": [],
            "control_updates": [],
        }
        vibration_started = False
        started_at = None
        try:
            # --- pre-condition checks ---
            with self.reader._lock:
                window_before = self.reader._la10.read_status()
                vibration_before = self.reader._la10.read_vibration_settings()
            if window_before.is_running:
                raise RuntimeError("Cannot start test while the LA10 window is moving")
            if window_before.fault_bits:
                raise RuntimeError(
                    f"LA10 fault bits before start: 0x{window_before.fault_bits:04X}"
                )
            if vibration_before.enabled:
                with self.reader._lock:
                    self.reader._la10.stop_vibration()
                raise RuntimeError(
                    "Vibration was already enabled; it has been stopped. Start a new test after inspection."
                )
            result["window_before"] = window_before.__dict__

            # --- move window ---
            self._set_test({"status": "running", "run_id": run_id, "phase": "window_move", "result": result})
            w_before, w_after, distance_reg = self._move_window_to(
                window_units, speed_mm_s=1.0, timeout_s=20.0,
            )
            result["window_distance_register"] = distance_reg
            result["window_after"] = w_after.__dict__

            # --- tare ---
            self._set_test({"status": "running", "run_id": run_id, "phase": "tare", "result": result})
            with self.reader._lock:
                tare_before = self.reader._at.read_measurement()
                self.reader._at.zero(authorized=True)
            result["tare_before"] = tare_before.__dict__
            tare_after = self._wait_for_stable_measurement(TARE_TIMEOUT_S)
            result["tare_after"] = tare_after.__dict__

            # --- start vibration ---
            self._set_test({"status": "running", "run_id": run_id, "phase": "vibration_start", "result": result})
            with self.reader._lock:
                self.reader._la10.start_vibration(
                    frequency_hz=frequency_hz, duty_permyriad=duty_permyriad,
                )
                vib_settings = self.reader._la10.read_vibration_settings()
            if not vib_settings.enabled:
                raise RuntimeError("LA10 did not confirm vibration enable after start")
            vibration_started = True
            result["vibration_started"] = vib_settings.__dict__
            vibration_start_time = time.monotonic()
            started_at = vibration_start_time
            startup_delay_s = None  # filled when first valid mass increase is detected

            # --- create rate-regulating feedback controller ---
            cr_controller = ConstantRateController(
                target_rate_mg_s=target_rate_mg_s,
                initial_duty_permyriad=duty_permyriad,
                initial_window_units=window_units,
                frequency_hz=frequency_hz,
            )

            # --- sample loop across all repeats ---
            motion_estimator = MassMotionEstimator(
                MassMotionEstimatorSettings(
                    alpha=0.20,      # smoother mass filtering (was 0.45)
                    beta=0.03,       # much smoother rate (was 0.10)
                    gamma=0.002,     # smoother acceleration (was 0.008)
                    jerk_alpha=0.08, # smoother jerk (was 0.20)
                    min_valid_elapsed_s=2.0,
                    min_valid_samples=10,
                    prediction_horizon_s=0.40,
                )
            )
            tare_mass_mg = float(
                tare_after.mass_mg if tare_after.mass_mg is not None else tare_after.raw_count
            )

            _last_locked = False  # track lock transitions for control_updates
            _last_clogged = False
            for seg_idx in range(1, repeat_count + 1):
                if self._cancel_event.is_set():
                    result["result"] = "cancelled"
                    break
                seg_started_at = time.monotonic()
                seg_samples = []
                self._set_test({
                    "status": "running", "run_id": run_id, "phase": "vibrating",
                    "segment_index": seg_idx, "segment_count": repeat_count,
                    "result": result,
                })
                while time.monotonic() - seg_started_at < segment_duration_s:
                    if self._cancel_event.is_set():
                        result["result"] = "cancelled"
                        break
                    sample_time = time.monotonic()
                    with self.reader._lock:
                        measurement = self.reader._at.read_measurement()
                    measured_mass = (
                        measurement.mass_mg
                        if measurement.mass_mg is not None
                        else measurement.raw_count
                    )
                    mass_mg = max(0.0, float(measured_mass))
                    motion = motion_estimator.update(sample_time, mass_mg)

                    # detect startup delay: first sample with valid rate > 0.5 mg/s
                    if startup_delay_s is None and motion.valid and motion.rate_mg_s > 0.5:
                        startup_delay_s = round(sample_time - vibration_start_time, 3)

                    # --- feedback step ---
                    cr_decision = cr_controller.step(
                        ConstantRateObservation(
                            rate_mg_s=motion.rate_mg_s,
                            predicted_rate_mg_s=motion.predicted_rate_mg_s,
                            acceleration_mg_s2=motion.acceleration_mg_s2,
                            valid=motion.valid,
                        ),
                        timestamp_s=sample_time,
                    )
                    if cr_decision.freq_changed or cr_decision.duty_changed:
                        try:
                            with self.reader._lock:
                                self.reader._la10.update_vibration_parameters(
                                    frequency_hz=cr_decision.frequency_after_hz,
                                    duty_permyriad=cr_decision.duty_after_permyriad,
                                )
                        except Exception:
                            pass  # parameter update failure is non-fatal; log and continue
                    if cr_decision.window_changed:
                        try:
                            self._move_window_to(
                                cr_decision.window_after_units,
                                speed_mm_s=1.0,
                                timeout_s=5.0,
                            )
                        except Exception:
                            pass  # window move failure is non-fatal
                    if (cr_decision.freq_changed or cr_decision.duty_changed
                            or cr_decision.window_changed
                            or cr_decision.locked != _last_locked
                            or cr_decision.clogged != _last_clogged):
                        result["control_updates"].append({
                            "elapsed_s": round(sample_time - started_at, 3),
                            "segment_index": seg_idx,
                            "freq_before": cr_decision.frequency_before_hz,
                            "freq_after": cr_decision.frequency_after_hz,
                            "duty_before": cr_decision.duty_before_permyriad,
                            "duty_after": cr_decision.duty_after_permyriad,
                            "window_before": cr_decision.window_before_units,
                            "window_after": cr_decision.window_after_units,
                            "rate_error_mg_s": round(cr_decision.rate_error_mg_s, 3),
                            "control_rate_mg_s": round(cr_decision.control_rate_mg_s, 3),
                            "locked": cr_decision.locked,
                            "clogged": cr_decision.clogged,
                            "reason": cr_decision.reason,
                        })
                    _last_locked = cr_decision.locked
                    _last_clogged = cr_decision.clogged

                    sample = {
                        "elapsed_s": round(sample_time - started_at, 3),
                        "segment_elapsed_s": round(sample_time - seg_started_at, 3),
                        "segment_index": seg_idx,
                        "mass_mg": round(mass_mg, 3),
                        "raw_count": measurement.raw_count,
                        "stable": measurement.stable,
                        "status_word": measurement.status_word,
                        "filtered_mass_mg": round(motion.filtered_mass_mg, 3),
                        "rate_valid": motion.valid,
                        "rate_mg_s": round(motion.rate_mg_s, 3),
                        "acceleration_mg_s2": round(motion.acceleration_mg_s2, 3),
                        "frequency_hz": cr_decision.frequency_after_hz,
                        "duty_permyriad": cr_decision.duty_after_permyriad,
                        "window_position_units": cr_decision.window_after_units,
                        "target_rate_mg_s": cr_decision.target_rate_mg_s,
                        "rate_error_mg_s": round(cr_decision.rate_error_mg_s, 3),
                        "locked": cr_decision.locked,
                        "clogged": cr_decision.clogged,
                        "feedback_reason": cr_decision.reason,
                    }
                    result["samples"].append(sample)
                    seg_samples.append(sample)

                    # running segment statistics
                    valid_rates = [s["rate_mg_s"] for s in seg_samples if s["rate_valid"]]
                    seg_mean = (
                        round(sum(valid_rates) / len(valid_rates), 3) if valid_rates else None
                    )
                    seg_peak = round(max(valid_rates), 3) if valid_rates else None
                    if valid_rates and len(valid_rates) >= 5:
                        mean_v = sum(valid_rates) / len(valid_rates)
                        variance = sum((r - mean_v) ** 2 for r in valid_rates) / len(valid_rates)
                        seg_cv = round((variance ** 0.5) / mean_v * 100, 2) if mean_v > 0 else None
                    else:
                        seg_cv = None

                    latest_sample = dict(sample)
                    latest_sample["segment_mean_rate_mg_s"] = seg_mean
                    latest_sample["segment_peak_rate_mg_s"] = seg_peak
                    latest_sample["segment_cv_pct"] = seg_cv
                    self._set_test({
                        "status": "running", "run_id": run_id, "phase": "vibrating",
                        "segment_index": seg_idx, "segment_count": repeat_count,
                        "latest_sample": latest_sample, "result": result,
                    })
                    wait_s = max(0.0, POLL_INTERVAL_S - (time.monotonic() - sample_time))
                    if self._cancel_event.wait(wait_s):
                        result["result"] = "cancelled"
                        break

                if result.get("result") == "cancelled":
                    break

                # --- per-segment final statistics ---
                valid_rates_seg = [s["rate_mg_s"] for s in seg_samples if s["rate_valid"]]
                if valid_rates_seg:
                    seg_mean_rate = round(sum(valid_rates_seg) / len(valid_rates_seg), 3)
                    seg_peak_rate = round(max(valid_rates_seg), 3)
                    seg_std = (
                        round(
                            (sum((r - seg_mean_rate) ** 2 for r in valid_rates_seg)
                             / len(valid_rates_seg)) ** 0.5,
                            3,
                        )
                        if len(valid_rates_seg) >= 2
                        else None
                    )
                    seg_cv_pct = (
                        round(seg_std / seg_mean_rate * 100, 2)
                        if seg_std is not None and seg_mean_rate > 0
                        else None
                    )
                else:
                    seg_mean_rate = seg_peak_rate = seg_std = seg_cv_pct = None

                seg_mass_gain = round(
                    seg_samples[-1]["mass_mg"] - seg_samples[0]["mass_mg"], 3
                ) if seg_samples else None
                seg_passed = (
                    seg_mean_rate is not None
                    and abs(seg_mean_rate - target_rate_mg_s) <= rate_tolerance_mg_s
                    and seg_cv_pct is not None
                    and seg_cv_pct <= cv_pct_limit
                )
                segment_summary = {
                    "segment_index": seg_idx,
                    "actual_duration_s": round(time.monotonic() - seg_started_at, 3),
                    "sample_count": len(seg_samples),
                    "mass_before_mg": seg_samples[0]["mass_mg"] if seg_samples else None,
                    "mass_after_mg": seg_samples[-1]["mass_mg"] if seg_samples else None,
                    "mass_gain_mg": seg_mass_gain,
                    "mean_rate_mg_s": seg_mean_rate,
                    "peak_rate_mg_s": seg_peak_rate,
                    "std_mg_s": seg_std,
                    "cv_pct": seg_cv_pct,
                    "rate_passed": (
                        seg_mean_rate is not None
                        and abs(seg_mean_rate - target_rate_mg_s) <= rate_tolerance_mg_s
                    ),
                    "stability_passed": seg_cv_pct is not None and seg_cv_pct <= cv_pct_limit,
                    "passed": seg_passed,
                }
                result["segments"].append(segment_summary)

            if result.get("result") != "cancelled":
                result["result"] = "completed"
            result["actual_duration_s"] = round(time.monotonic() - started_at, 3)
            result["startup_delay_s"] = startup_delay_s

        except Exception as error:
            result["result"] = "failed"
            result["error"] = str(error)
        finally:
            if vibration_started:
                try:
                    with self.reader._lock:
                        self.reader._la10.stop_vibration()
                        vibration_after = self.reader._la10.read_vibration_settings()
                    if started_at is not None:
                        result["actual_duration_s"] = round(time.monotonic() - started_at, 3)
                    result["vibration_after"] = vibration_after.__dict__
                    if vibration_after.enabled:
                        result["result"] = "failed"
                        result["stop_error"] = (
                            "LA10 vibration enable register remained set after stop command"
                        )
                    else:
                        self._set_test({
                            "status": "running", "run_id": run_id,
                            "phase": "post_vibration_settle", "result": result,
                        })
                        time.sleep(POST_VIBRATION_SETTLE_S)
                        mass_after = self._wait_for_stable_measurement(TARE_TIMEOUT_S)
                        result["mass_after_vibration"] = mass_after.__dict__
                        tare_raw = result["tare_after"]["raw_count"]
                        result["mass_gain_from_tare_mg"] = mass_after.raw_count - tare_raw
                        final_mass = (
                            mass_after.mass_mg
                            if mass_after.mass_mg is not None
                            else mass_after.raw_count
                        )
                        result["final_mass_mg"] = float(final_mass)
                except Exception as error:
                    result["result"] = "failed"
                    result["post_vibration_measurement_error"] = str(error)

            # --- compute overall summary from pooled raw samples ---
            # NOTE: must pool raw per-sample rates (not per-segment means) so CV
            # reflects actual sample-to-sample variability. With repeat_count=1
            # there's only one segment mean, which previously made overall_std
            # collapse to a hardcoded 0.0 regardless of real flow noise.
            overall_valid_rates = [
                s["rate_mg_s"] for s in result["samples"] if s["rate_valid"]
            ]
            if overall_valid_rates:
                overall_mean = round(sum(overall_valid_rates) / len(overall_valid_rates), 3)
                overall_std = (
                    round(
                        (
                            sum((r - overall_mean) ** 2 for r in overall_valid_rates)
                            / len(overall_valid_rates)
                        ) ** 0.5,
                        3,
                    )
                    if len(overall_valid_rates) >= 2
                    else 0.0
                )
                overall_cv = (
                    round(overall_std / overall_mean * 100, 2)
                    if overall_mean > 0
                    else None
                )
                overall_rate_passed = abs(overall_mean - target_rate_mg_s) <= rate_tolerance_mg_s
                # overall stability passed if every segment individually passes CV limit
                overall_stability_passed = bool(result["segments"]) and all(
                    s.get("stability_passed", False) for s in result["segments"]
                )
            else:
                overall_mean = overall_std = overall_cv = None
                overall_rate_passed = overall_stability_passed = False

            result["summary"] = {
                "overall_mean_rate_mg_s": overall_mean,
                "overall_std_mg_s": overall_std,
                "overall_cv_pct": overall_cv,
                "rate_passed": overall_rate_passed,
                "stability_passed": overall_stability_passed,
                "overall_passed": overall_rate_passed and overall_stability_passed,
                "startup_delay_s": result.get("startup_delay_s"),
            }

            result["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            self._persist_result("constant_rate_dispense", run_id, result)
            self._set_test({"status": result["result"], "run_id": run_id, "result": result})

    def _run_constant_rate_batch(self, run_id, submitted):
        """Execute a queue of constant-rate tests sequentially."""
        sets = submitted["sets"]
        target_rate_mg_s = submitted["target_rate_mg_s"]
        tare_between = submitted["tare_between_sets"]
        batch_result = {
            "run_id": run_id,
            "test_type": "constant_rate_batch",
            "target_rate_mg_s": target_rate_mg_s,
            "tare_between_sets": tare_between,
            "set_count": len(sets),
            "set_results": [],
            "started_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        try:
            for set_idx, params in enumerate(sets, start=1):
                if self._cancel_event.is_set():
                    batch_result["result"] = "cancelled"
                    break
                freq = params["frequency_hz"]
                duty = params["duty_permyriad"]
                win = params["window_position_units"]
                dur = params.get("duration_s", 15)
                reps = params.get("repeat_count", 3)
                set_run_id = (
                    f"{run_id}-set{set_idx:02d}-{freq}hz-{duty // 100}pct"
                )
                self._set_test({
                    "status": "running", "run_id": run_id,
                    "phase": f"batch_set_{set_idx}_of_{len(sets)}",
                    "batch_set_index": set_idx,
                    "batch_set_count": len(sets),
                    "batch_set_run_id": set_run_id,
                    "batch_summary": {
                        "completed": set_idx - 1,
                        "total": len(sets),
                        "passed_so_far": sum(
                            1 for r in batch_result["set_results"] if r.get("overall_passed")
                        ),
                    },
                })
                single_submitted = {
                    "test_type": "constant_rate_dispense",
                    "target_rate_mg_s": target_rate_mg_s,
                    "frequency_hz": freq,
                    "duty_permyriad": duty,
                    "window_position_units": win,
                    "duration_s": dur,
                    "repeat_count": reps,
                }
                # Run the set directly (not via start_constant_rate_test, which
                # checks for a running test). We already hold _test_lock implicitly
                # because the batch is the active test.
                self._run_constant_rate_test(set_run_id, single_submitted)
                set_snapshot = self.snapshot()
                set_result = set_snapshot.get("test", {}).get("result", {})
                if not set_result:
                    set_result = {"run_id": set_run_id, "result": "failed", "error": "no result snapshot"}
                batch_result["set_results"].append({
                    "set_index": set_idx,
                    "frequency_hz": freq,
                    "duty_permyriad": duty,
                    "window_position_units": win,
                    "duration_s": dur,
                    "repeat_count": reps,
                    "run_id": set_run_id,
                    "result": set_result.get("result"),
                    "error": set_result.get("error"),
                    "segments": set_result.get("segments", []),
                    "summary": set_result.get("summary", {}),
                    "startup_delay_s": set_result.get("startup_delay_s"),
                    "final_mass_mg": set_result.get("final_mass_mg"),
                })
                if set_result.get("result") == "failed":
                    if not tare_between:
                        break  # cannot continue without a clean tare
            if batch_result.get("result") != "cancelled":
                batch_result["result"] = "completed"
        except Exception as error:
            batch_result["result"] = "failed"
            batch_result["error"] = str(error)
        finally:
            # cross-set comparison summary
            set_summaries = [
                s["summary"] for s in batch_result["set_results"] if s.get("summary")
            ]
            passed_sets = sum(1 for s in set_summaries if s.get("overall_passed"))
            means_list = [
                s["overall_mean_rate_mg_s"] for s in set_summaries
                if s["overall_mean_rate_mg_s"] is not None
            ]
            batch_result["cross_set_summary"] = {
                "total_sets": len(batch_result["set_results"]),
                "passed_sets": passed_sets,
                "pass_rate": (
                    round(passed_sets / max(1, len(batch_result["set_results"])) * 100, 1)
                    if batch_result["set_results"]
                    else 0
                ),
                "best_mean_rate_mg_s": round(min(
                    means_list, key=lambda r: abs(r - target_rate_mg_s)
                ), 3) if means_list else None,
                "best_set_index": (
                    min(
                        range(len(means_list)),
                        key=lambda i: abs(means_list[i] - target_rate_mg_s),
                    )
                    + 1
                ) if means_list else None,
            }
            batch_result["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            self._persist_result("constant_rate_batch", run_id, batch_result)
            self._set_test({
                "status": batch_result["result"], "run_id": run_id, "result": batch_result,
            })

    def _run_dispense_batch(self, run_id, submitted):
        """Execute a queue of feedback-dispense tests sequentially."""
        sets = submitted["sets"]
        tare_between = submitted["tare_between_sets"]
        close_window_between_runs = submitted.get("close_window_between_runs", True)
        batch_result = {
            "run_id": run_id,
            "test_type": "dispense_batch",
            "powder_id": submitted["powder_id"],
            "powder_name": submitted["powder_name"],
            "tare_between_sets": tare_between,
            "close_window_between_runs": close_window_between_runs,
            "set_count": len(sets),
            "set_results": [],
            "started_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        try:
            for set_idx, params in enumerate(sets, start=1):
                if self._cancel_event.is_set():
                    batch_result["result"] = "cancelled"
                    break
                target_mg = int(params["target_mg"])
                freq = int(params["frequency_hz"])
                duty = int(params["duty_permyriad"])
                win = int(params["window_position_units"])
                requested_powder = {
                    "powder_id": params["powder_id"],
                    "powder_name": params["powder_name"],
                }
                reps = int(params.get("repeat_count", 1))
                configuration = self.resolve_dispense_configuration({
                    **requested_powder,
                    "target_mg": target_mg,
                    "preset_id": params.get("preset_id"),
                    "initial": {
                        "frequency_hz": freq,
                        "duty_permyriad": duty,
                        "window_position_units": win,
                    },
                })
                powder = {
                    "powder_id": configuration["powder_id"],
                    "powder_name": configuration["powder_name"],
                }
                preset_id = configuration["preset_id"]
                preset = configuration["initial"]
                profile = configuration["profile"]
                powder_overrides = configuration["controller"]
                set_results_for_entry = []
                batch_abort_reason = None
                for rep in range(reps):
                    if self._cancel_event.is_set():
                        break
                    set_run_id = (
                        f"{run_id}-s{set_idx:02d}r{rep+1:02d}-{target_mg}mg-{freq}hz-{duty//100}pct"
                    )
                    self._set_test({
                        "status": "running", "run_id": run_id,
                        "phase": f"set_{set_idx}_of_{len(sets)}_rep_{rep+1}_of_{reps}",
                        "batch_set_index": set_idx,
                        "batch_set_count": len(sets),
                        "batch_summary": {
                            "completed": set_idx - 1,
                            "total": len(sets),
                            "passed_so_far": sum(
                                1 for r in batch_result["set_results"]
                                if r.get("acceptance_passed")
                            ),
                        },
                    })
                    self._run_feedback_dispense(
                        set_run_id, target_mg, preset_id, preset, profile,
                        powder_overrides, powder
                    )
                    snap = self.snapshot()
                    rep_result = snap.get("test", {}).get("result", {})
                    if not rep_result:
                        rep_result = {"run_id": set_run_id, "result": "failed", "error": "no result snapshot"}
                    if close_window_between_runs:
                        self._set_test({
                            "status": "running", "run_id": run_id,
                            "phase": f"closing_window_after_set_{set_idx}_rep_{rep+1}",
                            "batch_set_index": set_idx,
                            "batch_set_count": len(sets),
                        })
                        try:
                            rep_result["window_close_after_run"] = self._move_window_to_minimum()
                        except Exception as error:
                            rep_result["window_close_error"] = str(error)
                            # Preserve a dispense failure as the primary batch
                            # outcome. It already prevents the next repeat;
                            # the close error remains in the detailed result.
                            if rep_result.get("result") == "completed":
                                batch_abort_reason = (
                                    "window close failed; remaining repetitions were not started"
                                )
                    set_results_for_entry.append(rep_result)
                    if batch_abort_reason:
                        break
                    # A measured mass outside the acceptance band is a test
                    # result, not a device-safety failure. Keep running the
                    # requested repetitions and retain the failed measurement
                    # in the batch log. Communication, motion, stability, or
                    # cancellation failures have no acceptance_error and still
                    # stop the queue safely.
                    if (
                        rep_result.get("result") != "completed"
                        and not rep_result.get("acceptance_error")
                    ):
                        batch_abort_reason = (
                            "dispense repetition failed acceptance; remaining repetitions were not started"
                        )
                        break
                    if "stall recovery exhausted" in str(rep_result.get("stop_reason", "")):
                        batch_abort_reason = (
                            "stall recovery exhausted; remaining repetitions were not started"
                        )
                        break
                    if rep_result.get("window_restore_error"):
                        batch_abort_reason = (
                            "window restore failed; remaining repetitions were not started"
                        )
                        break
                # Aggregate per-set result from all reps
                final_masses = [
                    r["final_mass_mg"] for r in set_results_for_entry
                    if r.get("final_mass_mg") is not None
                ]
                acceptance_passed = (
                    len(set_results_for_entry) == reps
                    and all(
                        r.get("result") == "completed"
                        for r in set_results_for_entry
                    )
                )
                durations = [
                    r.get("actual_duration_s", 0) for r in set_results_for_entry
                ]
                batch_result["set_results"].append({
                    "set_index": set_idx,
                    **powder,
                    "target_mg": target_mg,
                    "frequency_hz": freq,
                    "duty_permyriad": duty,
                    "window_position_units": win,
                    "repeat_count": reps,
                    "result": "completed" if acceptance_passed else "failed",
                    "acceptance_passed": acceptance_passed,
                    "final_mass_mg": round(sum(final_masses) / len(final_masses), 1) if final_masses else None,
                    "actual_duration_s": round(sum(durations) / len(durations), 1) if durations else None,
                    "rep_results": [
                        {"run_id": r.get("run_id"), "result": r.get("result"),
                         "powder_id": r.get("powder_id"),
                          "powder_name": r.get("powder_name"),
                          "final_mass_mg": r.get("final_mass_mg"),
                          "error": r.get("error"),
                          "acceptance_error": r.get("acceptance_error"),
                          "window_close_after_run": r.get("window_close_after_run"),
                         "window_close_error": r.get("window_close_error")}
                        for r in set_results_for_entry
                    ],
                })
                if batch_abort_reason:
                    batch_result["result"] = "failed"
                    batch_result["error"] = batch_abort_reason
                    break
                if set_results_for_entry and all(
                    r.get("result") == "failed" for r in set_results_for_entry
                ):
                    if not tare_between:
                        break
            if batch_result.get("result") not in ("cancelled", "failed"):
                all_sets_passed = bool(batch_result["set_results"]) and all(
                    item.get("acceptance_passed")
                    for item in batch_result["set_results"]
                )
                # Finishing the requested queue and passing its mass
                # acceptance criteria are separate outcomes. A mass deviation
                # remains visible in each set/repetition result and log, but
                # is not reported as a batch execution error.
                batch_result["result"] = "completed"
                batch_result["acceptance_passed"] = all_sets_passed
        except Exception as error:
            batch_result["result"] = "failed"
            batch_result["error"] = str(error)
        finally:
            total = len(batch_result["set_results"])
            passed = sum(1 for s in batch_result["set_results"] if s.get("acceptance_passed"))
            final_masses_all = [
                s["final_mass_mg"] for s in batch_result["set_results"]
                if s.get("final_mass_mg") is not None
            ]
            batch_result["cross_set_summary"] = {
                "total_sets": total,
                "passed_sets": passed,
                "pass_rate": round(passed / max(1, total) * 100, 1) if total else 0,
                "avg_final_mass_mg": round(sum(final_masses_all) / len(final_masses_all), 1) if final_masses_all else None,
            }
            batch_result["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            self._persist_result("dispense_batch", run_id, batch_result)
            self._set_test({
                "status": batch_result["result"], "run_id": run_id, "result": batch_result,
            })

    def _move_window_to(
        self,
        target_position,
        *,
        speed_mm_s,
        timeout_s,
        # The LA10 encoder can settle a few units beyond the requested target
        # when the mechanism stops. Treat positions within ±30 units as the
        # requested feed window; the close-window path still uses its separate
        # one-sided <= minimum rule.
        position_tolerance_units=30,
        respect_cancel=True,
        max_correction_attempts=2,
        max_result_retries=1,
        accept_at_or_below_target=False,
    ):
        with self.reader._lock:
            before = self.reader._la10.read_status()
        if before.fault_bits:
            raise RuntimeError(
                f"LA10 cannot move: running={before.is_running}, fault=0x{before.fault_bits:04X}"
            )
        if before.is_running:
            # A preceding move can leave the gateway's running flag asserted
            # briefly after the mechanism has physically stopped. Do not issue
            # another command into that transition; wait for a quiet state.
            before = self._wait_for_window_idle(
                min(timeout_s, 5.0), respect_cancel=respect_cancel
            )
        if max_correction_attempts < 0 or max_result_retries < 0:
            raise ValueError("window move retry limits must be non-negative")

        current = before
        total_distance_register = 0
        correction_attempts = 0
        result_retries = 0
        while True:
            delta = abs(target_position - current.position_units)
            if delta <= position_tolerance_units:
                return before, current, total_distance_register
            distance_register = motion_distance_register(delta)
            with self.reader._lock:
                self.reader._la10.enable()
                self.reader._la10.move_relative(
                    extend=target_position > current.position_units,
                    distance_mm=distance_register / MOTION_REGISTER_UNITS_PER_MM,
                    speed_mm_s=speed_mm_s,
                )
            total_distance_register += distance_register
            after = self._wait_for_window_move(
                timeout_s,
                target_position=target_position,
                position_tolerance_units=position_tolerance_units,
                respect_cancel=respect_cancel,
                initial_position=current.position_units,
                initial_last_result=current.last_result,
            )
            if after.fault_bits:
                raise RuntimeError(f"LA10 故障位：0x{after.fault_bits:04X}")
            # The encoder position is the useful completion signal for this
            # mechanism. A transient non-zero result code is not allowed to
            # reject an otherwise stopped, in-tolerance move and trigger a
            # second command while the gateway state is settling.
            current = after
            if (
                abs(current.position_units - target_position) <= position_tolerance_units
                or (
                    accept_at_or_below_target
                    and current.position_units <= target_position
                )
            ):
                return before, current, total_distance_register
            if after.last_result != 0:
                # A non-zero result can be stale from an earlier command. If
                # the encoder reached this command's target, the move itself
                # succeeded and the stale status must not abort dispensing.
                if abs(after.position_units - target_position) <= position_tolerance_units:
                    return before, after, total_distance_register
                if result_retries >= max_result_retries:
                    raise RuntimeError(
                        "LA10 运动结果码："
                        f"{after.last_result}，重试后仍失败；"
                        f"位置={after.position_units}，目标={target_position}，"
                        f"target寄存器={after.target_units}，运行中={after.is_running}，"
                        f"故障位=0x{after.fault_bits:04X}"
                    )
                result_retries += 1
                # The result code and running bit are published by the LA10
                # gateway asynchronously. Wait for the motion state to settle
                # before deciding whether a correction is actually needed.
                current = self._wait_for_window_idle(
                    min(timeout_s, 5.0), respect_cancel=respect_cancel
                )
                continue
            if correction_attempts >= max_correction_attempts:
                raise RuntimeError(
                    "LA10 stopped outside target tolerance after corrections: "
                    f"target={target_position}, actual={current.position_units}, "
                    f"tolerance={position_tolerance_units}"
                )
            correction_attempts += 1

    def _persist_result(self, prefix, run_id, result):
        try:
            log_path = PROJECT_ROOT / "logs" / f"{prefix}_{run_id}.json"
            samples_path = PROJECT_ROOT / "logs" / f"{prefix}_{run_id}_samples.csv"
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_path.write_text(
                json.dumps(result, ensure_ascii=False, indent=2, default=json_default),
                encoding="utf-8",
            )
            self._write_csv(samples_path, result.get("samples", []))
            result["log_path"] = str(log_path)
            result["samples_csv_path"] = str(samples_path)
            post_stop_samples = result.get("post_stop_samples", [])
            if post_stop_samples:
                post_stop_path = PROJECT_ROOT / "logs" / f"{prefix}_{run_id}_post_stop_samples.csv"
                self._write_csv(post_stop_path, post_stop_samples)
                result["post_stop_samples_csv_path"] = str(post_stop_path)
        except Exception as error:
            result["log_error"] = str(error)

    def _run_window_position_sweep(self, run_id, payload):
        """Vibrate continuously while moving through and sampling every window point."""
        positions = payload["position_schedule"]
        result = {
            "run_id": run_id,
            "test_type": "window_position_sweep",
            "position_schedule": positions,
            "frequency_hz": payload["frequency_hz"],
            "duty_permyriad": payload["duty_permyriad"],
            "duration_s_per_position": payload["duration_s"],
            "samples": [],
            "segments": [],
        }
        vibration_started = False
        started_at = time.monotonic()
        try:
            with self.reader._lock:
                window_before = self.reader._la10.read_status()
                vibration_before = self.reader._la10.read_vibration_settings()
            if window_before.is_running:
                raise RuntimeError("LA10 window is already moving")
            if window_before.fault_bits:
                raise RuntimeError(f"LA10 fault bits before sweep: 0x{window_before.fault_bits:04X}")
            if vibration_before.enabled:
                with self.reader._lock:
                    self.reader._la10.stop_vibration()
                raise RuntimeError("vibration was enabled and has been stopped; inspect before retrying")

            self._set_test({"status": "running", "run_id": run_id, "phase": "tare", "result": result})
            with self.reader._lock:
                tare_before = self.reader._at.read_measurement()
                self.reader._at.zero(authorized=True)
            result["tare_before"] = tare_before.__dict__
            tare_after = self._wait_for_stable_measurement(TARE_TIMEOUT_S)
            result["tare_after"] = tare_after.__dict__

            self._set_test(
                {"status": "running", "run_id": run_id, "phase": "vibration_start", "result": result}
            )
            # Keep the feeder material in its live flow state. Motion commands
            # below update only registers 1-4 and must not interrupt vibration.
            vibration_started = True
            with self.reader._lock:
                self.reader._la10.start_vibration(
                    frequency_hz=payload["frequency_hz"],
                    duty_permyriad=payload["duty_permyriad"],
                )
                vibration_settings = self.reader._la10.read_vibration_settings()
            result["vibration_started"] = vibration_settings.__dict__
            if not vibration_settings.enabled:
                raise RuntimeError("LA10 did not confirm vibration enable before the sweep")

            for segment_index, target_position in enumerate(positions, start=1):
                if self._cancel_event.is_set():
                    result["result"] = "cancelled"
                    break
                self._set_test(
                    {
                        "status": "running",
                        "run_id": run_id,
                        "phase": "window_move",
                        "segment_index": segment_index,
                        "segment_count": len(positions),
                        "target_window_position_units": target_position,
                        "result": result,
                    }
                )
                with self.reader._lock:
                    before = self.reader._la10.read_status()
                if before.fault_bits:
                    raise RuntimeError(f"LA10 fault bits before position {target_position}: 0x{before.fault_bits:04X}")
                if before.is_running:
                    raise RuntimeError("LA10 window began moving unexpectedly")
                delta = abs(target_position - before.position_units)
                distance_register = motion_distance_register(delta)
                # Calibration points are 50 units apart, narrower than the
                # normal 100-unit acceptance tolerance. Always command a
                # move when the requested point is different.
                if delta > 0:
                    with self.reader._lock:
                        self.reader._la10.enable()
                        self.reader._la10.move_relative(
                            extend=target_position > before.position_units,
                            distance_mm=distance_register / MOTION_REGISTER_UNITS_PER_MM,
                            speed_mm_s=payload["window_speed_mm_s"],
                        )
                    after = self._wait_for_window_move(
                        payload["window_timeout_s"],
                        target_position=target_position,
                        position_tolerance_units=10,
                    )
                else:
                    after = before
                    distance_register = 0
                if abs(after.position_units - target_position) > POSITION_TOLERANCE_UNITS:
                    raise RuntimeError(
                        f"window outside tolerance at position {target_position}: "
                        f"actual={after.position_units}"
                    )

                segment_started_at = time.monotonic()
                segment_samples = []
                estimator = MassMotionEstimator()
                while time.monotonic() - segment_started_at < payload["duration_s"]:
                    if self._cancel_event.is_set():
                        result["result"] = "cancelled"
                        break
                    sample_time = time.monotonic()
                    with self.reader._lock:
                        measurement = self.reader._at.read_measurement()
                    measured_mass = (
                        measurement.mass_mg
                        if measurement.mass_mg is not None
                        else measurement.raw_count
                    )
                    motion = estimator.update(sample_time, max(0.0, float(measured_mass)))
                    sample = {
                        "elapsed_s": round(sample_time - started_at, 3),
                        "segment_index": segment_index,
                        "window_position_units": target_position,
                        "frequency_hz": payload["frequency_hz"],
                        "duty_permyriad": payload["duty_permyriad"],
                        "mass_mg": round(max(0.0, float(measured_mass)), 3),
                        "raw_count": measurement.raw_count,
                        "stable": measurement.stable,
                        "status_word": measurement.status_word,
                        "filtered_mass_mg": round(motion.filtered_mass_mg, 3),
                        "rate_valid": motion.valid,
                        "rate_mg_s": round(motion.rate_mg_s, 3),
                        "acceleration_mg_s2": round(motion.acceleration_mg_s2, 3),
                        "jerk_mg_s3": round(motion.jerk_mg_s3, 3),
                    }
                    result["samples"].append(sample)
                    segment_samples.append(sample)
                    self._set_test(
                        {
                            "status": "running",
                            "run_id": run_id,
                            "phase": "vibrating",
                            "segment_index": segment_index,
                            "segment_count": len(positions),
                            "target_window_position_units": target_position,
                            "latest_sample": sample,
                            "result": result,
                        }
                    )
                    self._cancel_event.wait(
                        max(0, POLL_INTERVAL_S - (time.monotonic() - sample_time))
                    )

                valid_rates = [sample["rate_mg_s"] for sample in segment_samples if sample["rate_valid"]]
                result["segments"].append(
                    {
                        "segment_index": segment_index,
                        "target_window_position_units": target_position,
                        "window_before": before.__dict__,
                        "window_after": after.__dict__,
                        "window_distance_register": distance_register,
                        "actual_duration_s": round(time.monotonic() - segment_started_at, 3),
                        "sample_count": len(segment_samples),
                        "mass_before_mg": segment_samples[0]["mass_mg"] if segment_samples else None,
                        "mass_after_mg": segment_samples[-1]["mass_mg"] if segment_samples else None,
                        "mass_gain_mg": round(
                            segment_samples[-1]["mass_mg"] - segment_samples[0]["mass_mg"], 3
                        ) if segment_samples else None,
                        "net_rate_mg_s": round(
                            (segment_samples[-1]["mass_mg"] - segment_samples[0]["mass_mg"])
                            / max(time.monotonic() - segment_started_at, 0.001),
                            3,
                        ) if segment_samples else None,
                        "mean_rate_mg_s": round(sum(valid_rates) / len(valid_rates), 3) if valid_rates else None,
                    }
                )
                if result.get("result") == "cancelled":
                    break
            if result.get("result") != "cancelled":
                result["result"] = "completed"
        except Exception as error:
            result["result"] = "failed"
            result["error"] = str(error)
        finally:
            if vibration_started:
                try:
                    with self.reader._lock:
                        self.reader._la10.stop_vibration()
                except Exception as error:
                    result["result"] = "failed"
                    result["stop_error"] = str(error)
            result["actual_duration_s"] = round(time.monotonic() - started_at, 3)
            result["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            log_path = PROJECT_ROOT / "logs" / f"window_position_sweep_{run_id}.json"
            try:
                log_path.parent.mkdir(parents=True, exist_ok=True)
                log_path.write_text(
                    json.dumps(result, ensure_ascii=False, indent=2, default=json_default),
                    encoding="utf-8",
                )
                result["log_path"] = str(log_path)
                samples_path = PROJECT_ROOT / "logs" / f"window_position_sweep_{run_id}_samples.csv"
                summary_path = PROJECT_ROOT / "logs" / f"window_position_sweep_{run_id}_10s_summary.csv"
                self._write_csv(samples_path, result["samples"])
                self._write_csv(summary_path, result["segments"])
                result["samples_csv_path"] = str(samples_path)
                result["summary_csv_path"] = str(summary_path)
            except Exception as error:
                result["log_error"] = str(error)
            self._set_test({"status": result["result"], "run_id": run_id, "result": result})

    def _wait_for_stable_measurement(
        self,
        timeout_s,
        *,
        minimum_wait_s=0.0,
        continuous_stable_s=0.0,
        max_spread_mg=None,
        trace=None,
        on_sample=None,
    ):
        started_at = time.monotonic()
        deadline = started_at + timeout_s
        latest = None
        stable_since = None
        stable_values = []
        while time.monotonic() < deadline:
            if self._cancel_event.is_set():
                raise RuntimeError("Test cancelled before vibration")
            with self.reader._lock:
                latest = self.reader._at.read_measurement()
            sampled_at = time.monotonic()
            value_mg = latest.mass_mg if latest.mass_mg is not None else latest.raw_count
            elapsed_s = sampled_at - started_at
            sample = {
                "elapsed_after_stop_s": round(elapsed_s, 3),
                "mass_mg": float(value_mg),
                "stable": latest.stable,
                "status_word": latest.status_word,
            }
            if trace is not None:
                trace.append(sample)
            if on_sample is not None:
                on_sample(sample)
            if elapsed_s < minimum_wait_s or not latest.stable:
                stable_since = None
                stable_values = []
            else:
                if stable_since is None:
                    stable_since = sampled_at
                    stable_values = []
                stable_values.append(float(value_mg))
                if (
                    max_spread_mg is not None
                    and max(stable_values) - min(stable_values) > max_spread_mg
                ):
                    stable_since = sampled_at
                    stable_values = [float(value_mg)]
                elif sampled_at - stable_since >= continuous_stable_s:
                    return latest
            time.sleep(POLL_INTERVAL_S)
        raw_weight = latest.raw_count if latest is not None else "unavailable"
        raise TimeoutError(
            f"Scale did not satisfy the stability window within {timeout_s:.1f} s; "
            f"last weight={raw_weight}"
        )

    @staticmethod
    def _write_csv(path, rows):
        if not rows:
            return
        fieldnames = list(rows[0].keys())
        with path.open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)

    def _wait_for_window_move(
        self,
        timeout_s,
        *,
        target_position=None,
        position_tolerance_units=POSITION_TOLERANCE_UNITS,
        respect_cancel=True,
        initial_position=None,
        initial_last_result=0,
    ):
        started_at = time.monotonic()
        deadline = time.monotonic() + timeout_s
        saw_running = False
        idle_at_target_confirmations = 0
        while time.monotonic() < deadline:
            if respect_cancel and self._cancel_event.is_set():
                raise RuntimeError("测试已取消")
            with self.reader._lock:
                status = self.reader._la10.read_status()
            if status.fault_bits:
                raise RuntimeError(f"LA10 故障位：0x{status.fault_bits:04X}")
            if status.is_running:
                saw_running = True
                idle_at_target_confirmations = 0
            elif (
                target_position is not None
                and abs(status.position_units - target_position)
                <= position_tolerance_units
            ):
                # Some gateways publish the requested target before the motion
                # state flips to running. A few target-shaped idle reads are
                # therefore required before vibration can safely start.
                idle_at_target_confirmations += 1
                if saw_running or idle_at_target_confirmations >= WINDOW_TARGET_IDLE_CONFIRMATIONS:
                    return status
            elif saw_running:
                return status
            elif (
                target_position is None
                and
                initial_position is not None
                and status.position_units != initial_position
            ):
                return status
            elif (
                status.last_result != 0
                and (
                    status.last_result != initial_last_result
                    or time.monotonic() - started_at >= 2 * POLL_INTERVAL_S
                )
            ):
                return status
            else:
                idle_at_target_confirmations = 0
            time.sleep(POLL_INTERVAL_S)
        raise TimeoutError("窗口移动超时")

    def _wait_for_window_idle(self, timeout_s, *, respect_cancel=True):
        """Wait for an LA10 idle read after a move.

        The gateway occasionally reports a completed result while its running
        flag is still transitioning. Do not issue a follow-up command until it
        reports idle, but avoid requiring repeated confirmations because that
        status bit can fluctuate on the gateway after a valid move.
        """
        deadline = time.monotonic() + timeout_s
        idle_confirmations = 0
        latest = None
        while time.monotonic() < deadline:
            if respect_cancel and self._cancel_event.is_set():
                raise RuntimeError("测试已取消")
            with self.reader._lock:
                latest = self.reader._la10.read_status()
            if latest.fault_bits:
                raise RuntimeError(f"LA10 故障位：0x{latest.fault_bits:04X}")
            if latest.is_running:
                idle_confirmations = 0
            else:
                idle_confirmations += 1
                if idle_confirmations >= WINDOW_IDLE_CONFIRMATIONS:
                    return latest
            time.sleep(POLL_INTERVAL_S)
        state = "unknown" if latest is None else latest.is_running
        raise TimeoutError(f"LA10 motion did not become idle within {timeout_s:.1f} s; running={state}")

    @staticmethod
    def _validate_test_payload(payload):
        required = ("target_window_position_units", "window_speed_mm_s", "window_timeout_s", "frequency_hz", "duty_permyriad", "duration_s")
        if not all(key in payload for key in required):
            raise ValueError("测试参数不完整")
        if not 0 <= int(payload["target_window_position_units"]) <= 2000:
            raise ValueError("目标位置必须在 0..2000")
        if not 10 <= int(payload["frequency_hz"]) <= 80:
            raise ValueError("频率必须在 10..80 Hz")
        if not 1000 <= int(payload["duty_permyriad"]) <= 5000:
            raise ValueError("占空比必须在 1000..5000")
        schedule = payload.get("duty_schedule", [payload["duty_permyriad"]])
        if not isinstance(schedule, list) or not schedule:
            raise ValueError("duty_schedule must be a non-empty list")
        if any(not isinstance(value, int) or not 1000 <= value <= 5000 for value in schedule):
            raise ValueError("every duty_schedule value must be an integer in 1000..5000")
        segment_duration_s = float(payload.get("segment_duration_s", payload["duration_s"]))
        if float(payload["window_speed_mm_s"]) <= 0 or float(payload["window_timeout_s"]) <= 0 or segment_duration_s <= 0:
            raise ValueError("时间和速度必须为正数")

    @staticmethod
    def _validate_position_sweep_payload(payload):
        required = ("position_schedule", "window_speed_mm_s", "window_timeout_s", "frequency_hz", "duty_permyriad", "duration_s")
        if not all(key in payload for key in required):
            raise ValueError("position sweep parameters are incomplete")
        schedule = payload["position_schedule"]
        if not isinstance(schedule, list) or not schedule:
            raise ValueError("position_schedule must be a non-empty list")
        if any(not isinstance(value, int) or not 100 <= value <= 2050 for value in schedule):
            raise ValueError("every sweep position must be an integer in 100..2050")
        if len(set(schedule)) != len(schedule):
            raise ValueError("position_schedule must not contain duplicate positions")
        if not 10 <= int(payload["frequency_hz"]) <= 80:
            raise ValueError("frequency_hz must be in 10..80")
        if not 1000 <= int(payload["duty_permyriad"]) <= 5000:
            raise ValueError("duty_permyriad must be in 1000..5000")
        if min(float(payload["window_speed_mm_s"]), float(payload["window_timeout_s"]), float(payload["duration_s"])) <= 0:
            raise ValueError("speed, timeout, and duration must be positive")


def make_handler(service):
    class Handler(BaseHTTPRequestHandler):
        def _send_json(self, status, payload):
            body = json.dumps(payload, ensure_ascii=False, default=json_default).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/api/snapshot":
                self._send_json(200, service.snapshot())
            elif path == "/api/dispense-plans":
                self._send_json(200, [
                    {
                        "plan_id": f"dispense-{target_mg}mg",
                        "target_mg": target_mg,
                        "coarse_end_mg": round(
                            target_mg
                            * (
                                0.70
                                if target_mg <= 100
                                else 0.80 if target_mg > 200 else 0.75
                            ),
                            1,
                        ),
                        "acceptance_min_mg": target_mg - DISPENSE_ALLOWED_UNDERWEIGHT_MG,
                        "acceptance_max_mg": target_mg + DISPENSE_ALLOWED_OVERWEIGHT_MG,
                        "profile": profile,
                    }
                    for target_mg, profile in DISPENSE_TARGET_PROFILES.items()
                ])
            elif path == "/api/dispense/presets":
                self._send_json(200, [
                    {"preset_id": preset_id, **parameters}
                    for preset_id, parameters in DISPENSE_PRESETS.items()
                ])
            elif path == "/api/dispense-500mg/presets":
                self._send_json(200, [
                    {"preset_id": preset_id, **parameters}
                    for preset_id, parameters in DISPENSE_PRESETS.items()
                ])
            elif path == "/api/test-records":
                q = parse_qs(urlparse(self.path).query)
                result = service.list_test_records(
                    test_type=q.get("test_type", [None])[0],
                    date_from=q.get("date_from", [None])[0],
                    date_to=q.get("date_to", [None])[0],
                    result_filter=q.get("result", [None])[0],
                )
                self._send_json(200, result)
            elif path == "/api/powder/fingerprints":
                self._send_json(200, list_fingerprints())
            elif path == "/api/control-profiles":
                self._send_json(200, list_control_profiles())
            elif path == "/api/dispense/powders":
                self._send_json(200, service.list_dispense_powders())
            elif path == "/api/dispense/config":
                q = parse_qs(urlparse(self.path).query)
                try:
                    target_text = q.get("target_mg", [str(DISPENSE_TARGET_MG)])[0]
                    target_mg = int(target_text)
                    self._send_json(200, service.resolve_dispense_configuration({
                        "powder_id": q.get("powder_id", [""])[0],
                        "target_mg": target_mg,
                    }))
                except Exception as error:
                    self._send_json(400, {"error": str(error)})
            elif path == "/api/powder/status":
                self._send_json(200, service.get_powder_status())
            elif path == "/api/powder/rate-search/result":
                try:
                    self._send_json(200, service.get_rate_search_result())
                except Exception as error:
                    self._send_json(400, {"error": str(error)})
            elif path.startswith("/api/powder/probe/") and path.endswith("/result"):
                probe_id = path.split("/")[-2]
                try:
                    fingerprint = service.get_probe_result(probe_id)
                    self._send_json(200, fingerprint.to_dict())
                except Exception as error:
                    self._send_json(400, {"error": str(error)})
            elif path.startswith("/api/powder/probe/"):
                probe_id = path.split("/")[-1]
                test_snapshot = service.snapshot().get("test", {})
                self._send_json(200, {
                    "status": test_snapshot.get("status", "idle"),
                    "current_phase": test_snapshot.get("phase", ""),
                    "progress_pct": test_snapshot.get("progress_pct", 0),
                    "result": test_snapshot.get("result"),
                })
            elif path == "/api/experiment/grid/templates":
                self._send_json(200, service.get_grid_templates())
            elif path == "/":
                body = HTML.replace("setInterval(refresh,1000)", "setInterval(refresh,250)").encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)

        def do_POST(self):
            path = urlparse(self.path).path
            try:
                length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(length) or b"{}")
                if path == "/api/vibration-test":
                    self._send_json(202, service.start_feed_test(payload))
                elif path == "/api/window-position-sweep":
                    self._send_json(202, service.start_window_position_sweep(payload))
                elif path == "/api/vibration-test/cancel":
                    self._send_json(202, service.cancel_feed_test())
                elif path == "/api/test-result/save":
                    self._send_json(200, service.save_latest_test_result())
                elif path == "/api/balance/tare":
                    self._send_json(200, service.tare_balance())
                elif path == "/api/dispense/start":
                    self._send_json(202, service.start_dispense(payload))
                elif path == "/api/dispense/large/start":
                    self._send_json(202, service.start_large_dispense(payload))
                elif path == "/api/dispense/cancel":
                    self._send_json(202, service.cancel_feed_test())
                elif path == "/api/dispense-500mg/start":
                    self._send_json(202, service.start_500mg_dispense(payload))
                elif path == "/api/dispense-500mg/cancel":
                    self._send_json(202, service.cancel_feed_test())
                elif path == "/api/continuous-taper/start":
                    self._send_json(202, service.start_continuous_taper_test())
                elif path == "/api/continuous-taper/cancel":
                    self._send_json(202, service.cancel_feed_test())
                elif path == "/api/window-pid/start":
                    self._send_json(202, service.start_window_pid_test())
                elif path == "/api/window-pid/cancel":
                    self._send_json(202, service.cancel_feed_test())
                elif path == "/api/la10/register":
                    self._send_json(200, service.write_la10_register(payload))
                elif path == "/api/window/minimum":
                    self._send_json(200, service.move_window_to_minimum())
                elif path == "/api/constant-rate/start":
                    self._send_json(202, service.start_constant_rate_test(payload))
                elif path == "/api/constant-rate/batch":
                    self._send_json(202, service.start_constant_rate_batch(payload))
                elif path == "/api/dispense/batch":
                    self._send_json(202, service.start_dispense_batch(payload))
                # ── experiment orchestrator ──
                elif path == "/api/experiment/grid/start":
                    self._send_json(202, service.start_grid_experiment(payload))
                elif path == "/api/experiment/grid/collect":
                    self._send_json(200, service.collect_grid_to_library(payload))
                elif path == "/api/constant-rate/cancel":
                    self._send_json(202, service.cancel_feed_test())
                # ── powder fingerprint ──
                elif path == "/api/powder/probe/start":
                    self._send_json(202, service.start_powder_probe(payload))
                elif path == "/api/powder/rate-search/start":
                    self._send_json(202, service.start_rate_search(payload))
                elif path == "/api/powder/category":
                    self._send_json(200, service.set_powder_category(payload.get("category", "")))
                elif path.startswith("/api/powder/fingerprints/") and path.endswith("/apply"):
                    powder_id = path.split("/")[-2]
                    self._send_json(200, service.apply_powder_fingerprint(powder_id))
                elif path.startswith("/api/powder/fingerprints/") and not path.endswith("/apply"):
                    # DELETE — handle via method check
                    powder_id = path.split("/")[-1]
                    result = delete_fingerprint(powder_id)
                    if not result:
                        self._send_json(404, {"error": f"粉末不存在: {powder_id}"})
                    else:
                        self._send_json(200, {"deleted": True, "powder_id": powder_id})
                else:
                    self.send_error(404)
            except Exception as error:
                self._send_json(400, {"error": str(error)})

        def log_message(self, _format, *_args):
            return

    return Handler


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--at-port", default="COM9")
    parser.add_argument("--at-address", type=int, default=1)
    parser.add_argument("--la10-port", default="COM8")
    parser.add_argument("--la10-address", type=int, default=1)
    parser.add_argument(
        "--balance-calibration",
        type=Path,
        default=PROJECT_ROOT / "config" / "devices" / "at8811c-calibration.json",
        help="validated AT8811C count-to-mg calibration JSON",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    balance_calibration = None
    if args.balance_calibration.exists():
        balance_calibration = load_balance_calibration(args.balance_calibration)
        print(
            "balance calibration: "
            f"{balance_calibration.calibration_id}, "
            f"scale={balance_calibration.scale_mg_per_count:.9g} mg/count, "
            f"offset={balance_calibration.offset_mg:.6g} mg"
        )
    else:
        print(
            "WARNING: no validated balance calibration found at "
            f"{args.balance_calibration}; telemetry remains raw_count"
        )
    service = DeviceControlService(
        args.at_port,
        args.la10_port,
        args.at_address,
        args.la10_address,
        balance_calibration=balance_calibration,
    )
    service.start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(service))
    print(f"dashboard: http://{args.host}:{args.port} (poll {int(POLL_INTERVAL_S * 1000)} ms)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        service.close()


if __name__ == "__main__":
    main()
