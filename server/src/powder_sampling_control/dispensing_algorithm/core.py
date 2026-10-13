"""生产出料算法链——单一合并模块。

按依赖顺序合并的各部分：

    1. 产品需求、共享契约、验收限值（原 models.py；
       旧版 L0 仓储 / 良率估计器 / 四阶段支持已随
       four_stage_planner.py 和 ilc.py 一并移除）
    2. 质量-运动观测器（原 mass_motion_estimator.py）
    3. 档位种子、控制档位加载、草稿生成（原 profiles.py）
    4. 连续反馈控制器（原 feedback_controller.py）

纯算法层：此处不打开串口、不下发设备命令；
所有硬件写入均由执行器负责。
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum
from functools import lru_cache
from math import ceil
from pathlib import Path
from time import monotonic
from typing import Any

# ── 产品需求与共享契约（原 models.py）─────────────────────────


@dataclass(frozen=True)
class FeedAction:
    """单条有边界的 LA10 窗口/振动指令。"""

    window_position_units: int
    frequency_hz: int
    duty_permyriad: int
    duration_ms: int


@lru_cache(maxsize=1)
def load_product_requirements() -> dict[str, Any]:
    data = json.loads(DEFAULT_REQUIREMENTS_PATH.read_text(encoding="utf-8"))
    target = data.get("target_mass", {})
    acceptance = data.get("acceptance", {})
    minimum = target.get("minimum_mg")
    maximum = target.get("maximum_mg")
    if not isinstance(minimum, int) or not isinstance(maximum, int) or minimum >= maximum:
        raise ValueError("product target_mass limits must be increasing integers")
    if target.get("integer_only") is not True:
        raise ValueError("this product requires integer target masses")
    for key in ("minimum_offset_mg", "maximum_offset_mg"):
        if not isinstance(acceptance.get(key), (int, float)):
            raise ValueError(f"acceptance.{key} must be numeric")
    return data


def validate_target_mass(target_mg: Any) -> int:
    """返回配置的 100-1000 mg 范围内的有效整数目标质量。"""
    requirements = load_product_requirements()
    limits = requirements["target_mass"]
    if isinstance(target_mg, bool) or not isinstance(target_mg, int):
        raise ValueError(
            f"target_mg must be an integer between {limits['minimum_mg']} "
            f"and {limits['maximum_mg']}"
        )
    if not limits["minimum_mg"] <= target_mg <= limits["maximum_mg"]:
        raise ValueError(
            f"target_mg must be between {limits['minimum_mg']} and "
            f"{limits['maximum_mg']}, got {target_mg}"
        )
    return target_mg


def acceptance_limits(target_mg: Any) -> tuple[float, float]:
    """返回目标值 ±10 mg 的闭区间验收窗口。"""
    target = validate_target_mass(target_mg)
    acceptance = load_product_requirements()["acceptance"]
    return (
        target + float(acceptance["minimum_offset_mg"]),
        target + float(acceptance["maximum_offset_mg"]),
    )


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_REQUIREMENTS_PATH = PROJECT_ROOT / "config" / "product_requirements.json"


@dataclass(frozen=True)
class MassMotionEstimatorSettings:
    alpha: float = 0.45
    beta: float = 0.10
    gamma: float = 0.008
    jerk_alpha: float = 0.20
    min_dt_s: float = 0.05
    max_dt_s: float = 1.0
    min_valid_elapsed_s: float = 1.5
    min_valid_samples: int = 5
    max_rate_mg_s: float = 500.0
    max_acceleration_mg_s2: float = 60.0
    max_jerk_mg_s3: float = 120.0
    prediction_horizon_s: float = 0.75

    def __post_init__(self) -> None:
        for name in ("alpha", "beta", "gamma", "jerk_alpha"):
            value = getattr(self, name)
            if not 0.0 < value <= 1.0:
                raise ValueError(f"{name} must be in (0, 1]")
        if not 0 < self.min_dt_s <= self.max_dt_s:
            raise ValueError("estimator dt limits are invalid")
        if self.min_valid_elapsed_s <= 0 or self.min_valid_samples < 2:
            raise ValueError("estimator validity limits are invalid")
        if min(
            self.max_rate_mg_s,
            self.max_acceleration_mg_s2,
            self.max_jerk_mg_s3,
            self.prediction_horizon_s,
        ) <= 0:
            raise ValueError("estimator bounds and prediction horizon must be positive")


@dataclass(frozen=True)
class MassMotionState:
    timestamp_s: float
    measured_mass_mg: float
    filtered_mass_mg: float
    rate_mg_s: float
    acceleration_mg_s2: float
    jerk_mg_s3: float
    predicted_mass_mg: float
    predicted_rate_mg_s: float
    innovation_mg: float
    sample_count: int
    valid: bool


class MassMotionEstimator:
    """以可变采样周期估计质量、流量、加速度与加加速度。"""

    def __init__(self, settings: MassMotionEstimatorSettings | None = None) -> None:
        self.settings = settings or MassMotionEstimatorSettings()
        self.reset()

    def reset(self) -> None:
        self._timestamp_s: float | None = None
        self._started_at_s: float | None = None
        self._mass_mg = 0.0
        self._rate_mg_s = 0.0
        self._acceleration_mg_s2 = 0.0
        self._jerk_mg_s3 = 0.0
        self._sample_count = 0

    @staticmethod
    def _bounded(value: float, lower: float, upper: float) -> float:
        return min(upper, max(lower, value))

    def update(self, timestamp_s: float, measured_mass_mg: float) -> MassMotionState:
        if measured_mass_mg < 0:
            raise ValueError("measured_mass_mg must be non-negative")
        if self._timestamp_s is not None and timestamp_s <= self._timestamp_s:
            raise ValueError("estimator timestamps must be strictly increasing")

        settings = self.settings
        if self._timestamp_s is None:
            self._timestamp_s = timestamp_s
            self._started_at_s = timestamp_s
            self._mass_mg = measured_mass_mg
            self._sample_count = 1
            return self._state(measured_mass_mg, innovation_mg=0.0)

        raw_dt = timestamp_s - self._timestamp_s
        dt = self._bounded(raw_dt, settings.min_dt_s, settings.max_dt_s)
        predicted_mass = (
            self._mass_mg
            + self._rate_mg_s * dt
            + 0.5 * self._acceleration_mg_s2 * dt * dt
        )
        predicted_rate = self._rate_mg_s + self._acceleration_mg_s2 * dt
        innovation = measured_mass_mg - predicted_mass

        previous_acceleration = self._acceleration_mg_s2
        self._mass_mg = max(0.0, predicted_mass + settings.alpha * innovation)
        self._rate_mg_s = self._bounded(
            predicted_rate + settings.beta * innovation / dt,
            0.0,
            settings.max_rate_mg_s,
        )
        self._acceleration_mg_s2 = self._bounded(
            previous_acceleration + 2.0 * settings.gamma * innovation / (dt * dt),
            -settings.max_acceleration_mg_s2,
            settings.max_acceleration_mg_s2,
        )
        raw_jerk = (self._acceleration_mg_s2 - previous_acceleration) / dt
        raw_jerk = self._bounded(
            raw_jerk,
            -settings.max_jerk_mg_s3,
            settings.max_jerk_mg_s3,
        )
        self._jerk_mg_s3 = (
            settings.jerk_alpha * raw_jerk
            + (1.0 - settings.jerk_alpha) * self._jerk_mg_s3
        )
        self._timestamp_s = timestamp_s
        self._sample_count += 1
        return self._state(measured_mass_mg, innovation_mg=innovation)

    def _state(self, measured_mass_mg: float, *, innovation_mg: float) -> MassMotionState:
        assert self._timestamp_s is not None
        assert self._started_at_s is not None
        horizon = self.settings.prediction_horizon_s
        predicted_mass = max(
            self._mass_mg,
            self._mass_mg
            + self._rate_mg_s * horizon
            + 0.5 * self._acceleration_mg_s2 * horizon * horizon,
        )
        predicted_rate = self._bounded(
            self._rate_mg_s + self._acceleration_mg_s2 * horizon,
            0.0,
            self.settings.max_rate_mg_s,
        )
        valid = (
            self._sample_count >= self.settings.min_valid_samples
            and self._timestamp_s - self._started_at_s >= self.settings.min_valid_elapsed_s
        )
        return MassMotionState(
            timestamp_s=self._timestamp_s,
            measured_mass_mg=measured_mass_mg,
            filtered_mass_mg=self._mass_mg,
            rate_mg_s=self._rate_mg_s,
            acceleration_mg_s2=self._acceleration_mg_s2,
            jerk_mg_s3=self._jerk_mg_s3,
            predicted_mass_mg=predicted_mass,
            predicted_rate_mg_s=predicted_rate,
            innovation_mg=innovation_mg,
            sample_count=self._sample_count,
            valid=valid,
        )


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PROFILE_DIR = PROJECT_ROOT / "data" / "control_profiles"


# ── 反馈种子（原 feedback_profiles.py）─────────────────────────────────────
#
# 这些值是运行时反馈控制器的软件种子。它们不是标定结果，
# 只应依据已完成的稳定运行来更新。

DISPENSE_PRESETS = {
    "fast-80hz-p200": {
        "frequency_hz": 80,
        "duty_permyriad": 2000,
        "window_position_units": 200,
    },
    "steady-65hz-p300": {
        "frequency_hz": 65,
        "duty_permyriad": 2000,
        "window_position_units": 300,
    },
    "60hz-p150": {
        "frequency_hz": 60,
        "duty_permyriad": 2000,
        "window_position_units": 150,
    },
    "60hz-p200": {
        "frequency_hz": 60,
        "duty_permyriad": 2000,
        "window_position_units": 200,
    },
    "slow-60hz-p250": {
        "frequency_hz": 60,
        "duty_permyriad": 1600,
        "window_position_units": 250,
    },
    "slow-40hz-p250": {
        "frequency_hz": 40,
        "duty_permyriad": 2000,
        "window_position_units": 250,
    },
    "slow-10hz-p100": {
        "frequency_hz": 10,
        "duty_permyriad": 1000,
        "window_position_units": 100,
    },
    "slow-15hz-p100": {
        "frequency_hz": 15,
        "duty_permyriad": 1000,
        "window_position_units": 100,
    },
    "steady-70hz-p100": {
        "frequency_hz": 70,
        "duty_permyriad": 2000,
        "window_position_units": 100,
    },
    "starch-70hz-p200": {
        "frequency_hz": 70,
        "duty_permyriad": 2000,
        "window_position_units": 100,
    },
    "slow-70hz-p300": {
        "frequency_hz": 70,
        "duty_permyriad": 2000,
        "window_position_units": 500,
    },
    "lubricant-1-80hz-p300": {
        "frequency_hz": 80,
        "duty_permyriad": 2000,
        "window_position_units": 300,
    },
    "lubricant-1-80hz-p500": {
        "frequency_hz": 80,
        "duty_permyriad": 2000,
        "window_position_units": 500,
    },
    "lubricant-1-70hz-p400": {
        "frequency_hz": 70,
        "duty_permyriad": 2000,
        "window_position_units": 400,
    },
    "80hz-22pct-p350-water-loss-agent-2": {
        "frequency_hz": 80,
        "duty_permyriad": 2200,
        "window_position_units": 250,
    },
    "80hz-22pct-p300-bentonite": {
        "frequency_hz": 80,
        "duty_permyriad": 2200,
        "window_position_units": 300,
    },
    "80hz-22pct-p350-plugging-agent-1": {
        "frequency_hz": 80,
        "duty_permyriad": 2200,
        "window_position_units": 350,
    },
    "80hz-22pct-p400-plugging-agent-1": {
        "frequency_hz": 80,
        "duty_permyriad": 2200,
        "window_position_units": 400,
    },
    "80hz-22pct-p400-plugging-agent-2": {
        "frequency_hz": 80,
        "duty_permyriad": 2200,
        "window_position_units": 250,
    },
}

DISPENSE_TARGET_PROFILES = {
    100: {
        "coarse_rate_mg_s": 15.0,
        "fine_rate_mg_s": 10.0,
        "precision_rate_mg_s": 8.0,
        "maximum_flow_rate_mg_s": 25.0,
        "precision_start_remaining_mg": 35.0,
        "tail_taper_start_remaining_mg": 45.0,
        "tail_taper_end_remaining_mg": 6.0,
    },
    300: {
        "coarse_rate_mg_s": 24.0,
        "fine_rate_mg_s": 14.0,
        "precision_rate_mg_s": 10.0,
        "maximum_flow_rate_mg_s": 50.0,
        "precision_start_remaining_mg": 55.0,
        "tail_taper_start_remaining_mg": 85.0,
        "tail_taper_end_remaining_mg": 10.0,
    },
    500: {
        "coarse_rate_mg_s": 30.0,
        "fine_rate_mg_s": 14.0,
        "precision_rate_mg_s": 9.0,
        "maximum_flow_rate_mg_s": 50.0,
        "precision_start_remaining_mg": 50.0,
        "tail_taper_start_remaining_mg": 100.0,
        "tail_taper_end_remaining_mg": 10.0,
    },
}

_ACCEPTANCE_REQUIREMENTS = load_product_requirements()["acceptance"]
DISPENSE_ALLOWED_UNDERWEIGHT_MG = abs(
    float(_ACCEPTANCE_REQUIREMENTS["minimum_offset_mg"])
)
DISPENSE_ALLOWED_OVERWEIGHT_MG = float(
    _ACCEPTANCE_REQUIREMENTS["maximum_offset_mg"]
)


POWDER_CLASSIFICATION_PARAMS = {
    "Low-flow": {
        "preset_id": "steady-70hz-p100",
        "maximum_flow_rate_mg_s": 80.0,
        "fixed_tail_mass_mg": 5.0,
        "force_stop_offset_mg": 10.0,
        "duty_step_per_update": 50,
        "emergency_duty_step_per_update": 200,
        "position_feedback_enabled": False,
    },
    "Medium-flow": {
        "preset_id": "60hz-p150",
        "maximum_flow_rate_mg_s": 120.0,
        "fixed_tail_mass_mg": 2.0,
        "force_stop_offset_mg": 10.0,
        "duty_step_per_update": 30,
        "emergency_duty_step_per_update": 80,
        "position_feedback_enabled": False,
        "profile_overrides": {
            100: {
                "coarse_rate_mg_s": 25.0,
                "fine_rate_mg_s": 15.0,
                "precision_start_remaining_mg": 40.0,
                "tail_taper_start_remaining_mg": 70.0,
                "tail_taper_end_remaining_mg": 15.0,
            },
        },
    },
    "High-flow": {
        "preset_id": "slow-10hz-p100",
        "maximum_flow_rate_mg_s": 200.0,
        "fixed_tail_mass_mg": 0.0,
        "force_stop_offset_mg": 15.0,
        # 高流量：激进制动，开启窗口反馈（占空比立即触底）
        "duty_step_per_update": 100,
        "emergency_duty_step_per_update": 400,
        "position_feedback_enabled": True,
        "profile_overrides": {
            100: {
                "precision_start_remaining_mg": 40.0,
                "tail_taper_start_remaining_mg": 55.0,
                "tail_taper_end_remaining_mg": 12.0,
            },
            300: {
                "precision_start_remaining_mg": 80.0,
                "tail_taper_start_remaining_mg": 120.0,
                "tail_taper_end_remaining_mg": 15.0,
            },
            500: {
                "precision_start_remaining_mg": 80.0,
                "tail_taper_start_remaining_mg": 150.0,
                "tail_taper_end_remaining_mg": 20.0,
            },
        },
    },
}

# 当 unstable 标签生效时，在分类参数之上额外应用的覆盖项
UNSTABLE_TAG_OVERRIDES = {
    "settle_confirmations_delta": +1,
    "dead_zone_multiplier": 1.5,
}

# 当 sticky 标签生效时，将预设提升一档
STICKY_PRESET_BUMP = {
    "slow-10hz-p100": "slow-40hz-p250",
    "slow-40hz-p250": "slow-60hz-p250",
    "slow-60hz-p250": "60hz-p150",
    "60hz-p150": "60hz-p200",
    "60hz-p200": "steady-65hz-p300",
    "steady-65hz-p300": "steady-70hz-p100",
    "steady-70hz-p100": "slow-70hz-p300",
    "slow-70hz-p300": "fast-80hz-p200",
    "fast-80hz-p200": "fast-80hz-p200",  # 已处于最高档，不再提升
}


def powder_classification_params(category: str, tags: list[str] | None = None):
    """返回给定粉体分类对应的参数覆盖项。

    Args:
        category: "Low-flow"、"Medium-flow"、"High-flow" 之一。
        tags: 可选的标签列表，如 ["stable"]、["unstable"]、["sticky"]。

    Returns:
        包含 preset_id、maximum_flow_rate_mg_s、fixed_tail_mass_mg 的 dict，
        还可能包含 settle_confirmations_delta、dead_zone_multiplier。
    """
    if category not in POWDER_CLASSIFICATION_PARAMS:
        raise ValueError(f"unknown powder category: {category}; choose Low-flow, Medium-flow, or High-flow")
    tags = tags or []
    params = dict(POWDER_CLASSIFICATION_PARAMS[category])

    if "unstable" in tags:
        params.update(UNSTABLE_TAG_OVERRIDES)
    if "sticky" in tags:
        params["preset_id"] = STICKY_PRESET_BUMP.get(params["preset_id"], params["preset_id"])

    return params


def dispense_plan(target_mg):
    """返回受支持的整数目标质量及其反馈档位。"""
    try:
        target = int(target_mg)
    except (TypeError, ValueError) as error:
        raise ValueError("target_mg must be an integer") from error
    if target not in DISPENSE_TARGET_PROFILES:
        raise ValueError(f"unsupported dispense target: {target}; choose 100, 300, or 500 mg")
    return target, DISPENSE_TARGET_PROFILES[target]


# ── 任意质量的插值（反馈种子）───────────────────────────────────────

# DISPENSE_TARGET_PROFILES 条目中可插值的标量字段。
_INTERPOLATABLE_KEYS = (
    "coarse_rate_mg_s",
    "fine_rate_mg_s",
    "precision_rate_mg_s",
    "precision_start_remaining_mg",
    "tail_taper_start_remaining_mg",
    "tail_taper_end_remaining_mg",
)

# 各字段的截断边界（最小值, 最大值）。
_INTERPOLATION_CLAMPS = {
    "coarse_rate_mg_s": (5.0, 35.0),
    "fine_rate_mg_s": (8.0, 30.0),
    "precision_rate_mg_s": (5.0, 20.0),
    "precision_start_remaining_mg": (10.0, None),
    "tail_taper_start_remaining_mg": (15.0, None),
    "tail_taper_end_remaining_mg": (3.0, None),
}

# 来自 DISPENSE_TARGET_PROFILES 的已排序锚点质量。
_ANCHOR_MASSES = sorted(DISPENSE_TARGET_PROFILES.keys())


def _linear_interpolate(target, anchors, values):
    """在 *anchors* 之间对 *target* 处的 *values* 进行线性插值。

    若 *target* 低于最小锚点，则使用第一对锚点的斜率向下外推；
    若高于最大锚点，则使用最后一对锚点的斜率向上外推。
    """
    if target in anchors:
        idx = anchors.index(target)
        return values[idx]

    if target < anchors[0]:
        # 向下外推：使用 anchors[0] 与 anchors[1] 之间的斜率
        slope = (values[1] - values[0]) / (anchors[1] - anchors[0])
        return values[0] - slope * (anchors[0] - target)

    if target > anchors[-1]:
        # 向上外推：使用 anchors[-2] 与 anchors[-1] 之间的斜率
        slope = (values[-1] - values[-2]) / (anchors[-1] - anchors[-2])
        return values[-1] + slope * (target - anchors[-1])

    # 区间内部插值
    for i in range(len(anchors) - 1):
        if anchors[i] <= target <= anchors[i + 1]:
            frac = (target - anchors[i]) / (anchors[i + 1] - anchors[i])
            return values[i] + (values[i + 1] - values[i]) * frac

    raise RuntimeError(f"unexpected interpolation state for target={target}")  # 不可达


def interpolate_profile(target_mg):
    """返回 *target_mg*（100–1000）对应的反馈档位 dict。

    ``DISPENSE_TARGET_PROFILES`` 中随目标变化的字段在
    100 / 300 / 500 mg 锚点之间线性插值。
    ``maximum_flow_rate_mg_s`` 不做插值——调用方必须
    从粉体分类参数中提供该值。
    """
    target = validate_target_mass(target_mg)

    profile = {}
    for key in _INTERPOLATABLE_KEYS:
        anchor_values = [DISPENSE_TARGET_PROFILES[m][key] for m in _ANCHOR_MASSES]
        val = _linear_interpolate(target, _ANCHOR_MASSES, anchor_values)
        lo, hi = _INTERPOLATION_CLAMPS.get(key, (None, None))
        if lo is not None:
            val = max(lo, val)
        if hi is not None:
            val = min(hi, val)
        profile[key] = float(val)

    # 保护：tail_taper_end 必须严格小于 tail_taper_start。
    tts = profile["tail_taper_start_remaining_mg"]
    tte = profile["tail_taper_end_remaining_mg"]
    if tte >= tts:
        profile["tail_taper_end_remaining_mg"] = tts * 0.3

    # maximum_flow_rate_mg_s 不做插值。  在精确命中锚点时保留该锚点的
    # 取值，使 interpolate_profile(anchor) 能完整复现锚点档位；
    # 否则使用一个安全的默认值。
    # 调用方（start_dispense）会根据粉体会话覆盖该值。
    if target in DISPENSE_TARGET_PROFILES:
        profile["maximum_flow_rate_mg_s"] = float(
            DISPENSE_TARGET_PROFILES[target]["maximum_flow_rate_mg_s"]
        )
    else:
        profile["maximum_flow_rate_mg_s"] = 50.0

    return profile


def validate_dispense_target(target_mg):
    """若 *target_mg* 不是有效的出料目标则抛出 ValueError。"""
    validate_target_mass(target_mg)


# ── 运行时控制档位（原 control_profiles.py）─────────────────────────────────


INTERPOLATABLE_TARGET_KEYS = (
    "coarse_rate_mg_s",
    "fine_rate_mg_s",
    "precision_rate_mg_s",
    "precision_start_remaining_mg",
    "tail_taper_start_remaining_mg",
    "tail_taper_end_remaining_mg",
)

STAGE_RATE_KEYS = (
    "coarse_rate_mg_s",
    "fine_rate_mg_s",
    "precision_rate_mg_s",
    "settle_rate_mg_s",
)

EXACT_TARGET_PROFILE_OVERRIDE_KEYS = (
    "precision_start_remaining_mg",
)

EXACT_TARGET_CONTROLLER_OVERRIDE_KEYS = (
    "fixed_tail_mass_mg",
    "force_stop_offset_mg",
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
)


def normalize_target_keys(raw_profiles: dict[Any, Any]) -> dict[int, dict[str, Any]]:
    """规范化 JSON 字符串形式的质量键，不要求档位内容完整。"""
    if not isinstance(raw_profiles, dict):
        raise ValueError("target profiles must be an object")
    normalized: dict[int, dict[str, Any]] = {}
    for raw_mass, raw_values in raw_profiles.items():
        try:
            mass = int(raw_mass)
        except (TypeError, ValueError) as error:
            raise ValueError(f"invalid target profile key: {raw_mass!r}") from error
        if str(mass) != str(raw_mass) and raw_mass != mass:
            raise ValueError(f"target profile key must be an integer mass: {raw_mass!r}")
        if mass <= 0 or not isinstance(raw_values, dict):
            raise ValueError(f"invalid target profile at {raw_mass!r}")
        normalized[mass] = dict(raw_values)
    return dict(sorted(normalized.items()))


def normalize_target_profiles(raw_profiles: dict[Any, Any]) -> dict[int, dict[str, float]]:
    """规范化目标键并校验完整的插值锚点。"""
    if not isinstance(raw_profiles, dict) or len(raw_profiles) < 2:
        raise ValueError("target_profiles must contain at least two mass anchors")
    normalized: dict[int, dict[str, float]] = {}
    for mass, raw_values in normalize_target_keys(raw_profiles).items():
        values: dict[str, float] = {}
        for key in INTERPOLATABLE_TARGET_KEYS:
            value = raw_values.get(key)
            if not isinstance(value, (int, float)):
                raise ValueError(f"target {mass} is missing numeric {key}")
            values[key] = float(value)
        normalized[mass] = values
    return dict(sorted(normalized.items()))


def normalize_target_rate_bands(raw_bands: Any) -> list[dict[str, Any]]:
    """校验可选的、按整数目标区间划分的分段阶段速率策略。"""
    if raw_bands in (None, []):
        return []
    if not isinstance(raw_bands, list):
        raise ValueError("target_rate_bands must be an array")
    normalized: list[dict[str, Any]] = []
    for index, raw_band in enumerate(raw_bands):
        if not isinstance(raw_band, dict):
            raise ValueError(f"target rate band {index} must be an object")
        minimum = validate_target_mass(raw_band.get("minimum_target_mg"))
        maximum = validate_target_mass(raw_band.get("maximum_target_mg"))
        if minimum > maximum:
            raise ValueError(f"target rate band {index} has an inverted range")
        band: dict[str, Any] = {
            "minimum_target_mg": minimum,
            "maximum_target_mg": maximum,
        }
        for key in STAGE_RATE_KEYS:
            value = raw_band.get(key)
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise ValueError(f"target rate band {index} is missing numeric {key}")
            if key == "settle_rate_mg_s":
                if float(value) != 0.0:
                    raise ValueError("settle_rate_mg_s must be zero")
            elif float(value) <= 0.0:
                raise ValueError(f"target rate band {index} has non-positive {key}")
            band[key] = float(value)
        normalized.append(band)
    normalized.sort(key=lambda band: band["minimum_target_mg"])
    for previous, current in zip(normalized, normalized[1:]):
        if current["minimum_target_mg"] <= previous["maximum_target_mg"]:
            raise ValueError("target rate bands must not overlap")
    return normalized


def normalize_exact_target_overrides(raw_overrides: Any) -> dict[int, dict[str, Any]]:
    """校验仅作用于某个精确目标的非插值覆盖项。"""
    if raw_overrides in (None, {}):
        return {}
    allowed_keys = set(EXACT_TARGET_PROFILE_OVERRIDE_KEYS) | set(
        EXACT_TARGET_CONTROLLER_OVERRIDE_KEYS
    )
    normalized: dict[int, dict[str, Any]] = {}
    for mass, raw_values in normalize_target_keys(raw_overrides).items():
        validate_target_mass(mass)
        unknown_keys = set(raw_values) - allowed_keys
        if unknown_keys:
            raise ValueError(
                f"target {mass} has unsupported exact overrides: "
                + ", ".join(sorted(unknown_keys))
            )
        values: dict[str, Any] = {}
        for key, value in raw_values.items():
            if key == "tail_pulse_enabled":
                if not isinstance(value, bool):
                    raise ValueError(f"target {mass} has non-boolean exact override {key}")
                values[key] = value
                continue
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise ValueError(f"target {mass} has non-numeric exact override {key}")
            if float(value) <= 0:
                raise ValueError(f"target {mass} has non-positive exact override {key}")
            values[key] = float(value)
        normalized[mass] = values
    return dict(sorted(normalized.items()))


def resolve_target_rate_band(raw_bands: Any, target_mg: Any) -> dict[str, Any] | None:
    """返回包含该目标的唯一已配置阶段速率区间。"""
    target = validate_target_mass(target_mg)
    matches = [
        band
        for band in normalize_target_rate_bands(raw_bands)
        if band["minimum_target_mg"] <= target <= band["maximum_target_mg"]
    ]
    if not matches:
        return None
    if len(matches) != 1:
        raise ValueError(f"multiple target rate bands match {target} mg")
    return dict(matches[0])


def resolve_default_target_rate_band(target_mg: Any) -> dict[str, Any]:
    """解析产品级的速率策略，在粉体覆盖之前使用。"""
    bands = load_product_requirements().get("default_target_rate_bands", [])
    resolved = resolve_target_rate_band(bands, target_mg)
    if resolved is None:
        raise ValueError(f"no default target rate band covers {target_mg} mg")
    return resolved


def _piecewise_linear(target: int, anchors: list[int], values: list[float]) -> float:
    if target in anchors:
        return values[anchors.index(target)]
    if target < anchors[0]:
        left, right = 0, 1
    elif target > anchors[-1]:
        left, right = len(anchors) - 2, len(anchors) - 1
    else:
        for left in range(len(anchors) - 1):
            right = left + 1
            if anchors[left] < target < anchors[right]:
                break
    fraction = (target - anchors[left]) / (anchors[right] - anchors[left])
    return values[left] + fraction * (values[right] - values[left])


def resolve_target_profile(raw_profiles: dict[Any, Any], target_mg: Any) -> dict[str, Any]:
    """解析一个目标质量，并报告其命中方式（精确/插值/外推）。"""
    target = validate_target_mass(target_mg)
    profiles = normalize_target_profiles(raw_profiles)
    anchors = sorted(profiles)
    if target < anchors[0] or target > anchors[-1]:
        resolution = "extrapolated"
    elif target in profiles:
        resolution = "exact"
    else:
        resolution = "interpolated"
    values = {
        key: float(_piecewise_linear(target, anchors, [profiles[m][key] for m in anchors]))
        for key in INTERPOLATABLE_TARGET_KEYS
    }
    if values["tail_taper_end_remaining_mg"] >= values["tail_taper_start_remaining_mg"]:
        values["tail_taper_end_remaining_mg"] = (
            values["tail_taper_start_remaining_mg"] * 0.3
        )
    return {
        "target_mg": target,
        "resolution": resolution,
        "anchor_masses_mg": anchors,
        "profile": values,
    }


def resolve_partial_target_overrides(
    raw_profiles: dict[Any, Any], target_mg: Any
) -> dict[str, float]:
    """对旧版部分覆盖项中出现的每个数值字段进行插值。"""
    target = validate_target_mass(target_mg)
    profiles = normalize_target_keys(raw_profiles)
    result: dict[str, float] = {}
    keys = {
        key
        for values in profiles.values()
        for key, value in values.items()
        if isinstance(value, (int, float)) and not isinstance(value, bool)
    }
    for key in keys:
        anchors = [mass for mass, values in profiles.items() if key in values]
        values = [float(profiles[mass][key]) for mass in anchors]
        if len(anchors) == 1:
            result[key] = values[0]
        elif anchors:
            result[key] = float(_piecewise_linear(target, anchors, values))
    return result


def validate_control_profile(profile: dict[str, Any]) -> dict[str, Any]:
    required = ("powder_id", "powder_name", "status", "recommended_preset_id")
    for key in required:
        if not isinstance(profile.get(key), str) or not profile[key].strip():
            raise ValueError(f"control profile is missing {key}")
    controller = profile.get("recommended_controller")
    if not isinstance(controller, dict):
        raise ValueError("control profile is missing recommended_controller")
    normalize_target_profiles(profile.get("target_profiles", {}))
    normalize_target_rate_bands(profile.get("target_rate_bands", []))
    normalize_exact_target_overrides(profile.get("exact_target_overrides", {}))
    return profile


def load_control_profile(
    powder_id: str,
    profile_dir: Path | str = DEFAULT_PROFILE_DIR,
) -> dict[str, Any] | None:
    if not powder_id or Path(powder_id).name != powder_id:
        return None
    path = Path(profile_dir) / f"{powder_id}.json"
    if not path.exists():
        return None
    return validate_control_profile(json.loads(path.read_text(encoding="utf-8")))


def list_control_profiles(
    profile_dir: Path | str = DEFAULT_PROFILE_DIR,
) -> list[dict[str, Any]]:
    directory = Path(profile_dir)
    if not directory.exists():
        return []
    summaries = []
    for path in sorted(directory.glob("*.json")):
        try:
            profile = validate_control_profile(json.loads(path.read_text(encoding="utf-8")))
        except (OSError, ValueError, json.JSONDecodeError):
            continue
        summaries.append({
            "powder_id": profile["powder_id"],
            "powder_name": profile["powder_name"],
            "status": profile["status"],
            "recommended_preset_id": profile["recommended_preset_id"],
            "anchor_masses_mg": sorted(normalize_target_profiles(profile["target_profiles"])),
            "generation_warnings": profile.get("generation_warnings", []),
        })
    return summaries


def resolve_control_profile(profile: dict[str, Any], target_mg: Any) -> dict[str, Any]:
    validate_control_profile(profile)
    resolved = resolve_target_profile(profile["target_profiles"], target_mg)
    rate_band = resolve_target_rate_band(profile.get("target_rate_bands", []), target_mg)
    rate_band_source = "powder_profile"
    if rate_band is None:
        rate_band = resolve_default_target_rate_band(target_mg)
        rate_band_source = "product_default"
    target_profile = dict(resolved["profile"])
    if rate_band is not None:
        for key in STAGE_RATE_KEYS:
            if key != "settle_rate_mg_s":
                target_profile[key] = rate_band[key]
    recommended_controller = dict(profile["recommended_controller"])
    exact_target_overrides = normalize_exact_target_overrides(
        profile.get("exact_target_overrides", {})
    ).get(resolved["target_mg"], {})
    for key, value in exact_target_overrides.items():
        if key in EXACT_TARGET_PROFILE_OVERRIDE_KEYS:
            target_profile[key] = value
        elif key in EXACT_TARGET_CONTROLLER_OVERRIDE_KEYS:
            recommended_controller[key] = value
    return {
        "powder_id": profile["powder_id"],
        "powder_name": profile["powder_name"],
        "status": profile["status"],
        "recommended_preset_id": profile["recommended_preset_id"],
        "recommended_controller": recommended_controller,
        "target_resolution": {
            key: value for key, value in resolved.items() if key != "profile"
        },
        "target_profile": target_profile,
        "exact_target_overrides": dict(exact_target_overrides),
        "target_rate_band": rate_band,
        "target_rate_band_source": rate_band_source,
        "generation_warnings": list(profile.get("generation_warnings", [])),
    }


# ── 草稿生成（原 method_generator.py）────────────────────────────────────────


def build_candidate_control_profile(workspace: dict[str, Any]) -> dict[str, Any]:
    """构建草稿；实测数据缺失时生成告警，绝不静默使用默认值。"""
    powder_id = str(workspace.get("powder_id", "")).strip()
    powder_name = str(workspace.get("powder_name", "")).strip()
    if not powder_id or not powder_name:
        raise ValueError("workspace must contain powder_id and powder_name")
    profiles = normalize_target_profiles(workspace.get("recommended_profiles", {}))
    target_rate_bands = normalize_target_rate_bands(
        workspace.get("target_rate_bands", [])
    )
    controller = workspace.get("recommended_controller")
    if not isinstance(controller, dict):
        raise ValueError("workspace must contain recommended_controller")

    warnings: list[str] = []
    scan_grid = workspace.get("scan_grid", [])
    pending_count = sum(1 for point in scan_grid if point.get("status") == "pending")
    if pending_count:
        warnings.append(f"{pending_count} scan points are still pending")
    for region_name in ("safe_coarse_region", "safe_fine_region"):
        region = workspace.get(region_name, {})
        if region.get("confidence") in (None, "low"):
            warnings.append(f"{region_name} has low or unspecified confidence")
    if not workspace.get("path_dependency", {}).get("notes"):
        warnings.append("transition/path-dependency evidence is missing")
    tail = workspace.get("tail_behavior", {})
    if not isinstance(tail.get("mean_tail_mg"), (int, float)):
        warnings.append("measured stop-tail evidence is missing")

    return {
        "schema_version": 1,
        "powder_id": powder_id,
        "powder_name": powder_name,
        "status": "draft",
        "recommended_preset_id": workspace.get("recommended_preset_id", ""),
        "recommended_controller": dict(controller),
        "target_profiles": {str(mass): values for mass, values in profiles.items()},
        "target_rate_bands": target_rate_bands,
        "generation_warnings": warnings,
        "source_workspace_updated_at": workspace.get("updated_at"),
    }


class FeedbackStage(str, Enum):
    COARSE = "coarse"
    FINE = "fine"
    SETTLE = "settle"


@dataclass(frozen=True)
class FeedbackInitialParameters:
    """针对本次运行的种子，通常由 L0/ILC 或操作员选定。"""

    frequency_hz: int
    duty_permyriad: int
    window_position_units: int


@dataclass(frozen=True)
class FeedbackObservation:
    """来自天平与运动估计器的一次实时观测。"""

    mass_mg: float
    predicted_mass_mg: float
    rate_mg_s: float
    predicted_rate_mg_s: float
    acceleration_mg_s2: float = 0.0
    jerk_mg_s3: float = 0.0
    valid: bool = True


@dataclass(frozen=True)
class FeedbackControllerSettings:
    """保守的默认值；应根据记录的运行数据调优，而不是在硬件代码中调整。"""

    low_target_limit_mg: float = 100.0
    high_target_limit_mg: float = 200.0
    low_coarse_fraction: float = 0.70
    default_coarse_fraction: float = 0.75
    high_coarse_fraction: float = 0.80
    coarse_rate_mg_s: float = 10.0
    fine_rate_mg_s: float = 5.0
    min_frequency_hz: int = 10
    max_frequency_hz: int = 80
    min_duty_permyriad: int = 1000
    max_duty_permyriad: int = 5000
    min_window_position_units: int = 100
    max_window_position_units: int = 2000
    duty_deadband_mg_s: float = 0.75
    position_deadband_mg_s: float = 2.0
    duty_gain_per_mg_s: float = 100.0
    duty_integral_gain_per_mg: float = 2.0
    duty_step_per_update: int = 50
    position_step_units: int = 25
    position_update_interval_s: float = 2.0
    duty_update_interval_s: float = 0.5
    position_persistence_samples: int = 4
    position_increases_flow_with_units: bool = True
    acceleration_guard_mg_s2: float = 2.0
    jerk_guard_mg_s3: float = 12.0
    overspeed_multiple: float = 1.5
    maximum_flow_rate_mg_s: float = 30.0
    emergency_duty_step_per_update: int = 200
    emergency_duty_update_interval_s: float = 0.25
    settle_confirmations: int = 2
    allowed_underweight_mg: float = 0.0
    allowed_overweight_mg: float = 0.0
    precision_start_remaining_mg: float = 50.0
    precision_rate_mg_s: float = 8.0
    stop_prediction_horizon_s: float = 1.0
    fixed_tail_mass_mg: float = 2.0
    stop_target_offset_mg: float = 0.0
    predictive_stop_enabled: bool = True
    fast_predictive_stop_enabled: bool = False
    fast_predictive_stop_rate_mg_s: float = 20.0
    fast_predictive_stop_confirmations: int = 1
    projected_safety_stop_enabled: bool = False
    projected_safety_stop_offset_mg: float = 5.0
    position_feedback_enabled: bool = True
    tail_taper_enabled: bool = False
    tail_taper_start_remaining_mg: float = 100.0
    tail_taper_end_remaining_mg: float = 10.0
    tail_duty_reduction_permyriad: int = 200
    tail_window_reduction_units: int = 50
    # ── 强制停止 / 继续边界 ──
    force_stop_max_mass_mg: float = float("inf")
    force_continue_min_mass_mg: float = 0.0
    stall_rate_threshold_mg_s: float = 1.0
    stall_duration_s: float = 3.0
    # ── 尾段递减期堵料逃逸 ──
    taper_stall_timeout_s: float = 3.0
    # ── 轻质粉体流量保持 / 无效占空比回退 ──
    flow_hold_enabled: bool = False
    flow_hold_window_s: float = 4.0
    flow_hold_min_gain_mg: float = 5.0
    flow_response_check_s: float = 4.0
    flow_response_min_improvement_mg_s: float = 1.0
    # ── 针对在固定开度处架桥粉体的临时窗口恢复 ──
    stall_recovery_enabled: bool = False
    stall_recovery_observation_window_s: float = 3.0
    stall_recovery_min_gain_mg: float = 2.0
    stall_recovery_rate_threshold_mg_s: float = 1.0
    stall_recovery_resume_rate_mg_s: float = 2.0
    stall_recovery_retry_s: float = 3.0
    stall_recovery_window_step_units: int = 25
    stall_recovery_max_window_position_units: int = 400
    stall_recovery_max_attempts: int = 2
    stall_recovery_disable_remaining_mg: float = 50.0

    def __post_init__(self) -> None:
        if not 0 < self.low_target_limit_mg < self.high_target_limit_mg:
            raise ValueError("target mass limits must be positive and increasing")
        if not 0 < self.low_coarse_fraction < self.default_coarse_fraction < self.high_coarse_fraction < 1:
            raise ValueError("coarse fractions must be increasing and below one")
        if min(self.coarse_rate_mg_s, self.fine_rate_mg_s, self.precision_rate_mg_s) <= 0:
            raise ValueError("rate targets must be positive")
        if not 0 < self.min_frequency_hz <= self.max_frequency_hz:
            raise ValueError("frequency limits are invalid")
        if not 1000 <= self.min_duty_permyriad <= self.max_duty_permyriad:
            raise ValueError("duty limits are invalid")
        if not 0 <= self.min_window_position_units < self.max_window_position_units:
            raise ValueError("window limits are invalid")
        if min(
            self.duty_deadband_mg_s,
            self.position_deadband_mg_s,
            self.duty_gain_per_mg_s,
            self.duty_integral_gain_per_mg,
            self.position_update_interval_s,
            self.duty_update_interval_s,
        ) <= 0:
            raise ValueError("controller gains, deadbands, and intervals must be positive")
        if self.duty_step_per_update < 1 or self.position_step_units < 1:
            raise ValueError("controller steps must be positive")
        if self.position_persistence_samples < 1 or self.settle_confirmations < 1:
            raise ValueError("controller sample counts must be positive")
        if self.overspeed_multiple <= 1:
            raise ValueError("overspeed_multiple must be greater than one")
        if self.maximum_flow_rate_mg_s <= 0:
            raise ValueError("maximum_flow_rate_mg_s must be positive")
        if self.emergency_duty_step_per_update < self.duty_step_per_update:
            raise ValueError("emergency duty step must be at least the normal duty step")
        if not 0 < self.emergency_duty_update_interval_s <= self.duty_update_interval_s:
            raise ValueError("emergency duty interval must be positive and no slower than the normal interval")
        if min(self.allowed_underweight_mg, self.allowed_overweight_mg) < 0:
            raise ValueError("acceptance tolerances must be non-negative")
        # 精密段预留量是一个质量值，而不是目标规模的比例。
        # 大剂量（1–10 g）运行刻意将其最后 500 mg 预留给同一个
        # 闭环精密阶段，因此该值可以超过旧版 200 mg
        # 高目标阈值。
        if self.precision_start_remaining_mg <= 0:
            raise ValueError("precision_start_remaining_mg is invalid")
        if self.stop_prediction_horizon_s <= 0 or self.fixed_tail_mass_mg < 0:
            raise ValueError("stop prediction settings are invalid")
        if not 0 <= self.stop_target_offset_mg <= self.allowed_underweight_mg:
            raise ValueError("stop target offset must fit inside the allowed underweight range")
        if self.fast_predictive_stop_rate_mg_s <= 0:
            raise ValueError("fast predictive stop rate must be positive")
        if not 1 <= self.fast_predictive_stop_confirmations <= self.settle_confirmations:
            raise ValueError(
                "fast predictive stop confirmations must be between 1 and settle_confirmations"
            )
        if self.projected_safety_stop_offset_mg < 0 or (
            self.projected_safety_stop_enabled
            and self.projected_safety_stop_offset_mg > self.allowed_overweight_mg
        ):
            raise ValueError(
                "projected safety stop offset must fit inside the allowed overweight range"
            )
        if not 0 <= self.tail_taper_end_remaining_mg < self.tail_taper_start_remaining_mg:
            raise ValueError("tail taper remaining-mass boundaries are invalid")
        if min(self.tail_duty_reduction_permyriad, self.tail_window_reduction_units) < 0:
            raise ValueError("tail taper reductions must be non-negative")
        if self.force_stop_max_mass_mg <= 0:
            raise ValueError("force_stop_max_mass_mg must be positive")
        if self.force_continue_min_mass_mg < 0:
            raise ValueError("force_continue_min_mass_mg must be non-negative")
        if self.force_continue_min_mass_mg >= self.force_stop_max_mass_mg:
            raise ValueError(
                "force_continue_min_mass_mg must be less than force_stop_max_mass_mg"
            )
        if self.stall_rate_threshold_mg_s <= 0 or self.stall_duration_s <= 0:
            raise ValueError("stall parameters must be positive")
        if self.taper_stall_timeout_s <= 0:
            raise ValueError("taper_stall_timeout_s must be positive")
        if min(
            self.flow_hold_window_s,
            self.flow_hold_min_gain_mg,
            self.flow_response_check_s,
        ) <= 0:
            raise ValueError("flow-hold windows and minimum gain must be positive")
        if self.flow_response_min_improvement_mg_s < 0:
            raise ValueError("flow response improvement must not be negative")
        if min(
            self.stall_recovery_observation_window_s,
            self.stall_recovery_rate_threshold_mg_s,
            self.stall_recovery_resume_rate_mg_s,
            self.stall_recovery_retry_s,
        ) <= 0:
            raise ValueError("stall recovery time and rate settings must be positive")
        if self.stall_recovery_min_gain_mg < 0:
            raise ValueError("stall recovery minimum gain must not be negative")
        if self.stall_recovery_resume_rate_mg_s <= self.stall_recovery_rate_threshold_mg_s:
            raise ValueError("stall recovery resume rate must exceed its stall threshold")
        if self.stall_recovery_window_step_units < 1 or self.stall_recovery_max_attempts < 1:
            raise ValueError("stall recovery window step and attempt count must be positive")
        if not (
            self.min_window_position_units
            <= self.stall_recovery_max_window_position_units
            <= self.max_window_position_units
        ):
            raise ValueError("stall recovery maximum window is outside controller limits")
        if self.stall_recovery_disable_remaining_mg < 0:
            raise ValueError("stall recovery tail exclusion must not be negative")


@dataclass(frozen=True)
class FeedbackDecision:
    stage: FeedbackStage
    coarse_threshold_mg: float
    target_rate_mg_s: float
    frequency_hz: int
    duty_before_permyriad: int
    duty_after_permyriad: int
    window_before_units: int
    window_after_units: int
    rate_error_mg_s: float
    control_rate_mg_s: float
    estimated_tail_mg: float
    projected_stop_mass_mg: float
    stop_requested: bool
    duty_changed: bool
    window_changed: bool
    reason: str
    predictive_stop_candidate: bool = False
    stop_confirmation_count: int = 0
    stop_confirmation_required: int = 0
    stall_recovery_active: bool = False
    stall_recovery_attempt: int = 0


class ContinuousFeedbackController:
    """双环控制器：快速占空比微调与慢速窗口位置微调。

    :class:`FeedbackInitialParameters` 中的频率有意保持不可变。
    物理写入由调用方负责，若不满足设备联锁或状态机规则，
    调用方可以拒绝某条决策。
    """

    def __init__(
        self,
        *,
        target_mass_mg: float,
        initial: FeedbackInitialParameters,
        settings: FeedbackControllerSettings | None = None,
    ) -> None:
        if target_mass_mg <= 0:
            raise ValueError("target_mass_mg must be positive")
        self.target_mass_mg = float(target_mass_mg)
        self.settings = settings or FeedbackControllerSettings()
        self._validate_initial(initial)
        if (
            self.settings.stall_recovery_enabled
            and initial.window_position_units
            >= self.settings.stall_recovery_max_window_position_units
        ):
            raise ValueError(
                "stall recovery maximum window must exceed the configured window"
            )
        self.frequency_hz = initial.frequency_hz
        self.duty_permyriad = initial.duty_permyriad
        self.window_position_units = initial.window_position_units
        self._initial_duty_permyriad = initial.duty_permyriad
        self._initial_window_position_units = initial.window_position_units
        self._integral_error_mg = 0.0
        self._last_timestamp_s: float | None = None
        self._last_duty_update_s: float | None = None
        self._last_position_update_s: float | None = None
        self._under_rate_samples = 0
        self._over_rate_samples = 0
        self._settle_samples = 0
        self._last_predictive_stop_candidate = False
        self._stall_start_s: float | None = None
        self._taper_stall_start_s: float | None = None
        self._flow_history: list[tuple[float, float, float]] = []
        self._flow_duty_probe: dict[str, float] | None = None
        self._flow_hold_started_s: float | None = None
        self._stall_recovery_history: list[tuple[float, float]] = []
        self._stall_recovery_active = False
        self._stall_recovery_attempts = 0
        self._stall_recovery_attempt_started_s: float | None = None
        self._stall_recovery_waiting_for_observation = False
        self._stall_recovery_start_mass_mg = 0.0

    def coarse_fraction(self) -> float:
        if self.target_mass_mg <= self.settings.low_target_limit_mg:
            return self.settings.low_coarse_fraction
        if self.target_mass_mg > self.settings.high_target_limit_mg:
            return self.settings.high_coarse_fraction
        return self.settings.default_coarse_fraction

    def coarse_threshold_mg(self) -> float:
        return self.target_mass_mg * self.coarse_fraction()

    def reset(self, initial: FeedbackInitialParameters | None = None) -> None:
        if initial is not None:
            self._validate_initial(initial)
            self.frequency_hz = initial.frequency_hz
            self.duty_permyriad = initial.duty_permyriad
            self.window_position_units = initial.window_position_units
            self._initial_duty_permyriad = initial.duty_permyriad
            self._initial_window_position_units = initial.window_position_units
        self._integral_error_mg = 0.0
        self._last_timestamp_s = None
        self._last_duty_update_s = None
        self._last_position_update_s = None
        self._under_rate_samples = 0
        self._over_rate_samples = 0
        self._settle_samples = 0
        self._last_predictive_stop_candidate = False
        self._stall_start_s = None
        self._taper_stall_start_s = None
        self._flow_history = []
        self._flow_duty_probe = None
        self._flow_hold_started_s = None
        self._stall_recovery_history = []
        self._stall_recovery_active = False
        self._stall_recovery_attempts = 0
        self._stall_recovery_attempt_started_s = None
        self._stall_recovery_waiting_for_observation = False
        self._stall_recovery_start_mass_mg = 0.0

    def step(
        self,
        observation: FeedbackObservation,
        timestamp_s: float | None = None,
        *,
        bypass_rate_control: bool = False,
    ) -> FeedbackDecision:
        self._validate_observation(observation)
        settings = self.settings
        now = monotonic() if timestamp_s is None else float(timestamp_s)
        if self._last_timestamp_s is not None and now <= self._last_timestamp_s:
            raise ValueError("controller timestamps must be strictly increasing")
        dt = 0.0 if self._last_timestamp_s is None else min(2.0, now - self._last_timestamp_s)
        self._last_timestamp_s = now

        threshold = self.coarse_threshold_mg()
        projected_mass = max(observation.mass_mg, observation.predicted_mass_mg)
        control_rate = max(observation.rate_mg_s, observation.predicted_rate_mg_s)
        flow_detected = False
        flow_average_rate = control_rate
        if settings.flow_hold_enabled and observation.valid:
            if self._flow_hold_started_s is None:
                self._flow_hold_started_s = now
            self._flow_history.append((now, observation.mass_mg, control_rate))
            history_horizon = max(
                settings.flow_hold_window_s,
                settings.flow_response_check_s,
            )
            cutoff = now - history_horizon
            self._flow_history = [
                point for point in self._flow_history if point[0] >= cutoff
            ]
            flow_window_cutoff = now - settings.flow_hold_window_s
            flow_window = [
                point for point in self._flow_history if point[0] >= flow_window_cutoff
            ]
            if flow_window:
                flow_gain_mg = observation.mass_mg - flow_window[0][1]
                flow_average_rate = sum(point[2] for point in flow_window) / len(
                    flow_window
                )
                flow_detected = flow_gain_mg >= settings.flow_hold_min_gain_mg
        estimated_tail, projected_stop_mass = self._stop_projection(observation, control_rate)
        self._last_predictive_stop_candidate = False

        prediction_enabled_mass = self.target_mass_mg - self.settings.precision_start_remaining_mg
        projected_safety_stop = (
            settings.projected_safety_stop_enabled
            and observation.valid
            and observation.mass_mg >= prediction_enabled_mass
            and observation.mass_mg >= settings.force_continue_min_mass_mg
            and projected_stop_mass
            >= self.target_mass_mg + settings.projected_safety_stop_offset_mg
        )
        if projected_safety_stop:
            self._last_predictive_stop_candidate = True
            return self._decision(
                FeedbackStage.SETTLE,
                threshold,
                0.0,
                self.duty_permyriad,
                self.window_position_units,
                False,
                False,
                True,
                "projected safety stop: predicted final mass "
                f"{projected_stop_mass:.1f} >= "
                f"{self.target_mass_mg + settings.projected_safety_stop_offset_mg:.0f} mg",
                control_rate_mg_s=control_rate,
                estimated_tail_mg=estimated_tail,
                projected_stop_mass_mg=projected_stop_mass,
                stop_confirmation_required=1,
            )

        # ── 强制停止：质量已超过硬性上限 ──
        if observation.valid and observation.mass_mg >= settings.force_stop_max_mass_mg:
            return self._decision(
                FeedbackStage.SETTLE,
                threshold,
                0.0,
                self.duty_permyriad,
                self.window_position_units,
                False,
                False,
                True,
                f"force stop: mass {observation.mass_mg:.1f} >= "
                f"{settings.force_stop_max_mass_mg:.0f} mg",
                control_rate_mg_s=control_rate,
                estimated_tail_mg=estimated_tail,
                projected_stop_mass_mg=projected_stop_mass,
            )

        stall_recovery_decision = self._stall_recovery_step(
            observation=observation,
            timestamp_s=now,
            threshold_mg=threshold,
            projected_mass_mg=projected_mass,
            control_rate_mg_s=control_rate,
            estimated_tail_mg=estimated_tail,
            projected_stop_mass_mg=projected_stop_mass,
        )
        if stall_recovery_decision is not None:
            return stall_recovery_decision

        # ── 堵料检测：流量为零且仍低于继续阈值 ──
        if (
            observation.valid
            and observation.mass_mg < settings.force_continue_min_mass_mg
            and control_rate < settings.stall_rate_threshold_mg_s
        ):
            if self._stall_start_s is None:
                self._stall_start_s = now
            elif now - self._stall_start_s >= settings.stall_duration_s:
                return self._decision(
                    FeedbackStage.SETTLE,
                    threshold,
                    0.0,
                    self.duty_permyriad,
                    self.window_position_units,
                    False,
                    False,
                    True,
                    f"stall: rate < {settings.stall_rate_threshold_mg_s} mg/s "
                    f"for {settings.stall_duration_s:.0f} s "
                    f"below {settings.force_continue_min_mass_mg:.0f} mg",
                    control_rate_mg_s=control_rate,
                    estimated_tail_mg=estimated_tail,
                    projected_stop_mass_mg=projected_stop_mass,
                )
        else:
            self._stall_start_s = None

        lower_acceptance = self.target_mass_mg - self.settings.allowed_underweight_mg
        prediction_target = self.target_mass_mg - self.settings.stop_target_offset_mg
        predictive_stop = (
            self.settings.predictive_stop_enabled
            and observation.valid
            and observation.mass_mg >= prediction_enabled_mass
            and observation.mass_mg >= self.settings.force_continue_min_mass_mg
            and projected_stop_mass >= prediction_target
        )
        self._last_predictive_stop_candidate = predictive_stop
        stop_confirmation_required = settings.settle_confirmations
        if (
            predictive_stop
            and settings.fast_predictive_stop_enabled
            and control_rate >= settings.fast_predictive_stop_rate_mg_s
        ):
            stop_confirmation_required = settings.fast_predictive_stop_confirmations
        measured_stop = observation.mass_mg >= lower_acceptance
        if predictive_stop or measured_stop:
            self._settle_samples += 1
        else:
            self._settle_samples = 0
        stage = FeedbackStage.SETTLE if self._settle_samples >= stop_confirmation_required else (
            FeedbackStage.COARSE if projected_mass < threshold else FeedbackStage.FINE
        )
        remaining_mass = max(0.0, self.target_mass_mg - projected_mass)
        if stage is FeedbackStage.COARSE:
            target_rate = self.settings.coarse_rate_mg_s
        elif remaining_mass <= self.settings.precision_start_remaining_mg:
            target_rate = self.settings.precision_rate_mg_s
        else:
            target_rate = self.settings.fine_rate_mg_s
        if not bypass_rate_control:
            target_rate = min(target_rate, self.settings.maximum_flow_rate_mg_s)
        duty_before = self.duty_permyriad
        position_before = self.window_position_units
        if stage is FeedbackStage.SETTLE or not observation.valid:
            if stage is FeedbackStage.SETTLE and observation.valid:
                stop_reason = (
                    "predicted tail confirmed inside the acceptance window; "
                    "stop for final stable weighing"
                    if self._last_predictive_stop_candidate
                    else "measured mass confirmed inside the acceptance window; "
                    "stop for final stable weighing"
                )
                return self._decision(
                    stage,
                    threshold,
                    target_rate,
                    duty_before,
                    position_before,
                    False,
                    False,
                    True,
                    stop_reason,
                    control_rate_mg_s=control_rate,
                    estimated_tail_mg=estimated_tail,
                    projected_stop_mass_mg=projected_stop_mass,
                    stop_confirmation_required=stop_confirmation_required,
                )
            else:
                reason = "motion estimate not valid; hold seed parameters"
                return self._decision(
                    stage,
                    threshold,
                    target_rate,
                    duty_before,
                    position_before,
                    False,
                    False,
                    False,
                    reason,
                    control_rate_mg_s=control_rate,
                    estimated_tail_mg=estimated_tail,
                    projected_stop_mass_mg=projected_stop_mass,
                )

        # 固定占空比预投料有意以开环方式运行。  它仍会使用天平做
        # 安全/停止检查，但在精密闭环接管之前，不得对其限速
        # 或判定为超速。
        error = 0.0 if bypass_rate_control else target_rate - control_rate
        if dt > 0:
            self._integral_error_mg = max(-20.0, min(20.0, self._integral_error_mg + error * dt))
        absolute_overspeed = (
            not bypass_rate_control
            and control_rate > self.settings.maximum_flow_rate_mg_s
        )
        severe_overspeed = not bypass_rate_control and (
            absolute_overspeed
            or control_rate >= target_rate * self.settings.overspeed_multiple
        )
        if severe_overspeed:
            overspeed = max(0.0, control_rate - target_rate)
            emergency_steps = max(1, ceil(overspeed / 5.0))
            duty_delta = -min(
                self.settings.emergency_duty_step_per_update,
                emergency_steps * self.settings.duty_step_per_update,
            )
            self._over_rate_samples += 1
            self._under_rate_samples = 0
            reason = "predicted flow is above the overspeed guard; reduce duty"
        elif abs(error) <= self.settings.duty_deadband_mg_s:
            duty_delta = 0
            self._under_rate_samples = self._over_rate_samples = 0
            reason = "predicted flow is inside the duty deadband"
        else:
            self._under_rate_samples = self._under_rate_samples + 1 if error > 0 else 0
            self._over_rate_samples = self._over_rate_samples + 1 if error < 0 else 0
            duty_delta = round(
                self.settings.duty_gain_per_mg_s * error
                + self.settings.duty_integral_gain_per_mg * self._integral_error_mg
            )
            duty_delta = max(-self.settings.duty_step_per_update, min(self.settings.duty_step_per_update, duty_delta))
            if error > 0 and observation.acceleration_mg_s2 > self.settings.acceleration_guard_mg_s2:
                duty_delta = min(0, duty_delta)
                reason = "flow is below target but still accelerating; suppress duty increase"
            elif error < 0:
                reason = "predicted flow is above target; reduce duty"
            else:
                reason = "predicted flow is below target; increase duty"

        if settings.flow_hold_enabled and duty_delta > 0:
            probe = self._flow_duty_probe
            if probe is not None:
                probe_elapsed_s = now - probe["started_s"]
                if probe_elapsed_s < settings.flow_response_check_s:
                    duty_delta = 0
                    reason = "waiting for light-powder response before another duty increase"
                else:
                    self._flow_duty_probe = None
                    improved = (
                        flow_average_rate
                        >= probe["baseline_rate_mg_s"]
                        + settings.flow_response_min_improvement_mg_s
                    )
                    if flow_detected and not improved:
                        duty_delta = int(probe["baseline_duty_permyriad"]) - duty_before
                        self._integral_error_mg = min(0.0, self._integral_error_mg)
                        reason = (
                            "duty increase did not improve light-powder flow; "
                            "revert to the previous duty"
                        )
                    elif flow_detected:
                        duty_delta = 0
                        self._integral_error_mg = min(0.0, self._integral_error_mg)
                        reason = "light powder is gaining mass; hold duty"
            elif flow_detected:
                duty_delta = 0
                self._integral_error_mg = min(0.0, self._integral_error_mg)
                reason = "light powder is gaining mass; hold duty"
            elif (
                self._flow_hold_started_s is not None
                and now - self._flow_hold_started_s < settings.flow_hold_window_s
            ):
                duty_delta = 0
                self._integral_error_mg = min(0.0, self._integral_error_mg)
                reason = "observe light-powder mass gain before increasing duty"

        duty_interval = (
            self.settings.emergency_duty_update_interval_s
            if severe_overspeed
            else self.settings.duty_update_interval_s
        )
        if self._last_duty_update_s is None or now - self._last_duty_update_s >= duty_interval:
            duty_before_update = self.duty_permyriad
            self.duty_permyriad = self._bounded_duty(self.duty_permyriad + duty_delta)
            self._last_duty_update_s = now
            if settings.flow_hold_enabled:
                if self.duty_permyriad > duty_before_update:
                    self._flow_duty_probe = {
                        "started_s": now,
                        "baseline_duty_permyriad": float(duty_before_update),
                        "baseline_rate_mg_s": float(flow_average_rate),
                    }
                elif self.duty_permyriad < duty_before_update:
                    self._flow_duty_probe = None

        taper_progress = self._tail_taper_progress(projected_mass)
        if taper_progress > 0:
            # 只有在递减带内发生持续堵料后才跳过递减上限。
            # 这让 PI 环得以推高占空比以冲破架桥，一旦流量恢复
            # 即重新生效。
            skip_ceiling = False
            if control_rate < settings.stall_rate_threshold_mg_s:
                if self._taper_stall_start_s is None:
                    self._taper_stall_start_s = now
                elif now - self._taper_stall_start_s >= settings.taper_stall_timeout_s:
                    skip_ceiling = True
            else:
                self._taper_stall_start_s = None

            if not skip_ceiling:
                duty_ceiling = round(
                    self._initial_duty_permyriad
                    - settings.tail_duty_reduction_permyriad * taper_progress
                )
                duty_ceiling = self._bounded_duty(duty_ceiling)
                if self.duty_permyriad > duty_ceiling:
                    self.duty_permyriad = max(
                        duty_ceiling,
                        duty_before - settings.duty_step_per_update,
                    )
                    reason += "; remaining mass is small, taper duty for loose-powder tail feed"

        # 位置环有意设计得更慢，只有在持续出现速率误差
        # 或占空比环达到硬性极限时才进行微调。
        position_due = self._last_position_update_s is None or now - self._last_position_update_s >= self.settings.position_update_interval_s
        persistent_error = max(self._under_rate_samples, self._over_rate_samples) >= self.settings.position_persistence_samples
        duty_at_limit = self.duty_permyriad in (self.settings.min_duty_permyriad, self.settings.max_duty_permyriad)
        position_changed = False
        if (
            settings.position_feedback_enabled
            and position_due
            and persistent_error
            and (duty_at_limit or abs(error) >= self.settings.position_deadband_mg_s)
        ):
            direction = 1 if error > 0 else -1
            if not self.settings.position_increases_flow_with_units:
                direction *= -1
            self.window_position_units = max(
                self.settings.min_window_position_units,
                min(
                    self.settings.max_window_position_units,
                    self.window_position_units + direction * self.settings.position_step_units,
                ),
            )
            position_changed = self.window_position_units != position_before
            self._last_position_update_s = now
            if position_changed:
                reason += "; persistent error, trim window position"
        if settings.position_feedback_enabled and taper_progress > 0 and position_due:
            position_ceiling = round(
                self._initial_window_position_units
                - settings.tail_window_reduction_units * taper_progress
            )
            position_ceiling = max(settings.min_window_position_units, position_ceiling)
            if self.window_position_units > position_ceiling:
                self.window_position_units = max(
                    position_ceiling,
                    position_before - settings.position_step_units,
                )
                position_changed = self.window_position_units != position_before
                self._last_position_update_s = now
                if position_changed:
                    reason += "; taper window for loose-powder tail feed"

        return self._decision(
            stage,
            threshold,
            target_rate,
            duty_before,
            position_before,
            self.duty_permyriad != duty_before,
            position_changed,
            False,
            reason,
            rate_error_mg_s=error,
            control_rate_mg_s=control_rate,
            estimated_tail_mg=estimated_tail,
            projected_stop_mass_mg=projected_stop_mass,
        )

    def _decision(
        self,
        stage,
        threshold,
        target_rate,
        duty_before,
        position_before,
        duty_changed,
        position_changed,
        stop_requested,
        reason,
        *,
        rate_error_mg_s=0.0,
        control_rate_mg_s=0.0,
        estimated_tail_mg=0.0,
        projected_stop_mass_mg=0.0,
        stop_confirmation_required=None,
    ):
        return FeedbackDecision(
            stage=stage,
            coarse_threshold_mg=threshold,
            target_rate_mg_s=target_rate,
            frequency_hz=self.frequency_hz,
            duty_before_permyriad=duty_before,
            duty_after_permyriad=self.duty_permyriad,
            window_before_units=position_before,
            window_after_units=self.window_position_units,
            rate_error_mg_s=rate_error_mg_s,
            control_rate_mg_s=control_rate_mg_s,
            estimated_tail_mg=estimated_tail_mg,
            projected_stop_mass_mg=projected_stop_mass_mg,
            stop_requested=stop_requested,
            duty_changed=duty_changed,
            window_changed=position_changed,
            reason=reason,
            predictive_stop_candidate=self._last_predictive_stop_candidate,
            stop_confirmation_count=self._settle_samples,
            stop_confirmation_required=(
                self.settings.settle_confirmations
                if stop_confirmation_required is None
                else stop_confirmation_required
            ),
            stall_recovery_active=self._stall_recovery_active,
            stall_recovery_attempt=self._stall_recovery_attempts,
        )

    def _validate_initial(self, initial: FeedbackInitialParameters) -> None:
        settings = self.settings
        if not settings.min_frequency_hz <= initial.frequency_hz <= settings.max_frequency_hz:
            raise ValueError("initial frequency is outside controller limits")
        if not settings.min_duty_permyriad <= initial.duty_permyriad <= settings.max_duty_permyriad:
            raise ValueError("initial duty is outside controller limits")
        if not settings.min_window_position_units <= initial.window_position_units <= settings.max_window_position_units:
            raise ValueError("initial window position is outside controller limits")

    @staticmethod
    def _validate_observation(observation: FeedbackObservation) -> None:
        for name in ("mass_mg", "predicted_mass_mg", "rate_mg_s", "predicted_rate_mg_s"):
            if getattr(observation, name) < 0:
                raise ValueError(f"{name} must be non-negative")

    def _bounded_duty(self, value: int) -> int:
        return max(self.settings.min_duty_permyriad, min(self.settings.max_duty_permyriad, int(value)))

    def _stall_recovery_stage_and_rate(
        self,
        projected_mass_mg: float,
        threshold_mg: float,
    ) -> tuple[FeedbackStage, float]:
        stage = (
            FeedbackStage.COARSE
            if projected_mass_mg < threshold_mg
            else FeedbackStage.FINE
        )
        remaining_mass = max(0.0, self.target_mass_mg - projected_mass_mg)
        if stage is FeedbackStage.COARSE:
            target_rate = self.settings.coarse_rate_mg_s
        elif remaining_mass <= self.settings.precision_start_remaining_mg:
            target_rate = self.settings.precision_rate_mg_s
        else:
            target_rate = self.settings.fine_rate_mg_s
        return stage, min(target_rate, self.settings.maximum_flow_rate_mg_s)

    def _stall_recovery_step(
        self,
        *,
        observation: FeedbackObservation,
        timestamp_s: float,
        threshold_mg: float,
        projected_mass_mg: float,
        control_rate_mg_s: float,
        estimated_tail_mg: float,
        projected_stop_mass_mg: float,
    ) -> FeedbackDecision | None:
        """临时开大发生架桥的粉体出口，随后恢复到种子窗口位置。"""
        settings = self.settings
        if not settings.stall_recovery_enabled:
            return None
        if not observation.valid:
            self._stall_recovery_history = []
            return None

        self._stall_recovery_history.append((timestamp_s, observation.mass_mg))
        cutoff_s = timestamp_s - settings.stall_recovery_observation_window_s
        while (
            len(self._stall_recovery_history) >= 2
            and self._stall_recovery_history[1][0] <= cutoff_s
        ):
            self._stall_recovery_history.pop(0)

        stage, target_rate = self._stall_recovery_stage_and_rate(
            projected_mass_mg,
            threshold_mg,
        )
        remaining_mass = self.target_mass_mg - observation.mass_mg
        tail_recovery_blocked = (
            remaining_mass <= settings.stall_recovery_disable_remaining_mg
        )

        if self._stall_recovery_active:
            duty_before = self.duty_permyriad
            window_before = self.window_position_units
            recovered = (
                control_rate_mg_s >= settings.stall_recovery_resume_rate_mg_s
                or observation.mass_mg - self._stall_recovery_start_mass_mg
                >= settings.stall_recovery_min_gain_mg
            )
            if recovered or tail_recovery_blocked:
                self.duty_permyriad = min(
                    self.duty_permyriad,
                    self._initial_duty_permyriad,
                )
                self.window_position_units = self._initial_window_position_units
                self._stall_recovery_active = False
                self._stall_recovery_attempt_started_s = None
                self._stall_recovery_waiting_for_observation = False
                self._stall_recovery_history = [(timestamp_s, observation.mass_mg)]
                self._last_duty_update_s = timestamp_s
                self._last_position_update_s = timestamp_s
                reason = (
                    "stall recovery flow resumed; restore the configured window"
                    if recovered
                    else "stall recovery reached the protected final "
                    f"{settings.stall_recovery_disable_remaining_mg:g} mg; "
                    "restore the configured window"
                )
                decision = self._decision(
                    stage,
                    threshold_mg,
                    target_rate,
                    duty_before,
                    window_before,
                    self.duty_permyriad != duty_before,
                    self.window_position_units != window_before,
                    False,
                    reason,
                    control_rate_mg_s=control_rate_mg_s,
                    estimated_tail_mg=estimated_tail_mg,
                    projected_stop_mass_mg=projected_stop_mass_mg,
                )
                # 尝试次数限定于一次连续的堵料事件。  一旦流量恢复，
                # 之后独立的堵料事件必须获得全新的恢复预算，
                # 而不是继承已耗尽的尝试次数。
                self._stall_recovery_attempts = 0
                return decision

            if self._stall_recovery_waiting_for_observation:
                self._stall_recovery_waiting_for_observation = False
                self._stall_recovery_attempt_started_s = timestamp_s
                return self._decision(
                    stage,
                    threshold_mg,
                    target_rate,
                    duty_before,
                    window_before,
                    False,
                    False,
                    False,
                    "stall recovery active; begin observing the temporary window",
                    control_rate_mg_s=control_rate_mg_s,
                    estimated_tail_mg=estimated_tail_mg,
                    projected_stop_mass_mg=projected_stop_mass_mg,
                )

            attempt_elapsed_s = timestamp_s - (
                self._stall_recovery_attempt_started_s or timestamp_s
            )
            if attempt_elapsed_s >= settings.stall_recovery_retry_s:
                if self._stall_recovery_attempts >= settings.stall_recovery_max_attempts:
                    self._stall_recovery_active = False
                    return self._decision(
                        FeedbackStage.SETTLE,
                        threshold_mg,
                        0.0,
                        duty_before,
                        window_before,
                        False,
                        False,
                        True,
                        "stall recovery exhausted after "
                        f"{self._stall_recovery_attempts} temporary-window attempts",
                        control_rate_mg_s=control_rate_mg_s,
                        estimated_tail_mg=estimated_tail_mg,
                        projected_stop_mass_mg=projected_stop_mass_mg,
                    )
                self._stall_recovery_attempts += 1
                requested_window = min(
                    self._initial_window_position_units
                    + settings.stall_recovery_window_step_units
                    * self._stall_recovery_attempts,
                    settings.stall_recovery_max_window_position_units,
                )
                self.window_position_units = requested_window
                self._stall_recovery_attempt_started_s = timestamp_s
                self._stall_recovery_waiting_for_observation = True
                self._stall_recovery_start_mass_mg = observation.mass_mg
                self._last_position_update_s = timestamp_s
                return self._decision(
                    stage,
                    threshold_mg,
                    target_rate,
                    duty_before,
                    window_before,
                    False,
                    self.window_position_units != window_before,
                    False,
                    "stall recovery attempt "
                    f"{self._stall_recovery_attempts}: keep duty frozen and open window to "
                    f"{self.window_position_units}",
                    control_rate_mg_s=control_rate_mg_s,
                    estimated_tail_mg=estimated_tail_mg,
                    projected_stop_mass_mg=projected_stop_mass_mg,
                )

            return self._decision(
                stage,
                threshold_mg,
                target_rate,
                duty_before,
                window_before,
                False,
                False,
                False,
                "stall recovery active; hold duty and temporary window",
                control_rate_mg_s=control_rate_mg_s,
                estimated_tail_mg=estimated_tail_mg,
                projected_stop_mass_mg=projected_stop_mass_mg,
            )

        if tail_recovery_blocked or len(self._stall_recovery_history) < 2:
            return None
        observed_span_s = (
            timestamp_s - self._stall_recovery_history[0][0]
        )
        if observed_span_s < settings.stall_recovery_observation_window_s:
            return None
        observed_gain_mg = (
            observation.mass_mg - self._stall_recovery_history[0][1]
        )
        stalled = (
            observed_gain_mg <= settings.stall_recovery_min_gain_mg
            and control_rate_mg_s < settings.stall_recovery_rate_threshold_mg_s
        )
        if not stalled:
            return None

        duty_before = self.duty_permyriad
        window_before = self.window_position_units
        if self._stall_recovery_attempts >= settings.stall_recovery_max_attempts:
            return self._decision(
                FeedbackStage.SETTLE,
                threshold_mg,
                0.0,
                duty_before,
                window_before,
                False,
                False,
                True,
                "stall recovery exhausted after "
                f"{self._stall_recovery_attempts} temporary-window attempts",
                control_rate_mg_s=control_rate_mg_s,
                estimated_tail_mg=estimated_tail_mg,
                projected_stop_mass_mg=projected_stop_mass_mg,
            )

        self._stall_recovery_active = True
        self._stall_recovery_attempts += 1
        self._stall_recovery_attempt_started_s = timestamp_s
        self._stall_recovery_waiting_for_observation = True
        self._stall_recovery_start_mass_mg = observation.mass_mg
        self.duty_permyriad = min(
            self.duty_permyriad,
            self._initial_duty_permyriad,
        )
        self.window_position_units = min(
            self._initial_window_position_units
            + settings.stall_recovery_window_step_units
            * self._stall_recovery_attempts,
            settings.stall_recovery_max_window_position_units,
        )
        self._integral_error_mg = 0.0
        self._settle_samples = 0
        self._last_duty_update_s = timestamp_s
        self._last_position_update_s = timestamp_s
        return self._decision(
            stage,
            threshold_mg,
            target_rate,
            duty_before,
            window_before,
            self.duty_permyriad != duty_before,
            self.window_position_units != window_before,
            False,
            "stall detected: freeze duty and start temporary-window recovery "
            f"attempt {self._stall_recovery_attempts} at {self.window_position_units}",
            control_rate_mg_s=control_rate_mg_s,
            estimated_tail_mg=estimated_tail_mg,
            projected_stop_mass_mg=projected_stop_mass_mg,
        )

    def _tail_taper_progress(self, projected_mass_mg: float) -> float:
        settings = self.settings
        if not settings.tail_taper_enabled:
            return 0.0
        remaining = max(0.0, self.target_mass_mg - projected_mass_mg)
        if remaining >= settings.tail_taper_start_remaining_mg:
            return 0.0
        if remaining <= settings.tail_taper_end_remaining_mg:
            return 1.0
        span = settings.tail_taper_start_remaining_mg - settings.tail_taper_end_remaining_mg
        return (settings.tail_taper_start_remaining_mg - remaining) / span

    def _stop_projection(self, observation: FeedbackObservation, control_rate_mg_s: float) -> tuple[float, float]:
        """预测计入指令、天平与粉体下落延迟之后的质量。"""
        settings = self.settings
        horizon = settings.stop_prediction_horizon_s
        positive_acceleration = max(0.0, observation.acceleration_mg_s2)
        dynamic_tail = (
            control_rate_mg_s * horizon
            + 0.5 * positive_acceleration * horizon * horizon
        )
        observer_tail = max(0.0, observation.predicted_mass_mg - observation.mass_mg)
        estimated_tail = max(dynamic_tail, observer_tail) + settings.fixed_tail_mass_mg
        return estimated_tail, observation.mass_mg + estimated_tail
