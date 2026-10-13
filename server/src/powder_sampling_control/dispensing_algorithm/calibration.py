"""标定与实验算法 —— 单一整合模块。

合并的各部分：

    1. 恒速率控制器（原 constant_rate_controller.py）
    2. 连续锥形规划器（原 continuous_taper_planner.py）
    3. 窗口位置 PID 规划器（原 window_pid_planner.py）
    4. 粉末指纹探针（原 powder_fingerprint.py）
    5. 目标速率带搜索器（原 rate_band_searcher.py）
    6. 实验网格编排器（原 experiment_orchestrator.py）

原先同时存在于指纹模块与速率搜索模块中的重复辅助函数（天平读数、
窗口移动常量、启动阈值）现在只定义一次。共享的生产契约来自 ``core``。
"""

from __future__ import annotations

import itertools
import json
import math
import os
import shutil
import time
import uuid
from collections import deque
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from time import monotonic
from typing import Any, Callable, Optional

from .core import FeedAction, normalize_target_keys, powder_classification_params

@dataclass(frozen=True)
class ConstantRateSettings:
    """默认参数针对以频率+占空比为主、严格速率调节的场景调校。"""

    # ── 频率环路（粗调旋钮 —— 仅在占空比饱和时使用） ──
    freq_gain_per_mg_s: float = 8.0
    freq_integral_gain_per_mg: float = 0.5
    freq_step_hz: int = 1
    freq_update_interval_s: float = 2.0
    freq_deadband_mg_s: float = 0.50

    # ── 占空比环路（主细调旋钮 —— 所有工况下优先调整） ──
    duty_gain_per_mg_s: float = 60.0
    duty_integral_gain_per_mg: float = 1.5
    duty_step_per_update: int = 20
    emergency_duty_step_per_update: int = 30
    duty_update_interval_s: float = 2.0
    emergency_duty_update_interval_s: float = 1.0
    duty_deadband_mg_s: float = 0.50

    # ── 参数锁定 ──
    # 当 |rate_error| <= lock_range_mg_s 时，频率和占空比冻结在当前值。
    # 当速率漂移出该区间或检测到严重超速时，自动解锁。
    lock_range_mg_s: float = 1.5

    # ── 频率锁定 ──
    # 当系统在此时长内未发生超速和堵塞时，频率锁定在当前值，
    # 仅调整占空比。发生超速、堵塞或占空比触及限值时，频率解锁。
    freq_lock_stable_s: float = 3.0

    # ── 窗口位置环路（最后手段，非常保守） ──
    position_step_units: int = 5
    position_update_interval_s: float = 5.0
    position_deadband_mg_s: float = 4.0
    position_persistence_samples: int = 12

    # ── 边界 ──
    min_frequency_hz: int = 10
    max_frequency_hz: int = 80
    min_duty_permyriad: int = 1000
    max_duty_permyriad: int = 5000
    min_window_units: int = 100
    # 服务端使用 LA10 窗口绝对坐标。Web 客户端会将其相对的 100..750
    # 显示范围平移 +700，因此像 300 这样的正常请求到达该控制器时
    # 坐标为 1000。此项需与服务端执行器限值保持一致。
    max_window_units: int = 2050

    # ── 堵塞检测 ──
    # 当估计速率在 clog_duration_s 内持续低于该阈值时，控制器进入
    # "堵塞"模式：仅增大频率（以抖松粉末），占空比保持不变。
    clog_rate_threshold_mg_s: float = 0.5
    clog_duration_s: float = 3.0

    # ── 振荡触发的窗口微调 ──
    # 当频率在狭窄范围内来回翻转时，±2 Hz 的步长对该粉末而言过粗。
    # 用滑动窗口跟踪最近的频率变化；当以下条件全部满足时判定为振荡：
    #   1. 窗口内方向翻转次数 ≥ oscillation_flip_threshold
    #   2. 窗口内频率散布 ≤ oscillation_max_range_hz
    #   3. 不存在 ≥ oscillation_max_consecutive_same 个连续同方向步进
    # 响应方式：比较各观测频率下的平均速率；若较高频率值更接近目标
    # → 收小窗口，否则开大窗口。
    oscillation_flip_threshold: int = 2
    oscillation_max_range_hz: int = 12
    oscillation_max_consecutive_same: int = 8
    oscillation_window_step: int = 10
    oscillation_cooldown_s: float = 6.0

    # ── 安全 ──
    acceleration_guard_mg_s2: float = 2.0
    overspeed_multiple: float = 1.5
    maximum_flow_rate_mg_s: float = 100.0

    def __post_init__(self) -> None:
        if min(self.freq_gain_per_mg_s, self.freq_integral_gain_per_mg,
               self.duty_gain_per_mg_s, self.duty_integral_gain_per_mg,
               self.freq_deadband_mg_s, self.duty_deadband_mg_s,
               self.position_deadband_mg_s,
               self.freq_update_interval_s, self.duty_update_interval_s,
               self.position_update_interval_s) <= 0:
            raise ValueError("controller gains, deadbands, and intervals must be positive")
        if self.lock_range_mg_s <= 0:
            raise ValueError("lock_range_mg_s must be positive")
        if self.freq_step_hz < 1 or self.duty_step_per_update < 1 or self.position_step_units < 1:
            raise ValueError("controller steps must be positive")
        if not 10 <= self.min_frequency_hz < self.max_frequency_hz <= 80:
            raise ValueError("frequency limits are invalid")
        if not 1000 <= self.min_duty_permyriad < self.max_duty_permyriad <= 5000:
            raise ValueError("duty limits are invalid")
        if not 0 <= self.min_window_units < self.max_window_units:
            raise ValueError("window limits are invalid")
        if self.overspeed_multiple <= 1:
            raise ValueError("overspeed_multiple must be greater than one")
        if self.maximum_flow_rate_mg_s <= 0:
            raise ValueError("maximum_flow_rate_mg_s must be positive")
        if self.emergency_duty_step_per_update < self.duty_step_per_update:
            raise ValueError("emergency duty step must be at least the normal duty step")
        if not 0 < self.emergency_duty_update_interval_s <= self.duty_update_interval_s:
            raise ValueError("emergency duty interval must be positive and no slower than normal")
        if self.oscillation_flip_threshold < 2:
            raise ValueError("oscillation_flip_threshold must be >= 2")
        if not 2 <= self.oscillation_max_range_hz <= 20:
            raise ValueError("oscillation_max_range_hz must be 2–20 Hz")
        if self.oscillation_max_consecutive_same < 2:
            raise ValueError("oscillation_max_consecutive_same must be >= 2")
        if not 1 <= self.oscillation_window_step <= self.position_step_units * 3:
            raise ValueError("oscillation_window_step must be 1–position_step_units*3")
        if self.oscillation_cooldown_s <= 0:
            raise ValueError("oscillation_cooldown_s must be positive")


@dataclass(frozen=True)
class ConstantRateObservation:
    """来自天平与运动估计器的一次实时观测。"""

    rate_mg_s: float
    predicted_rate_mg_s: float
    acceleration_mg_s2: float = 0.0
    valid: bool = True


@dataclass(frozen=True)
class ConstantRateDecision:
    target_rate_mg_s: float
    control_rate_mg_s: float
    rate_error_mg_s: float
    frequency_before_hz: int
    frequency_after_hz: int
    duty_before_permyriad: int
    duty_after_permyriad: int
    window_before_units: int
    window_after_units: int
    freq_changed: bool
    duty_changed: bool
    window_changed: bool
    locked: bool
    clogged: bool
    reason: str


class ConstantRateController:
    """用于维持恒定粉末流量的三层级控制器。

    频率与占空比同为一级执行器 —— 每隔约 1.5 秒根据速率误差同时调整两者，
    给粉末流动留出每次变化后的稳定时间。频率是较粗的旋钮，占空比是较细的。

    一旦速率稳定在目标附近 lock_range_mg_s 范围内，控制器将频率和占空比
    冻结在当前值。若速率漂移出锁定区间或发生严重超速，则自动解冻。

    正常运行期间窗口位置被有意固定。仅当频率和占空比均已达到最大限值、
    而流量仍低于目标时，才以一个极小步长开大窗口。反馈绝不会收小窗口
    —— 收小会导致粉末堵塞。
    """

    def __init__(
        self,
        *,
        target_rate_mg_s: float,
        initial_duty_permyriad: int,
        initial_window_units: int,
        frequency_hz: int,
        settings: ConstantRateSettings | None = None,
    ) -> None:
        if target_rate_mg_s <= 0:
            raise ValueError("target_rate_mg_s must be positive")
        self.target_rate_mg_s = float(target_rate_mg_s)
        self.settings = settings or ConstantRateSettings()
        s = self.settings
        if not s.min_frequency_hz <= frequency_hz <= s.max_frequency_hz:
            raise ValueError("frequency_hz is outside controller limits")
        if not s.min_duty_permyriad <= initial_duty_permyriad <= s.max_duty_permyriad:
            raise ValueError("initial_duty_permyriad is outside controller limits")
        if not s.min_window_units <= initial_window_units <= s.max_window_units:
            raise ValueError("initial_window_units is outside controller limits")
        self.frequency_hz = frequency_hz
        self.duty_permyriad = initial_duty_permyriad
        self.window_position_units = initial_window_units

        # 独立的积分项 —— 频率和占空比是物理上不同的执行器
        self._freq_integral_error_mg = 0.0
        self._duty_integral_error_mg = 0.0

        # 锁定状态 —— 为 True 时频率和占空比被冻结
        self._locked = False

        # 堵塞状态 —— 为 True 时频率+占空比一同增大
        self._clogged = False
        self._zero_rate_since_s: float | None = None

        # 频率锁定 —— 在 freq_lock_stable_s 内无超速/无堵塞后，
        # 频率保持不变，仅占空比调整。发生超速、堵塞或占空比触及
        # 硬限值时解锁。
        self._freq_locked = False
        self._freq_stable_start: float | None = None

        # 振荡跟踪 —— 滑动窗口方向翻转检测
        self._osc_history: deque[tuple[int, int]] = deque(maxlen=16)  # (频率, 方向增量)
        self._osc_rate_map: dict[int, tuple[float, int]] = {}  # 频率 → (速率和, 计数)
        self._osc_last_change_time: float | None = None
        self._osc_last_window_time: float | None = None
        self._osc_lock_start_time: float | None = None  # 连续锁定时长
        self._osc_same_dir_count: int = 0  # 连续同方向窗口微调次数
        self._osc_last_dir: int = 0  # 上次微调方向：1=开大, -1=收小

        self._last_timestamp_s: float | None = None
        self._last_freq_update_s: float | None = None
        self._last_duty_update_s: float | None = None
        self._last_position_update_s: float | None = None
        self._under_rate_samples = 0
        self._over_rate_samples = 0

    def reset(self) -> None:
        self._freq_integral_error_mg = 0.0
        self._duty_integral_error_mg = 0.0
        self._locked = False
        self._clogged = False
        self._zero_rate_since_s = None
        self._freq_locked = False
        self._freq_stable_start = None
        self._osc_history.clear()
        self._osc_rate_map.clear()
        self._osc_last_change_time = None
        self._osc_last_window_time = None
        self._osc_lock_start_time = None
        self._osc_same_dir_count = 0
        self._osc_last_dir = 0
        self._last_timestamp_s = None
        self._last_freq_update_s = None
        self._last_duty_update_s = None
        self._last_position_update_s = None
        self._under_rate_samples = 0
        self._over_rate_samples = 0

    @property
    def locked(self) -> bool:
        """频率和占空比冻结在当前值时为 True。"""
        return self._locked

    def step(
        self,
        observation: ConstantRateObservation,
        timestamp_s: float | None = None,
    ) -> ConstantRateDecision:
        self._validate_observation(observation)
        s = self.settings
        now = monotonic() if timestamp_s is None else float(timestamp_s)
        if self._last_timestamp_s is not None and now <= self._last_timestamp_s:
            raise ValueError("controller timestamps must be strictly increasing")
        dt = 0.0 if self._last_timestamp_s is None else min(2.0, now - self._last_timestamp_s)
        self._last_timestamp_s = now

        # 稳定控制信号：滤波速率与预测速率的加权融合
        control_rate = 0.8 * observation.rate_mg_s + 0.2 * observation.predicted_rate_mg_s
        error = self.target_rate_mg_s - control_rate

        # 更新各自的积分项（限幅）—— 仅在未锁定时
        if dt > 0 and not self._locked:
            self._freq_integral_error_mg = max(
                -20.0, min(20.0, self._freq_integral_error_mg + error * dt)
            )
            self._duty_integral_error_mg = max(
                -20.0, min(20.0, self._duty_integral_error_mg + error * dt)
            )

        freq_before = self.frequency_hz
        duty_before = self.duty_permyriad
        position_before = self.window_position_units

        # --- 预热期间保持 ---
        if not observation.valid:
            return ConstantRateDecision(
                target_rate_mg_s=self.target_rate_mg_s,
                control_rate_mg_s=control_rate,
                rate_error_mg_s=error,
                frequency_before_hz=freq_before,
                frequency_after_hz=freq_before,
                duty_before_permyriad=duty_before,
                duty_after_permyriad=duty_before,
                window_before_units=position_before,
                window_after_units=position_before,
                freq_changed=False,
                duty_changed=False,
                window_changed=False,
                locked=False,
                clogged=False,
                reason="motion estimate not valid; hold initial parameters",
            )

        # ── 超速检测 ──
        absolute_overspeed = control_rate > s.maximum_flow_rate_mg_s
        severe_overspeed = absolute_overspeed or control_rate >= self.target_rate_mg_s * s.overspeed_multiple

        # ── 锁定管理 ──
        # 严重超速始终会解除锁定（安全优先）。
        # 否则，速率位于锁定区间内时加锁；漂移出区间时解锁。
        if severe_overspeed:
            self._locked = False
            self._freq_integral_error_mg = 0.0
            self._duty_integral_error_mg = 0.0
        elif abs(error) <= s.lock_range_mg_s:
            if not self._locked:
                self._locked = True
                # 进入锁定时清零积分项以防止积分饱和
                self._freq_integral_error_mg = 0.0
                self._duty_integral_error_mg = 0.0
        else:
            self._locked = False

        # ── 堵塞检测 ──
        # 速率在 clog_duration_s 内接近零 → 可能发生粉末架桥/堵塞。
        if observation.rate_mg_s < s.clog_rate_threshold_mg_s:
            if self._zero_rate_since_s is None:
                self._zero_rate_since_s = now
            elif now - self._zero_rate_since_s >= s.clog_duration_s:
                if not self._clogged:
                    self._clogged = True
                    # 清零积分项，让频率 PI 从头开始
                    self._freq_integral_error_mg = 0.0
        else:
            self._clogged = False
            self._zero_rate_since_s = None

        # ── 频率锁定管理 ──
        # 在 freq_lock_stable_s 内无超速且无堵塞后，系统即被认为已找到
        # 一个可用的频率。从那时起频率保持不变，仅由占空比微调速率。
        # 超速或堵塞会立即解除频率锁定。
        if severe_overspeed or self._clogged:
            self._freq_locked = False
            self._freq_stable_start = None
        elif self._freq_stable_start is None:
            self._freq_stable_start = now
        elif now - self._freq_stable_start >= s.freq_lock_stable_s:
            self._freq_locked = True

        reason = ""
        freq_delta = 0
        duty_delta = 0

        # ── 占空比优先工况 ──
        # 所有工况下都先调整占空比。仅当占空比触及硬限值，或处于堵塞
        # 状态（此时两者一同增大以突破粉末架桥）时，频率才会变动。
        if severe_overspeed:
            # ── 紧急处理：先降占空比；仅当占空比处于最小值时才降频率 ──
            overspeed = max(0.0, control_rate - self.target_rate_mg_s)
            emergency_steps = max(1, int(overspeed / 5.0))

            if self.duty_permyriad > s.min_duty_permyriad:
                duty_delta = -min(
                    s.emergency_duty_step_per_update,
                    emergency_steps * s.duty_step_per_update,
                )
                freq_delta = 0
                reason = "flow is above the overspeed guard; reduce duty"
            else:
                duty_delta = 0
                freq_delta = -s.freq_step_hz
                reason = "flow is above the overspeed guard; duty at minimum, reduce frequency"
            self._over_rate_samples += 1
            self._under_rate_samples = 0
        elif self._locked:
            # ── 已锁定：保持当前参数 ──
            freq_delta = 0
            duty_delta = 0
            self._under_rate_samples = self._over_rate_samples = 0
            reason = f"flow is within ±{s.lock_range_mg_s} mg/s of target; parameters locked"
        elif abs(error) <= s.freq_deadband_mg_s:
            # ── 死区：两者均保持 ──
            freq_delta = 0
            duty_delta = 0
            self._under_rate_samples = self._over_rate_samples = 0
            reason = "flow is inside the deadband"
        elif self._clogged and error > 0:
            # ── 堵塞：频率与占空比一同增大以突破堵塞 ──
            # 频率+占空比并行上升，使粉末在更低、更平缓的频率下
            # 达到流动阈值。
            freq_delta = round(
                s.freq_gain_per_mg_s * error
                + s.freq_integral_gain_per_mg * self._freq_integral_error_mg
            )
            freq_delta = max(0, min(s.freq_step_hz, freq_delta))
            duty_delta = round(
                s.duty_gain_per_mg_s * error
                + s.duty_integral_gain_per_mg * self._duty_integral_error_mg
            )
            duty_delta = max(0, min(s.duty_step_per_update, duty_delta))
            self._under_rate_samples += 1
            self._over_rate_samples = 0
            reason = "clog suspected; increase frequency and duty"
        elif error < 0:
            # ── 高于目标：先降占空比 ──
            self._under_rate_samples = 0
            self._over_rate_samples += 1

            if self.duty_permyriad > s.min_duty_permyriad:
                duty_delta = round(
                    s.duty_gain_per_mg_s * error
                    + s.duty_integral_gain_per_mg * self._duty_integral_error_mg
                )
                duty_delta = max(-s.duty_step_per_update, min(0, duty_delta))
                freq_delta = 0
                reason = "flow is above target; reduce duty"
            else:
                freq_delta = round(
                    s.freq_gain_per_mg_s * error
                    + s.freq_integral_gain_per_mg * self._freq_integral_error_mg
                )
                freq_delta = max(-s.freq_step_hz, min(0, freq_delta))
                duty_delta = 0
                reason = "flow is above target; duty at minimum, reduce frequency"
        else:
            # ── 低于目标：先增占空比 ──
            self._under_rate_samples += 1
            self._over_rate_samples = 0

            if self.duty_permyriad < s.max_duty_permyriad:
                duty_delta = round(
                    s.duty_gain_per_mg_s * error
                    + s.duty_integral_gain_per_mg * self._duty_integral_error_mg
                )
                duty_delta = max(0, min(s.duty_step_per_update, duty_delta))
                # 加速度保护：若已在加速则不再增大
                if observation.acceleration_mg_s2 > s.acceleration_guard_mg_s2:
                    duty_delta = 0
                freq_delta = 0
                reason = "flow is below target; increase duty"
            else:
                freq_delta = round(
                    s.freq_gain_per_mg_s * error
                    + s.freq_integral_gain_per_mg * self._freq_integral_error_mg
                )
                freq_delta = max(0, min(s.freq_step_hz, freq_delta))
                if observation.acceleration_mg_s2 > s.acceleration_guard_mg_s2:
                    freq_delta = 0
                duty_delta = 0
                reason = "flow is below target; duty at maximum, increase frequency"

        # ── 频率锁定 ──
        # 锁定时抑制所有频率变化。若误差仍朝该方向推动而占空比已触及
        # 硬限值，则解锁频率，让其接管控制。
        if self._freq_locked:
            if (self.duty_permyriad >= s.max_duty_permyriad and error > s.duty_deadband_mg_s) or \
               (self.duty_permyriad <= s.min_duty_permyriad and error < -s.duty_deadband_mg_s):
                self._freq_locked = False
                self._freq_stable_start = None
                reason += " (freq unlocked: duty at limit)"
            else:
                freq_delta = 0

        # ── 应用频率变化 ──
        freq_interval = s.freq_update_interval_s
        if self._last_freq_update_s is None or now - self._last_freq_update_s >= freq_interval:
            self.frequency_hz = self._bounded_freq(self.frequency_hz + freq_delta)
            self._last_freq_update_s = now

        # ── 应用占空比变化 ──
        duty_interval = (
            s.emergency_duty_update_interval_s
            if severe_overspeed
            else s.duty_update_interval_s
        )
        if self._last_duty_update_s is None or now - self._last_duty_update_s >= duty_interval:
            self.duty_permyriad = self._bounded_duty(self.duty_permyriad + duty_delta)
            self._last_duty_update_s = now

        # ── 振荡触发的窗口微调 ──
        # 滑动窗口方法：在固定大小的 deque 中跟踪最近的频率变化。
        # 当以下条件全部满足时判定为振荡：
        #   1. 方向翻转次数 ≥ oscillation_flip_threshold
        #   2. 频率范围 ≤ oscillation_max_range_hz
        #   3. 最长同方向连续步进 < oscillation_max_consecutive_same
        # 这能捕获 62→64→66→62→60→64 这类多值模式，
        # 而不仅是严格的 A↔B↔A 双值切换。
        position_changed = False
        freq_changed = self.frequency_hz != freq_before
        in_cooldown = (
            self._osc_last_window_time is not None
            and now - self._osc_last_window_time < s.oscillation_cooldown_s
        )

        # ── 锁定时长跟踪 ──
        # 仅在*连续*锁定超过 4 秒后才清空历史。振荡期间对锁定区间的
        # 短暂触碰不得清除历史。
        if self._locked:
            if self._osc_lock_start_time is None:
                self._osc_lock_start_time = now
            elif now - self._osc_lock_start_time > 4.0:
                self._osc_history.clear()
                self._osc_rate_map.clear()
                self._osc_last_change_time = None
                self._osc_lock_start_time = None
        else:
            self._osc_lock_start_time = None

        # 严重超速是振荡模式的一部分（堵塞爬升 → 超速 → 回切 → 堵塞 …）。
        # 此处切勿清空历史 —— 清空会导致振荡检测永远无法触发。
        if not self._locked:
            # 若距上次频率变化已间隔很久，则重新开始 ——
            # 旧历史不属于当前模式。
            if (
                self._osc_last_change_time is not None
                and now - self._osc_last_change_time > s.freq_update_interval_s * 3
            ):
                self._osc_history.clear()
                self._osc_rate_map.clear()

            if freq_changed:
                direction = 1 if self.frequency_hz > freq_before else -1
                self._osc_history.append((self.frequency_hz, direction))
                self._osc_last_change_time = now

            # 每一步都累积速率样本（而不仅在频率变化时），
            # 使每个频率获得多个样本以得到可靠的平均值。
            # 冷却期间仍继续累积 —— 仅暂停触发；历史应持续积累，
            # 以便冷却结束后能立即触发。
            key = self.frequency_hz
            prev_sum, prev_cnt = self._osc_rate_map.get(key, (0.0, 0))
            self._osc_rate_map[key] = (prev_sum + control_rate, prev_cnt + 1)

        # ── 检查振荡触发条件 ──
        min_history = s.oscillation_flip_threshold + 2
        if len(self._osc_history) >= min_history and not in_cooldown:
            directions = [d for _, d in self._osc_history]

            # 1. 统计方向翻转次数
            flips = sum(
                1 for i in range(1, len(directions))
                if directions[i] != directions[i - 1]
            )

            # 2. 最长连续同方向步进
            max_same = 1
            cur = 1
            for i in range(1, len(directions)):
                if directions[i] == directions[i - 1]:
                    cur += 1
                    if cur > max_same:
                        max_same = cur
                else:
                    cur = 1

            # 3. 窗口内频率范围
            freqs = [f for f, _ in self._osc_history]
            freq_range = max(freqs) - min(freqs)

            if (
                flips >= s.oscillation_flip_threshold
                and freq_range <= s.oscillation_max_range_hz
                and max_same < s.oscillation_max_consecutive_same
            ):
                # 确认振荡 —— 比较各频率下的平均速率以决定窗口调整方向。
                sorted_freqs = sorted(self._osc_rate_map.keys())
                if len(sorted_freqs) >= 2:
                    # 按中位数将观测到的频率分为"下半部分"和
                    # "上半部分"，然后比较两者的平均速率误差，
                    # 以决定窗口调整方向。
                    mid = len(sorted_freqs) // 2
                    low_freqs = sorted_freqs[:mid]
                    high_freqs = sorted_freqs[mid:]

                    low_sum = 0.0
                    low_cnt = 0
                    for f in low_freqs:
                        s_rate, cnt = self._osc_rate_map[f]
                        low_sum += s_rate
                        low_cnt += cnt
                    high_sum = 0.0
                    high_cnt = 0
                    for f in high_freqs:
                        s_rate, cnt = self._osc_rate_map[f]
                        high_sum += s_rate
                        high_cnt += cnt

                    if low_cnt > 0 and high_cnt > 0:
                        low_avg = low_sum / low_cnt
                        high_avg = high_sum / high_cnt
                        low_err = abs(self.target_rate_mg_s - low_avg)
                        high_err = abs(self.target_rate_mg_s - high_avg)

                        low_str = ",".join(str(f) for f in low_freqs)
                        high_str = ",".join(str(f) for f in high_freqs)

                        # 决定方向：
                        #   高频更接近 且 高频 > 目标  → 收小（减少过冲）
                        #   高频更接近 且 高频 < 目标  → 开大（两者都偏低，需要更多）
                        #   低频更接近                → 开大（与之前相同）
                        tgt = self.target_rate_mg_s
                        if high_err < low_err and high_avg > tgt:
                            direction = -1  # 收小
                        else:
                            direction = 1   # 开大

                        # ── 同方向保护 ──
                        # 若最近 3 次微调均为同一方向，则跳过本次微调，
                        # 防止失控（例如窗口一路收小陷入死循环）。
                        if (
                            direction == self._osc_last_dir
                            and self._osc_same_dir_count >= 3
                        ):
                            reason += (
                                f"; freq oscillation [{low_str}]<->[{high_str}]Hz "
                                f"(window trim skipped: {self._osc_same_dir_count} "
                                f"consecutive same-direction trims)"
                            )
                        else:
                            if direction == 1:
                                self.window_position_units = min(
                                    s.max_window_units,
                                    self.window_position_units + s.oscillation_window_step,
                                )
                                position_changed = self.window_position_units != position_before
                                reason += (
                                    f"; freq oscillation [{low_str}]<->[{high_str}]Hz "
                                    f"(low {low_str}Hz avg {low_avg:.1f}, "
                                    f"high {high_str}Hz avg {high_avg:.1f} — "
                                    f"open window {s.oscillation_window_step}u)"
                                )
                            else:
                                self.window_position_units = max(
                                    s.min_window_units,
                                    self.window_position_units - s.oscillation_window_step,
                                )
                                position_changed = self.window_position_units != position_before
                                reason += (
                                    f"; freq oscillation [{low_str}]<->[{high_str}]Hz "
                                    f"(low {low_str}Hz avg {low_avg:.1f}, "
                                    f"high {high_str}Hz avg {high_avg:.1f} — "
                                    f"close window {s.oscillation_window_step}u)"
                                )

                            # 记录方向用于同方向保护
                            if direction == self._osc_last_dir:
                                self._osc_same_dir_count += 1
                            else:
                                self._osc_same_dir_count = 1
                            self._osc_last_dir = direction

                # 窗口调整后重置跟踪并启动冷却
                self._osc_history.clear()
                self._osc_rate_map.clear()
                self._osc_last_change_time = None
                self._osc_last_window_time = now
                self._freq_integral_error_mg = 0.0

        # ── 窗口位置调整（最后手段） ──
        # 正常运行期间窗口保持固定 —— 仅调整频率和占空比。当频率或占空比
        # 任一（或两者）达到最大限值而流量仍低于目标时，以极小步长开大
        # 窗口。反馈绝不会收小窗口（收小会导致粉末堵塞）。
        position_due = (
            self._last_position_update_s is None
            or now - self._last_position_update_s >= s.position_update_interval_s
        )
        persistent_low = self._under_rate_samples >= s.position_persistence_samples
        freq_at_limit = self.frequency_hz == s.max_frequency_hz
        duty_at_limit = self.duty_permyriad == s.max_duty_permyriad
        if (
            position_due
            and persistent_low
            and error > 0
            and (freq_at_limit or duty_at_limit)
        ):
            self.window_position_units = min(
                s.max_window_units,
                self.window_position_units + s.position_step_units,
            )
            position_changed = self.window_position_units != position_before
            self._last_position_update_s = now
            if position_changed:
                who = "freq" if freq_at_limit else "duty"
                reason += f"; {who} at max, open window slightly"
            self._under_rate_samples = 0

        return ConstantRateDecision(
            target_rate_mg_s=self.target_rate_mg_s,
            control_rate_mg_s=control_rate,
            rate_error_mg_s=error,
            frequency_before_hz=freq_before,
            frequency_after_hz=self.frequency_hz,
            duty_before_permyriad=duty_before,
            duty_after_permyriad=self.duty_permyriad,
            window_before_units=position_before,
            window_after_units=self.window_position_units,
            freq_changed=self.frequency_hz != freq_before,
            duty_changed=self.duty_permyriad != duty_before,
            window_changed=position_changed,
            locked=self._locked,
            clogged=self._clogged,
            reason=reason,
        )

    @staticmethod
    def _validate_observation(observation: ConstantRateObservation) -> None:
        for name in ("rate_mg_s", "predicted_rate_mg_s"):
            if getattr(observation, name) < 0:
                raise ValueError(f"{name} must be non-negative")

    def _bounded_freq(self, value: int) -> int:
        return max(
            self.settings.min_frequency_hz,
            min(self.settings.max_frequency_hz, int(value)),
        )

    def _bounded_duty(self, value: int) -> int:
        return max(
            self.settings.min_duty_permyriad,
            min(self.settings.max_duty_permyriad, int(value)),
        )


class ContinuousTaperStage(str, Enum):
    COARSE = "coarse"
    TAPER = "taper"
    PRECISION = "precision"
    SETTLE = "settle"
    REVIEW = "review"


@dataclass(frozen=True)
class ContinuousAction:
    window_position_units: int
    frequency_hz: int
    duty_permyriad: int


@dataclass(frozen=True)
class ContinuousTaperConfig:
    target_mass_mg: float = 500.0
    hard_overweight_mg: float = 510.0
    coarse_end_mg: float = 420.0
    stop_raw_mg: float = 498.0
    stop_filtered_mg: float = 495.0
    stop_prediction_mg: float = 500.0
    min_raw_for_predictive_stop_mg: float = 485.0
    prediction_horizon_s: float = 0.75
    fixed_tail_mg: float = 2.0
    coarse: ContinuousAction = ContinuousAction(200, 80, 2000)
    # 末段 80 mg 模式有意一次性下调所有执行量然后保持不变。
    # 它不是速率控制器，而是一种低能量缓流模式。
    taper: ContinuousAction = ContinuousAction(100, 65, 1600)
    precision: ContinuousAction = ContinuousAction(100, 65, 1600)


@dataclass(frozen=True)
class ContinuousTaperDecision:
    stage: ContinuousTaperStage
    command: str
    action: ContinuousAction | None
    raw_mass_mg: float
    filtered_mass_mg: float
    predicted_mass_mg: float
    remaining_predicted_mg: float
    reason: str


class ContinuousTaperPlanner:
    """固定阶段连续规划器，带预测性尾部停止。"""

    def __init__(self, config: ContinuousTaperConfig | None = None) -> None:
        self.config = config or ContinuousTaperConfig()

    def decide(
        self,
        *,
        raw_mass_mg: float,
        filtered_mass_mg: float | None = None,
        rate_mg_s: float = 0.0,
        acceleration_mg_s2: float = 0.0,
        predicted_mass_mg: float | None = None,
        current_window_position_units: int | None = None,
        current_frequency_hz: int | None = None,
        current_duty_permyriad: int | None = None,
    ) -> ContinuousTaperDecision:
        if raw_mass_mg < 0:
            raise ValueError("raw_mass_mg must be non-negative")
        if filtered_mass_mg is not None and filtered_mass_mg < 0:
            raise ValueError("filtered_mass_mg must be non-negative")
        if rate_mg_s < 0:
            raise ValueError("rate_mg_s must be non-negative")

        cfg = self.config
        filtered = raw_mass_mg if filtered_mass_mg is None else filtered_mass_mg
        predicted = self._predict_mass(
            raw_mass_mg=raw_mass_mg,
            filtered_mass_mg=filtered,
            rate_mg_s=rate_mg_s,
            acceleration_mg_s2=acceleration_mg_s2,
            predicted_mass_mg=predicted_mass_mg,
        )

        if raw_mass_mg >= cfg.hard_overweight_mg:
            return self._stop(
                ContinuousTaperStage.REVIEW,
                raw_mass_mg,
                filtered,
                predicted,
                "hard overweight boundary reached",
            )
        if raw_mass_mg >= cfg.stop_raw_mg:
            return self._stop(
                ContinuousTaperStage.SETTLE,
                raw_mass_mg,
                filtered,
                predicted,
                "raw mass reached the continuous stop threshold",
            )
        if filtered >= cfg.stop_filtered_mg and raw_mass_mg >= cfg.min_raw_for_predictive_stop_mg:
            return self._stop(
                ContinuousTaperStage.SETTLE,
                raw_mass_mg,
                filtered,
                predicted,
                "filtered mass reached the continuous stop threshold",
            )
        if raw_mass_mg >= cfg.min_raw_for_predictive_stop_mg and predicted >= cfg.stop_prediction_mg:
            return self._stop(
                ContinuousTaperStage.SETTLE,
                raw_mass_mg,
                filtered,
                predicted,
                "predicted mass reaches target after tail and balance delay",
            )

        control_mass = max(raw_mass_mg, filtered)
        if control_mass < cfg.coarse_end_mg:
            stage = ContinuousTaperStage.COARSE
            action = cfg.coarse
            reason = "below coarse end; keep the stable initial feed"
        elif control_mass < cfg.target_mass_mg:
            stage = ContinuousTaperStage.TAPER
            action = cfg.taper
            reason = "last 80 mg; switch once to fixed low-energy continuous tail feed"
        else:
            stage = ContinuousTaperStage.PRECISION
            action = cfg.precision
            reason = "target range reached but stop guards have not voted yet"

        command = "KEEP"
        if (
            current_window_position_units is not None
            and current_frequency_hz is not None
            and current_duty_permyriad is not None
            and (
                current_window_position_units != action.window_position_units
                or current_frequency_hz != action.frequency_hz
                or current_duty_permyriad != action.duty_permyriad
            )
        ):
            command = "UPDATE"
        return ContinuousTaperDecision(
            stage=stage,
            command=command,
            action=action,
            raw_mass_mg=raw_mass_mg,
            filtered_mass_mg=filtered,
            predicted_mass_mg=predicted,
            remaining_predicted_mg=cfg.target_mass_mg - predicted,
            reason=reason,
        )

    def _predict_mass(
        self,
        *,
        raw_mass_mg: float,
        filtered_mass_mg: float,
        rate_mg_s: float,
        acceleration_mg_s2: float,
        predicted_mass_mg: float | None,
    ) -> float:
        cfg = self.config
        positive_acceleration = max(0.0, acceleration_mg_s2)
        projected = (
            filtered_mass_mg
            + rate_mg_s * cfg.prediction_horizon_s
            + 0.5 * positive_acceleration * cfg.prediction_horizon_s * cfg.prediction_horizon_s
            + cfg.fixed_tail_mg
        )
        if predicted_mass_mg is not None:
            projected = max(projected, predicted_mass_mg)
        return max(raw_mass_mg, filtered_mass_mg, projected)

    def _stop(
        self,
        stage: ContinuousTaperStage,
        raw_mass_mg: float,
        filtered_mass_mg: float,
        predicted_mass_mg: float,
        reason: str,
    ) -> ContinuousTaperDecision:
        return ContinuousTaperDecision(
            stage=stage,
            command="STOP",
            action=None,
            raw_mass_mg=raw_mass_mg,
            filtered_mass_mg=filtered_mass_mg,
            predicted_mass_mg=predicted_mass_mg,
            remaining_predicted_mg=self.config.target_mass_mg - predicted_mass_mg,
            reason=reason,
        )


class WindowPidStage(str, Enum):
    COARSE = "coarse"
    SLOW = "slow"
    FINE = "fine"
    RECOVERY = "recovery"
    SETTLE = "settle"


@dataclass(frozen=True)
class WindowRatePoint:
    window_position_units: int
    reference_mg_per_s: float


@dataclass(frozen=True)
class PidGains:
    kp_window_per_mg_s: float = 3.0
    ki_window_per_mg: float = 0.25
    kd_window_per_mg_s2: float = 0.8


@dataclass(frozen=True)
class StageWindowConfig:
    base_window_position_units: int
    target_rate_mg_s: float
    reserve_after_action_mg: float
    max_expected_mass_mg: float
    min_duration_ms: int
    max_duration_ms: int


@dataclass(frozen=True)
class WindowPidConfig:
    target_mass_mg: float = 500.0
    allowed_overweight_mg: float = 10.0
    frequency_hz: int = 80
    duty_permyriad: int = 2000
    min_window_position_units: int = 100
    max_window_position_units: int = 650
    pid_adjust_limit_units: int = 80
    output_window_step_units: int = 10
    tail_seconds: float = 0.20
    max_tail_mg: float = 6.0
    no_flow_rate_mg_s: float = 1.0
    no_flow_timeout_s: float = 1.5
    weak_flow_rate_mg_s: float = 3.0
    pid_gains: PidGains = field(default_factory=PidGains)
    rate_points: tuple[WindowRatePoint, ...] = (
        WindowRatePoint(100, 6.0),
        WindowRatePoint(200, 27.0),
        WindowRatePoint(400, 42.0),
        WindowRatePoint(650, 55.0),
    )
    coarse_entry_margin_mg: float = 160.0
    fine_entry_margin_mg: float = 35.0
    coarse: StageWindowConfig = StageWindowConfig(
        base_window_position_units=650,
        target_rate_mg_s=50.0,
        reserve_after_action_mg=160.0,
        max_expected_mass_mg=220.0,
        min_duration_ms=1500,
        max_duration_ms=4500,
    )
    slow: StageWindowConfig = StageWindowConfig(
        base_window_position_units=200,
        target_rate_mg_s=22.0,
        reserve_after_action_mg=35.0,
        max_expected_mass_mg=80.0,
        min_duration_ms=900,
        max_duration_ms=4000,
    )
    fine: StageWindowConfig = StageWindowConfig(
        base_window_position_units=100,
        target_rate_mg_s=6.0,
        reserve_after_action_mg=0.0,
        max_expected_mass_mg=15.0,
        min_duration_ms=500,
        max_duration_ms=2500,
    )
    recovery_duration_ms: int = 600
    recovery_window_position_units: int = 650
    recovery_duty_permyriad: int = 2600


@dataclass(frozen=True)
class WindowPidState:
    integral_error: float = 0.0
    previous_error_mg_s: float = 0.0
    no_flow_seconds: float = 0.0


@dataclass(frozen=True)
class WindowPidDecision:
    stage: WindowPidStage
    action: FeedAction | None
    remaining_mg: float
    predicted_tail_mg: float
    target_rate_mg_s: float
    observed_rate_mg_s: float | None
    expected_mass_mg: float
    stop_mass_mg: float
    pid_adjust_units: int
    next_state: WindowPidState
    reason: str


class WindowPidPlanner:
    """基于前馈与 PID 校正规划窗口位置指令。"""

    def __init__(self, config: WindowPidConfig | None = None) -> None:
        self.config = config or WindowPidConfig()

    def next_decision(
        self,
        *,
        current_mass_mg: float,
        observed_rate_mg_s: float | None = None,
        state: WindowPidState | None = None,
        dt_s: float = 1.0,
    ) -> WindowPidDecision:
        if current_mass_mg < 0:
            raise ValueError("current_mass_mg must be non-negative")
        if observed_rate_mg_s is not None and observed_rate_mg_s < 0:
            raise ValueError("observed_rate_mg_s must be non-negative")
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")

        state = state or WindowPidState()
        cfg = self.config
        predicted_tail_mg = self._predict_tail(observed_rate_mg_s)
        remaining_mg = cfg.target_mass_mg - current_mass_mg - predicted_tail_mg
        hard_limit_mg = cfg.target_mass_mg + cfg.allowed_overweight_mg

        if current_mass_mg + predicted_tail_mg >= hard_limit_mg:
            return self._stop_decision(
                current_mass_mg,
                remaining_mg,
                predicted_tail_mg,
                observed_rate_mg_s,
                state,
                "hard overweight boundary reached",
            )
        if remaining_mg <= 0:
            return self._stop_decision(
                current_mass_mg,
                remaining_mg,
                predicted_tail_mg,
                observed_rate_mg_s,
                state,
                "target reached by measured mass plus predicted tail",
            )

        no_flow_seconds = self._updated_no_flow_seconds(observed_rate_mg_s, state.no_flow_seconds, dt_s)
        if (
            observed_rate_mg_s is not None
            and no_flow_seconds >= cfg.no_flow_timeout_s
            and remaining_mg > cfg.fine_entry_margin_mg
        ):
            next_state = WindowPidState(
                integral_error=0.0,
                previous_error_mg_s=0.0,
                no_flow_seconds=no_flow_seconds,
            )
            action = FeedAction(
                cfg.recovery_window_position_units,
                cfg.frequency_hz,
                cfg.recovery_duty_permyriad,
                cfg.recovery_duration_ms,
            )
            return WindowPidDecision(
                stage=WindowPidStage.RECOVERY,
                action=self._bound_action(action),
                remaining_mg=remaining_mg,
                predicted_tail_mg=predicted_tail_mg,
                target_rate_mg_s=0.0,
                observed_rate_mg_s=observed_rate_mg_s,
                expected_mass_mg=0.0,
                stop_mass_mg=current_mass_mg,
                pid_adjust_units=0,
                next_state=next_state,
                reason="flow rate stayed near zero; use a short high-window recovery pulse",
            )

        stage, stage_cfg = self._stage_for_remaining(remaining_mg)
        expected_mass_mg = self._expected_mass(remaining_mg, stage_cfg)
        pid_adjust_units, next_state = self._pid_adjust(
            target_rate_mg_s=stage_cfg.target_rate_mg_s,
            observed_rate_mg_s=observed_rate_mg_s,
            state=state,
            dt_s=dt_s,
            no_flow_seconds=no_flow_seconds,
        )
        feedforward_window = self._feedforward_window(stage_cfg)
        window = self._round_window(feedforward_window + pid_adjust_units)
        duration_ms = round(expected_mass_mg / stage_cfg.target_rate_mg_s * 1000.0)
        duration_ms = min(max(duration_ms, stage_cfg.min_duration_ms), stage_cfg.max_duration_ms)
        action = self._bound_action(
            FeedAction(
                window,
                cfg.frequency_hz,
                cfg.duty_permyriad,
                duration_ms,
            )
        )
        stop_mass_mg = min(current_mass_mg + expected_mass_mg, cfg.target_mass_mg)
        reason = "feedforward window from remaining mass"
        if observed_rate_mg_s is not None:
            reason += "; PID trims window from recent flow-rate error"
        return WindowPidDecision(
            stage=stage,
            action=action,
            remaining_mg=remaining_mg,
            predicted_tail_mg=predicted_tail_mg,
            target_rate_mg_s=stage_cfg.target_rate_mg_s,
            observed_rate_mg_s=observed_rate_mg_s,
            expected_mass_mg=expected_mass_mg,
            stop_mass_mg=stop_mass_mg,
            pid_adjust_units=pid_adjust_units,
            next_state=next_state,
            reason=reason,
        )

    def _stage_for_remaining(self, remaining_mg: float) -> tuple[WindowPidStage, StageWindowConfig]:
        cfg = self.config
        if remaining_mg > cfg.coarse_entry_margin_mg:
            return WindowPidStage.COARSE, cfg.coarse
        if remaining_mg > cfg.fine_entry_margin_mg:
            return WindowPidStage.SLOW, cfg.slow
        return WindowPidStage.FINE, cfg.fine

    def _expected_mass(self, remaining_mg: float, stage_cfg: StageWindowConfig) -> float:
        planned = remaining_mg - stage_cfg.reserve_after_action_mg
        if stage_cfg.reserve_after_action_mg == 0.0:
            planned = remaining_mg * 0.5
        return max(0.0, min(planned, stage_cfg.max_expected_mass_mg))

    def _pid_adjust(
        self,
        *,
        target_rate_mg_s: float,
        observed_rate_mg_s: float | None,
        state: WindowPidState,
        dt_s: float,
        no_flow_seconds: float,
    ) -> tuple[int, WindowPidState]:
        if observed_rate_mg_s is None:
            return 0, WindowPidState(state.integral_error, state.previous_error_mg_s, no_flow_seconds)

        cfg = self.config
        gains = cfg.pid_gains
        error = target_rate_mg_s - observed_rate_mg_s
        integral = state.integral_error + error * dt_s
        integral = min(max(integral, -120.0), 120.0)
        derivative = (error - state.previous_error_mg_s) / dt_s
        raw_adjust = (
            gains.kp_window_per_mg_s * error
            + gains.ki_window_per_mg * integral
            + gains.kd_window_per_mg_s2 * derivative
        )
        limited = min(max(raw_adjust, -cfg.pid_adjust_limit_units), cfg.pid_adjust_limit_units)
        rounded = round(limited / cfg.output_window_step_units) * cfg.output_window_step_units
        next_state = WindowPidState(
            integral_error=integral,
            previous_error_mg_s=error,
            no_flow_seconds=no_flow_seconds,
        )
        return int(rounded), next_state

    def _predict_tail(self, observed_rate_mg_s: float | None) -> float:
        if observed_rate_mg_s is None:
            return 0.0
        return min(max(observed_rate_mg_s * self.config.tail_seconds, 0.0), self.config.max_tail_mg)

    def _feedforward_window(self, stage_cfg: StageWindowConfig) -> int:
        if not self.config.rate_points:
            return stage_cfg.base_window_position_units
        closest = min(
            self.config.rate_points,
            key=lambda point: abs(point.reference_mg_per_s - stage_cfg.target_rate_mg_s),
        )
        return closest.window_position_units

    def _updated_no_flow_seconds(self, observed_rate_mg_s: float | None, previous: float, dt_s: float) -> float:
        if observed_rate_mg_s is None:
            return 0.0
        if observed_rate_mg_s < self.config.no_flow_rate_mg_s:
            return previous + dt_s
        if observed_rate_mg_s >= self.config.weak_flow_rate_mg_s:
            return 0.0
        return previous

    def _round_window(self, window_position_units: int) -> int:
        step = self.config.output_window_step_units
        return int(round(window_position_units / step) * step)

    def _bound_action(self, action: FeedAction) -> FeedAction:
        cfg = self.config
        window = min(max(action.window_position_units, cfg.min_window_position_units), cfg.max_window_position_units)
        return FeedAction(window, action.frequency_hz, action.duty_permyriad, action.duration_ms)

    def _stop_decision(
        self,
        current_mass_mg: float,
        remaining_mg: float,
        predicted_tail_mg: float,
        observed_rate_mg_s: float | None,
        state: WindowPidState,
        reason: str,
    ) -> WindowPidDecision:
        return WindowPidDecision(
            stage=WindowPidStage.SETTLE,
            action=None,
            remaining_mg=remaining_mg,
            predicted_tail_mg=predicted_tail_mg,
            target_rate_mg_s=0.0,
            observed_rate_mg_s=observed_rate_mg_s,
            expected_mass_mg=0.0,
            stop_mass_mg=current_mass_mg,
            pid_adjust_units=0,
            next_state=state,
            reason=reason,
        )


PROJECT_ROOT = Path(__file__).resolve().parents[2]
FINGERPRINTS_DIR = PROJECT_ROOT / "data" / "powder_fingerprints"
MAX_VERSIONS = 3

# --- 安全限值 ---
PROBE_SAFETY_RATE_LIMIT_MG_S = 120.0
PROBE_SAFETY_MASS_LIMIT_MG = 3000.0
PROBE_TIMEOUT_S = 30.0
PROBE_DEFAULT_DURATION_S = 10.0
PROBE_DEFAULT_FREQUENCY_HZ = 40
PROBE_DEFAULT_DUTY_PERMYRIAD = 2000
PROBE_DEFAULT_WINDOW_POSITION = 250

# --- 窗口 / 运动常量 ---
_POSITION_UNITS_PER_MM = 200
_MOTION_REGISTER_UNITS_PER_MM = 100
_POSITION_TOLERANCE_UNITS = 100
_WINDOW_MOVE_SPEED_MM_S = 1.0
_WINDOW_MOVE_TIMEOUT_S = 10.0

# --- 特征提取 ---
STEADY_EXCLUDE_FRACTION = 0.20  # 排除振动数据的前 20%
SETTLE_MASS_CHANGE_THRESHOLD_MG = 0.5  # 一个窗口内质量变化 < 0.5 mg → 已沉降
SETTLE_WINDOW_S = 1.0
SETTLE_TIMEOUT_S = 5.0

# --- 分类阈值 ---
LOW_FLOW_THRESHOLD_MG_S = 25.0
HIGH_FLOW_THRESHOLD_MG_S = 60.0
CV_STABLE_THRESHOLD_PCT = 20.0
CV_UNSTABLE_THRESHOLD_PCT = 40.0
STARTUP_STICKY_THRESHOLD_S = 4.0
STARTUP_MASS_THRESHOLD_MG = 1.0  # 启动检测的质量增重阈值


def _powder_id_from_name(name: str) -> str:
    safe = name.strip().lower().replace(" ", "-")
    suffix = uuid.uuid4().hex[:6]
    return f"{safe}-{suffix}"


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------

@dataclass
class ProbeParameters:
    frequency_hz: int = PROBE_DEFAULT_FREQUENCY_HZ
    duty_permyriad: int = PROBE_DEFAULT_DUTY_PERMYRIAD
    window_position_units: int = PROBE_DEFAULT_WINDOW_POSITION
    duration_s: float = PROBE_DEFAULT_DURATION_S


@dataclass
class FeatureVector:
    startup_delay_s: float = 0.0
    steady_rate_mg_s: float = 0.0
    rate_cv_pct: float = 0.0
    tail_mass_mg: float = 0.0
    settling_time_s: float = 0.0


@dataclass
class Classification:
    category: str = ""
    tags: list[str] = field(default_factory=list)


@dataclass
class ProbeRawSummary:
    startup_phase: list[dict] = field(default_factory=list)
    steady_phase: list[dict] = field(default_factory=list)
    settling_phase: list[dict] = field(default_factory=list)


@dataclass
class PowderFingerprint:
    powder_id: str = ""
    powder_name: str = ""
    created_at: str = ""
    probe_parameters: ProbeParameters = field(default_factory=ProbeParameters)
    feature_vector: FeatureVector = field(default_factory=FeatureVector)
    classification: Classification = field(default_factory=Classification)
    applied_profile: dict = field(default_factory=dict)
    probe_raw_summary: ProbeRawSummary = field(default_factory=ProbeRawSummary)

    def to_dict(self) -> dict:
        return {
            "powder_id": self.powder_id,
            "powder_name": self.powder_name,
            "created_at": self.created_at,
            "probe_parameters": asdict(self.probe_parameters),
            "feature_vector": asdict(self.feature_vector),
            "classification": asdict(self.classification),
            "applied_profile": self.applied_profile,
            "probe_raw_summary": asdict(self.probe_raw_summary),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "PowderFingerprint":
        applied_profile = dict(d.get("applied_profile", {}))
        raw_overrides = applied_profile.get("profile_overrides")
        if raw_overrides:
            applied_profile["profile_overrides"] = normalize_target_keys(
                raw_overrides
            )
        return cls(
            powder_id=d.get("powder_id", ""),
            powder_name=d.get("powder_name", ""),
            created_at=d.get("created_at", ""),
            probe_parameters=ProbeParameters(**d.get("probe_parameters", {})),
            feature_vector=FeatureVector(**d.get("feature_vector", {})),
            classification=Classification(**d.get("classification", {})),
            applied_profile=applied_profile,
            probe_raw_summary=ProbeRawSummary(**d.get("probe_raw_summary", {})),
        )


# ---------------------------------------------------------------------------
# 探针执行
# ---------------------------------------------------------------------------

def _read_mass(reader) -> float:
    """读取天平质量（mg），并钳位到 >= 0。"""
    raw = reader._at.read_measurement()
    if raw.mass_mg is not None:
        return max(0.0, float(raw.mass_mg))
    # 回退方案：当 mass_mg 为 None 时使用原始计数
    return max(0.0, float(raw.raw_count))


def execute_probe(
    reader,
    probe_params: ProbeParameters,
    cancel_event,
    progress_callback: Callable[[str, float], None],
) -> PowderFingerprint:
    """同步运行粉末探针测试（调用方在线程中执行此函数）。"""
    started_at = time.monotonic()
    samples: list[dict] = []
    settling_samples: list[dict] = []
    vibration_started_at: float | None = None
    vibration_started = False

    try:
        # ---- 1. 去皮 ----
        progress_callback("去皮中", 3.0)
        with reader._lock:
            reader._at.zero(authorized=True)
        tare_deadline = time.monotonic() + 5.0
        while time.monotonic() < tare_deadline:
            if cancel_event.is_set():
                raise RuntimeError("探针被取消")
            time.sleep(0.1)

        # ---- 2. 将窗口移动到探针位置 ----
        progress_callback("窗口定位", 7.0)
        with reader._lock:
            window_status = reader._la10.read_status()
        current_pos = window_status.position_units
        delta = abs(probe_params.window_position_units - current_pos)
        if delta > _POSITION_TOLERANCE_UNITS:
            extend = probe_params.window_position_units > current_pos
            dist_reg = max(1, round(delta / _POSITION_UNITS_PER_MM * _MOTION_REGISTER_UNITS_PER_MM))
            with reader._lock:
                reader._la10.enable()
                reader._la10.move_relative(
                    extend=extend,
                    distance_mm=dist_reg / _MOTION_REGISTER_UNITS_PER_MM,
                    speed_mm_s=_WINDOW_MOVE_SPEED_MM_S,
                )
            move_deadline = time.monotonic() + _WINDOW_MOVE_TIMEOUT_S
            while time.monotonic() < move_deadline:
                if cancel_event.is_set():
                    raise RuntimeError("探针被取消")
                with reader._lock:
                    cur = reader._la10.read_status()
                if not cur.is_running:
                    break
                time.sleep(0.1)
            with reader._lock:
                final = reader._la10.read_status()
            if abs(final.position_units - probe_params.window_position_units) > _POSITION_TOLERANCE_UNITS:
                raise RuntimeError(f"窗口未到位: 实际 {final.position_units}, 目标 {probe_params.window_position_units}")

        # ---- 3. 启动振动 ----
        progress_callback("振动中", 10.0)
        with reader._lock:
            reader._la10.start_vibration(
                frequency_hz=probe_params.frequency_hz,
                duty_permyriad=probe_params.duty_permyriad,
            )
            vibration_check = reader._la10.read_vibration_settings()
        if not vibration_check.enabled:
            raise RuntimeError("LA10 振动未能启动确认")
        vibration_started = True
        vibration_started_at = time.monotonic()

        # ---- 4. 振动期间采集原始质量 ----
        while time.monotonic() - vibration_started_at < probe_params.duration_s:
            elapsed = time.monotonic() - started_at
            if elapsed > PROBE_TIMEOUT_S:
                raise RuntimeError("探针超时")
            if cancel_event.is_set():
                raise RuntimeError("探针被取消")

            with reader._lock:
                mass_mg = _read_mass(reader)
            now = time.monotonic()

            samples.append({
                "elapsed_s": round(now - vibration_started_at, 3),
                "mass_mg": round(mass_mg, 2),
            })

            # 安全检查
            if mass_mg >= PROBE_SAFETY_MASS_LIMIT_MG:
                raise RuntimeError(f"质量达到 {mass_mg:.0f} mg，触发安全停机")

            # 进度：10% → 80%
            vib_pct = (time.monotonic() - vibration_started_at) / probe_params.duration_s
            progress_callback("振动中", 10.0 + 70.0 * min(vib_pct, 1.0))
            time.sleep(0.1)

        # ---- 5. 停止振动 ----
        progress_callback("停振中", 82.0)
        with reader._lock:
            reader._la10.stop_vibration()
        vibration_stopped_at = time.monotonic()
        mass_at_stop = samples[-1]["mass_mg"] if samples else 0.0

        # ---- 6. 沉降阶段 ----
        progress_callback("沉降等待", 85.0)
        settle_deadline = time.monotonic() + SETTLE_TIMEOUT_S
        settle_started_at = None

        while time.monotonic() < settle_deadline:
            if cancel_event.is_set():
                raise RuntimeError("探针被取消")

            with reader._lock:
                mass_mg = _read_mass(reader)
            now = time.monotonic()

            settling_samples.append({
                "elapsed_s": round(now - vibration_stopped_at, 3),
                "mass_mg": round(mass_mg, 2),
            })

            # 检查质量是否已稳定：最近 {SETTLE_WINDOW_S} 秒内的散布 < 阈值
            if len(settling_samples) >= 5:
                recent = [s["mass_mg"] for s in settling_samples
                          if now - (vibration_stopped_at + s["elapsed_s"]) < SETTLE_WINDOW_S]
                if recent and (max(recent) - min(recent)) < SETTLE_MASS_CHANGE_THRESHOLD_MG:
                    if settle_started_at is None:
                        settle_started_at = now
                    # 需要连续 5 个稳定点
                    recent_check = settling_samples[-5:]
                    if len(recent_check) >= 5:
                        masses = [s["mass_mg"] for s in recent_check]
                        if (max(masses) - min(masses)) < SETTLE_MASS_CHANGE_THRESHOLD_MG:
                            break
                else:
                    settle_started_at = None

            settle_pct = (time.monotonic() - vibration_stopped_at) / SETTLE_TIMEOUT_S
            progress_callback("沉降等待", 85.0 + 10.0 * min(settle_pct, 1.0))
            time.sleep(0.1)

        final_mass = settling_samples[-1]["mass_mg"] if settling_samples else mass_at_stop
        tail_mass = final_mass - mass_at_stop
        settling_time = (settle_started_at - vibration_stopped_at) if settle_started_at is not None else SETTLE_TIMEOUT_S

        # ---- 7. 从原始质量数据中提取特征 ----
        progress_callback("特征提取", 96.0)
        features = _extract_features(samples, mass_at_stop, tail_mass, settling_time, vibration_started_at)

        # ---- 8. 分类 ----
        classification = _classify(features, probe_params)

        # ---- 9. 构建指纹 ----
        progress_callback("分类完成", 99.0)
        applied_profile = powder_classification_params(classification.category, classification.tags)

        raw_summary = ProbeRawSummary(
            startup_phase=samples[:max(1, int(len(samples) * STEADY_EXCLUDE_FRACTION))],
            steady_phase=samples,
            settling_phase=settling_samples,
        )

        fingerprint = PowderFingerprint(
            powder_id="",
            powder_name="",
            created_at=datetime.now(timezone.utc).isoformat(),
            probe_parameters=probe_params,
            feature_vector=features,
            classification=classification,
            applied_profile=applied_profile,
            probe_raw_summary=raw_summary,
        )

        progress_callback("完成", 100.0)
        return fingerprint

    finally:
        if vibration_started:
            try:
                with reader._lock:
                    reader._la10.stop_vibration()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# 特征提取（基于原始质量）
# ---------------------------------------------------------------------------

def _extract_features(
    samples: list[dict],
    mass_at_stop: float,
    tail_mass: float,
    settling_time: float,
    vibration_started_at: float | None,
) -> FeatureVector:
    """从原始质量-时间数据中提取 5 个特征。"""

    if not samples or len(samples) < 3:
        return FeatureVector()

    # --- startup_delay：质量增加超过 1 mg 的首个采样点 ---
    startup_delay = 0.0
    base_mass = samples[0]["mass_mg"]
    for s in samples:
        if s["mass_mg"] - base_mass >= STARTUP_MASS_THRESHOLD_MG:
            startup_delay = s["elapsed_s"]
            break

    # --- steady_rate：对稳定段（后 80%）做线性回归 ---
    n = len(samples)
    start_idx = max(1, int(n * STEADY_EXCLUDE_FRACTION))
    steady = samples[start_idx:]
    mean_rate = 0.0
    cv_pct = 0.0

    if len(steady) >= 3:
        # 线性回归：质量 = 速率 * 时间 + 截距
        times = [s["elapsed_s"] for s in steady]
        masses = [s["mass_mg"] for s in steady]
        n_steady = len(steady)
        sum_t = sum(times)
        sum_m = sum(masses)
        sum_tt = sum(t * t for t in times)
        sum_tm = sum(t * m for t, m in zip(times, masses))

        denominator = n_steady * sum_tt - sum_t * sum_t
        if denominator > 0:
            mean_rate = (n_steady * sum_tm - sum_t * sum_m) / denominator
            mean_rate = max(0.0, mean_rate)

        # --- rate_cv：基于每秒速率分段 ---
        if n_steady > 1:
            # 计算每秒速率
            seg_rates = []
            duration = times[-1] - times[0]
            if duration >= 1.0:
                seg_count = max(2, int(duration))
                seg_width = duration / seg_count
                for i in range(seg_count):
                    seg_start = times[0] + i * seg_width
                    seg_end = seg_start + seg_width
                    seg_masses = [m for t, m in zip(times, masses) if seg_start <= t < seg_end]
                    if len(seg_masses) >= 2:
                        seg_rate = (seg_masses[-1] - seg_masses[0]) / seg_width
                        seg_rates.append(max(0.0, seg_rate))
            if seg_rates and len(seg_rates) >= 2:
                seg_mean = sum(seg_rates) / len(seg_rates)
                if seg_mean > 0:
                    seg_var = sum((r - seg_mean) ** 2 for r in seg_rates) / (len(seg_rates) - 1)
                    cv_pct = (seg_var ** 0.5) / seg_mean * 100.0

    return FeatureVector(
        startup_delay_s=round(startup_delay, 3),
        steady_rate_mg_s=round(mean_rate, 2),
        rate_cv_pct=round(cv_pct, 1),
        tail_mass_mg=round(max(0.0, tail_mass), 2),
        settling_time_s=round(max(0.0, settling_time), 3),
    )


# ---------------------------------------------------------------------------
# 分类
# ---------------------------------------------------------------------------

def _classify(features: FeatureVector, probe_params: ProbeParameters) -> Classification:
    """基于特征向量的规则分类。"""
    rate = features.steady_rate_mg_s
    if probe_params.frequency_hz != PROBE_DEFAULT_FREQUENCY_HZ:
        rate = rate * (PROBE_DEFAULT_FREQUENCY_HZ / probe_params.frequency_hz)

    if rate < LOW_FLOW_THRESHOLD_MG_S:
        category = "Low-flow"
    elif rate > HIGH_FLOW_THRESHOLD_MG_S:
        category = "High-flow"
    else:
        category = "Medium-flow"

    tags: list[str] = []
    if features.rate_cv_pct < CV_STABLE_THRESHOLD_PCT:
        tags.append("stable")
    if features.rate_cv_pct > CV_UNSTABLE_THRESHOLD_PCT:
        tags.append("unstable")
    if features.startup_delay_s > STARTUP_STICKY_THRESHOLD_S:
        tags.append("sticky")

    return Classification(category=category, tags=tags)


# ---------------------------------------------------------------------------
# 存储（文件 I/O）
# ---------------------------------------------------------------------------

def _fingerprint_path(powder_id: str, version: int = 0) -> Path:
    FINGERPRINTS_DIR.mkdir(parents=True, exist_ok=True)
    if version == 0:
        return FINGERPRINTS_DIR / f"{powder_id}.json"
    return FINGERPRINTS_DIR / f"{powder_id}.{version}.json"


def save_fingerprint(fingerprint: PowderFingerprint) -> Path:
    path_latest = _fingerprint_path(fingerprint.powder_id, 0)
    path_v2 = _fingerprint_path(fingerprint.powder_id, 2)
    if path_v2.exists():
        path_v2.unlink()
    path_v1 = _fingerprint_path(fingerprint.powder_id, 1)
    if path_v1.exists():
        shutil.copy2(path_v1, path_v2)
    if path_latest.exists():
        shutil.copy2(path_latest, path_v1)
    path_latest.write_text(
        json.dumps(fingerprint.to_dict(), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return path_latest


def load_fingerprint(powder_id: str) -> PowderFingerprint | None:
    path = _fingerprint_path(powder_id, 0)
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return PowderFingerprint.from_dict(data)
    except Exception:
        return None


def list_fingerprints() -> list[dict]:
    FINGERPRINTS_DIR.mkdir(parents=True, exist_ok=True)
    result = []
    for path in sorted(FINGERPRINTS_DIR.glob("*.json")):
        if path.name.endswith((".1.json", ".2.json")):
            continue
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            result.append({
                "powder_id": data.get("powder_id", ""),
                "powder_name": data.get("powder_name", ""),
                "created_at": data.get("created_at", ""),
                "category": data.get("classification", {}).get("category", ""),
                "tags": data.get("classification", {}).get("tags", []),
                "steady_rate_mg_s": data.get("feature_vector", {}).get("steady_rate_mg_s", 0),
            })
        except Exception:
            continue
    return result


def delete_fingerprint(powder_id: str) -> bool:
    deleted = False
    for version in (0, 1, 2):
        path = _fingerprint_path(powder_id, version)
        if path.exists():
            path.unlink()
            deleted = True
    return deleted


def apply_fingerprint(fingerprint: PowderFingerprint) -> dict:
    return {
        "powder_id": fingerprint.powder_id,
        "powder_name": fingerprint.powder_name,
        "category": fingerprint.classification.category,
        "tags": fingerprint.classification.tags,
        "preset_id": fingerprint.applied_profile.get("preset_id", ""),
        "initial": fingerprint.applied_profile.get("initial", {}),
        "maximum_flow_rate_mg_s": fingerprint.applied_profile.get("maximum_flow_rate_mg_s", 30.0),
        "fixed_tail_mass_mg": fingerprint.applied_profile.get("fixed_tail_mass_mg", 2.0),
        "settle_confirmations_delta": fingerprint.applied_profile.get("settle_confirmations_delta", 0),
        "dead_zone_multiplier": fingerprint.applied_profile.get("dead_zone_multiplier", 1.0),
        "profile_overrides": fingerprint.applied_profile.get("profile_overrides", {}),
        "duty_step_per_update": fingerprint.applied_profile.get("duty_step_per_update", 50),
        "emergency_duty_step_per_update": fingerprint.applied_profile.get("emergency_duty_step_per_update", 200),
        "position_feedback_enabled": fingerprint.applied_profile.get("position_feedback_enabled", False),
        "force_stop_offset_mg": fingerprint.applied_profile.get("force_stop_offset_mg", 10.0),
        "stall_recovery_enabled": fingerprint.applied_profile.get("stall_recovery_enabled", False),
        "stall_recovery_observation_window_s": fingerprint.applied_profile.get("stall_recovery_observation_window_s", 3.0),
        "stall_recovery_min_gain_mg": fingerprint.applied_profile.get("stall_recovery_min_gain_mg", 2.0),
        "stall_recovery_rate_threshold_mg_s": fingerprint.applied_profile.get("stall_recovery_rate_threshold_mg_s", 1.0),
        "stall_recovery_resume_rate_mg_s": fingerprint.applied_profile.get("stall_recovery_resume_rate_mg_s", 2.0),
        "stall_recovery_retry_s": fingerprint.applied_profile.get("stall_recovery_retry_s", 3.0),
        "stall_recovery_window_step_units": fingerprint.applied_profile.get("stall_recovery_window_step_units", 25),
        "stall_recovery_max_window_position_units": fingerprint.applied_profile.get("stall_recovery_max_window_position_units", 400),
        "stall_recovery_max_attempts": fingerprint.applied_profile.get("stall_recovery_max_attempts", 2),
        "stall_recovery_disable_remaining_mg": fingerprint.applied_profile.get("stall_recovery_disable_remaining_mg", 50.0),
    }


# --- 安全限值（按单步计，相对于该步自身起始质量） ---
SAFETY_RATE_LIMIT_MG_S = 120.0
SAFETY_MASS_LIMIT_MG = 3000.0
SAFETY_STEP_MASS_CAP_MG = 1500.0  # 若单步出粉达到该量则提前一步停止

# --- 目标速率带（搜索的核心目标） ---
TARGET_RATE_RANGE_MG_S = (10.0, 30.0)

# --- 搜索起点 + 导向参数 ---
START_FREQUENCY_HZ = 40
START_DUTY_PERMYRIAD = 2000
START_WINDOW_POSITION = 250

STARTUP_LADDER_HZ = [40, 55, 70, 80]
STARTUP_STEP_DURATION_S = 4.0
APPROACH_STEP_DURATION_S = 6.0
VERIFY_STEP_DURATION_S = 8.0
VERIFY_REPEAT_COUNT = 2
EXPLORE_FREQ_OFFSET_HZ = 10

# 双杠杆导向步长：
#   频率负责连续性（打破内部架桥），
#   窗口负责速率，占空比是触界时的最后手段杠杆。
FREQUENCY_STEP_HZ = 10
WINDOW_STEP_UNITS = 50
DUTY_STEP_PERMYRIAD = 500
APPROACH_MAX_STEPS = 20

SEARCH_TIMEOUT_S = 180.0

# --- 稳定性判据（可依据真实扫描数据调优） ---
STABLE_RATE_CV_MAX_PCT = 30.0
STABLE_RATE_INTERRUPTION_MAX_FRACTION = 0.20
INTERRUPTION_RATE_EPSILON_MG_S = 0.5
# 在判断稳定性之前先平滑原始速率样本，使脉冲式出粉的粉末在反馈控制器
# 实际感知的尺度上被评判，而非在 100 ms 原始采样尺度上：
# 5 个样本 × 0.1 s = 0.5 s = duty_update_interval_s。
RATE_SMOOTH_WINDOW_SAMPLES = 5

# --- 硬件边界（与服务端校验保持一致） ---
MIN_FREQUENCY_HZ = 10
MAX_FREQUENCY_HZ = 80
MIN_DUTY_PERMYRIAD = 1000
MAX_DUTY_PERMYRIAD = 5000
MIN_WINDOW_UNITS = 100
MAX_WINDOW_UNITS = 750


@dataclass
class RateSearchResult:
    powder_id: str = ""
    powder_name: str = ""
    created_at: str = ""
    target_rate_range_mg_s: list[float] = field(
        default_factory=lambda: list(TARGET_RATE_RANGE_MG_S)
    )
    search_start_params: dict = field(default_factory=dict)
    search_trace: list[dict] = field(default_factory=list)
    locked_point: dict | None = None
    stable_band: dict | None = None

    def to_dict(self) -> dict:
        return {
            "powder_id": self.powder_id,
            "powder_name": self.powder_name,
            "created_at": self.created_at,
            "target_rate_range_mg_s": self.target_rate_range_mg_s,
            "search_start_params": self.search_start_params,
            "search_trace": self.search_trace,
            "locked_point": self.locked_point,
            "stable_band": self.stable_band,
        }


def _move_window_to_position(reader, window_units: int, cancel_event) -> None:
    """将加料头窗口移动到 *window_units*（绝对坐标）。"""
    with reader._lock:
        window_status = reader._la10.read_status()
    current_pos = window_status.position_units
    delta = abs(window_units - current_pos)
    if delta <= _POSITION_TOLERANCE_UNITS:
        return
    extend = window_units > current_pos
    dist_reg = max(1, round(delta / _POSITION_UNITS_PER_MM * _MOTION_REGISTER_UNITS_PER_MM))
    with reader._lock:
        reader._la10.enable()
        reader._la10.move_relative(
            extend=extend,
            distance_mm=dist_reg / _MOTION_REGISTER_UNITS_PER_MM,
            speed_mm_s=_WINDOW_MOVE_SPEED_MM_S,
        )
    if cancel_event.is_set():
        raise RuntimeError("速率搜索被取消")
    # 等待运动真正启动并停止后再读取稳定位置。在 move_relative 返回后
    # 立即读取 is_running 会得到假"已停止"（执行器尚未置位运行位），
    # 这曾导致旧循环提前退出并误读移动前的位置。
    with reader._lock:
        final = reader._la10.wait_until_stopped(
            timeout_s=_WINDOW_MOVE_TIMEOUT_S,
            poll_interval_s=0.1,
            require_running_transition=True,
        )
    if abs(final.position_units - window_units) > _POSITION_TOLERANCE_UNITS:
        raise RuntimeError(f"窗口未到位: 实际 {final.position_units}, 目标 {window_units}")


def _tare_balance(reader, cancel_event, settle_s: float = 1.0) -> None:
    """重新对天平去皮并短暂等待其稳定（每个测点执行一次）。"""
    with reader._lock:
        reader._at.zero(authorized=True)
    deadline = time.monotonic() + settle_s
    while time.monotonic() < deadline:
        if cancel_event.is_set():
            raise RuntimeError("速率搜索被取消")
        time.sleep(0.1)


def _moving_average(values: list[float], window: int) -> list[float]:
    """对 *values* 做 *window* 个样本的后向滑动平均。

    保持输入长度不变，使下游索引保持对齐。前 ``window - 1`` 个样本
    仅基于较短的历史尾部求平均。
    """
    if window <= 1 or not values:
        return list(values)
    out = []
    for i in range(len(values)):
        start = max(0, i - window + 1)
        out.append(sum(values[start:i + 1]) / (i - start + 1))
    return out


def _run_step(reader, frequency_hz: int, duty_permyriad: int,
              cancel_event, duration_s: float) -> dict:
    """以 *frequency_hz* 振动 *duration_s* 并表征流动特性。

    返回一个汇总字典（原始样本在特征计算后即丢弃）：
        frequency_hz, flow_started, startup_delay_s, peak_rate_mg_s,
        mean_rate_mg_s, steady_rate_mg_s, cv_pct, interruption_fraction
    """
    summary = {
        "frequency_hz": frequency_hz,
        "duty_permyriad": duty_permyriad,
        "flow_started": False,
        "startup_delay_s": None,
        "peak_rate_mg_s": 0.0,
        "mean_rate_mg_s": 0.0,
        "steady_rate_mg_s": 0.0,
        "cv_pct": None,
        "interruption_fraction": None,
    }

    with reader._lock:
        reader._la10.start_vibration(frequency_hz=frequency_hz, duty_permyriad=duty_permyriad)
        enabled = reader._la10.read_vibration_settings()
    if not enabled.enabled:
        raise RuntimeError(f"LA10 振动未能启动确认（{frequency_hz}Hz）")

    vib_started = time.monotonic()
    base_mass = None
    last_mass = None
    last_t = None
    startup_delay = None
    rates: list[tuple[float, float]] = []  # (已过秒数, 速率)

    try:
        while time.monotonic() - vib_started < duration_s:
            if cancel_event.is_set():
                raise RuntimeError("速率搜索被取消")
            with reader._lock:
                mass = _read_mass(reader)
            now = time.monotonic()
            elapsed = now - vib_started

            if base_mass is None:
                base_mass = mass
            if startup_delay is None and (mass - base_mass) >= STARTUP_MASS_THRESHOLD_MG:
                startup_delay = elapsed
            if last_mass is not None and (now - last_t) > 0:
                rate = (mass - last_mass) / (now - last_t)
                rates.append((elapsed, rate))
            last_mass = mass
            last_t = now

            # 单步安全检查：防范的是单步内的失控出粉。
            if base_mass is not None and (mass - base_mass) >= SAFETY_MASS_LIMIT_MG:
                raise RuntimeError(f"单步质量增益达到 {mass - base_mass:.0f} mg，触发安全停机")
            if base_mass is not None and (mass - base_mass) >= SAFETY_STEP_MASS_CAP_MG:
                break  # 出粉远超预期 —— 提前停止，保留部分数据
            time.sleep(0.1)
    finally:
        with reader._lock:
            reader._la10.stop_vibration()

    if startup_delay is not None:
        summary["flow_started"] = True
        summary["startup_delay_s"] = round(startup_delay, 3)

    if rates:
        rate_values = [r for _, r in rates]
        summary["peak_rate_mg_s"] = round(max(rate_values), 3)
        summary["mean_rate_mg_s"] = round(sum(rate_values) / len(rate_values), 3)
        steady_anchor = (startup_delay if startup_delay is not None else 0.0) + 1.0
        steady_rates = [r for t, r in rates if t >= steady_anchor]
        if steady_rates:
            smoothed = _moving_average(steady_rates, RATE_SMOOTH_WINDOW_SAMPLES)
            mean = sum(smoothed) / len(smoothed)
            summary["steady_rate_mg_s"] = round(mean, 3)
            if len(smoothed) >= 2 and mean > 0:
                variance = sum((r - mean) ** 2 for r in smoothed) / (len(smoothed) - 1)
                summary["cv_pct"] = round((variance ** 0.5) / mean * 100, 1)
            jammed = [r for r in smoothed if r <= INTERRUPTION_RATE_EPSILON_MG_S]
            summary["interruption_fraction"] = round(len(jammed) / len(smoothed), 3)

    return summary


def _is_stable(step: dict) -> bool:
    """若 *step* 位于目标速率带内且足够稳定则返回 True。"""
    lo, hi = TARGET_RATE_RANGE_MG_S
    rate = step.get("steady_rate_mg_s") or 0.0
    if not (lo <= rate <= hi):
        return False
    cv = step.get("cv_pct")
    if cv is not None and cv > STABLE_RATE_CV_MAX_PCT:
        return False
    frac = step.get("interruption_fraction")
    if frac is not None and frac > STABLE_RATE_INTERRUPTION_MAX_FRACTION:
        return False
    return True


def _steer_next(rate: float, interruption_fraction, freq: int, window: int, duty: int):
    """返回将当前测点导向目标速率带的下一组 ``(freq, window, duty, note)``；
    若该测点是稳定候选点（可直接进入验证阶段）则返回 ``None``。

    双杠杆策略 —— 频率负责连续性，窗口负责速率，占空比是频率/窗口
    触界时的最后手段杠杆：

      * 断流/架桥 → 提升能量（先频率，后占空比）；
      * 过快（连续出粉） → 收小窗口（然后占空比，再频率）；
      * 过慢（连续出粉） → 开大窗口（然后占空比，再频率）。
    """
    lo, hi = TARGET_RATE_RANGE_MG_S

    if interruption_fraction is not None and interruption_fraction > STABLE_RATE_INTERRUPTION_MAX_FRACTION:
        if freq < MAX_FREQUENCY_HZ:
            return freq + FREQUENCY_STEP_HZ, window, duty, f"升频解架桥 {freq + FREQUENCY_STEP_HZ}Hz"
        if duty < MAX_DUTY_PERMYRIAD:
            return freq, window, duty + DUTY_STEP_PERMYRIAD, f"升占空比解架桥 {duty + DUTY_STEP_PERMYRIAD}"
        raise RuntimeError("频率与占空比均到上限仍断流，粉末无法连续出粉")

    if rate > hi:
        if window > MIN_WINDOW_UNITS:
            return freq, window - WINDOW_STEP_UNITS, duty, f"关窗压流速 窗{window - WINDOW_STEP_UNITS}"
        if duty > MIN_DUTY_PERMYRIAD:
            return freq, window, duty - DUTY_STEP_PERMYRIAD, f"降占空比压流速 {duty - DUTY_STEP_PERMYRIAD}"
        if freq > MIN_FREQUENCY_HZ:
            return freq - FREQUENCY_STEP_HZ, window, duty, f"降频压流速 {freq - FREQUENCY_STEP_HZ}Hz"
        raise RuntimeError("窗口/占空比/频率均到下限仍超速")

    if rate < lo:
        if window < MAX_WINDOW_UNITS:
            return freq, window + WINDOW_STEP_UNITS, duty, f"开窗提流速 窗{window + WINDOW_STEP_UNITS}"
        if duty < MAX_DUTY_PERMYRIAD:
            return freq, window, duty + DUTY_STEP_PERMYRIAD, f"升占空比提流速 {duty + DUTY_STEP_PERMYRIAD}"
        if freq < MAX_FREQUENCY_HZ:
            return freq + FREQUENCY_STEP_HZ, window, duty, f"升频提流速 {freq + FREQUENCY_STEP_HZ}Hz"
        raise RuntimeError("窗口/占空比/频率均到上限仍低速")

    return None


def _run_point(reader, frequency_hz: int, duty_permyriad: int,
               window_units: int, cancel_event, duration_s: float) -> dict:
    """移动窗口、重新去皮、振动，并返回一个测量点。"""
    _move_window_to_position(reader, window_units, cancel_event)
    _tare_balance(reader, cancel_event)
    step = _run_step(reader, frequency_hz, duty_permyriad, cancel_event, duration_s)
    step["window_units"] = window_units
    return step


def execute_rate_search(
    reader,
    cancel_event,
    progress_callback: Callable[[str, float], None],
    target_lo: float = TARGET_RATE_RANGE_MG_S[0],
    target_hi: float = TARGET_RATE_RANGE_MG_S[1],
    start_freq_hz: int = START_FREQUENCY_HZ,
    start_duty_permyriad: int = START_DUTY_PERMYRIAD,
    start_window_units: int = START_WINDOW_POSITION,
) -> RateSearchResult:
    """同步运行目标速率搜索（调用方在线程中执行此函数）。"""
    started_at = time.monotonic()
    band = (target_lo, target_hi)
    result = RateSearchResult(
        target_rate_range_mg_s=list(band),
        search_start_params={
            "frequency_hz": start_freq_hz,
            "duty_permyriad": start_duty_permyriad,
            "window_position_units": start_window_units,
        },
    )

    freq = start_freq_hz
    duty = start_duty_permyriad
    window = start_window_units

    def _check():
        if time.monotonic() - started_at > SEARCH_TIMEOUT_S:
            raise RuntimeError("速率搜索超时")
        if cancel_event.is_set():
            raise RuntimeError("速率搜索被取消")

    def _record(phase: str, step: dict):
        result.search_trace.append({
            "phase": phase,
            "frequency_hz": step.get("frequency_hz"),
            "duty_permyriad": step.get("duty_permyriad"),
            "window_units": step.get("window_units"),
            "mean_rate_mg_s": step.get("mean_rate_mg_s"),
            "steady_rate_mg_s": step.get("steady_rate_mg_s"),
            "cv_pct": step.get("cv_pct"),
            "interruption_fraction": step.get("interruption_fraction"),
            "flow_started": step.get("flow_started"),
        })

    try:
        # ---- 0. 去皮 + 移动到起始窗口 ----
        progress_callback("去皮中", 2.0)
        _tare_balance(reader, cancel_event)
        progress_callback("窗口定位", 5.0)
        _move_window_to_position(reader, window, cancel_event)

        # ---- 1. 启动（STARTUP）：建立流动 ----
        progress_callback("启动定位", 6.0)
        step = _run_step(reader, freq, duty, cancel_event, STARTUP_STEP_DURATION_S)
        _record("startup", step)
        if not step["flow_started"]:
            # 沿频率阶梯逐级升频，直到粉末开始流动。
            for candidate in STARTUP_LADDER_HZ:
                if candidate <= freq:
                    continue
                _check()
                freq = candidate
                progress_callback(f"启动定位 {freq}Hz", 10.0 + 15.0 * (candidate / 80))
                step = _run_step(reader, freq, duty, cancel_event, STARTUP_STEP_DURATION_S)
                _record("startup", step)
                if step["flow_started"]:
                    break
            # 仍未起流：开大窗口并在高频下重试。
            if not step["flow_started"]:
                while window < MAX_WINDOW_UNITS and not step["flow_started"]:
                    _check()
                    window = min(MAX_WINDOW_UNITS, window + 100)
                    _move_window_to_position(reader, window, cancel_event)
                    step = _run_step(reader, freq, duty, cancel_event, STARTUP_STEP_DURATION_S)
                    _record("startup", step)
                if not step["flow_started"]:
                    raise RuntimeError("启动失败：最高参数下仍无流量，粉末无法起振")

        # ---- 2. 逼近（APPROACH）：双杠杆导向 ----
        # 频率负责连续性（打破内部架桥），窗口负责速率，
        # 占空比是最后手段杠杆。决策表见 _steer_next。
        progress_callback("双杠杆逼近", 30.0)
        approach_guard = 0
        while True:
            _check()
            approach_guard += 1
            if approach_guard > APPROACH_MAX_STEPS:
                raise RuntimeError("双杠杆逼近未收敛（步数耗尽）")

            rate = step["steady_rate_mg_s"] or 0.0
            frac = step.get("interruption_fraction")
            nxt = _steer_next(rate, frac, freq, window, duty)
            if nxt is None:
                break  # 连续出粉且位于带内 → 进入验证

            freq, window, duty, note = nxt
            progress_callback(note, 30.0 + 25.0 * min(approach_guard, APPROACH_MAX_STEPS) / APPROACH_MAX_STEPS)
            step = _run_point(reader, freq, duty, window, cancel_event, APPROACH_STEP_DURATION_S)
            _record("approach", step)

        # ---- 3. 验证（VERIFY）：通过重复确认稳定性 ----
        progress_callback("锁定验证", 65.0)
        verify_steps = [step]
        for i in range(VERIFY_REPEAT_COUNT):
            _check()
            progress_callback(f"锁定验证 {i + 1}/{VERIFY_REPEAT_COUNT}", 65.0 + 15.0 * (i + 1) / VERIFY_REPEAT_COUNT)
            s = _run_point(reader, freq, duty, window, cancel_event, VERIFY_STEP_DURATION_S)
            _record("verify", s)
            verify_steps.append(s)

        stable_steps = [s for s in verify_steps if _is_stable(s)]
        # 验证未能保持在速率带内 → 报告未收敛。
        if len(stable_steps) < VERIFY_REPEAT_COUNT // 2 + 1:
            progress_callback("未锁定", 90.0)
            result.stable_band = _not_converged_band(freq, window)
            return result

        rates = [s["steady_rate_mg_s"] for s in stable_steps]
        cvs = [s["cv_pct"] for s in stable_steps if s["cv_pct"] is not None]
        locked_point = {
            "frequency_hz": freq,
            "duty_permyriad": duty,
            "window_units": window,
            "mean_rate_mg_s": round(sum(rates) / len(rates), 3),
            "cv_pct": round(sum(cvs) / len(cvs), 1) if cvs else None,
            "stable": True,
        }

        # ---- 4. 探索（EXPLORE）：在邻近频率处界定速率带边界 ----
        progress_callback("邻域确认", 85.0)
        neighbours = [locked_point]
        for offset in (EXPLORE_FREQ_OFFSET_HZ, -EXPLORE_FREQ_OFFSET_HZ):
            nf = freq + offset
            if not (MIN_FREQUENCY_HZ <= nf <= MAX_FREQUENCY_HZ):
                continue
            _check()
            progress_callback(f"邻域确认 {nf}Hz", 88.0)
            s = _run_point(reader, nf, duty, window, cancel_event, VERIFY_STEP_DURATION_S)
            _record("explore", s)
            if _is_stable(s):
                neighbours.append(s)

        stable_freqs = sorted({n["frequency_hz"] for n in neighbours})
        stable_windows = sorted({n.get("window_units", window) for n in neighbours})
        recommended = min(
            neighbours,
            key=lambda n: (
                n["cv_pct"] if n["cv_pct"] is not None else 999.0,
                abs(n["mean_rate_mg_s"] - (target_lo + target_hi) / 2),
            ),
        )
        result.stable_band = {
            "target_rate_range_mg_s": list(band),
            "found": True,
            "stable_point_count": len(neighbours),
            "frequency_hz_range": [stable_freqs[0], stable_freqs[-1]],
            "window_units_range": [stable_windows[0], stable_windows[-1]],
            "recommended": recommended,
            "notes": (
                f"稳定 {target_lo:.0f}–{target_hi:.0f} mg/s：频率 "
                f"{stable_freqs[0]}–{stable_freqs[-1]} Hz；推荐 {recommended['frequency_hz']} Hz / "
                f"窗 {recommended.get('window_units', window)}（CV {recommended['cv_pct']}%）"
            ),
        }
        result.locked_point = locked_point

        progress_callback("完成", 100.0)
        return result

    finally:
        try:
            with reader._lock:
                reader._la10.stop_vibration()
        except Exception:
            pass


def _not_converged_band(freq: int, window: int) -> dict:
    """为验证失败构建"未找到"的 stable_band 字典。"""
    return {
        "target_rate_range_mg_s": list(TARGET_RATE_RANGE_MG_S),
        "found": False,
        "stable_point_count": 0,
        "frequency_hz_range": None,
        "window_units_range": None,
        "recommended": None,
        "notes": (
            f"锁定验证未稳：{freq}Hz / 窗{window} 下 CV 或断流率超限，"
            f"需缩小占空比步进或更换搜索起点"
        ),
    }


def apply_rate_search(result: RateSearchResult) -> dict:
    """将搜索结果映射为供加料器使用的粉末会话字典。

    ``start_dispense`` 会读取 ``initial`` 以覆盖由预设推导的起始振动参数，
    因此搜索得到的锁定点（而非低/中/高分类结果）将成为出粉起始点。
    """
    lo, hi = result.target_rate_range_mg_s
    point = None
    if result.stable_band and result.stable_band.get("recommended"):
        point = result.stable_band["recommended"]
    elif result.locked_point:
        point = result.locked_point
    if point is None:
        return {}

    initial = {
        "frequency_hz": int(point["frequency_hz"]),
        "duty_permyriad": int(point.get("duty_permyriad", START_DUTY_PERMYRIAD)),
        "window_position_units": int(point.get("window_units", START_WINDOW_POSITION)),
    }
    mean_rate = point.get("mean_rate_mg_s") or (lo + hi) / 2
    coarse = round(max(lo, min(hi, mean_rate)), 1)
    fine = round(max(lo * 0.6, coarse * 0.6), 1)
    profile_overrides = {
        m: {"coarse_rate_mg_s": coarse, "fine_rate_mg_s": fine}
        for m in (100, 300, 500)
    }
    band = result.stable_band or {}
    return {
        "powder_id": result.powder_id,
        "powder_name": result.powder_name,
        "category": "rate-search",
        "tags": [],
        "preset_id": None,
        "initial": initial,
        "maximum_flow_rate_mg_s": float(hi * 2.0),
        "fixed_tail_mass_mg": 3.0,
        "profile_overrides": profile_overrides,
        "duty_step_per_update": 50,
        "emergency_duty_step_per_update": 200,
        "position_feedback_enabled": False,
        "force_stop_offset_mg": 10.0,
        # 携带搜索得到的稳定速率带，使仪表盘能够渲染推荐的频率/窗口
        # 范围（而不仅仅是单一的起始点）。
        "rate_search": {
            "found": bool(band.get("found")),
            "frequency_hz_range": band.get("frequency_hz_range"),
            "window_units_range": band.get("window_units_range"),
            "recommended": point,
        },
    }


# ---------------------------------------------------------------------------
# 数据模型
# ---------------------------------------------------------------------------


@dataclass
class GridDimension:
    """实验网格中的一条扫描轴。"""

    name: str  # "frequency_hz" | "duty_permyriad" | "window_position_units"
    values: list[int | float]

    def __post_init__(self) -> None:
        if not self.values:
            raise ValueError(f"dimension {self.name!r} must have at least one value")
        allowed = {"frequency_hz", "duty_permyriad", "window_position_units", "duration_s"}
        if self.name not in allowed:
            raise ValueError(
                f"unknown dimension {self.name!r}; allowed: {sorted(allowed)}"
            )


@dataclass
class GridConstraint:
    """筛选网格的硬件或安全约束。

    内置约束类型：
      - ``noise_budget``：frequency_hz × duty_permyriad ≤ max_value
    """

    type: str
    max_value: float | int

    def __post_init__(self) -> None:
        if self.type not in ("noise_budget",):
            raise ValueError(f"unknown constraint type {self.type!r}")

    def check(self, params: dict[str, int | float]) -> bool:
        if self.type == "noise_budget":
            freq = params.get("frequency_hz", 0)
            duty = params.get("duty_permyriad", 0)
            return (freq * duty) <= self.max_value
        return True  # 不可达


@dataclass
class GridSpec:
    """一次自动化实验扫描的完整规格。"""

    experiment_id: str  # 短标识，如 "starch-freq-scan"
    powder_name: str  # 人类可读，用于标注
    powder_id: str  # 对应 powder_library/{powder_id}.json
    experiment_type: str = "constant_rate"  # "constant_rate" | "dispense"
    dimensions: list[GridDimension] = field(default_factory=list)
    fixed_params: dict[str, int | float] = field(default_factory=dict)
    constraints: list[GridConstraint] = field(default_factory=list)
    duration_s: float = 15.0
    repeat_count: int = 1
    tare_between_sets: bool = True
    target_rate_mg_s: float = 10.0
    # dispense 类型专用参数
    target_mg: int = 0
    preset_id: str = ""

    def __post_init__(self) -> None:
        if not self.dimensions:
            raise ValueError("at least one dimension is required")
        if self.experiment_type == "constant_rate" and self.duration_s <= 0:
            raise ValueError("duration_s must be positive for constant_rate")
        if self.experiment_type == "dispense" and self.target_mg <= 0:
            raise ValueError("target_mg must be positive for dispense experiments")
        if self.repeat_count < 1:
            raise ValueError("repeat_count must be >= 1")


# ---------------------------------------------------------------------------
# 网格展开
# ---------------------------------------------------------------------------

# 硬件范围限值（与 device_control_server 的校验保持一致）。
_HARDWARE_LIMITS = {
    "frequency_hz": (10, 80),
    "duty_permyriad": (1000, 5000),
    "window_position_units": (100, 750),
    "duration_s": (1, 120),
}


def expand_grid(spec: GridSpec) -> tuple[list[dict[str, Any]], list[str]]:
    """将 *spec* 展开为扁平的参数字典列表和警告列表。

    返回 ``(valid_sets, warnings)``。未通过约束检查的组合会被丢弃，
    并附上说明原因的描述性警告。
    """
    # 构建笛卡尔积
    dim_names = [d.name for d in spec.dimensions]
    value_lists = [d.values for d in spec.dimensions]
    product = list(itertools.product(*value_lists))

    # 检查硬件限值
    warnings: list[str] = []

    def _in_range(name: str, value: int | float) -> bool:
        lo, hi = _HARDWARE_LIMITS.get(name, (-math.inf, math.inf))
        ok = lo <= value <= hi
        if not ok:
            warnings.append(
                f"Skip {name}={value}: out of hardware range [{lo}, {hi}]"
            )
        return ok

    expanded: list[dict[str, Any]] = []
    for combo in product:
        params: dict[str, int | float] = dict(zip(dim_names, combo))
        # 合并固定参数
        params.update(spec.fixed_params)

        # 硬件范围检查
        if not all(_in_range(k, v) for k, v in params.items() if k in _HARDWARE_LIMITS):
            continue

        # 约束检查
        failed = False
        for constraint in spec.constraints:
            if not constraint.check(params):
                warnings.append(
                    f"Skip {params}: violates {constraint.type} "
                    f"(max {constraint.max_value})"
                )
                failed = True
                break
        if failed:
            continue

        # 构建与现有批次端点兼容的批次组合条目
        entry: dict[str, Any] = {
            "frequency_hz": int(params["frequency_hz"]),
            "duty_permyriad": int(params["duty_permyriad"]),
            "window_position_units": int(params["window_position_units"]),
            "duration_s": spec.duration_s,
            "repeat_count": spec.repeat_count,
        }
        if spec.experiment_type == "dispense":
            entry["target_mg"] = spec.target_mg
            entry["preset_id"] = spec.preset_id or "fast-80hz-p200"
        expanded.append(entry)

    return expanded, warnings


# ---------------------------------------------------------------------------
# 结果收集 → powder_library 集成
# ---------------------------------------------------------------------------


def _load_powder_library(library_path: str) -> tuple[dict[str, dict], dict[str, str]]:
    """加载 *library_path* 下的所有粉末工作区文件。

    返回 ``(data_by_id, filename_by_id)``，使调用方无论原文件是以中文
    还是英文命名，都能写回原始文件。
    """
    data_by_id: dict[str, dict] = {}
    fname_by_id: dict[str, str] = {}
    if not os.path.isdir(library_path):
        return data_by_id, fname_by_id
    for fname in os.listdir(library_path):
        if fname.endswith(".json") and fname != "README.md":
            fpath = os.path.join(library_path, fname)
            try:
                with open(fpath, "r", encoding="utf-8") as fh:
                    data = json.load(fh)
                pid = data.get("powder_id", fname.replace(".json", ""))
                data_by_id[pid] = data
                fname_by_id[pid] = fname
            except (json.JSONDecodeError, OSError):
                continue
    return data_by_id, fname_by_id


def collect_scan_results(
    batch_result: dict,
    powder_id: str,
    library_path: str = "data/powder_library",
) -> dict:
    """根据批次结果更新粉末工作区的 ``scan_grid``。

    读取 *powder_id* 对应的既有工作区文件，使用 *batch_result* 中各批次
    组合的汇总结果新增或替换扫描网格条目，更新 ``updated_at`` 时间戳，
    并写回磁盘。

    返回更新后的工作区字典（同时已保存到磁盘）。
    """
    library, fname_by_id = _load_powder_library(library_path)
    workspace = library.get(powder_id)
    if workspace is None:
        raise FileNotFoundError(
            f"powder_id {powder_id!r} not found in {library_path}"
        )

    existing_grid = {_grid_key(e): e for e in workspace.get("scan_grid", [])}
    set_results = batch_result.get("set_results", [])

    for entry in set_results:
        summary = entry.get("summary") or {}
        freq = entry.get("frequency_hz")
        duty = entry.get("duty_permyriad")
        window = entry.get("window_position_units")
        if freq is None or duty is None:
            continue

        key = _grid_key({"frequency_hz": freq, "duty_permyriad": duty,
                          "window_position_units": window})
        mean_rate = summary.get("overall_mean_rate_mg_s")
        cv_pct = summary.get("overall_cv_pct")
        is_stable = bool(summary.get("overall_passed"))
        startup = entry.get("startup_delay_s")

        grid_entry = {
            "frequency_hz": freq,
            "duty_permyriad": duty,
            "window_position_units": window or 250,
            "status": "constant_rate",
            "mean_rate_mg_s": round(mean_rate, 1) if mean_rate is not None else None,
            "cv_pct": round(cv_pct, 1) if cv_pct is not None else None,
            "startup_delay_s": round(startup, 3) if startup is not None else None,
            "is_stable": is_stable,
            "batch_run_id": batch_result.get("run_id", ""),
            "notes": f"auto-collected from experiment {batch_result.get('run_id', '')}",
        }
        existing_grid[key] = grid_entry

    workspace["scan_grid"] = sorted(
        existing_grid.values(),
        key=lambda e: (e.get("frequency_hz", 0), e.get("duty_permyriad", 0)),
    )
    workspace["updated_at"] = datetime.now(timezone.utc).isoformat()

    # 写回原始文件（保留中文/英文命名）
    fname = fname_by_id.get(powder_id, f"{powder_id}.json")
    fpath = os.path.join(library_path, fname)
    os.makedirs(library_path, exist_ok=True)
    with open(fpath, "w", encoding="utf-8") as fh:
        json.dump(workspace, fh, ensure_ascii=False, indent=2)

    return workspace


def _grid_key(entry: dict) -> str:
    freq = entry.get("frequency_hz", 0)
    duty = entry.get("duty_permyriad", 0)
    window = entry.get("window_position_units", 0)
    return f"{freq}-{duty}-{window}"


# ---------------------------------------------------------------------------
# 4 种参考粉末的预定义实验模板
# ---------------------------------------------------------------------------

REFERENCE_POWDER_SCANS: dict[str, GridSpec] = {
    "baking_soda_low_freq": GridSpec(
        experiment_id="baking-soda-low-freq-scan",
        powder_name="小苏打",
        powder_id="baking_soda",
        experiment_type="constant_rate",
        dimensions=[
            GridDimension("frequency_hz", [15, 20, 25, 30]),
            GridDimension("duty_permyriad", [1000, 1200]),
            GridDimension("window_position_units", [250]),
        ],
        constraints=[GridConstraint("noise_budget", 90000)],
        duration_s=20.0,
        repeat_count=3,
        tare_between_sets=True,
    ),
    "bentonite_mid_freq": GridSpec(
        experiment_id="bentonite-mid-freq-scan",
        powder_name="膨润土",
        powder_id="bentonite",
        experiment_type="constant_rate",
        dimensions=[
            GridDimension("frequency_hz", [40, 65]),
            GridDimension("duty_permyriad", [2000, 2400, 2800]),
            GridDimension("window_position_units", [250]),
        ],
        constraints=[GridConstraint("noise_budget", 200000)],
        duration_s=20.0,
        repeat_count=3,
        tare_between_sets=True,
    ),
    "slaked_lime_mid_freq": GridSpec(
        experiment_id="slaked-lime-mid-freq-scan",
        powder_name="熟石灰",
        powder_id="slaked_lime",
        experiment_type="constant_rate",
        dimensions=[
            GridDimension("frequency_hz", [40, 55, 65]),
            GridDimension("duty_permyriad", [1600, 2000]),
            GridDimension("window_position_units", [250]),
        ],
        constraints=[GridConstraint("noise_budget", 90000)],
        duration_s=20.0,
        repeat_count=3,
        tare_between_sets=True,
    ),
    "corn_starch_high_freq": GridSpec(
        experiment_id="corn-starch-high-freq-scan",
        powder_name="玉米淀粉",
        powder_id="corn_starch",
        experiment_type="constant_rate",
        dimensions=[
            GridDimension("frequency_hz", [50, 55, 60, 65, 70, 75]),
            GridDimension("duty_permyriad", [2000]),
            GridDimension("window_position_units", [250]),
        ],
        constraints=[GridConstraint("noise_budget", 90000)],
        duration_s=15.0,
        repeat_count=3,
        tare_between_sets=True,
    ),
    "corn_starch_duty_sweep": GridSpec(
        experiment_id="corn-starch-duty-sweep",
        powder_name="玉米淀粉",
        powder_id="corn_starch",
        experiment_type="constant_rate",
        dimensions=[
            GridDimension("frequency_hz", [70]),
            GridDimension("duty_permyriad", [1600, 2400]),
            GridDimension("window_position_units", [250]),
        ],
        constraints=[GridConstraint("noise_budget", 90000)],
        duration_s=15.0,
        repeat_count=3,
        tare_between_sets=True,
    ),
}
