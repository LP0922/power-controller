"""MD30、Emm42 与 QLS5800 三设备的加粉执行器组合适配层。

设备角色（2026-10-20 确认）：
- MD30 电缸   -> 窗口位置（升降杆）；
- ZDT Emm42   -> **旋转搅拌电机**（搅拌粉体防结块，非给料通道）；
- QLS5800     -> 振动信号源，驱动振动料斗给料（给料速率主通道）。
control_channel 仅保留历史兼容：给料占空比应映射到 qls_amplitude；
emm42_speed 分支语义改为搅拌转速，不再承担给料闭环。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .md30_modbus_rtu import MD30ModbusRTU, MD30Status
from .qls5800_signal_gen import QLS5800SignalGenerator
from .zdt_emm42 import Emm42Status, ZDTEmm42


@dataclass(frozen=True)
class FeederActuatorConfig:
    """旧控制字段到新硬件的显式标定映射（映射函数由开环实验标定后注入）。"""

    control_channel: str = "qls_amplitude"
    duty_to_speed_rpm: Callable[[int], float] | None = None
    duty_to_amplitude_raw: Callable[[int], int] | None = None
    speed_raw_per_mm_s: float | None = None

    def __post_init__(self) -> None:
        if self.control_channel not in {"emm42_speed", "qls_amplitude"}:
            raise ValueError("control_channel must be emm42_speed or qls_amplitude")
        if self.speed_raw_per_mm_s is not None and self.speed_raw_per_mm_s <= 0:
            raise ValueError("speed_raw_per_mm_s must be positive")


@dataclass(frozen=True)
class FeederStatus:
    window: MD30Status
    motor: Emm42Status


class FeederActuator:
    """对上保留旧的高层方法名，内部路由到新设备。"""

    def __init__(self, md30: MD30ModbusRTU, emm42: ZDTEmm42,
                 qls5800: QLS5800SignalGenerator,
                 config: FeederActuatorConfig | None = None) -> None:
        self.md30 = md30
        self.emm42 = emm42
        self.qls5800 = qls5800
        self.config = config or FeederActuatorConfig()

    def move_to(self, position_raw: int, *, speed_raw: int | None = None) -> None:
        self.md30.move_to(position_raw, speed_raw=speed_raw)

    def move_to_mm(self, position_mm: float, *, speed_raw: int | None = None) -> None:
        self.md30.move_to_mm(position_mm, speed_raw=speed_raw)

    def move_relative(self, *, extend: bool, distance_mm: float, speed_mm_s: float,
                      check_fault: bool = True) -> None:
        """兼容旧 LA10 适配器调用方的方法。

        MD30 是绝对定位硬件：先读当前原始位置，叠加标定后的增量，
        再下发一条绝对位置指令。不做任何未标定的毫米-计数或速度换算。
        """
        if distance_mm <= 0 or self.md30.config.position_scale_mm_per_count is None:
            raise ValueError("MD30 position_scale_mm_per_count is required for relative motion")
        status = self.md30.read_status()
        if check_fault and status.error_code:
            raise RuntimeError(f"MD30 reports error 0x{status.error_code:02X}")
        delta = round(distance_mm / self.md30.config.position_scale_mm_per_count)
        target = status.position_raw + (delta if extend else -delta)
        speed_raw = None
        if self.config.speed_raw_per_mm_s is not None:
            speed_raw = round(speed_mm_s * self.config.speed_raw_per_mm_s)
        self.md30.move_to(target, speed_raw=speed_raw)

    def read_status(self) -> FeederStatus:
        return FeederStatus(self.md30.read_status(), self.emm42.read_status())

    def start_vibration(self, *, frequency_hz: int, duty_permyriad: int) -> None:
        if not 0 <= duty_permyriad <= 10000:
            raise ValueError("duty_permyriad must be in 0..10000")
        self.qls5800.set_frequency(frequency_hz)
        if self.config.control_channel == "emm42_speed":
            if self.config.duty_to_speed_rpm is None:
                raise ValueError("duty_to_speed_rpm calibration is required")
            self.emm42.set_speed(self.config.duty_to_speed_rpm(duty_permyriad))
        else:
            if self.config.duty_to_amplitude_raw is None:
                raise ValueError("duty_to_amplitude_raw calibration is required")
            self.qls5800.set_amplitude(self.config.duty_to_amplitude_raw(duty_permyriad))
        self.qls5800.set_output(True)

    def update_vibration_parameters(self, *, frequency_hz: int, duty_permyriad: int) -> None:
        self.start_vibration(frequency_hz=frequency_hz, duty_permyriad=duty_permyriad)

    def update_vibration_frequency(self, frequency_hz: int) -> None:
        self.qls5800.set_frequency(frequency_hz)

    def update_vibration_duty(self, duty_permyriad: int) -> None:
        if self.config.duty_to_amplitude_raw is None:
            raise ValueError("duty_to_amplitude_raw calibration is required")
        self.qls5800.set_amplitude(self.config.duty_to_amplitude_raw(duty_permyriad))

    def stop_vibration(self) -> None:
        self.emm42.stop()
        self.qls5800.set_output(False)

    def emergency_stop(self) -> None:
        self.stop_vibration()

