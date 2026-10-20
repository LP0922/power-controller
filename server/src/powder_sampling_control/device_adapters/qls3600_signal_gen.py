"""QLS3600S 任意波信号发生器驱动（粉末站振动/给料信号源）。

通讯链路（2026-10-20 实测）：
- 硬件：USB 转串口（CH340，当前 COM16），**57600 8N1**，ASCII 命令行协议；
- 写指令：``W`` + 指令字母 + 数值 + ``s``，例如 ``WB100000s``；
  设备应答 ``<指令><值>ok\\r\\n``（实测部分指令回显被截断，如 ``WB100000s``
  只回 ``B1ok``，因此**只校验结尾 ``ok``**，不比对数值）；
- 读指令：``R`` + 指令字母 + ``s``，例如 ``RBs``；
  设备应答 ``#R<指令><10 位数字>\\r\\n``，例如 ``#RB0000000000\\r\\n``；
- **设备不回读设定值**（无法查询当前频率/幅度/波形），所以本驱动在内存里
  缓存最近一次写入值（:attr:`QLS3600SignalGenerator.state`），仅作参考。

波形编号（协议只给 WA0s~WA32s，未给对照表；按说明书「波形种类」列举顺序排列，
实测 0/1/2 均被设备接受并回 ``A<n>ok``）：
    0 正弦波    1 方波    2 三角波    3 升锯齿波    4 降锯齿波
    5 半波      6 全波    7 多音频    8 洛伦磁力    9 调幅波
    10 辛克脉冲 11 白噪声 12 指数升   13 指数降    14 梯形波
    15 直流信号 16 升梯形波 17 降梯形波
    18..32 十五组用户自定义任意波（对应存储位 M1..M15）

命令表（来自《QLS3600 通讯协议》）：
    WA<n>s      波形 0..32（33 种，见上方编号表）
    WB<n*100>s  频率，0.01Hz 分辨率（1000.00Hz → WB100000s）
    WC<n*100>s  幅度（峰峰），0.01V 分辨率（8.00V → WC800s）
    WE<n*10>s   占空比，0.1%（50.0% → WE500s）
    WD<n>s      偏置，(偏置值+1000)，即 0..2000 ↔ -10.00V..+10.00V
    WF0..3s     测频闸门 0.01/0.1/1/10 s
    Wc1s/Wc0s   **输出开关（开/关）**——振动通断就靠这条
    Wd1s/We1s   保存/调出参数（M1..M15）
    Wr1s/Wr0s   脉冲输出开/关；Wg/Wh 脉宽/周期（us*100）
    WM<n>s      猝发个数（1..999999）；WN0/1s 触发模式；WO0/1s 触发控制；WP1s 手动触发

读命令：
    RBs 测量频率（raw）    RCs 计数值          RDs 正脉宽    REs 负脉宽
    RFs 压控 ADC 电压      RSs 扫频实时值
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional, Protocol


class SerialPort(Protocol):
    def read(self, size: int = 1) -> bytes: ...
    def read_until(self, expected: bytes = b"\n") -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def flush(self) -> None: ...
    def close(self) -> None: ...


class QLS3600Error(RuntimeError):
    """QLS3600 通讯/协议错误的基类。"""


class QLS3600CommunicationError(QLS3600Error):
    """串口无应答或应答不完整。"""


class QLS3600ProtocolError(QLS3600Error):
    """设备返回了不合协议的内容（缺少 ok / 帧头不对）。"""


@dataclass(frozen=True)
class QLS3600Config:
    port: str = "COM16"
    baudrate: int = 57600
    timeout_s: float = 0.5
    retries: int = 1
    # 型号量程（QLS3605S：0~5MHz，20Vpp，偏置 -10~+12V）
    max_frequency_hz: float = 5_000_000.0
    min_frequency_hz: float = 0.01
    max_amplitude_vpp: float = 20.0
    min_amplitude_vpp: float = 0.01
    min_offset_v: float = -10.0
    max_offset_v: float = 10.0
    # 测量值换算系数（设备只回原始数字，协议未明示单位）
    frequency_scale: float = 0.01        # 测量频率 raw × 0.01 Hz
    pulse_width_scale_us: float = 0.00625  # 脉宽 raw × 0.625/100 us（协议原文）

    def __post_init__(self) -> None:
        if not (0 < self.min_frequency_hz < self.max_frequency_hz):
            raise ValueError("频率量程非法")
        if not (0 < self.min_amplitude_vpp < self.max_amplitude_vpp):
            raise ValueError("幅度量程非法")
        if self.timeout_s <= 0 or self.retries < 0:
            raise ValueError("timeout/retries 非法")


@dataclass
class QLS3600State:
    """最近一次写入的参数缓存（设备不支持回读设定值）。"""
    waveform: Optional[int] = None
    frequency_hz: Optional[float] = None
    amplitude_vpp: Optional[float] = None
    duty_percent: Optional[float] = None
    offset_v: Optional[float] = None
    output_on: Optional[bool] = None

    @property
    def waveform_name(self) -> Optional[str]:
        """当前波形的中文名（未设置时为 None）。"""
        if self.waveform is None:
            return None
        return WAVEFORM_NAMES.get(self.waveform)


# 波形编号 → 名称（0..17 内置波形，18..32 用户自定义任意波）
WAVEFORM_NAMES: dict[int, str] = {
    0: "正弦波", 1: "方波", 2: "三角波", 3: "升锯齿波", 4: "降锯齿波",
    5: "半波", 6: "全波", 7: "多音频", 8: "洛伦磁力", 9: "调幅波",
    10: "辛克脉冲", 11: "白噪声", 12: "指数升", 13: "指数降", 14: "梯形波",
    15: "直流信号", 16: "升梯形波", 17: "降梯形波",
}
WAVEFORM_NAMES.update({n: f"任意波{n - 17}" for n in range(18, 33)})

# 常用波形常量
WAVEFORM_SINE = 0
WAVEFORM_SQUARE = 1
WAVEFORM_TRIANGLE = 2


class QLS3600SignalGenerator:
    """QLS3600S 命令行协议驱动。

    典型用法（振动给料）::

        gen = QLS3600SignalGenerator(QLS3600Config(port="COM16"))
        gen.connect()
        gen.set_waveform(WAVEFORM_SINE)  # 正弦波（0），给料默认波形
        gen.set_frequency(50.0)          # 50 Hz 振动
        gen.set_amplitude(3.0)           # 3 Vpp（振动强度）
        gen.output(True)                 # 开始振动
        ...
        gen.output(False)            # 停止振动
        gen.close()
    """

    # 指令字母（写）
    W_WAVEFORM = "A"
    W_FREQUENCY = "B"
    W_AMPLITUDE = "C"
    W_DUTY = "E"
    W_OFFSET = "D"
    W_OUTPUT = "c"      # 注意小写：Wc1s 输出开
    W_SAVE = "d"        # Wd1s 保存参数
    W_RECALL = "e"      # We1s 调出参数
    W_PULSE_OUTPUT = "r"  # Wr1s/Wr0s 脉冲输出
    W_PULSE_WIDTH = "g"
    W_PULSE_PERIOD = "h"

    # 指令字母（读）
    R_FREQ = "B"
    R_COUNT = "C"
    R_PULSE_POS = "D"
    R_PULSE_NEG = "E"
    R_ADC = "F"
    R_SWEEP = "S"

    def __init__(self, config: QLS3600Config | None = None,
                 serial_port: Optional[SerialPort] = None,
                 serial_factory: Optional[Callable[..., SerialPort]] = None) -> None:
        self.config = config or QLS3600Config()
        self.state = QLS3600State()
        self._serial = serial_port
        self._serial_factory = serial_factory

    # ------------------------------------------------------------------
    # 连接管理
    # ------------------------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._serial is not None and getattr(self._serial, "is_open", True)

    def connect(self) -> None:
        if self._serial is not None:
            return
        factory = self._serial_factory or self._default_serial_factory
        self._serial = factory(port=self.config.port, baudrate=self.config.baudrate,
                               bytesize=8, parity="N", stopbits=1,
                               timeout=self.config.timeout_s)

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    # ------------------------------------------------------------------
    # 参数设置（写指令）
    # ------------------------------------------------------------------

    def set_waveform(self, waveform: int) -> None:
        """设置输出波形（0..32）。

        0=正弦波、1=方波、2=三角波……（完整对照见模块文档与 WAVEFORM_NAMES）。
        """
        if not 0 <= int(waveform) <= 32:
            raise ValueError("波形序号必须在 0..32")
        self._command(f"W{self.W_WAVEFORM}{int(waveform)}s")
        self.state.waveform = int(waveform)

    def set_waveform_name(self, name: str) -> None:
        """按中文名设置波形，例如 ``set_waveform_name("正弦波")``。"""
        for index, wave_name in WAVEFORM_NAMES.items():
            if wave_name == name:
                self.set_waveform(index)
                return
        raise ValueError(f"未知波形名: {name!r}（可用: {', '.join(WAVEFORM_NAMES.values())}）")

    def set_frequency(self, frequency_hz: float) -> None:
        """设置输出频率（0.01Hz 分辨率，协议下发值为 频率×100）。"""
        if not self.config.min_frequency_hz <= frequency_hz <= self.config.max_frequency_hz:
            raise ValueError(f"频率必须在 {self.config.min_frequency_hz}..{self.config.max_frequency_hz} Hz")
        raw = int(round(frequency_hz * 100))
        self._command(f"W{self.W_FREQUENCY}{raw}s")
        self.state.frequency_hz = raw / 100.0

    def set_amplitude(self, amplitude_vpp: float) -> None:
        """设置输出幅度（峰峰值，0.01V 分辨率，协议下发值为 幅度×100）。"""
        if not self.config.min_amplitude_vpp <= amplitude_vpp <= self.config.max_amplitude_vpp:
            raise ValueError(
                f"幅度必须在 {self.config.min_amplitude_vpp}..{self.config.max_amplitude_vpp} Vpp")
        raw = int(round(amplitude_vpp * 100))
        self._command(f"W{self.W_AMPLITUDE}{raw}s")
        self.state.amplitude_vpp = raw / 100.0

    def set_duty(self, duty_percent: float) -> None:
        """设置方波占空比（0.1%..99.9%，协议下发值为 占空比×10）。

        **脉冲波就是方波 + 占空比**：设备没有独立的「脉冲波」波形编号，
        波形选 1（方波）后用本方法改占空比即可得到任意脉宽的脉冲波，
        占空比 50% 即对称方波。占空比只对波形 1 生效。
        """
        if not 0.1 <= duty_percent <= 99.9:
            raise ValueError("占空比必须在 0.1..99.9 %")
        raw = int(round(duty_percent * 10))
        self._command(f"W{self.W_DUTY}{raw}s")
        self.state.duty_percent = raw / 10.0

    def set_pulse_wave(self, duty_percent: float = 50.0) -> None:
        """切换为脉冲波：波形=方波(1) + 指定占空比（一步到位）。"""
        self.set_waveform(WAVEFORM_SQUARE)
        self.set_duty(duty_percent)

    def set_pulse_train(self, *, width_us: float | None = None,
                        period_us: float | None = None,
                        enabled: bool | None = None) -> None:
        """独立脉冲输出通道（脉冲串）：Wg 脉宽 / Wh 周期 / Wr 开关。

        与主通道波形互不影响，用于输出固定脉宽/周期的高速脉冲串。
        """
        if period_us is not None:
            self.set_pulse_period_us(period_us)
        if width_us is not None:
            self.set_pulse_width_us(width_us)
        if enabled is not None:
            self.pulse_output(enabled)

    def set_offset(self, offset_v: float) -> None:
        """设置直流偏置（-10V..+10V，协议下发值为 偏置+1000）。"""
        if not self.config.min_offset_v <= offset_v <= self.config.max_offset_v:
            raise ValueError(f"偏置必须在 {self.config.min_offset_v}..{self.config.max_offset_v} V")
        raw = int(round(offset_v + 1000))
        self._command(f"W{self.W_OFFSET}{raw}s")
        self.state.offset_v = raw - 1000

    # ------------------------------------------------------------------
    # 输出控制
    # ------------------------------------------------------------------

    def output(self, enabled: bool) -> None:
        """输出开关：``Wc1s`` 开 / ``Wc0s`` 关（振动通断）。"""
        self._command(f"W{self.W_OUTPUT}{1 if enabled else 0}s")
        self.state.output_on = bool(enabled)

    def pulse_output(self, enabled: bool) -> None:
        """脉冲输出开关：``Wr1s`` 开 / ``Wr0s`` 关。"""
        self._command(f"W{self.W_PULSE_OUTPUT}{1 if enabled else 0}s")

    def set_pulse_width_us(self, width_us: float) -> None:
        """脉冲输出脉宽（协议下发值为 us×100）。"""
        self._command(f"W{self.W_PULSE_WIDTH}{int(round(width_us * 100))}s")

    def set_pulse_period_us(self, period_us: float) -> None:
        """脉冲输出周期（协议下发值为 us×100）。"""
        self._command(f"W{self.W_PULSE_PERIOD}{int(round(period_us * 100))}s")

    def save_data(self) -> None:
        """保存当前参数到设备存储位（Wd1s）。"""
        self._command(f"W{self.W_SAVE}1s")

    def recall_data(self) -> None:
        """调出设备存储的参数（We1s）。"""
        self._command(f"W{self.W_RECALL}1s")

    # ------------------------------------------------------------------
    # 便捷组合：振动给料
    # ------------------------------------------------------------------

    def start_vibration(self, *, frequency_hz: float | None = None,
                        amplitude_vpp: float | None = None,
                        waveform: int | None = None) -> dict:
        """按需设置参数后开启输出（一步开始振动）。"""
        if waveform is not None:
            self.set_waveform(waveform)
        if frequency_hz is not None:
            self.set_frequency(frequency_hz)
        if amplitude_vpp is not None:
            self.set_amplitude(amplitude_vpp)
        self.output(True)
        return {"waveform": self.state.waveform, "frequency_hz": self.state.frequency_hz,
                "amplitude_vpp": self.state.amplitude_vpp, "output_on": True}

    def stop_vibration(self) -> dict:
        """关闭输出（停止振动）。"""
        self.output(False)
        return {"output_on": False}

    def run_for(self, seconds: float, *, waveform: int | None = None,
                frequency_hz: float | None = None,
                amplitude_vpp: float | None = None) -> dict:
        """定时振动：按需设置参数后开启输出，运行 ``seconds`` 秒后自动关闭。

        **输出关闭放在 finally 里**——中途异常/超时也一定会停，避免长时间无人值守起振。
        返回本次运行的参数与实测时长。
        """
        if seconds <= 0:
            raise ValueError("运行时长必须大于 0")
        if waveform is not None:
            self.set_waveform(waveform)
        if frequency_hz is not None:
            self.set_frequency(frequency_hz)
        if amplitude_vpp is not None:
            self.set_amplitude(amplitude_vpp)
        import time
        started = time.monotonic()
        self.output(True)
        try:
            time.sleep(seconds)
        finally:
            self.output(False)
        return {"waveform": self.state.waveform_name,
                "frequency_hz": self.state.frequency_hz,
                "amplitude_vpp": self.state.amplitude_vpp,
                "requested_s": seconds,
                "elapsed_s": round(time.monotonic() - started, 2),
                "output_on": False}

    # ------------------------------------------------------------------
    # 测量读取（读指令）
    # ------------------------------------------------------------------

    def measured_frequency_hz(self) -> float:
        """读取 Ext.In 输入信号的测量频率（raw × frequency_scale）。"""
        return self._query(self.R_FREQ) * self.config.frequency_scale

    def count_value(self) -> int:
        """读取脉冲计数值（0..4294967295）。"""
        return self._query(self.R_COUNT)

    def pulse_width_us(self, positive: bool = True) -> float:
        """读取输入信号脉宽（us）：协议原文 脉宽=(raw*0.625/100) us。"""
        raw = self._query(self.R_PULSE_POS if positive else self.R_PULSE_NEG)
        return raw * self.config.pulse_width_scale_us

    def vco_adc_voltage(self) -> int:
        """读取压控 ADC 采集值（raw）。"""
        return self._query(self.R_ADC)

    def sweep_value(self) -> int:
        """读取扫频实时值（raw）。"""
        return self._query(self.R_SWEEP)

    # ------------------------------------------------------------------
    # 底层收发
    # ------------------------------------------------------------------

    def _command(self, frame: str) -> str:
        """发送写指令并校验应答以 ``ok`` 结尾。"""
        response = self._exchange(frame, expect_prefix=False)
        if not response.endswith("ok"):
            raise QLS3600ProtocolError(f"QLS3600 写指令 {frame!r} 应答异常: {response!r}")
        return response

    def _query(self, command_letter: str) -> int:
        """发送读指令 ``R<letter>s``，解析 ``#R<letter><数字>`` 中的数值。"""
        frame = f"R{command_letter}s"
        response = self._exchange(frame, expect_prefix=True)
        prefix = f"#R{command_letter}"
        if not response.startswith(prefix):
            raise QLS3600ProtocolError(f"QLS3600 读指令 {frame!r} 应答异常: {response!r}")
        digits = "".join(ch for ch in response[len(prefix):] if ch.isdigit())
        if not digits:
            raise QLS3600ProtocolError(f"QLS3600 读指令 {frame!r} 无数值: {response!r}")
        return int(digits)

    def _exchange(self, frame: str, *, expect_prefix: bool) -> str:
        """发一帧收一行；失败重试 config.retries 次。"""
        last: Exception | None = None
        for _ in range(self.config.retries + 1):
            try:
                port = self._require_port()
                reset = getattr(port, "reset_input_buffer", None)
                if callable(reset):
                    reset()
                port.write(frame.encode("ascii"))
                port.flush()
                line = self._read_line(port)
                # 部分设备会把上一条残留应答一起吐出来，取以 # / 字母开头的最后一行有效内容。
                line = line.strip()
                if not line:
                    raise QLS3600CommunicationError(f"QLS3600 对 {frame!r} 无应答")
                if expect_prefix and not line.startswith("#"):
                    raise QLS3600ProtocolError(f"QLS3600 读应答帧头异常: {line!r}")
                return line
            except OSError as error:
                last = QLS3600CommunicationError(f"QLS3600 串口 I/O 失败: {error}")
            except QLS3600CommunicationError as error:
                last = error
        assert last is not None
        raise last

    @staticmethod
    def _read_line(port: SerialPort) -> str:
        """读到 \\n 为止（协议应答均以 \\r\\n 结尾）。"""
        data = bytearray()
        while True:
            chunk = port.read(1)
            if not chunk:
                if data:
                    break  # 超时但有半截数据：交给上层判定
                raise QLS3600CommunicationError("QLS3600 应答超时")
            data.extend(chunk)
            if chunk == b"\n":
                break
        return data.decode("ascii", errors="replace")

    def _require_port(self) -> SerialPort:
        if self._serial is None:
            self.connect()
        assert self._serial is not None
        return self._serial

    @staticmethod
    def _default_serial_factory(**kwargs: object) -> SerialPort:
        import serial
        return serial.Serial(**kwargs)
