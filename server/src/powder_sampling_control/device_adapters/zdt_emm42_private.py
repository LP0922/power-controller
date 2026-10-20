"""ZDT Emm42 V5.0 闭环步进驱动——私有协议（固定校验 0x6B）驱动。

角色：粉末加样机构中的**旋转搅拌电机**（搅拌粉体、防止结块搭桥）。

为什么需要这个文件：Emm42 出厂默认校验方式是私有协议 0x6B（OLED 菜单
Checksum 项），`zdt_emm42.py` 的 Modbus-RTU 模式需要手动切菜单后才能用。
实测（2026-10-20，COM9 @115200，从站 0x01）当前设备即处于私有模式，
因此提供本驱动直接可用；若日后切到 Modbus 模式，请改用 `zdt_emm42.py`。

帧格式：``地址 + 功能码 + [参数...] + 0x6B``，应答 ``地址 + 功能码 + [数据] + 0x6B``。

已实测验证的帧（地址 0x01）：
- 使能:   01 F3 AB 01 00 6B  →  01 F3 02 6B
- 断使能: 01 F3 AB 00 00 6B
- 速度模式: 01 F6 方向 速度高 速度低 加速度 同步 6B
  （方向 0=顺/1=逆；速度 0..3000 RPM 整数；加速度 0=直启、200≈357 RPM/s）
- 立即停止: 01 FE 98 00 6B  →  01 FE 02 6B
  ⚠️ 停止帧必须带 0x98 标志字节；写成 01 FE 00 6B 会返回错误 01 00 EE 6B
  且电机不停（2026-10-20 实测教训）。
- 减速停止: 速度模式写「速度=0 + 加速度」（曲线双向对称，acc=200 → ~357 RPM/s）
- 读转速: 01 35 6B → 01 35 符号 转速高 转速低 6B（6 字节）
- 读位置: 01 36 6B → 01 36 符号 位置(4B) 6B（8 字节，65536=一圈，多圈累计）
- 读状态: 01 3A 6B → 01 3A 标志 6B（bit0=使能 bit1=到位 bit2=堵转 bit3=堵转保护）

实测要点：0–3000 RPM 全速域可达；acc=0 直接启动在 1400+ RPM 会间歇性
失步（掉到几十转），高转速务必用曲线启动（acc≥200）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol


class SerialPort(Protocol):
    def read(self, size: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def flush(self) -> None: ...
    def close(self) -> None: ...


class Emm42PrivateCommunicationError(RuntimeError):
    """私有协议串口链路异常。"""


class Emm42PrivateProtocolError(Emm42PrivateCommunicationError):
    """应答不是合法的 0x6B 私有协议帧。"""


@dataclass(frozen=True)
class Emm42PrivateConfig:
    port: str
    slave_address: int = 1
    baudrate: int = 115200
    timeout_s: float = 0.3
    retries: int = 1

    def __post_init__(self) -> None:
        if not 1 <= self.slave_address <= 255:
            raise ValueError("slave_address must be in 1..255")
        if self.timeout_s <= 0 or self.retries < 0:
            raise ValueError("timeout_s must be positive and retries non-negative")


@dataclass(frozen=True)
class Emm42PrivateStatus:
    speed_rpm: int | None = None      # 带符号实时转速
    position_counts: int | None = None  # 带符号多圈累计位置（65536=一圈）
    state_flags: int | None = None    # bit0 使能 / bit1 到位 / bit2 堵转 / bit3 堵转保护

    @property
    def enabled(self) -> bool:
        return bool(self.state_flags is not None and self.state_flags & 0x01)

    @property
    def reached(self) -> bool:
        return bool(self.state_flags is not None and self.state_flags & 0x02)

    @property
    def stall(self) -> bool:
        return bool(self.state_flags is not None and self.state_flags & 0x04)

    @property
    def stall_protected(self) -> bool:
        return bool(self.state_flags is not None and self.state_flags & 0x08)


def _checksum_6b(body: bytes) -> int:
    """私有协议固定校验字节：恒为 0x6B（手册默认校验方式）。"""
    return 0x6B


class ZDTEmm42Private:
    """张大头 Emm42 V5.0 私有协议（0x6B）控制：搅拌电机速度模式与状态读取。"""

    CMD_ENABLE = 0xF3
    CMD_SPEED_MODE = 0xF6
    CMD_POSITION_MODE = 0xFD
    CMD_STOP = 0xFE
    CMD_READ_SPEED = 0x35
    CMD_READ_POSITION = 0x36
    CMD_READ_STATUS = 0x3A
    CMD_CLEAR_STALL = 0x0E
    CMD_READ_VERSION = 0x1F
    CMD_POSITION_ZERO = 0x0A

    FLAG_ENABLE_BYTE = 0xAB   # 使能/停止指令的固定标志字节
    FLAG_STOP_BYTE = 0x98     # 立即停止指令的固定标志字节（实测漏写会被拒绝）

    def __init__(self, config: Emm42PrivateConfig, serial_port: Optional[SerialPort] = None,
                 serial_factory: Optional[Callable[..., SerialPort]] = None) -> None:
        self.config = config
        self._serial = serial_port
        self._serial_factory = serial_factory

    @property
    def connected(self) -> bool:
        return self._serial is not None

    def connect(self) -> None:
        if self._serial is None:
            factory = self._serial_factory or self._default_serial_factory
            self._serial = factory(port=self.config.port, baudrate=self.config.baudrate,
                                   bytesize=8, parity="N", stopbits=1,
                                   timeout=self.config.timeout_s)

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    # ------------------------------------------------------------------
    # 运动控制（均为实测验证帧）
    # ------------------------------------------------------------------

    def enable(self, enabled: bool = True) -> None:
        """使能/断使能。"""
        frame = bytes((self.config.slave_address, self.CMD_ENABLE,
                       self.FLAG_ENABLE_BYTE, int(enabled), 0x00,
                       _checksum_6b(b"")))
        self._exchange(frame, reply_len=4)

    def set_speed(self, speed_rpm: float, *, direction: int = 0,
                  acceleration: int = 0, synchronous: bool = False) -> None:
        """速度模式连续旋转（搅拌速率控制）。

        acceleration=0 为直接启动（仅限 <1400 RPM）；更高转速必须用
        曲线启动（acc>=200，约 (256-acc)*50us/RPM 的爬升时间）。
        """
        if not 0 <= speed_rpm <= 3000:
            raise ValueError("speed_rpm must be in the documented range 0..3000")
        if direction not in (0, 1) or not 0 <= acceleration <= 255:
            raise ValueError("direction must be 0/1 and acceleration 0..255")
        rpm = round(speed_rpm)
        frame = bytes((self.config.slave_address, self.CMD_SPEED_MODE,
                       direction, (rpm >> 8) & 0xFF, rpm & 0xFF,
                       acceleration, int(synchronous), _checksum_6b(b"")))
        self._exchange(frame, reply_len=4)

    def stop_immediate(self, *, synchronous: bool = False) -> None:
        """立即停止（急停）。⚠️ 必须带 0x98 标志字节。"""
        frame = bytes((self.config.slave_address, self.CMD_STOP,
                       self.FLAG_STOP_BYTE, int(synchronous), _checksum_6b(b"")))
        self._exchange(frame, reply_len=4)

    def stop_decelerated(self, acceleration: int = 200) -> None:
        """减速停止：速度模式写「速度=0 + 加速度」，停止柔和度由 acc 决定。

        acc=200 实测约 357 RPM/s（3000→0 约 8.4s）；acc 越小停得越缓。
        """
        self.set_speed(0, acceleration=acceleration)

    def clear_stall(self) -> None:
        """解除堵转保护状态。"""
        frame = bytes((self.config.slave_address, self.CMD_CLEAR_STALL,
                       0x00, _checksum_6b(b"")))
        self._exchange(frame, reply_len=4)

    # ------------------------------------------------------------------
    # 状态读取
    # ------------------------------------------------------------------

    def read_speed(self) -> int:
        """读实时转速（RPM，带符号）。"""
        frame = bytes((self.config.slave_address, self.CMD_READ_SPEED, _checksum_6b(b"")))
        reply = self._exchange(frame, reply_len=6)
        value = int.from_bytes(reply[3:5], "big")
        return -value if reply[2] else value

    def read_position(self) -> int:
        """读实时位置（带符号，65536 counts = 一圈，多圈累计）。"""
        frame = bytes((self.config.slave_address, self.CMD_READ_POSITION, _checksum_6b(b"")))
        reply = self._exchange(frame, reply_len=8)
        value = int.from_bytes(reply[3:7], "big")
        return -value if reply[2] else value

    def read_status(self) -> Emm42PrivateStatus:
        """聚合读取转速/位置/状态标志（任一读取失败则对应字段为 None）。"""
        speed: int | None = None
        position: int | None = None
        flags: int | None = None
        try:
            speed = self.read_speed()
        except Emm42PrivateCommunicationError:
            pass
        try:
            position = self.read_position()
        except Emm42PrivateCommunicationError:
            pass
        try:
            frame = bytes((self.config.slave_address, self.CMD_READ_STATUS, _checksum_6b(b"")))
            reply = self._exchange(frame, reply_len=4)
            flags = reply[2]
        except Emm42PrivateCommunicationError:
            pass
        return Emm42PrivateStatus(speed_rpm=speed, position_counts=position, state_flags=flags)

    # ------------------------------------------------------------------
    # 收发框架
    # ------------------------------------------------------------------

    def _exchange(self, request: bytes, *, reply_len: int) -> bytes:
        last: Emm42PrivateCommunicationError | None = None
        for _ in range(self.config.retries + 1):
            try:
                return self._exchange_once(request, reply_len)
            except Emm42PrivateCommunicationError as error:
                last = error
        assert last is not None
        raise last

    def _exchange_once(self, request: bytes, reply_len: int) -> bytes:
        try:
            port = self._require_port()
            reset = getattr(port, "reset_input_buffer", None)
            if callable(reset):
                reset()
            port.write(request)
            port.flush()
            reply = self._read_exactly(port, reply_len)
            if reply[0] != self.config.slave_address:
                raise Emm42PrivateProtocolError("Emm42 private reply address mismatch")
            if reply[-1] != 0x6B:
                raise Emm42PrivateProtocolError(
                    f"Emm42 private reply tail is not 0x6B: {reply.hex(' ')}")
            return reply
        except OSError as error:
            raise Emm42PrivateCommunicationError(
                f"Emm42 private serial I/O failed on {self.config.port}: {error}") from error

    @staticmethod
    def _read_exactly(port: SerialPort, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            chunk = port.read(size - len(data))
            if not chunk:
                raise Emm42PrivateCommunicationError("Emm42 private response timed out")
            data.extend(chunk)
        return bytes(data)

    def _require_port(self) -> SerialPort:
        if self._serial is None:
            self.connect()
        assert self._serial is not None
        return self._serial

    @staticmethod
    def _default_serial_factory(**kwargs: object) -> SerialPort:
        try:
            import serial
        except ImportError as error:  # pragma: no cover
            raise RuntimeError("pyserial is required; install requirements.txt") from error
        return serial.Serial(**kwargs)
