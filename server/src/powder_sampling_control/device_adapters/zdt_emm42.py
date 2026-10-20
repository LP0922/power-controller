"""ZDT Emm42 V5.0 闭环步进驱动的 Modbus-RTU 驱动。

角色：粉末加样机构中的**旋转搅拌电机**（搅拌粉体、防止结块搭桥），
不是给料通道——给料由振动料斗（QLS5800 信号驱动）承担。

实测要点（2026-10-20，COM9，私有 0x6B 协议 @115200，从站地址 0x01）：
- 0–3000 RPM 全速域可达；高转速（>1300）必须用曲线启动（acc≥200），
  acc=0 直接启动在 1400 以上会间歇性失步（掉到几十转）；
- 状态标志位（0x3A）：bit0=使能、bit1=到位、bit2=堵转、bit3=堵转保护；
- 读转速（0x35）应答 = 地址+0x35+符号(1B)+转速(int16)+6B；
- 读位置（0x36）应答 = 地址+0x36+符号(1B)+位置(int32)+6B，8 字节，
  0–65535 = 一圈，多圈累计，可用 0x0A 清零。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol


class SerialPort(Protocol):
    def read(self, size: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def flush(self) -> None: ...
    def close(self) -> None: ...


class Emm42CommunicationError(RuntimeError): ...
class Emm42ProtocolError(Emm42CommunicationError): ...


@dataclass(frozen=True)
class Emm42Config:
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
class Emm42Status:
    raw_registers: tuple[int, ...]
    bus_voltage_mv: int | None = None
    phase_current_ma: int | None = None
    realtime_speed_rpm: float | None = None
    state_flags: int | None = None

    @property
    def enabled(self) -> bool:
        # 2026-10-20 实测（COM9 私有协议 0x3A）：bit0=使能、bit1=到位。
        # 待机时 0x03（使能+到位），运动中 0x01（仅使能）。
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


def modbus_crc16(payload: bytes) -> int:
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _append_crc(payload: bytes) -> bytes:
    return payload + modbus_crc16(payload).to_bytes(2, "little")


class ZDTEmm42:
    READ_INPUT_REGISTERS = 0x04
    WRITE_MULTIPLE_REGISTERS = 0x10
    SPEED_REGISTER = 0x00F6
    STOP_REGISTER = 0x00FE
    ENABLE_REGISTER = 0x00F3
    STATUS_REGISTER = 0x0043

    def __init__(self, config: Emm42Config, serial_port: Optional[SerialPort] = None,
                 serial_factory: Optional[Callable[..., SerialPort]] = None) -> None:
        self.config = config
        self._serial = serial_port
        self._serial_factory = serial_factory

    def connect(self) -> None:
        if self._serial is None:
            factory = self._serial_factory or self._default_serial_factory
            self._serial = factory(port=self.config.port, baudrate=self.config.baudrate,
                                   bytesize=8, parity="N", stopbits=1, timeout=self.config.timeout_s)

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def set_speed(self, speed_rpm: float, *, direction: int = 0, acceleration: int = 0,
                  synchronous: bool = False) -> None:
        if not 0 <= speed_rpm <= 3000:
            raise ValueError("speed_rpm must be in the documented range 0..3000")
        if direction not in (0, 1) or not 0 <= acceleration <= 255:
            raise ValueError("direction must be 0/1 and acceleration 0..255")
        # Modbus 表把「方向/加速度」与「同步标志」打包在 16 位速度值的
        # 高/低字节两侧（高字节在前，速度值居中）。
        values = [(direction << 8) | acceleration, round(speed_rpm), (int(synchronous) << 8)]
        self._write_multiple(self.SPEED_REGISTER, values)

    def enable(self, enabled: bool = True) -> None:
        self._write_multiple(self.ENABLE_REGISTER, [0xAB00 | int(enabled)])

    def stop(self, *, synchronous: bool = False) -> None:
        # Modbus 模式：FC10 写 0xFE，值高字节固定 0x98（立即停止标志），
        # 低字节为多机同步标志——与私有协议「01 FE 98 同步 6B」同构（实测验证）。
        self._write_multiple(self.STOP_REGISTER, [(0x98 << 8) | int(synchronous)])

    def read_status(self) -> Emm42Status:
        registers = self.read_input_registers(self.STATUS_REGISTER, 16)
        # 第一个状态块刻意原样返回：不同固件版本对「参数个数」字节是否
        # 包含在响应内的处理不一致，交由调用方自行解析。
        voltage = registers[1] if len(registers) > 1 else None
        current = registers[2] if len(registers) > 2 else None
        speed = registers[7] if len(registers) > 7 else None
        state = registers[15] if len(registers) > 15 else None
        return Emm42Status(registers, voltage, current, speed, state)

    def read_input_registers(self, start_register: int, count: int) -> tuple[int, ...]:
        if not 0 <= start_register <= 0xFFFF or not 1 <= count <= 125:
            raise ValueError("invalid Emm42 register range")
        payload = bytes((self.config.slave_address, self.READ_INPUT_REGISTERS)) + \
            start_register.to_bytes(2, "big") + count.to_bytes(2, "big")
        response = self._exchange(_append_crc(payload), self.READ_INPUT_REGISTERS, 1 + count * 2)
        if response[0] != count * 2:
            raise Emm42ProtocolError("Emm42 response byte count mismatch")
        return tuple(int.from_bytes(response[i:i + 2], "big") for i in range(1, len(response), 2))

    def _write_multiple(self, start_register: int, values: list[int]) -> None:
        if not values or not 0 <= start_register <= 0xFFFF:
            raise ValueError("invalid Emm42 write")
        payload = bytes((self.config.slave_address, self.WRITE_MULTIPLE_REGISTERS)) + \
            start_register.to_bytes(2, "big") + len(values).to_bytes(2, "big") + \
            bytes((len(values) * 2,)) + b"".join(v.to_bytes(2, "big") for v in values)
        response = self._exchange(_append_crc(payload), self.WRITE_MULTIPLE_REGISTERS, 4)
        if response != payload[2:6]:
            raise Emm42ProtocolError("Emm42 write response does not acknowledge request")

    def _exchange(self, request: bytes, function: int, response_size: int) -> bytes:
        last: Emm42CommunicationError | None = None
        for _ in range(self.config.retries + 1):
            try:
                port = self._require_port()
                reset = getattr(port, "reset_input_buffer", None)
                if callable(reset):
                    reset()
                port.write(request)
                port.flush()
                header = self._read_exactly(port, 2)
                if header[0] != self.config.slave_address:
                    raise Emm42ProtocolError("Emm42 response address mismatch")
                if header[1] == (function | 0x80):
                    frame = header + self._read_exactly(port, 3)
                    self._validate_crc(frame)
                    raise Emm42ProtocolError(f"Emm42 exception code 0x{frame[2]:02X}")
                if header[1] != function:
                    raise Emm42ProtocolError("Emm42 response function mismatch")
                frame = header + self._read_exactly(port, response_size + 2)
                self._validate_crc(frame)
                return frame[2:-2]
            except OSError as error:
                last = Emm42CommunicationError(f"Emm42 serial I/O failed: {error}")
            except Emm42CommunicationError as error:
                last = error
        assert last is not None
        raise last

    @staticmethod
    def _read_exactly(port: SerialPort, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            chunk = port.read(size - len(data))
            if not chunk:
                raise Emm42CommunicationError("Emm42 response timed out")
            data.extend(chunk)
        return bytes(data)

    @staticmethod
    def _validate_crc(frame: bytes) -> None:
        if modbus_crc16(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
            raise Emm42ProtocolError("Emm42 CRC mismatch")

    def _require_port(self) -> SerialPort:
        if self._serial is None:
            self.connect()
        assert self._serial is not None
        return self._serial

    @staticmethod
    def _default_serial_factory(**kwargs: object) -> SerialPort:
        import serial
        return serial.Serial(**kwargs)

