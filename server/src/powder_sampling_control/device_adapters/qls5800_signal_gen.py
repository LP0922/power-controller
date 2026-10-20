"""QLS5800 信号发生器串口适配器。

厂商协议文档描述的帧格式为：地址 + 功能码 + 1 字节寄存器号 + 4 字节数据，
不含 CRC。寄存器号与波特率均可配置；写指令通过随后的一次寄存器读回做校验。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol


class SerialPort(Protocol):
    def read(self, size: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def flush(self) -> None: ...
    def close(self) -> None: ...


class QLS5800CommunicationError(RuntimeError): ...
class QLS5800ProtocolError(QLS5800CommunicationError): ...


@dataclass(frozen=True)
class QLS5800Config:
    port: str
    address: int = 0x09
    baudrate: int = 9600
    timeout_s: float = 0.3
    retries: int = 1
    waveform_register: int = 0x01
    frequency_register: int = 0x02
    amplitude_register: int = 0x03
    output_register: int = 0x0B

    def __post_init__(self) -> None:
        if not 0 <= self.address <= 255 or self.timeout_s <= 0 or self.retries < 0:
            raise ValueError("invalid QLS5800 address, timeout, or retries")


class QLS5800SignalGenerator:
    WRITE = 0x06
    READ = 0x03

    def __init__(self, config: QLS5800Config, serial_port: Optional[SerialPort] = None,
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

    def write_register(self, register: int, value: int, *, verify: bool = True) -> None:
        if not 0 <= register <= 0xFF or not 0 <= value <= 0xFFFFFFFF:
            raise ValueError("QLS5800 register must be one byte and value unsigned 32-bit")
        request = bytes((self.config.address, self.WRITE, register)) + value.to_bytes(4, "big")
        self._send_write(request)
        if verify and self.read_register(register) != value:
            raise QLS5800ProtocolError(f"QLS5800 register 0x{register:04X} readback mismatch")

    def read_register(self, register: int) -> int:
        if not 0 <= register <= 0xFF:
            raise ValueError("QLS5800 register must fit in one byte")
        request = bytes((self.config.address, self.READ, register))
        response = self._exchange(request, expected_size=7)
        if response[:3] != request or len(response) != 7:
            raise QLS5800ProtocolError("QLS5800 read response header mismatch")
        return int.from_bytes(response[3:], "big")

    def set_waveform(self, waveform: int) -> None:
        self.write_register(self.config.waveform_register, waveform)

    def set_frequency(self, frequency_raw: int) -> None:
        self.write_register(self.config.frequency_register, frequency_raw)

    def set_amplitude(self, amplitude_raw: int) -> None:
        self.write_register(self.config.amplitude_register, amplitude_raw)

    def set_output(self, enabled: bool) -> None:
        self.write_register(self.config.output_register, int(enabled))

    def _exchange(self, request: bytes, *, expected_size: int) -> bytes:
        last: QLS5800CommunicationError | None = None
        for _ in range(self.config.retries + 1):
            try:
                port = self._require_port()
                reset = getattr(port, "reset_input_buffer", None)
                if callable(reset):
                    reset()
                port.write(request)
                port.flush()
                return self._read_exactly(port, expected_size)
            except OSError as error:
                last = QLS5800CommunicationError(f"QLS5800 serial I/O failed: {error}")
            except QLS5800CommunicationError as error:
                last = error
        assert last is not None
        raise last

    def _send_write(self, request: bytes) -> None:
        """发送写请求；厂商协议规定只有读操作才有应答帧，写操作发完即止。"""
        last: QLS5800CommunicationError | None = None
        for _ in range(self.config.retries + 1):
            try:
                port = self._require_port()
                reset = getattr(port, "reset_input_buffer", None)
                if callable(reset):
                    reset()
                port.write(request)
                port.flush()
                return
            except OSError as error:
                last = QLS5800CommunicationError(f"QLS5800 serial I/O failed: {error}")
        assert last is not None
        raise last

    @staticmethod
    def _read_exactly(port: SerialPort, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            chunk = port.read(size - len(data))
            if not chunk:
                raise QLS5800CommunicationError("QLS5800 response timed out")
            data.extend(chunk)
        return bytes(data)

    def _require_port(self) -> SerialPort:
        if self._serial is None:
            self.connect()
        assert self._serial is not None
        return self._serial

    @staticmethod
    def _default_serial_factory(**kwargs: object) -> SerialPort:
        import serial
        return serial.Serial(**kwargs)

