"""因时 LA 系列微型伺服电缸驱动（RS-485 接口，LA 私有 UART 协议）。

角色：粉末加样机构中的**推杆**（原窗口角色，机构形状改变后不再是窗口）。

⚠️ 本驱动不是 `la10_modbus_rtu.py` 的替代——那是 ESP32 网关的 Modbus 版本；
LA 电缸本体使用**自有协议**（非 Modbus），实测（2026-10-20，COM10 总线）确认：

- 指令帧头 ``55 AA``，应答帧头 ``AA 55``；
- 帧格式：帧头(2B) + 帧长度(1B, =N+2) + ID(1B) + 指令号(1B) + 控制表索引(1B)
  + 数据段(NB) + 校验和(1B, 除帧头外累加和低 8 位)；
- RS-485 型 115200/8N1，出厂 ID=1，本机 ID=3；
- 目标位置范围 0..2000（数据越大伸长越长），控制表偏移 0x37；
- 当前位置在控制表偏移 0x1A（2 字节小端）。

已实测验证的控制帧：
- 使能（工作）:        55 AA 03 ID 04 00 04 chk
- 急停:               55 AA 03 ID 04 00 23 chk
- 暂停:               55 AA 03 ID 04 00 14 chk
- 清除故障:            55 AA 03 ID 04 00 1E chk
- 参数装订（写 Flash）: 55 AA 03 ID 04 00 20 chk
- 状态查询:            55 AA 03 ID 04 00 22 chk（应答 22 字节）
- 定位模式（带反馈）:   55 AA 04 ID 21 37 位置低 位置高 chk
- 定位模式（无反馈）:   55 AA 04 ID 03 37 位置低 位置高 chk
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol


class SerialPort(Protocol):
    def read(self, size: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def flush(self) -> None: ...
    def close(self) -> None: ...


class LACommunicationError(RuntimeError):
    """串口链路未返回完整应答。"""


class LAProtocolError(LACommunicationError):
    """应答不是合法的 LA 协议帧。"""


class LAFaultError(LACommunicationError):
    """电缸报告了真实故障（堵转/过温/过流/电机异常）。"""


@dataclass(frozen=True)
class LAConfig:
    port: str
    node_id: int = 3
    baudrate: int = 115200
    timeout_s: float = 0.3
    retries: int = 1
    min_position: int = 0
    max_position: int = 2000

    def __post_init__(self) -> None:
        if not 1 <= self.node_id <= 254:
            raise ValueError("LA node_id must be in 1..254 (255 is broadcast)")
        if self.timeout_s <= 0 or self.retries < 0:
            raise ValueError("timeout_s must be positive and retries non-negative")
        if not 0 <= self.min_position < self.max_position <= 2000:
            raise ValueError("LA position limits must be within 0..2000")


@dataclass(frozen=True)
class LAStatus:
    """状态查询（单控 0x22）应答解析结果。

    error 字节仅 bit0..bit3 有文档定义（堵转/过温/过流/电机异常）；
    bit4..bit7 为未文档化的内部状态位，实测数值持续漂移（运动完全正常时
    也可能在变），**不代表真实故障**——驱动只暴露低 4 位。
    另实测：bit3「电机异常」在静止且电流为 0 时会误报，电机电流超过
    30mA 后自动消失，判断健康状态时应结合运动表现。
    """

    target_position: int
    current_position: int
    temperature_c: int
    current_ma: int
    error_bits: int  # 仅低 4 位有效：bit0 堵转 / bit1 过温 / bit2 过流 / bit3 电机异常

    @property
    def stalled(self) -> bool:
        return bool(self.error_bits & 0x01)

    @property
    def over_temperature(self) -> bool:
        return bool(self.error_bits & 0x02)

    @property
    def over_current(self) -> bool:
        return bool(self.error_bits & 0x04)

    @property
    def motor_abnormal(self) -> bool:
        return bool(self.error_bits & 0x08)

    @property
    def healthy(self) -> bool:
        return self.error_bits == 0


class LAActuator:
    """因时 LA 电缸（推杆）控制：定位、急停、状态查询、参数维护。"""

    # 指令类型
    CMD_RD = 0x01            # 读控制表
    CMD_WR = 0x02            # 写控制表
    CMD_MC = 0x04            # 单控指令
    MOVE_POSITION_FEEDBACK = 0x21    # 定位模式（返回状态帧）
    MOVE_POSITION_SILENT = 0x03      # 定位模式（无应答）
    MOVE_SERVO_FEEDBACK = 0x20       # 随动模式（10~50ms 间隔连续发，返回状态帧）
    MOVE_SERVO_SILENT = 0x19         # 随动模式（无应答）
    BROADCAST_POSITION = 0xF2        # 广播定位（一帧控 N 缸不同目标）

    # 单控指令
    MC_WORK = 0x04           # 工作（使能功率输出）
    MC_BIND_PARAMS = 0x20    # 参数装订（RAM -> Flash，掉电保存）
    MC_QUERY_STATUS = 0x22   # 查询状态信息
    MC_CLEAR_FAULT = 0x1E    # 清除故障
    MC_PAUSE = 0x14          # 暂停（发位置指令即可恢复运动）
    MC_EMERGENCY_STOP = 0x23  # 急停（需先发工作指令才能再运动）

    # 控制表偏移
    IDX_CURRENT_POSITION = 0x1A   # 当前位置，2 字节，范围 -20..2020
    IDX_OVER_CURRENT = 0x20       # 过流保护，2 字节，300..1500 mA
    IDX_TARGET_POSITION = 0x37    # 目标位置，2 字节，0..2000
    IDX_OVER_TEMP = 0x62          # 过温保护，2 字节，温度*10
    IDX_RESUME_TEMP = 0x64        # 回温启动，2 字节，温度*10

    def __init__(self, config: LAConfig, serial_port: Optional[SerialPort] = None,
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
    # 运动控制
    # ------------------------------------------------------------------

    def move_to(self, target: int, *, feedback: bool = True) -> Optional[LAStatus]:
        """定位模式运动到目标位置（内部按最短时间路径规划）。

        feedback=True 时返回运动指令的状态应答帧（起始状态），
        到位判断需另行调用 :meth:`read_status` / :meth:`read_position`。
        """
        self._validate_target(target)
        frame = self._build_frame(
            self.MOVE_POSITION_FEEDBACK if feedback else self.MOVE_POSITION_SILENT,
            self.IDX_TARGET_POSITION, target.to_bytes(2, "little"))
        if not feedback:
            self._exchange(frame, expect_reply=False)
            return None
        reply = self._exchange(frame)
        return self._parse_status(reply)

    def enable(self) -> LAStatus:
        """工作（使能功率输出）。急停后需先调用本方法才能再运动。"""
        reply = self._exchange(self._single_control(self.MC_WORK))
        return self._parse_status(reply)

    def pause(self) -> LAStatus:
        """暂停（禁止功率输出；直接发位置指令即可恢复运动）。"""
        reply = self._exchange(self._single_control(self.MC_PAUSE))
        return self._parse_status(reply)

    def emergency_stop(self) -> LAStatus:
        """急停（禁止功率输出；恢复运动前必须先 :meth:`enable`）。"""
        reply = self._exchange(self._single_control(self.MC_EMERGENCY_STOP))
        return self._parse_status(reply)

    def clear_fault(self) -> LAStatus:
        """清除故障（过流/堵转/电机异常后恢复）。"""
        reply = self._exchange(self._single_control(self.MC_CLEAR_FAULT))
        return self._parse_status(reply)

    def bind_parameters(self) -> LAStatus:
        """参数装订：把当前 RAM 控制表烧写到 Flash，掉电不丢失。"""
        reply = self._exchange(self._single_control(self.MC_BIND_PARAMS))
        return self._parse_status(reply)

    # ------------------------------------------------------------------
    # 状态读取
    # ------------------------------------------------------------------

    def read_status(self) -> LAStatus:
        """查询状态（目标/当前位置、温度、电流、故障位）。"""
        reply = self._exchange(self._single_control(self.MC_QUERY_STATUS))
        return self._parse_status(reply)

    def read_position(self) -> int:
        """读控制表当前位置（-20..2020）。"""
        reply = self._exchange(self._build_frame(
            self.CMD_RD, self.IDX_CURRENT_POSITION, b"\x02"))
        if len(reply) < 8:
            raise LAProtocolError("LA position reply too short")
        return reply[6] | (reply[7] << 8)

    def read_target(self) -> int:
        """读控制表目标位置。"""
        reply = self._exchange(self._build_frame(
            self.CMD_RD, self.IDX_TARGET_POSITION, b"\x02"))
        if len(reply) < 8:
            raise LAProtocolError("LA target reply too short")
        return reply[6] | (reply[7] << 8)

    def wait_until_stopped(self, *, target: int | None = None, tolerance: int = 3,
                           timeout_s: float = 10.0, poll_interval_s: float = 0.2) -> int:
        """轮询直到位置进入目标容差，返回最终位置。"""
        import time
        if target is None:
            target = self.read_target()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            position = self.read_position()
            if abs(position - target) <= tolerance:
                return position
            time.sleep(poll_interval_s)
        raise LACommunicationError(f"LA did not reach target {target} within {timeout_s}s")

    # ------------------------------------------------------------------
    # 协议帧构造与收发
    # ------------------------------------------------------------------

    def _single_control(self, command: int) -> bytes:
        """单控指令帧：55 AA 03 ID 04 00 <指令> chk。"""
        return self._build_frame(self.CMD_MC, 0x00, bytes((command,)))

    def _build_frame(self, command: int, index: int, data: bytes) -> bytes:
        body = bytes((len(data) + 2, self.config.node_id, command, index)) + data
        checksum = sum(body) & 0xFF
        return b"\x55\xAA" + body + bytes((checksum,))

    def _exchange(self, request: bytes, *, expect_reply: bool = True) -> bytes:
        if expect_reply:
            last: LACommunicationError | None = None
            for _ in range(self.config.retries + 1):
                try:
                    return self._exchange_once(request)
                except LACommunicationError as error:
                    last = error
            assert last is not None
            raise last
        self._exchange_once(request, expect_reply=False)
        return b""

    def _exchange_once(self, request: bytes, *, expect_reply: bool = True) -> bytes:
        try:
            port = self._require_port()
            reset = getattr(port, "reset_input_buffer", None)
            if callable(reset):
                reset()
            port.write(request)
            port.flush()
            if not expect_reply:
                return b""
            # 应答帧头 AA 55
            header = self._read_exactly(port, 2)
            if header != b"\xAA\x55":
                raise LAProtocolError(f"LA reply header mismatch: {header.hex(' ')}")
            length = self._read_exactly(port, 1)[0]
            # 长度字节之后共 length+2 字节：ID(1) + 长度区(length) + 校验(1)。
            # 长度字段统计 CMD+索引+数据（不含 ID 与校验），故须补 ID 与校验各 1 字节。
            rest = self._read_exactly(port, length + 2)
            frame = header + bytes((length,)) + rest
            payload = frame[2:-1]
            if sum(payload) & 0xFF != frame[-1]:
                raise LAProtocolError("LA checksum mismatch")
            if payload[1] != self.config.node_id:
                raise LAProtocolError("LA reply node id mismatch")
            return frame
        except OSError as error:
            raise LACommunicationError(f"LA serial I/O failed on {self.config.port}: {error}") from error

    @staticmethod
    def _parse_status(frame: bytes) -> LAStatus:
        # 状态帧: AA 55 11 ID 04 p1 p2 目标(2) 当前(2) 温度 电流(2) 错误 力(2) 内部(4) 校验
        if len(frame) < 22:
            raise LAProtocolError(f"LA status frame too short: {frame.hex(' ')}")
        b = frame
        return LAStatus(
            target_position=b[7] | (b[8] << 8),
            current_position=b[9] | (b[10] << 8),
            temperature_c=b[11],
            current_ma=b[12] | (b[13] << 8),
            # 只保留文档定义的低 4 位；高位为未文档内部状态，忽略。
            error_bits=b[14] & 0x0F,
        )

    def _validate_target(self, target: int) -> None:
        if not isinstance(target, int) or not self.config.min_position <= target <= self.config.max_position:
            raise ValueError(
                f"LA target must be in {self.config.min_position}..{self.config.max_position}")

    @staticmethod
    def _read_exactly(port: SerialPort, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            chunk = port.read(size - len(data))
            if not chunk:
                raise LACommunicationError("LA response timed out")
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
