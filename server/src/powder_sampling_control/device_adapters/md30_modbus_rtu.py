"""RS-485 MD30 伺服电缸的 Modbus RTU 驱动。

角色：粉末加样机构中的**升降电机**——两台同款 MD30（从站 1、2）挂在
同一条 485 总线上，**必须同时运动**，同步控制见文件末尾的
:class:`MD30DualLift`（广播写地址 0x00）。

MD30 手册中位置值以原始控制表计数（30..980）表示，机械行程为 0..30 mm。
因此 ``position_raw`` 是主要 API；毫米换算通过显式的 ``position_scale``
配置项按需开启，现场标定时无需改动协议代码。

2026-10-20 双机实测修正（COM10 总线，从站 1/2）：
- **实际可用行程为 20..980**：下限 20 是机械硬限位（最小位置寄存器
  曾被放宽到 10，实测打不到，已改写为 20）；
- FC04 的当前速度 / 负载寄存器是**带符号 int16**（回程速度读到
  65411 = -125，负载 65436 = -100），必须按有符号解析；
- 运动标志必须通过 FC04 从 0x0002 起**块读 7 个寄存器取第 6 个**
  （即 0x0008）获取；单寄存器读 0x0008 不可靠（连续返回非零）；
- 配置寄存器（0x000C 等，FC03/FC06 空间）写入**可能无回显**，
  必须以读回值验证成功，不能依赖 FC06 应答；
- 轮询间隔不要快于 100ms（40ms 高频轮询会把从站问聋）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol


class SerialPort(Protocol):
    def read(self, size: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def flush(self) -> None: ...
    def close(self) -> None: ...


class MD30CommunicationError(RuntimeError):
    """串口链路未返回完整响应。"""


class MD30ProtocolError(MD30CommunicationError):
    """响应不是合法的 Modbus RTU 帧。"""


@dataclass(frozen=True)
class MD30Config:
    port: str
    slave_address: int = 1
    baudrate: int = 115200
    timeout_s: float = 0.3
    retries: int = 1
    max_consecutive_failures: int = 3
    # 2026-10-20 实测：下限 20 为机械硬限位（寄存器允许 10 但打不到）。
    min_position_raw: int = 20
    max_position_raw: int = 980
    position_scale_mm_per_count: float | None = None
    position_offset_mm: float = 0.0

    def __post_init__(self) -> None:
        if not 1 <= self.slave_address <= 247:
            raise ValueError("slave_address must be in the Modbus unicast range 1..247")
        if self.timeout_s <= 0 or self.retries < 0 or self.max_consecutive_failures < 1:
            raise ValueError("invalid timeout, retry, or failure limit")
        if not 0 <= self.min_position_raw < self.max_position_raw <= 0xFFFF:
            raise ValueError("invalid MD30 raw position limits")
        if self.position_scale_mm_per_count is not None and self.position_scale_mm_per_count <= 0:
            raise ValueError("position_scale_mm_per_count must be positive")


@dataclass(frozen=True)
class MD30Status:
    position_raw: int
    moving: bool
    error_code: int = 0
    speed_raw: int | None = None    # 带符号 int16，单位推算 0.1 mm/s
    load_raw: int | None = None     # 带符号 int16
    voltage_raw: int | None = None  # 0.1 V（实测 120 = 12.0V）

    def position_mm(self, config: MD30Config) -> float:
        if config.position_scale_mm_per_count is None:
            raise ValueError("MD30 millimetre conversion is not configured")
        return (self.position_raw * config.position_scale_mm_per_count) + config.position_offset_mm


def modbus_crc16(payload: bytes) -> int:
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _append_crc(payload: bytes) -> bytes:
    return payload + modbus_crc16(payload).to_bytes(2, "little")


class MD30ModbusRTU:
    """通过标准 Modbus 变体读写与控制 MD30。"""

    READ_INPUT_REGISTERS = 0x04
    READ_HOLDING_REGISTERS = 0x03
    WRITE_SINGLE_REGISTER = 0x06
    POSITION_REGISTER = 0x0002
    SPEED_REGISTER = 0x0003
    CURRENT_POSITION_REGISTER = POSITION_REGISTER
    MOVING_REGISTER = 0x0007

    # 配置表（FC03/FC06 空间，2026-10-20 实测核对）
    CFG_TARGET_POSITION = 0x0002
    CFG_MAX_SPEED = 0x0003
    CFG_MIN_POSITION = 0x000C
    CFG_MAX_POSITION = 0x000D
    CFG_DEAD_ZONE = 0x000E
    CFG_MAX_TORQUE = 0x000F

    def __init__(self, config: MD30Config, serial_port: Optional[SerialPort] = None,
                 serial_factory: Optional[Callable[..., SerialPort]] = None) -> None:
        self.config = config
        self._serial = serial_port
        self._serial_factory = serial_factory
        self._consecutive_failures = 0

    @property
    def connected(self) -> bool:
        return self._serial is not None

    @property
    def healthy(self) -> bool:
        return self.connected and self._consecutive_failures < self.config.max_consecutive_failures

    def connect(self) -> None:
        if self._serial is not None:
            return
        factory = self._serial_factory or self._default_serial_factory
        self._serial = factory(port=self.config.port, baudrate=self.config.baudrate,
                                bytesize=8, parity="N", stopbits=1, timeout=self.config.timeout_s)

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def move_to(self, position_raw: int, *, speed_raw: int | None = None) -> None:
        self._validate_position(position_raw)
        if speed_raw is not None:
            if not 0 <= speed_raw <= 100:
                raise ValueError("MD30 speed_raw must be in the documented range 0..100")
            self._write_register(self.SPEED_REGISTER, speed_raw)
        # 写位置寄存器会直接触发电缸运动，因此必须先配置完速度，再下发目标位置。
        self._write_register(self.POSITION_REGISTER, position_raw)

    def move_to_mm(self, position_mm: float, *, speed_raw: int | None = None) -> None:
        if self.config.position_scale_mm_per_count is None:
            raise ValueError("configure position_scale_mm_per_count before using move_to_mm")
        raw = round((position_mm - self.config.position_offset_mm) /
                    self.config.position_scale_mm_per_count)
        self.move_to(raw, speed_raw=speed_raw)

    def read_status(self) -> MD30Status:
        # 2026-10-20 实测：FC04 从 0x0002 起块读 7 个寄存器——
        # 0x0002 当前位置、0x0003 当前速度（int16 带符号）、0x0004 负载（int16 带符号）、
        # 0x0005 电压（0.1V）、0x0008 运动标志。
        # ⚠️ 运动标志必须走块读取；单寄存器读 0x0008 实测不可靠（连续返回非零）。
        values = self.read_input_registers(self.CURRENT_POSITION_REGISTER, 7)
        position = values[0]
        speed = _to_int16(values[1])
        load = _to_int16(values[2])
        voltage = values[3]
        moving = bool(values[6] & 0x01)
        return MD30Status(position_raw=position, moving=moving,
                          speed_raw=speed, load_raw=load, voltage_raw=voltage)

    def read_input_registers(self, start_register: int, count: int) -> tuple[int, ...]:
        if not 0 <= start_register <= 0xFFFF or not 1 <= count <= 125:
            raise ValueError("invalid MD30 register range")
        payload = bytes((self.config.slave_address, self.READ_INPUT_REGISTERS)) + \
            start_register.to_bytes(2, "big") + count.to_bytes(2, "big")
        response = self._transact(_append_crc(payload), self.READ_INPUT_REGISTERS, 1 + count * 2)
        if response[0] != count * 2:
            raise MD30ProtocolError("MD30 response byte count does not match request")
        data = response[1:]
        return tuple(int.from_bytes(data[i:i + 2], "big") for i in range(0, len(data), 2))

    def read_holding_registers(self, start_register: int, count: int) -> tuple[int, ...]:
        """FC03 读保持寄存器（配置表：从站地址/波特率/目标位置/限位/死区/扭矩等）。

        注意与 FC04 输入寄存器（状态表）分属两个空间：0x0002 在 FC04 是
        当前位置（只读），在 FC03 是目标位置（读写）——2026-10-20 实测确认。
        """
        if not 0 <= start_register <= 0xFFFF or not 1 <= count <= 125:
            raise ValueError("invalid MD30 register range")
        payload = bytes((self.config.slave_address, self.READ_HOLDING_REGISTERS)) + \
            start_register.to_bytes(2, "big") + count.to_bytes(2, "big")
        response = self._transact(_append_crc(payload), self.READ_HOLDING_REGISTERS, 1 + count * 2)
        if response[0] != count * 2:
            raise MD30ProtocolError("MD30 response byte count does not match request")
        data = response[1:]
        return tuple(int.from_bytes(data[i:i + 2], "big") for i in range(0, len(data), 2))

    def write_config_register(self, register: int, value: int) -> int:
        """写配置寄存器并**以读回值验证**。

        ⚠️ 实测部分配置寄存器（如 0x000C 最小位置）的 FC06 写入没有回显，
        不能依赖写应答判断成功——本方法写后用 FC03 读回比对，返回读回值，
        不一致时抛出协议错误。
        """
        self._write_register(register, value)
        readback = self.read_holding_registers(register, 1)[0]
        if readback != value:
            raise MD30ProtocolError(
                f"MD30 config register 0x{register:04X} readback {readback} != written {value}")
        return readback

    def _write_register(self, register: int, value: int) -> None:
        if not 0 <= register <= 0xFFFF or not 0 <= value <= 0xFFFF:
            raise ValueError("MD30 register and value must be unsigned 16-bit integers")
        payload = bytes((self.config.slave_address, self.WRITE_SINGLE_REGISTER)) + \
            register.to_bytes(2, "big") + value.to_bytes(2, "big")
        response = self._transact(_append_crc(payload), self.WRITE_SINGLE_REGISTER, 4)
        if response != payload[2:6]:
            raise MD30ProtocolError("MD30 write response does not acknowledge request")

    def _transact(self, request: bytes, function: int, response_size: int) -> bytes:
        if self._consecutive_failures >= self.config.max_consecutive_failures:
            raise MD30CommunicationError("MD30 communication failure limit reached")
        last_error: MD30CommunicationError | None = None
        for _ in range(self.config.retries + 1):
            try:
                result = self._exchange(request, function, response_size)
            except MD30CommunicationError as error:
                self._consecutive_failures += 1
                last_error = error
                if self._consecutive_failures >= self.config.max_consecutive_failures:
                    raise
            else:
                self._consecutive_failures = 0
                return result
        assert last_error is not None
        raise last_error

    def _exchange(self, request: bytes, function: int, response_size: int) -> bytes:
        try:
            port = self._require_port()
            reset = getattr(port, "reset_input_buffer", None)
            if callable(reset):
                reset()
            port.write(request)
            port.flush()
            header = self._read_exactly(port, 2)
            if header[0] != self.config.slave_address or header[1] != function:
                raise MD30ProtocolError("MD30 response address or function mismatch")
            frame = header + self._read_exactly(port, response_size + 2)
            self._validate_crc(frame)
            return frame[2:-2]
        except OSError as error:
            raise MD30CommunicationError(f"MD30 serial I/O failed on {self.config.port}: {error}") from error

    @staticmethod
    def _read_exactly(port: SerialPort, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            chunk = port.read(size - len(data))
            if not chunk:
                raise MD30CommunicationError("MD30 response timed out")
            data.extend(chunk)
        return bytes(data)

    @staticmethod
    def _validate_crc(frame: bytes) -> None:
        expected = modbus_crc16(frame[:-2])
        received = int.from_bytes(frame[-2:], "little")
        if expected != received:
            raise MD30ProtocolError("MD30 CRC mismatch")

    def _require_port(self) -> SerialPort:
        if self._serial is None:
            self.connect()
        assert self._serial is not None
        return self._serial

    def _validate_position(self, position_raw: int) -> None:
        if not isinstance(position_raw, int) or not self.config.min_position_raw <= position_raw <= self.config.max_position_raw:
            raise ValueError(f"MD30 position_raw must be in {self.config.min_position_raw}..{self.config.max_position_raw}")

    @staticmethod
    def _default_serial_factory(**kwargs: object) -> SerialPort:
        try:
            import serial
        except ImportError as error:  # pragma: no cover
            raise RuntimeError("pyserial is required; install requirements.txt") from error
        return serial.Serial(**kwargs)


def _to_int16(value: int) -> int:
    """把 16 位原始寄存器值解析为带符号整数（速度/负载实测为 int16）。"""
    return value - 0x10000 if value >= 0x8000 else value


BROADCAST_ADDRESS = 0x00  # 2026-10-20 实测：Modbus 模式广播地址是 0x00（TTL 手册的 255 无效）


class MD30SyncGuardError(MD30CommunicationError):
    """双机同步保障触发：宁可不动/急停，也不允许两台不同步地运动。

    升降双机机械上刚性关联，一快一慢或单台运动会别坏机构——
    这是安全级约束，任何同步异常都必须立即停机并报警。
    """


class MD30DualLift:
    """双 MD30 升降电机同步控制层（两台同款、必须同时运动）。

    同步原理（2026-10-20 实测）：背靠背单写每条 FC06 有 ~150ms 应答回环，
    会造成一快一慢；改用**广播写地址 0x00**（一条帧同时命中两台，无应答）
    实测两台同时到位。广播局限：只能发同一目标值——升降双机同目标，正好合适。

    三重同步保障（2026-10-20 应用户安全要求实现）：
    1. 起跳前校验：双机位置偏差超过 sync_tolerance_raw 时拒绝启动（先对齐）；
    2. 起跳监督：广播下发后必须确认**两台都**离开原位，任一台未动或
       无法确认状态 → 立即急停双机并抛 :class:`MD30SyncGuardError`
       （禁止再退化为单写——那本身就制造 150ms 启动差）；
    3. 运行中看门狗：由上层轮询（控制面板）持续比对双机位置偏差，
       超阈值调用 :meth:`emergency_stop_both` 并锁存告警。

    广播帧无应答，执行结果靠后续读位置确认；本类内部持有串口锁，
    因为升降总线（COM10）上还挂着推杆 LA（LA 私有协议），共享访问需互斥。
    """

    # 同步容差（原始计数）：起跳前偏差超限拒绝启动；看门狗超限急停。
    SYNC_TOLERANCE_RAW = 5

    def __init__(self, port: str, slave_addresses: tuple[int, int] = (1, 2),
                 baudrate: int = 115200, timeout_s: float = 0.3,
                 default_speed: int = 30, serial_factory: Optional[Callable[..., SerialPort]] = None,
                 sync_tolerance_raw: int = SYNC_TOLERANCE_RAW) -> None:
        self._configs = [MD30Config(port=port, slave_address=a, baudrate=baudrate,
                                    timeout_s=timeout_s,
                                    min_position_raw=20, max_position_raw=980)
                         for a in slave_addresses]
        self._drivers = [MD30ModbusRTU(c, serial_factory=serial_factory) for c in self._configs]
        self._port = port
        self._baudrate = baudrate
        self.default_speed = default_speed
        self.sync_tolerance_raw = int(sync_tolerance_raw)
        self._serial: SerialPort | None = None
        self._serial_factory = serial_factory
        import threading
        self._lock = threading.Lock()

    @property
    def slave_addresses(self) -> tuple[int, int]:
        return tuple(d.config.slave_address for d in self._drivers)  # type: ignore[return-value]

    def connect(self) -> None:
        with self._lock:
            if self._serial is None:
                factory = self._serial_factory or MD30ModbusRTU._default_serial_factory
                self._serial = factory(port=self._port, baudrate=self._baudrate,
                                       bytesize=8, parity="N", stopbits=1, timeout=0.3)
            for driver in self._drivers:
                # 各驱动复用同一个已打开的串口对象（同一物理总线）。
                driver._serial = self._serial

    def close(self) -> None:
        with self._lock:
            for driver in self._drivers:
                driver._serial = None
            if self._serial is not None:
                self._serial.close()
                self._serial = None

    def _broadcast_write(self, register: int, value: int) -> None:
        """广播写（地址 0x00，无应答）。调用方需持有锁。"""
        if self._serial is None:
            self.connect()
        assert self._serial is not None
        payload = bytes((BROADCAST_ADDRESS, MD30ModbusRTU.WRITE_SINGLE_REGISTER)) + \
            register.to_bytes(2, "big") + value.to_bytes(2, "big")
        reset = getattr(self._serial, "reset_input_buffer", None)
        if callable(reset):
            reset()
        self._serial.write(_append_crc(payload))
        self._serial.flush()

    def _read_positions_with_retry(self, attempts: int = 3, delay_s: float = 0.15) -> list[MD30Status]:
        """读双机状态（运动中电气干扰可能造成偶发读错，重试几次再下结论）。

        调用方需持有锁。
        """
        import time
        last_error: Exception | None = None
        for _ in range(attempts):
            try:
                return [driver.read_status() for driver in self._drivers]
            except Exception as error:  # noqa: BLE001 - 重试到次数用尽
                last_error = error
                time.sleep(delay_s)
        assert last_error is not None
        raise last_error

    def _emergency_stop_both_locked(self) -> None:
        """同步急停双机（调用方需持有锁）。

        广播写目标位置 = 两机当前位置的平均值：较快的一台先减速停下，
        随后逐台把目标写回各自当前位置做精确停机。偏差已被看门狗限制在
        小范围内，广播均值停机不会造成超程。
        """
        import time
        try:
            positions = [s.position_raw for s in self._read_positions_with_retry()]
            average = int(round(sum(positions) / len(positions)))
            average = max(min(average, self._drivers[0].config.max_position_raw),
                          self._drivers[0].config.min_position_raw)
            self._broadcast_write(MD30ModbusRTU.POSITION_REGISTER, average)
        except Exception:  # noqa: BLE001 - 读不到位置就直接逐台停
            pass
        for driver in self._drivers:
            try:
                status = driver.read_status()
                driver._write_register(MD30ModbusRTU.POSITION_REGISTER, status.position_raw)
            except Exception:  # noqa: BLE001 - 急停尽力而为
                pass
        time.sleep(0.1)

    def emergency_stop_both(self) -> None:
        """同步急停双机（带锁版本，供看门狗/上层直接调用）。"""
        with self._lock:
            self._emergency_stop_both_locked()

    def move_both(self, target_raw: int, *, speed_raw: int | None = None) -> None:
        """双机同步运动到同一目标位置（广播：先速度后目标，帧间隔 50ms）。

        三重保障之 1+2（起跳前校验 + 起跳监督）：
        - 起跳前双机位置偏差超过 sync_tolerance_raw → 拒绝启动（先 align 对齐）；
        - 广播后 0.35s 内必须确认两台**都**离开原位，否则立即急停双机并抛
          :class:`MD30SyncGuardError`。绝不为"能动"而退化为单写——
          单写有 ~150ms 启动差，违反"必须同时运动"的安全约束。
        """
        import time
        for driver in self._drivers:
            driver._validate_position(target_raw)
        speed = self.default_speed if speed_raw is None else speed_raw
        if not 0 <= speed <= 100:
            raise ValueError("MD30 speed_raw must be in the documented range 0..100")
        with self._lock:
            # ---- 保障 1：起跳前同步校验 ----
            try:
                before = self._read_positions_with_retry(attempts=2)
            except Exception as error:  # noqa: BLE001
                raise MD30SyncGuardError(
                    "双机位置读取失败，无法确认同步状态，拒绝启动运动") from error
            deviation = abs(before[0].position_raw - before[1].position_raw)
            if deviation > self.sync_tolerance_raw:
                raise MD30SyncGuardError(
                    f"双机位置偏差 {deviation} 超过容差 {self.sync_tolerance_raw}，"
                    f"拒绝启动（当前位置 {[s.position_raw for s in before]}，"
                    f"请先执行对齐）")
            if before[0].moving or before[1].moving:
                raise MD30SyncGuardError("双机尚在运动中，拒绝叠加新目标（先停止再下发）")

            # ---- 广播下发（先速度后目标，50ms 间隔防吞帧）----
            self._broadcast_write(MD30ModbusRTU.SPEED_REGISTER, speed)
            # 实测：两条广播帧背靠背（~1ms）时第二条会被从站吞掉（位置帧丢失，
            # 双机不动）；间隔 ≥50ms 才可靠。20ms 仍不稳，取 50ms。
            time.sleep(0.05)
            self._broadcast_write(MD30ModbusRTU.POSITION_REGISTER, target_raw)

            # ---- 保障 2：起跳监督（0.35s 后确认两台都离开原位）----
            time.sleep(0.35)
            try:
                after = self._read_positions_with_retry(attempts=3)
            except Exception as error:  # noqa: BLE001
                # 状态读不回来 = 无法确认同步，安全原则：立即停。
                self._emergency_stop_both_locked()
                raise MD30SyncGuardError(
                    "广播下发后无法读取双机状态（无法确认同步），已急停双机") from error
            started = [abs(a.position_raw - b.position_raw) > 2 or a.moving
                       for a, b in zip(after, before)]
            if not any(started):
                # 两台都没动：广播未生效（可能断电重启后不认广播），但没造成不同步。
                raise MD30SyncGuardError(
                    "广播指令未生效（双机均未启动，已安全停留在原位）。"
                    "请检查驱动器供电/模式后重试")
            if not all(started):
                # 一动一不动：正是必须杜绝的险情 → 立即同步急停 + 报警。
                self._emergency_stop_both_locked()
                raise MD30SyncGuardError(
                    f"双机启动不同步（{started}），已同步急停双机并锁存告警，"
                    "请检查后先对齐再继续")

    def stop_both(self) -> None:
        """停止双机：把目标写为各自当前位置（广播只能同值，逐台单写）。

        仅在双机静止/基本同步时使用；运行中急停请用 :meth:`emergency_stop_both`。
        """
        with self._lock:
            for driver in self._drivers:
                status = driver.read_status()
                driver._write_register(MD30ModbusRTU.POSITION_REGISTER, status.position_raw)

    def align(self, *, speed_raw: int | None = None) -> dict:
        """对齐双机位置（仅在双机静止时调用）。

        逐台单写把两台都移到当前位置的中点，等停止后校验偏差归零。
        逐台单写在这里是安全的：静止状态下的先后运动不构成同步险情。
        """
        import time
        speed = self.default_speed if speed_raw is None else speed_raw
        with self._lock:
            statuses = self._read_positions_with_retry(attempts=2)
            if any(s.moving for s in statuses):
                raise MD30SyncGuardError("双机尚在运动中，禁止对齐（先停止）")
            midpoint = int(round(sum(s.position_raw for s in statuses) / len(statuses)))
            for driver in self._drivers:
                driver._validate_position(midpoint)
                driver.move_to(midpoint, speed_raw=speed)
        # 等待停止并校验（读操作重新拿锁）。
        self.wait_until_stopped(timeout_s=15.0, poll_interval_s=0.3)
        final = self.read_positions()
        deviation = abs(final[0].position_raw - final[1].position_raw)
        return {"midpoint": midpoint, "final_positions": [s.position_raw for s in final],
                "deviation": deviation, "aligned": deviation <= self.sync_tolerance_raw}

    def sync_deviation(self) -> int:
        """当前双机位置偏差（绝对值）。"""
        statuses = self.read_positions()
        return abs(statuses[0].position_raw - statuses[1].position_raw)

    def read_positions(self) -> list[MD30Status]:
        """读两台状态（位置/速度/负载/电压/运动标志），按从站地址顺序返回。"""
        with self._lock:
            return [driver.read_status() for driver in self._drivers]

    def wait_until_stopped(self, *, timeout_s: float = 30.0,
                           poll_interval_s: float = 0.4) -> list[MD30Status]:
        """轮询直到两台运动标志都归零，返回最终状态。"""
        import time
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            statuses = self.read_positions()
            if all(not s.moving for s in statuses):
                return statuses
            time.sleep(poll_interval_s)
        raise MD30CommunicationError("MD30 dual lift did not stop within timeout")

    def home(self, home_raw: int = 33, *, speed_raw: int | None = None) -> None:
        """双机同步回原位（默认 33，即原始记录的收拢位附近）。"""
        self.move_both(home_raw, speed_raw=speed_raw)

