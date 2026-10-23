"""执行机构实时控制页面后端。

角色拓扑（2026-10-20 定案）：
- COM9：ZDT Emm42 V5.0 = **旋转搅拌电机**（私有 0x6B 协议 @115200，从站 0x01）；
- COM10：双 MD30 = **升降电机**（Modbus RTU 从站 1+2，必须同时运动，广播 0x00 同步）
        + 因时 LA = **推杆**（LA 私有协议，ID=3）——三设备共一条 485 总线。

用法：
    python app.py [--lift-port COM10] [--stir-port COM9] [--http-port 8765]

然后浏览器打开 http://127.0.0.1:8765/ 即可实时控制。

设计要点：
- 仅依赖标准库 + pyserial；
- COM10 上的 MD30 与 LA 共享总线，所有 COM10 事务经同一把锁串行化；
- 所有运动指令走已实测验证的帧（详见各驱动文件 docstring）；
- 急停路径不经过任何可能阻塞的读操作。
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable

from powder_sampling_control.device_adapters import (
    AT8811CConfig,
    AT8811CModbusRTU,
    LAActuator,
    LAConfig,
    MD30CommunicationError,
    MD30DualLift,
    MD30SerialError,
    MD30SyncGuardError,
    QLS3600CommunicationError,
    QLS3600Config,
    QLS3600SignalGenerator,
    ZDTEmm42Private,
    Emm42PrivateConfig,
)
from powder_sampling_control.control_panel.dispense_runner import DispenseRunner
from powder_sampling_control.control_panel.powder_feedforward import (
    CATEGORIES as POWDER_CATEGORIES,
    ROD_MAX_UNITS,
    ROD_MIN_UNITS,
    create_powder,
    delete_feedforward,
    delete_points as delete_powder_points,
    list_powders,
    list_powders_report,
    powder_detail,
)
from powder_sampling_control.control_panel.rate_calibration import (
    DEFAULT_AMPLITUDE_VPP as CALIB_AMPLITUDE_VPP,
    DEFAULT_DURATION_S as CALIB_DEFAULT_DURATION_S,
    DEFAULT_DUTY_PERCENT as CALIB_DUTY_PERCENT,
    DEFAULT_FREQUENCY_HZ as CALIB_FREQUENCY_HZ,
    DEFAULT_ROD_POINTS as CALIB_ROD_POINTS,
    DEFAULT_STIR_LEVELS as CALIB_STIR_LEVELS,
    RateCalibrationRunner,
)

# ----------------------------------------------------------------------
# 默认硬件配置（与 2026-10-20 实测一致）
# ----------------------------------------------------------------------

LIFT_PORT = "COM10"      # 双 MD30 升降 + LA 推杆 共享总线
STIR_PORT = "COM9"       # ZDT 搅拌电机
SIG_PORT = "COM16"       # QLS3600S 信号发生器（振动给料）
BAL_PORT = "COM11"       # AT8811C 天平（Modbus RTU 从站 1 @9600，FTDI FT232）
# 天平原始计数 → 质量（mg）。1 count = 1 mg 已由历史闭环加粉记录标定：
# batch_test/feedback_dispense_*100mg*.json 里 raw_count=93 且 final_mass_mg=93.0，
# samples.csv 的 mass_mg 全程为整数 mg。设备配置实读也是分度值 1、满量程 320g。
BAL_SCALE_MG_PER_COUNT = 1.0
LIFT_HOME_RAW = 33       # 升降原位（实测收拢位 32/33）
# 目标位置上/下限（软限位）：下限 20 是机械硬限位；上限由用户定为 910（不再用 980），
# 超出范围直接拒绝，不下发到电缸。
LIFT_MIN_RAW = 20
LIFT_MAX_RAW = 910
ROD_HOME = 208           # 推杆原位（实测起点）
# 升降速度固定 8（2026-10-20 实测：5 时 MD30 走不到位/一步就停，10 时六次移动
# 全部正常。8 为折中值，待上机确认；页面不可调，保证看门狗有足够轮询余量）。
LIFT_SPEED_RAW = 8

# 推杆（LA，ID=3）的轮询策略：**不再每轮都读**。
# COM10 一条总线挂着 2 台 MD30 + 1 台 LA，若每 500ms 把三个设备全读一遍，
# 总线占比偏高，实测出现"只有从站 2 偶发超时"（末端从站信号余量最小）。
# 推杆是"设置完就不动"的机构，没必要持续高频读：
#   - 刚下发过定位/停止后 ROD_ACTIVE_WINDOW_S 秒内，按轮询周期正常读（能看到它在动）；
#   - 平时只每 ROD_IDLE_INTERVAL_S 秒兜底读一次（防止状态僵死在旧值）。
ROD_ACTIVE_WINDOW_S = 8.0      # 下发动作后的高频读取窗口
ROD_IDLE_INTERVAL_S = 10.0     # 空闲时的兜底读取间隔

HERE = Path(__file__).resolve().parent

# ----------------------------------------------------------------------
# 运行日志（控制端除页面内存日志外没有历史记录，事后排障全靠这个文件）
# ----------------------------------------------------------------------
LOG_DIR = HERE / "logs"
_LOG_LOCK = threading.Lock()


def _log(event: str) -> None:
    """写一行带时间戳的运行日志：控制台 + logs/control-YYYY-MM-DD.log。"""
    line = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {event}"
    print(line, flush=True)
    try:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        with _LOG_LOCK:
            path = LOG_DIR / f"control-{time.strftime('%Y-%m-%d')}.log"
            with open(path, "a", encoding="utf-8") as handle:
                handle.write(line + "\n")
    except OSError:      # 磁盘只读/被占用：日志失败绝不能影响控制
        pass


def _log_page_event(body: dict) -> dict:
    """接收页面日志并落盘（前端内存日志刷新即丢，排障时全靠这条通道）。"""
    message = str(body.get("message", "")).strip()[:400]
    if not message:
        return {"logged": False}
    level = str(body.get("level") or "info")[:10]
    _log(f"[页面/{level}] {message}")
    return {"logged": True}


class InterlockError(RuntimeError):
    """安全互锁拒绝执行。

    与通讯故障（502）、参数错误（400）区分开：页面要原样显示"为什么不让动"，
    而不是一句 bad request。HTTP 层返回 409。
    """


class ActuatorHub:
    """三个执行器的聚合管理：串口锁 + 设备生命周期 + 统一急停。"""

    # 锁等待上限：串口操作一旦挂死，绝不能让 HTTP 线程无限排队（页面假死的根源）。
    POLL_LOCK_TIMEOUT_S = 0.3   # 状态轮询：拿不到锁就跳过本轮，回缓存数据
    CTRL_LOCK_TIMEOUT_S = 3.0   # 控制指令：最多等 3s，拿不到就报错给页面
    WATCHDOG_INTERVAL_S = 0.25  # 独立同步看门狗周期（与页面是否打开无关）
    # 看门狗连续读不到位置的次数上限（20×0.25s≈5s）：超过即认为同步保护失效必须报警，
    # 否则读位置一直超时时看门狗会"静默跳过"，保护形同虚设。
    WATCHDOG_MAX_MISSES = 20

    def __init__(self, lift_port: str, stir_port: str, sig_port: str = SIG_PORT,
                 bal_port: str = BAL_PORT) -> None:
        self.lift_port = lift_port
        self.stir_port = stir_port
        self.sig_port = sig_port
        self.bal_port = bal_port
        # COM10 共享总线锁：MD30（Modbus）与 LA（55AA 协议）事务互斥。
        self._bus10_lock = threading.Lock()
        # COM9 锁：ZDT 搅拌。
        self._bus9_lock = threading.Lock()
        # COM16 锁：QLS3600 信号发生器（独立串口，无需共享）。
        self._bus16_lock = threading.Lock()
        # COM11 锁：天平（独立串口）。
        self._bus11_lock = threading.Lock()
        # 最近一次成功的状态缓存（轮询拿不到锁时返回它，页面不会断流）。
        self._last_status: dict = {"lift": None, "rod": None, "stir": None, "bal": None}
        # 共享串口句柄惰性创建：设备掉线/重插后自动重开，服务进程不退出。
        self._shared_bus10 = None
        # 同步保障（保障 3：运行中看门狗）——锁存告警直到页面确认恢复。
        self.sync_tolerance = MD30DualLift.SYNC_TOLERANCE_RAW
        self._sync_alarm: dict | None = None
        self._sync_alarm_lock = threading.Lock()
        self._lift_was_moving = False   # 用于识别"运动结束"瞬间，记录最终偏差
        self._lift_last_target: int | None = None   # 本次运动目标，用于算未到位量
        # 推杆轮询节流：直到这个时刻前都高频读；空闲时走兜底间隔。
        self._rod_active_until = 0.0
        self._rod_last_read = 0.0
        # 最近一次成功读到的双机状态（原始对象）+ 时刻：看门狗可直接复用，省一次总线事务。
        self._last_lift_status: list | None = None
        self._last_lift_at = 0.0
        # 独立看门狗线程：保护不能依赖浏览器轮询（页面关掉/最小化就失效）。
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None
        # 振动定时运行：后台线程到点关输出（不阻塞 HTTP）。
        self._sig_cancel = threading.Event()
        self._sig_deadline: float | None = None
        # 天平缓存：最后一次成功读数 + 读取时刻 + 失败退避时刻。
        # 天平断线时每轮都等满超时会把页面拖慢（每次读 = 2 个事务），
        # 因此读失败后按 BAL_ERROR_BACKOFF_S 退避，期间直接回缓存。
        self._bal_last: dict | None = None
        self._bal_at = 0.0
        self._bal_next_try = 0.0
        self._bal_fail_logged = False
        self._bal_base: dict | None = None      # 扣重基准（页面 Δ 增量的起点）

        self.lift = MD30DualLift(port=lift_port, slave_addresses=(1, 2),
                                 default_speed=LIFT_SPEED_RAW,
                                 max_position_raw=LIFT_MAX_RAW,
                                 serial_factory=lambda **kwargs: self._get_bus10())
        self.rod = LAActuator(LAConfig(port=lift_port, node_id=3),
                              serial_factory=lambda **kwargs: self._get_bus10())
        self.stir = ZDTEmm42Private(Emm42PrivateConfig(port=stir_port, slave_address=1))
        self.sig = QLS3600SignalGenerator(QLS3600Config(port=sig_port))
        # 天平：retries=0 让单次读的最坏耗时被限制在 2×timeout(=1s) 内；
        # 反正页面每 0.5s 会再读一次，驱动内部重试只会让卡顿叠加。
        self.bal = AT8811CModbusRTU(AT8811CConfig(
            port=bal_port, slave_address=1, baudrate=9600, timeout_s=0.5,
            retries=0, max_consecutive_failures=5,
            scale_mg_per_count=BAL_SCALE_MG_PER_COUNT))
        # 闭环加粉执行器：把 dispensing_algorithm 的纯算法接到上面这些设备上。
        self.dispense = DispenseRunner(self, _log)
        # 开环速率标定执行器：输入粉名 → 扫 (搅拌, 推杆开度) → 得到该粉的速率表。
        self.calib = RateCalibrationRunner(self, _log)

    def _get_bus10(self):
        """获取 COM10 共享句柄；掉线后下次访问自动重开（USB 重插即恢复）。"""
        if self._shared_bus10 is None or not getattr(self._shared_bus10, "is_open", False):
            import serial as _serial_mod
            self._shared_bus10 = _serial_mod.Serial(
                port=self.lift_port, baudrate=115200, bytesize=8, parity="N",
                stopbits=1, timeout=0.3)
            # MD30 双机驱动内部缓存了旧句柄，重开后重新指向新句柄。
            self.lift._serial = self._shared_bus10
            for driver in self.lift._drivers:
                driver._serial = self._shared_bus10
            self.rod._serial = self._shared_bus10
        return self._shared_bus10

    def _reset_bus10(self) -> None:
        """串口报错后丢弃句柄，下次访问自动重开（应对 USB 掉线）。"""
        try:
            if self._shared_bus10 is not None:
                self._shared_bus10.close()
        except Exception:  # noqa: BLE001
            pass
        self._shared_bus10 = None
        self.lift._serial = None
        for driver in self.lift._drivers:
            driver._serial = None
            # 同时清零失败计数：MD30 达到上限后会拒绝一切事务直到重置。
            driver._consecutive_failures = 0
        self.rod._serial = None

    # ------------------------------------------------------------------
    # 带超时的总线锁（防止串口挂死拖垮整个 HTTP 服务）
    # ------------------------------------------------------------------

    @contextmanager
    def _bus10(self, timeout_s: float):
        if not self._bus10_lock.acquire(timeout=timeout_s):
            raise TimeoutError(f"{self.lift_port} 总线忙（等锁超时 {timeout_s}s）")
        try:
            yield
        finally:
            self._bus10_lock.release()

    @contextmanager
    def _bus9(self, timeout_s: float):
        if not self._bus9_lock.acquire(timeout=timeout_s):
            raise TimeoutError(f"{self.stir_port} 总线忙（等锁超时 {timeout_s}s）")
        try:
            yield
        finally:
            self._bus9_lock.release()

    @contextmanager
    def _bus16(self, timeout_s: float):
        """COM16 互斥锁（纯锁；串口失效自动重连见 _sig_call）。"""
        if not self._bus16_lock.acquire(timeout=timeout_s):
            raise TimeoutError(f"{self.sig_port} 总线忙（等锁超时 {timeout_s}s）")
        try:
            yield
        finally:
            self._bus16_lock.release()

    @contextmanager
    def _bus11(self, timeout_s: float):
        """COM11 互斥锁（天平独占一条串口，纯锁即可）。"""
        if not self._bus11_lock.acquire(timeout=timeout_s):
            raise TimeoutError(f"{self.bal_port} 串口忙（等锁超时 {timeout_s}s）")
        try:
            yield
        finally:
            self._bus11_lock.release()

    def _sig_call(self, action: Callable[[], Any], *, timeout_s: float | None = None):
        """在 COM16 锁下执行 ``action``；串口 I/O 失败时重开句柄并重试一次。

        USB 抖动/重插会让已打开的句柄失效——``is_open`` 仍为 True，但
        ``WriteFile`` 直接报 ``PermissionError(13, '拒绝访问')``。这种句柄只能
        丢弃重开。重试一次才能真正做到「点一下就好」，而不是报错后再点第二次。

        注意：驱动把底层 OSError 包装成了 ``QLS3600CommunicationError``，
        这里必须连它一起捕获，否则坏句柄会被反复复用。
        """
        with self._bus16(self.CTRL_LOCK_TIMEOUT_S if timeout_s is None else timeout_s):
            try:
                return action()
            except (OSError, QLS3600CommunicationError):
                self.sig._serial = None         # 丢弃疑似失效的句柄
                try:
                    self.sig.connect()          # 重开失败说明端口确实没释放
                except Exception as reconnect_error:  # noqa: BLE001 - 转中文提示
                    raise QLS3600CommunicationError(
                        f"{self.sig_port} 重新打开失败：{reconnect_error}"
                        f"（端口可能被串口助手等程序占用，或 USB 已断开）"
                    ) from reconnect_error
                return action()                 # 重开成功：本条指令重试一次

    # ------------------------------------------------------------------
    # 状态轮询
    # ------------------------------------------------------------------

    def _raise_sync_alarm(self, message: str, positions: list[int] | None = None) -> bool:
        """锁存同步告警（页面持续红条显示，直到用户确认恢复）。

        返回 True 表示这是**新**锁存的告警（调用方据此决定是否写日志，避免刷屏）。
        """
        with self._sync_alarm_lock:
            if self._sync_alarm is None:  # 只记第一条，避免刷屏覆盖
                self._sync_alarm = {"message": message,
                                    "positions": positions,
                                    "time": time.strftime("%H:%M:%S")}
                return True
            return False

    def acknowledge_sync_alarm(self) -> dict:
        """清除锁存的同步告警（用户在页面上点「确认恢复」）。"""
        with self._sync_alarm_lock:
            alarm = self._sync_alarm
            self._sync_alarm = None
        return {"acknowledged": True, "cleared": alarm}

    def _sync_watchdog(self, statuses: list) -> None:
        """保障 3：偏差看门狗（每轮状态轮询都会执行）。

        两种情况都要管，缺一不可：
        - **运动中超差** → 立即同步急停双机 + 锁存告警（这是原有的保障）；
        - **静止时超差** → 电机已经停了，急停没用，但绝不能就此放过：
          锁存告警并保持，直到用户执行「对齐双机」后确认恢复。否则页面
          会一直显示一个 80 的偏差却什么保护都不触发（2026-10-20 实测漏报）。
        """
        deviation = abs(statuses[0].position_raw - statuses[1].position_raw)
        positions = [s.position_raw for s in statuses]
        any_moving = any(s.moving for s in statuses)

        # 运动结束那一刻的最终位置/偏差快照——事后追查"什么时候差开的"全靠这条。
        # 同时带上目标值：一眼就能看出这次到底有没有走到位（速度过低会中途停）。
        if self._lift_was_moving and not any_moving:
            target = self._lift_last_target
            if target is None:
                _log(f"升降运动结束 位置={positions} 偏差={deviation}")
            else:
                missed = [abs(p - target) for p in positions]
                _log(f"升降运动结束 目标={target} 位置={positions} 偏差={deviation} "
                     f"未到位量={missed}")
        self._lift_was_moving = any_moving

        if deviation <= self.sync_tolerance:
            return

        if any_moving:
            try:
                self.lift.emergency_stop_both()
                message = (f"运行中双机偏差 {deviation} 超容差 "
                           f"{self.sync_tolerance}，已同步急停双机")
            except Exception as error:  # noqa: BLE001 - 急停失败更要大声报警
                message = (f"运行中双机偏差 {deviation} 超容差，且急停执行失败"
                           f"（{error}）——请立即断电！")
            if self._raise_sync_alarm(message, positions):
                _log(f"同步告警(运动中) 偏差={deviation} 容差={self.sync_tolerance} "
                     f"位置={positions} 已急停")
            return

        # 静止超差：锁存告警；move_both 的起跳前校验会拒绝继续移动，逼用户先对齐。
        message = (f"双机静止时位置偏差 {deviation} 已超容差 {self.sync_tolerance}"
                   f"（{positions[0]} / {positions[1]}）——请先点「对齐双机」，"
                   f"对齐前禁止下发升降移动")
        if self._raise_sync_alarm(message, positions):
            _log(f"同步告警(静止超差) 偏差={deviation} 容差={self.sync_tolerance} "
                 f"位置={positions} 未运动，已锁存告警")

    def start_watchdog(self) -> None:
        """启动独立同步看门狗线程（服务启动时调用一次）。"""
        if self._watchdog_thread is not None:
            return
        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, name="sync-watchdog", daemon=True)
        self._watchdog_thread.start()
        _log(f"同步看门狗已启动：每 {self.WATCHDOG_INTERVAL_S:.2f}s 独立巡检"
             f"（容差 {self.sync_tolerance}，不依赖页面是否打开）")

    def stop_watchdog(self) -> None:
        self._watchdog_stop.set()

    def _watchdog_loop(self) -> None:
        """后台巡检双机偏差——**与页面轮询解耦**。

        原先看门狗挂在 /api/status 里，页面一关、浏览器最小化（定时器被节流）
        保护就停了。现在无论页面开不开，服务端都在按固定周期自检。

        总线复用：页面开着时每 0.5s 已经读过一次双机状态，看门狗直接拿这份
        **新鲜数据**(<0.45s) 判断即可，不必再占用 COM10 重读一遍。页面不开或
        数据过期时才自己读——既保住独立保护，又把总线事务量砍掉约一半
        （COM10 上还挂着 LA 推杆，从站 2 偶发超时需要留余量）。
        """
        consecutive_fail = 0
        last_fail_log_at = 0.0
        while not self._watchdog_stop.wait(self.WATCHDOG_INTERVAL_S):
            try:
                cached = self._last_lift_status
                fresh = (cached is not None
                         and time.monotonic() - self._last_lift_at < 0.45)
                if fresh:
                    statuses = cached
                else:
                    with self._bus10(0.15):   # 拿不到锁（正在下发指令）就跳过本轮
                        statuses = self.lift.read_positions()
                if len(statuses) == 2:
                    self._sync_watchdog(statuses)
                if consecutive_fail:
                    _log(f"看门狗巡检恢复（此前连续失败 {consecutive_fail} 次）")
                consecutive_fail = 0
            except TimeoutError:
                continue                     # 总线被控制指令占用，属正常，下一轮再来
            except Exception as error:       # noqa: BLE001 - 巡检异常绝不能让线程死掉
                # 读不到位置 = 保护失效：连续失败超限必须报警，不能无声跳过。
                consecutive_fail += 1
                # ‼️ 串口级故障在这里也要重开句柄：页面不开时没有别的线程会复位
                # COM10，只报警不重开就永远好不了（2026-10-20 僵死 32 分钟）。
                if isinstance(error, (MD30SerialError, OSError)):
                    self._reset_bus10()
                # 日志节流：句柄僵死时每 0.25s 一条，30 分钟能刷出 7000+ 条，
                # 把有效记录全埋掉。同样的错误 10s 记一条，恢复时另记一条。
                now = time.monotonic()
                if now - last_fail_log_at >= 10.0:
                    last_fail_log_at = now
                    _log(f"看门狗巡检异常（连续第 {consecutive_fail} 次，"
                         f"同错 10s 内不重复记录）：{error}")
                if consecutive_fail == self.WATCHDOG_MAX_MISSES:
                    self._raise_sync_alarm(
                        f"同步保护失效：看门狗连续 {consecutive_fail} 次读不到双机位置"
                        f"（约 {consecutive_fail * self.WATCHDOG_INTERVAL_S:.1f}s）"
                        f"——请检查 COM10 与电缸供电，确认两台位置后再继续操作")

    def poll_status(self) -> dict:
        status: dict = {"lift": None, "rod": None, "stir": None, "bal": None,
                        "errors": [], "stale": False}
        try:
            with self._bus10(self.POLL_LOCK_TIMEOUT_S):
                try:
                    statuses = self.lift.read_positions()
                    # 供看门狗复用（避免它再占一次总线读同样的数据）。
                    if len(statuses) == 2:
                        self._last_lift_status = statuses
                        self._last_lift_at = time.monotonic()
                    status["lift"] = [
                        {"addr": d.config.slave_address,
                         "pos": s.position_raw, "moving": s.moving,
                         "speed": s.speed_raw, "voltage": s.voltage_raw}
                        for d, s in zip(self.lift._drivers, statuses)
                    ]
                    # 保障 3（看门狗）由独立后台线程执行，不在这里做：
                    # 挂在页面轮询上的话，页面一关/最小化保护就停了。
                except Exception as error:  # noqa: BLE001 - 轮询不容错会中断页面
                    status["errors"].append(f"lift: {error}")
                    # ‼️ 只有「串口本身坏了」才重开句柄。单纯"从站没应答"每次都
                    # 关/开 COM10，会让 USB-485 反复拉 DTR/RTS 并清空缓冲，把
                    # 偶发一次超时放大成持续失联（2026-10-20 实测的恶化环）。
                    if isinstance(error, (MD30SerialError, OSError)):
                        self._reset_bus10()
                        status["errors"].append("lift: 串口已重置，下一轮将自动重连")
                # 推杆节流读：只有"刚下发过动作"或"空闲兜底到期"才占用总线。
                # 总线让给升降双机——从站 2 偶发超时，减少一半事务量能明显改善。
                now = time.monotonic()
                rod_due = (now < self._rod_active_until
                           or now - self._rod_last_read >= ROD_IDLE_INTERVAL_S)
                if rod_due:
                    try:
                        rod = self.rod.read_status()
                        status["rod"] = {"pos": rod.current_position,
                                         "target": rod.target_position,
                                         "temp": rod.temperature_c,
                                         "current": rod.current_ma,
                                         "error": rod.error_bits}
                        self._rod_last_read = now
                    except Exception as error:  # noqa: BLE001
                        status["errors"].append(f"rod: {error}")
                        self._rod_last_read = now   # 失败也算读过，避免每轮都撞同一个超时
                else:
                    status["rod"] = self._last_status["rod"]
        except TimeoutError:
            # 总线被控制指令长期占用：跳过本轮，回上一次的缓存，页面不断流。
            status["stale"] = True
            status["lift"] = self._last_status["lift"]
            status["rod"] = self._last_status["rod"]
        if status["lift"] is not None:
            self._last_status["lift"] = status["lift"]
        if status["rod"] is not None:
            self._last_status["rod"] = status["rod"]
        try:
            with self._bus9(self.POLL_LOCK_TIMEOUT_S):
                try:
                    stir = self.stir.read_status()
                    status["stir"] = {"rpm": stir.speed_rpm, "flags": stir.state_flags,
                                      "enabled": stir.enabled}
                except Exception as error:  # noqa: BLE001
                    status["errors"].append(f"stir: {error}")
                    # 同上：只在串口级故障时重开，单纯超时不打断链路。
                    if isinstance(error, OSError):
                        self.stir.close()  # 串口级错误：丢弃句柄，下轮自动重连
        except TimeoutError:
            status["stale"] = True
            status["stir"] = self._last_status["stir"]
        if status["stir"] is not None:
            self._last_status["stir"] = status["stir"]
        # 天平：独占 COM11，不参与 COM10/COM9 的锁竞争。读失败只回错误文字，
        # **不写进 status["errors"]**——天平只是量值来源，断线不该把整页判成故障
        # （否则红字会一直挂在那里，掩盖真正影响运动的错误）。
        status["bal"] = self.bal_poll()
        self._last_status["bal"] = status["bal"]
        # 闭环加粉：纯内存状态，不碰串口，直接带上（页面用它画进度与结果）。
        status["dispense"] = self.dispense.status()
        # 结束后「当前质量」还要继续实时读：收尾（settling）和结束（done/aborted）时
        # 闭环主循环已经不再采数，页面拿本炉基线把这一轮天平计数换算成净质量自己刷新。
        # 用 raw 而不是 bal.mg，因为基线本身就是由 raw 计数取的，两者口径必须一致。
        bal_now = status.get("bal") or {}
        if bal_now.get("raw") is not None:
            net = self.dispense.net_mass(bal_now["raw"])
            if net is not None:
                status["dispense"]["net_mass_mg"] = round(net, 1)
                status["dispense"]["net_at"] = bal_now.get("at")
                status["dispense"]["net_stale"] = bool(bal_now.get("stale"))
        # 开环速率标定：同样是纯内存状态。
        status["calib"] = self.calib.status()
        # 同步保障信息：当前偏差 + 锁存告警（页面据此显示告警红条）。
        # 另附每台的通讯健康度：熔断/失败次数，页面上能直接看出是哪一台不回话。
        status["lift_link"] = [
            {"addr": d.config.slave_address,
             "failures": d._consecutive_failures,
             "tripped": d.tripped}
            for d in self.lift._drivers
        ]
        if status["lift"] and len(status["lift"]) == 2:
            status["sync"] = {
                "dev": abs(status["lift"][0]["pos"] - status["lift"][1]["pos"]),
                "tol": self.sync_tolerance,
                "alarm": self._sync_alarm,
            }
        else:
            status["sync"] = {"dev": None, "tol": self.sync_tolerance,
                              "alarm": self._sync_alarm}
        # 信号发生器：设备不回读设定值，只回缓存 + 定时剩余秒数（不发串口事务）。
        status["sig"] = self.sig_snapshot()
        status["ok"] = not status["errors"] and self._sync_alarm is None
        return status

    # ------------------------------------------------------------------
    # 升降（双 MD30，广播同步）
    # ------------------------------------------------------------------

    # 搅拌互锁：转速高于这个值即认为"搅拌在转"，一律禁止升降运动。
    # 停机后转子要时间滑行归零，读到 0 才放行（宁可多等两秒，别带着搅拌动升降）。
    LIFT_STIR_INTERLOCK_RPM = 1

    def _stir_speed_rpm(self) -> float | None:
        """读一次搅拌实时转速；读不到（或设备没回转速）返回 None。

        锁等待用控制级超时（3s）而不是轮询级（0.3s）：页面每 500ms 也在读
        COM9，若正好撞上就 0.3s 超时，会把"用户点按钮"误判成"读不到转速"。
        """
        try:
            with self._bus9(self.CTRL_LOCK_TIMEOUT_S):
                status = self.stir.read_status()
        except Exception as error:  # noqa: BLE001 - 互锁检查不允许把异常吞成"放行"
            _log(f"读取搅拌转速失败：{error}")
            return None
        if status.speed_rpm is None:
            return None
        return float(status.speed_rpm)

    def stir_speed_rpm(self) -> float | None:
        """公开的转速读取：标定/加粉收尾要确认"真的停下来了"。"""
        return self._stir_speed_rpm()

    def _guard_lift_requires_stir_stopped(self, action: str) -> None:
        """搅拌在转时禁止升降运动（2026-10-20 用户要求）。

        ‼️ 读不到转速时**拒绝**而不是放行：无法确认搅拌是否停着就动升降，
        等于拿机构去赌"应该没事"——赌输的代价是撞坏。
        注意：本检查不拿 COM10 锁（只碰 COM9），必须在进入 _bus10 之前调用，
        否则会形成 bus10→bus9 的嵌套，与其它路径的锁顺序不一致。
        """
        rpm = self._stir_speed_rpm()
        if rpm is None:
            raise InterlockError(
                f"无法读取搅拌转速，不能确认搅拌已停，拒绝{action}。"
                f"请检查 COM9 连接后重试")
        if abs(rpm) > self.LIFT_STIR_INTERLOCK_RPM:
            raise InterlockError(
                f"搅拌正在运行（{rpm:.0f} RPM），禁止{action}："
                f"请先停搅拌，等转速显示归零后再操作")

    def lift_move(self, target: int, speed: int | None = None) -> dict:
        # 只下发目标、立即返回：运动过程由页面 500ms 状态轮询实时刷新。
        # 安全约束：速度固定 LIFT_SPEED_RAW（慢速），页面传值一律忽略。
        target = int(target)
        if not LIFT_MIN_RAW <= target <= LIFT_MAX_RAW:
            _log(f"升降移动被拒 → {target}：超出允许范围 {LIFT_MIN_RAW}..{LIFT_MAX_RAW}")
            raise ValueError(f"目标位置必须在 {LIFT_MIN_RAW}..{LIFT_MAX_RAW} 之间（当前 {target}）")
        # 互锁：搅拌在转 / 无法确认搅拌已停 → 一律不许动升降。
        try:
            self._guard_lift_requires_stir_stopped(f"升降移动（目标 {target}）")
        except InterlockError as error:
            _log(f"升降移动被拒 → {target}：{error}")
            raise
        with self._bus10(self.CTRL_LOCK_TIMEOUT_S):
            try:
                self.lift.move_both(int(target), speed_raw=LIFT_SPEED_RAW)
            except MD30SyncGuardError as error:
                _log(f"升降移动被拒 → {target}：{error}")
                raise
        _log(f"升降移动下发 → {target}（速度 {LIFT_SPEED_RAW}）")
        self._lift_last_target = target   # 运动结束时用它算"未到位量"
        return {"target": int(target), "sent": True, "speed": LIFT_SPEED_RAW}

    def lift_home(self, speed: int | None = None) -> dict:
        _log("升降回原位")
        return self.lift_move(LIFT_HOME_RAW)

    def lift_stop(self) -> dict:
        with self._bus10(self.CTRL_LOCK_TIMEOUT_S):
            self.lift.stop_both()
        _log("升降停止指令")
        return {"stopped": True}

    def lift_align(self, speed: int | None = None) -> dict:
        """对齐双机（把两台都移到当前位置中点，消除起跳前偏差）。"""
        # 对齐同样会让两台电缸动起来 → 同一把互锁。
        try:
            self._guard_lift_requires_stir_stopped("升降对齐")
        except InterlockError as error:
            _log(f"升降对齐被拒：{error}")
            raise
        with self._bus10(15.0):  # 对齐含等待停止，允许更长时间
            result = self.lift.align(speed_raw=speed)
        _log(f"升降对齐 中点={result['midpoint']} 最终位置={result['final_positions']} "
             f"偏差={result['deviation']}")
        if not result["aligned"]:
            raise MD30SyncGuardError(f"对齐后偏差仍为 {result['deviation']}，请检查机构")
        return result

    # ------------------------------------------------------------------
    # 推杆（LA）
    # ------------------------------------------------------------------

    def rod_move(self, target: int) -> dict:
        # 同升降：只下发定位指令立即返回，位置由状态轮询刷新。
        # 注意：LA 急停（0x23）后会拒绝定位帧，必须先重新使能（0x04）才能运动，
        # 因此这里每次运动前自动使能一次（帧很短，开销可忽略）。
        with self._bus10(self.CTRL_LOCK_TIMEOUT_S):
            self.rod.enable()
            self.rod.move_to(int(target), feedback=True)
        # 开启高频读取窗口：接下来几秒页面能实时看到推杆在走，窗口过后恢复空闲低频。
        self._rod_active_until = time.monotonic() + ROD_ACTIVE_WINDOW_S
        _log(f"推杆定位 → {target}")
        return {"target": int(target), "sent": True}

    def rod_home(self) -> dict:
        _log("推杆回原位")
        return self.rod_move(ROD_HOME)

    def rod_stop(self) -> dict:
        with self._bus10(self.CTRL_LOCK_TIMEOUT_S):
            self.rod.emergency_stop()
        self._rod_active_until = time.monotonic() + ROD_ACTIVE_WINDOW_S
        _log("推杆急停")
        return {"stopped": True}

    def rod_refresh(self) -> dict:
        """强制立即读一次推杆（页面「刷新」按钮用，跳过节流）。"""
        with self._bus10(self.CTRL_LOCK_TIMEOUT_S):
            rod = self.rod.read_status()
        self._rod_last_read = time.monotonic()
        return {"pos": rod.current_position, "target": rod.target_position,
                "temp": rod.temperature_c, "current": rod.current_ma,
                "error": rod.error_bits}

    # ------------------------------------------------------------------
    # 旋转搅拌（ZDT）
    # ------------------------------------------------------------------

    def _guard_stir_requires_lift_stopped(self) -> None:
        """反向互锁：升降在运动时禁止启动搅拌。

        不拿 COM9 锁，只短暂占用 COM10 读一次双机状态；读不到就拒绝，
        理由同前：无法确认升降已停，就不能让搅拌转起来。
        """
        try:
            with self._bus10(self.CTRL_LOCK_TIMEOUT_S):
                statuses = self.lift.read_positions()
        except Exception as error:  # noqa: BLE001
            raise InterlockError(
                f"无法读取升降位置，不能确认升降已停，拒绝启动搅拌（{error}）") from error
        moving = [s.moving for s in statuses]
        if any(moving):
            raise InterlockError(
                f"升降电机正在运动（{moving}），禁止启动搅拌："
                f"请等升降停稳后再开搅拌")

    def stir_enable(self, on: bool) -> dict:
        with self._bus9(self.CTRL_LOCK_TIMEOUT_S):
            self.stir.enable(on)
        _log(f"搅拌使能 {'开' if on else '关'}")
        return {"enabled": on}

    def stir_start(self, rpm: float, acc: int, direction: int) -> dict:
        # 反向互锁：升降还在动的时候不许起搅拌——否则"搅拌时不动升降"这条
        # 约束可以被"先动升降、再开搅拌"直接绕过，等于没加。
        try:
            self._guard_stir_requires_lift_stopped()
        except InterlockError as error:
            _log(f"搅拌启动被拒：{error}")
            raise
        with self._bus9(self.CTRL_LOCK_TIMEOUT_S):
            self.stir.enable(True)
            self.stir.set_speed(float(rpm), acceleration=int(acc), direction=int(direction))
        _log(f"搅拌启动 {rpm} RPM（acc={acc}，方向={direction}）")
        return {"rpm": rpm, "acc": acc, "direction": direction}

    def stir_stop(self, mode: str = "immediate", acc: int = 200) -> dict:
        # 停止这条路不能因为参数越界就失败：acc 是 0..255 的加减速曲线等级，
        # 传大了（标定收尾曾写 300）驱动会直接抛 "acceleration 0..255"，
        # 结果就是"该停没停"——搅拌一直转，粉一直落，天平还在涨。
        # 这里统一钳位；decel 万一失败再降级立即停：停下来永远比停得柔和重要。
        acc = max(0, min(255, int(acc)))
        with self._bus9(self.CTRL_LOCK_TIMEOUT_S):
            if mode == "decel":
                try:
                    self.stir.stop_decelerated(acceleration=acc)
                except Exception as error:      # noqa: BLE001
                    _log(f"搅拌减速停止失败（acc={acc}：{error}），改为立即停止")
                    self.stir.stop_immediate()
            else:
                self.stir.stop_immediate()
        _log(f"搅拌停止（{mode}，acc={acc}）")
        return {"mode": mode, "acc": acc}

    def stir_clear_stall(self) -> dict:
        """解除搅拌驱动的堵转保护（0x0E），并把清除前后的状态字节都报出来。

        堵转保护是**锁存**的：一旦触发，哪怕堵转原因早就消失、转速已经是 0，
        驱动也会拒绝新的转速指令，0x3A 的 bit2/bit3 一直亮着 —— 这时点"启动搅拌"
        没反应，不是驱动坏了，就是需要显式清一次。清之前先读一次状态，清完再读一次，
        免得操作员只能靠"能不能转"来猜到底清掉没有。
        """
        with self._bus9(self.CTRL_LOCK_TIMEOUT_S):
            before = self.stir.read_status()
            cleared = self.stir.clear_stall()
            after = self.stir.read_status()

        def brief(status) -> dict:
            return {"flags": status.state_flags,
                    "enabled": status.enabled,
                    "stall": status.stall,
                    "stall_protected": status.stall_protected,
                    "rpm": status.speed_rpm}

        _log(f"搅拌堵转保护清除：状态 {before.state_flags} → {after.state_flags}"
             f"（堵转={after.stall}，保护={after.stall_protected}，"
             f"{'驱动报告本来就是干净的' if not cleared else '驱动已执行清除'}）")
        return {"before": brief(before), "after": brief(after), "cleared": cleared}

    def startup_reset(self) -> None:
        """服务刚起来时把执行机构恢复成安全静止态。

        上一个进程可能是被强杀的（taskkill /F 不走 finally）：搅拌会保持最后的速度指令
        一直转，振动输出也可能留在开着的状态。启动时如果不主动清，现场就是
        "没人操作、电机却在转"。这里只做两件事：关振动、把残留转速减速停到 0。
        """
        try:
            self.sig_output(False)
        except Exception as error:      # noqa: BLE001
            _log(f"启动复位：关闭振动失败（{error}）")
        rpm = self._stir_speed_rpm()
        if rpm is None or abs(rpm) <= 1.0:
            return
        _log(f"启动复位：搅拌残留 {rpm:.0f} RPM（上个进程遗留），减速停止")
        try:
            self.stir_stop("decel", acc=200)
        except Exception as error:      # noqa: BLE001
            _log(f"启动复位：停搅拌失败（{error}）")

    # ------------------------------------------------------------------
    # 振动给料（QLS3600S 信号发生器）
    # ------------------------------------------------------------------

    def sig_snapshot(self) -> dict:
        """信号发生器状态快照：设定值取驱动缓存，定时剩余由本类计算。"""
        state = self.sig.state
        remaining = None
        if self._sig_deadline is not None:
            remaining = max(0.0, round(self._sig_deadline - time.monotonic(), 1))
        # 方波的实际强度由占空比决定，显示时直接带上占空比（设备无独立脉冲编号）
        display_name = state.waveform_name
        if state.waveform == 2 and state.duty_percent is not None:   # 2=方波（真机实测）
            display_name = f"方波 {state.duty_percent:g}%"
        return {
            "online": self.sig.connected,
            "waveform": state.waveform,
            "waveform_name": display_name,
            "freq": state.frequency_hz,
            "amp": state.amplitude_vpp,
            "duty": state.duty_percent,
            "offset": state.offset_v,
            "output": state.output_on,
            "remaining": remaining,
        }

    def sig_apply_defaults(self, *, waveform: int = 2, frequency_hz: float = 80.0,
                           duty_percent: float = 50.0,
                           amplitude_vpp: float = 2.0) -> None:
        """服务启动时把已知参数重新下发一次。

        QLS3600 不支持回读设定值，驱动只能缓存；进程重启后缓存为空，
        页面波形/幅度会显示空白。这里按默认值重发，保证页面显示与真机一致。
        输出保持关闭（不会自己开始振动），失败只打印提示、不影响服务启动。

        默认值（2026-10-21 用户定）：**方波** 80 Hz / **50%** 占空比 / 2 Vpp。
        标定与闭环加粉运行时也会各自把波形强制设成方波（占空比只对方波生效）。
        """
        try:
            def _init_signal() -> None:
                self.sig.set_waveform(int(waveform))
                self.sig.set_frequency(float(frequency_hz))
                self.sig.set_duty(float(duty_percent))
                self.sig.set_amplitude(float(amplitude_vpp))
                self.sig.output(False)

            self._sig_call(_init_signal)
        except Exception as error:  # noqa: BLE001 - 设备未接不阻止服务启动
            print(f"信号发生器初始化跳过（{self.sig_port}）：{error}")

    def _apply_waveform(self, waveform: object) -> None:
        """波形切换（调用方必须已持有 COM16 锁）。

        支持编号（0..32）或名称（sine/square/triangle/pulse）；
        ``pulse`` = 方波 + 当前占空比（设备没有独立的脉冲波形编号）。
        """
        if isinstance(waveform, str):
            key = waveform.strip().lower()
            if key == "pulse":
                self.sig.set_pulse_wave(self.sig.state.duty_percent or 50.0)
                return
            mapping = {"sine": 0, "square": 2, "triangle": 1}   # 真机实测：1=三角、2=方波
            if key not in mapping:
                raise ValueError(f"未知波形 {waveform!r}（可用 sine/square/triangle/pulse）")
            self.sig.set_waveform(mapping[key])
            return
        self.sig.set_waveform(int(waveform))

    def sig_set_waveform(self, waveform: object) -> dict:
        self._sig_call(lambda: self._apply_waveform(waveform))
        return self.sig_snapshot()

    def sig_set_frequency(self, hz: float) -> dict:
        self._sig_call(lambda: self.sig.set_frequency(float(hz)))
        return self.sig_snapshot()

    def sig_set_amplitude(self, vpp: float) -> dict:
        self._sig_call(lambda: self.sig.set_amplitude(float(vpp)))
        return self.sig_snapshot()

    def sig_set_duty(self, percent: float) -> dict:
        self._sig_call(lambda: self.sig.set_duty(float(percent)))
        return self.sig_snapshot()

    def sig_output(self, on: bool) -> dict:
        """OUT 输出开关（手动开/关，直接生效）。"""
        if not on:
            self._sig_cancel.set()          # 手动关输出同时取消定时任务
            self._sig_deadline = None
        self._sig_call(lambda: self.sig.output(bool(on)))
        return self.sig_snapshot()

    def sig_output_user(self, on: bool) -> dict:
        """页面手动开输出：加粉运行中禁止开启（会和闭环抢振动），关闭永远允许。"""
        if on:
            self._guard_no_dispense_running("开启振动输出")
        return self.sig_output(on)

    def sig_burst(self, count: int, *, frequency_hz: float | None = None,
                  amplitude_vpp: float | None = None) -> dict:
        """发一组猝发脉冲（真脉冲，设备自动输出 count 个周期后自己停）。

        与 /api/sig/run 的区别：run 是"软件掐表开 OUT 再关"，脉冲的尾巴
        靠轮询精度；burst 是设备硬件计数，第 count 个周期一结束输出立即停。
        首次调用会做 burst_setup（WN0 手动触发 + WM 个数）+ burst_arm（WO1+Wc1）。
        """
        count = int(count)
        if not 1 <= count <= 999_999:
            raise ValueError("猝发个数须在 1–999999")
        self._guard_no_dispense_running("发猝发脉冲")

        def _setup_and_fire() -> dict:
            if frequency_hz is not None:
                self.sig.set_frequency(float(frequency_hz))
            if amplitude_vpp is not None:
                self.sig.set_amplitude(float(amplitude_vpp))
            self.sig.burst_setup(count)
            self.sig.burst_arm()
            return self.sig.burst_fire(count)

        info = self._sig_call(_setup_and_fire)
        snapshot = self.sig_snapshot()
        snapshot["burst"] = info
        return snapshot

    def sig_run(self, seconds: float, *, frequency_hz: float | None = None,
                amplitude_vpp: float | None = None, waveform: object | None = None) -> dict:
        """定时振动：按需设置参数后开 OUT，运行 seconds 秒自动关。

        定时靠后台线程，接口立即返回；页面用状态轮询显示剩余时间。
        """
        seconds = float(seconds)
        if not 0 < seconds <= 600:
            raise ValueError("定时运行时长须在 0–600 秒")
        self._sig_cancel.set()              # 取消上一轮定时

        def _start() -> None:
            if waveform is not None:
                self._apply_waveform(waveform)
            if frequency_hz is not None:
                self.sig.set_frequency(float(frequency_hz))
            if amplitude_vpp is not None:
                self.sig.set_amplitude(float(amplitude_vpp))
            self.sig.output(True)

        self._sig_call(_start)
        self._sig_cancel = threading.Event()   # 新一轮的取消标志
        self._sig_deadline = time.monotonic() + seconds
        threading.Thread(target=self._sig_run_worker, args=(seconds,), daemon=True).start()
        snapshot = self.sig_snapshot()
        snapshot["requested_s"] = seconds
        return snapshot

    def _sig_run_worker(self, seconds: float) -> None:
        """定时结束关输出（含手动停止/异常路径）——关输出绝不会被跳过。"""
        cancel = self._sig_cancel
        end = time.monotonic() + seconds
        while True:
            left = end - time.monotonic()
            if left <= 0 or cancel.wait(min(0.1, max(0.0, left))):
                break
        try:
            self._sig_call(lambda: self.sig.output(False))
        except Exception:  # noqa: BLE001 - 定时关闭失败不能让线程崩
            pass
        finally:
            if self._sig_cancel is cancel:
                self._sig_deadline = None

    def sig_stop(self) -> dict:
        """停止振动：取消定时并立即关 OUT（页面「停止」按钮）。"""
        self._sig_cancel.set()
        self._sig_deadline = None
        self._sig_call(lambda: self.sig.output(False))
        return self.sig_snapshot()

    # ------------------------------------------------------------------
    # 天平（AT8811C，COM11）
    # ------------------------------------------------------------------

    BAL_MIN_READ_INTERVAL_S = 0.4   # 两次真实读取的最小间隔（≈一个轮询周期）
    BAL_ERROR_BACKOFF_S = 5.0       # 读失败后的退避：断线时不再每轮都白等超时

    def _bal_empty(self, error: str | None = None) -> dict:
        """从未读到过数据时的空快照。"""
        return {"online": False, "raw": None, "mg": None, "g": None,
                "stable": None, "zero_band": None, "status_word": None,
                "at": None, "base": self._bal_base, "delta_mg": None,
                "stale": True, "error": error}

    def _bal_snapshot(self, measurement: Any) -> dict:
        """把驱动 Measurement 转成页面用的 dict（含基准增量 Δ）。"""
        mass_mg = measurement.mass_mg
        base = self._bal_base
        delta = None
        if base is not None and mass_mg is not None:
            delta = round(mass_mg - float(base["mg"]), 1)
        return {
            "online": True,
            "raw": measurement.raw_count,
            "mg": None if mass_mg is None else round(float(mass_mg), 1),
            "g": None if mass_mg is None else round(float(mass_mg) / 1000.0, 4),
            "stable": measurement.stable,
            "zero_band": measurement.in_zero_band,
            "status_word": measurement.status_word,
            "at": time.strftime("%H:%M:%S"),
            "base": base,
            "delta_mg": delta,
            "stale": False,
            "error": None,
        }

    def _bal_read_locked(self) -> dict:
        """读一次天平并更新缓存。**调用方必须已持有 COM11 锁。**"""
        measurement = self.bal.read_measurement()
        snapshot = self._bal_snapshot(measurement)
        self._bal_last = snapshot
        self._bal_at = time.monotonic()
        self._bal_next_try = 0.0
        if self._bal_fail_logged:
            self._bal_fail_logged = False
            _log("天平通讯恢复")
        return dict(snapshot)

    def bal_poll(self) -> dict:
        """轮询专用：节流 + 失败退避，**绝不抛异常**（天平断线不能拖垮页面）。"""
        now = time.monotonic()
        if now < self._bal_next_try:
            return self._bal_stale()
        if self._bal_last is not None and now - self._bal_at < self.BAL_MIN_READ_INTERVAL_S:
            return dict(self._bal_last)
        try:
            with self._bus11(self.POLL_LOCK_TIMEOUT_S):
                return self._bal_read_locked()
        except TimeoutError:
            return self._bal_stale()
        except Exception as error:  # noqa: BLE001 - 轮询不容错会中断页面
            self._bal_note_failure(error)
            return self._bal_stale(str(error))

    def _bal_stale(self, error: str | None = None) -> dict:
        """没读成功：回上一次的缓存（标记 stale），从没读过则回空快照。"""
        if self._bal_last is None:
            return self._bal_empty(error)
        snapshot = dict(self._bal_last)
        snapshot["stale"] = True
        snapshot["error"] = error or snapshot.get("error")
        return snapshot

    def _bal_note_failure(self, error: Exception) -> None:
        self._bal_next_try = time.monotonic() + self.BAL_ERROR_BACKOFF_S
        # 驱动在连续失败后会自行重置连接；这里只负责别再每轮都等满超时。
        if not self._bal_fail_logged:
            self._bal_fail_logged = True
            _log(f"天平读取失败（{error}）——{self.BAL_ERROR_BACKOFF_S:.0f}s 退避后重试，"
                 f"请检查 {self.bal_port} 与天平电源")

    def bal_refresh(self) -> dict:
        """强制立即读一次（页面「刷新」按钮，跳过节流与退避）。"""
        try:
            with self._bus11(self.CTRL_LOCK_TIMEOUT_S):
                return self._bal_read_locked()
        except Exception as error:  # noqa: BLE001 - 如实回传页面
            self._bal_note_failure(error)
            raise

    def _guard_balance_idle(self, action: str) -> None:
        """去皮/清零前的机构静止校验。

        ‼️ 天平是整条闭环的唯一量值来源：「去皮」会把此刻的重量强行写成 0。
        只要还有机构在动（振动还在出粉、搅拌在转、推杆在推、升降在走），
        清零就会把"正在变化的量"当成皮重扣掉，之后所有读数都偏，且不可恢复，
        只能重新标定。所以任何一项读不到也一律拒绝，不拿量值去赌。
        """
        reasons: list[str] = []
        if self.sig.state.output_on:
            reasons.append("振动给料 OUT 仍开着")
        rpm = self._stir_speed_rpm()          # 只碰 COM9，必须在拿 COM10 锁之前
        if rpm is None:
            raise InterlockError(
                f"无法读取搅拌转速，不能确认机构已停，拒绝{action}。请检查 COM9 后重试")
        if abs(rpm) > self.LIFT_STIR_INTERLOCK_RPM:
            reasons.append(f"搅拌在转（{rpm:.0f} RPM）")
        try:
            with self._bus10(self.CTRL_LOCK_TIMEOUT_S):
                statuses = self.lift.read_positions()
                rod = self.rod.read_status()
        except Exception as error:  # noqa: BLE001
            raise InterlockError(
                f"无法读取升降/推杆状态，不能确认机构已停，拒绝{action}（{error}）") from error
        if any(status.moving for status in statuses):
            reasons.append("升降在运动")
        if abs(rod.current_position - rod.target_position) > 2:
            reasons.append(f"推杆在动（{rod.current_position}→{rod.target_position}）")
        if reasons:
            raise InterlockError(
                f"{action}被拒绝：" + "、".join(reasons) +
                "——请等机构完全停止（含振动输出关闭）后再操作，否则皮重会被扣错")

    def bal_tare(self) -> dict:
        """去皮归零：把天平实时重量寄存器写 0（驱动要求显式授权）。

        清零后连读几次直到接近 0（设备内部结算需要几百毫秒），把稳定后的值返回页面。
        """
        self._guard_balance_idle("天平去皮归零")
        before_mg: float | None = None
        try:
            with self._bus11(self.CTRL_LOCK_TIMEOUT_S):
                before = self.bal.read_measurement()
                before_mg = before.mass_mg
                self.bal.zero(authorized=True)
                snapshot: dict | None = None
                for _ in range(6):          # 最多 0.9s，等设备内部把零点结算完
                    time.sleep(0.15)
                    snapshot = self._bal_read_locked()
                    if snapshot["raw"] is not None and abs(snapshot["raw"]) <= 2:
                        break
        except Exception as error:  # noqa: BLE001
            self._bal_note_failure(error)
            raise
        assert snapshot is not None
        if self._bal_base is not None:       # 清零后旧基准失去意义，直接清掉
            self._bal_base = None
            snapshot["base"] = None
            snapshot["delta_mg"] = None
            if self._bal_last is not None:
                self._bal_last["base"] = None
                self._bal_last["delta_mg"] = None
        _log(f"天平去皮归零完成：清零前 {before_mg} mg，清零后 {snapshot['mg']} mg")
        result = dict(snapshot)
        result["tare_before_mg"] = None if before_mg is None else round(float(before_mg), 1)
        return result

    def bal_stable(self, timeout_s: float = 6.0, samples: int = 8) -> dict:
        """等天平稳定后取平均（单次读数会跳，计量前后要一个可信值）。

        判据：连续 ``samples`` 个读数都带稳定位且极差 ≤ 2 mg 才认；超时则
        返回已采到的样本并标 ``stable=False``（页面据此提示"还在波动"）。
        """
        timeout_s = max(1.0, min(float(timeout_s), 20.0))
        samples = max(2, min(int(samples), 50))
        deadline = time.monotonic() + timeout_s
        collected: list[float] = []
        best: list[float] = []
        while time.monotonic() < deadline:
            try:
                with self._bus11(self.CTRL_LOCK_TIMEOUT_S):
                    measurement = self.bal.read_measurement()
            except Exception as error:  # noqa: BLE001
                self._bal_note_failure(error)
                raise
            if measurement.mass_mg is not None:
                collected.append(float(measurement.mass_mg))
                if measurement.stable and len(collected) > samples:
                    recent = collected[-samples:]
                    if max(recent) - min(recent) <= 2.0:
                        best = recent
                        break
            time.sleep(0.1)
        window = best or collected[-samples:]
        self.bal_refresh()                    # 让页面上的实时值也同步到最新
        if not window:
            raise ValueError("未能读到任何有效重量样本")
        mean = sum(window) / len(window)
        variance = sum((value - mean) ** 2 for value in window) / len(window)
        result = {
            "stable": bool(best),
            "samples": len(window),
            "mean_mg": round(mean, 1),
            "sd_mg": round(variance ** 0.5, 2),
            "min_mg": round(min(window), 1),
            "max_mg": round(max(window), 1),
            "mg": round(float(window[-1]), 1),
        }
        _log(f"天平稳定采样：{result['samples']} 个样本 均值 {result['mean_mg']} mg "
             f"标准差 {result['sd_mg']} mg（{'稳定' if result['stable'] else '未判稳'}）")
        return result

    def bal_set_base(self, *, clear: bool = False) -> dict:
        """记录/清除扣重基准（页面 Δ 增量的起点，用于"加了粉多少"）。"""
        if clear:
            self._bal_base = None
            _log("天平基准已清除")
            if self._bal_last is None:
                return self._bal_empty()
            snapshot = dict(self._bal_last)
            snapshot["base"] = None
            snapshot["delta_mg"] = None
            return snapshot
        snapshot = self.bal_refresh()
        if snapshot["mg"] is None:
            raise ValueError("当前无有效重量读数，无法记录基准")
        self._bal_base = {"mg": snapshot["mg"], "at": snapshot["at"]}
        snapshot["base"] = self._bal_base
        snapshot["delta_mg"] = 0.0
        if self._bal_last is not None:      # 缓存同步，避免下次轮询 Δ 闪回空值
            self._bal_last["base"] = self._bal_base
            self._bal_last["delta_mg"] = 0.0
        _log(f"天平基准已记录：{snapshot['mg']} mg")
        return snapshot

    def _guard_no_dispense_running(self, action: str) -> None:
        """闭环加粉/标定运行中禁止任何手动动作。

        加粉期间振动频率/占空比/推杆开度全由控制器逐帧接管，此时人手插一脚
        （改振幅、动推杆、开搅拌）会让控制器基于错误的执行器状态继续算，
        轻则这一炉废掉，重则过冲超重。标定同理：它要的是固定参数下的稳态流量，
        中途被人改一个数，整张速率表就废了。停止/急停永远允许，不拦。
        """
        if self.dispense.running:
            raise InterlockError(
                f"闭环加粉运行中，禁止{action}：振动与推杆由控制器接管，"
                f"请先点「停止加粉」（或直接急停）")
        if self.calib.running:
            raise InterlockError(
                f"速率标定运行中，禁止{action}：标定要求参数固定，"
                f"请先停止标定（或直接急停）")

    def assert_idle_for_dispense(self) -> dict:
        """闭环加粉启动前的机构静止校验。

        与「天平去皮」同源但更严：加粉时振动会真的往秤上落粉，任何一个
        还在运动的机构都会让基线质量和闭环观测失真。
        """
        reasons: list[str] = []
        if self.sig.state.output_on:
            reasons.append("振动输出已开着")
        rpm = self._stir_speed_rpm()
        if rpm is None:
            raise InterlockError(
                "无法读取搅拌转速，不能确认搅拌已停，拒绝启动闭环加粉。请检查 COM9 后重试")
        if abs(rpm) > self.LIFT_STIR_INTERLOCK_RPM:
            reasons.append(f"搅拌在转（{rpm:.0f} RPM）")
        try:
            with self._bus10(self.CTRL_LOCK_TIMEOUT_S):
                statuses = self.lift.read_positions()
                rod = self.rod.read_status()
        except Exception as error:  # noqa: BLE001
            raise InterlockError(
                f"无法读取升降/推杆状态，不能确认机构已停，拒绝启动闭环加粉（{error}）") from error
        if any(status.moving for status in statuses):
            reasons.append("升降在运动")
        if abs(rod.current_position - rod.target_position) > 2:
            reasons.append(f"推杆在动（{rod.current_position}→{rod.target_position}）")
        if reasons:
            raise InterlockError(
                "闭环加粉启动被拒：" + "、".join(reasons) + "——请等机构完全停止后再开始")
        snapshot = self.bal_poll()
        if not snapshot.get("online"):
            raise InterlockError(
                f"天平不可用（{snapshot.get('error') or self.bal_port}），"
                f"没有天平就没有闭环，拒绝启动加粉")
        return {"rod_position": rod.current_position,
                "lift_positions": [status.position_raw for status in statuses]}

    def dispense_start(self, target_mg: Any, *, category: str | None = None,
                       preset_id: str | None = None,
                       amplitude_vpp: float | None = None,
                       tail_mass_mg: float | None = None,
                       powder: str | None = None,
                       use_feedforward: bool = True,
                       stages: list[dict] | None = None,
                       settle_gate: Any = None,
                       settle_gate_rate_mg_s: Any = None,
                       taper_factor: Any = None,
                       trim_enabled: Any = None,
                       trim_enter_remain_mg: Any = None,
                       trim_pulse_target_mg: Any = None) -> dict:
        if self.calib.running:
            raise InterlockError("速率标定正在运行，不能同时开始闭环加粉——请先停止标定")
        try:
            status = self.dispense.start(
                target_mg, category=category, preset_id=preset_id,
                amplitude_vpp=None if amplitude_vpp in (None, "") else float(amplitude_vpp),
                tail_mass_mg=None if tail_mass_mg in (None, "") else float(tail_mass_mg),
                powder=powder or None,
                use_feedforward=bool(use_feedforward),
                stages=stages or None,
                # 切段沉降门（重粉）：页面不传就走算法默认（开、≥30mg/s、0.6×速率）
                settle_gate=None if settle_gate in (None, "") else bool(settle_gate),
                settle_gate_rate_mg_s=(None if settle_gate_rate_mg_s in (None, "")
                                       else float(settle_gate_rate_mg_s)),
                taper_factor=None if taper_factor in (None, "") else float(taper_factor),
                # 缓补：页面不传 trim_enabled 就跟随粉体档案；两个数值留空用算法默认
                trim_enabled=None if trim_enabled in (None, "") else bool(trim_enabled),
                trim_enter_remain_mg=(None if trim_enter_remain_mg in (None, "")
                                      else float(trim_enter_remain_mg)),
                trim_pulse_target_mg=(None if trim_pulse_target_mg in (None, "")
                                      else float(trim_pulse_target_mg)))
        except Exception as error:
            _log(f"闭环加粉启动失败：{error}")
            raise
        return status

    def dispense_plan(self, target_mg: Any, *, powder: str | None = None,
                      category: str | None = None,
                      amplitude_vpp: float | None = None,
                      tail_mass_mg: float | None = None,
                      stages: list[dict] | None = None,
                      settle_gate: Any = None,
                      settle_gate_rate_mg_s: Any = None,
                      taper_factor: Any = None) -> dict:
        """只算三阶段计划、不动硬件：页面改目标质量 / 换粉时用它回填参数表。"""
        return self.dispense.preview(
            target_mg, powder=powder or None, category=category,
            amplitude_vpp=None if amplitude_vpp in (None, "") else float(amplitude_vpp),
            tail_mass_mg=None if tail_mass_mg in (None, "") else float(tail_mass_mg),
            stages=stages or None,
            settle_gate=None if settle_gate in (None, "") else bool(settle_gate),
            settle_gate_rate_mg_s=(None if settle_gate_rate_mg_s in (None, "")
                                   else float(settle_gate_rate_mg_s)),
            taper_factor=None if taper_factor in (None, "") else float(taper_factor))

    def dispense_stop(self) -> dict:
        return self.dispense.stop("操作员停止")

    # ------------------------------------------------------------------
    # 开环速率标定（粉体前馈表）
    # ------------------------------------------------------------------

    def calib_start(self, name: str, **kwargs: Any) -> dict:
        if self.dispense.running:
            raise InterlockError("闭环加粉正在运行，不能同时做标定——请先停止加粉")
        try:
            return self.calib.start(name, **kwargs)
        except Exception as error:
            _log(f"速率标定启动失败：{error}")
            raise

    def calib_stop(self) -> dict:
        return self.calib.stop("操作员停止")

    def calib_options(self) -> dict:
        """标定的默认工作点与允许范围（页面两个输入框可以直接改）。

        默认只取**一个**工作点：搅拌转速一个值、推杆开度一个值——标定就是"我要跑
        的那个工况出多少粉"，不需要凑三档。页面输入框可以改成任意合法值，也接受
        逗号分隔的多点做小范围扫描；这里返回的只是初值。
        """
        stir_defaults = [float(v) for v in CALIB_STIR_LEVELS]
        rod_defaults = [int(v) for v in CALIB_ROD_POINTS]
        return {
            "stir_levels": stir_defaults,
            "rod_points": rod_defaults,
            "points": len(stir_defaults) * len(rod_defaults),
            "limits": {
                "stir_min": 0.0,
                "stir_max": 1500.0,
                "rod_min": int(ROD_MIN_UNITS),
                "rod_max": int(ROD_MAX_UNITS),
            },
            "duration_s": CALIB_DEFAULT_DURATION_S,
            "frequency_hz": CALIB_FREQUENCY_HZ,
            "duty_percent": CALIB_DUTY_PERCENT,
            "amplitude_vpp": CALIB_AMPLITUDE_VPP,
            "categories": list(POWDER_CATEGORIES),
        }

    def powder_create(self, name: str, category: str | None = None) -> dict:
        if self.calib.running or self.dispense.running:
            raise InterlockError("加粉/标定运行中，禁止新建粉末档案")
        result = create_powder(name, category=category)
        _log(f"新建粉末档案：{result['name']}（{result['file']}）")
        return result

    def powder_list(self) -> dict:
        report = list_powders_report()
        self._log_powder_read_errors(report)
        return report

    def _log_powder_read_errors(self, report: dict) -> None:
        """粉体档案读不出来时必须留痕：同一批错误只记一次，避免页面轮询刷屏。"""
        errors = report.get("errors") or []
        signature = "|".join(f"{item.get('file')}:{item.get('reason')}" for item in errors)
        if signature == getattr(self, "_powder_err_sig", None):
            return
        self._powder_err_sig = signature
        if not errors:
            if signature != "":
                _log("粉体库读取恢复正常")
            return
        detail = "；".join(f"{item.get('file')}（{item.get('reason')}）" for item in errors[:6])
        extra = "" if len(errors) <= 6 else f" 等 {len(errors)} 个"
        _log(f"粉体库有 {len(errors)} 个档案读不出来{extra}：{detail}"
             + ("" if report.get("powders") else "；下拉框将显示为空"))

    def powder_detail(self, name: str) -> dict:
        try:
            return powder_detail(name)
        except ValueError as error:
            _log(f"读取粉体档案失败：{error}")
            raise

    def powder_forget(self, name: str) -> dict:
        if self.calib.running or self.dispense.running:
            raise InterlockError("加粉/标定运行中，禁止删除标定数据")
        return delete_feedforward(name)

    def powder_delete_points(self, name: str, indices: Any = None,
                             mode: str | None = None) -> dict:
        """「标定管理」删脏数据：按行删（indices）或一键删全部不可信点（mode=invalid）。"""
        if self.calib.running or self.dispense.running:
            raise InterlockError("加粉/标定运行中，禁止删除标定数据——先停下再整理标定表")
        raw = indices or []
        if isinstance(raw, (int, float, str)):
            raw = [raw]
        try:
            cleaned = [int(i) for i in raw]
        except (TypeError, ValueError) as error:
            raise ValueError(f"标定点索引不合法：{error}") from error
        result = delete_powder_points(name, cleaned, mode=mode)
        if result.get("removed"):
            _log(f"删除标定点：{result.get('name')} 删 {result['removed']} 点，"
                 f"剩余 {result['remaining']} 点"
                 f"（{'一键清理不可信' if mode == 'invalid' else '手动勾选'}）")
        return result

    # ------------------------------------------------------------------
    # 全局急停
    # ------------------------------------------------------------------

    def emergency_stop_all(self) -> dict:
        # 急停路径：锁等待也设上限，绝不能让急停被卡住（急停优先于一切）。
        # 先让加粉执行器停止接管（否则它会在下一轮观测后重新拉起振动），
        # 信号发生器再关（振动源），最后停运动机构。
        self.dispense.abort("全局急停")
        self.calib.abort("全局急停")
        try:
            self._sig_cancel.set()
            self._sig_deadline = None
            self._sig_call(lambda: self.sig.output(False), timeout_s=1.0)
        except Exception:  # noqa: BLE001 - 急停尽力而为
            pass
        try:
            with self._bus9(1.0):
                self.stir.stop_immediate()
        except Exception:  # noqa: BLE001 - 急停尽力而为
            pass
        try:
            with self._bus10(1.0):
                self.rod.emergency_stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            with self._bus10(1.0):
                # 用「广播均值同步急停」而不是逐台各停各的：两台同时减速，
                # 不会因为"先停一台、后停一台"把偏差继续拉大。
                self.lift.emergency_stop_both()
        except Exception:  # noqa: BLE001
            pass
        _log("全局急停执行（振动关 / 搅拌立即停 / 推杆急停 / 双机同步急停）")
        return {"emergency_stop": True}


class ControlServer(ThreadingHTTPServer):
    """禁止端口复用：Windows 下两个实例同时绑 8765 会静默分流请求，
    导致页面一半请求打到没有串口的僵尸实例上（表现为控件全部失灵）。"""

    allow_reuse_address = False


HUB: ActuatorHub  # 在 main() 中按命令行参数创建


class ControlPageHandler(BaseHTTPRequestHandler):
    """HTTP 路由：/ 页面 + /api/* JSON 接口。"""

    def log_message(self, fmt: str, *args: object) -> None:  # 静默访问日志
        pass

    # ------------------------------------------------------------------

    def _send_json(self, payload: dict, code: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self) -> dict:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    # ------------------------------------------------------------------

    def do_GET(self) -> None:  # noqa: N802 - http.server 命名
        if self.path in ("/", "/index.html"):
            page = (HERE / "index.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            # 页面是随代码一起改的，必须禁用缓存：否则浏览器拿旧 HTML，
            # 新加的互锁提示/按钮置灰不生效（改完还得教用户强刷）。
            self.send_header("Cache-Control", "no-store, must-revalidate")
            self.send_header("Content-Length", str(len(page)))
            self.end_headers()
            self.wfile.write(page)
        elif self.path == "/api/status":
            try:
                self._send_json(HUB.poll_status())
            except Exception as error:  # noqa: BLE001
                self._send_json({"ok": False, "errors": [str(error)]}, code=500)
        else:
            self._send_json({"error": "not found"}, code=404)

    def do_POST(self) -> None:  # noqa: N802 - http.server 命名
        def manual(action: str, handler: Callable[[dict], Any]) -> Callable[[dict], Any]:
            """手动操作包装：加粉运行期间一律拒绝（停止/急停/读数据不走这里）。"""
            def wrapped(body: dict) -> Any:
                HUB._guard_no_dispense_running(action)
                return handler(body)
            return wrapped

        routes = {
            "/api/lift/move": manual("升降运动", lambda body: HUB.lift_move(
                body["target"], body.get("speed"))),
            "/api/lift/home": manual("升降回原位", lambda body: HUB.lift_home()),
            "/api/lift/stop": lambda body: HUB.lift_stop(),
            "/api/lift/align": manual("升降对齐", lambda body: HUB.lift_align()),
            "/api/lift/ack": lambda body: HUB.acknowledge_sync_alarm(),
            "/api/rod/move": manual("推杆运动", lambda body: HUB.rod_move(body["target"])),
            "/api/rod/home": manual("推杆回原位", lambda body: HUB.rod_home()),
            "/api/rod/stop": lambda body: HUB.rod_stop(),
            "/api/rod/refresh": lambda body: HUB.rod_refresh(),
            "/api/stir/start": manual("启动搅拌", lambda body: HUB.stir_start(
                body.get("rpm", 0), body.get("acc", 0), body.get("direction", 0))),
            "/api/stir/stop": lambda body: HUB.stir_stop(
                body.get("mode", "immediate"), body.get("acc", 200)),
            "/api/stir/enable": manual("切换搅拌使能", lambda body: HUB.stir_enable(
                bool(body.get("on", True)))),
            "/api/stir/clear-stall": manual("解除搅拌堵转保护",
                                            lambda body: HUB.stir_clear_stall()),
            # ---- 振动给料（QLS3600S 信号发生器）----
            "/api/sig/waveform": manual("切换波形", lambda body: HUB.sig_set_waveform(body["waveform"])),
            "/api/sig/freq": manual("改振动频率", lambda body: HUB.sig_set_frequency(body["hz"])),
            "/api/sig/amp": manual("改振动幅度", lambda body: HUB.sig_set_amplitude(body["vpp"])),
            "/api/sig/duty": manual("改振动占空比", lambda body: HUB.sig_set_duty(body["percent"])),
            # 开输出要拦（会和闭环抢振动），关输出永远允许（关掉是最安全的方向）
            "/api/sig/output": lambda body: HUB.sig_output_user(bool(body.get("on", True))),
            "/api/sig/run": manual("定时振动", lambda body: HUB.sig_run(
                body["seconds"], frequency_hz=body.get("frequency_hz"),
                amplitude_vpp=body.get("amplitude_vpp"),
                waveform=body.get("waveform"))),
            # 猝发脉冲（真脉冲：设备数完 count 个周期自动停，不用软件掐表）
            "/api/sig/burst": manual("猝发脉冲", lambda body: HUB.sig_burst(
                int(body["count"]), frequency_hz=body.get("frequency_hz"),
                amplitude_vpp=body.get("amplitude_vpp"))),
            "/api/sig/stop": lambda body: HUB.sig_stop(),
            # ---- 天平（AT8811C，COM11）----
            "/api/bal/refresh": lambda body: HUB.bal_refresh(),
            # 加粉途中去皮会把闭环的基线干掉，必须拦
            "/api/bal/tare": manual("天平去皮归零", lambda body: HUB.bal_tare()),
            "/api/bal/stable": lambda body: HUB.bal_stable(
                body.get("timeout_s", 6.0), body.get("samples", 8)),
            "/api/bal/base": lambda body: HUB.bal_set_base(
                clear=bool(body.get("clear", False))),
            # ---- 闭环加粉（dispensing_algorithm）----
            "/api/dispense/start": lambda body: HUB.dispense_start(
                body["target_mg"], category=body.get("category"),
                preset_id=body.get("preset_id"),
                amplitude_vpp=body.get("amplitude_vpp"),
                tail_mass_mg=body.get("tail_mass_mg"),
                powder=body.get("powder"),
                use_feedforward=body.get("use_feedforward", True),
                stages=body.get("stages"),
                settle_gate=body.get("settle_gate"),
                settle_gate_rate_mg_s=body.get("settle_gate_rate_mg_s"),
                taper_factor=body.get("taper_factor"),
                trim_enabled=body.get("trim_enabled"),
                trim_enter_remain_mg=body.get("trim_enter_remain_mg"),
                trim_pulse_target_mg=body.get("trim_pulse_target_mg")),
            "/api/dispense/plan": lambda body: HUB.dispense_plan(
                body["target_mg"], powder=body.get("powder"),
                category=body.get("category"),
                amplitude_vpp=body.get("amplitude_vpp"),
                tail_mass_mg=body.get("tail_mass_mg"),
                stages=body.get("stages"),
                settle_gate=body.get("settle_gate"),
                settle_gate_rate_mg_s=body.get("settle_gate_rate_mg_s"),
                taper_factor=body.get("taper_factor")),
            "/api/dispense/stop": lambda body: HUB.dispense_stop(),
            # ---- 粉体前馈库 + 开环速率标定 ----
            "/api/powder/list": lambda body: HUB.powder_list(),
            "/api/powder/detail": lambda body: HUB.powder_detail(body["name"]),
            "/api/powder/forget": lambda body: HUB.powder_forget(body["name"]),
            # 「标定管理」：勾选删点 / 一键删全部不可信点（mode=invalid）
            "/api/powder/points/delete": lambda body: HUB.powder_delete_points(
                body["name"], body.get("indices"), body.get("mode")),
            "/api/powder/create": lambda body: HUB.powder_create(
                body["name"], body.get("category")),
            "/api/calib/options": lambda body: HUB.calib_options(),
            "/api/calib/start": lambda body: HUB.calib_start(
                body["name"],
                category=body.get("category"),
                stir_levels=body.get("stir_levels"),
                rod_points=body.get("rod_points"),
                frequency_hz=body.get("frequency_hz", 60.0),
                duty_percent=body.get("duty_percent", 20.0),
                amplitude_vpp=body.get("amplitude_vpp", 0.3),
                duration_s=body.get("duration_s", 6.0)),
            "/api/calib/stop": lambda body: HUB.calib_stop(),
            "/api/estop": lambda body: HUB.emergency_stop_all(),
            # 页面日志落盘：浏览器内存里的日志刷新即丢，重启就再也查不到，
            # 这里让前端把关键日志同步写进 logs/control-YYYY-MM-DD.log。
            "/api/log": lambda body: _log_page_event(body),
        }
        handler = routes.get(self.path)
        if handler is None:
            self._send_json({"error": "not found"}, code=404)
            return
        try:
            body = self._read_json_body()
            self._send_json(handler(body))
        except InterlockError as error:
            # 安全互锁：原样回消息（页面直接把这句话显示给用户）。
            self._send_json({"error": str(error), "interlock": True}, code=409)
        except (KeyError, ValueError, TypeError) as error:
            self._send_json({"error": f"bad request: {error}"}, code=400)
        except Exception as error:  # noqa: BLE001 - 设备通信错误如实回传页面
            self._send_json({"error": str(error)}, code=502)


def main() -> None:
    global HUB
    parser = argparse.ArgumentParser(description="粉末加样执行机构实时控制页面")
    parser.add_argument("--lift-port", default=LIFT_PORT, help="双 MD30 升降 + LA 推杆 共享串口")
    parser.add_argument("--stir-port", default=STIR_PORT, help="ZDT 搅拌电机串口")
    parser.add_argument("--sig-port", default=SIG_PORT, help="QLS3600S 信号发生器串口")
    parser.add_argument("--bal-port", default=BAL_PORT, help="AT8811C 天平串口")
    parser.add_argument("--http-port", type=int, default=8765, help="HTTP 服务端口")
    args = parser.parse_args()

    # 按命令行参数创建设备聚合层（避免运行期改写 frozen dataclass）。
    HUB = ActuatorHub(args.lift_port, args.stir_port, args.sig_port, args.bal_port)

    # 信号发生器参数无法回读，启动时补一次默认值让页面显示完整（输出保持关闭）。
    HUB.sig_apply_defaults()
    # 上个进程若被强杀，搅拌/振动可能残留动作；启动先复位到静止态。
    HUB.startup_reset()

    # 同步看门狗独立于页面运行：页面关掉也照样保护"双机必须一起动"。
    HUB.start_watchdog()
    _log(f"服务启动 lift={args.lift_port} stir={args.stir_port} "
         f"sig={args.sig_port} bal={args.bal_port} http=127.0.0.1:{args.http_port}")
    _log(f"升降参数：速度固定 {LIFT_SPEED_RAW}，目标范围 {LIFT_MIN_RAW}..{LIFT_MAX_RAW}，"
         f"同步容差 {HUB.sync_tolerance}")

    # 启动自检：粉体库能不能读。以前读不出来只有页面显示"空列表"，日志里一个字都没有。
    try:
        _powder_report = list_powders_report()
        if _powder_report.get("errors"):
            _log(f"粉体库自检失败：{_powder_report.get('files')} 个档案里 "
                 f"{len(_powder_report['errors'])} 个读不出来（首个："
                 f"{_powder_report['errors'][0].get('file')} / "
                 f"{_powder_report['errors'][0].get('reason')}）"
                 f"；页面下拉框会是空的，但档案文件并没有丢")
        else:
            _log(f"粉体库自检正常：{len(_powder_report.get('powders') or [])} 个档案可读")
    except Exception as error:  # 自检本身不许影响启动
        _log(f"粉体库自检异常：{error}")

    try:
        server = ControlServer(("127.0.0.1", args.http_port), ControlPageHandler)
    except OSError as error:
        print(f"启动失败：端口 {args.http_port} 被占用——大概率已有一个控制页面实例在运行。")
        print(f"  （{error}）请勿重复启动 app.py；如需重启，先结束旧的 python 进程。")
        raise SystemExit(1) from error
    print(f"控制页面已启动: http://127.0.0.1:{args.http_port}/")
    print(f"  升降+推杆总线: {args.lift_port} (MD30 从站 1/2 + LA ID3)")
    print(f"  搅拌电机串口 : {args.stir_port} (ZDT 私有 0x6B)")
    print(f"  信号发生器   : {args.sig_port} (QLS3600S 振动给料)")
    print(f"  天平         : {args.bal_port} (AT8811C Modbus RTU 9600，1 count = 1 mg)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n正在关闭…")
    finally:
        HUB.stop_watchdog()
        HUB.emergency_stop_all()
        HUB.lift.close()
        HUB.rod.close()
        HUB.stir.close()
        HUB.sig.close()
        HUB.bal.close()
        server.server_close()
        _log("服务已关闭")


if __name__ == "__main__":
    main()
