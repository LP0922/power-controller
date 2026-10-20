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

from powder_sampling_control.device_adapters import (
    LAActuator,
    LAConfig,
    MD30DualLift,
    MD30SyncGuardError,
    QLS3600Config,
    QLS3600SignalGenerator,
    ZDTEmm42Private,
    Emm42PrivateConfig,
)

# ----------------------------------------------------------------------
# 默认硬件配置（与 2026-10-20 实测一致）
# ----------------------------------------------------------------------

LIFT_PORT = "COM10"      # 双 MD30 升降 + LA 推杆 共享总线
STIR_PORT = "COM9"       # ZDT 搅拌电机
SIG_PORT = "COM16"       # QLS3600S 信号发生器（振动给料）
LIFT_HOME_RAW = 33       # 升降原位（实测收拢位 32/33）
ROD_HOME = 208           # 推杆原位（实测起点）
# 升降速度固定为 10（用户要求：慢速给看门狗留足轮询反应时间，页面不可调）。
LIFT_SPEED_RAW = 10

HERE = Path(__file__).resolve().parent


class ActuatorHub:
    """三个执行器的聚合管理：串口锁 + 设备生命周期 + 统一急停。"""

    # 锁等待上限：串口操作一旦挂死，绝不能让 HTTP 线程无限排队（页面假死的根源）。
    POLL_LOCK_TIMEOUT_S = 0.3   # 状态轮询：拿不到锁就跳过本轮，回缓存数据
    CTRL_LOCK_TIMEOUT_S = 3.0   # 控制指令：最多等 3s，拿不到就报错给页面

    def __init__(self, lift_port: str, stir_port: str, sig_port: str = SIG_PORT) -> None:
        self.lift_port = lift_port
        self.stir_port = stir_port
        self.sig_port = sig_port
        # COM10 共享总线锁：MD30（Modbus）与 LA（55AA 协议）事务互斥。
        self._bus10_lock = threading.Lock()
        # COM9 锁：ZDT 搅拌。
        self._bus9_lock = threading.Lock()
        # COM16 锁：QLS3600 信号发生器（独立串口，无需共享）。
        self._bus16_lock = threading.Lock()
        # 最近一次成功的状态缓存（轮询拿不到锁时返回它，页面不会断流）。
        self._last_status: dict = {"lift": None, "rod": None, "stir": None}
        # 共享串口句柄惰性创建：设备掉线/重插后自动重开，服务进程不退出。
        self._shared_bus10 = None
        # 同步保障（保障 3：运行中看门狗）——锁存告警直到页面确认恢复。
        self.sync_tolerance = MD30DualLift.SYNC_TOLERANCE_RAW
        self._sync_alarm: dict | None = None
        self._sync_alarm_lock = threading.Lock()
        # 振动定时运行：后台线程到点关输出（不阻塞 HTTP）。
        self._sig_cancel = threading.Event()
        self._sig_deadline: float | None = None

        self.lift = MD30DualLift(port=lift_port, slave_addresses=(1, 2),
                                 default_speed=LIFT_SPEED_RAW,
                                 serial_factory=lambda **kwargs: self._get_bus10())
        self.rod = LAActuator(LAConfig(port=lift_port, node_id=3),
                              serial_factory=lambda **kwargs: self._get_bus10())
        self.stir = ZDTEmm42Private(Emm42PrivateConfig(port=stir_port, slave_address=1))
        self.sig = QLS3600SignalGenerator(QLS3600Config(port=sig_port))

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
        """信号发生器总线：串口 I/O 出错时丢弃句柄，下条指令自动重连（USB 掉线自恢复）。"""
        if not self._bus16_lock.acquire(timeout=timeout_s):
            raise TimeoutError(f"{self.sig_port} 总线忙（等锁超时 {timeout_s}s）")
        try:
            yield
        except OSError:
            self.sig._serial = None
            raise
        finally:
            self._bus16_lock.release()

    # ------------------------------------------------------------------
    # 状态轮询
    # ------------------------------------------------------------------

    def _raise_sync_alarm(self, message: str, positions: list[int] | None = None) -> None:
        """锁存同步告警（页面持续红条显示，直到用户确认恢复）。"""
        with self._sync_alarm_lock:
            if self._sync_alarm is None:  # 只记第一条，避免刷屏覆盖
                self._sync_alarm = {"message": message,
                                    "positions": positions,
                                    "time": time.strftime("%H:%M:%S")}

    def acknowledge_sync_alarm(self) -> dict:
        """清除锁存的同步告警（用户在页面上点「确认恢复」）。"""
        with self._sync_alarm_lock:
            alarm = self._sync_alarm
            self._sync_alarm = None
        return {"acknowledged": True, "cleared": alarm}

    def _sync_watchdog(self, statuses: list) -> None:
        """保障 3：运行中偏差看门狗（每轮状态轮询都会执行）。

        任一电机在运动且双机偏差超容差 → 立即同步急停双机 + 锁存告警。
        """
        deviation = abs(statuses[0].position_raw - statuses[1].position_raw)
        any_moving = any(s.moving for s in statuses)
        if any_moving and deviation > self.sync_tolerance:
            positions = [s.position_raw for s in statuses]
            try:
                self.lift.emergency_stop_both()
                message = (f"运行中双机偏差 {deviation} 超容差 "
                           f"{self.sync_tolerance}，已同步急停双机")
            except Exception as error:  # noqa: BLE001 - 急停失败更要大声报警
                message = (f"运行中双机偏差 {deviation} 超容差，且急停执行失败"
                           f"（{error}）——请立即断电！")
            self._raise_sync_alarm(message, positions)

    def poll_status(self) -> dict:
        status: dict = {"lift": None, "rod": None, "stir": None, "errors": [], "stale": False}
        try:
            with self._bus10(self.POLL_LOCK_TIMEOUT_S):
                try:
                    statuses = self.lift.read_positions()
                    status["lift"] = [
                        {"addr": d.config.slave_address,
                         "pos": s.position_raw, "moving": s.moving,
                         "speed": s.speed_raw, "voltage": s.voltage_raw}
                        for d, s in zip(self.lift._drivers, statuses)
                    ]
                    # 保障 3：看门狗（持有总线锁内执行急停，避免与控制指令竞争）。
                    if len(statuses) == 2:
                        self._sync_watchdog(statuses)
                except Exception as error:  # noqa: BLE001 - 轮询不容错会中断页面
                    status["errors"].append(f"lift: {error}")
                    self._reset_bus10()  # 串口级错误：丢弃句柄，下轮自动重连
                try:
                    rod = self.rod.read_status()
                    status["rod"] = {"pos": rod.current_position, "target": rod.target_position,
                                     "temp": rod.temperature_c, "current": rod.current_ma,
                                     "error": rod.error_bits}
                except Exception as error:  # noqa: BLE001
                    status["errors"].append(f"rod: {error}")
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
                    self.stir.close()  # 串口级错误：丢弃句柄，下轮自动重连
        except TimeoutError:
            status["stale"] = True
            status["stir"] = self._last_status["stir"]
        if status["stir"] is not None:
            self._last_status["stir"] = status["stir"]
        # 同步保障信息：当前偏差 + 锁存告警（页面据此显示告警红条）。
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

    def lift_move(self, target: int, speed: int | None = None) -> dict:
        # 只下发目标、立即返回：运动过程由页面 500ms 状态轮询实时刷新。
        # 安全约束：速度固定 LIFT_SPEED_RAW（慢速），页面传值一律忽略。
        with self._bus10(self.CTRL_LOCK_TIMEOUT_S):
            self.lift.move_both(int(target), speed_raw=LIFT_SPEED_RAW)
        return {"target": int(target), "sent": True, "speed": LIFT_SPEED_RAW}

    def lift_home(self, speed: int | None = None) -> dict:
        return self.lift_move(LIFT_HOME_RAW)

    def lift_stop(self) -> dict:
        with self._bus10(self.CTRL_LOCK_TIMEOUT_S):
            self.lift.stop_both()
        return {"stopped": True}

    def lift_align(self, speed: int | None = None) -> dict:
        """对齐双机（把两台都移到当前位置中点，消除起跳前偏差）。"""
        with self._bus10(15.0):  # 对齐含等待停止，允许更长时间
            result = self.lift.align(speed_raw=speed)
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
        return {"target": int(target), "sent": True}

    def rod_home(self) -> dict:
        return self.rod_move(ROD_HOME)

    def rod_stop(self) -> dict:
        with self._bus10(self.CTRL_LOCK_TIMEOUT_S):
            self.rod.emergency_stop()
        return {"stopped": True}

    # ------------------------------------------------------------------
    # 旋转搅拌（ZDT）
    # ------------------------------------------------------------------

    def stir_enable(self, on: bool) -> dict:
        with self._bus9(self.CTRL_LOCK_TIMEOUT_S):
            self.stir.enable(on)
        return {"enabled": on}

    def stir_start(self, rpm: float, acc: int, direction: int) -> dict:
        with self._bus9(self.CTRL_LOCK_TIMEOUT_S):
            self.stir.enable(True)
            self.stir.set_speed(float(rpm), acceleration=int(acc), direction=int(direction))
        return {"rpm": rpm, "acc": acc, "direction": direction}

    def stir_stop(self, mode: str = "immediate", acc: int = 200) -> dict:
        with self._bus9(self.CTRL_LOCK_TIMEOUT_S):
            if mode == "decel":
                self.stir.stop_decelerated(acceleration=int(acc))
            else:
                self.stir.stop_immediate()
        return {"mode": mode}

    # ------------------------------------------------------------------
    # 振动给料（QLS3600S 信号发生器）
    # ------------------------------------------------------------------

    def sig_snapshot(self) -> dict:
        """信号发生器状态快照：设定值取驱动缓存，定时剩余由本类计算。"""
        state = self.sig.state
        remaining = None
        if self._sig_deadline is not None:
            remaining = max(0.0, round(self._sig_deadline - time.monotonic(), 1))
        # 方波 + 非 50% 占空比 = 脉冲波（设备无独立脉冲编号），显示上标出来。
        display_name = state.waveform_name
        if state.waveform == 1 and state.duty_percent is not None:
            display_name = f"脉冲波 {state.duty_percent}%"
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
            mapping = {"sine": 0, "square": 1, "triangle": 2}
            if key not in mapping:
                raise ValueError(f"未知波形 {waveform!r}（可用 sine/square/triangle/pulse）")
            self.sig.set_waveform(mapping[key])
            return
        self.sig.set_waveform(int(waveform))

    def sig_set_waveform(self, waveform: object) -> dict:
        with self._bus16(self.CTRL_LOCK_TIMEOUT_S):
            self._apply_waveform(waveform)
        return self.sig_snapshot()

    def sig_set_frequency(self, hz: float) -> dict:
        with self._bus16(self.CTRL_LOCK_TIMEOUT_S):
            self.sig.set_frequency(float(hz))
        return self.sig_snapshot()

    def sig_set_amplitude(self, vpp: float) -> dict:
        with self._bus16(self.CTRL_LOCK_TIMEOUT_S):
            self.sig.set_amplitude(float(vpp))
        return self.sig_snapshot()

    def sig_set_duty(self, percent: float) -> dict:
        with self._bus16(self.CTRL_LOCK_TIMEOUT_S):
            self.sig.set_duty(float(percent))
        return self.sig_snapshot()

    def sig_output(self, on: bool) -> dict:
        """OUT 输出开关（手动开/关，直接生效）。"""
        if not on:
            self._sig_cancel.set()          # 手动关输出同时取消定时任务
            self._sig_deadline = None
        with self._bus16(self.CTRL_LOCK_TIMEOUT_S):
            self.sig.output(bool(on))
        return self.sig_snapshot()

    def sig_run(self, seconds: float, *, frequency_hz: float | None = None,
                amplitude_vpp: float | None = None, waveform: object | None = None) -> dict:
        """定时振动：按需设置参数后开 OUT，运行 seconds 秒自动关。

        定时靠后台线程，接口立即返回；页面用状态轮询显示剩余时间。
        """
        seconds = float(seconds)
        if not 0 < seconds <= 600:
            raise ValueError("定时运行时长须在 0–600 秒")
        self._sig_cancel.set()              # 取消上一轮定时
        with self._bus16(self.CTRL_LOCK_TIMEOUT_S):
            if waveform is not None:
                self._apply_waveform(waveform)
            if frequency_hz is not None:
                self.sig.set_frequency(float(frequency_hz))
            if amplitude_vpp is not None:
                self.sig.set_amplitude(float(amplitude_vpp))
            self.sig.output(True)
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
            with self._bus16(self.CTRL_LOCK_TIMEOUT_S):
                self.sig.output(False)
        except Exception:  # noqa: BLE001 - 定时关闭失败不能让线程崩
            pass
        finally:
            if self._sig_cancel is cancel:
                self._sig_deadline = None

    def sig_stop(self) -> dict:
        """停止振动：取消定时并立即关 OUT（页面「停止」按钮）。"""
        self._sig_cancel.set()
        self._sig_deadline = None
        with self._bus16(self.CTRL_LOCK_TIMEOUT_S):
            self.sig.output(False)
        return self.sig_snapshot()

    # ------------------------------------------------------------------
    # 全局急停
    # ------------------------------------------------------------------

    def emergency_stop_all(self) -> dict:
        # 急停路径：锁等待也设上限，绝不能让急停被卡住（急停优先于一切）。
        # 信号发生器先关（振动源），再停运动机构。
        try:
            self._sig_cancel.set()
            self._sig_deadline = None
            with self._bus16(1.0):
                self.sig.output(False)
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
                self.lift.stop_both()
        except Exception:  # noqa: BLE001
            pass
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
        routes = {
            "/api/lift/move": lambda body: HUB.lift_move(body["target"], body.get("speed")),
            "/api/lift/home": lambda body: HUB.lift_home(),
            "/api/lift/stop": lambda body: HUB.lift_stop(),
            "/api/lift/align": lambda body: HUB.lift_align(),
            "/api/lift/ack": lambda body: HUB.acknowledge_sync_alarm(),
            "/api/rod/move": lambda body: HUB.rod_move(body["target"]),
            "/api/rod/home": lambda body: HUB.rod_home(),
            "/api/rod/stop": lambda body: HUB.rod_stop(),
            "/api/stir/start": lambda body: HUB.stir_start(
                body.get("rpm", 0), body.get("acc", 0), body.get("direction", 0)),
            "/api/stir/stop": lambda body: HUB.stir_stop(
                body.get("mode", "immediate"), body.get("acc", 200)),
            "/api/stir/enable": lambda body: HUB.stir_enable(bool(body.get("on", True))),
            # ---- 振动给料（QLS3600S 信号发生器）----
            "/api/sig/waveform": lambda body: HUB.sig_set_waveform(body["waveform"]),
            "/api/sig/freq": lambda body: HUB.sig_set_frequency(body["hz"]),
            "/api/sig/amp": lambda body: HUB.sig_set_amplitude(body["vpp"]),
            "/api/sig/duty": lambda body: HUB.sig_set_duty(body["percent"]),
            "/api/sig/output": lambda body: HUB.sig_output(bool(body.get("on", True))),
            "/api/sig/run": lambda body: HUB.sig_run(
                body["seconds"], frequency_hz=body.get("frequency_hz"),
                amplitude_vpp=body.get("amplitude_vpp"),
                waveform=body.get("waveform")),
            "/api/sig/stop": lambda body: HUB.sig_stop(),
            "/api/estop": lambda body: HUB.emergency_stop_all(),
        }
        handler = routes.get(self.path)
        if handler is None:
            self._send_json({"error": "not found"}, code=404)
            return
        try:
            body = self._read_json_body()
            self._send_json(handler(body))
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
    parser.add_argument("--http-port", type=int, default=8765, help="HTTP 服务端口")
    args = parser.parse_args()

    # 按命令行参数创建设备聚合层（避免运行期改写 frozen dataclass）。
    HUB = ActuatorHub(args.lift_port, args.stir_port, args.sig_port)

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
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n正在关闭…")
    finally:
        HUB.emergency_stop_all()
        HUB.lift.close()
        HUB.rod.close()
        HUB.stir.close()
        HUB.sig.close()
        server.server_close()


if __name__ == "__main__":
    main()
