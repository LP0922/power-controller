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
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from powder_sampling_control.device_adapters import (
    LAActuator,
    LAConfig,
    MD30DualLift,
    ZDTEmm42Private,
    Emm42PrivateConfig,
)

# ----------------------------------------------------------------------
# 默认硬件配置（与 2026-10-20 实测一致）
# ----------------------------------------------------------------------

LIFT_PORT = "COM10"      # 双 MD30 升降 + LA 推杆 共享总线
STIR_PORT = "COM9"       # ZDT 搅拌电机
LIFT_HOME_RAW = 33       # 升降原位（实测收拢位 32/33）
ROD_HOME = 208           # 推杆原位（实测起点）

HERE = Path(__file__).resolve().parent


class ActuatorHub:
    """三个执行器的聚合管理：串口锁 + 设备生命周期 + 统一急停。"""

    # 锁等待上限：串口操作一旦挂死，绝不能让 HTTP 线程无限排队（页面假死的根源）。
    POLL_LOCK_TIMEOUT_S = 0.3   # 状态轮询：拿不到锁就跳过本轮，回缓存数据
    CTRL_LOCK_TIMEOUT_S = 3.0   # 控制指令：最多等 3s，拿不到就报错给页面

    def __init__(self, lift_port: str, stir_port: str) -> None:
        self.lift_port = lift_port
        self.stir_port = stir_port
        # COM10 共享总线锁：MD30（Modbus）与 LA（55AA 协议）事务互斥。
        self._bus10_lock = threading.Lock()
        # COM9 锁：ZDT 搅拌。
        self._bus9_lock = threading.Lock()
        # 最近一次成功的状态缓存（轮询拿不到锁时返回它，页面不会断流）。
        self._last_status: dict = {"lift": None, "rod": None, "stir": None}
        # 共享串口句柄惰性创建：设备掉线/重插后自动重开，服务进程不退出。
        self._shared_bus10 = None

        self.lift = MD30DualLift(port=lift_port, slave_addresses=(1, 2),
                                 default_speed=30,
                                 serial_factory=lambda **kwargs: self._get_bus10())
        self.rod = LAActuator(LAConfig(port=lift_port, node_id=3),
                              serial_factory=lambda **kwargs: self._get_bus10())
        self.stir = ZDTEmm42Private(Emm42PrivateConfig(port=stir_port, slave_address=1))

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

    # ------------------------------------------------------------------
    # 状态轮询
    # ------------------------------------------------------------------

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
        status["ok"] = not status["errors"]
        return status

    # ------------------------------------------------------------------
    # 升降（双 MD30，广播同步）
    # ------------------------------------------------------------------

    def lift_move(self, target: int, speed: int | None = None) -> dict:
        # 只下发目标、立即返回：运动过程由页面 500ms 状态轮询实时刷新。
        with self._bus10(self.CTRL_LOCK_TIMEOUT_S):
            self.lift.move_both(int(target), speed_raw=speed)
        return {"target": int(target), "sent": True}

    def lift_home(self, speed: int | None = None) -> dict:
        return self.lift_move(LIFT_HOME_RAW, speed=speed)

    def lift_stop(self) -> dict:
        with self._bus10(self.CTRL_LOCK_TIMEOUT_S):
            self.lift.stop_both()
        return {"stopped": True}

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
    # 全局急停
    # ------------------------------------------------------------------

    def emergency_stop_all(self) -> dict:
        # 急停路径：锁等待也设上限，绝不能让急停被卡住（急停优先于一切）。
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
            "/api/lift/home": lambda body: HUB.lift_home(body.get("speed")),
            "/api/lift/stop": lambda body: HUB.lift_stop(),
            "/api/rod/move": lambda body: HUB.rod_move(body["target"]),
            "/api/rod/home": lambda body: HUB.rod_home(),
            "/api/rod/stop": lambda body: HUB.rod_stop(),
            "/api/stir/start": lambda body: HUB.stir_start(
                body.get("rpm", 0), body.get("acc", 0), body.get("direction", 0)),
            "/api/stir/stop": lambda body: HUB.stir_stop(
                body.get("mode", "immediate"), body.get("acc", 200)),
            "/api/stir/enable": lambda body: HUB.stir_enable(bool(body.get("on", True))),
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
    parser.add_argument("--http-port", type=int, default=8765, help="HTTP 服务端口")
    args = parser.parse_args()

    # 按命令行参数创建设备聚合层（避免运行期改写 frozen dataclass）。
    HUB = ActuatorHub(args.lift_port, args.stir_port)

    try:
        server = ControlServer(("127.0.0.1", args.http_port), ControlPageHandler)
    except OSError as error:
        print(f"启动失败：端口 {args.http_port} 被占用——大概率已有一个控制页面实例在运行。")
        print(f"  （{error}）请勿重复启动 app.py；如需重启，先结束旧的 python 进程。")
        raise SystemExit(1) from error
    print(f"控制页面已启动: http://127.0.0.1:{args.http_port}/")
    print(f"  升降+推杆总线: {args.lift_port} (MD30 从站 1/2 + LA ID3)")
    print(f"  搅拌电机串口 : {args.stir_port} (ZDT 私有 0x6B)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n正在关闭…")
    finally:
        HUB.emergency_stop_all()
        HUB.lift.close()
        HUB.rod.close()
        HUB.stir.close()
        server.server_close()


if __name__ == "__main__":
    main()
