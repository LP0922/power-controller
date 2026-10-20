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

    def __init__(self, lift_port: str, stir_port: str) -> None:
        self.lift_port = lift_port
        self.stir_port = stir_port
        # COM10 共享总线锁：MD30（Modbus）与 LA（55AA 协议）事务互斥。
        self._bus10_lock = threading.Lock()
        # COM9 锁：ZDT 搅拌。
        self._bus9_lock = threading.Lock()

        # Windows 上同一串口只允许一个句柄：COM10 上的 MD30 与 LA 必须共享
        # 同一个已打开的串口对象，互斥由 _bus10_lock 保证。
        import serial as _serial_mod
        self._shared_bus10 = _serial_mod.Serial(
            port=lift_port, baudrate=115200, bytesize=8, parity="N",
            stopbits=1, timeout=0.3)

        self.lift = MD30DualLift(port=lift_port, slave_addresses=(1, 2),
                                 default_speed=30,
                                 serial_factory=lambda **kwargs: self._shared_bus10)
        self.rod = LAActuator(LAConfig(port=lift_port, node_id=3),
                              serial_port=self._shared_bus10)
        self.stir = ZDTEmm42Private(Emm42PrivateConfig(port=stir_port, slave_address=1))

    # ------------------------------------------------------------------
    # 状态轮询
    # ------------------------------------------------------------------

    def poll_status(self) -> dict:
        status: dict = {"lift": None, "rod": None, "stir": None, "errors": []}
        with self._bus10_lock:
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
            try:
                rod = self.rod.read_status()
                status["rod"] = {"pos": rod.current_position, "target": rod.target_position,
                                 "temp": rod.temperature_c, "current": rod.current_ma,
                                 "error": rod.error_bits}
            except Exception as error:  # noqa: BLE001
                status["errors"].append(f"rod: {error}")
        with self._bus9_lock:
            try:
                stir = self.stir.read_status()
                status["stir"] = {"rpm": stir.speed_rpm, "flags": stir.state_flags,
                                  "enabled": stir.enabled}
            except Exception as error:  # noqa: BLE001
                status["errors"].append(f"stir: {error}")
        status["ok"] = not status["errors"]
        return status

    # ------------------------------------------------------------------
    # 升降（双 MD30，广播同步）
    # ------------------------------------------------------------------

    def lift_move(self, target: int, speed: int | None = None) -> dict:
        with self._bus10_lock:
            self.lift.move_both(int(target), speed_raw=speed)
            statuses = self.lift.wait_until_stopped(timeout_s=30.0)
        return {"lift": [{"addr": d.config.slave_address, "pos": s.position_raw,
                          "moving": s.moving} for d, s in zip(self.lift._drivers, statuses)]}

    def lift_home(self, speed: int | None = None) -> dict:
        return self.lift_move(LIFT_HOME_RAW, speed=speed)

    def lift_stop(self) -> dict:
        with self._bus10_lock:
            self.lift.stop_both()
        return {"stopped": True}

    # ------------------------------------------------------------------
    # 推杆（LA）
    # ------------------------------------------------------------------

    def rod_move(self, target: int) -> dict:
        with self._bus10_lock:
            self.rod.move_to(int(target), feedback=True)
            position = self.rod.wait_until_stopped(target=int(target), timeout_s=15.0)
        return {"pos": position}

    def rod_home(self) -> dict:
        return self.rod_move(ROD_HOME)

    def rod_stop(self) -> dict:
        with self._bus10_lock:
            self.rod.emergency_stop()
        return {"stopped": True}

    # ------------------------------------------------------------------
    # 旋转搅拌（ZDT）
    # ------------------------------------------------------------------

    def stir_enable(self, on: bool) -> dict:
        with self._bus9_lock:
            self.stir.enable(on)
        return {"enabled": on}

    def stir_start(self, rpm: float, acc: int, direction: int) -> dict:
        with self._bus9_lock:
            self.stir.enable(True)
            self.stir.set_speed(float(rpm), acceleration=int(acc), direction=int(direction))
        return {"rpm": rpm, "acc": acc, "direction": direction}

    def stir_stop(self, mode: str = "immediate", acc: int = 200) -> dict:
        with self._bus9_lock:
            if mode == "decel":
                self.stir.stop_decelerated(acceleration=int(acc))
            else:
                self.stir.stop_immediate()
        return {"mode": mode}

    # ------------------------------------------------------------------
    # 全局急停
    # ------------------------------------------------------------------

    def emergency_stop_all(self) -> dict:
        # 搅拌立即停（COM9 与 COM10 无关，可并行；但简化为顺序，帧都很短）。
        try:
            with self._bus9_lock:
                self.stir.stop_immediate()
        except Exception:  # noqa: BLE001 - 急停尽力而为
            pass
        try:
            with self._bus10_lock:
                self.rod.emergency_stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            with self._bus10_lock:
                self.lift.stop_both()
        except Exception:  # noqa: BLE001
            pass
        return {"emergency_stop": True}


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

    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), ControlPageHandler)
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
