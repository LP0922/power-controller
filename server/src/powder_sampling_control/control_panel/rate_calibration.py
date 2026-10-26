"""开环速率标定：输入粉名 → 逐点测出这种粉"什么开度、什么搅拌转速出多少粉"。

为什么要单独做这一步
--------------------
闭环控制器只会"往目标上修"，它不知道**这个粉在某个开度下本来能出多少**。没有这个
前置关系，每换一种粉、每换一个开度，都只能靠控制器从零试错，粗加阶段白白浪费十几秒，
精加阶段又容易一脚踩空。

标定做的事很朴素：固定振动参数，把 (搅拌转速, 推杆开度) 扫一遍，每点开环喷 T 秒，
用天平斜率算出稳定速率。得到的数据写进 ``data/powder_library/<粉名>.json`` 的
``feedforward_v2`` 段，闭环加粉启动时按阶段目标速率反解推杆开度当作前馈种子。

测量口径
--------
- **速率取振动期间的天平斜率**（去掉前 20% 启动段与末尾 0.2s），不用"停振后总重/时间"
  ——后者把停车拖尾一起算进去，会让前馈系统性偏大。
- 同时记录 delta_mg / cv_pct / startup_delay_s，用于判断该点可不可信。
- 每测完一点立刻落盘：中途想停，已测的点不会丢。

安全边界
--------
单点增重超过 ``MAX_POINT_MASS_MG`` 或速率超过 ``MAX_POINT_RATE_MG_S`` 立即停振并中止；
天平连续读不到即中止；任何异常路径都先关振动、再停搅拌。
"""

from __future__ import annotations

import json
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from powder_sampling_control.control_panel.powder_feedforward import (
    ROD_MAX_UNITS,
    ROD_MIN_UNITS,
    load_powder,
    now_text,
)

# 标定默认只测**一个工作点**：搅拌转速 + 推杆开度各一个值。
# 页面上的两个输入框可以随便改（就是你要跑的那个工况），也允许填逗号分隔的多个值
# 做小范围扫描；留空才回退到这里。
DEFAULT_STIR_LEVELS = (100.0,)
DEFAULT_ROD_POINTS = (600,)
DEFAULT_DURATION_S = 20.0
# 默认振动参数（2026-10-21 用户定）：方波 80Hz / 50% 占空比 / 2Vpp
DEFAULT_FREQUENCY_HZ = 80.0
DEFAULT_DUTY_PERCENT = 50.0
DEFAULT_AMPLITUDE_VPP = 2.0

# 每次标定的完整记录（含当时的频率/占空比/振幅）单独落一份，和加粉的 runs 放一起。
# 为什么不能只靠粉体库：粉体库是按"搅拌+开度"覆盖的汇总表，同一张表可能混着几次
# 不同振动参数的测点；哪一点是 2Vpp、哪一点是 3Vpp，只有按次记录才查得清。
RUN_DIR = Path(__file__).resolve().parent / "runs"


def _safe_stem(text: str) -> str:
    """粉名转文件名：剔掉 Windows 非法字符，长度限 40（括号、空格保留，方便看懂）。"""
    bad = '<>:"/\\|?*\n\r\t'
    cleaned = "".join("_" if ch in bad else ch for ch in str(text or "").strip())
    cleaned = " ".join(cleaned.split())
    return cleaned[:40] or "powder"


class RateCalibrationRunner:
    """开环速率标定的后台执行器（同一时间只允许一个）。"""

    SAMPLE_INTERVAL_S = 0.12
    SETTLE_TIMEOUT_S = 6.0
    # 溢料保护：只防"参数彻底失控把粉倒出来"，不是流量指标。
    # 2026-10-21 用户要求把速率上限从 150 提到 300 mg/s（他实测到 154 就被打断了），
    # 单点质量上限跟着放宽到 6 g —— 否则 300 mg/s 跑满 20 s 会先撞质量墙，
    # 等于速率上限形同虚设。
    MAX_POINT_MASS_MG = 6000.0      # 单点最多出 6 g
    MAX_POINT_RATE_MG_S = 300.0
    MAX_BALANCE_FAILS = 5
    ROD_ARRIVE_TIMEOUT_S = 5.0
    STIR_SETTLE_S = 2.0             # 换搅拌转速后等转速稳住再开振
    STIR_START_ACC = 200            # 启动曲线（>1300 RPM 必须曲线启动）
    STIR_STOP_ACC = 200             # 收尾减速停曲线：驱动只接受 0..255，写 300 会抛错停不下来
    WAVEFORM_SQUARE = 2

    def __init__(self, hub: Any, log: Callable[..., None]) -> None:
        self.hub = hub
        self._log = log
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._state = "idle"
        self._detail = "尚未标定"
        self._config: dict[str, Any] = {}
        self._points: list[dict] = []
        self._planned: list[dict] = []
        self._current: dict[str, Any] = {}
        self._index = 0
        self._total = 0
        self._internal_stop = ""
        self._last_run_path: str | None = None
        self._last_samples: list[tuple[float, float]] = []
        self._last_rod_units: int | None = None   # 推杆最后停在哪（收尾不回位，只记录）

    # ------------------------------------------------------------------
    # 对外状态
    # ------------------------------------------------------------------

    @property
    def running(self) -> bool:
        return self._state in ("preparing", "running", "settling")

    def status(self) -> dict:
        with self._lock:
            return {
                "state": self._state,
                "running": self.running,
                "detail": self._detail,
                "config": dict(self._config),
                "points": list(self._points),
                "planned": list(self._planned),
                "current": dict(self._current),
                "done": self._index,
                "total": self._total,
                "log_path": self._last_run_path,
            }

    # ------------------------------------------------------------------
    # 启动 / 停止
    # ------------------------------------------------------------------

    def start(
        self,
        name: str,
        *,
        category: str | None = None,
        stir_levels: list[float] | str | None = None,
        rod_points: list[int] | str | None = None,
        frequency_hz: float = DEFAULT_FREQUENCY_HZ,
        duty_percent: float = DEFAULT_DUTY_PERCENT,
        amplitude_vpp: float = DEFAULT_AMPLITUDE_VPP,
        duration_s: float = DEFAULT_DURATION_S,
    ) -> dict:
        if self.running:
            raise RuntimeError(f"标定正在进行（{self._state}），请先停止")
        if getattr(self.hub.dispense, "running", False):
            raise RuntimeError("闭环加粉正在运行，不能同时做标定")

        powder = str(name or "").strip()
        if not powder:
            raise ValueError("粉末名称不能为空")

        stirs = _parse_floats(stir_levels, DEFAULT_STIR_LEVELS, 0.0, 1500.0, "搅拌转速")
        rods = _parse_ints(rod_points, DEFAULT_ROD_POINTS, ROD_MIN_UNITS, ROD_MAX_UNITS, "推杆开度")
        duration = float(duration_s)
        if not 2.0 <= duration <= 30.0:
            raise ValueError("每点测量时长必须在 2–30 s（太短测不出斜率，太长白出粉）")
        if not 0.01 <= float(amplitude_vpp) <= 20.0:
            raise ValueError("振幅必须在 0.01–20 Vpp")
        if not 10.0 <= float(frequency_hz) <= 80.0:
            raise ValueError("振动频率必须在 10–80 Hz（与闭环控制器的限幅一致）")
        if not 5.0 <= float(duty_percent) <= 50.0:
            raise ValueError("占空比必须在 5–50 %")

        workspace = load_powder(powder)
        if category:
            workspace.raw["flow_category"] = category
        workspace.set_signal(frequency_hz=frequency_hz, duty_percent=duty_percent,
                             amplitude_vpp=amplitude_vpp)
        workspace.save()

        idle = self.hub.assert_idle_for_dispense()

        planned = [{"stir_rpm": stir, "rod_units": rod} for stir in stirs for rod in rods]
        with self._lock:
            self._state = "preparing"
            self._detail = f"准备标定「{workspace.name}」：{len(planned)} 个点"
            self._config = {
                "name": workspace.name,
                "file": workspace.path.name,
                "category": workspace.category,
                "frequency_hz": round(float(frequency_hz), 2),
                "duty_percent": round(float(duty_percent), 2),
                "amplitude_vpp": round(float(amplitude_vpp), 3),
                "duration_s": round(duration, 2),
                "stir_levels": list(stirs),
                "rod_points": list(rods),
                "rod_start": idle["rod_position"],
                "started_at": now_text(),
            }
            self._points = []
            self._planned = planned
            self._current = {}
            self._index = 0
            self._total = len(planned)

        self._internal_stop = "操作员停止"
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._thread_main,
            args=(workspace, planned, float(frequency_hz), float(duty_percent),
                  float(amplitude_vpp), duration),
            name="rate-calibration", daemon=True)
        self._thread.start()
        self._log(f"开环速率标定启动：「{workspace.name}」 {len(planned)} 点，"
                  f"{frequency_hz:g}Hz / {duty_percent:g}% / {amplitude_vpp:g}Vpp，"
                  f"每点 {duration:.0f}s")
        return self.status()

    def stop(self, reason: str = "操作员停止") -> dict:
        if not self.running:
            return {"stopped": False, "reason": "当前没有运行中的标定"}
        self._internal_stop = reason
        self._stop_event.set()
        self._log(f"标定收到停止请求：{reason}")
        return {"stopped": True, "reason": reason}

    def abort(self, reason: str) -> None:
        """急停路径：只置位，硬件停止由 emergency_stop_all 负责。"""
        if self.running:
            self._internal_stop = f"急停：{reason}"
            self._stop_event.set()

    # ------------------------------------------------------------------
    # 线程主体
    # ------------------------------------------------------------------

    def _thread_main(self, workspace, planned, frequency_hz, duty_percent,
                     amplitude_vpp, duration_s) -> None:
        reason = "全部标定点完成"
        try:
            self._set_state("preparing", "设置振动参数（频率/占空比/振幅）")
            self.hub._sig_call(lambda: (
                self.hub.sig.set_waveform(self.WAVEFORM_SQUARE),
                self.hub.sig.set_frequency(frequency_hz),
                self.hub.sig.set_duty(duty_percent),
                self.hub.sig.set_amplitude(amplitude_vpp),
            ))

            for index, point in enumerate(planned):
                if self._stop_event.is_set():
                    raise _Abort(self._internal_stop)
                with self._lock:
                    self._index = index
                    self._current = {"stir_rpm": point["stir_rpm"],
                                     "rod_units": point["rod_units"],
                                     "phase": "准备", "elapsed_s": 0.0}
                self._set_state("running",
                                f"第 {index + 1}/{len(planned)} 点："
                                f"搅拌 {point['stir_rpm']:g} RPM / 推杆 {point['rod_units']}")
                # 搅拌的起停统一由 _measure 管：每点结束时振动和搅拌是**同时**停的，
                # 所以这里不能再按"档位有没有变"决定要不要起搅拌——上一个点结束时
                # 搅拌已经停了，档位没变但机器是静止的。
                record = self._measure(workspace, point, duration_s)
                workspace.record_point(record)
                workspace.save()
                with self._lock:
                    self._points.append(record)
                    self._index = index + 1
                self._log(f"标定点完成：搅拌 {record['stir_rpm']:g} RPM / 开度 "
                          f"{record['rod_units']} → {record['rate_mg_s']:.2f} mg/s"
                          f"（CV {record['cv_pct']:.1f}%，Δ{record['delta_mg']:.0f}mg，"
                          f"振动 {record['duration_s']:g}s，"
                          f"方波 {record['signal']['frequency_hz']:g}Hz / "
                          f"{record['signal']['duty_percent']:g}% / "
                          f"{record['signal']['amplitude_vpp']:g}Vpp）")
            self._finish(workspace, reason, ok=True)
        except _Abort as abort:
            reason = abort.reason
            self._safe_stop_all()
            self._finish(workspace, reason, ok=False)
        except Exception as error:      # noqa: BLE001 - 线程不能带异常死掉
            self._safe_stop_all()
            self._log(f"标定异常终止：{error}")
            for line in traceback.format_exc().strip().splitlines()[-4:]:
                self._log(f"  {line}")
            with self._lock:
                self._state = "failed"
                self._detail = f"异常：{error}"
                self._current = {}

    # ------------------------------------------------------------------
    # 单点测量
    # ------------------------------------------------------------------

    def _measure(self, workspace, point: dict, duration_s: float) -> dict:
        stir = float(point["stir_rpm"])
        rod = int(point["rod_units"])
        self._last_samples = []

        self._ensure_vibration_off()
        # 每点开始先把搅拌拉到目标转速并等它稳住，再动推杆。
        # 上一个点结束时搅拌是**跟着振动一起停**的，所以这里每点都要重新起。
        if self._stop_event.is_set():
            raise _Abort(self._internal_stop)
        self._set_phase(f"搅拌 → {stir:g} RPM")
        self._ensure_stir_at(stir)
        # 先把推杆开到本点位置，**再**取基线。
        # 顺序不能反：搅拌在转时推杆一动就可能掉下一团料，先取基线会把这一团算进
        # 本点流量里（斜率虚高）。先动杆、等沉降、再取基线，掉下来的那团就被排除了。
        if self._stop_event.is_set():
            raise _Abort(self._internal_stop)
        self._set_phase(f"推杆 → {rod}")
        self.hub.rod_move(rod)
        self._wait_rod(rod)
        self._last_rod_units = int(rod)      # 收尾不回位，得记住最后停在哪

        self._set_phase("等沉降取基线")
        baseline, _baseline_stable = self._settle()

        if self._stop_event.is_set():
            raise _Abort(self._internal_stop)
        self._set_phase(f"起振 {duration_s:.0f}s")
        samples = self._run_vibration(duration_s)
        # 振动和搅拌**同一时刻停**，不在"等称重"的那几秒里让搅拌空转（见方法注释）。
        self._stop_feed_and_stir()

        self._set_phase("称重并计算速率")
        final_raw, stable = self._settle()
        delta = float(final_raw) - float(baseline)

        fit = _fit_rate(samples, delta, duration_s)
        record = {
            "stir_rpm": round(stir, 1),
            "rod_units": int(rod),
            "rate_mg_s": round(fit["rate_mg_s"], 2),
            "cv_pct": round(fit["cv_pct"], 1),
            "delta_mg": round(delta, 1),
            "duration_s": round(duration_s, 2),
            "startup_delay_s": round(fit["startup_delay_s"], 2),
            "samples": fit["samples"],
            "stable_after_stop": bool(stable),
            "signal": {
                "frequency_hz": self._config["frequency_hz"],
                "duty_percent": self._config["duty_percent"],
                "amplitude_vpp": self._config["amplitude_vpp"],
            },
            "at": now_text(),
        }
        if fit["rate_mg_s"] <= 0.05:
            self._log(f"标定点无流量：搅拌 {stir:g} RPM / 开度 {rod}，"
                      f"{duration_s:.0f}s 内只增重 {delta:.0f} mg（可能搭桥或开度过小）")
        return record

    def _run_vibration(self, duration_s: float) -> list[tuple[float, float]]:
        samples: list[tuple[float, float]] = []
        # 挂到实例上：中途被保护中止时 _run_vibration 不会 return，只有这样才能把
        # 已经采到的曲线带进落盘的 JSON —— 事后区分"真爆料"和"天平跳了一下"全靠它。
        self._last_samples = samples
        self.hub.sig_output(True)
        t0 = time.monotonic()
        fails = 0
        try:
            while True:
                elapsed = time.monotonic() - t0
                if elapsed >= duration_s:
                    break
                if self._stop_event.is_set():
                    raise _Abort(self._internal_stop)
                try:
                    raw = self._read_raw()
                    fails = 0
                except Exception as error:      # noqa: BLE001
                    fails += 1
                    if fails >= self.MAX_BALANCE_FAILS:
                        raise _Abort(f"天平失联，标定中止（{error}）") from error
                    time.sleep(0.05)
                    continue
                samples.append((elapsed, raw))
                with self._lock:
                    self._current.update({
                        "phase": "振动中",
                        "elapsed_s": round(elapsed, 1),
                        "mass_gain_mg": round(raw - samples[0][1], 1),
                    })
                if raw - samples[0][1] > self.MAX_POINT_MASS_MG:
                    raise _Abort(f"单点增重超过 {self.MAX_POINT_MASS_MG:.0f} mg，"
                                 f"参数失控，已停振")
                if len(samples) >= 12:
                    recent = samples[-12:]
                    span = recent[-1][0] - recent[0][0]
                    if span > 0.4:
                        rate = (recent[-1][1] - recent[0][1]) / span
                        if rate > self.MAX_POINT_RATE_MG_S:
                            raise _Abort(f"瞬时速率 {rate:.0f} mg/s 超过上限 "
                                         f"{self.MAX_POINT_RATE_MG_S:.0f}，已停振")
                time.sleep(self.SAMPLE_INTERVAL_S)
        finally:
            self._ensure_vibration_off()
        return samples

    # ------------------------------------------------------------------
    # 收尾
    # ------------------------------------------------------------------

    def _finish(self, workspace, reason: str, *, ok: bool) -> None:
        # 收尾只停振动 + 减速停搅拌，**不动推杆**（用户 2026-10-21 15:35 要求）。
        # 以前这里会 rod_move(rod_start) 把推杆退回标定开始时的位置，纯属多余：
        # ① 振动已经关了，推杆开着也不出粉（振动才是料流的源）；
        # ② 每个标定点本来就要动推杆，收尾再多一次机械动作，除了多磨一次丝杆没别的作用；
        # ③ 真上秤场景下这一下会晃秤，看着像"标定完还在出料"。
        # 要回原位，用页面「推杆回原位」按钮，什么时候回由操作员决定。
        self._set_state("settling", "停振动 + 搅拌减速停止（推杆保持不动）")
        self._safe_stop_all()
        self._log(f"标定收尾：推杆停在 {self._last_rod_units if self._last_rod_units is not None else '（本点未动作）'}"
                  f"，不回原位")

        points = list(self._points)
        summary = (workspace.feedforward or {}).get("summary") or {}
        self._save_run(workspace, points, summary, reason, ok)
        with self._lock:
            self._state = "done" if ok else "aborted"
            self._current = {}
            self._detail = (
                f"完成 {len(points)} 点：速率 {summary.get('rate_min_mg_s', '–')}–"
                f"{summary.get('rate_max_mg_s', '–')} mg/s，"
                f"三阶段搅拌 {summary.get('stir_levels') or '–'}（{reason}）"
                if ok else f"已中止：{reason}（已完成 {len(points)} 点，已保留）")
        self._log(f"标定结束：「{workspace.name}」{self._detail}")

    def _save_run(self, workspace, points: list[dict], summary: dict,
                  reason: str, ok: bool) -> None:
        """每次标定落一份 runs/calib_*.json。

        为什么粉体库不够：粉体库按「搅拌+开度」覆盖汇总，同一张表可能混着几次不同
        振动参数的测点（例如先 3Vpp 测一点、再 2Vpp 测两点），事后分不清哪点是什么条件。
        按次记录把"当时的频率/占空比/振幅"提到顶层，配合每点自带的 signal，可逐点对账。
        """
        try:
            RUN_DIR.mkdir(parents=True, exist_ok=True)
            cfg = dict(self._config or {})
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = RUN_DIR / f"calib_{stamp}_{_safe_stem(workspace.name)}.json"
            payload = {
                "kind": "rate_calibration",
                "powder": workspace.name,
                "file": workspace.path.name,
                "signal": {
                    "waveform": "方波",
                    "frequency_hz": cfg.get("frequency_hz"),
                    "duty_percent": cfg.get("duty_percent"),
                    "amplitude_vpp": cfg.get("amplitude_vpp"),
                },
                "duration_s": cfg.get("duration_s"),
                "stir_levels": cfg.get("stir_levels"),
                "rod_points": cfg.get("rod_points"),
                "rod_start_units": cfg.get("rod_start"),
                "rod_final_units": self._last_rod_units,
                "planned": list(self._planned or []),
                "points": points,
                "ok": bool(ok),
                "reason": reason,
                "started_at": cfg.get("started_at"),
                "finished_at": now_text(),
                "summary": summary,
            }
            # 振动期间的称重曲线（t 秒 / mg）。中止时这段数据尤其重要：
            # 单看"154 mg/s 超限"没法判断是真塌料还是天平跳了一下，有曲线就能看出来。
            series = list(self._last_samples or [])
            if series:
                step = max(1, len(series) // 300)
                payload["samples"] = [
                    {"t": round(t, 2), "mg": round(v, 1)} for t, v in series[::step]
                ]
            path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            with self._lock:
                self._last_run_path = str(path)
            self._log(f"标定记录已保存：{path.name}")
        except Exception as error:      # noqa: BLE001 - 落盘失败不影响标定结果
            self._log(f"标定记录保存失败：{error}")

    def _safe_stop_all(self) -> None:
        """标定收尾只做两件事：先停振动（断料），再让搅拌**减速**停。

        顺序不能反：振动是料流的源，先断料再减速，转子上挂的粉才不会被"边转边送"。
        2026-10-21 教训：这里曾写 acc=300（驱动只收 0..255），停止命令直接抛错、搅拌一直转，
        现场就是"时间到了没停"。现在 acc 用 STIR_STOP_ACC（=200）的减速曲线，
        常规路径就是**减速停**；只有停止命令本身失败（串口/参数）才降级立即停，
        那是保底，不是常规动作。
        """
        self._ensure_vibration_off()
        try:
            self.hub.stir_stop("decel", acc=self.STIR_STOP_ACC)
        except Exception as error:      # noqa: BLE001
            self._log(f"减速停搅拌失败：{error}，改立即停止")
            self._stop_stir_immediate()
            return
        rpm = self.hub.stir_speed_rpm()
        self._log(f"收尾完成：振动已关，搅拌按 acc={self.STIR_STOP_ACC} 减速停"
                  + (f"（当前 {rpm:.0f} RPM，正在减速）" if rpm is not None else ""))

    def _stop_stir_immediate(self) -> bool:
        try:
            self.hub.stir_stop("immediate")
            return True
        except Exception as error:      # noqa: BLE001
            self._log(f"立即停搅拌失败：{error}（请手动拍急停）")
            return False

    def _ensure_vibration_off(self) -> None:
        for attempt in (1, 2):
            try:
                self.hub.sig_output(False)
                return
            except Exception as error:      # noqa: BLE001
                self._log(f"关闭振动失败（第 {attempt} 次）：{error}")
                time.sleep(0.15)

    def _ensure_stir_at(self, rpm: float) -> None:
        """确保搅拌在目标转速：已经在转且对得上就不动，否则启动并等转速稳住。

        为什么每点都要起：晚点的收尾是"振动和搅拌一起停"，所以进下一点时搅拌是静止的，
        不能再像以前那样只看"档位变没变"。
        """
        if rpm <= 0.5:
            return
        actual = None
        try:
            actual = self.hub.stir_speed_rpm()
        except Exception as error:      # noqa: BLE001 - 读不到就按需要启动处理
            self._log(f"读取搅拌转速失败（{error}），按静止处理")
        if actual is not None and abs(float(actual) - float(rpm)) <= 15.0:
            return
        self.hub.stir_start(float(rpm), acc=self.STIR_START_ACC, direction=0)
        time.sleep(self.STIR_SETTLE_S)      # 等转速爬到位，否则本点速率偏低

    def _stop_feed_and_stir(self) -> None:
        """振动和搅拌**同一时刻**停：先断振动的料流，紧接着给搅拌减速指令。

        2026-10-21 现场观察：以前停振之后要等沉降称重（最长 6 s）才在收尾里停搅拌，
        这几秒搅拌还转着、还在往下送粉，看到的就是"两个不是一个时间停"。而且那部分
        尾部粉落在秤上，被算进本点重量，速率会系统性偏大。

        这里只要求**指令同时下发**，不要求物理上同一毫秒静止：振动是断输出（瞬时），
        搅拌按 acc=200 减速（100 RPM 约 0.3 s、200 RPM 约 0.6 s），减速期间落下的粉
        本来也是这一点出的粉，留在称重结果里是对的。
        """
        t0 = time.monotonic()
        self._ensure_vibration_off()
        stir_ok = True
        try:
            self.hub.stir_stop("decel", acc=self.STIR_STOP_ACC)
        except Exception as error:      # noqa: BLE001 - 停不下来比停得不柔和更危险
            self._log(f"停搅拌失败（{error}），改立即停")
            stir_ok = self._stop_stir_immediate()
        gap_ms = (time.monotonic() - t0) * 1000.0
        self._log(f"停振 + 搅拌减速停（acc={self.STIR_STOP_ACC}，"
                  f"两条指令间隔 {gap_ms:.0f} ms{'' if stir_ok else '，已降级立即停'}）")

    # ------------------------------------------------------------------
    # 硬件小工具
    # ------------------------------------------------------------------

    def _read_raw(self) -> float:
        snapshot = self.hub.bal_refresh()
        if snapshot.get("raw") is None:
            raise RuntimeError("天平没有返回有效计数")
        return float(snapshot["raw"])

    def _settle(self) -> tuple[float, bool]:
        """等重量稳定：连续 5 个样本极差 ≤2 mg 即认为稳，超时取末段均值。

        循环里也看停止请求：操作员点了停止就别傻等到超时。不然一个 20 秒的点位
        加上沉降最长要再等 6 秒，现场看到的就是"点了停止还在出粉"。
        """
        deadline = time.monotonic() + self.SETTLE_TIMEOUT_S
        history: list[float] = []
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                raise _Abort(self._internal_stop)
            try:
                history.append(self._read_raw())
            except Exception:       # noqa: BLE001
                time.sleep(0.15)
                continue
            if len(history) >= 5:
                window = history[-5:]
                if max(window) - min(window) <= 2:
                    return float(round(sum(window) / len(window))), True
            time.sleep(0.15)
        window = history[-5:] if len(history) >= 5 else history
        if not window:
            raise _Abort("称重阶段一个样本都没读到（天平异常）")
        return float(round(sum(window) / len(window))), False

    def _wait_rod(self, target: int) -> None:
        deadline = time.monotonic() + self.ROD_ARRIVE_TIMEOUT_S
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                raise _Abort(self._internal_stop)
            try:
                position = int(self.hub.rod_refresh()["pos"])
            except Exception:       # noqa: BLE001
                time.sleep(0.1)
                continue
            if abs(position - target) <= 3:
                return
            time.sleep(0.1)
        raise _Abort(f"推杆未在 {self.ROD_ARRIVE_TIMEOUT_S:.0f}s 内到达 {target}")

    # ------------------------------------------------------------------

    def _set_state(self, state: str, detail: str) -> None:
        with self._lock:
            self._state = state
            self._detail = detail

    def _set_phase(self, phase: str) -> None:
        with self._lock:
            self._current.update({"phase": phase})


# ----------------------------------------------------------------------
# 数学小工具
# ----------------------------------------------------------------------

def _fit_rate(samples: list[tuple[float, float]], delta: float,
              duration_s: float) -> dict:
    """从振动期间的 (t, 计数) 序列拟合速率。

    去掉前 20%（启动爬坡，粉还没下来）与末尾 0.2s（已下令停振，读数在掉），
    对剩下部分做最小二乘斜率。窗口速率用来算 CV：粉流是"一股一股"的，
    只看总斜率会把"脉动得很厉害"误判成"很稳"。
    """
    if len(samples) < 8:
        fallback = delta / max(1e-6, duration_s)
        return {"rate_mg_s": max(0.0, fallback), "cv_pct": 0.0,
                "startup_delay_s": 0.0, "samples": len(samples)}

    t0 = samples[0][0]
    start_cut = max(t0 + 0.8, t0 + 0.2 * duration_s)
    end_cut = samples[-1][0] - 0.2
    steady = [(t, m) for t, m in samples if start_cut <= t <= end_cut]
    if len(steady) < 6:
        steady = samples[max(1, len(samples) // 4):-1] or samples

    rate = _slope(steady)
    if rate <= 0:
        rate = max(0.0, delta / max(1e-6, duration_s))

    # 1s 窗口速率的离散度
    windows: list[float] = []
    window_span = 1.0
    start = steady[0][0]
    while start + window_span <= steady[-1][0]:
        chunk = [(t, m) for t, m in steady if start <= t <= start + window_span]
        if len(chunk) >= 4:
            windows.append(_slope(chunk))
        start += window_span
    windows = [value for value in windows if value > 0.05]
    if len(windows) >= 2:
        mean = sum(windows) / len(windows)
        var = sum((value - mean) ** 2 for value in windows) / (len(windows) - 1)
        cv = 100.0 * (var ** 0.5) / mean if mean > 1e-6 else 0.0
    else:
        cv = 0.0

    base = samples[0][1]
    threshold = max(2.0, 0.05 * max(1.0, delta))
    startup = 0.0
    for t, mass in samples:
        if mass - base >= threshold:
            startup = t - t0
            break

    return {
        "rate_mg_s": max(0.0, float(rate)),
        "cv_pct": float(cv),
        "startup_delay_s": float(startup),
        "samples": len(steady),
    }


def _slope(points: list[tuple[float, float]]) -> float:
    n = len(points)
    if n < 2:
        return 0.0
    mean_t = sum(p[0] for p in points) / n
    mean_m = sum(p[1] for p in points) / n
    num = sum((p[0] - mean_t) * (p[1] - mean_m) for p in points)
    den = sum((p[0] - mean_t) ** 2 for p in points)
    if den <= 1e-9:
        return 0.0
    return num / den


def _parse_floats(raw: Any, default: tuple[float, ...],
                  low: float, high: float, label: str) -> list[float]:
    values = _split_numbers(raw) if raw not in (None, "", []) else list(default)
    result: list[float] = []
    for value in values:
        number = float(value)
        if not low <= number <= high:
            raise ValueError(f"{label} {number:g} 超出允许范围 {low:g}–{high:g}")
        if number not in result:
            result.append(number)
    if not result:
        raise ValueError(f"{label}至少要有一个值")
    return result


def _parse_ints(raw: Any, default: tuple[int, ...],
                low: int, high: int, label: str) -> list[int]:
    values = _split_numbers(raw) if raw not in (None, "", []) else list(default)
    result: list[int] = []
    for value in values:
        number = int(round(float(value)))
        if not low <= number <= high:
            raise ValueError(f"{label} {number} 超出允许范围 {low}–{high}")
        if number not in result:
            result.append(number)
    if not result:
        raise ValueError(f"{label}至少要有一个值")
    return result


def _split_numbers(raw: Any) -> list[float]:
    if isinstance(raw, (list, tuple)):
        items = list(raw)
    else:
        text = str(raw).replace("，", ",").replace(" ", ",")
        items = [chunk for chunk in text.split(",") if chunk.strip()]
    values: list[float] = []
    for item in items:
        try:
            values.append(float(item))
        except (TypeError, ValueError) as error:
            raise ValueError(f"无法解析数值：{item!r}") from error
    return values


class _Abort(Exception):
    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason
