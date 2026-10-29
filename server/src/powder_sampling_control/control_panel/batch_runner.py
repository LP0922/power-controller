"""批量加粉执行器：把队列里的每一项依次交给 DispenseRunner 跑。

为什么单独一层：``DispenseRunner`` 一次只能跑一项（它有 ``self.running`` 互锁），
批量只是"排队 + 等这一项真的结束 + 换下一项 + 项间去皮"。**不复用也不复制**
闭环逻辑——每一项目前就是一次正常的加粉，参数和从页面单点启动完全一致。

队列项（``sets`` 里的一个对象）支持的键：

    target_mg            目标质量（必填）
    repeat_count         重复次数（1–20，展开成多条）
    powder / powder_name 粉末名（整批必须一致）
    stages               逐段覆盖（振幅/推杆/搅拌），可选
    settle_gate          切段沉降门开关，可选
    surge_guard/surge_ratio  段内超速熔断，可选
    trim_enabled         尾量缓补开关，可选

其它键（如老前端的 ``frequency_hz`` / ``window_position_units``）一律忽略：
本引擎没有"窗口位置 + 定速"这套参数，硬映射只会把闭环算歪。
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from powder_sampling_control.dispensing_algorithm import validate_target_mass

RUN_DIR = Path(__file__).resolve().parent / "runs"

MAX_SETS = 20                       # 队列行数上限（老后端也是 20）
MAX_REPEAT = 20                     # 单行重复次数上限
MAX_ITEMS = 40                      # 展开（含重复）后的总项数上限
ITEM_TIMEOUT_S = 1800.0             # 单项最长 30 分钟（20g 大目标也够）
POLL_S = 0.20                       # 等单项结束的轮询间隔
TARE_SETTLE_S = 1.5                 # 项间去皮后等天平稳定
STOP_WAIT_S = 15.0                  # 停止时最多等这么久让本项收尾

# 可以原样转给 DispenseRunner.start 的键（白名单：其余键静默丢弃）
DISPENSE_KEYS = (
    "category", "preset_id", "amplitude_vpp", "tail_mass_mg", "use_feedforward",
    "stages", "settle_gate", "settle_gate_rate_mg_s", "taper_factor",
    "surge_guard", "surge_ratio", "trim_enabled", "trim_enter_remain_mg",
    "trim_pulse_target_mg",
)


def _num_or_none(value: Any) -> float | None:
    if value is None or value == "":
        return None
    return float(value)


def _bool_or_none(value: Any) -> bool | None:
    if value is None or value == "":
        return None
    return bool(value)


class DispenseBatchRunner:
    """串行执行批量加粉队列。

    状态机：``idle → running → done / aborted``。跑完保留结果直到下一次启动，
    这样页面刷新后还能看到上一批的结果（和老后端行为一致）。
    """

    def __init__(self, hub: Any, log: Callable[[str], None]) -> None:
        self._hub = hub
        self._log = log
        self._lock = threading.RLock()
        self._state = "idle"
        self._detail = ""
        self._run_id = ""
        self._items: list[dict] = []
        self._results: list[dict] = []
        self._current = -1
        self._tare_between = True
        self._started_at = ""
        self._finished_at = ""
        self._cancel = threading.Event()
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # 查询
    # ------------------------------------------------------------------
    @property
    def running(self) -> bool:
        with self._lock:
            return self._state == "running"

    def status(self) -> dict:
        with self._lock:
            total = len(self._items)
            current = self._current
            item = self._items[current] if 0 <= current < total else None
            return {
                "state": self._state,
                "running": self._state == "running",
                "run_id": self._run_id,
                "detail": self._detail,
                "set_count": total,
                "current_index": current,           # 0 基；-1 = 还没开始
                "current_target_mg": (item or {}).get("target_mg"),
                "items": [dict(it) for it in self._items],
                "results": [dict(r) for r in self._results],
                "summary": self._summary_locked(),
                "tare_between_sets": self._tare_between,
                "started_at": self._started_at,
                "finished_at": self._finished_at,
            }

    def _summary_locked(self) -> dict:
        passed = sum(1 for r in self._results if r.get("passed"))
        total = len(self._results)
        return {
            "done": total,
            "passed": passed,
            "failed": total - passed,
            "pass_rate": round(passed / total * 100, 1) if total else 0.0,
        }

    # ------------------------------------------------------------------
    # 启动 / 停止
    # ------------------------------------------------------------------
    def start(self, sets: Any, *, powder: Any = None,
              tare_between_sets: Any = None) -> dict:
        if self.running:
            raise RuntimeError(f"已有批量任务在运行（第 {self._current + 1} 项），请先停止")
        items = self._expand(sets, powder)
        if self._hub.calib.running:
            raise RuntimeError("速率标定正在运行，不能开始批量加粉——请先停止标定")
        if self._hub.dispense.running:
            raise RuntimeError("已有加粉任务在运行，不能开始批量加粉——请先停止")

        with self._lock:
            self._items = items
            self._results = []
            self._current = -1
            self._tare_between = True if tare_between_sets is None else bool(tare_between_sets)
            self._cancel.clear()
            self._state = "running"
            self._detail = f"批量 {len(items)} 项，准备开始"
            self._run_id = (f"disp-batch-{len(items)}sets-"
                            f"{datetime.now().strftime('%Y%m%d_%H%M%S')}-"
                            f"{uuid.uuid4().hex[:6]}")
            self._started_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self._finished_at = ""

        targets = "、".join(str(it["target_mg"]) for it in items[:6])
        more = "…" if len(items) > 6 else ""
        self._log(f"批量加粉开始：{len(items)} 项（{targets}{more} mg），"
                  f"粉末「{items[0].get('powder') or '（不选粉）'}」，"
                  f"组间去皮 {'开' if self._tare_between else '关'}，编号 {self._run_id}")
        self._thread = threading.Thread(target=self._loop, name="dispense-batch",
                                        daemon=True)
        self._thread.start()
        return self.status()

    def stop(self) -> dict:
        with self._lock:
            if self._state != "running":
                return self.status()
            self._cancel.set()
            self._detail = "正在停止批量（等本项收尾）"
        self._log("批量加粉收到停止指令")
        if self._hub.dispense.running:
            self._hub.dispense.stop("批量停止")
        return self.status()

    # ------------------------------------------------------------------
    # 队列展开与校验
    # ------------------------------------------------------------------
    def _expand(self, sets: Any, powder: Any) -> list[dict]:
        if not isinstance(sets, list) or not sets:
            raise ValueError("sets 必须是非空数组（每项一组加粉参数）")
        if len(sets) > MAX_SETS:
            raise ValueError(f"队列最多 {MAX_SETS} 行，收到 {len(sets)} 行")

        top_powder = (str(powder).strip() if powder not in (None, "") else "") or None
        rows: list[dict] = []
        powders: set[str] = set()

        for idx, raw in enumerate(sets):
            if not isinstance(raw, dict):
                raise ValueError(f"第 {idx + 1} 行不是参数对象")
            target = raw.get("target_mg")
            try:
                target_mg = validate_target_mass(
                    int(target) if not isinstance(target, bool) else target)
            except (TypeError, ValueError) as error:
                raise ValueError(f"第 {idx + 1} 行目标质量不合法：{error}") from error

            repeat = raw.get("repeat_count", 1) or 1
            if isinstance(repeat, bool) or not isinstance(repeat, (int, float)):
                raise ValueError(f"第 {idx + 1} 行重复次数必须是整数")
            repeat = int(repeat)
            if not 1 <= repeat <= MAX_REPEAT:
                raise ValueError(f"第 {idx + 1} 行重复次数需在 1–{MAX_REPEAT}")

            name = top_powder or raw.get("powder") or raw.get("powder_name") or None
            if name:
                powders.add(str(name))

            base = {
                "row": idx + 1,
                "target_mg": target_mg,
                "powder": str(name) if name else None,
                "repeat_count": repeat,
            }
            for key in DISPENSE_KEYS:
                if key in raw:
                    if key in ("settle_gate", "surge_guard", "trim_enabled",
                               "use_feedforward"):
                        base[key] = _bool_or_none(raw.get(key))
                    elif key in ("stages",):
                        base[key] = raw.get(key)
                    else:
                        base[key] = _num_or_none(raw.get(key))
            for rep in range(repeat):
                item = dict(base)
                item["repeat_index"] = rep + 1
                rows.append(item)

        if len(powders) > 1:
            raise ValueError("同一批队列只能使用一种粉末，请清空队列后重新添加")
        if len(rows) > MAX_ITEMS:
            raise ValueError(f"展开后共 {len(rows)} 项，超过上限 {MAX_ITEMS}，请减少重复次数")
        return rows

    # ------------------------------------------------------------------
    # 执行循环
    # ------------------------------------------------------------------
    def _loop(self) -> None:
        total = len(self._items)
        aborted = ""
        try:
            for idx, item in enumerate(self._items):
                if self._cancel.is_set():
                    aborted = "操作员停止"
                    break
                with self._lock:
                    self._current = idx
                    tag = (f"第 {idx + 1}/{total} 项"
                           f"（{item['target_mg']}mg"
                           + (f"，重复 {item['repeat_index']}/{item['repeat_count']}"
                              if item.get("repeat_count", 1) > 1 else "")
                           + "）")
                    self._detail = tag + "：准备"
                self._log(f"批量加粉 {tag} 开始")

                if idx > 0 and self._tare_between:
                    self._tare_between_items()

                payload = {k: v for k, v in item.items()
                           if k in DISPENSE_KEYS and v is not None}
                try:
                    self._hub.dispense.start(item["target_mg"],
                                             powder=item.get("powder"),
                                             **payload)
                except Exception as error:      # noqa: BLE001 - 单项起不来也要记账
                    self._log(f"批量加粉 {tag} 启动失败：{error}")
                    self._append_result(item, {"ok": False, "verdict": "未启动",
                                               "stop_reason": f"启动失败：{error}"})
                    continue

                stopped = self._wait_item(tag)
                result = self._hub.dispense.status().get("result") or {}
                self._append_result(item, result, stopped=stopped)
                if stopped and self._cancel.is_set():
                    aborted = "操作员停止"
                    break

            with self._lock:
                if aborted:
                    self._state = "aborted"
                    self._detail = f"批量已停止（{aborted}），完成 {len(self._results)}/{total} 项"
                else:
                    self._state = "done"
                    summary = self._summary_locked()
                    self._detail = (f"批量完成：{summary['passed']}/{summary['done']} 合格"
                                    f"（通过率 {summary['pass_rate']}%）")
                self._finished_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        except Exception as error:              # noqa: BLE001 - 线程不能静默死掉
            with self._lock:
                self._state = "aborted"
                self._detail = f"批量异常中止：{error}"
                self._finished_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self._log(f"批量加粉异常中止：{error}")
        finally:
            self._save()

    def _tare_between_items(self) -> None:
        """项间去皮：清零天平，让下一项的基线从 0 开始。"""
        with self._lock:
            self._detail = "组间去皮：清零天平并等稳定"
        try:
            self._hub.bal_tare()
            time.sleep(TARE_SETTLE_S)
            self._log("批量加粉：组间去皮完成")
        except Exception as error:              # noqa: BLE001 - 去皮失败不中止批量
            # 不去皮也能跑：加粉引擎自己会在起振前取基线，只是天平累计读数会一直往上涨。
            self._log(f"批量加粉：组间去皮失败（{error}），本项仍继续，"
                      f"基线由引擎自行采样")

    def _wait_item(self, tag: str) -> bool:
        """等当前这一项真正结束。返回 True 表示是被停止/超时打断的。"""
        t0 = time.monotonic()
        last_note = ""
        while self._hub.dispense.running:
            if self._cancel.is_set():
                self._hub.dispense.stop("批量停止")
            if time.monotonic() - t0 > ITEM_TIMEOUT_S:
                self._log(f"批量加粉 {tag} 超过 {ITEM_TIMEOUT_S:.0f}s，强制停止")
                self._hub.dispense.stop("单项超时")
                return True
            live = self._hub.dispense.status().get("live") or {}
            note = (f"{live.get('stage_text') or '加粉中'} "
                    f"{live.get('mass_mg') or 0:.0f}mg")
            with self._lock:
                self._detail = f"{tag}：{note}" if note != last_note else self._detail
            last_note = note
            time.sleep(POLL_S)
        # 停下后还要等它把收尾称重做完（running 已经变 False，但 result 可能还没写）
        wait_until = time.monotonic() + STOP_WAIT_S
        while time.monotonic() < wait_until:
            result = self._hub.dispense.status()
            if result.get("result") and not result.get("running"):
                break
            time.sleep(POLL_S)
        return self._cancel.is_set()

    def _append_result(self, item: dict, result: dict, *, stopped: bool = False) -> None:
        achieved = result.get("achieved_mg")
        target = item["target_mg"]
        row = {
            "row": item.get("row"),
            "repeat_index": item.get("repeat_index", 1),
            "repeat_count": item.get("repeat_count", 1),
            "target_mg": target,
            "achieved_mg": achieved,
            "delta_mg": result.get("delta_mg"),
            "verdict": result.get("verdict") or ("已停止" if stopped else "无结果"),
            "passed": bool(result.get("ok")) and not stopped,
            "acceptance_passed": bool(result.get("ok")),
            "acceptance_mg": result.get("acceptance_mg"),
            "final_mass_mg": achieved,
            "duration_s": result.get("duration_s"),
            "stop_reason": result.get("stop_reason") or ("操作员停止" if stopped else ""),
            "amplitude_vpp": result.get("amplitude_vpp"),
            "powder": item.get("powder"),
            "powder_name": item.get("powder"),
        }
        with self._lock:
            self._results.append(row)
        self._log(f"批量加粉第 {item.get('row')} 行"
                  f"{'（重复 %d/%d）' % (item.get('repeat_index', 1), item.get('repeat_count', 1)) if item.get('repeat_count', 1) > 1 else ''}"
                  f"：{row['verdict']}，"
                  f"{'%.1f' % achieved if achieved is not None else '--'} / {target} mg"
                  f"（{row['stop_reason'] or '-'}）")

    # ------------------------------------------------------------------
    # 落盘
    # ------------------------------------------------------------------
    def _save(self) -> None:
        status = self.status()
        if not status["results"]:
            return
        payload = {
            "run_id": status["run_id"],
            "started_at": status["started_at"],
            "finished_at": status["finished_at"],
            "state": status["state"],
            "tare_between_sets": status["tare_between_sets"],
            "items": status["items"],
            "results": status["results"],
            "summary": status["summary"],
        }
        try:
            RUN_DIR.mkdir(parents=True, exist_ok=True)
            name = f"batch_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{len(status['results'])}sets.json"
            path = RUN_DIR / name
            path.write_text(json.dumps(payload, ensure_ascii=False, indent=2),
                            encoding="utf-8")
            self._log(f"批量结果已保存：runs/{name}")
        except Exception as error:              # noqa: BLE001 - 写盘失败不影响结论
            self._log(f"批量结果保存失败：{error}")
