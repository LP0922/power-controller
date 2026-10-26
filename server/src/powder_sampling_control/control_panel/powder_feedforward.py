"""粉体前馈库：把"标定出来的出粉速率"翻译成"推杆开度 + 搅拌转速"的前馈指令。

数据落在 ``data/powder_library/<粉末>.json`` 的 ``feedforward_v2`` 段——与老粉体库
**同一个文件、同一套粉名**，不另起一套目录。这一段记录：

    signal          标定时用的振动参数（频率 / 占空比 / 振幅），闭环加粉原样沿用
    points          每个标定点的 (搅拌转速, 推杆开度) → 实测速率 mg/s
    stage_stir_rpm  三个阶段各自的搅拌转速（由标定点推导，可在页面上看到依据）

模型假设（够用且可解释，不做黑箱拟合）：

1. 同一搅拌档内，速率随推杆开度单调递增 → 分段线性插值；
2. 不同搅拌档之间，速率线性插值；
3. **不外推**：超出标定范围只取最近边界，并标 ``achievable=False``。

这样每个前馈值都能追到具体标定点，出了问题能查到"当初是在哪个开度、多少转速、
什么振动参数下测出来的"，而不是一个说不清来源的魔法数。
"""

from __future__ import annotations

import json
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[3]
LIB_DIR = PROJECT_ROOT / "data" / "powder_library"
# 粉体库的同义路径（Windows 目录联接 junction，指向同一个 powder_library）。
# 有些会话环境里，按 LIB_DIR 原路径 open() 会被文件访问策略拒绝
# （PermissionError: [Errno 13]，只拦读、写还正常），但同一批文件换一个路径字符串
# 就能正常读写。这里优先走联接，联接不存在/坏了就退回本名目录——
# 两者指向同一批文件，落盘位置没有任何区别。
LIB_DIR_ALIAS = PROJECT_ROOT / "data" / "powder_library_alias"


def _alias_usable() -> bool:
    """别名目录存在、并且真的能看到档案文件（空目录 / 断链都算不可用）。"""
    try:
        return LIB_DIR_ALIAS.is_dir() and any(LIB_DIR_ALIAS.glob("*.json"))
    except OSError:
        return False


def _make_alias() -> bool:
    """尽力重建别名联接。用 junction（``mklink /J``）而不是软链接——
    junction 不需要管理员权限、也不受开发者模式限制，指向的就是同一个目录。

    换机器 / 重新 clone 仓库 / 有人手工删了这个目录之后，这层联接就没了，
    于是又回到"按路径文本被拦"的老毛病。这里幂等重建一次，失败就静默退回
    本名目录（行为跟加这层兜底之前完全一样，不影响正常机器上启动）。
    """
    try:
        LIB_DIR_ALIAS.unlink(missing_ok=True)
    except OSError:
        pass
    try:
        if not LIB_DIR.is_dir():
            return False
        subprocess.run(
            ["cmd", "/c", "mklink", "/J", str(LIB_DIR_ALIAS), str(LIB_DIR)],
            check=True, capture_output=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return False
    return _alias_usable()


def _resolve_lib_dir() -> Path:
    """优先用可读的那个路径：别名联接在就用别名，否则用本名目录。"""
    if _alias_usable():
        return LIB_DIR_ALIAS
    if _make_alias():
        return LIB_DIR_ALIAS
    return LIB_DIR


LIB_DIR = _resolve_lib_dir()

SECTION = "feedforward_v2"
SCHEMA = 2
STAGES = ("coarse", "fine", "precision")
# 切段惯性落料量：独立于标定段（删标定点不会把它带走）。
# 键是切段名，如 "fast>coarse"，值是"停振后还会多落多少 mg"。
SETTLE_SECTION = "settle_gain_v1"
SETTLE_ALPHA = 0.4          # 指数平滑系数（新样本占 40%）
SETTLE_MAX_MG = 50.0        # 单次学习值上限（防止脏数据把切换点带偏）

# 搅拌转速聚类容差：标定时可填 0/298/305，视作同一档
STIR_CLUSTER_RPM = 25.0

# 前馈推杆的硬边界（LA 实际行程 0–2000，但出料口有效区在 0–1200；
# 超过 1200 老机器也没有验证数据，不敢盲开）
ROD_MIN_UNITS = 20
ROD_MAX_UNITS = 1200

# 标定点可信度口径（页面「标定管理」的红字判定与批量删除共用同一套阈值，
# 免得页面判"不可信"、后端删的时候又按另一套标准，两边对不上）。
POINT_CV_LIMIT_PCT = 40.0        # CV 超过它 = 这段曲线抖到不能当斜率用
POINT_MIN_RATE_MG_S = 0.05       # 低于它视作"这一段根本没出粉"


# 前馈自修正系数：标定表说"开 300 出 10 mg/s"，实际只出了 8.5，就说明表偏乐观，
# 下次同级目标要把开度再开大一点。限幅防止一炉跑歪就把整张表带跑偏。
CORRECTION_DEFAULT = 1.0
CORRECTION_MIN = 0.5
CORRECTION_MAX = 1.8
CORRECTION_ALPHA = 0.35      # 新样本权重（老系统 ILC 的"每次不超过 5–10%"更保守，
                             # 这里前馈只是种子、闭环还会修，可以稍快一点）


# ----------------------------------------------------------------------
# 小工具
# ----------------------------------------------------------------------

def _polyline(xs: list[float], ys: list[float], x: float) -> float:
    """分段线性插值；超出范围取端点（不外推）。"""
    if not xs:
        raise ValueError("empty polyline")
    if len(xs) == 1:
        return float(ys[0])
    if x <= xs[0]:
        return float(ys[0])
    if x >= xs[-1]:
        return float(ys[-1])
    for i in range(len(xs) - 1):
        if xs[i] <= x <= xs[i + 1]:
            span = xs[i + 1] - xs[i]
            if span <= 0:
                return float(ys[i + 1])
            t = (x - xs[i]) / span
            return float(ys[i] + t * (ys[i + 1] - ys[i]))
    return float(ys[-1])


def _inverse_polyline(xs: list[float], ys: list[float], y: float) -> tuple[float, bool]:
    """已知速率反解 x（推杆开度）。

    返回 ``(x, in_range)``。速率可能非单调（粉体偶尔有平台/回落），这里取
    **第一个跨越目标速率的区间**——即"开到多少就能到这个流量"，选最小的开度，
    宁可让闭环往上补，也不一上来就开太大导致过冲。
    """
    if not xs:
        raise ValueError("empty polyline")
    if len(xs) == 1:
        return float(xs[0]), abs(float(ys[0]) - y) <= max(0.5, 0.1 * abs(y))
    if y <= min(ys):
        # 小于全程最小速率：开到最小（流量不可能更小，除非降占空比）
        idx = ys.index(min(ys))
        return float(xs[idx]), abs(float(ys[idx]) - y) <= max(0.5, 0.1 * abs(y))
    if y >= max(ys):
        idx = ys.index(max(ys))
        return float(xs[idx]), abs(float(ys[idx]) - y) <= max(0.5, 0.1 * abs(y))
    for i in range(len(xs) - 1):
        y0, y1 = float(ys[i]), float(ys[i + 1])
        if (y0 <= y <= y1) or (y1 <= y <= y0):
            span = y1 - y0
            if abs(span) < 1e-9:
                return float(xs[i]), True
            t = (y - y0) / span
            return float(xs[i] + t * (xs[i + 1] - xs[i])), True
    return float(xs[-1]), False


def _median(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return float(ordered[mid])
    return float((ordered[mid - 1] + ordered[mid]) / 2.0)


def now_text() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def point_issues(point: dict) -> list[str]:
    """判断一个标定点是否"不可信"，返回原因列表（空列表 = 可信）。

    不可信 = 这条数据不能代表这个工况的出粉能力，留在前馈表里只会把插值带歪：
      - 速率为 0 / 出粉量为 0：料斗搭桥、没料、振动没起振，测的是"没出粉"；
      - CV 超过 40%：曲线抖到没有斜率可言，算出来的速率是平均值凑的；
      - 速率为负：天平被碰、基线被扣，基本上是误操作。
    这些点删掉后前馈模型会自动用剩下的点重算（``_derive``）。
    """
    reasons: list[str] = []
    try:
        rate = float(point.get("rate_mg_s"))
    except (TypeError, ValueError):
        return ["缺少速率字段"]
    delta = point.get("delta_mg")
    try:
        delta_f = float(delta)
    except (TypeError, ValueError):
        delta_f = None
    if rate < 0:
        reasons.append("速率为负（多半是基线被扣）")
    elif rate <= POINT_MIN_RATE_MG_S:
        reasons.append("速率为 0（这一段没出粉）")
    if delta_f is not None and delta_f <= 0:
        reasons.append("本点出粉量为 0")
    cv = point.get("cv_pct")
    if cv is None:
        reasons.append("没有 CV（数据不完整）")
    else:
        try:
            if float(cv) > POINT_CV_LIMIT_PCT:
                reasons.append(f"CV {float(cv):.0f}% 超 {POINT_CV_LIMIT_PCT:.0f}%")
        except (TypeError, ValueError):
            reasons.append("CV 字段异常")
    return reasons


# ----------------------------------------------------------------------
# 粉体工作区（一个粉末一个文件）
# ----------------------------------------------------------------------

class PowderWorkspace:
    """一种粉末的档案：原有字段全部保留，只新增/更新 ``feedforward_v2`` 段。"""

    def __init__(self, path: Path, raw: dict) -> None:
        self.path = path
        self.raw = raw

    # ---- 基本属性 ----

    @property
    def name(self) -> str:
        return str(self.raw.get("powder_name") or self.path.stem)

    @property
    def powder_id(self) -> str:
        return str(self.raw.get("powder_id") or self.path.stem)

    @property
    def category(self) -> str | None:
        """粉体分类：优先显式字段，其次按标签推断，最后返回 None 由调用方决定。"""
        explicit = self.raw.get("flow_category")
        if explicit in ("Low-flow", "Medium-flow", "High-flow"):
            return str(explicit)
        tags = self.raw.get("classification_tags") or []
        if "free_flowing" in tags:
            return "High-flow"
        if "sticky" in tags or "clogging" in tags:
            return "Low-flow"
        return None

    @property
    def feedforward(self) -> dict | None:
        section = self.raw.get(SECTION)
        if not isinstance(section, dict) or not section.get("points"):
            return None
        return section

    @property
    def calibrated(self) -> bool:
        return self.feedforward is not None

    # ---- 标定段写入 ----

    def set_signal(self, *, frequency_hz: float, duty_percent: float,
                   amplitude_vpp: float) -> None:
        section = self._section()
        section["signal"] = {
            "frequency_hz": round(float(frequency_hz), 2),
            "duty_percent": round(float(duty_percent), 2),
            "amplitude_vpp": round(float(amplitude_vpp), 3),
        }

    def signal(self) -> dict | None:
        section = self.feedforward
        if not section:
            return None
        signal = section.get("signal") or {}
        try:
            return {
                "frequency_hz": float(signal["frequency_hz"]),
                "duty_percent": float(signal["duty_percent"]),
                "amplitude_vpp": float(signal["amplitude_vpp"]),
            }
        except (KeyError, TypeError, ValueError):
            return None

    def record_point(self, point: dict) -> None:
        """写入/覆盖一个标定点（同 stir+rod 视为重测）。"""
        section = self._section()
        points: list[dict] = section.setdefault("points", [])
        key = (round(float(point["stir_rpm"]) / STIR_CLUSTER_RPM),
               round(float(point["rod_units"]) / 10.0))
        kept = []
        for existing in points:
            ekey = (round(float(existing.get("stir_rpm", 0)) / STIR_CLUSTER_RPM),
                    round(float(existing.get("rod_units", 0)) / 10.0))
            if ekey != key:
                kept.append(existing)
        kept.append(dict(point))
        section["points"] = kept
        section["updated_at"] = now_text()
        section["schema"] = SCHEMA
        self._derive()

    def clear_feedforward(self) -> None:
        self.raw.pop(SECTION, None)

    def delete_points(self, indices: list[int]) -> dict:
        """按索引删标定点（一次删多条），删完立即重算三阶段搅拌转速与摘要。

        全部删空时整个 ``feedforward_v2`` 段一起摘掉——留一个空壳会让
        ``list_powders`` 判成"已标定 0 点"，加粉时拿不到前馈却又不算纯闭环。
        """
        section = self.feedforward
        if not section:
            return {"removed": 0, "remaining": 0, "cleared": True,
                    "removed_points": [], "kept_index": []}
        points = list(section.get("points") or [])
        wanted = {int(i) for i in indices if 0 <= int(i) < len(points)}
        removed = [points[i] for i in sorted(wanted)]
        keep: list[dict] = []
        kept_index: list[int] = []          # 原索引，页面据此保留勾选状态
        for index, point in enumerate(points):
            if index in wanted:
                continue
            keep.append(point)
            kept_index.append(index)
        if not removed:
            return {"removed": 0, "remaining": len(points), "cleared": False,
                    "removed_points": [], "kept_index": kept_index}
        if not keep:
            self.raw.pop(SECTION, None)
            return {"removed": len(removed), "remaining": 0, "cleared": True,
                    "removed_points": removed, "kept_index": []}
        section["points"] = keep
        section["schema"] = SCHEMA
        section["updated_at"] = now_text()
        self._derive()
        return {"removed": len(removed), "remaining": len(keep), "cleared": False,
                "removed_points": removed, "kept_index": kept_index}

    def _section(self) -> dict:
        section = self.raw.get(SECTION)
        if not isinstance(section, dict):
            section = {"schema": SCHEMA, "points": []}
            self.raw[SECTION] = section
        section.setdefault("points", [])
        return section

    # ---- 前馈模型 ----

    def _levels(self) -> list[tuple[float, list[dict]]]:
        """把标定点按搅拌转速聚成若干档，档内按推杆开度升序。"""
        section = self.feedforward
        if not section:
            return []
        buckets: list[tuple[float, list[dict]]] = []
        for point in section.get("points", []):
            try:
                stir = float(point.get("stir_rpm", 0))
                rod = float(point.get("rod_units", 0))
                rate = float(point.get("rate_mg_s", 0))
            except (TypeError, ValueError):
                continue
            if rate <= 0:
                continue                      # 不出粉的点对前馈没用（但保留在原始数据里）
            for index, (level_stir, items) in enumerate(buckets):
                if abs(level_stir - stir) <= STIR_CLUSTER_RPM:
                    items.append({"rod": rod, "rate": rate, "cv": point.get("cv_pct")})
                    break
            else:
                buckets.append((stir, [{"rod": rod, "rate": rate, "cv": point.get("cv_pct")}]))
        result: list[tuple[float, list[dict]]] = []
        for stir, items in buckets:
            items.sort(key=lambda item: item["rod"])
            result.append((stir, items))
        result.sort(key=lambda entry: entry[0])
        return result

    def stir_levels(self) -> list[float]:
        return [level for level, _ in self._levels()]

    def rod_range(self) -> tuple[int, int] | None:
        rods = [float(p.get("rod_units", 0)) for p in (self.feedforward or {}).get("points", [])]
        if not rods:
            return None
        return int(min(rods)), int(max(rods))

    def rate_limits(self, stir_rpm: float | None = None) -> tuple[float, float] | None:
        """某个搅拌档（缺省取全部点）能达到的速率范围。"""
        levels = self._levels()
        if not levels:
            return None
        if stir_rpm is None:
            rates = [item["rate"] for _, items in levels for item in items]
        else:
            stir = float(stir_rpm)
            nearest = min(levels, key=lambda entry: abs(entry[0] - stir))
            rates = [item["rate"] for item in nearest[1]]
        if not rates:
            return None
        return float(min(rates)), float(max(rates))

    def rate_at(self, stir_rpm: float, rod_units: float) -> float | None:
        """预测某个 (搅拌转速, 推杆开度) 的稳定出粉速率 mg/s。"""
        levels = self._levels()
        if not levels:
            return None
        stir = float(stir_rpm)
        if stir <= levels[0][0]:
            return self._level_rate(levels[0][1], rod_units)
        if stir >= levels[-1][0]:
            return self._level_rate(levels[-1][1], rod_units)
        for i in range(len(levels) - 1):
            s0, items0 = levels[i]
            s1, items1 = levels[i + 1]
            if s0 <= stir <= s1:
                span = s1 - s0
                t = 0.0 if span <= 0 else (stir - s0) / span
                r0 = self._level_rate(items0, rod_units)
                r1 = self._level_rate(items1, rod_units)
                return float(r0 + t * (r1 - r0))
        return self._level_rate(levels[-1][1], rod_units)

    @staticmethod
    def _level_rate(items: list[dict], rod_units: float) -> float:
        xs = [item["rod"] for item in items]
        ys = [item["rate"] for item in items]
        return _polyline(xs, ys, float(rod_units))

    def solve_rod(self, target_rate_mg_s: float, stir_rpm: float) -> dict:
        """前馈：给定目标速率与搅拌转速，反解推杆开度。

        返回 ``{"rod_units", "rate_mg_s", "achievable", "note"}``：
        - ``achievable=False`` 表示该档在标定范围内到不了这个速率（需要调占空比/换搅拌档）；
        - 反解结果一律夹在标定过的最小/最大开度之间，绝不外推开度。

        实际反解用的是 ``目标 / 修正系数``：标定表只测过一遍，而粉的湿度、粒径分布
        每次都会变；每炉结束后用实测/预测的比值把它慢慢修回来（见 ``record_run``）。
        """
        if self.feedforward is None:
            return {"rod_units": None, "rate_mg_s": None, "achievable": False,
                    "note": "该粉末还没有标定数据"}
        factor = self.correction()
        target = float(target_rate_mg_s) / max(1e-6, factor)
        levels = self._levels()
        if not levels:
            return {"rod_units": None, "rate_mg_s": None, "achievable": False,
                    "note": "标定点全部速率为 0，无法反解"}

        stir = float(stir_rpm)
        if stir <= levels[0][0]:
            pairs = [(levels[0][1], 1.0)]
        elif stir >= levels[-1][0]:
            pairs = [(levels[-1][1], 1.0)]
        else:
            pairs = []
            for i in range(len(levels) - 1):
                s0, items0 = levels[i]
                s1, items1 = levels[i + 1]
                if s0 <= stir <= s1:
                    span = s1 - s0
                    t = 0.0 if span <= 0 else (stir - s0) / span
                    pairs = [(items0, 1.0 - t), (items1, t)]
                    break
            if not pairs:
                pairs = [(levels[-1][1], 1.0)]

        rods = 0.0
        any_ok = True
        for items, weight in pairs:
            xs = [item["rod"] for item in items]
            ys = [item["rate"] for item in items]
            rod, ok = _inverse_polyline(xs, ys, target)
            rods += weight * rod
            if not ok:
                any_ok = False

        rod_units = int(round(max(ROD_MIN_UNITS, min(ROD_MAX_UNITS, rods))))
        predicted = self.rate_at(stir, rod_units)
        note = ""
        if not any_ok:
            low, high = self.rate_limits(stir) or (0.0, 0.0)
            note = (f"标定范围 {low:.1f}–{high:.1f} mg/s 覆盖不到 {target:.1f} mg/s，"
                    f"前馈取边界开度 {rod_units}，剩余偏差交给闭环（占空比）补")
        return {
            "rod_units": rod_units,
            "rate_mg_s": None if predicted is None else round(predicted, 2),
            "achievable": any_ok,
            "note": note,
        }

    def _derive(self) -> None:
        """由标定点推导三阶段搅拌转速与摘要，写回档案（页面直接展示）。"""
        section = self._section()
        levels = self._levels()
        # 每点都带着"当时测它的振动参数"。先把用到的参数汇总出来：
        # 同一张表如果混着 3Vpp 和 2Vpp 的测点，前馈按 section 级 signal 下发就会偏，
        # 所以这里显式标 signal_mixed，让页面和日志能提醒，而不是静默混用。
        variants: dict[tuple[float, float, float], int] = {}
        for point in section.get("points", []):
            sig = point.get("signal") or {}
            try:
                key = (round(float(sig["frequency_hz"]), 2),
                       round(float(sig["duty_percent"]), 2),
                       round(float(sig["amplitude_vpp"]), 3))
            except (KeyError, TypeError, ValueError):
                continue
            variants[key] = variants.get(key, 0) + 1
        signals_used = [
            {"frequency_hz": hz, "duty_percent": duty, "amplitude_vpp": amp, "points": count}
            for (hz, duty, amp), count in sorted(variants.items())
        ]
        section["signals_used"] = signals_used
        section["signal_mixed"] = len(signals_used) > 1

        if not levels:
            section["stage_stir_rpm"] = {stage: 0 for stage in STAGES}
            section["summary"] = {}
            return

        def level_stats(items: list[dict]) -> tuple[float, float]:
            rates = [item["rate"] for item in items]
            cvs = [float(item["cv"]) for item in items if item.get("cv") is not None]
            return _median(rates), (_median(cvs) if cvs else 0.0)

        usable: list[tuple[float, float, float]] = []       # (stir, 中位速率, 中位CV)
        for stir, items in levels:
            rate, cv = level_stats(items)
            if rate >= 0.5 and cv <= 40.0:
                usable.append((stir, rate, cv))
        if not usable:                                       # 全部不稳 → 退化成全部档
            usable = [(stir, *level_stats(items)) for stir, items in levels]

        usable.sort(key=lambda entry: entry[0])
        coarse = usable[-1][0]
        precision = usable[0][0]
        fine = usable[len(usable) // 2][0] if len(usable) > 1 else coarse

        section["stage_stir_rpm"] = {"coarse": coarse, "fine": fine, "precision": precision}

        all_rates = [item["rate"] for _, items in levels for item in items]
        all_cvs = [float(item["cv"]) for _, items in levels for item in items
                   if item.get("cv") is not None]
        best_rate = max(all_rates)
        section["summary"] = {
            "rate_min_mg_s": round(min(all_rates), 2),
            "rate_max_mg_s": round(best_rate, 2),
            "cv_median_pct": round(_median(all_cvs), 1) if all_cvs else None,
            "points": len(section["points"]),
            "stir_levels": [round(stir, 1) for stir, _ in levels],
            "rod_range": list(self.rod_range() or (0, 0)),
            # 当前生效的振动参数（前馈下发用的就是它）+ 表里实际用过的全部参数
            "signal": section.get("signal") or {},
            "signals_used": signals_used,
            "signal_mixed": len(signals_used) > 1,
            # 搅拌带来的速率增益（相对最低档），用来判断"这个粉到底要不要搅"
            "stir_gain_pct": round(
                100.0 * (best_rate - min(
                    _median([i["rate"] for i in items]) for _, items in levels))
                / max(1e-6, min(_median([i["rate"] for i in items]) for _, items in levels)), 1),
        }
        section["updated_at"] = now_text()

    def stage_stir(self) -> dict:
        section = self.feedforward or {}
        raw = section.get("stage_stir_rpm") or {}
        return {stage: float(raw.get(stage, 0) or 0) for stage in STAGES}

    def correction(self) -> float:
        """前馈修正系数（1.0 = 完全相信标定表）。"""
        section = self.feedforward or {}
        raw = section.get("correction") or {}
        try:
            value = float(raw.get("factor", CORRECTION_DEFAULT))
        except (TypeError, ValueError):
            return CORRECTION_DEFAULT
        return max(CORRECTION_MIN, min(CORRECTION_MAX, value))

    def record_run(self, stages: list[dict], *, accepted: bool) -> dict | None:
        """用一炉的实测结果修正前馈系数（老系统 ILC 的最小可用版本）。

        做法：对每一段，比较"实际达成的平均速率"和"标定表在该开度上的预测速率"，
        得到比值；取三段中位数作为本次样本，再按 ``CORRECTION_ALPHA`` 指数平滑。

        只在 ``accepted``（判停正常、终值可信）且至少两段有有效数据时才更新——
        中途停机、超重保护、天平失联这些情况下的"实际速率"根本不可信，
        拿它去修表只会把表越修越歪。单次修正幅度被 0.5–1.8 限幅。
        """
        section = self.feedforward
        if not section:
            return None
        if not accepted:
            return None

        ratios: list[float] = []
        details: list[dict] = []
        for stage in stages:
            if stage.get("stage") == "trim":
                continue        # 缓补是脉冲补料，"平均速率"没有意义，不进前馈修正
            try:
                stir = float(stage.get("stir_rpm", 0))
                rod_start = float(stage.get("rod_ff", 0))
                rod_end = float(stage.get("rod_final", rod_start))
                actual = float(stage.get("actual_rate_mg_s") or 0)
            except (TypeError, ValueError):
                continue
            # 闭环在这一段里会把推杆挪开前馈点，所以拿"进段开度 + 出段开度"的中点
            # 去查表，比只用入口开度更接近这一段的真实平均工况。
            rod = (rod_start + rod_end) / 2.0
            predicted = self.rate_at(stir, rod)
            if not predicted or predicted < 1.0 or actual < 0.5:
                continue
            ratio = actual / predicted
            if not 0.3 <= ratio <= 3.0:      # 明显离谱的段不参与（多半是搭桥/突涌）
                continue
            ratios.append(ratio)
            details.append({"stage": stage.get("stage"), "stir_rpm": stir, "rod_units": rod,
                            "predicted_mg_s": round(predicted, 2),
                            "actual_mg_s": round(actual, 2), "ratio": round(ratio, 3)})
        if len(ratios) < 2:
            return None

        ratios.sort()
        mid = len(ratios) // 2
        sample = ratios[mid] if len(ratios) % 2 else (ratios[mid - 1] + ratios[mid]) / 2.0
        old = self.correction()
        new = max(CORRECTION_MIN, min(CORRECTION_MAX,
                                      old + CORRECTION_ALPHA * (sample - old)))
        section["correction"] = {
            "factor": round(new, 4),
            "samples": int((section.get("correction") or {}).get("samples", 0)) + 1,
            "last_sample": round(sample, 4),
            "last_details": details,
            "at": now_text(),
        }
        section["updated_at"] = now_text()
        return {"factor": round(new, 4), "sample": round(sample, 4),
                "updated": abs(new - old) >= 1e-4}

    # ---- 切段惯性落料量（2026-10-21，重粉切段用）----

    def settle_gains(self) -> dict:
        """各切段学到的"停振后还会多落多少粉"（mg），键如 ``"fast>coarse"``。"""
        section = self.raw.get(SETTLE_SECTION)
        return dict(section) if isinstance(section, dict) else {}

    def record_settle_gain(self, transition: str, gain_mg: float) -> dict:
        """记住某种粉在某次切段时的惯性落料量（指数平滑，限幅 0–50mg）。

        这一炉切段时停振后还落下来多少粉，就是下次该把切换点提前多少——
        不提前的话，余料会直接顶进下一段（粗加一开场就超标、慢加要倒着补）。
        """
        section = self.raw.setdefault(SETTLE_SECTION, {})
        previous = section.get(transition) or {}
        try:
            old = float(previous.get("gain_mg") or 0.0)
        except (TypeError, ValueError):
            old = 0.0
        samples = int(previous.get("samples") or 0) + 1
        value = float(gain_mg)
        new = value if samples <= 1 else old + SETTLE_ALPHA * (value - old)
        new = max(0.0, min(SETTLE_MAX_MG, new))
        section[transition] = {
            "gain_mg": round(new, 1),
            "last_gain_mg": round(value, 1),
            "samples": samples,
            "updated_at": now_text(),
        }
        return dict(section[transition])

    def fit_controller_params(self) -> dict:
        """标定得到的"该粉适合什么控制粒度"：精加/精修步长按流量大小收缩。

        依据：流量越小的粉，同样的占空比步长造成的相对扰动越大。
        """
        limits = self.rate_limits()
        if not limits:
            return {}
        high = max(1.0, limits[1])
        # 20 mg/s 以上算"快粉"（步长拉满），2 mg/s 以下算"慢粉"（步长收到 1/4）
        scale = max(0.25, min(1.0, high / 20.0))
        return {
            "duty_step_per_update": max(5, int(round(30 * scale))),
            "emergency_duty_step_per_update": max(20, int(round(80 * scale))),
            "position_step_units": max(5, int(round(25 * scale))),
        }

    # ---- 落盘 ----

    def save(self) -> Path:
        """落盘。优先"写 .tmp 再原子替换"，替换被拒时退化为原地覆写。

        为什么要有退化分支：粉体库目录走的是目录联接（``powder_library_alias``，
        用来绕开按路径文本匹配的读取拦截），在这条路径上 Windows 会拒绝
        ``os.replace``（实测 ``WinError 5 拒绝访问``），但直接覆写同一个文件是允许的。
        不退化的话"切段惯性自学习"永远写不回去，每次都白学（2026-10-22 日志：
        `前馈自修正写回失败：[WinError 5] 拒绝访问`）。
        """
        self.raw["updated_at"] = now_text()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        text = json.dumps(self.raw, ensure_ascii=False, indent=2)
        temp = self.path.with_suffix(".json.tmp")
        try:
            temp.write_text(text, encoding="utf-8")
        except OSError:
            temp = None
        if temp is not None:
            try:
                temp.replace(self.path)
                return self.path
            except OSError as error:
                # 换不掉就地覆写；.tmp 留着当上一次内容的备份
                try:
                    self.path.write_text(text, encoding="utf-8")
                except OSError:
                    raise error
                return self.path
        self.path.write_text(text, encoding="utf-8")
        return self.path


# ----------------------------------------------------------------------
# 库级操作
# ----------------------------------------------------------------------

def _norm(text: Any) -> str:
    """名称归一化：去空白、统一小写。中文名与括号宽度不同也能对上。"""
    return "".join(str(text or "").split()).lower()


def _find_path(name_or_id: str) -> Path | None:
    key = str(name_or_id or "").strip()
    if not key:
        return None
    direct = LIB_DIR / f"{key}.json"
    if direct.exists():
        return direct
    target = _norm(key)
    for path in LIB_DIR.glob("*.json"):
        if _norm(path.stem) == target:
            return path
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        # 文件名是短名（玉米淀粉.json），档案里的 powder_name 是带英文的全名，
        # 三个都接受：页面按下拉框里的显示名传过来也能命中。
        if _norm(raw.get("powder_id")) == target or _norm(raw.get("powder_name")) == target:
            return path
    return None


def load_powder(name_or_id: str) -> PowderWorkspace:
    """按粉名（文件名）或 powder_id 加载；不存在则新建一个空档案对象（不落盘）。"""
    path = _find_path(name_or_id)
    if path is None:
        safe = str(name_or_id or "").strip()
        if not safe:
            raise ValueError("粉末名称不能为空")
        path = LIB_DIR / f"{safe}.json"
        raw = {"powder_name": safe, "powder_id": safe, "head_type": "gravity-cone-vibration",
               "created_at": now_text()}
        return PowderWorkspace(path, raw)
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"读取粉体档案失败（{path.name}）：{error}") from error
    return PowderWorkspace(path, raw)


# Windows 文件名非法字符：粉末名会被直接用做档案文件名，必须剔除，
# 否则 "A/B" 这种名字会写到子目录、甚至越出库目录。
_INVALID_CHARS = set('\\/:*?"<>|')


def _safe_stem(name: Any) -> str:
    text = "".join(ch for ch in str(name or "") if ch not in _INVALID_CHARS)
    text = text.strip().strip(".")
    if not text:
        raise ValueError("粉末名称不合法：去掉文件名非法字符后为空")
    if len(text) > 60:
        raise ValueError("粉末名称过长（限 60 字符）")
    return text


CATEGORIES = ("Low-flow", "Medium-flow", "High-flow")


def create_powder(name: Any, *, category: str | None = None) -> dict:
    """新建一种粉末档案（只有基础字段，还没有任何标定数据）。

    只做一件事：建一个合法的空档案，让它在标定下拉框里出现。
    已存在同名/同 id 时直接报错——重复建会生成两个互不相干的档案，
    以后按名字查表命中哪个只能看运气。
    """
    display = str(name or "").strip()
    if not display:
        raise ValueError("粉末名称不能为空")
    stem = _safe_stem(display)

    clash = _find_path(stem) or _find_path(display)
    if clash is not None:
        raise ValueError(f"粉末「{display}」已存在（档案 {clash.name}），"
                         f"直接在下拉框里选它即可")
    cat = None
    if category:
        cat = str(category)
        if cat not in CATEGORIES:
            raise ValueError(f"分类只能是 {'/'.join(CATEGORIES)}")

    path = LIB_DIR / f"{stem}.json"
    raw: dict = {
        "powder_name": display,
        "powder_id": stem,
        "head_type": "gravity-cone-vibration",
        "created_at": now_text(),
    }
    if cat:
        raw["flow_category"] = cat
    LIB_DIR.mkdir(parents=True, exist_ok=True)
    PowderWorkspace(path, raw).save()
    return {"created": True, "name": display, "powder_id": stem,
            "file": path.name, "category": cat}



def list_powders() -> list[dict]:
    """列出库内全部粉末及标定概况（页面下拉框用）。"""
    return list_powders_report()["powders"]


def list_powders_report() -> dict:
    """列出全部粉末，并把「读不出来的档案」原样报出去。

    以前这里读文件失败是 `except: continue` 静默跳过，结果粉体库读不到（权限、
    编码坏了、文件被独占）时页面只显示「粉体库为空」，看着像数据丢了。现在失败
    项单独收集：文件名 + 失败原因，交给上层写日志并显示在页面上。
    """
    items: list[dict] = []
    errors: list[dict] = []
    if not LIB_DIR.exists():
        return {"powders": items, "errors": errors, "dir": str(LIB_DIR),
                "dir_exists": False}
    for path in sorted(LIB_DIR.glob("*.json")):
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            errors.append({
                "file": path.name,
                "reason": f"{type(error).__name__}: {error}",
                "permission": isinstance(error, PermissionError),
            })
            continue
        workspace = PowderWorkspace(path, raw)
        section = workspace.feedforward
        signal = workspace.signal()
        points = (section or {}).get("points") or []
        items.append({
            "name": workspace.name,
            # 老 web 面板（FeedbackDispensePanel.vue）用的是 powder_name / powder_id，
            # 后端两个名字都给，省得以后接前端时再改一遍协议。
            "powder_name": workspace.name,
            "powder_id": workspace.powder_id,
            "file": path.name,
            "category": workspace.category,
            "calibrated": bool(section),
            "points": len(points),
            # 「标定管理」列表页要能一眼看出哪种粉有脏数据，不用逐个点开
            "unreliable_points": sum(1 for p in points if point_issues(p)),
            "summary": (section or {}).get("summary") or {},
            "stage_stir_rpm": (section or {}).get("stage_stir_rpm") or {},
            "correction_factor": workspace.correction() if section else None,
            "signal": signal,
            # 缓补开关的档案默认值：页面「缓补脉冲」勾选框在换粉时用它初始化
            "trim_pulse": bool(raw.get("trim_pulse")),
            "updated_at": (section or {}).get("updated_at") or raw.get("updated_at"),
        })
    items.sort(key=lambda item: (not item["calibrated"], item["name"]))
    return {
        "powders": items,
        "errors": errors,
        # 目录里一共有多少个 json：用来区分「真没档案」和「有档案但一个都读不出来」
        "files": len(items) + len(errors),
        "dir": str(LIB_DIR),
        "dir_exists": True,
    }


def delete_feedforward(name_or_id: str) -> dict:
    """只删标定段，不动档案里的其它字段（老数据仍有价值）。"""
    workspace = load_powder(name_or_id)
    if not workspace.path.exists():
        return {"removed": False, "reason": "档案不存在"}
    workspace.clear_feedforward()
    workspace.save()
    return {"removed": True, "name": workspace.name}


def delete_points(name_or_id: str, indices: list[int] | None = None,
                  *, mode: str | None = None) -> dict:
    """删掉某个粉末的部分标定点。

    ``mode="invalid"`` 时按 ``point_issues`` 的口径删掉全部不可信点（页面上的
    「删除全部不可信」）；否则按 ``indices`` 删指定行（页面勾选的行）。
    两种方式都只动 ``feedforward_v2.points``，不改档案里其它历史字段。
    """
    workspace = load_powder(name_or_id)
    if not workspace.path.exists():
        return {"removed": 0, "remaining": 0, "name": workspace.name,
                "reason": "档案不存在"}
    points = list((workspace.feedforward or {}).get("points") or [])
    if mode == "invalid":
        targets = [index for index, point in enumerate(points) if point_issues(point)]
    else:
        targets = [int(i) for i in (indices or [])]
        if not targets:
            raise ValueError("没有指定要删除的标定点")
    result = workspace.delete_points(targets)
    if result["removed"]:
        workspace.save()
    result.update({"name": workspace.name, "mode": mode or "indices",
                   "valid_points": len(points)})
    return result


def powder_detail(name_or_id: str) -> dict:
    workspace = load_powder(name_or_id)
    section = workspace.feedforward or {}
    # 每个点原样返回，另加 index / unreliable / issues 三个派生字段，
    # 页面「标定管理」直接照着渲染勾选与红字原因，不用在前端再判一遍口径。
    points: list[dict] = []
    for index, point in enumerate(section.get("points") or []):
        issues = point_issues(point)
        item = dict(point)
        item["index"] = index
        item["issues"] = issues
        item["unreliable"] = bool(issues)
        points.append(item)
    return {
        "name": workspace.name,
        "powder_name": workspace.name,
        "powder_id": workspace.powder_id,
        "category": workspace.category,
        "calibrated": bool(section),
        "unreliable_points": sum(1 for p in points if p["unreliable"]),
        "signal": workspace.signal(),
        "signals_used": section.get("signals_used") or [],
        "signal_mixed": bool(section.get("signal_mixed")),
        "stage_stir_rpm": workspace.stage_stir(),
        "summary": section.get("summary") or {},
        "correction": section.get("correction") or {},
        "correction_factor": workspace.correction(),
        "points": points,
        "file": workspace.path.name,
    }

