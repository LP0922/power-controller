# -*- coding: utf-8 -*-
"""把 runs/dispense_*.json 汇总成加粉记录表。

筛选口径（2026-10-23 16:43 用户最终要求）：
  - **结束方式只看「正常判停」和「缓补收尾」这两种**，其余一律不进统计；
  - 中途手动取消（stop_reason 含"操作员停止"）单独列出；
  - 保护中止（超时保护/超重保护/天平失联/推杆未到位）也算没跑完，单独列出备查。

也就是说：明细/汇总只统计"程序自己判断收得住、自己收的"那些加粉次数。

用法：
    python lab/tools/collect_dispense_history.py            # 看统计
    python lab/tools/collect_dispense_history.py --xlsx out.xlsx
"""
from __future__ import annotations

import argparse
import glob
import json
import re
import statistics
from collections import Counter
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
RUN_DIR = PROJECT_ROOT / "server" / "src" / "powder_sampling_control" / "control_panel" / "runs"

# 这些 stop_reason 视为"中途没跑完"（用户取消 / 人为中止）
CANCELLED_PATTERNS = ("操作员停止", "force stop")

# 这两类不是真实粉料的加粉（仿真粉=离线/假粉测试，未选粉=纯闭环调试），
# 2026-10-23 16:45 用户要求**从统计里彻底删掉**，明细和汇总都不出现。
EXCLUDE_POWDERS = ("仿真粉", "（未选粉）")


def is_excluded_powder(name: str) -> bool:
    name = (name or "").strip()
    return any(name == p or name.startswith(p) for p in EXCLUDE_POWDERS)

# 正常收尾的两种结束方式（**只有这两种进统计**）：
#   "predicted tail …"                 → 正常判停（慢加判停直接收尾）
#   "缓补完成 …" / "缓补结束（已超验收）" / "缓补结束但未到目标 …" → 缓补收尾
NORMAL_ENDS = ("正常判停", "缓补收尾")
END_KIND_ORDER = ("缓补收尾", "正常判停", "超时保护", "超重保护", "天平失联", "推杆未到位", "其他")


def classify_end(reason: str) -> str:
    if reason.startswith("缓补"):
        return "缓补收尾"
    if reason.startswith("predicted tail"):
        return "正常判停"
    if reason.startswith("超时保护"):
        return "超时保护"
    if reason.startswith("超重保护"):
        return "超重保护"
    if reason.startswith("天平失联"):
        return "天平失联"
    if reason.startswith("推杆未在"):
        return "推杆未到位"
    return "其他"


def trim_pulses(reason: str) -> int | None:
    m = re.search(r"(\d+)\s*发脉冲", reason or "")
    return int(m.group(1)) if m else None


def stage_seconds(stages: list[dict], name: str) -> float | None:
    for s in stages or []:
        if s.get("stage") == name:
            v = s.get("duration_s")
            return float(v) if v is not None else None
    return None


def load_rows() -> tuple[list[dict], list[dict], list[dict], list[dict]]:
    """读全部记录，按口径分成四堆：(正常收尾, 保护中止, 中途取消, 已排除粉种)。

    正常收尾 = `NORMAL_ENDS`（正常判停 / 缓补收尾），这一堆才进统计；
    保护中止和手动取消各列一页，只做核对用；仿真粉/未选粉（`EXCLUDE_POWDERS`）
    直接丢掉，不出现任何一页。
    """
    normal, aborted, cancelled, excluded = [], [], [], []
    for path in sorted(glob.glob(str(RUN_DIR / "dispense_*.json"))):
        data = json.load(open(path, encoding="utf-8"))
        name = Path(path).name
        m = re.match(r"dispense_(\d{8})_(\d{6})_", name)
        stamp = f"{m.group(1)[:4]}-{m.group(1)[4:6]}-{m.group(1)[6:]} " \
                f"{m.group(2)[:2]}:{m.group(2)[2:4]}:{m.group(2)[4:]}" if m else ""
        reason = (data.get("stop_reason") or "").strip()
        stages = data.get("stages") or []
        target = float(data.get("target_mg") or 0)
        achieved = float(data.get("achieved_mg") or 0)
        row = {
            "start": stamp,
            "finished_at": data.get("finished_at") or "",
            "powder": data.get("powder") or "（未选粉）",
            "target_mg": target,
            "achieved_mg": achieved,
            "delta_mg": float(data.get("delta_mg") or (achieved - target)),
            "accept_low": float((data.get("acceptance_mg") or [0, 0])[0]),
            "accept_high": float((data.get("acceptance_mg") or [0, 0])[1]),
            "verdict": data.get("verdict") or "",
            "duration_s": float(data.get("duration_s") or 0),
            "end_kind": classify_end(reason),
            "trim_pulses": trim_pulses(reason),
            "fast_s": stage_seconds(stages, "fast"),
            "coarse_s": stage_seconds(stages, "coarse"),
            "slow_s": stage_seconds(stages, "slow"),
            "trim_s": stage_seconds(stages, "trim"),
            "start_mg": float((stages[0] or {}).get("start_mass_mg") or 0) if stages else 0.0,
            "amp_max": max([float(s.get("amp_max_vpp") or 0) for s in stages] or [0]) if stages else 0.0,
            "amp_changes": sum(int(s.get("amp_changes") or 0) for s in stages),
            "stop_reason": reason,
            "file": name,
        }
        if is_excluded_powder(row["powder"]):
            excluded.append(row)
        elif any(p in reason for p in CANCELLED_PATTERNS):
            cancelled.append(row)
        elif row["end_kind"] in NORMAL_ENDS:
            normal.append(row)
        else:
            aborted.append(row)
    return normal, aborted, cancelled, excluded


def summarize(rows: list[dict]) -> dict:
    if not rows:
        return {}
    deltas = [r["delta_mg"] for r in rows]
    absd = [abs(d) for d in deltas]
    dur = [r["duration_s"] for r in rows]
    verdicts = Counter(r["verdict"] for r in rows)
    ends = Counter(r["end_kind"] for r in rows)
    return {
        "n": len(rows),
        "合格": verdicts.get("合格", 0),
        "偏重": verdicts.get("偏重", 0),
        "偏轻": verdicts.get("偏轻", 0),
        "合格率": verdicts.get("合格", 0) / len(rows),
        "平均绝对偏差": statistics.fmean(absd),
        "最大绝对偏差": max(absd),
        "平均偏差": statistics.fmean(deltas),
        "平均用时": statistics.fmean(dur),
        "总用时": sum(dur),
        "ends": dict(ends),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--xlsx", default=None, help="导出 xlsx 的路径")
    ap.add_argument("--show", type=int, default=25, help="打印最近多少行")
    args = ap.parse_args()

    done, aborted, cancelled, excluded = load_rows()
    done.sort(key=lambda r: r["start"])

    s = summarize(done)
    print(f"记录总数 {len(done) + len(aborted) + len(cancelled) + len(excluded)}"
          f"，正常收尾 {len(done)}，保护中止 {len(aborted)}，中途取消 {len(cancelled)}"
          f"，仿真粉/未选粉已删 {len(excluded)}")
    print(f"口径：只统计结束方式为「正常判停 / 缓补收尾」的加粉次数；"
          f"仿真粉、未选粉（纯调试）已剔除")
    print(f"正常收尾的加粉：合格 {s['合格']}（{s['合格率']:.0%}）、偏重 {s['偏重']}、偏轻 {s['偏轻']}")
    print(f"偏差：平均 {s['平均偏差']:+.1f} mg，平均绝对 {s['平均绝对偏差']:.1f} mg，"
          f"最大 {s['最大绝对偏差']:.1f} mg")
    print(f"用时：平均 {s['平均用时']:.1f}s，合计 {s['总用时'] / 60:.1f}min")
    print(f"结束方式：{s['ends']}")

    print("\n按日期：")
    by_day: dict[str, list[dict]] = {}
    for r in done:
        by_day.setdefault(r["start"][:10], []).append(r)
    for day, rs in sorted(by_day.items()):
        d = summarize(rs)
        print(f"  {day}  {d['n']:3d} 次加粉  合格 {d['合格']:2d}（{d['合格率']:.0%}）"
              f"  平均|偏差| {d['平均绝对偏差']:.1f}mg  平均用时 {d['平均用时']:.0f}s")

    print("\n按目标量段：")
    bands = [(0, 300, "<300"), (300, 1000, "300–1000"), (1000, 2000, "1000–2000"),
             (2000, 5000, "2000–5000"), (5000, 10 ** 9, ">5000")]
    for lo, hi, label in bands:
        rs = [r for r in done if lo <= r["target_mg"] < hi]
        if not rs:
            continue
        d = summarize(rs)
        print(f"  {label:10s} {d['n']:3d} 次加粉  合格 {d['合格']:2d}（{d['合格率']:3.0%}）"
              f"  平均|偏差| {d['平均绝对偏差']:5.1f}mg  平均用时 {d['平均用时']:5.0f}s")

    print(f"\n最近 {args.show} 次（含判定）：")
    print(f"{'开始时间':16s} {'粉体':6s} {'目标':>6s} {'实得':>8s} {'偏差':>7s} "
          f"{'判定':4s} {'用时':>6s} {'结束方式':8s} 缓补")
    for r in done[-args.show:]:
        print(f"{r['start']:16s} {r['powder'][:6]:6s} {r['target_mg']:6.0f} "
              f"{r['achieved_mg']:8.1f} {r['delta_mg']:+7.1f} {r['verdict']:4s} "
              f"{r['duration_s']:6.1f} {r['end_kind']:8s} "
              f"{'' if r['trim_pulses'] is None else r['trim_pulses']}")

    if args.xlsx:
        export_xlsx(done, aborted, cancelled, Path(args.xlsx), len(excluded))
        print(f"\n已导出 {args.xlsx}")
    return 0


def export_xlsx(normal: list[dict], aborted: list[dict],
                cancelled: list[dict], out: Path, n_excluded: int = 0) -> None:
    import xlsxwriter

    wb = xlsxwriter.Workbook(str(out), {"nan_inf_to_errors": True})
    f_head = wb.add_format({"bold": True, "font_color": "#FFFFFF", "bg_color": "#1F4E78",
                            "align": "center", "valign": "vcenter", "border": 1, "font_size": 10})
    f_ok = wb.add_format({"bg_color": "#E2EFDA", "border": 1, "font_size": 10})
    f_bad = wb.add_format({"bg_color": "#FCE4E4", "border": 1, "font_size": 10})
    f_warn = wb.add_format({"bg_color": "#FFF2CC", "border": 1, "font_size": 10})
    f_txt = wb.add_format({"border": 1, "font_size": 10})
    f_num1 = wb.add_format({"border": 1, "font_size": 10, "num_format": "0.0"})
    f_sig = wb.add_format({"border": 1, "font_size": 10, "num_format": "+0.0;-0.0;0.0"})
    f_title = wb.add_format({"bold": True, "font_size": 12, "font_color": "#1F4E78"})
    f_key = wb.add_format({"bold": True, "font_size": 10, "bg_color": "#DDEBF7", "border": 1})

    # ---------------- 明细 ----------------
    cols = [("开始时间", 19), ("完成时间", 19), ("粉体", 12), ("目标 mg", 10),
            ("实得 mg", 10), ("偏差 mg", 10), ("验收区", 14),
            ("判定", 8), ("用时 s", 9), ("结束方式", 12), ("缓补发数", 9),
            ("快加 s", 8), ("粗加 s", 8), ("慢加 s", 8), ("缓补 s", 8),
            ("峰值振幅 Vpp", 12), ("调幅次数", 9), ("停止原因", 66)]
    ws = wb.add_worksheet("加粉明细（正常收尾）")
    ws.write_row(0, 0, [c[0] for c in cols], f_head)
    for i, (_, w) in enumerate(cols):
        ws.set_column(i, i, w)

    for row, r in enumerate(normal, start=1):
        is_ok = r["verdict"] == "合格"
        base = f_ok if is_ok else f_bad
        ws.write_string(row, 0, r["start"], f_txt)
        ws.write_string(row, 1, r["finished_at"], f_txt)
        ws.write_string(row, 2, r["powder"], f_txt)
        ws.write_number(row, 3, r["target_mg"], f_num1)
        ws.write_number(row, 4, r["achieved_mg"], f_num1)
        ws.write_number(row, 5, r["delta_mg"], f_sig if is_ok else base)
        ws.write_string(row, 6, f"{r['accept_low']:.0f}~{r['accept_high']:.0f}", f_txt)
        ws.write_string(row, 7, r["verdict"], base)
        ws.write_number(row, 8, r["duration_s"], f_num1)
        ws.write_string(row, 9, r["end_kind"], f_txt)
        if r["trim_pulses"] is None:
            ws.write_string(row, 10, "", f_txt)
        else:
            ws.write_number(row, 10, r["trim_pulses"], f_txt)
        for j, key in enumerate(("fast_s", "coarse_s", "slow_s", "trim_s"), start=11):
            ws.write_number(row, j, r[key] if r[key] is not None else 0, f_num1)
        ws.write_number(row, 15, r["amp_max"], f_num1)
        ws.write_number(row, 16, r["amp_changes"], f_txt)
        ws.write_string(row, 17, r["stop_reason"], f_txt)
    ws.freeze_panes(1, 3)
    ws.autofilter(0, 0, max(1, len(normal)), len(cols) - 1)

    # ---------------- 汇总 ----------------
    sm = wb.add_worksheet("汇总")
    sm.set_column(0, 0, 26)
    sm.set_column(1, 5, 15)
    sm.write(0, 0, "粉体加粉记录汇总", f_title)
    sm.write(1, 0, "口径：只统计结束方式为「正常判停 / 缓补收尾」的加粉次数；"
                   "仿真粉与未选粉（纯调试）已删除，保护中止与中途取消各列一页备查", f_txt)

    head = ["统计口径", "加粉次数", "合格", "合格率", "平均偏差 mg", "平均绝对偏差 mg", "平均用时 s"]
    sm.write_row(3, 0, head, f_head)
    def stat_row(i: int, name: str, rs: list[dict]) -> None:
        s = summarize(rs)
        sm.write_string(i, 0, name, f_key)
        sm.write_number(i, 1, s["n"], f_txt)
        sm.write_number(i, 2, s["合格"], f_txt)
        sm.write_number(i, 3, round(s["合格率"], 4), wb.add_format(
            {"border": 1, "font_size": 10, "num_format": "0.0%"}))
        sm.write_number(i, 4, round(s["平均偏差"], 1), f_num1)
        sm.write_number(i, 5, round(s["平均绝对偏差"], 1), f_num1)
        sm.write_number(i, 6, round(s["平均用时"], 1), f_num1)

    stat_row(4, "正常收尾（正常判停+缓补）", normal)
    stat_row(5, "其中：正常判停", [r for r in normal if r["end_kind"] == "正常判停"])
    stat_row(6, "其中：缓补收尾", [r for r in normal if r["end_kind"] == "缓补收尾"])
    sm.write_string(7, 0, "另（不计入统计）", f_key)
    sm.write_string(7, 1, f"仿真粉/未选粉 {n_excluded} 次（已删） / 保护中止 {len(aborted)} 次 / "
                          f"中途取消 {len(cancelled)} 次", f_txt)

    r = 10
    sm.write(r, 0, "按日期", f_title)
    sm.write_row(r + 1, 0, ["日期", "加粉次数", "合格", "合格率", "平均偏差 mg", "平均绝对偏差 mg", "平均用时 s"], f_head)
    by_day: dict[str, list[dict]] = {}
    for x in normal:
        by_day.setdefault(x["start"][:10], []).append(x)
    for i, (day, rs) in enumerate(sorted(by_day.items()), start=r + 2):
        sm.write_string(i, 0, day, f_txt)
        stat_row(i, day, rs)          # 复用（会覆盖 A 列文字为标准键格式，下面重写）
        sm.write_string(i, 0, day, f_txt)

    r = r + len(by_day) + 3
    sm.write(r, 0, "按目标量段", f_title)
    sm.write_row(r + 1, 0, ["目标区间 mg", "加粉次数", "合格", "合格率", "平均偏差 mg", "平均绝对偏差 mg", "平均用时 s"], f_head)
    bands = [(0, 300, "<300"), (300, 1000, "300–1000"), (1000, 2000, "1000–2000"),
             (2000, 5000, "2000–5000"), (5000, 10 ** 9, ">5000")]
    i = r + 2
    for lo, hi, label in bands:
        rs = [x for x in normal if lo <= x["target_mg"] < hi]
        if not rs:
            continue
        stat_row(i, label, rs)
        sm.write_string(i, 0, label, f_txt)
        i += 1

    r = i + 1
    sm.write(r, 0, "按粉体", f_title)
    sm.write_row(r + 1, 0, ["粉体", "加粉次数", "合格", "合格率", "平均偏差 mg", "平均绝对偏差 mg", "平均用时 s"], f_head)
    by_pow: dict[str, list[dict]] = {}
    for x in normal:
        by_pow.setdefault(x["powder"], []).append(x)
    i = r + 2
    for name, rs in sorted(by_pow.items(), key=lambda kv: -len(kv[1])):
        stat_row(i, name, rs)
        sm.write_string(i, 0, name, f_txt)
        i += 1

    r = i + 1
    sm.write(r, 0, "结束方式（统计口径内）", f_title)
    ends: dict[str, int] = {}
    for x in normal:
        ends[x["end_kind"]] = ends.get(x["end_kind"], 0) + 1
    i = r + 1
    for k, v in sorted(ends.items(), key=lambda kv: -kv[1]):
        sm.write_string(i, 0, k, f_txt)
        sm.write_number(i, 1, v, f_txt)
        i += 1

    r = i + 1
    sm.write(r, 0, "被剔除的记录", f_title)
    sm.write_string(r + 1, 0, "保护中止（超时/超重/天平失联/推杆不到位）")
    sm.write_number(r + 1, 1, len(aborted), f_txt)
    sm.write_string(r + 2, 0, "中途手动取消（操作员停止）")
    sm.write_number(r + 2, 1, len(cancelled), f_txt)

    # ---------------- 最近 10 次加粉（方便一眼看到最新几次做实的情况） ----------------
    r = r + 4
    sm.write(r, 0, "最近 10 次加粉（正常收尾）", f_title)
    recent_head = ["开始时间", "粉体", "目标 mg", "实得 mg", "偏差 mg", "判定", "用时 s", "结束方式"]
    sm.write_row(r + 1, 0, recent_head, f_head)
    for i2, x in enumerate(sorted(normal, key=lambda y: y["start"])[-10:], start=r + 2):
        sm.write_string(i2, 0, x["start"], f_txt)
        sm.write_string(i2, 1, x["powder"], f_txt)
        sm.write_number(i2, 2, x["target_mg"], f_num1)
        sm.write_number(i2, 3, x["achieved_mg"], f_num1)
        sm.write_number(i2, 4, x["delta_mg"], f_sig if x["verdict"] == "合格" else f_bad)
        sm.write_string(i2, 5, x["verdict"], f_ok if x["verdict"] == "合格" else f_bad)
        sm.write_number(i2, 6, x["duration_s"], f_num1)
        sm.write_string(i2, 7, x["end_kind"], f_txt)

    # ---------------- 保护中止（已剔除） ----------------
    if aborted:
        wa = wb.add_worksheet("保护中止（已剔除）")
        acols = [("开始时间", 19), ("粉体", 12), ("目标 mg", 10), ("中止时实得 mg", 14),
                 ("偏差 mg", 10), ("已用时 s", 10), ("结束方式", 12), ("停止原因", 46)]
        wa.write_row(0, 0, [c[0] for c in acols], f_head)
        for i2, (_, w) in enumerate(acols):
            wa.set_column(i2, i2, w)
        for row, x in enumerate(aborted, start=1):
            wa.write_string(row, 0, x["start"], f_txt)
            wa.write_string(row, 1, x["powder"], f_txt)
            wa.write_number(row, 2, x["target_mg"], f_num1)
            wa.write_number(row, 3, x["achieved_mg"], f_num1)
            wa.write_number(row, 4, x["delta_mg"], f_sig)
            wa.write_number(row, 5, x["duration_s"], f_num1)
            wa.write_string(row, 6, x["end_kind"], f_warn)
            wa.write_string(row, 7, x["stop_reason"], f_txt)
        wa.freeze_panes(1, 2)

    # ---------------- 已取消 ----------------
    wc = wb.add_worksheet("已取消（已剔除）")
    ccols = [("开始时间", 19), ("粉体", 12), ("目标 mg", 10), ("取消时实得 mg", 14),
             ("偏差 mg", 10), ("已用时 s", 10), ("停止原因", 40)]
    wc.write_row(0, 0, [c[0] for c in ccols], f_head)
    for i, (_, w) in enumerate(ccols):
        wc.set_column(i, i, w)
    for row, x in enumerate(cancelled, start=1):
        wc.write_string(row, 0, x["start"], f_txt)
        wc.write_string(row, 1, x["powder"], f_txt)
        wc.write_number(row, 2, x["target_mg"], f_num1)
        wc.write_number(row, 3, x["achieved_mg"], f_num1)
        wc.write_number(row, 4, x["delta_mg"], f_sig)
        wc.write_number(row, 5, x["duration_s"], f_num1)
        wc.write_string(row, 6, x["stop_reason"], f_txt)
    wc.freeze_panes(1, 2)

    wb.close()


if __name__ == "__main__":
    raise SystemExit(main())
