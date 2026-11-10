# -*- coding: utf-8 -*-
"""按起始日期统计加粉记录（默认 2026-10-27 起）。

和 `collect_dispense_history.py` 的区别：这里多一个「成功率」视角——把启动次数
（含没跑完的）也当分母，能看到真实一次成功率。

口径：
  - 剔粉种：仿真粉、未选粉（纯调试，与 collect_dispense_history 一致）；
  - 正常收尾 = 结束原因以「predicted tail」开头（正常判停）或「缓补」开头（缓补收尾）；
  - 中途取消 = stop_reason 含「操作员停止 / force stop」；
  - 其余进「保护中止」（超时/超重/天平失联/推杆未到位等）。

四个率：
  完成率     = 正常收尾 / 启动次数
  合格率     = 合格 / 正常收尾      （跑完的那些里，落在验收带宽内的比例）
  综合成功率 = 合格 / 启动次数      （一次按下去直接拿到合格结果的比例）
  超差率     = (偏重+偏轻) / 正常收尾

用法：
    python lab/tools/stats_dispense_since.py                       # 默认 2026-10-27 起
    python lab/tools/stats_dispense_since.py --since 2026-10-28
    python lab/tools/stats_dispense_since.py --xlsx outputs/加粉统计.xlsx
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

CANCELLED_PATTERNS = ("操作员停止", "force stop")
EXCLUDE_POWDERS = ("仿真粉", "（未选粉）")


def is_excluded_powder(name: str) -> bool:
    name = (name or "").strip()
    return any(name == p or name.startswith(p) for p in EXCLUDE_POWDERS)


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


def load_rows(since: str) -> list[dict]:
    rows: list[dict] = []
    for path in sorted(glob.glob(str(RUN_DIR / "dispense_*.json"))):
        name = Path(path).name
        m = re.match(r"dispense_(\d{8})_(\d{6})_", name)
        if not m:
            continue
        day = f"{m.group(1)[:4]}-{m.group(1)[4:6]}-{m.group(1)[6:]}"
        if day < since:
            continue
        data = json.load(open(path, encoding="utf-8"))
        powder = data.get("powder") or "（未选粉）"
        if is_excluded_powder(powder):
            continue
        stamp = f"{day} {m.group(2)[:2]}:{m.group(2)[2:4]}:{m.group(2)[4:]}"
        reason = (data.get("stop_reason") or "").strip()
        stages = data.get("stages") or []
        target = float(data.get("target_mg") or 0)
        achieved = float(data.get("achieved_mg") or 0)
        acc = data.get("acceptance_mg") or [target, target]
        end_kind = classify_end(reason)
        if any(p in reason for p in CANCELLED_PATTERNS):
            bucket = "中途取消"
        elif end_kind in ("正常判停", "缓补收尾"):
            bucket = "正常收尾"
        else:
            bucket = "保护中止"
        settings = (data.get("config") or {}).get("settings") or {}
        rows.append({
            "day": day,
            "start": stamp,
            "finished_at": data.get("finished_at") or "",
            "powder": powder,
            "target_mg": target,
            "achieved_mg": achieved,
            "delta_mg": float(data.get("delta_mg") if data.get("delta_mg") is not None else achieved - target),
            "accept_low": float(acc[0]),
            "accept_high": float(acc[1]),
            "verdict": data.get("verdict") or "",
            "duration_s": float(data.get("duration_s") or 0),
            "end_kind": end_kind,
            "bucket": bucket,
            "trim_pulses": trim_pulses(reason),
            "fast_s": stage_seconds(stages, "fast"),
            "coarse_s": stage_seconds(stages, "coarse"),
            "slow_s": stage_seconds(stages, "slow"),
            "trim_s": stage_seconds(stages, "trim"),
            "amp_max": max([float(s.get("amp_max_vpp") or 0) for s in stages] or [0]) if stages else 0.0,
            "amp_changes": sum(int(s.get("amp_changes") or 0) for s in stages),
            "rates": "/".join(
                f"{settings.get(k) or 0:g}" for k in
                ("fast_rate_mg_s", "coarse_rate_mg_s", "slow_rate_mg_s")
            ) if settings else "",
            "stop_reason": reason,
            "file": name,
        })
    return rows


def reason_category(r: dict) -> str:
    """把停止原因归成几类，便于看「没跑完」到底卡在哪一步。"""
    s = r["stop_reason"]
    if r["bucket"] == "正常收尾":
        return "正常收尾"
    if r["bucket"] == "中途取消":
        return "操作员取消"
    for k in ("超速熔断后已到", "沉降余料已到", "超重保护", "超时保护",
              "断料中止", "天平失联", "推杆未在"):
        if s.startswith(k):
            return k
    return "其他"


# 宽松口径：这两类虽然是"保护中止"，但实际是提前停住、质量已经落在验收区附近，
# 按一次加粉算跑完了（2026-10-30 观察：39 次里有 14 次判定合格）。
EARLY_STOP_PREFIX = ("超速熔断后已到", "沉降余料已到")


def is_wide_finish(r: dict) -> bool:
    return r["bucket"] == "正常收尾" or (
        r["bucket"] == "保护中止" and r["stop_reason"].startswith(EARLY_STOP_PREFIX)
    )


def stats(all_rows: list[dict], wide: bool = False) -> dict:
    normal = [r for r in all_rows if (is_wide_finish(r) if wide else r["bucket"] == "正常收尾")]
    aborted = [r for r in all_rows if r["bucket"] == "保护中止" and not (wide and is_wide_finish(r))]
    cancelled = [r for r in all_rows if r["bucket"] == "中途取消"]
    n_start = len(all_rows)
    n = len(normal)
    if n == 0:
        return {"n_start": n_start, "n": 0, "aborted": len(aborted), "cancelled": len(cancelled)}
    deltas = [r["delta_mg"] for r in normal]
    absd = [abs(d) for d in deltas]
    verdicts = Counter(r["verdict"] for r in normal)
    ok = verdicts.get("合格", 0)
    return {
        "n_start": n_start,
        "n": n,
        "aborted": len(aborted),
        "cancelled": len(cancelled),
        "ok": ok,
        "heavy": verdicts.get("偏重", 0),
        "light": verdicts.get("偏轻", 0),
        "finish_rate": n / n_start if n_start else 0.0,
        "ok_rate": ok / n,
        "overall_rate": ok / n_start if n_start else 0.0,
        "bad_rate": (verdicts.get("偏重", 0) + verdicts.get("偏轻", 0)) / n,
        "mean_delta": statistics.fmean(deltas),
        "mean_abs": statistics.fmean(absd),
        "median_abs": statistics.median(absd),
        "p90_abs": sorted(absd)[max(0, int(round(0.9 * (len(absd) - 1))))],
        "max_abs": max(absd),
        "mean_dur": statistics.fmean(r["duration_s"] for r in normal),
        "total_dur": sum(r["duration_s"] for r in normal),
        "within5": sum(1 for a in absd if a <= 5) / n,
        "within10": sum(1 for a in absd if a <= 10) / n,
        "within20": sum(1 for a in absd if a <= 20) / n,
    }


def fmt(s: dict, label: str) -> str:
    if not s.get("n"):
        return (f"  {label:12s} 启动 {s['n_start']:3d} 次，无正常收尾"
                f"（中止 {s['aborted']}、取消 {s['cancelled']}）")
    return (f"  {label:12s} 启动 {s['n_start']:3d}  "
            f"完成 {s['n']:3d}（{s['finish_rate']:4.0%}）  "
            f"合格 {s['ok']:3d}  合格率 {s['ok_rate']:4.0%}  "
            f"综合成功率 {s['overall_rate']:4.0%}  "
            f"|偏差| 均 {s['mean_abs']:5.1f} 中位 {s['median_abs']:5.1f} "
            f"P90 {s['p90_abs']:5.1f} 最大 {s['max_abs']:5.1f}  用时 {s['mean_dur']:5.0f}s")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--since", default="2026-10-27")
    ap.add_argument("--xlsx", default=None)
    ap.add_argument("--show", type=int, default=0, help="额外打印最近 N 次明细")
    args = ap.parse_args()

    rows = load_rows(args.since)
    if not rows:
        print(f"{args.since} 之后没有加粉记录")
        return 0
    rows.sort(key=lambda r: r["start"])
    s = stats(rows)

    print(f"统计区间：{args.since} 起 ～ 记录末尾（共 {len({r['day'] for r in rows})} 个工作日有记录）")
    print(f"启动次数 {s['n_start']}，正常收尾 {s['n']}，保护中止 {s['aborted']}，中途取消 {s['cancelled']}")
    print(f"完成率 {s['finish_rate']:.1%}｜合格率 {s['ok_rate']:.1%}（合格/正常收尾）"
          f"｜综合成功率 {s['overall_rate']:.1%}（合格/启动）｜超差率 {s['bad_rate']:.1%}")
    print(f"合格 {s['ok']}、偏重 {s['heavy']}、偏轻 {s['light']}")
    print(f"偏差：平均 {s['mean_delta']:+.1f}mg｜平均绝对 {s['mean_abs']:.1f}｜中位绝对 {s['median_abs']:.1f}"
          f"｜P90 {s['p90_abs']:.1f}｜最大 {s['max_abs']:.1f}")
    print(f"落在 ±5mg {s['within5']:.0%}、±10mg {s['within10']:.0%}、±20mg {s['within20']:.0%}")
    print(f"用时：平均 {s['mean_dur']:.0f}s，合计 {s['total_dur'] / 60:.1f}min")

    print("\n【按日期】")
    by_day: dict[str, list[dict]] = {}
    for r in rows:
        by_day.setdefault(r["day"], []).append(r)
    for day, rs in sorted(by_day.items()):
        print(fmt(stats(rs), day))

    print("\n【按粉体】")
    by_pow: dict[str, list[dict]] = {}
    for r in rows:
        by_pow.setdefault(r["powder"], []).append(r)
    for name, rs in sorted(by_pow.items(), key=lambda kv: -len(kv[1])):
        print(fmt(stats(rs), name[:12]))

    print("\n【按目标量段】")
    bands = [(0, 300, "<300"), (300, 1000, "300-1000"), (1000, 2000, "1000-2000"),
             (2000, 5000, "2000-5000"), (5000, 10 ** 9, ">5000")]
    for lo, hi, label in bands:
        rs = [r for r in rows if lo <= r["target_mg"] < hi]
        if rs:
            print(fmt(stats(rs), label))

    print("\n【按粉体 × 目标量段（≥5 次启动才列）】")
    for name, rs in sorted(by_pow.items(), key=lambda kv: -len(kv[1])):
        for lo, hi, label in bands:
            sub = [r for r in rs if lo <= r["target_mg"] < hi]
            if len(sub) >= 5:
                print(fmt(stats(sub), f"{name[:6]} {label}"))

    print("\n【终止原因分类（全部启动）】")
    for k, v in Counter(reason_category(r) for r in rows).most_common():
        print(f"  {k:14s} {v:3d}  ({v / len(rows):4.0%})")

    w = stats(rows, wide=True)
    print(f"\n【宽松口径】把「超速熔断后已到 / 沉降余料已到」（提前停在验收区附近）也算跑完：")
    print(f"  完成 {w['n']}/{w['n_start']}（{w['finish_rate']:.1%}）｜合格 {w['ok']}"
          f"｜合格率 {w['ok_rate']:.1%}｜综合成功率 {w['overall_rate']:.1%}"
          f"｜|偏差| 均 {w['mean_abs']:.1f} 中位 {w['median_abs']:.1f} P90 {w['p90_abs']:.1f}mg")

    big = [r for r in rows if abs(r["delta_mg"]) >= 100]
    if big:
        print(f"\n【异常记录：|偏差| ≥ 100mg，共 {len(big)} 次】")
        for r in sorted(big, key=lambda x: -abs(x["delta_mg"]))[:15]:
            print(f"  {r['start']}  {r['powder'][:6]:6s} 目标 {r['target_mg']:6.0f}  "
                  f"实得 {r['achieved_mg']:8.1f}  偏差 {r['delta_mg']:+8.1f}  {r['verdict']}  "
                  f"{r['stop_reason'][:38]}")

    print("\n【未跑完的记录】")
    for r in rows:
        if r["bucket"] != "正常收尾":
            print(f"  {r['start']}  {r['powder'][:6]:6s} 目标 {r['target_mg']:6.0f}  "
                  f"实得 {r['achieved_mg']:7.1f}  {r['bucket']}  {r['stop_reason'][:60]}")

    if args.show:
        print(f"\n最近 {args.show} 次：")
        for r in rows[-args.show:]:
            print(f"  {r['start']}  {r['powder'][:6]:6s} 目标 {r['target_mg']:6.0f} "
                  f"实得 {r['achieved_mg']:8.1f} {r['delta_mg']:+7.1f} {r['verdict']:4s} "
                  f"{r['end_kind']}  速率 {r['rates']}")

    if args.xlsx:
        export_xlsx(rows, Path(args.xlsx), args.since)
        print(f"\n已导出 {args.xlsx}")
    return 0


def export_xlsx(rows: list[dict], out: Path, since: str) -> None:
    import xlsxwriter

    out.parent.mkdir(parents=True, exist_ok=True)
    wb = xlsxwriter.Workbook(str(out), {"nan_inf_to_errors": True})
    f_head = wb.add_format({"bold": True, "font_color": "#FFFFFF", "bg_color": "#1F4E78",
                            "align": "center", "valign": "vcenter", "border": 1, "font_size": 10})
    f_ok = wb.add_format({"bg_color": "#E2EFDA", "border": 1, "font_size": 10})
    f_bad = wb.add_format({"bg_color": "#FCE4E4", "border": 1, "font_size": 10})
    f_txt = wb.add_format({"border": 1, "font_size": 10})
    f_num1 = wb.add_format({"border": 1, "font_size": 10, "num_format": "0.0"})
    f_sig = wb.add_format({"border": 1, "font_size": 10, "num_format": "+0.0;-0.0;0.0"})
    f_pct = wb.add_format({"border": 1, "font_size": 10, "num_format": "0.0%"})
    f_title = wb.add_format({"bold": True, "font_size": 12, "font_color": "#1F4E78"})
    f_key = wb.add_format({"bold": True, "font_size": 10, "bg_color": "#DDEBF7", "border": 1})
    f_note = wb.add_format({"font_size": 9, "font_color": "#666666"})

    sm = wb.add_worksheet("汇总")
    sm.set_column(0, 0, 26)
    sm.set_column(1, 11, 13)
    sm.write(0, 0, f"加粉统计（{since} 起）", f_title)
    sm.write(1, 0, "口径：剔除仿真粉/未选粉；完成率=正常收尾/启动；合格率=合格/正常收尾；"
                   "综合成功率=合格/启动", f_note)

    head = ["分组", "启动次数", "完成(正常收尾)", "完成率", "合格", "合格率",
            "综合成功率", "偏重", "偏轻", "平均偏差mg", "平均|偏差|mg", "P90|偏差|mg", "平均用时s"]
    row = 3
    total = stats(rows)
    sm.write_row(row, 0, head, f_head)
    row += 1

    def put(label: str, st: dict) -> None:
        nonlocal row
        sm.write_string(row, 0, label, f_key)
        sm.write_number(row, 1, st.get("n_start", 0), f_txt)
        sm.write_number(row, 2, st.get("n", 0), f_txt)
        sm.write_number(row, 3, round(st.get("finish_rate", 0), 4), f_pct)
        sm.write_number(row, 4, st.get("ok", 0), f_txt)
        sm.write_number(row, 5, round(st.get("ok_rate", 0), 4), f_pct)
        sm.write_number(row, 6, round(st.get("overall_rate", 0), 4), f_pct)
        sm.write_number(row, 7, st.get("heavy", 0), f_txt)
        sm.write_number(row, 8, st.get("light", 0), f_txt)
        sm.write_number(row, 9, round(st.get("mean_delta", 0), 1), f_num1)
        sm.write_number(row, 10, round(st.get("mean_abs", 0), 1), f_num1)
        sm.write_number(row, 11, round(st.get("p90_abs", 0), 1), f_num1)
        sm.write_number(row, 12, round(st.get("mean_dur", 0), 1), f_num1)
        row += 1

    put("全部（严格口径）", total)
    put("全部（宽松口径：含提前到位停住）", stats(rows, wide=True))
    sm.write_string(row, 0, "其中：保护中止（严格口径未跑完）", f_txt)
    sm.write_number(row, 1, total.get("aborted", 0), f_txt)
    row += 1
    sm.write_string(row, 0, "其中：中途取消", f_txt)
    sm.write_number(row, 1, total.get("cancelled", 0), f_txt)
    row += 2

    def section(title: str, groups: list[tuple[str, list[dict]]]) -> None:
        nonlocal row
        sm.write(row, 0, title, f_title)
        row += 1
        sm.write_row(row, 0, head, f_head)
        row += 1
        for label, rs in groups:
            put(label, stats(rs))
        row += 1

    by_day: dict[str, list[dict]] = {}
    for r in rows:
        by_day.setdefault(r["day"], []).append(r)
    section("按日期", list(sorted(by_day.items())))

    by_pow: dict[str, list[dict]] = {}
    for r in rows:
        by_pow.setdefault(r["powder"], []).append(r)
    section("按粉体", sorted(by_pow.items(), key=lambda kv: -len(kv[1])))

    bands = [(0, 300, "<300"), (300, 1000, "300-1000"), (1000, 2000, "1000-2000"),
             (2000, 5000, "2000-5000"), (5000, 10 ** 9, ">5000")]
    section("按目标量段",
            [(label, [r for r in rows if lo <= r["target_mg"] < hi]) for lo, hi, label in bands])

    combo = []
    for name, rs in sorted(by_pow.items(), key=lambda kv: -len(kv[1])):
        for lo, hi, label in bands:
            sub = [r for r in rs if lo <= r["target_mg"] < hi]
            if len(sub) >= 5:
                combo.append((f"{name} / {label}", sub))
    section("按粉体 × 目标量段（≥5 次启动）", combo)

    # 终止原因分类
    sm.write(row, 0, "终止原因分类（全部启动）", f_title)
    row += 1
    sm.write_row(row, 0, ["原因", "次数", "占比"], f_head)
    row += 1
    for k, v in Counter(reason_category(r) for r in rows).most_common():
        sm.write_string(row, 0, k, f_txt)
        sm.write_number(row, 1, v, f_txt)
        sm.write_number(row, 2, round(v / len(rows), 4), f_pct)
        row += 1
    row += 1

    # 明细
    cols = [("开始时间", 19), ("完成时间", 19), ("粉体", 10), ("目标 mg", 9),
            ("实得 mg", 10), ("偏差 mg", 9), ("验收区", 13), ("判定", 7),
            ("结果", 10), ("用时 s", 8), ("结束方式", 10), ("缓补发数", 8),
            ("快加 s", 7), ("粗加 s", 7), ("慢加 s", 7), ("峰值振幅", 9),
            ("调幅次数", 8), ("三阶段速率", 12), ("停止原因", 60)]
    ws = wb.add_worksheet("加粉明细")
    ws.write_row(0, 0, [c[0] for c in cols], f_head)
    for i, (_, w) in enumerate(cols):
        ws.set_column(i, i, w)
    for i, r in enumerate(rows, start=1):
        done = r["bucket"] == "正常收尾"
        base = (f_ok if r["verdict"] == "合格" else f_bad) if done else f_txt
        ws.write_string(i, 0, r["start"], f_txt)
        ws.write_string(i, 1, r["finished_at"], f_txt)
        ws.write_string(i, 2, r["powder"], f_txt)
        ws.write_number(i, 3, r["target_mg"], f_num1)
        ws.write_number(i, 4, r["achieved_mg"], f_num1)
        ws.write_number(i, 5, r["delta_mg"], f_sig if (done and r["verdict"] == "合格") else base)
        ws.write_string(i, 6, f"{r['accept_low']:.0f}~{r['accept_high']:.0f}", f_txt)
        ws.write_string(i, 7, r["verdict"], base)
        ws.write_string(i, 8, r["bucket"], base)
        ws.write_number(i, 9, r["duration_s"], f_num1)
        ws.write_string(i, 10, r["end_kind"], f_txt)
        if r["trim_pulses"] is None:
            ws.write_string(i, 11, "", f_txt)
        else:
            ws.write_number(i, 11, r["trim_pulses"], f_txt)
        for j, key in enumerate(("fast_s", "coarse_s", "slow_s"), start=12):
            ws.write_number(i, j, r[key] if r[key] is not None else 0, f_num1)
        ws.write_number(i, 15, r["amp_max"], f_num1)
        ws.write_number(i, 16, r["amp_changes"], f_txt)
        ws.write_string(i, 17, r["rates"], f_txt)
        ws.write_string(i, 18, r["stop_reason"], f_txt)
    ws.freeze_panes(1, 3)
    ws.autofilter(0, 0, max(1, len(rows)), len(cols) - 1)

    wb.close()


if __name__ == "__main__":
    raise SystemExit(main())
