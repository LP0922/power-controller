# -*- coding: utf-8 -*-
"""对比「上一版硬件」与「本版硬件」的加粉效果，只出数据。

数据来源：
  上一版：outputs/refs/old_hw.xlsx（2026-09-23~24 试验记录，126 条）
  本  版：control_panel/runs/dispense_*.json（10-21~23；剔除仿真粉、(未选粉)、人为取消）

两个口径（2026-10-23 16:53 用户定：**本版只看正常判停 + 缓补收尾**）：
  上一版：备注以 predicted tail / measured mass reached 开头（88 条）
  本  版：正常判停 + 缓补收尾（82 条）
两边都是程序自己判断收得住、自己收的加粉次数；保护性中止与被取消的都不进对比。

验收判据两边一致：|偏差| ≤ 10 mg（product_requirements.json 的 ±10，旧表也是这个口径）。

用法：
    python lab/tools/compare_hardware_versions.py                    # 打印
    python lab/tools/compare_hardware_versions.py --xlsx out.xlsx    # 另存对比表
"""
from __future__ import annotations

import argparse
import statistics as st
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lab" / "tools"))

from collect_dispense_history import load_rows              # noqa: E402
from dump_xlsx import read_sheet, shared_strings, sheets     # noqa: E402

OLD_XLSX = ROOT / "outputs" / "refs" / "old_hw.xlsx"
TOL_MG = 10.0
OLD_NORMAL_PREFIX = ("predicted tail", "measured mass")

POWDER_MAP = {
    "润滑剂1": "润滑剂1",
    "封堵剂1": "封堵剂1",
    "封堵剂2": "封堵剂2",
    "膨润土": "膨润土 (Bentonite)",
    "失水剂1": "失水剂1",
    "抑制剂1": "抑制剂",
    "失水剂2": "失水剂2（本版无此粉）",
    "大分子包被剂-text": "大分子包被剂（本版无此粉）",
}


def load_old() -> list[dict]:
    z = zipfile.ZipFile(OLD_XLSX)
    grid = read_sheet(z, sheets(z)[0][1], shared_strings(z))
    out = []
    for r in grid[2:]:
        if not r or r[4] is None:
            continue
        note = r[12] or ""
        out.append({
            "start": "", "powder": r[4], "powder_key": POWDER_MAP.get(r[4], r[4]),
            "target": float(r[7]), "final": float(r[8]), "delta": float(r[9]),
            "dur": float(r[11]), "note": note,
            "normal": note.startswith(OLD_NORMAL_PREFIX),
        })
    return out


def load_new() -> list[dict]:
    normal, abn, _c, _e = load_rows()
    def row(r, is_normal):
        return {"start": r["start"], "powder": r["powder"], "powder_key": r["powder"],
                "target": r["target_mg"], "final": r["achieved_mg"], "delta": r["delta_mg"],
                "dur": r["duration_s"], "note": r["stop_reason"], "normal": is_normal}
    return [row(r, True) for r in normal] + [row(r, False) for r in abn]


def stat(rows: list[dict]) -> dict:
    if not rows:
        return {}
    d = [x["delta"] for x in rows]
    a = [abs(x) for x in d]
    t = [x["dur"] for x in rows]
    srt = sorted(a)
    p90 = srt[max(0, int(round(0.9 * (len(srt) - 1))))]
    return {
        "n": len(rows),
        "pass": sum(1 for x in a if x <= TOL_MG),
        "rate": sum(1 for x in a if x <= TOL_MG) / len(a),
        "mean": sum(d) / len(d),
        "mae": sum(a) / len(a),
        "med": st.median(a),
        "p90": p90,
        "max": max(a),
        "w30": sum(1 for x in a if x <= 30) / len(a),
        "dur": sum(t) / len(t),
        "spg": sum(x["dur"] / x["target"] * 1000 for x in rows) / len(rows),
    }


def bucket(rows: list[dict]) -> dict:
    a = [abs(x["delta"]) for x in rows]
    return {"n": len(a), "≤10": sum(1 for x in a if x <= 10),
            "10–20": sum(1 for x in a if 10 < x <= 20),
            "20–30": sum(1 for x in a if 20 < x <= 30),
            "30–100": sum(1 for x in a if 30 < x <= 100),
            ">100": sum(1 for x in a if x > 100)}


BANDS = [("<300", 0, 300), ("300–1000", 300, 1000), ("1000–2000", 1000, 2000),
         ("2000–5000", 2000, 5000), (">5000", 5000, 10 ** 9)]
TARGETS = [200, 500, 1000, 2000, 3000, 5000, 10000]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--xlsx", default=None)
    args = ap.parse_args()

    old, new = load_old(), load_new()
    old_n = [x for x in old if x["normal"]]
    new_n = [x for x in new if x["normal"]]

    def line(tag: str, a: dict, b: dict) -> None:
        print(f"{tag:14s} 旧 n={a['n']:3d} 通过{a['rate']:6.1%} 平均{a['mean']:+6.1f} "
              f"MAE{a['mae']:6.1f} 中位{a['med']:5.1f} P90{a['p90']:5.1f} 最大{a['max']:6.1f} "
              f"耗时{a['dur']:6.1f}s  |  新 n={b['n']:3d} 通过{b['rate']:6.1%} 平均{b['mean']:+6.1f} "
              f"MAE{b['mae']:6.1f} 中位{b['med']:5.1f} P90{b['p90']:5.1f} 最大{b['max']:6.1f} "
              f"耗时{b['dur']:6.1f}s")

    print("== 对比口径（只算正常收尾）")
    line("正常收尾", stat(old_n), stat(new_n))

    print("\n== 同目标量段（正常收尾）")
    for t in TARGETS:
        a = [x for x in old_n if x["target"] == t]
        b = [x for x in new_n if x["target"] == t]
        if not a and not b:
            continue
        sa, sb = stat(a), stat(b)
        fmt = lambda s, k, w, p="": (f"{s[k]:{w}{p}}" if s else "-".rjust(w))
        print(f"  {t:6d}mg 旧 n={fmt(sa,'n',3)} 通过{fmt(sa,'rate',7,'.0%')} "
              f"中位{fmt(sa,'med',5,'.1f')} MAE{fmt(sa,'mae',6,'.1f')} 耗时{fmt(sa,'dur',6,'.1f')} || "
              f"新 n={fmt(sb,'n',3)} 通过{fmt(sb,'rate',7,'.0%')} "
              f"中位{fmt(sb,'med',5,'.1f')} MAE{fmt(sb,'mae',6,'.1f')} 耗时{fmt(sb,'dur',6,'.1f')}")

    print("\n== 分粉体（正常收尾）")
    keys = []
    for x in old_n + new_n:
        if x["powder_key"] not in keys:
            keys.append(x["powder_key"])
    for k in keys:
        a = [x for x in old_n if x["powder_key"] == k]
        b = [x for x in new_n if x["powder_key"] == k]
        sa, sb = stat(a), stat(b)
        fmt = lambda s, kk, w, p="": (f"{s[kk]:{w}{p}}" if s else "-".rjust(w))
        print(f"  {k:24s} 旧 n={fmt(sa,'n',3)} {fmt(sa,'rate',7,'.0%')} 中位{fmt(sa,'med',5,'.1f')} "
              f"MAE{fmt(sa,'mae',6,'.1f')} || 新 n={fmt(sb,'n',3)} {fmt(sb,'rate',7,'.0%')} "
              f"中位{fmt(sb,'med',5,'.1f')} MAE{fmt(sb,'mae',6,'.1f')}")

    if args.xlsx:
        export(Path(args.xlsx), old, new, old_n, new_n)
        print(f"\n已导出 {args.xlsx}")
    return 0


def export(out: Path, old: list[dict], new: list[dict],
           old_n: list[dict], new_n: list[dict]) -> None:
    import xlsxwriter

    wb = xlsxwriter.Workbook(str(out), {"nan_inf_to_errors": True})
    H = wb.add_format({"bold": True, "font_color": "#FFFFFF", "bg_color": "#1F4E78",
                       "align": "center", "valign": "vcenter", "border": 1, "font_size": 10})
    T = wb.add_format({"border": 1, "font_size": 10})
    N1 = wb.add_format({"border": 1, "font_size": 10, "num_format": "0.0"})
    SG = wb.add_format({"border": 1, "font_size": 10, "num_format": "+0.0;-0.0;0.0"})
    PC = wb.add_format({"border": 1, "font_size": 10, "num_format": "0.0%"})
    KEY = wb.add_format({"bold": True, "font_size": 10, "bg_color": "#DDEBF7", "border": 1})
    UP = wb.add_format({"border": 1, "font_size": 10, "bg_color": "#E2EFDA", "num_format": "0.0"})
    DN = wb.add_format({"border": 1, "font_size": 10, "bg_color": "#FCE4E4", "num_format": "0.0"})
    TITLE = wb.add_format({"bold": True, "font_size": 12, "font_color": "#1F4E78"})
    NOTE = wb.add_format({"font_size": 9, "font_color": "#666666"})

    # ---------------- 对比总览 ----------------
    ws = wb.add_worksheet("对比总览")
    ws.set_column(0, 0, 22)
    ws.set_column(1, 2, 15)
    ws.set_column(3, 3, 13)
    ws.set_column(4, 4, 46)
    ws.write(0, 0, "上一版硬件 vs 本版硬件：加粉效果对比（只统计数据）", TITLE)
    ws.write(1, 0, "验收判据两边相同：|偏差| ≤ 10 mg", NOTE)
    ws.write(2, 0, "上一版 = 2026-09-23~24 试验记录；本版 = 2026-10-21~23 记录"
                   "（已剔除仿真粉、(未选粉)、人为取消）", NOTE)

    def block(row: int, title: str, a: list[dict], b: list[dict]) -> int:
        ws.write(row, 0, title, TITLE)
        ws.write_row(row + 1, 0, ["指标", "上一版硬件", "本版硬件", "变化", "说明"], H)
        sa, sb = stat(a), stat(b)
        items = [
            ("记录数", sa["n"], sb["n"], "int", "参与统计的加粉次数"),
            ("±10mg 通过数", sa["pass"], sb["pass"], "int", "|偏差| ≤ 10 mg 计为通过"),
            ("±10mg 通过率", sa["rate"], sb["rate"], "%", "通过数 ÷ 记录数"),
            ("平均偏差 mg", sa["mean"], sb["mean"], "sg", "带符号，正=整体偏重"),
            ("平均绝对偏差 mg", sa["mae"], sb["mae"], "1", "越小越准"),
            ("中位绝对偏差 mg", sa["med"], sb["med"], "1", "抗离群值"),
            ("P90 绝对偏差 mg", sa["p90"], sb["p90"], "1", "90% 的加粉次数在此以内"),
            ("最大绝对偏差 mg", sa["max"], sb["max"], "1", "最差一次"),
            ("|Δ|≤30mg 占比", sa["w30"], sb["w30"], "%", "粗合格率"),
            ("平均耗时 s", sa["dur"], sb["dur"], "1", "从启动到判停/收尾"),
            ("单位质量耗时 s/1000mg", sa["spg"], sb["spg"], "1", "受目标量构成影响，仅参考"),
        ]
        for i, (name, va, vb, kind, memo) in enumerate(items, start=row + 2):
            ws.write_string(i, 0, name, KEY)
            if kind == "%":
                ws.write_number(i, 1, va, PC)
                ws.write_number(i, 2, vb, PC)
                ws.write_string(i, 3, f"{(vb - va) * 100:+.1f} pt", T)
            elif kind == "sg":
                ws.write_number(i, 1, va, SG)
                ws.write_number(i, 2, vb, SG)
                ws.write_string(i, 3, f"{vb - va:+.1f}", T)
            elif kind == "1":
                ws.write_number(i, 1, va, N1)
                ws.write_number(i, 2, vb, N1)
                good = vb < va if name != "平均耗时 s" else vb < va
                ws.write_number(i, 3, vb - va, UP if good else DN)
            else:
                ws.write_number(i, 1, int(va), T)
                ws.write_number(i, 2, int(vb), T)
                ws.write_string(i, 3, f"{int(vb) - int(va):+d}", T)
            ws.write_string(i, 4, memo, NOTE)
        return row + 3 + len(items)

    nxt = block(4, "口径：只看程序自己正常收起的加粉次数（本版=正常判停 + 缓补收尾；"
                   "上一版=predicted tail + measured mass）", old_n, new_n)

    # ---------------- 分目标量段 ----------------
    ws2 = wb.add_worksheet("分目标量段")
    cols = ["目标量段", "旧 n", "旧 通过率", "旧 中位|Δ|", "旧 平均|Δ|", "旧 平均耗时 s",
            "新 n", "新 通过率", "新 中位|Δ|", "新 平均|Δ|", "新 平均耗时 s"]
    ws2.write_row(0, 0, cols, H)
    for i, w in enumerate([14, 7, 11, 11, 11, 13, 7, 11, 11, 11, 13]):
        ws2.set_column(i, i, w)
    r = 1
    for label, lo, hi in BANDS:
        a = [x for x in old_n if lo <= x["target"] < hi]
        b = [x for x in new_n if lo <= x["target"] < hi]
        sa, sb = stat(a), stat(b)
        ws2.write_string(r, 0, label, KEY)
        for base, s in ((1, sa), (6, sb)):
            if not s:
                for c in range(base, base + 5):
                    ws2.write_string(r, c, "-", T)
                continue
            ws2.write_number(r, base, s["n"], T)
            ws2.write_number(r, base + 1, s["rate"], PC)
            ws2.write_number(r, base + 2, s["med"], N1)
            ws2.write_number(r, base + 3, s["mae"], N1)
            ws2.write_number(r, base + 4, s["dur"], N1)
        r += 1

    # 关键目标量逐点对齐
    r += 1
    ws2.write(r, 0, "同目标量逐点对齐（正常收尾）", TITLE)
    r += 1
    cols2 = ["目标 mg", "旧 n", "旧 通过率", "旧 中位|Δ|", "旧 平均|Δ|", "旧 平均耗时 s",
             "新 n", "新 通过率", "新 中位|Δ|", "新 平均|Δ|", "新 平均耗时 s"]
    ws2.write_row(r, 0, cols2, H)
    r += 1
    for t in TARGETS:
        a = [x for x in old_n if x["target"] == t]
        b = [x for x in new_n if x["target"] == t]
        if not a and not b:
            continue
        sa, sb = stat(a), stat(b)
        ws2.write_number(r, 0, t, KEY)
        for base, s in ((1, sa), (6, sb)):
            if not s:
                for c in range(base, base + 5):
                    ws2.write_string(r, c, "-", T)
                continue
            ws2.write_number(r, base, s["n"], T)
            ws2.write_number(r, base + 1, s["rate"], PC)
            ws2.write_number(r, base + 2, s["med"], N1)
            ws2.write_number(r, base + 3, s["mae"], N1)
            ws2.write_number(r, base + 4, s["dur"], N1)
        r += 1

    # ---------------- 分粉体 ----------------
    ws3 = wb.add_worksheet("分粉体")
    ws3.write_row(0, 0, ["粉体", "旧 n", "旧 通过率", "旧 中位|Δ|", "旧 平均|Δ|",
                         "新 n", "新 通过率", "新 中位|Δ|", "新 平均|Δ|"], H)
    for i, w in enumerate([26, 8, 11, 11, 11, 8, 11, 11, 11]):
        ws3.set_column(i, i, w)
    ws3.write(1, 0, "口径：只算正常收尾；两版共有的粉才对得上", NOTE)
    keys = []
    for x in old_n + new_n:
        if x["powder_key"] not in keys:
            keys.append(x["powder_key"])
    r = 2
    for k in keys:
        a = [x for x in old_n if x["powder_key"] == k]
        b = [x for x in new_n if x["powder_key"] == k]
        sa, sb = stat(a), stat(b)
        ws3.write_string(r, 0, k, KEY)
        for base, s in ((1, sa), (5, sb)):
            if not s:
                for c in range(base, base + 4):
                    ws3.write_string(r, c, "-", T)
                continue
            ws3.write_number(r, base, s["n"], T)
            ws3.write_number(r, base + 1, s["rate"], PC)
            ws3.write_number(r, base + 2, s["med"], N1)
            ws3.write_number(r, base + 3, s["mae"], N1)
        r += 1

    # ---------------- 偏差分布 ----------------
    ws4 = wb.add_worksheet("偏差分布")
    cols4 = ["口径", "版本", "n", "≤10mg", "10–20mg", "20–30mg", "30–100mg", ">100mg"]
    ws4.write_row(0, 0, cols4, H)
    for i, w in enumerate([14, 10, 8, 10, 10, 10, 11, 10]):
        ws4.set_column(i, i, w)
    r = 1
    for name, a, b in (("正常收尾", old_n, new_n),):
        for tag, rows in (("上一版", a), ("本版", b)):
            bk = bucket(rows)
            ws4.write_string(r, 0, name, T)
            ws4.write_string(r, 1, tag, KEY)
            ws4.write_number(r, 2, bk["n"], T)
            for j, k in enumerate(("≤10", "10–20", "20–30", "30–100", ">100"), start=3):
                ws4.write_number(r, j, bk[k], T)
            r += 1
        for tag, rows in (("上一版 占比", a), ("本版 占比", b)):
            bk = bucket(rows)
            ws4.write_string(r, 0, name, T)
            ws4.write_string(r, 1, tag, KEY)
            ws4.write_number(r, 2, bk["n"], T)
            for j, k in enumerate(("≤10", "10–20", "20–30", "30–100", ">100"), start=3):
                ws4.write_number(r, j, bk[k] / bk["n"], PC)
            r += 1
        r += 1

    # ---------------- 说明 ----------------
    ws5 = wb.add_worksheet("说明")
    ws5.set_column(0, 0, 120)
    for i, t in enumerate([
        "口径说明",
        "1. 验收判据：|偏差| = |最终质量 − 目标质量| ≤ 10 mg，两版完全相同（product_requirements.json 的 ±10，旧表也是这个口径）。",
        "2. 上一版数据：outputs/refs/old_hw.xlsx「试验记录」126 条（2026-09-23 至 09-24，目标 200–10000 mg，8 种粉）。",
        "3. 本版数据：control_panel/runs/dispense_*.json 中 10-21 至 10-23 的记录，"
        "只取结束方式为「正常判停」51 条 + 「缓补收尾」38 条，再剔除仿真粉、(未选粉) 共 7 条，"
        "实际参与对比 82 条；保护性中止与人为取消都不计入。",
        "4. 上一版同口径：备注以 predicted tail（73 条）+ measured mass reached（15 条）开头，共 88 条；"
        "其 force stop 15 条、projected safety stop 23 条属保护性中止，不进对比。",
        "5. 两边判据、粉种、目标量都对齐后再比，因此表中每一行都是同条件对比。",
        "6. 中位/P90 比平均值抗离群值：本版 10-21 18:17 那次加粉封堵剂2 5000mg 天平读到 38344 mg 触发超重保护，"
        "记录质量 1183 mg、偏差 −3817 mg，属保护性中止，已不在对比口径内。",
        "7. 单位质量耗时 s/1000mg 受目标量构成影响（旧版 62/126 条是 5000 mg，本版多是小目标），"
        "横向比耗时请看「同目标量逐点对齐」。",
        "8. 样本量提示：本版 5000 mg 只有 7 条正常收尾、10000 mg 只有 1 条，这些量段只能看趋势，不能当结论。",
    ], start=0):
        ws5.write(i, 0, t, TITLE if i == 0 else T)

    wb.close()


if __name__ == "__main__":
    raise SystemExit(main())
