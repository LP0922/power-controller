# -*- coding: utf-8 -*-
"""不依赖第三方库，直接把 xlsx 的单元格值 dump 出来（本机没装 openpyxl 时用）。

用法：
    python lab/tools/dump_xlsx.py 文件.xlsx            # 列出所有子表名
    python lab/tools/dump_xlsx.py 文件.xlsx 0 1 40     # 打印第 0 个子表的前 1~40 行
"""
from __future__ import annotations

import re
import sys
import zipfile
import xml.etree.ElementTree as ET

NS = "{http://schemas.openxmlformats.org/spreadsheetml/2006/main}"


def shared_strings(z: zipfile.ZipFile) -> list[str]:
    try:
        root = ET.fromstring(z.read("xl/sharedStrings.xml"))
    except KeyError:
        return []
    out = []
    for si in root.findall(f"{NS}si"):
        out.append("".join(t.text or "" for t in si.iter(f"{NS}t")))
    return out


def sheets(z: zipfile.ZipFile) -> list[tuple[str, str]]:
    """返回 [(表名, xml 路径)]，按工作簿顺序。"""
    wb = ET.fromstring(z.read("xl/workbook.xml"))
    rels = ET.fromstring(z.read("xl/_rels/workbook.xml.rels"))
    rid2tgt = {r.get("Id"): r.get("Target") for r in rels}
    rns = "{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id"
    out = []
    for sh in wb.find(f"{NS}sheets"):
        tgt = rid2tgt.get(sh.get(rns), "")
        if not tgt.startswith("xl/"):
            tgt = "xl/" + tgt.lstrip("/")
        out.append((sh.get("name"), tgt))
    return out


def col_index(ref: str) -> int:
    letters = re.match(r"([A-Z]+)", ref).group(1)
    idx = 0
    for ch in letters:
        idx = idx * 26 + (ord(ch) - 64)
    return idx - 1


def read_sheet(z: zipfile.ZipFile, path: str, ss: list[str]) -> list[list[str | None]]:
    root = ET.fromstring(z.read(path))
    rows: dict[int, dict[int, str]] = {}
    for row in root.iter(f"{NS}row"):
        r = int(row.get("r")) - 1
        for c in row.findall(f"{NS}c"):
            ref = c.get("r") or ""
            ci = col_index(ref) if ref else 0
            t = c.get("t")
            v = c.find(f"{NS}v")
            isel = c.find(f"{NS}is")
            if t == "s" and v is not None:
                val = ss[int(v.text)]
            elif t == "inlineStr" and isel is not None:
                val = "".join(x.text or "" for x in isel.iter(f"{NS}t"))
            elif v is not None:
                val = v.text
            else:
                continue
            rows.setdefault(r, {})[ci] = val
    if not rows:
        return []
    maxr, maxc = max(rows), max(max(d) for d in rows.values())
    grid = [[None] * (maxc + 1) for _ in range(maxr + 1)]
    for r, d in rows.items():
        for c, val in d.items():
            grid[r][c] = val
    return grid


def main() -> int:
    path = sys.argv[1]
    z = zipfile.ZipFile(path)
    ss = shared_strings(z)
    shs = sheets(z)
    if len(sys.argv) < 3:
        for i, (name, p) in enumerate(shs):
            g = read_sheet(z, p, ss)
            print(f"[{i}] {name}  rows={len(g)} cols={max((len(r) for r in g), default=0)}")
        return 0
    idx = int(sys.argv[2])
    lo = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    hi = int(sys.argv[4]) if len(sys.argv) > 4 else 30
    name, p = shs[idx]
    g = read_sheet(z, p, ss)
    print(f"== {name}（{len(g)} 行）")
    for r in range(lo, min(hi, len(g))):
        cells = [("" if v is None else v) for v in g[r]]
        print(f"{r:3d} | " + " | ".join(cells))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
