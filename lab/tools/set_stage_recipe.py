# -*- coding: utf-8 -*-
"""把页面上那套「三阶段配方」写进粉档的 ``stage_recipe_v1`` 段。

为什么单独写个脚本：
  - 粉档是**整份读写**的，不能只改某一段就落盘，否则会抹掉 feedforward_v2 /
    settle_gain_v1 等其它段。这里走「读整份 → 改目标段 → 写 .tmp → replace」；
  - 后端在一次加粉过程中会把档案 raw 留在内存里，**加粉结束时整体写回**。
    如果这期间我们改了文件，那次写回会把它盖掉。所以加 ``--wait-idle``：
    轮询 /api/status，等到 `dispense.running=False` 且 `calib.running=False`
    再落盘，避免改动静默丢失。

用法：
    python lab/tools/set_stage_recipe.py --powder 失水剂2 --file recipe.json
    python lab/tools/set_stage_recipe.py --powder 失水剂2 --file recipe.json --wait-idle

    # 直接读页面当前这套配置（从正在跑/刚跑完的 dispense config 里抄），不落盘，只看
    python lab/tools/set_stage_recipe.py --powder 失水剂2 --from-live
"""
from __future__ import annotations

import argparse
import json
import time
import urllib.request
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
LIB_DIR = PROJECT_ROOT / "server" / "data" / "powder_library"
LIB_ALIAS = PROJECT_ROOT / "server" / "data" / "powder_library_alias"
API = "http://127.0.0.1:8765/api"


def lib_dir() -> Path:
    try:
        if LIB_ALIAS.is_dir() and any(LIB_ALIAS.glob("*.json")):
            return LIB_ALIAS
    except OSError:
        pass
    return LIB_DIR


def status() -> dict:
    with urllib.request.urlopen(f"{API}/status", timeout=5) as r:
        return json.load(r)


def busy() -> tuple[bool, str]:
    s = status()
    d = (s.get("dispense") or {})
    c = (s.get("calib") or {})
    if d.get("running"):
        return True, f"加粉中：{d.get('detail')}"
    if c.get("running"):
        return True, f"标定中：{c.get('detail')}"
    return False, "空闲"


def wait_idle(max_s: int = 900) -> None:
    t0 = time.time()
    while True:
        b, why = busy()
        if not b:
            print(f"服务空闲，开始落盘（等待 {time.time() - t0:.0f}s）")
            return
        if time.time() - t0 > max_s:
            raise SystemExit(f"等待超时（{max_s}s），仍在：{why}")
        print(f"  …{why}，20s 后重试")
        time.sleep(20)


def recipe_from_live() -> dict:
    """从后端当前 dispense config 里抄出页面这套三段工作点（只读，不改任何东西）。"""
    s = status()
    cfg = ((s.get("dispense") or {}).get("config") or {})
    overrides = cfg.get("stage_overrides") or []
    if not overrides:
        raise SystemExit("后端没有正在用/刚用过的三段配置（stage_overrides 为空）")
    stages = []
    text = {"fast": "快加", "coarse": "粗加", "slow": "慢加"}
    for o in overrides:
        stages.append({
            "stage": o["name"],
            "stage_text": text.get(o["name"], o["name"]),
            "enabled": True,
            "stir_rpm": o.get("stir_rpm"),
            "rod_units": o.get("rod_units"),
            "amplitude_vpp": o.get("amplitude_vpp"),
            "rate_mg_s": o.get("rate_mg_s"),
            "duty_percent": o.get("duty_percent"),
            "switch_mass_mg": o.get("switch_mass_mg"),
            "settle_wait_s": None,
        })
    return {
        "signal": {"frequency_hz": cfg.get("frequency_hz"),
                   "duty_percent": cfg.get("duty_percent")},
        "target_mg": cfg.get("target_mg"),
        "stages": stages,
    }


def write_recipe(powder: str, recipe: dict) -> Path:
    d = lib_dir()
    path = d / f"{powder}.json"
    if not path.exists():
        raise SystemExit(f"找不到粉档：{path}")
    raw = json.loads(path.read_text(encoding="utf-8"))
    raw["stage_recipe_v1"] = recipe
    raw["updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(raw, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)
    return path


def guard(powder: str, recipe: dict, max_s: int) -> None:
    """盯一段时间：每次加粉/标定结束后的整体写回可能有几秒延迟，
    会把我们刚写的配方盖回内存里的旧 raw。这里在每次「忙→空闲」后多等几秒，
    比对配方段，缺了/旧了就重写一遍。
    """
    d = lib_dir()
    path = d / f"{powder}.json"

    def current() -> dict:
        try:
            return json.loads(path.read_text(encoding="utf-8")).get("stage_recipe_v1") or {}
        except (OSError, json.JSONDecodeError):
            return {}

    def same(a: dict) -> bool:
        ka = [(s.get("stage"), s.get("stir_rpm"), s.get("rate_mg_s"),
               s.get("amplitude_vpp"), s.get("switch_mass_mg"))
              for s in a.get("stages") or []]
        kb = [(s.get("stage"), s.get("stir_rpm"), s.get("rate_mg_s"),
               s.get("amplitude_vpp"), s.get("switch_mass_mg"))
              for s in recipe.get("stages") or []]
        return ka == kb

    t0 = time.time()
    was_busy = busy()[0]
    fixed = 0
    print(f"守护 {max_s}s：结束后若被覆盖会自动重写")
    while time.time() - t0 < max_s:
        b, why = busy()
        if was_busy and not b:
            time.sleep(6)          # 给运行器留出整体落盘的时间
            cur = current()
            if not same(cur):
                write_recipe(powder, recipe)
                fixed += 1
                print(f"  发现被覆盖（{time.strftime('%H:%M:%S')}），已重写（第 {fixed} 次）")
            else:
                print(f"  加粉结束后配方完好（{time.strftime('%H:%M:%S')}）")
        was_busy = b
        time.sleep(5)
    cur = current()
    print(f"守护结束：当前配方{'与目标一致' if same(cur) else '仍不一致！'}，重写 {fixed} 次")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--powder", required=True)
    ap.add_argument("--file", default=None, help="配方 json（只含 stages/signal/target 的部分字段，其余原样保留）")
    ap.add_argument("--from-live", action="store_true", help="从后端当前三段配置抄")
    ap.add_argument("--wait-idle", action="store_true", help="等加粉/标定都停下再落盘")
    ap.add_argument("--guard", type=int, default=0, metavar="SECONDS",
                    help="落盘后继续盯这么多秒，被运行器写回覆盖就自动重写")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    d = lib_dir()
    path = d / f"{args.powder}.json"
    if not path.exists():
        raise SystemExit(f"找不到粉档：{path}")
    raw = json.loads(path.read_text(encoding="utf-8"))
    old = raw.get("stage_recipe_v1") or {}

    if args.from_live:
        live = recipe_from_live()
        # 保留旧档案里的描述性字段（source/notes/estimated_s 等），只换三段与信号
        new = dict(old)
        new.update({k: v for k, v in live.items() if v is not None})
    else:
        if not args.file:
            raise SystemExit("要么给 --file，要么用 --from-live")
        patch = json.loads(Path(args.file).read_text(encoding="utf-8"))
        new = dict(old)
        new.update(patch)

    new.setdefault("schema", 1)
    new["updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")

    print(f"粉档：{path}")
    for key in ("stir_rpm", "rod_units", "amplitude_vpp", "rate_mg_s", "duty_percent", "switch_mass_mg"):
        before = [s.get(key) for s in old.get("stages") or []]
        after = [s.get(key) for s in new.get("stages") or []]
        flag = "  ← 变" if before != after else ""
        print(f"  {key:14s} {before} → {after}{flag}")

    if args.dry_run:
        print("（dry-run，未落盘）")
        return 0

    if args.wait_idle:
        wait_idle()
    else:
        b, why = busy()
        if b:
            print(f"⚠ 服务{why}——这次落盘可能在加粉结束时被内存 raw 覆盖，建议加 --wait-idle")

    write_recipe(args.powder, new)
    print("已写入 stage_recipe_v1")

    # 回读校验
    back = json.loads(path.read_text(encoding="utf-8")).get("stage_recipe_v1") or {}
    ok = [s.get("stir_rpm") for s in back.get("stages") or []] == \
         [s.get("stir_rpm") for s in new.get("stages") or []]
    print(f"回读校验：{'OK' if ok else '不一致！'}")

    if args.guard:
        guard(args.powder, new, args.guard)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
