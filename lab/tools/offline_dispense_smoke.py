#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""闭环加粉离线冒烟：假 hub + 虚拟时钟，把 DispenseRunner 整条线程跑完。

用途：改完 dispense_runner.py 后不用上真机，就能确认"起振后会不会炸"。
2026-10-23 的 `float >= NoneType` 就是靠这个脚本在离线复现并验证修好的——
真机上必须等 hist 攒够 8 个采样点（约 1 秒）才走到出问题那行，所以现象永远是
"点开始、2 秒后异常停止"，光看日志很难对上代码。

用法：
    python lab/tools/offline_dispense_smoke.py                # 默认 200mg，跑完看结果
    python lab/tools/offline_dispense_smoke.py --target 1000
    python lab/tools/offline_dispense_smoke.py --keep-csv     # 保留逐拍 CSV 便于复盘

安全：全程不碰任何串口，hub 是纯 Python 假的，机构不会动。
"""
from __future__ import annotations

import argparse
import sys
import tempfile
import threading
from pathlib import Path

SRC = Path(__file__).resolve().parents[2] / "server" / "src"
sys.path.insert(0, str(SRC))

import powder_sampling_control.control_panel.dispense_runner as dr  # noqa: E402


class FakeClock:
    """虚拟时钟：sleep 只是把时间往前推，整炉几十秒的仿真瞬间跑完。"""

    def __init__(self) -> None:
        self.now = 1000.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, dt: float) -> None:
        self.now += max(0.0, float(dt))


class FakeEvent(threading.Event):
    """停止事件替身：wait(timeout) 只把虚拟时钟推走，不在真实时间里等。

    真线程里 DispenseRunner 用 ``self._stop_event.wait(0.05)`` 当节拍器（缓补脉冲、
    等落定都用它）。如果只用假时钟、事件还是真的 threading.Event，虚拟时间就不会
    前进，收尾那几个 while 会永远转下去——所以这里两个一起替。
    """

    def __init__(self, clock: FakeClock) -> None:
        super().__init__()
        self._clock = clock

    def wait(self, timeout=None):
        if timeout:
            self._clock.now += float(timeout)
        return super().wait(0)


class FakeSignal:
    def __init__(self) -> None:
        self.amp = 2.0
        self.out = False
        self.duty = 50.0
        self.freq = 80.0
        self.waveform = None
        self.burst_off_calls = 0

    def burst_off(self) -> None:
        self.burst_off_calls += 1

    def set_waveform(self, *a, **k) -> None:
        self.waveform = a

    def set_frequency(self, v) -> None:
        self.freq = float(v)

    def set_duty(self, v) -> None:
        self.duty = float(v)

    def set_amplitude(self, v) -> None:
        self.amp = float(v)
        self._marks.append((self._clock.now, self.amp))

    # 猝发（真脉冲）：设备自己数个数，数完停；仿真里等价于"又振了 n/f 秒"
    def burst_setup(self, n) -> None:
        self.burst_n = int(n)

    def burst_arm(self) -> None:
        pass

    def burst_fire(self, n) -> dict:
        seconds = float(n) / max(1.0, self.freq)
        self._hub._integrate_burst(seconds)   # 猝发期间照样落粉
        return {"estimated_s": seconds}

    _marks: list
    _clock: FakeClock
    burst_n: int = 0


class FakeHub:
    """假机构：天平按"振幅 → 速率"的物理模型累加质量，带 1.5s 纯滞后。

    ``STIR_LEAK_MG_S_PER_100RPM``：振动关着、搅拌还在转时的"漏料"（mg/s @100RPM）。
    这是 2026-10-23 11:09 那炉实测出来的现象——润滑剂1 在杆 680/搅 100 下，
    振动一关粉照样以 ≈5mg/s 塌下来（腔里堆的料被搅拌推出去），缓补"关振等落定"
    永远等不到稳。默认 0 = 关振就断料（封堵剂1 那种），要复现润滑剂1 就传 0.05。
    """

    TAU_S = 1.5
    DEAD_AMP = 1.6       # 低于这个振幅一粒不出（模拟起振门槛）
    # mg/s per V。**标定口径（2026-10-23 11:56 用户实测）**：稳定 3.0V 绝对有 20mg/s
    # ——(3.0−1.6)×14.3≈20，所以取 14.0。旧值 6.0 在 3.0V 只给 8.4mg/s，是假模型自己
    # 太瘦，曾让我误判"标定 2.5V 的粉追不到 20mg/s"，用户已纠正。
    GAIN = 14.0
    STIR_LEAK_MG_S_PER_100RPM = 0.0
    STIR_COAST_S = 10.0  # 搅拌停转后余料还会继续落多久（线性衰减到 0）
    STIR_SPINUP_S = 0.0  # 起搅拌后多久才真正把腔口粉层推到落料口（0=不模拟）
    # 第几发缓补脉冲"放量"（模拟腔里塌料：1200mg 那炉 3.10V 先 +12mg 再 +38mg）
    TRIM_SURGE_AT = 0
    TRIM_SURGE_MULT = 1.0
    # ★ 起振塌料（2026-10-26 11:30 现场）：料口上方堆着的粉在**第一次开振动**的
    # 0.5s 内一次性塌下来。11:30:25 那炉 0→110mg 只用 0.5s（207mg/s），缓补一发
    # 都没发出去就 279mg。`--start-surge 110` 复现它。
    START_SURGE_MG = 0.0
    SURGE_WINDOW_S = 0.5
    # 塌料之后料道里的存量还在往下走：**振着才走**，停振后只残余 STOP_RESIDUAL_S 秒
    # （现场 11:30:23 停振后还落 +102mg，就是这 1 秒内的存量；停久了就没了）。
    AVALANCHE_RATIO = 0.35
    AVALANCHE_S = 2.5
    STOP_RESIDUAL_S = 0.8

    def __init__(self, clock: FakeClock, baseline: int = 100000,
                 dead_amp: float | None = None,
                 stir_spinup_s: float | None = None,
                 gain: float | None = None,
                 trim_surge: tuple[int, float] | None = None,
                 start_surge_mg: float | None = None) -> None:
        self.clock = clock
        self.baseline = baseline
        if dead_amp is not None:
            self.DEAD_AMP = float(dead_amp)
        if stir_spinup_s is not None:
            self.STIR_SPINUP_S = float(stir_spinup_s)
        if gain is not None:
            self.GAIN = float(gain)
        if trim_surge is not None:
            self.TRIM_SURGE_AT, self.TRIM_SURGE_MULT = int(trim_surge[0]), float(trim_surge[1])
        if start_surge_mg is not None:
            self.START_SURGE_MG = float(start_surge_mg)
        self._surge_t0: float | None = None
        self._sig_on_at: float | None = None
        self._sig_off_at: float | None = None
        self._residual_done = False
        self._sig_state = False
        self.pulse_n = 0
        self.leak = self.STIR_LEAK_MG_S_PER_100RPM
        self._stir_stop_t: float | None = None
        self._stir_start_t: float | None = None
        self.sig = FakeSignal()
        self.sig._clock = clock
        self.sig._marks = [(clock.now, 2.0)]
        self.sig._hub = self
        self.mass = 0.0
        self._last_t = clock.now
        self.rod = 600
        self.stir = 0.0
        self.max_amp_seen = 0.0
        self.amp_writes = 0

    def _flow_ready(self, now: float) -> bool:
        """搅拌刚起转的前 STIR_SPINUP_S 秒里，腔口的粉还没被推到位 → 振动也白振。

        复现 2026-10-23 11:23 那炉：0.15~0.22s 的猝发永远 +0mg，0.6s 的能出 5~6mg。
        """
        if self.STIR_SPINUP_S <= 0.0:
            return True
        if self.stir <= 0.0 or self._stir_start_t is None:
            return True
        return (now - self._stir_start_t) >= self.STIR_SPINUP_S - 1e-3

    # ---- 天平 ----
    def _eff_amp(self) -> float:
        """1.5s 前那一刻的振幅（纯滞后）。"""
        target_t = self.clock.now - self.TAU_S
        amp = self.sig._marks[0][1]
        for t, a in self.sig._marks:
            if t <= target_t:
                amp = a
        return amp

    def _integrate(self) -> None:
        now = self.clock.now
        dt = max(0.0, now - self._last_t)
        self._last_t = now
        if dt <= 0:
            return
        # 起振塌料：第一次开振动后的 SURGE_WINDOW_S 内额外落 START_SURGE_MG；
        # 之后 AVALANCHE_S 秒内继续按峰值的一部分落（与振动开关无关 = 料道里的存量）
        if self.START_SURGE_MG > 0.0 and self.sig.out:
            if self._surge_t0 is None:
                self._surge_t0 = now
        if bool(self.sig.out) != bool(self._sig_state):
            self._sig_state = bool(self.sig.out)
            if self._sig_state:
                self._sig_on_at = now
            else:
                self._sig_off_at = now      # 只在"真的关了"那一刻记一次
        if self._surge_t0 is not None:
            age = now - self._surge_t0
            peak = self.START_SURGE_MG / self.SURGE_WINDOW_S
            if age <= self.SURGE_WINDOW_S:
                self.mass += peak * dt
            elif self.sig.out and age <= self.SURGE_WINDOW_S + self.AVALANCHE_S:
                self.mass += peak * self.AVALANCHE_RATIO * dt
            # 停振后残余只算一次（每发脉冲都算一次会把总量吹到天上）
            elif (not self.sig.out and self._sig_off_at is not None
                    and now - self._sig_off_at <= self.STOP_RESIDUAL_S
                    and not self._residual_done):
                self.mass += peak * self.AVALANCHE_RATIO * dt
                if now - self._sig_off_at >= self.STOP_RESIDUAL_S - 0.15:
                    self._residual_done = True
        if self.sig.out:
            if not self._flow_ready(now):
                rate = 0.0                       # 搅拌还没转到位，振了也不出
            else:
                rate = max(0.0, (self._eff_amp() - self.DEAD_AMP) * self.GAIN)
        elif self.leak <= 0.0:
            return
        elif self.stir > 0.0:
            rate = self.leak * self.stir / 100.0     # 振动关、搅拌转：塌料
        elif self._stir_stop_t is not None:
            # 搅拌停转后的惯性余料：STIR_COAST_S 内线性衰减到 0。
            # 10-23 那炉判停 170mg 后还在落（收尾 233），就是这部分没被算进去。
            left = self.STIR_COAST_S - (now - self._stir_stop_t)
            if left <= 0:
                return
            rate = self.leak * (left / self.STIR_COAST_S)
        else:
            return
        self.mass += rate * dt

    def bal_refresh(self) -> dict:
        self._integrate()
        # ±1 count 的读数噪声，让回归不至于完美（真实天平就是这样）
        jitter = 1 if int(self.clock.now * 7) % 2 else -1
        return {"raw": int(round(self.baseline + self.mass)) + jitter, "stable": True}

    def _integrate_burst(self, seconds: float) -> None:
        """猝发（设备自己数的真脉冲）期间的质量累加。

        这里**不用 1.5s 纯滞后**：猝发只有 0.15~0.6s，比滞后还短，用滞后振幅
        会得出"这一发一定不出粉"（真机实测 4V/0.6s 是能出 5~6mg 的）。脉冲是
        "当下这一下振"的事件，滞后体现在**称重读数**上（脉冲后等 2~3s 才读），
        不体现在质量累加里。
        """
        self._integrate()
        self.pulse_n += 1
        mult = (self.TRIM_SURGE_MULT
                if (self.TRIM_SURGE_AT and self.pulse_n == self.TRIM_SURGE_AT) else 1.0)
        if self._flow_ready(self.clock.now):
            rate = max(0.0, (float(self.sig.amp) - self.DEAD_AMP) * self.GAIN) * mult
        else:
            rate = 0.0
        self.clock.now += seconds
        self.mass += rate * seconds
        self._last_t = self.clock.now

    # ---- 机构 ----
    def assert_idle_for_dispense(self) -> dict:
        return {"rod_position": self.rod, "lift_positions": [500, 500]}

    def rod_move(self, pos) -> None:
        self.rod = int(pos)

    def rod_refresh(self) -> dict:
        return {"pos": self.rod}

    def stir_start(self, rpm, **k) -> None:
        if self.stir <= 0.0:
            self._stir_start_t = self.clock.now    # 从冷态起转：后面要爬升
        self.stir = float(rpm)
        self._stir_stop_t = None

    def stir_stop(self, mode="decel", acc=None) -> None:
        self._integrate()                    # 先把转着的那段流量结清
        self.stir = 0.0
        self._stir_stop_t = self.clock.now

    def sig_output(self, on) -> None:
        self._integrate()
        self.sig.out = bool(on)

    def _sig_call(self, action) -> None:
        before = self.sig.amp
        action()
        if self.sig.amp != before:
            self.amp_writes += 1
            self.max_amp_seen = max(self.max_amp_seen, self.sig.amp)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", type=float, default=200.0)
    ap.add_argument("--powder", default=None, help="粉体名（留空 = 纯闭环，不读粉体库）")
    ap.add_argument("--amp", type=float, default=2.5)
    ap.add_argument("--stir-leak", type=float, default=0.0,
                    help="振动关、搅拌转时的漏料，单位 mg/s @100RPM（润滑剂1 实测约 5，传 5）")
    ap.add_argument("--stir-rpm", type=float, default=100.0, help="阶段搅拌转速（漏料模型要用）")
    ap.add_argument("--dead-amp", type=float, default=None,
                    help="起振门槛 Vpp（润滑剂1 实测 3.5 左右；默认 1.6 即一振就出）")
    ap.add_argument("--stir-spinup", type=float, default=None,
                    help="搅拌从冷态起转到真正推料需要的秒数（模拟 0.15s 短脉冲发不出的现象）")
    ap.add_argument("--gain", type=float, default=None,
                    help="假机构增益 mg/s per V（默认 14.0 = 稳定 3.0V 约 20mg/s，"
                         "2026-10-23 11:56 用户实测口径）")
    ap.add_argument("--trim-surge", default=None, metavar="N:MULT",
                    help="第 N 发缓补脉冲放量 MULT 倍（模拟腔里塌料：1200mg 那炉 "
                         "3.10V 先 +12mg 再 +38mg），例：--trim-surge 3:3")
    ap.add_argument("--start-surge", type=float, default=None, metavar="MG",
                    help="起振 0.5s 内一次性塌落 MG 粉（复现 11:30 那几次 200mg 超重）")
    ap.add_argument("--surge-ratio", type=float, default=None,
                    help="段内超速熔断倍率（0 = 关，用来看关掉会怎样）")
    ap.add_argument("--keep-csv", action="store_true", help="保留逐拍 CSV")
    ap.add_argument("--show-log", action="store_true", help="打印整炉日志（排查判停/缓补用）")
    ap.add_argument("--timeout", type=float, default=120.0)
    args = ap.parse_args()

    clock = FakeClock()
    real_time_ref = dr.time
    dr.time = clock                      # 只改本模块看到的 time，不污染全局

    tmpdir = Path(tempfile.mkdtemp(prefix="offline_dispense_"))
    original_run_dir = dr.RUN_DIR
    dr.RUN_DIR = tmpdir

    lines: list[str] = []

    def log(msg, *a, **k) -> None:
        lines.append(str(msg))

    surge = None
    if args.trim_surge:
        at, _, mult = args.trim_surge.partition(":")
        surge = (int(at), float(mult or 3.0))
    hub = FakeHub(clock, dead_amp=args.dead_amp, stir_spinup_s=args.stir_spinup,
                  gain=args.gain, trim_surge=surge,
                  start_surge_mg=args.start_surge)
    hub.leak = float(args.stir_leak)
    runner = dr.DispenseRunner(hub, log)
    runner._stop_event = FakeEvent(clock)     # 换成会把虚拟时钟推走的停止事件
    err = None
    try:
        runner.start(args.target, powder=args.powder, amplitude_vpp=args.amp,
                     stages=[{"name": "coarse", "rate_mg_s": 20.0,
                              "stir_rpm": args.stir_rpm},
                             {"name": "slow", "stir_rpm": args.stir_rpm}],
                     trim_enabled=True, surge_ratio=args.surge_ratio)
        runner._thread.join(timeout=args.timeout)
        if runner._thread.is_alive():
            runner.stop("离线冒烟超时")
            runner._thread.join(timeout=5.0)
            err = "线程未在超时内结束"
    except Exception as exc:             # noqa: BLE001
        err = f"{type(exc).__name__}: {exc}"
    finally:
        dr.time = real_time_ref
        dr.RUN_DIR = original_run_dir

    crash = [ln for ln in lines if "异常终止" in ln or "Traceback" in ln
             or "Error" in ln or "Traceback (most recent call last)" in ln]
    result = runner.status()

    print(f"虚拟用时 {clock.now - 1000.0:.1f}s ｜ 实得 {hub.mass:.1f} mg "
          f"/ 目标 {args.target:.0f} mg ｜ 状态 {result.get('state')}")
    print(f"振幅写入次数 {hub.amp_writes}（峰值 {hub.max_amp_seen:.2f}V）｜ "
          f"退猝发 {hub.sig.burst_off_calls} 次 ｜ 搅拌末值 {hub.stir:.0f} rpm")
    print(f"日志 {len(lines)} 条，异常相关 {len(crash)} 条")
    for ln in crash:
        print("  ! " + ln.strip().splitlines()[0])
    if err:
        print("  ! 脚本级错误：" + err)

    ok = not crash and err is None
    print("结果：", "通过 ✅" if ok else "失败 ❌")
    if args.keep_csv:
        print("逐拍 CSV：" + str(tmpdir))
    if args.show_log:
        print("\n完整日志：")
        for ln in lines:
            print("  " + ln)

    if not ok:
        print("\n完整日志尾部：")
        for ln in lines[-30:]:
            print("  " + ln)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
