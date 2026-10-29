"""闭环加粉执行器：把 dispensing_algorithm 的纯算法接到当前 ActuatorHub 上。

拓扑映射（2026-10-21 现场拓扑）
------------------------------
    FeedAction.frequency_hz          → QLS3600S 频率（COM16，振动料斗）
    FeedAction.duty_permyriad        → QLS3600S 方波占空比（%，permyriad/100）
    FeedAction.window_position_units → 因时 LA 推杆位置（COM10，出料口开度）
    「搅拌转速」（本模块新增）        → ZDT Emm42（COM9）
    天平 mass_mg                     → AT8811C（COM11），1 count = 1 mg

算法层 ``dispensing_algorithm/core.py`` **一行不改**：它是纯函数式的
"观测进、决策出"，没有串口、没有设备命令、没有状态机副作用。
本模块是唯一的"决策 → 设备指令"翻译层，也是唯一知道"当前硬件长什么样"的地方。

三段式加粉（为什么必须分阶段）
------------------------------
同一个执行器不可能既快又准：流量大时速率波动大，流量小时又容易搭桥断流。
所以按"离目标还有多远"分三段，每段用**不同的目标速率与振幅**：

| 阶段 | 目标速率 | 推杆 | 振幅 | 控制方式 |
|---|---|---|---|---|
| 快加 fast | 最高（追速度，不管稳不稳） | 标定过的大开度，**整段锁死** | **标定值 ±0.2V 内微调** | 振幅小步跟，最多改 2 次 |
| 粗加 coarse | 中（既跟得上又不太毛躁） | 同上，**整段锁死** | **标定值 ±0.2V 内微调** | 同上 |
| 慢加 slow | 低（宁可慢，要稳） | 同上，**整段锁死** | **标定值，整段一动不动** | 不调振幅，追不上就按实际速率收 |

**速率只由振幅调，但振幅的基准是标定值**（2026-10-22 用户当面定）：
标定表里每个点的速率都是在那个粉的标定振幅下测的（封堵剂1 = 2Vpp），
离开这个振幅，速率表就不成立——所以快加/粗加只能在 ±0.2V 内小步微调，
**慢加段一个信号都不许写**。推杆只在阶段入口开到该段配方开度，之后整段不再动；
占空比按该段配方固定下发，也不参与闭环。这样调节量单一、可解释，
也不会出现"推杆和占空比互相追着改"的抖动。

收尾只做三件事：**停振 → 搅拌减速 → 等尾料沉降称重落盘**。
推杆停在最终开度**不复位**（用户 2026-10-21 要求）：复位会晃秤、也徒增一次机械动作，
下一次加粉由阶段入口自行开到配方开度。

阶段切换点（2026-10-21 用户定的规则，`stage_boundaries()` 是唯一实现处）：

    target > 5000 mg   快加 0→(目标−800)，粗加 →(目标−100)，慢加收尾（大目标不按百分比切）
    target > 1000 mg   快加 0→75%，粗加 75%→95%（5% 超过 100mg 时改为 到 剩余 100mg），慢加收尾
    target ≤ 1000 mg   不启用快加，粗加 0→(目标−预留)，慢加收尾
    target < 300 mg    粗加只到 目标−40mg，保证慢加段至少还有 40mg 可调

三个阶段的**搅拌转速 / 推杆开度 / 目标速率 / 占空比 / 切换质量**都可以在页面上
按自己标定过的点逐段填写；不填就按标定表自动分配（最快的点给快加、中位给粗加、
最慢的点给慢加）。

前馈从哪来
----------
``powder_feedforward`` 里的标定表：目标速率 + 搅拌档 → 反解推杆开度。
没有标定的粉也能跑（退回预设种子、搅拌保持 0），但每段入口就没有"该开多少"的依据，
只能靠闭环从零试错——所以页面会明确标出"未标定"。

安全边界（本模块承诺）
----------------------
1. 启动前必须确认 搅拌已停 / 升降静止 / 推杆静止 / 振动输出关闭；
2. 运行中天平连续 5 次读不到（约 0.6s）→ 停振动、停搅拌、标记失败；
3. 质量超过 目标+30mg → 立即停（不依赖控制器的预测停车）；
4. 最长运行 180s，超时强制停；
5. 任何异常/停止都保证先关振动（振动是最危险的那个执行器），再停搅拌；
6. 振幅永远钳在 1–4 Vpp，且快加/粗加只在标定值 ±0.2V 内动、慢加不动。
"""

from __future__ import annotations

import csv
import json
import math
import threading
import time
import traceback
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from powder_sampling_control.dispensing_algorithm import (
    DISPENSE_PRESETS,
    ContinuousFeedbackController,
    FeedbackControllerSettings,
    FeedbackInitialParameters,
    FeedbackObservation,
    MassMotionEstimator,
    acceptance_limits,
    interpolate_profile,
    powder_classification_params,
    resolve_partial_target_overrides,
    validate_target_mass,
)
from powder_sampling_control.control_panel.powder_feedforward import (
    ROD_MAX_UNITS,
    ROD_MIN_UNITS,
    PowderWorkspace,
    load_powder,
)

RUN_DIR = Path(__file__).resolve().parent / "runs"

# 三个阶段，顺序就是执行顺序。note：这是"段"的名字，控制器内部分支另有 mode
STAGE_ORDER = ("fast", "coarse", "slow")
STAGE_TEXT = {"fast": "快加", "coarse": "粗加", "slow": "慢加", "settle": "收尾判停",
              "trim": "缓补", "gate": "沉降等待", "surge": "超速熔断"}
# 段名 → 控制器分支（dispensing_algorithm 里只有 coarse/fine/precision 三档脾气）
STAGE_MODE = {"fast": "coarse", "coarse": "fine", "slow": "precision"}
STAGE_NOTE = {
    "fast": "振幅按速率闭环跟（基准 3.0V，±0.5V 内 0.05–0.15V 小步调）",
    "coarse": "振幅按速率闭环跟（基准 2.5V，±0.5V 内小步调）",
    "slow": "振幅锁 2.0V 只做恢复出料，不追速率（5mg/s 追不上就按实际速率收）",
}
# 三阶段默认目标速率（2026-10-22 用户定）：快加 75 / 粗加 20 / 慢加 5 mg/s。
# 页面留空就是这三个数，标定表按它们反解推杆开度（这张表就是"速率→开度"的查找表）。
DEFAULT_STAGE_RATE_MG_S = {"fast": 75.0, "coarse": 20.0, "slow": 5.0}
# 三阶段默认振幅（2026-10-23 18:36 用户定）：快加 3.0 / 粗加 2.5 / 慢加 2.0 Vpp。
# 页面三行「振幅 Vpp」留空就是这三个数（页面也会直接把这三个值填进输入框，手填即覆盖）。
# 这三个数同时是**调幅基准**：本段的闭环区间 = 本段振幅 −0.5V ~ +0.5V（见 _amp_band_for），
# 所以快加在 2.5–3.5V 里追流量、粗加在 2.0–3.0V、慢加在 1.5–2.5V。
# 与粉体标定振幅的关系：标定表只提供"速率→转速/开度"的工作点，振幅按这里的分段给。
STAGE_DEFAULT_AMP_VPP = {"fast": 3.0, "coarse": 2.5, "slow": 2.0}
# 按速率选标定点时，CV>40%（塌料/天平跳数）的点要额外加多少距离惩罚。
# 0.7 ≈ 相当于"速率再差一倍"——好点够用时永远不会选到脏点。
STAGE_POINT_CV_PENALTY = 0.7
# 页面显示用的目标范围；真正的校验在 dispensing_algorithm.validate_target_mass
TARGET_MIN_MG = 100
TARGET_MAX_MG = 20000

# ── 振幅调节（2026-10-22 用户改口径：基准是标定值，慢加一律不动）─────────────
# 2026-10-27 11:08 用户："1V 下限删掉，最低下限 0.01V"。
# 原来卡 1.0V 是软件自己定的，设备（QLS3600S）本身就能从 0.01V 起（驱动
# `QLS3600Config.min_amplitude_vpp = 0.01`）。快加/粗加各有自己的带下界
# （base−0.5 = 2.5 / 2.0），所以放开这里只影响慢加和异常降幅路径。
AMP_MIN_VPP = 0.01
AMP_MAX_VPP = 4.0
# 振动参数：**按粉记住**（2026-10-21 用户定，第三次修正）——
# 每种粉的标定就是它自己的工作点，加粉时必须换回它当初标定时用的频率/占空比：
# 22% 标出来的速率曲线如果按 50% 下发，整条曲线都会偏快。
# 下面两个常数只在"没选粉 / 该粉还没标定 / 标定段里没存信号"时兜底。
VIB_FREQUENCY_HZ = 80.0
VIB_DUTY_PERCENT = 50.0
# ── 振幅调节铁律（2026-10-22，用户 11:20 再次纠正后重写）──────────────────────
#  1. **慢加段的振幅一动不动**（用户原话"我只要求你慢加不许乱改振幅"）。这条最硬：
#     慢加只求慢和稳，追 5 mg/s 追不上就认，绝不靠加振幅硬顶。
#  2. **快加/粗加的振幅必须真的会动**。上一版把边界硬卡在标定值 ±0.2V（2.0→2.2）
#     又限定每段只改 2 次，结果粗加段实测速率掉到 0.9 mg/s、振幅还死死钉在 2.20V
#     —— 用户当场质问"速率都为 0 了你的振幅还是 2.2 一直不变，你在干嘛"。
#     断料和"偏慢"是两件事：**断料必须加振幅**，只是加的路径要干净。
#  3. 所以振幅分两个档位走：
#     - **微调带**（标定值 ±AMP_BAND_V）：有流量、只是比目标快一点/慢一点的场合，
#       只允许在这个带里小步修，绝不为了追一个标定表里不存在的流量把振幅顶飞
#       （那是 10:57 那次 2.0→3.01V 的错误）。
#     - **断料救济**（标定值 +AMP_ESCALATE_V 之内）：整整一个响应窗（纯滞后 +
#       落料周期）**一粒粉都没落**，这是断料/搭桥，不是"偏慢"，允许越过微调带
#       按 0.2V/步往上探，硬顶 = 标定值 + 0.6V。到顶还是不出，就停手并在日志里
#       提示"该查料斗了"——再往上加没有标定依据（3.2V 那种就是没依据的）。
#  4. **粉末有纯滞后，且滞后/落料周期是在线估计的（v6）**：写完振幅，粉要走完
#     "料槽→落料口→秤盘"，落料本身还是脉冲式。v6 不再用固定 8s+10s，而是：
#     每次写振幅记一个时间戳，之后第一次 ≥15mg 的落料阶跃出现 → τ_est = 阶跃时刻−写入
#     时刻（EMA）；相邻两次落料阶跃的间隔 → T_est（EMA）。判据窗口永远取
#     [上次写入+τ_est, 现在] 且长度 ≥ T_est，窗口不够就明确"数据无效，不动"。
#  5. **趋势闸（消除"效果还在路上就重复加幅"）**：比较相邻两个生效窗的速率——
#     速率正在明显回升（Δr > 目标×15%）说明上一发的效果还没出完，**先等**，
#     不叠加新的动作；速率持平或下降才按误差决定动不动。
#  6. **起振门槛实测（2026-10-22 12:38 真机脉冲测试，杆 680 开度、方波 80Hz）**：
#     2Vpp 0.6s×3发+3s长发 = 一粒不出（0mg）；3Vpp 0.6s = +37mg；3.5Vpp 0.6s = +23/+20mg；
#     3Vpp 0.3s = +5mg。**结论：粉位低时标定振幅可能整段低于起振门槛**，门槛在 2~3V 之间，
#     3V 以上短脉冲就能出料。所以断料救济的上探上限从 标定+0.6V 放宽到 标定+1.5V。
#     （旧版还有"落了粉就把该档记为起振门槛、微调带以 max(标定, 门槛−0.2V) 为中心"——
#     这套门槛学习 2026-10-27 11:44 已被用户整体否掉：门槛只由页面填的值决定，踢脚
#     回来就回到踢前的振幅，不做任何上移。）
#  7. **慢加也允许卡粉救济**（用户 12:38："粗加和慢加的时候都会有卡粉的情况"）：
#     慢加的"速率微调"依旧禁止（追不上就认），但**真卡粉**（一个生效窗 0 落料且无阶跃）
#     允许走同一条救济通道——卡粉不是"追速率"，是"恢复出料"。
#  8. **踢一脚（2026-10-22 13:31 用户定，取代 0.5V/步逐级上探）**：卡粉救济不再
#     一步步磨，而是模仿脉冲——瞬间抬到 标定+1.0V（2→3V）持续 0.5s 后立刻回落
#     标定值。12:38 实测 3V 短脉冲 +37mg、2V 一粒不出：一脚正好跨过起振门槛，
#     高幅只停 0.5s 也不会像连续抬幅那样把速率冲飞。一脚没踢开（回落标定值后
#     又攒满一个生效窗仍 0 落料）→ 下一脚 +0.5V，硬顶仍是 标定+1.5V。
AMP_BAND_V = 0.20            # 微调带：标定值 ±0.2V（有流量时只在这带里修）
AMP_ESCALATE_V = 1.50        # 断料救济硬顶 = 标定值 +1.5V（2.0 → 3.5V；
                             # 12:38 实测 3~3.5V 短脉冲可出料，2.6V 在门槛之下）
AMP_TAU_INIT_S = 8.0         # τ_est 初值：纯滞后（料槽→落料口→秤盘）
AMP_PERIOD_INIT_S = 10.0     # T_est 初值：落料周期（实测 4~8s，取保守值）
AMP_TAU_EMA = 0.3            # τ/T 在线估计的 EMA 系数（新样本占三成）
AMP_DROP_EVENT_MG = 15.0     # 单次采样增重 ≥ 此值 = 一次"落料阶跃"（判落料事件）
AMP_KICK_DELTA_V = 1.00      # 踢一脚：振幅瞬间抬到 标定+1.0V（2.0→3.0V；12:38 实测
                             # 3V 短脉冲 +37mg、2V 一粒不出——一脚跨过起振门槛）
AMP_KICK_HOLD_S = 0.50       # 高幅持续 0.5s 后立刻回落标定值（用户 13:31 定）
AMP_KICK_ESCALATE_V = 0.50   # 一脚没踢开，下一脚在此基础上再 +0.5V（顶=标定+1.5V）
# 到顶之后怎么办（2026-10-27 16:25 修 bug）：以前"振幅不能再往上"就判"踢粉到顶、
# 永久停手"，可当时往往只踢过**一脚**（16:19 那次加粉润滑剂1：anchor 已被速率环抬到带顶
# 2.50V，第一脚 2.50+1.00=3.50V 正好等于硬顶 3.50V，第二脚算出来还是 3.50V ≤ 上一脚
# → 直接锁死，全程真正踢到的只有 1 脚，而那一脚其实踢出了 9mg）。
# 现在的口径：硬顶只是"振幅不再加"，**在硬顶振幅上可以反复踢**，用"连续空踢脚数"判停。
AMP_KICK_MAX_TRIES = 8       # 本段踢脚总脚数上限（含到顶后的重复踢），超了也停手
AMP_KICK_MIN_TRIES_BEFORE_STOP = 4   # 到顶后至少踢这么多脚，才允许用"累计没出料"判停
AMP_KICK_MIN_TOTAL_GAIN_MG = 5.0     # 这几脚**累计**增重低于此值 = 真没料
                                     # （必须用累计：单拍质量差只有天平噪声那么大，
                                     #   一脚本来也只出 4~8mg，按单拍判会永远"踢出了料"）
AMP_STALL_GAIN_FLOOR = 0.8   # 增益学习：k_est 低于此值(mg/s per V) = 升幅无效，不再升
AMP_ACCEL_GATE_RATIO = 0.15  # 趋势闸：速率回升超过目标×此值 → 上一发还在生效，先等
AMP_STALL_RATE_RATIO = 0.30  # 实测速率低于目标的这个比例 → 判为断料（不是"偏慢"）
AMP_FLOOD_RATE_RATIO = 1.35  # 实测速率高于目标的这个比例 → 往下收，避免冲过头

# ── 粗调坡道·丝滑版（2026-10-22 18:09 用户定调）──────────────────────────────
# 快加/粗加段振幅 = 标定值 ±AMP_RAMP_BAND_V 的带内**丝滑**调整，不跳变、不冲击：
#   速率偏慢但有出粉（>0.5mg/s）→ 每 4s 抬 +0.08V（"速度小了就先加到 2.58，慢慢变"）
#   速率为 0（≤0.5mg/s）      → 每 3s 抬 +0.12V（"速度为 0 就加到 2.62"，步子稍大）
# **有出粉就绝不发脉冲冲击**（用户原话：出粉量长时间不为 0 就不需要发脉冲信号来
# 冲击粉末）。脉冲冲击（踢一脚）只留给最后手段：坡道已爬到带顶仍断料。
# 慢加不适用（慢加振幅锁死是更高级别的用户铁律）。
AMP_RAMP_BAND_V = 0.50       # 调幅带：标定 ±0.5V（2.5V 启动 → 2.0–3.0V，用户举例）
AMP_RAMP_STEP_ZERO_V = 0.12  # 零落料爬坡：每档 +0.12V
AMP_RAMP_INTERVAL_ZERO_S = 3.0   # 零落料爬坡 3s 一档

# ── v7 自适应调幅（2026-10-23 09:52，**已被下面的 v8 取代**，常量保留仅供对照）──
# 平时调节 = 自适应小步：按"前 5s 平均速率 + 加速度"外推纯滞后后的速率走向，
# 缺口 ÷ 增益(k_est) 换算步长，钳在 0.05–0.15V——丝滑不跳变。
# 为什么被取代：步长上限 0.15V + 必须写幅后 4s 才评估，一个 12mg/s 的缺口要磨
# 七八十秒，而粗加段往往撑不到那么久（400mg 那次加粉 71s 全程 2.50V、平均 3.95mg/s）。
AMP_EVAL_INTERVAL_S = 2.0     # （v7 legacy）
AMP_LOOKBACK_S = 5.0          # （v7 legacy）
AMP_REEVAL_MIN_S = 4.0        # （v7 legacy）
AMP_STEP_MIN_V = 0.05         # （v7 legacy）
AMP_STEP_MAX_V = 0.15         # （v7 legacy）
AMP_DEFAULT_K = 12.0          # （v7 legacy；v8 用 AMP_CTRL_K_DEFAULT）
AMP_ACCEL_MAX = 5.0           # 加速度估计限幅 mg/s²（防噪声把预测吹飞）
AMP_KICK_GATE_WINDOW_S = 4.0  # 踢脚 gate：到顶后观察窗 4s
AMP_KICK_GATE_RATE_MG_S = 1.0 # 踢脚 gate：4s 内速率 <1 mg/s 才算"长时间无速率"
AMP_SLOW_CLIMB_STEP_V = 0.10  # 慢加零落料爬坡每档 +0.10V（涓流）
AMP_SLOW_CLIMB_INTERVAL_S = 2.0
AMP_SLOW_DOWN_MAX_V = 0.30    # 慢加**降幅**单步上限（2026-10-27 11:02 修 bug 新增）
                              # 慢加段的任务就是把速率压到 5mg/s，2.0V 实测 19~24mg/s
                              # （超 4 倍）时必须快速往下收；0.15V/步太慢，慢加段
                              # 只有 5~15s，还没降到位就切段了。降幅不会超调：
                              # 低于起振门槛最坏是断料，踢一脚能救；反过来冲过头是报废。
AMP_SLOW_FLOOR_VPP = AMP_MIN_VPP   # 慢加下界 = 设备绝对下限 0.01V（2026-10-27 11:08 用户
                                   # "1V 下限删掉，最低下限 0.01V"；原先 基准−1.0V=1.0）。
                                   # 只有慢加需要往低走：目标才 5mg/s，2.0V 常有 15~25mg/s。
                                   # 上限仍按 基准+0.5V 不放。
AMP_SLOW_EVAL_S = 1.5         # 慢加评估间隔（默认 3.0s 太长：慢加段本身只有 5~15s，
                              # 3s 一评估全程只能走 1~2 步，等于没调）。

# ── 速率闭环（2026-10-23 11:33 立，11:50 用户收回上限/步长口径）───────────────
# 用户原话："前面粗加阶段长时间 6-12mg/s，但是你也不提升幅度……你就一直改动幅度
# 就行了，少了你就加，速度快了你就减"。
# 推杆锁死、占空比按段固定，振幅是唯一能改流量的执行器。
#
# 算法 = **增益归一化的增量式 PI（速度式 PI）**：
#     e      = 目标速率 − 预测速率      （预测 = 6s 回归均速 + 加速度 × min(τ,3s)）
#     Δu     = e / k_est                 （k_est = 在线学到的 mg/s per V）
#     u_next = clamp(u + Δu, 标定−0.5V, 标定+0.5V)，并对 Δu 钳 0.05–0.15V
#   1. 振幅→速率是单调静态增益，但 k 随粉位/粉种/转速变化，事先不可知。e/k 让缺口
#      大时步子顶格、缺口小时自动变细，一次一步收敛，不用人工调步长。
#   2. 每次小步累加 = 自带积分，稳态误差自动归零；死区挡住天平 ±1mg 噪声引起的抖。
#   3. 单步统一 0.05–0.15V：8s 纯滞后里每 3s 评估一次，步长大就容易叠成冲料。
#   4. 超调制动：这次误差方向与上次相反（说明上一步跨多了）→ 本步减半，抑制振荡。
AMP_CTRL_LOOKBACK_S = 6.0      # 测速回看窗
AMP_CTRL_MIN_EVAL_S = 3.0      # 两次调幅的最小间隔（写幅后至少等这么久）
AMP_CTRL_BIG_STEP_WAIT_S = 5.0 # （保留）大步延后评估；单步已钳 0.15V，实际不走这条
AMP_CTRL_DEADBAND_RATIO = 0.08 # 死区 = max(1.5, 目标×8%)
AMP_CTRL_DEADBAND_ABS = 1.5
AMP_CTRL_K_DEFAULT = 10.0      # 增益缺省 mg/s per V（k_est 学到之前用）
AMP_CTRL_K_MIN = 1.0
AMP_CTRL_K_MAX = 80.0
# 上限与步长（2026-10-23 11:50 用户改回 v7 口径）：
#   上限 = **标定 +0.5V**（标定 2.5V → 顶 3.00V），不再放到 标定+2.5V/5.0V；
#   单步 = **0.05–0.15V**（升降同），不再有 0.50V 升 / 0.60V 降的大步。
# 理由：振幅必须贴着标定工况走，离标定越远，速率表的参考意义越小（标定 2V 下的
# 速率反推 5V 行为没有依据），所以宁可慢一点、不越带。
AMP_CTRL_STEP_MIN_V = 0.05     # 单步下限：缺口÷增益算出来比这还小也走 0.05V，别原地磨
AMP_CTRL_UP_SMALL_V = 0.15     # 升幅上限（缺口 < 目标 15% 的细调）
AMP_CTRL_UP_MAX_V = 0.15       # 升幅上限（缺口再大也只走 0.15V/步）
AMP_CTRL_DOWN_MAX_V = 0.15     # 降幅上限（偏快也一步一步收，单步统一 0.05–0.15V）
AMP_CTRL_RAISE_SPAN_V = 0.50   # 自动升幅跨距 = 标定 +0.5V（标定 2.5V → 顶 3.00V）
AMP_CTRL_CEIL_VPP = 5.0        # 设备层安全兜底（20Vpp 量程），实际被 RAISE_SPAN 卡在 标定+0.5V
AMP_CTRL_ZERO_STEP_V = 0.15    # 近 6s 一粒粉没落（断料/搭桥）时的爬坡步长（同样受 0.15V 限制）
AMP_CTRL_ZERO_WAIT_S = 3.5     # 断料爬坡间隔

# ── 进段起步振幅：上一段甩太快就先压下来（2026-10-26 18:04 现场）──────────────
# 那次加粉（2000mg 封堵剂2）的账：快加 22.2s 跑到 1455mg（65.58 mg/s，目标 49.29），
# 进粗加后 **3.0s 冲到 1831mg = 125.88 mg/s（目标 20 的 6.3 倍）**，振幅环的
# 响应窗（AMP_CTRL_MIN_EVAL_S=3.0s）整个走完都还没到评估点，一次没调幅。
# 接着切段沉降余料 +127mg 把慢加顶到 1958（只剩 42mg 空间），慢加 0.9s 只出 20mg，
# 进缓补时腔里存料崩落 +89mg → 0 发脉冲、超验收 48mg、最终 2083（+83mg）。
# 物理原因不是"粗加加太快"，是**进段把上一段的料流惯性原样带过来了**：推杆 525 没动，
# 管腔里全是料，只把搅拌 100→50rpm、振幅 2.57→2.50V 根本压不住存量倾泻。
# 规矩：上一段实测速率 ≥ 本段目标 × 这个倍率时，起步振幅按 k_est 往下压，
# **压到本段振幅带下界为止**（再往下就空振不出料，由踢脚去救）。
STAGE_ENTRY_RATE_RATIO = 3.0


# 每段的判据参数（慢加不在这里——慢加只走卡粉救济，不走速率微调）
#   deadband / deadband_abs  速率误差死区：相对目标速率 + 绝对下限，两个取大
#   step                     微调步长 V（有流量时）
#   min_gain_mg              窗口内增重低于它 = 这窗里没落粉（判据：断料）
AMP_RATE_CTRL = {
    "fast":   {"deadband": 0.20, "deadband_abs": 3.0, "step": 0.10, "min_gain_mg": 30.0},
    "coarse": {"deadband": 0.20, "deadband_abs": 2.0, "step": 0.10, "min_gain_mg": 15.0},
}
# 目标速率钳制：标定表里该工作点（转速+开度）的实测速率乘这个余量就是"机器能力"。
# 超过能力的目标一律按能力执行——快加目标 75、而 200rpm/700 标定只有 49.1。
AMP_RATE_CAP_RATIO = 1.15
# 慢加段的"动态速率目标"时间常数 s：目标速率 = 剩余量 / TAU。
# 注意这是"目标"，**不是"必须靠振幅追到的量"**：追不上就按实际速率收，
# 慢加段振幅不许为此上抬（用户 2026-10-22 明确）。
AMP_SLOW_TAU_S = 18.0
# 慢加段动态目标的上界倍数：最多允许到标定靶子的 1.6 倍（别为了赶时间把慢加变成粗加）
AMP_SLOW_CMD_CAP_RATIO = 1.6
# 慢加段的**连续振幅上限** = 标定/页面振幅 + 0.5V（2.0 → 最高 2.5V）。
# 用户 2026-10-22 17:29 定："慢加的振幅最多 2V 左右、不超 2.5V 调节，不能一下变 3V"。
# 只有卡粉救济的踢一脚（0.5s 脉冲）允许短暂越过——踢完立刻回落标定值。
AMP_SLOW_RAISE_V = 0.50
# 振幅过程日志的心跳间隔 s（用户要求：要能看出"这段时间到底在干嘛"）
AMP_HEARTBEAT_S = 5.0
# 振幅"暂不调整"的原因日志限流 s（数据不足这类每轮都会发生，不刷屏）
AMP_SKIP_LOG_S = 15.0
# 断料兜底（2026-10-26 11:17 事故的延伸）：振幅已经到顶、踢脚也踢到顶仍不出料之后，
# 继续空转毫无意义——旧版会一直挂到 `_max_run_s`（大目标时是十几分钟）。现在只要
# 「已饱和 + 连续 30s 净重没涨 2mg」就主动中止并说明原因，操作员不用盯着一个不动的
# 面板猜"是不是死了"。
STALL_ABORT_S = 30.0
STALL_ABORT_GAIN_MG = 2.0

# ── 段内超速熔断（2026-10-26 11:33 用户："有时候 200mg 加粉会出现一短时间内加的速率
# 非常快的情况……最近几次没触发脉冲而且还都超重"）────────────────────────────
# 现场数据（runs/dispense_20261026_11*.json，全部 200mg/抑制剂）：
#   11:30:13 → +26mg（0→99mg 只用 2.4s=41mg/s；缓补 0 发）
#   11:30:25 → +79mg（0→110mg 只用 0.5s=207mg/s；缓补 0 发）
#   11:30:36 → +52mg（0→118mg 只用 0.5s=216mg/s；缓补 0 发）
#   11:13:51 → +21mg（160→212mg 只用 1.1s=48mg/s；缓补 0 发）
# 共同点：**料口上方堆着的粉一次性塌下来**，0.5~1s 就落 60~130mg。这时候
#   ① 6s 回归均速算不出来（窗还没攒满），振幅环根本来不及反应；
#   ② 断料判停/预测切段用的提前量是常数（1s），在 200mg/s 面前等于没提前；
#   ③ 质量一步跨过"剩 40mg 进缓补"的门槛，缓补一发都发不出去就超重了。
# 所以加一层**只看瞬时速率**的熔断：短窗（0.7s）速率一旦超过本段规定速率的
# N 倍，或者"剩余量已经不够这一瞬间的落料"，立刻停振等余料落完，再重新起振。
# 这一层不参与稳态调幅，只在异常时接管，正常加粉完全不会触发。
SURGE_GUARD_ON = True
SURGE_RATE_RATIO = 2.5          # 段内速率门槛 = 本段规定速率 × 2.5（粗加 20→50mg/s）
SURGE_ABS_MIN_MG_S = 12.0       # 门槛绝对下限（慢加 5mg/s×2.5=12.5 时不至于太敏感）
SURGE_STOP_HORIZON_S = 1.5      # 剩余量刹车：剩余 ≤ 速率 × 1.5s 就停振（停振到断料有延迟）
SURGE_STOP_MIN_REMAIN_MG = 12.0  # 剩余量刹车的最小剩余（低速时别一上来就停）
SURGE_SETTLE_MAX_S = 3.0        # 熔断后等余料落完的上限
SURGE_SETTLE_STABLE_MG = 2.0    # 判稳：窗口极差
SURGE_SETTLE_SAMPLES = 4        # 判稳：窗口样本数（4×0.12s≈0.5s）
SURGE_SETTLE_RATE_EPS_MG_S = 1.0  # 判稳：窗口斜率上限
SURGE_COOLDOWN_S = 1.0          # 两次熔断的最小间隔（防抖）
SURGE_AMP_CUT_V = 0.10          # 每次熔断把振幅压下来这么多（塌料往往就是振幅过强）
SURGE_AMP_CUT_MAX = 3           # 一次加粉最多降这么多档（降多了会把自己降进死区，出不了料）
SURGE_MAX_TIMES = 15            # 单次加粉最多熔断次数（到数就不再熔断，交给原逻辑）
SURGE_MIN_GAIN_MG = 5.0         # 短窗增重太小就不判（1count=1mg 的噪声挡掉）
# 塌料够大就**别再连续加**了 —— 但出口不是缓补，是**提前切段**（2026-10-26 18:35 用户：
# "熔断只发生在阶段切换之间就行了"）。熔断的语义 = "这一段到此为止"：停振、余料落定、
# 交给下一段用更低的速率/振幅接着加；剩余量永远不由脉冲整包兜底。
# 旧版（11:33 加的 `to_pulse`）把剩余量全塞给缓补脉冲：26% 进度就跳进"缓补"，
# 单发 ~100mg、粗加和慢加整段被绕过——那是把"安全出口"当成了"主通道"。
# 判据只看"停振之后还在不在落"：短窗速率高但停振期间一两粉没再落，说明那一坨已经
# 落完了（sim 里 300mg 一次性塌料就是这种），本段继续加就行，不该越级跳到慢加。
SURGE_ADVANCE_MG = 60.0         # 熔断停振期间又落 ≥60mg → 料道还在塌，本段提前结束

# ── 切段沉降门 + 提前降幅（2026-10-21 用户批准，重粉必需）──────────────────
# 快加/粗加以高速率切出时，料槽里的粉还在落：切段那一刻"质量到位"不等于"落完了"。
# 那几 mg～几十 mg 余料会直接顶进下一段的切换点/判停点，重粉尤其明显
# （落差短、单次落料量大），结果是粗加一开场就超标、慢加要倒着补。
# 做法：停振 → 等余料落干净（判稳或超时）→ 再进下一段，并把这段余料记进粉体档案。
# **2026-10-26 11:05 用户定**：门槛按段算，不再是一个全局绝对值——
# 切出速率 ≥ **本段规定速率 × 1.5** 才值得停振等余料。粗加规定 20mg/s → 门槛 30mg/s；
# 快加规定 75mg/s → 门槛 112.5mg/s（实际跑不到，等于快加不再停振等待）。
# 理由：同样的 30mg/s 放在慢加（5mg/s）是 6 倍超速、放在快加只是 0.4 倍，用一把尺子量
# 三个段没有物理意义。页面「触发速率」填了数就是显式绝对值（覆盖这条比例）。
SETTLE_GATE_RATE_RATIO = 1.5           # 切出速率 / 本段规定速率 的门槛倍率
SETTLE_GATE_MIN_TRIGGER_MG_S = 3.0     # 比例门槛的绝对下限（防低速段门槛低到必然触发）
SETTLE_GATE_LEAD_S = 1.0               # 预测切段：质量 + 速率×1s 够到切点就提前停振
SETTLE_GATE_MAX_S = 8.0                # 单次等待**硬上限**（2026-10-27 11:35 从 1.2 → 8.0）
                                       # 旧值 1.2s 是致命错误：重粉停振后料道里的粉要好几秒
                                       # 才落得完，1.2s 到点就切 = 材料还在半空就把下一段的
                                       # 振幅/推杆全开（11:19 那次加粉沉降 +127mg、11:06 +93mg，
                                       # 全是"1.2s 到点、stable=False"）。真正的出口是判稳，
                                       # 这个上限只防"永远等下去"。
SETTLE_GATE_DROP_MG = 2.0              # 判稳：一个 hold 窗口内净增重 ≤ 此值 = 料走完了
SETTLE_GATE_HOLD_MIN_S = 2.0           # 判稳：窗口最少这么长
SETTLE_GATE_HOLD_MAX_S = 5.0           # 判稳：窗口最长（由落料周期 T_est 决定，钳在此区间）
                                       # 判据 = **一个完整落料周期内净增重 ≤2mg**（≈0.4mg/s）。
                                       # 旧判据是"5 个样本(0.5s)极差 ≤2mg 且斜率 ≤0.5mg/s"——
                                       # 脉冲式落料两坨之间本来就静，0.5s 窗随便就能满足，
                                       # 等于没判稳；而单拍阈值又会被天平 ±1mg 抖动反复触发。
SETTLE_GATE_STIR_OFF_S = 2.0           # 停振后这么久还没判稳 → **把搅拌也减速停**。
                                       # 漏料型粉（润滑剂1 实测：振动关着、搅拌 100RPM 时
                                       # 仍有 ≈5mg/s 往下塌）光停振永远判不稳，搅拌本身在推料；
                                       # 不停它就只能等满上限、带着 40mg 余料进下一段。
                                       # 停搅拌后下一段 `_enter_stage` 会按该段配方重新起转。
SETTLE_GATE_STIR_OFF_EXTRA_S = 4.0     # 停搅拌后额外给的判稳时间（余料还要走一小段）
SETTLE_GATE_STIR_OFF_RATE_MG_S = 3.0   # 停搅拌的触发速率：近 1.2s 仍 ≥3mg/s 才算"搅拌在推料"
# 提前降幅：距切点还有"当前速率 × 系数"mg 时，先把振幅压到下一段的预置值，
# 让流速在切点之前就下来——沉降门只需收最后几 mg，不会站那儿干等。
TAIL_TAPER_FACTOR = 0.6
TAIL_TAPER_MIN_MG = 8.0                # 提前降幅的最小距离（低速段不至于提前太久）
# ★ 降幅窗口的看门狗（2026-10-26 11:17 现场事故）：粗加 79/200mg，距切点（85.2mg）
# 只剩 6mg 时触发提前降幅 2.80→2.00V；结果 2.00V 这粉一粒不出（低于起振门槛），
# 质量冻在 79.0mg，切点永远到不了；而 `_taper_hold` 把振幅环**连踢一脚一起**冻住，
# 于是面板"加粉中"空转 115s（77.3s 时截图 +0.0mg/s、幅 2.00V），只能人工停止。
# 规矩：提前降幅只许"把流速压下来"，**绝不许把出料压停**。窗口内一旦断料就撤幅。
TAIL_TAPER_DRY_S = 2.0                 # 降幅后连续这么久没落料 → 判"降过头了"
TAIL_TAPER_DRY_RATE_MG_S = 1.0         # 低于这个速率算"没落料"
TAIL_TAPER_HOLD_MAX_S = 6.0            # 窗口总时长上限：到点无论出不出都放开振幅环

# ── 缓补（trim pulse，2026-10-22 11:30 用户提出）────────────────────────────
# 慢加段最后一段连续出料的落点误差有 ±几 mg 的不确定性（惯性余料 + 判停滞后），
# 对"验收 ±10mg"来说太毛。缓补 = 剩余量进入最后 20~40mg 区间时停掉连续振动，
# 改为**脉冲补料**：短促开一次振（≈10mg 一发）→ 关振等落定称重 → 再发下一发，
# 一发一发凑到目标值。是否启用按粉体档案里的 ``trim_pulse`` 字段（默认关，
# 慢加速率浮动小的粉如膨润土/封堵剂1 不需要开）。
TRIM_ENTER_REMAIN_MG = 40.0   # 慢加段剩余 ≤40mg 进入缓补（用户：剩 30 左右，20~40 区间）
TRIM_PULSE_TARGET_MG = 10.0   # 每发期望量级的参考值（**只用于日志/回溯，不再驱动时长**）
# 每发脉冲时长**恒定 0.40s**（2026-10-26 10:32 用户："全部设为 0.4 秒，不变，
# 只变幅度即可，慢慢出就行"）。所以这里只有一个数：
#   · 不再"首发 0.60s 之后按增重自适应"（原来 ×1.5 拉长 / ×0.65 缩短）；
#   · 不再按剩余量比例缩短（原来 duration×remaining/ref_gain）；
#   · 加多了（>18mg）也不再砍半——**压量完全交给振幅**（超 3V 退档、≤3V 就这档慢慢出）；
#   · 振幅到顶还不出也不拉长脉冲（只拉长搅拌预转，见 `_trim_escalate_on_no_gain`）。
# 唯一还会让实际振动时间变短的是**脉冲中盯秤掐断**（到 目标−3mg 立即停），
# 那是安全刹车，不是时长规则。
TRIM_PULSE_S = 0.40
TRIM_PULSE_MAX_N = 24         # 脉冲发数上限（防死循环；24 发 × 0.40s 足够凑 240mg）
# 真脉冲模式（2026-10-22 12:38 用户指出：信号发生器自带脉冲输出，不必用方波
# 开-关去模拟）："burst" = 猝发（WM 个数 + WP1s 触发，设备数完 count 个周期
# **自动停**，关断时刻由硬件保证，粒度 = 1/频率 = 12.5ms@80Hz）；
# 猝发指令失败（设备不认/超时）自动降级回 "timed"（软件掐表开-关）。
TRIM_PULSE_MODE = "burst"
# 每发脉冲后的称重等待（2026-10-23 用户要求"每缓补一次等 2~3s，再按重量决定要不要补下一发"）。
# 为什么必须有下限：猝发刚停那 1s 里秤上极差很快收窄到 2mg，但粉还在输送段往下走，
# 0.9s 就判"落定"等于把"暂时不动"当成"落完了"（10-23 那次加粉 205mg 判稳收工、最终漂到 211mg）。
# 下限 2s 等够尾料，上限 3s 兜住"永远等不到稳"的情况，到点用当前读数继续。
TRIM_SETTLE_MIN_S = 2.0
TRIM_SETTLE_MAX_S = 3.0
# 缓补入口等慢加尾料：尾料是几十 mg 量级（10-23 那次加粉 161→205mg，44mg），
# 给足时间落干净再发第一发，否则第一发的增重里混着尾料，自适应时长全是错的。
TRIM_TAIL_SETTLE_MAX_S = 8.0
# 进缓补的**第一段观察窗**（2026-10-26 18:04 用户："慢加之后的缓补好像没有等待粉末沉积"）：
# 先只停振、**搅拌保持**这么久，再停搅拌。那次加粉 1978 → 2048 的 +70mg 全落在
# "停振"和"stir_stop 串口回来"之间（约 0.4s），程序既没等也没看见，一读数就越过
# 2010 停手线，缓补 0 发脉冲收场。给一个明确窗口，让沉积真的有一次"被等"的机会，
# 顺带能分清崩落是停振引起的还是停搅拌（急停）引起的。
TRIM_ENTRY_HOLD_S = 1.2
TRIM_STABLE_RANGE_MG = 2.0    # 判稳：约 1s 窗口内极差 ≤2mg 视为落定
TRIM_NO_GAIN_LIMIT = 6        # 连续多少发完全没出粉就停手（每发之间振幅上抬试探）
TRIM_PULSE_AMP_RAISE_V = 0.5  # 已废弃（保号不删，防外部脚本 import 报错）：
                              # 现在抬幅走 `_trim_escalate_on_no_gain`，每发没出 +0.5V。
# ── 缓补**存粉保护**（2026-10-28 14:30 用户，抑制剂 3000mg 病案）──────────────
# 现场：粗加中途长时间不出粉（搅拌 150RPM 一直转，粉全挂在料道里），最后几秒
# 料道一松倾泻：判停时 2987mg 还差 3mg 进带，此后"只要搅拌一丢就出大量粉末"，
# 缓补 0 发脉冲、收尾称重却涨到 3052（+52 偏重）。
# 用户方案："不搅拌，先低幅度振动试试，再慢慢加振动"——存粉时搅拌是倾泻的扳机，
# 振动才是细水长流的阀门。
# 触发判据：**缓补入口泄料 ≥ TRIM_STOCK_GAIN_MG**（停振+停搅拌两段观察窗的合计
# 落料，那次加粉实测 +19mg 还只是冰山一角）。只用于正常缓补（hold_amp 定幅补差
# 振幅锁定是 2026-10-27 的独立规矩，不掺）。
# 存粉模式下：每发**不起搅拌**、振幅从 TRIM_STOCK_AMP_START_VPP 起步、
# 没出粉每发只 +TRIM_STOCK_AMP_STEP_VPP（比正常模式的 0.5V 温柔得多）；
# 出了粉就停在这档慢慢出（不退档不抬档）；连续 TRIM_STOCK_MAX_NOGAIN 发没出粉
# 才转回正常脉冲模式（起搅拌），振幅也恢复到进缓补时的值。
TRIM_STOCK_GAIN_MG = 15.0
TRIM_STOCK_AMP_START_VPP = 1.0
TRIM_STOCK_AMP_STEP_VPP = 0.2
TRIM_STOCK_MAX_NOGAIN = 8
# ── 缓补脉冲振幅（2026-10-23 12:16 用户纠正：**不许一步跳到 4V**）────────────
# 现场那次加粉 300mg/润滑剂1：前两发 2.2V 没出粉，第三发直接跳 4.0V ——用户明确：
# "不应该这样，而是调节到 2.7V 即可"。起振门槛常常只比当前值高半档，一步跳到 4V
# 那一发就多冲 20~30mg（4V 满流实测 28mg/s），直接偏重。
# 所以改成**每发没出粉只抬 0.5V**：2.2 → 2.7 → 3.2 → 3.7 → 4.2 …顶 5.0V。
# 设备 QLS3600S 本身支持到 20Vpp，5.0V 只是"连续脉冲"的自我约束，不是设备上限。
TRIM_AMP_MAX_VPP = 5.0
TRIM_AMP_NO_GAIN_STEP_VPP = 0.5   # 每发完全没出粉，振幅上抬这么多
# ── 缓补振幅**也要能往下走**（2026-10-23 16:05 用户定的规矩）──────────────────
# 用户原话："同一幅度第二次脉冲要根据上次的增重来判断，比如这次同一幅度加多了太多，
# 应该稍微减小一些幅度"。
# 那次加粉（1200mg/抑制剂）的现场：3.10V 第一发 +12mg、第二发 **+38mg**，同样 0.60s、
# 同样振幅，出料量差 3 倍；而程序只会在"没出粉"时往上抬，**从来不降**，
# 于是剩 13mg 还照发一整发 3.10V，直接冲到 1225（+25mg）。
# 所以补两条：① 剩的量已经接近"这档振幅上一发出了多少" → 发之前先降一档；
#          ② 同一振幅下这一发比上一发暴涨 → 判为放量，降一档给下一发用。
# 下限取"最近一次完全没出粉的振幅 + 0.25V"（低过它就是白振），再兜设备绝对下限。
TRIM_AMP_DOWN_STEP_VPP = 0.30     # 降幅一档（升是 0.5，降比升细）
TRIM_AMP_FLOOR_VPP = AMP_MIN_VPP  # 0.01：2026-10-27 11:08 用户"1V 下限删掉，最低 0.01V"。
                                  # 真正的有效地板还是 dead+0.25（实测死点），这里只是兜底。
TRIM_AMP_DEAD_MARGIN_V = 0.25     # 地板 = 起振死点 + 这个余量
TRIM_AMP_OVER_RATIO = 1.5         # 剩余量 < 同振幅上发增重 ×1.5 → 发之前先降幅
TRIM_AMP_BLOWUP_RATIO = 1.8       # 同振幅下本发增重 ≥ 上发 ×1.8 → 判放量，降幅
# ── 抬档之后"一出粉就退一档"（2026-10-26 10:18 用户定的硬规矩）────────────────
# 现场（2000mg/抑制剂）：2.5V +4mg → 2.5V +0 → 3.0V +0 → 3.5V +0 → **4.0V +16mg**
# → 4.0V 再来一发 +38mg。用户原话：
#   "假如 2V/2.5V 没出粉、3V 不出，尝试 3.5V 出后也就是大于 3V 的值时，如果出了粉
#    就需要回退到 3V 加粉；4V 出了就回退到 3.5V，回退之后如果 3.5V 出的多了就回退
#    到 3V，一次超过 18mg 算多了。"
# 也就是说：**抬档位是迫不得已的试探，试探成功就得退回去**，不能像旧版那样
# 一出粉就把新档位当成工作点——那个档位对上这种粉是"要么不出、要么一次冲 40mg"。
# 退到某档后如果出料 ≤18mg，就停在这一档（记 `_trim_amp_ok`），不再来回试探；
# 只有"单发 >18mg"才继续往下退。
# **3.0V 是回退触发线**（2026-10-26 10:26 用户）：只有高于 3.0V 的振幅才需要往回退，
# ≤3.0V 当成稳定工作点、不再触发回退。
# **一次退 1.0V**（2026-10-26 10:44 用户："回退机制再强力一点，一次回退 1V，比如
# 3.5V 脉冲出了就回退到 2.5V"）：退档不再以 3.0V 为地板——3.5V 出粉就退到 2.5V，
# 4.0V 出粉退到 3.0V，4.5V 退到 3.5V。退下去那档若一粒不出，会走"抬 0.5V"回来，
# 由 `_trim_backoff_target` 防抖（同一次加粉不再退到同一档 → 认定该档是工作点，不再试）。
# 注意"触发条件"与"退到哪"是两件事：**只有 >3V 才触发回退，但退到的目标可以低于 3V**。
TRIM_AMP_BACKOFF_MIN_VPP = 3.00   # 回退**触发线**：振幅 > 这个值出粉才回退
TRIM_AMP_BACKOFF_STEP_VPP = 1.00  # 一次退 1.0V（3.5→2.5、4.0→3.0、4.5→3.5）
TRIM_GAIN_TOO_MUCH_MG = 18.0      # 单发增重超过这个 → 算"加多了"：退一档
# 起搅拌后先空转多久再开猝发（**关键**，2026-10-23 11:23 那次加粉的真正死因之一）：
# Emm42 acc=200 到 100RPM 约需 0.28s 爬升，0.15~0.22s 的猝发几乎整段落在爬升段，
# 转子还没把腔口那层粉推到位，振动就结束了——所以"0.15s 脉冲永远 +0mg"，
# 而 0.6s 脉冲能出 5~6mg。预转 0.35s 让整段猝发都发生在额定转速上。
TRIM_STIR_SPINUP_S = 0.35
# 一直不出粉时预转逐步拉长（振幅已到顶还不出 = 腔里没料，得靠搅拌多带一会儿下来）
TRIM_STIR_SPINUP_MAX_S = 2.0
TRIM_STIR_SPINUP_GROW_S = 0.5
# 猝发过程中盯秤：读数一旦到"目标 − 该余量"立刻掐脉冲。
# 长脉冲 + 突然来料就是一大坨（4V 满流 28mg/s，2.5s 能冲出 70mg），要有刹车。
TRIM_PULSE_WATCH_S = 0.15
TRIM_PULSE_WATCH_MARGIN_MG = 3.0
# 判停预测失误的补差上限（2026-10-22 18:33 用户指出"慢加直接结束了没有缓补"）：
# predictive_stop 高估尾料时提前停振 → 收尾称重偏轻。缓补开启且缺口 ≤ 此值时
# 判停后转缓补脉冲把差的补齐，不直接落"偏轻"结果。缺口更大说明工作点选错，
# 脉冲一颗颗补不现实，如实报偏轻让用户查原因。
TRIM_RESCUE_MAX_MG = 100.0
# 判停偏轻后、重启搅拌之前的"静置窗口"（2026-10-23 11:09 教训）：那次加粉判停 170mg 就立刻
# 重启搅拌补差，转子上挂的粉被带下来 58mg，收尾 233mg 偏重 33mg。搅拌停着先等 3~6s，
# 质量常常自己就走进了验收窗口，这种时候根本不该动搅拌。等完还偏轻才重启搅拌补差。
TRIM_RESCUE_QUIET_MIN_S = 3.0
TRIM_RESCUE_QUIET_MAX_S = 6.0


def _clamp_amp(value: float) -> float:
    """振幅统一钳位（0.01–4 Vpp）：任何路径写振幅都必须过这一关。

    下限 0.01V 是设备物理下限（QLS3600S 驱动 ``min_amplitude_vpp``），
    不是工艺下限——2026-10-27 用户明确要求删掉原先软件自定的 1.0V。
    """
    return round(max(AMP_MIN_VPP, min(AMP_MAX_VPP, float(value))), 2)


def _amp_band_for(spec: "StagePlan") -> tuple[float, float]:
    """本段振幅的允许区间（页面/CSV 显示用，必须与真实控制区间一致）。

    **三段统一（2026-10-23 11:50 用户改回带子宽度）**：下界 = 本段基准 −0.5V，
    上界 = 本段基准 +0.5V。这里说的"基准"是页面那一行的振幅（留空则用
    STAGE_DEFAULT_AMP_VPP：快加 3.0 / 粗加 2.5 / 慢加 2.0），不再是粉体标定值。
    曾经放开到 +2.5V/硬顶 5.0V（想让粗加硬追 20mg/s），用户看过写入记录后要求收回：
    振幅只在基准附近 ±0.5V 里小步走，越远越没有实测依据。

    **慢加**：同样是 基准 ±0.5V，而且只许"恢复出料"，绝不追速率。

    **粉级覆盖（2026-10-29 用户定）**：档案 ``dispense_flags.slow_amp_span_vpp``
    给了半宽时，慢加带 = 基准 ± 该值（封堵剂2 = ±1.0V，2.00V → 1.00–3.00V）。
    这是"该粉不许零速率脉冲"的补偿：不给脉冲，就得给更大的连续振幅调节余量。
    覆盖只作用于慢加——快加/粗加各自的 0.5V 是追速率的安全边界，不跟着放。
    """
    base = float(spec.amplitude_vpp)
    span = getattr(spec, "amp_span_vpp", None)
    if spec.name == "slow" and span is not None:
        span = abs(float(span))
        return (round(max(AMP_MIN_VPP, base - span), 2),
                round(min(AMP_MAX_VPP, base + span), 2))
    if spec.name == "slow":
        # 慢加下界 = 设备绝对下限 0.01V（2026-10-27 11:08 用户："1V 下限删掉"）：
        # 慢加是唯一"必须往低走"的段，目标 5mg/s 而 2.0V 常跑 15~25mg/s，
        # 任何软件自定的下界都会变成"压不下去"的天花板。
        return (round(max(AMP_MIN_VPP, AMP_SLOW_FLOOR_VPP), 2),
                round(min(AMP_MAX_VPP, base + AMP_SLOW_RAISE_V), 2))
    return (round(max(AMP_MIN_VPP, base - AMP_RAMP_BAND_V), 2),
            round(min(AMP_CTRL_CEIL_VPP, base + AMP_CTRL_RAISE_SPAN_V), 2))


def _powder_flags(workspace: Any) -> dict:
    """取档案里的加粉标记段 ``dispense_flags``（没有/格式不对就给空 dict）。

    目前认两个键（2026-10-29 用户定，只给封堵剂2 用）：
      · ``zero_rate_pulse``（bool，默认 True）：速率为 0 时允不允许踢脚脉冲救济。
      · ``slow_amp_span_vpp``（float，默认 None=0.5）：慢加振幅带半宽。
    """
    raw = getattr(workspace, "raw", None) if workspace is not None else None
    flags = (raw or {}).get("dispense_flags")
    return flags if isinstance(flags, dict) else {}


@dataclass
class StagePlan:
    """一个阶段的完整执行配方（页面会原样显示，方便操作员知道"现在在干什么"）。"""

    name: str                       # fast / coarse / slow
    mode: str                       # 控制器分支：coarse / fine / precision
    rate_mg_s: float                # 该阶段的目标流量
    stir_rpm: float                 # 该阶段的搅拌转速
    switch_mass_mg: float | None    # 实测质量达到它就进下一阶段；None = 一直跑到判停
    rod_ff: int                     # 前馈推杆开度（阶段入口开到这里，之后整段不动）
    rod_min: int
    rod_max: int
    duty_seed_permyriad: int
    duty_min: int
    duty_max: int
    position_feedback: bool         # 是否允许闭环动推杆（恒 False：速率靠振幅调）
    amplitude_vpp: float = 2.0      # 该段振幅**基准**（页面填/段默认；闭环在 ±0.5V 内调）
    amp_explicit: bool = False      # 振幅是不是页面手填的（手填就不再按上一段继承）
    # 本段闭环带**半宽**（None = 按段默认 0.5V，慢加也是 0.5V）。
    # 2026-10-29 用户定：封堵剂2 在档案 ``dispense_flags`` 里把慢加放宽到 ±1.0V
    # （慢加基准 2.00V → 允许 1.00–3.00V），作为"不许零速率脉冲"的补偿。
    amp_span_vpp: float | None = None
    # 该粉**允不允许在加粉速率为 0 时脉冲**（踢一脚那种 0.5s 高幅救济）。
    # 2026-10-29 用户："其他都可以，只有他（封堵剂2）不行"——别的粉零速率时靠脉冲
    # 震开料桥，封堵剂2 起振即成团爆发，脉冲只会甩出一大坨，所以改成靠放宽后的
    # 振幅带（连续振动）把料逼出来。False = 该段完全不踢脚。
    zero_rate_pulse: bool = True
    settle_wait_s: float | None = None   # 本段切出时等余料落完的上限 s（None=默认，0=不等）
    switch_compensation_mg: float = 0.0  # 切换点已按学到的惯性落料量提前了多少 mg
    control_note: str = ""
    rod_ff_expected_rate: float | None = None
    rod_ff_achievable: bool = True
    matched_rate_mg_s: float | None = None   # 标定表里离本段目标速率最近的实测点
    rate_capped_from: float | None = None    # 目标速率被"该工作点实际能力"钳掉之前的原值

    def to_dict(self) -> dict:
        return {
            "stage": self.name,
            "stage_text": STAGE_TEXT.get(self.name, self.name),
            "mode": self.mode,
            "rate_mg_s": round(self.rate_mg_s, 2),
            "stir_rpm": round(self.stir_rpm, 1),
            "switch_mass_mg": None if self.switch_mass_mg is None else round(self.switch_mass_mg, 1),
            "rod_ff": int(self.rod_ff),
            "rod_range": [int(self.rod_min), int(self.rod_max)],
            "duty_seed_pct": round(self.duty_seed_permyriad / 100.0, 1),
            "duty_range_pct": [round(self.duty_min / 100.0, 1), round(self.duty_max / 100.0, 1)],
            "position_feedback": bool(self.position_feedback),
            "amplitude_vpp": round(float(self.amplitude_vpp), 2),
            # 本段**真实闭环带**（基准 ±0.5V），不是设备量程 1–4V：
            # 页面拿这个显示"这一段振幅会在哪个区间里动"，和运行时的 cap/floor 完全一致。
            "amplitude_range_vpp": list(_amp_band_for(self)),
            "amplitude_span_vpp": (None if self.amp_span_vpp is None
                                   else round(float(self.amp_span_vpp), 2)),
            "zero_rate_pulse": bool(self.zero_rate_pulse),
            "amp_explicit": bool(self.amp_explicit),
            "settle_wait_s": (None if self.settle_wait_s is None else round(self.settle_wait_s, 2)),
            "switch_compensation_mg": round(float(self.switch_compensation_mg), 1),
            "control_note": self.control_note,
            "rod_ff_expected_rate_mg_s": (None if self.rod_ff_expected_rate is None
                                          else round(self.rod_ff_expected_rate, 2)),
            "rod_ff_achievable": bool(self.rod_ff_achievable),
            "matched_rate_mg_s": (None if self.matched_rate_mg_s is None
                                  else round(float(self.matched_rate_mg_s), 2)),
            "rate_capped_from": (None if self.rate_capped_from is None
                                 else round(float(self.rate_capped_from), 2)),
        }


@dataclass
class TrimSpec:
    """缓补段的记账壳子：只为复用 ``_start_stage_log`` / ``_close_stage_log`` 的字段。

    缓补不是 StagePlan（不进三阶段计划、没有切换点），它发生在慢加段尾部；
    记账时把当时的搅拌/推杆实际值填进来，日志里能对上"缓补时设备是什么状态"。
    """

    name: str = "trim"
    rate_mg_s: float = 0.0          # 记 0：缓补不追速率，只数"每发多少 mg"
    stir_rpm: float = 0.0
    rod_ff: int = 0
    amplitude_vpp: float = 2.0      # 记账用：缓补期间振幅保持慢加段基准值不动
    # 这段脉冲是怎么进来的：slow=慢加剩 ≤40mg 正常缓补 / topup=判停偏轻补差。
    # （熔断不再转脉冲，2026-10-26 18:35 用户："熔断只发生在阶段切换之间"）
    source: str = "slow"


class DispenseRunner:
    """单次闭环加粉的后台执行器（同一时间只允许一个实例在跑）。"""

    SAMPLE_INTERVAL_S = 0.12      # 观测周期：比页面 0.5s 快，够用到观测器的 0.75s 预测
    MAX_RUN_S = 180.0             # 基准超时；大目标会按计划估算放大（见 _resolve）
    # 绝对兜底上限（2026-10-28 用户定）：**第一段（快加）走完后就不再用计划估算的
    # 上限掐**——料斗断流时慢加靠"顶幅→救济冷却"反复磨，用时远超估算（11:00 那次
    # 1986mg 差 4mg 进带被 291s 掐掉），但只要快加正常走完就说明给料链路没问题，
    # 剩下的量是有限的，让它自己收。只保留这个 2 小时绝对兜底防"料斗真空转"。
    MAX_RUN_ABS_S = 7200.0
    OVERSHOOT_ABORT_MG = 30.0     # 兜底超重保护（验收是 ±10，留 20mg 给惯性/抖动）
    MAX_BALANCE_FAILS = 5         # 连续读不到天平 ≈ 0.6s 就停，闭环不能"盲跑"
    SETTLE_TIMEOUT_S = 6.0        # 停振后等尾料沉降
    # 判稳最短时间：5 个样本（0.75s）极差 ≤2mg 就返回太早了——搅拌刚减速、
    # 转子上挂的粉还在断续掉，秤会先平一下再掉一坨。10-23 那两次加粉都是"判停 205、
    # 最终 211"这种尾差，就是 0.75s 判稳放走的。等满 2s 再允许判稳。
    SETTLE_MIN_STABLE_S = 2.0
    ROD_ARRIVE_TIMEOUT_S = 5.0    # 推杆到位等待
    WAVEFORM_SQUARE = 2           # 真机实测 2 = 方波（占空比只对方波生效）
    STIR_ACC = 200                # 搅拌加/减速曲线等级（>1300 RPM 必须曲线启动）
    STIR_SETTLE_S = 1.2           # 起振前等搅拌爬到位
    # 振幅环的判据参数在模块级 AMP_RATE_CTRL / AMP_BAND_V 里（慢加段不参与闭环）

    # 阶段切换规则（用户 2026-10-21 定的口径，改规则只改 stage_boundaries）
    FAST_TARGET_MIN_MG = 1000.0   # 目标 > 1000 才启用快加
    SLOW_MIN_RESERVE_MG = 40.0    # 慢加段至少留 40mg（<300mg 的小目标）
    SLOW_MAX_RESERVE_MG = 150.0   # 慢加段最多留 150mg（1000mg 目标时 15%）
    # 大目标口径（2026-10-21 14:40 用户定）：目标 > 5000mg 时快加不看百分比，
    # 一路冲到"只剩 800mg"就切粗加 —— 大目标按 75% 切，粗加段要走几千克，太久了。
    LARGE_TARGET_MIN_MG = 5000.0
    LARGE_FAST_TAIL_MG = 800.0    # 大目标快加段留到最后的余量

    def __init__(self, hub: Any, log: Callable[..., None]) -> None:
        self.hub = hub
        self._log = log
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._state = "idle"          # idle/preparing/running/settling/done/aborted/failed
        self._detail = "尚未运行"
        self._config: dict[str, Any] = {}
        self._live: dict[str, Any] = {}
        self._result: dict[str, Any] | None = None
        self._samples: list[tuple[float, float, float, str]] = []   # (t, mass, rate, stage)
        self._curve: list[list[float]] = []
        self._stage_log: list[dict] = []
        # 线程用到的运行期字段给默认值，避免"准备阶段就中止"时踩到未定义属性
        self._baseline = 0.0
        # 有没有读过本次加粉基线：没有基线时页面不显示"净质量"，
        # 免得把秤上的绝对示数（可能还是上一次加粉的皮重）当成这一次加粉的增量。
        self._has_baseline = False
        self._rod_start = 0
        self._applied_freq = 0
        self._applied_duty = 0
        self._applied_window = 0
        self._applied_stir = 0.0
        self._applied_amp = 0.0
        self._amp_last_write_s = 0.0    # 上次写振幅的时刻（响应等待窗起点）
        self._amp_wait_s = AMP_CTRL_MIN_EVAL_S   # 本步写完后要等多久再评估
        self._amp_last_dir = None       # 上一步调幅方向（+1 升 / -1 降，超调制动用）
        self._amp_last_eval_s = 0.0
        # 振幅环的段内相位（2026-10-22 11:20 按用户"慢加绝对不许动、粗加断料必须动"重定）
        # 上一次改动的探针：{amp0, rate0, step, elapsed}，只用于日志复盘。
        self._amp_probe: dict | None = None
        self._amp_band = (AMP_MIN_VPP, AMP_MAX_VPP)   # 本段振幅允许区间（按标定值算）
        # ── v6 延迟模型（2026-10-22 12:38 用户要求"消除延迟的影响"）──────────
        # 每次写振幅记 (t, from, to)；之后第一次落料阶跃出现 → τ_est = 阶跃−写入；
        # 相邻落料阶跃间隔 → T_est。判据窗口永远取 [写入+τ_est, now] 且 ≥ T_est。
        self._amp_write_marks: list[tuple[float, float]] = []  # (t, amp_to) 未关联的写入
        self._tau_est = AMP_TAU_INIT_S                # 纯滞后估计 s（在线 EMA）
        self._period_est = AMP_PERIOD_INIT_S          # 落料周期估计 s（在线 EMA）
        self._last_mass_for_drop: float | None = None # 阶跃检测用上一拍质量
        self._last_drop_s: float | None = None        # 上一次落料阶跃时刻
        self._prev_eval_rate: float | None = None     # 上一个生效窗速率（趋势闸用）
        self._k_est: float | None = None              # 振幅→速率增益 mg/s per V（在线学习）
        self._stall_wait_s = -99.0                    # 卡粉救济上一次动作时刻
        self._amp_kick: dict | None = None  # 踢一脚进行中：{amp, base, until}
        self._amp_kick_tries = 0      # 本段已踢了几脚（段入口复位）
        self._amp_kick_last_amp = 0.0 # 上一脚的振幅（判断是否已到顶）
        self._amp_kick_m0_total = 0.0 # 本段第一脚踢出时刻的质量（判这几脚累计有没有出料）
        self._amp_lock_reason = ""      # 本段为什么锁住不再调（写进日志用）
        self._amp_log_s = -99.0         # 上次振幅日志时刻（防刷屏）
        self._amp_skip_log_s = -99.0    # 上次"暂不调整"日志时刻（防刷屏）
        self._amp_heartbeat_s = -99.0   # 上次振幅心跳日志时刻
        self._amp_saturated = False     # 本段已锁定（顶到边界 / 升幅无效 / 改够次数）
        self._ramp_below_since: float | None = None  # 粗调坡道：速率开始偏慢的时刻
        self._ramp_last_s = -99.0       # v7：上一次调幅时刻（评估间隔共用）
        # v7 自适应状态：前次评估的均速/时刻、加速度 EMA、到带顶的时刻
        self._v7_prev_rate: float | None = None
        self._v7_prev_t: float | None = None
        self._v7_a_est: float | None = None
        self._amp_at_cap_since: float | None = None
        self._amp_changes = 0           # 本段振幅改动次数（段结束汇总用）
        self._amp_min = 0.0             # 本段振幅最小值
        self._amp_max = 0.0             # 本段振幅最大值
        self._stage_start_s = 0.0       # 本段起点时刻（慢加动态目标要用）
        self._stage_start_mass = 0.0    # 本段起点质量
        # 质量历史：（elapsed, 净质量 mg），振幅环用它做回归算速率。
        # 为什么不用 estimate 的瞬时速率：封堵剂1 那种脉冲出料，天平只在粉块落下时
        # 跳一下，中间是平的，差值算出来的速率中位数直接是 0（实测 77% 的采样读成 0），
        # 拿它做反馈等于闭眼调参。回归斜率对脉冲+噪声都稳。
        self._mass_hist: list[tuple[float, float]] = []
        # ── 逐拍采样 CSV（对齐老 feedback_dispense 的复盘习惯，2026-10-22 用户要求）──
        # 每个天平采样一行（约 0.12s 一个），把振幅环当时的全部决策现场都记下来，
        # 事后才能回答"这一拍它为什么不动/动了多少"——光靠 5s 心跳复盘不了算法。
        self._samples_csv = None            # 文件句柄（首拍惰性打开）
        self._samples_csv_path: str | None = None
        self._amp_cmd = 0.0                 # 振幅环当前目标速率 mg/s
        self._amp_detail = ""               # 振幅环最近一拍的判定原因
        self._kick_count = 0                # 本次加粉累计踢脚次数
        self._t0 = 0.0
        self._max_run_s = self.MAX_RUN_S
        self._internal_stop = ""
        self._last_raw: int | None = None     # 最后一次成功读数：天平失联时当估计终值
        self._plan: list[StagePlan] = []
        self._workspace: PowderWorkspace | None = None
        # 切段沉降门（本次运行的开关与阈值，页面可覆盖；默认值见类常量）
        self._gate_enabled = True
        # None = 按段比例门槛（本段规定速率 ×1.5）；填了数 = 页面指定的绝对门槛 mg/s
        self._gate_trigger: float | None = None
        self._taper_factor = TAIL_TAPER_FACTOR
        self._vib_running = False             # 振动输出当前是否开着（沉降门会临时关掉）
        self._settle_events: list[dict] = []  # 每次切段测到的"停振后还落了多少"
        self._taper_done_index = -1           # 已经提前降过幅的段（每段只降一次）
        self._taper_hold = False              # 切点前降幅窗口内：暂停振幅环
        self._taper_hold_since = 0.0          # 降幅窗口起点（看门狗计时用）
        self._taper_prev_amp: float | None = None  # 降幅前的工作振幅（断料要能退回去）
        self._stall_abort_ref = 0.0           # 断料兜底：参考质量
        self._stall_abort_s = 0.0             # 断料兜底：参考时刻
        # 段内超速熔断（2026-10-26 11:33）：开关/倍率/计数/最近一次时刻/事件留档
        self._surge_on = SURGE_GUARD_ON
        self._surge_ratio = SURGE_RATE_RATIO
        self._surge_times = 0
        self._surge_last_s = -99.0
        self._surge_amp_cuts = 0
        self._surge_events: list[dict] = []
        self._surge_advance = False     # 熔断请求"本段提前结束"（下一拍按切段处理）

    # ------------------------------------------------------------------
    # 对外状态
    # ------------------------------------------------------------------

    @property
    def running(self) -> bool:
        """是否正在占用硬件（preparing 也算：那会儿已经在动推杆/搅拌）。"""
        return self._state in ("preparing", "running", "settling")

    def status(self) -> dict:
        with self._lock:
            payload = {
                "state": self._state,
                "running": self.running,
                "detail": self._detail,
                "config": dict(self._config),
                "live": dict(self._live),
                "result": self._result,
                "plan": [stage.to_dict() for stage in self._plan],
                "stages": list(self._stage_log),
            }
        return payload

    def net_mass(self, raw: float) -> float | None:
        """把天平原始计数换算成「本次加粉净质量」（计数 − 开始加粉前的基线）。

        为什么需要它：闭环主循环只在运行期间每 0.12s 采样，一旦进入收尾（settling）
        或结束（done/aborted），``live.mass_mg`` 就冻在最后一帧（例如 991），
        而沉降完的终值可能是 1009 —— 页面上「当前质量」和「实得」两个数对不上。
        页面轮询时用它自己读到的最新天平计数换算，收尾和结束后都能继续实时刷新。

        没有基线（服务刚起、还没跑过加粉）时返回 None。
        """
        with self._lock:
            if not self._has_baseline:
                return None
            return float(raw) - self._baseline

    # ------------------------------------------------------------------
    # 启动 / 停止
    # ------------------------------------------------------------------

    def start(
        self,
        target_mg: Any,
        *,
        category: str | None = None,
        preset_id: str | None = None,
        amplitude_vpp: float | None = None,
        tail_mass_mg: float | None = None,
        powder: str | None = None,
        use_feedforward: bool = True,
        stages: list[dict] | None = None,
        settle_gate: bool | None = None,
        settle_gate_rate_mg_s: float | None = None,
        taper_factor: float | None = None,
        surge_guard: bool | None = None,
        surge_ratio: float | None = None,
        trim_enabled: bool | None = None,
        trim_enter_remain_mg: float | None = None,
        trim_pulse_target_mg: float | None = None,
    ) -> dict:
        """校验参数 → 定阶段计划 → 起线程。真正的硬件动作都在线程里。

        参数来源优先级（从高到低）：
        1. 页面显式传入的三阶段配方（``stages``：搅拌/推杆/速率/占空比/切换质量）；
        2. 该粉末的标定段（振动参数、前馈推杆、三阶段搅拌转速）；
        3. 算法层预设（``DISPENSE_PRESETS`` 种子 + ``interpolate_profile`` 速率档）。

        ``powder`` 为空时退化成"纯闭环 + 预设种子"，搅拌不参与（转速 0）。
        """
        if self.running:
            raise RuntimeError(f"已有加粉任务在运行（{self._state}），请先停止")
        r = self._resolve(target_mg, category=category, preset_id=preset_id,
                          amplitude_vpp=amplitude_vpp, tail_mass_mg=tail_mass_mg,
                          powder=powder, use_feedforward=use_feedforward, stages=stages)
        target = r["target"]
        plan = r["plan"]
        workspace = r["workspace"]
        ff_ready = r["ff_ready"]
        tail = r["tail"]
        ff_fit = r["ff_fit"]

        # 切段沉降门参数：页面没传就按段算（本段规定速率 ×1.5），传了就是绝对门槛。
        gate_on = True if settle_gate is None else bool(settle_gate)
        gate_trigger = (None if settle_gate_rate_mg_s is None
                        else float(settle_gate_rate_mg_s))
        if gate_trigger is not None and not 0.0 <= gate_trigger <= 300.0:
            raise ValueError(f"沉降门触发速率 {gate_trigger:g} mg/s 超出 0–300")
        taper = TAIL_TAPER_FACTOR if taper_factor is None else float(taper_factor)
        if not 0.0 <= taper <= 3.0:
            raise ValueError(f"提前降幅系数 {taper:g} 超出 0–3（0 = 不提前降幅）")
        # 段内超速熔断：页面没传就开（默认 2.5 倍），传了就按页面（0 = 关）
        surge_on = SURGE_GUARD_ON if surge_guard is None else bool(surge_guard)
        surge_ratio = SURGE_RATE_RATIO if surge_ratio is None else float(surge_ratio)
        if not 0.0 <= surge_ratio <= 20.0:
            raise ValueError(f"超速熔断倍率 {surge_ratio:g} 超出 0–20（0 = 关）")
        if surge_ratio <= 0.0:
            surge_on = False
        # 缓补（trim pulse）数值参数：页面留空 = 模块默认（触发剩余 40mg、每发 10mg）。
        # 校验放在动任何机构之前，跟沉降门参数同一批。
        trim_enter = (TRIM_ENTER_REMAIN_MG if trim_enter_remain_mg is None
                      else float(trim_enter_remain_mg))
        trim_pulse_mg = (TRIM_PULSE_TARGET_MG if trim_pulse_target_mg is None
                         else float(trim_pulse_target_mg))
        if not 5.0 <= trim_enter <= 200.0:
            raise ValueError(f"缓补触发剩余量 {trim_enter:g} mg 超出 5–200")
        if not 3.0 <= trim_pulse_mg <= 50.0:
            raise ValueError(f"缓补每发目标 {trim_pulse_mg:g} mg 超出 3–50")

        # 启动前的机构静止校验（COM10/COM9/COM16 都会真实读一次）
        idle = self.hub.assert_idle_for_dispense()

        # 缓补开关三态：页面显式传 true/false 就以页面为准；
        # 没传（None）跟随粉体档案里的 trim_pulse 字段（布尔，没写 = 关）。
        # （必须在 _config 之前取——_config 里的 trim_pulse 条目要用它）
        trim_raw = (workspace.raw or {}).get("trim_pulse") if workspace is not None else None
        trim_on = bool(trim_raw) if trim_enabled is None else bool(trim_enabled)

        with self._lock:
            self._state = "preparing"
            self._detail = f"准备中：目标 {target} mg，{len(plan)} 段，种子 {r['seed_id']}"
            self._result = None
            self._live = {}
            self._samples = []
            self._curve = []
            self._mass_hist = []
            self._stage_log = []
            self._plan = plan
            self._config = {
                "target_mg": target,
                "powder": workspace.name if workspace else None,
                "powder_file": workspace.path.name if workspace else None,
                "calibrated": bool(ff_ready),
                "category": r["category"],
                "preset_id": r["seed_id"],
                "amplitude_vpp": round(r["amp_vpp"], 3),
                "amplitude_range_vpp": [AMP_MIN_VPP, AMP_MAX_VPP],
                "frequency_hz": round(r["freq_hz"], 2),
                "duty_percent": round(r["duty_pct"], 2),
                "signal_source": r["signal_source"],
                "tail_mass_mg": tail,
                "trim_pulse": {"enabled": trim_on,
                               "enter_remain_mg": round(trim_enter, 1),
                               "pulse_target_mg": round(trim_pulse_mg, 1),
                               "source": ("page" if trim_enabled is not None
                                          else "powder_profile")},
                "settle_gate": {"enabled": gate_on,
                                "rate_mg_s": (None if gate_trigger is None
                                              else round(gate_trigger, 1)),
                                "rate_ratio": SETTLE_GATE_RATE_RATIO,
                                "max_s": SETTLE_GATE_MAX_S,
                                "taper_factor": round(taper, 2)},
                "surge_guard": {"enabled": surge_on,
                                "rate_ratio": round(surge_ratio, 2),
                                "abs_min_mg_s": SURGE_ABS_MIN_MG_S,
                                "stop_horizon_s": SURGE_STOP_HORIZON_S,
                                "max_times": SURGE_MAX_TIMES},
                "feedforward": ff_fit,
                "seed": dict(r["seed"]),
                "stage_overrides": r["stage_overrides"],
                "settings": {
                    spec.name + "_rate_mg_s": round(spec.rate_mg_s, 2) for spec in plan
                } | {"fixed_tail_mass_mg": tail},
                "rod_start": idle["rod_position"],
                "started_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            }
        self._max_run_s = r["max_run_s"]
        self._stop_event.clear()
        self._workspace = workspace if ff_ready else None
        self._trim_on = trim_on
        # 缓补可调参数落到实例上（页面留空 = 模块默认）：触发剩余量 / 每发目标。
        # 注：`_trim_pulse_mg` 自 2026-10-26 10:32 起**不再驱动脉冲时长**（时长恒 0.4s），
        # 只作为"这粉一发大概多少 mg"的参考值打进日志，方便对照。
        self._trim_enter_mg = trim_enter
        self._trim_pulse_mg = trim_pulse_mg
        self._trim_rescued = False       # 判停失误补差每次加粉只允许一次
        self._trim_stir_rpm = 0.0        # 缓补脉冲期间临时用的搅拌转速（入口从慢加转速记下）
        # 缓补上一发的实测增重（mg）。用户 16:05 要求"同一幅度第二次脉冲要根据上次的
        # 增重来判断"——这个数现在只用来判"这档振幅是不是太凶"（放量/剩量不足就降幅），
        # **不再用来算脉冲时长**（时长恒 0.40s，2026-10-26 10:32）。
        # 同时记住这个增重是**哪个振幅**下测的，换了振幅这条数据就作废，否则拿 3.1V 的
        # 增重去推 2.8V 该出多少，越推越离谱。
        self._trim_last_gain = 0.0
        self._trim_last_gain_amp = 0.0
        # 最近一次"完全没出粉"的振幅（起振死点），用来给降幅定地板
        self._trim_amp_dead = 0.0
        # 本发是不是"刚抬上来的新档位"（2026-10-26）：抬档后一出粉就要退回去
        self._trim_pulse_escalated = False
        # 已验证"出料 ≤18mg"的振幅：退回到这一档就停手，不再来回试探
        self._trim_amp_ok = 0.0
        # 上一次退档退到的振幅：同一次加粉里再想退到 ≤ 这一档就直接认定工作点
        # （那档已知一粒不出），避免 3.5↔2.5 这种来回空振。
        self._trim_backoff_target = 0.0
        # 沉降门参数落到实例上：主循环只在同一个线程里读，不需要再加锁
        self._gate_enabled = gate_on
        self._gate_trigger = gate_trigger
        self._taper_factor = taper
        self._vib_running = False
        self._settle_events = []
        self._taper_done_index = -1
        self._taper_hold = False
        self._taper_hold_since = 0.0
        self._taper_prev_amp = None
        self._surge_on = surge_on
        self._surge_ratio = surge_ratio
        self._surge_times = 0
        self._surge_last_s = -99.0
        self._surge_amp_cuts = 0
        self._surge_events = []
        # 熔断判"这一段撑不住、提前切段"的请求位：由 `_surge_brake` 置位，
        # 主循环顶部当成第三个切段触发消费掉（2026-10-26 18:35 用户口径）。
        self._surge_advance = False
        self._thread = threading.Thread(
            target=self._thread_main,
            args=(plan, target, r["cls"], tail, ff_fit),
            name="dispense-runner", daemon=True)
        self._thread.start()
        self._log(
            f"闭环加粉启动：目标 {target} mg，粉体「{workspace.name if workspace else '未指定'}」"
            f"（{'已标定，用前馈' if ff_ready else '未标定，纯闭环'}），"
            f"{r['freq_hz']:g}Hz / {r['duty_pct']:g}%"
            f"（{'该粉标定参数' if r['signal_source'] == 'powder' else '默认参数'}）"
            f" / 段振幅 "
            + "、".join(f"{STAGE_TEXT[s.name]}{s.amplitude_vpp:g}V" for s in plan)
            + f"（每段在基准 ±{AMP_RAMP_BAND_V:g}V 内按速率闭环 0.05–{AMP_CTRL_UP_MAX_V:g}V 小步调，"
            f"慢了加、快了减；慢加只恢复出料不追速率），分段："
            + self._plan_brief(plan, target))
        if gate_on:
            if gate_trigger is None:
                gate_desc = (f"按段算门槛 = 本段规定速率 ×{SETTLE_GATE_RATE_RATIO:g}"
                             f"（" + "、".join(
                                 f"{STAGE_TEXT[s.name]} {s.rate_mg_s:g}×"
                                 f"{SETTLE_GATE_RATE_RATIO:g}={s.rate_mg_s * SETTLE_GATE_RATE_RATIO:g}"
                                 for s in plan if s.name != "slow") + " mg/s）")
            else:
                gate_desc = f"切出速率 ≥{gate_trigger:g} mg/s（页面指定绝对值）"
            self._log(f"切段沉降门：开（{gate_desc} 时停振等余料落完，"
                      f"最多 {SETTLE_GATE_MAX_S:g}s/段；质量+速率×{SETTLE_GATE_LEAD_S:g}s "
                      f"够到切点就提前停振；提前降幅 {taper:g}×速率、"
                      f"最少 {TAIL_TAPER_MIN_MG:g}mg）")
        else:
            self._log("切段沉降门：关（达标即刻切段，余料会顶进下一段）")
        if surge_on:
            self._log(f"段内超速熔断：开（0.7s 短窗速率 ≥ 本段规定速率×{surge_ratio:g}"
                      f"（" + "、".join(
                          f"{STAGE_TEXT[s.name]} {max(SURGE_ABS_MIN_MG_S, surge_ratio * s.rate_mg_s):.0f}"
                          for s in plan if s.name != "slow") + " mg/s）"
                      f"或 剩余 ≤ 速率×{SURGE_STOP_HORIZON_S:g}s 就停振等余料，"
                      f"最多 {SURGE_SETTLE_MAX_S:g}s；停振期间又落 ≥{SURGE_ADVANCE_MG:g}mg "
                      f"就判定本段撑不住、提前切下一段（不转缓补）；"
                      f"每次熔断顺带降 {SURGE_AMP_CUT_V:g}V，"
                      f"单次加粉最多 {SURGE_MAX_TIMES} 次）")
        else:
            self._log("段内超速熔断：关（塌料时只能等超重保护）")
        if self._trim_on:
            src = "页面指定" if trim_enabled is not None else "粉体档案"
            self._log(f"缓补：开（{src}；慢加剩 ≤{self._trim_enter_mg:.0f}mg 停连续振动，"
                      f"改为 每发固定 {TRIM_PULSE_S:.2f}s 的脉冲补料（出多少只看振幅），"
                      f"一发一发凑到目标）")
        else:
            self._log(f"缓补：关（慢加直接判停收尾；需要时在页面「缓补脉冲」勾选开启）")
        for spec in plan:
            if spec.switch_compensation_mg >= 0.5:
                self._log(f"{STAGE_TEXT[spec.name]}切换点按学到的惯性落料提前 "
                          f"{spec.switch_compensation_mg:g} mg（用该粉历史数据）")
        # 慢加段是不是被切点"甩了包袱"：收尾段正常只该占目标 5~15%。
        # 2026-10-23 15:46 那次加粉：目标改成 3000，但页面提交的还是 2000 那套切点
        # （快加 1450 / 粗加 1812），1812mg 就进慢加，剩 1186mg 全用 5mg/s 磨，
        # 既慢又容易在慢加里撞超时。这里显式报出来，别让人对着一屏日志猜。
        _sw = [float(s.switch_mass_mg) for s in plan
               if s.name != "slow" and s.switch_mass_mg is not None]
        if _sw:
            _rest = float(target) - max(_sw)
            if _rest > max(0.30 * float(target), 150.0):
                self._log(f"注意：慢加段要补 {_rest:.0f} mg（占目标 "
                          f"{_rest / max(1.0, float(target)) * 100:.0f}%）——收尾段正常只该 "
                          f"5~15%，三阶段切点可能没按新目标重算"
                          f"（页面点「按标定表重算」）")
        return self.status()

    def preview(self, target_mg: Any, *, powder: str | None = None,
                category: str | None = None, preset_id: str | None = None,
                amplitude_vpp: float | None = None, tail_mass_mg: float | None = None,
                use_feedforward: bool = True,
                stages: list[dict] | None = None,
                settle_gate: bool | None = None,
                settle_gate_rate_mg_s: float | None = None,
                taper_factor: float | None = None,
                surge_guard: bool | None = None,
                surge_ratio: float | None = None) -> dict:
        """只算计划、不动硬件：页面改目标质量/换粉时用它回填三阶段表格。"""
        r = self._resolve(target_mg, category=category, preset_id=preset_id,
                          amplitude_vpp=amplitude_vpp, tail_mass_mg=tail_mass_mg,
                          powder=powder, use_feedforward=use_feedforward, stages=stages,
                          log_warnings=False)
        return {
            "target_mg": r["target"],
            "powder": r["workspace"].name if r["workspace"] else None,
            "calibrated": bool(r["ff_ready"]),
            "category": r["category"],
            "frequency_hz": round(r["freq_hz"], 2),
            "duty_percent": round(r["duty_pct"], 2),
            "amplitude_vpp": round(r["amp_vpp"], 3),
            # "powder" = 用的是该粉标定时存的频率/占空比；"default" = 无标定信号，走默认
            "signal_source": r["signal_source"],
            "tail_mass_mg": r["tail"],
            "boundaries": r["boundaries"],
            "plan": [spec.to_dict() for spec in r["plan"]],
            "estimated_s": round(r["estimated_s"], 1),
            "max_run_s": round(r["max_run_s"], 1),
            "limits": {"target_min_mg": TARGET_MIN_MG, "target_max_mg": TARGET_MAX_MG},
            "calibration_points": r["points"],
            # 三阶段配方（档案 stage_recipe_v1）：操作者在页面上定下的三段工作点。
            # 纯透传给页面「按标定表重算」时回填三行振幅（用户 2026-10-27 13:20 定：
            # 只是把档案里的值填进表格，填完仍可手改，改了以页面那一行为准）。
            # 后端算计划依旧只认页面提交的 stages，这里**不参与任何控制**。
            "stage_recipe": ((r["workspace"].raw.get("stage_recipe_v1") or None)
                             if r["workspace"] is not None else None),
            # 沉降门参数原样回显，页面用它确认"这次按什么口径切段"
            "settle_gate": {
                "enabled": True if settle_gate is None else bool(settle_gate),
                # rate_mg_s = None 表示"按段比例门槛"（本段规定速率 × rate_ratio）
                "rate_mg_s": (None if settle_gate_rate_mg_s is None
                              else round(float(settle_gate_rate_mg_s), 1)),
                "rate_ratio": SETTLE_GATE_RATE_RATIO,
                "max_s": SETTLE_GATE_MAX_S,
                "taper_factor": round(TAIL_TAPER_FACTOR if taper_factor is None
                                      else float(taper_factor), 2),
            },
            # 段内超速熔断参数原样回显（2026-10-26 11:33）
            "surge_guard": {
                "enabled": (SURGE_GUARD_ON if surge_guard is None else bool(surge_guard)),
                "rate_ratio": round(SURGE_RATE_RATIO if surge_ratio is None
                                    else float(surge_ratio), 2),
                "abs_min_mg_s": SURGE_ABS_MIN_MG_S,
                "stop_horizon_s": SURGE_STOP_HORIZON_S,
                "max_times": SURGE_MAX_TIMES,
            },
        }

    def _resolve(self, target_mg: Any, *, category: str | None, preset_id: str | None,
                 amplitude_vpp: float | None, tail_mass_mg: float | None,
                 powder: str | None, use_feedforward: bool,
                 stages: list[dict] | None, log_warnings: bool = True) -> dict:
        """页面参数 + 标定表 + 算法预设 → 一份完整计划。不碰硬件，preview 也走这里。"""
        target = validate_target_mass(int(target_mg))

        workspace = self._load_workspace(powder) if powder else None
        signal = workspace.signal() if workspace else None
        ff_ready = bool(use_feedforward and workspace and workspace.feedforward)

        # 振动参数：频率 + 占空比**跟着粉走**。
        #   1) 该粉有标定 → 用标定表里存的 signal（那就是这套速率曲线的测量工况）；
        #   2) 没选粉/没标定/标定段缺 signal → 退回全局默认 80Hz / 50%。
        # 振幅本来就是按粉取的（标定振幅起步，再被闭环在 1–4V 内调），这里只补频率/占空比。
        freq_hz = VIB_FREQUENCY_HZ
        duty_pct = VIB_DUTY_PERCENT
        signal_source = "default"
        if signal:
            try:
                powder_freq = float(signal["frequency_hz"])
                powder_duty = float(signal["duty_percent"])
            except (KeyError, TypeError, ValueError):
                powder_freq = powder_duty = None
            if powder_freq is not None and powder_duty is not None:
                freq_hz, duty_pct, signal_source = powder_freq, powder_duty, "powder"
        if log_warnings:
            who = workspace.name if workspace else "未选粉"
            how = "按该粉标定振动参数" if signal_source == "powder" else "无标定信号，按默认"
            self._log(f"{how}下发：{who} → {freq_hz:g}Hz / {duty_pct:g}%")
        # 振幅是本次唯一的速率调节量；2026-10-27 起允许 0.01–4 Vpp
        # （下限从软件自定的 1.0V 改为设备物理下限 0.01V，用户 11:08 要求）。
        # 页面手填越界直接拒；标定表里的值超范围就钳进来并留一条日志（老标定不一定在这个区间）。
        raw_amp = (float(signal["amplitude_vpp"])
                   if signal and signal.get("amplitude_vpp") is not None else None)
        if amplitude_vpp is not None:
            amp_vpp = float(amplitude_vpp)
            if not AMP_MIN_VPP <= amp_vpp <= AMP_MAX_VPP:
                raise ValueError(
                    f"振幅 {amp_vpp:g} Vpp 超出允许范围 {AMP_MIN_VPP:g}–{AMP_MAX_VPP:g} Vpp")
        elif raw_amp is not None:
            amp_vpp = _clamp_amp(raw_amp)
            if log_warnings and abs(amp_vpp - raw_amp) > 1e-6:
                self._log(f"标定振幅 {raw_amp:g} Vpp 不在 {AMP_MIN_VPP:g}–{AMP_MAX_VPP:g} Vpp 内，"
                          f"按 {amp_vpp:g} Vpp 起步")
        else:
            amp_vpp = 2.0
        if tail_mass_mg is not None and not 0.0 <= float(tail_mass_mg) <= 50.0:
            raise ValueError("尾部余量必须在 0..50 mg")

        # 分类：页面没选就按粉体档案推断，再没有就用 Medium-flow
        category = (category or (workspace.category if workspace else None) or "Medium-flow")
        cls = powder_classification_params(category)
        seed_id = preset_id or cls["preset_id"]
        if seed_id not in DISPENSE_PRESETS:
            raise ValueError(f"未知种子档位 {seed_id!r}")
        seed = DISPENSE_PRESETS[seed_id]

        # 控制器内部频率限幅就是 10–80 Hz，这里先卡死，免得到了阶段切换才报参数错
        if not 10.0 <= freq_hz <= 80.0:
            raise ValueError(f"振动频率 {freq_hz:g} Hz 超出允许范围 10–80")
        if not 5.0 <= duty_pct <= 50.0:
            raise ValueError(f"占空比 {duty_pct:g}% 超出允许范围 5–50%")

        # 标定表如果混着几组振动参数，频率/占空比只能按"这张表的主体"下发
        # （section 级 signal，标定时最后一次写入的那组）。参数不同的那些点速率会偏，
        # 不拦（都是实测点，总比没有强），但必须说清是哪几个点，事后能对上账。
        if ff_ready and log_warnings:
            used = (workspace.feedforward or {}).get("signals_used") or []
            if len(used) > 1:
                odd = 0
                for u in used:
                    same = (abs(float(u.get("frequency_hz") or 0) - freq_hz) <= 0.05
                            and abs(float(u.get("duty_percent") or 0) - duty_pct) <= 0.05)
                    if not same:
                        odd += int(u.get("points") or 0)
                brief = "、".join(
                    f"{float(u.get('frequency_hz') or 0):g}Hz/"
                    f"{float(u.get('duty_percent') or 0):g}%/"
                    f"{float(u.get('amplitude_vpp') or 0):g}Vpp×{u.get('points')}点"
                    for u in used)
                self._log(f"注意：「{workspace.name}」的标定点混了 {len(used)} 组振动参数"
                          f"（{brief}）；本次按该粉生效的 {freq_hz:g}Hz / {duty_pct:g}% 下发，"
                          f"其中 {odd} 个点是在别的频率/占空比下测的，这些点的速率会偏"
                          f"（振幅差异不影响，闭环会自己调）")

        # 速率档：按目标质量插值，再用粉体分类里针对该目标的覆盖项优先
        profile = interpolate_profile(target)
        overrides = resolve_partial_target_overrides(cls.get("profile_overrides", {}), target)
        profile.update(overrides)

        tail = (cls["fixed_tail_mass_mg"] if tail_mass_mg is None else float(tail_mass_mg))
        stage_overrides = self._parse_stage_overrides(stages, target)
        plan, boundaries = self._build_plan(
            target, profile, cls, workspace, seed, duty_pct=duty_pct, amp_vpp=amp_vpp,
            overrides=stage_overrides)
        # 学到的切段惯性落料量：把切换点**提前**这么多，让"停振后还会落下来的粉"
        # 正好落在原定切换点上。只对后面还有段的阶段生效；限幅 50mg，脏数据带不偏。
        if ff_ready and workspace is not None:
            gains = workspace.settle_gains()
            applied: list[str] = []
            for idx in range(len(plan) - 1):
                key = f"{plan[idx].name}>{plan[idx + 1].name}"
                try:
                    gain = float((gains.get(key) or {}).get("gain_mg") or 0.0)
                except (TypeError, ValueError):
                    gain = 0.0
                if gain < 0.5 or plan[idx].switch_mass_mg is None:
                    continue
                gain = min(50.0, gain)
                before = float(plan[idx].switch_mass_mg)
                after = round(max(1.0, before - gain), 1)
                if after >= before:
                    continue
                plan[idx].switch_mass_mg = after
                plan[idx].switch_compensation_mg = round(before - after, 1)
                if boundaries.get(plan[idx].name) is not None:
                    boundaries[plan[idx].name] = after   # 页面显示的口径跟实际一致
                applied.append(f"{STAGE_TEXT[plan[idx].name]} {before:g}→{after:g}mg")
            if applied and log_warnings:
                self._log("切段点按该粉学到的惯性落料量提前：" + "、".join(applied))
        ff_fit = workspace.fit_controller_params() if ff_ready else {}

        # 目标速率超出该工作点实际能力时的交代。
        # 现在默认靶子（75/20/5）会被自动钳到"标定实测速率 ×1.15"，这里把钳制
        # 结果明明白白写进日志：操作员填的靶子没变，执行的是机器真实能给的值。
        if ff_ready and log_warnings:
            capped = [spec for spec in plan if spec.rate_capped_from is not None]
            for spec in capped:
                self._log(f"{STAGE_TEXT[spec.name]}目标速率 {spec.rate_capped_from:.1f} mg/s "
                          f"超过该工作点（搅 {spec.stir_rpm:.0f}rpm / 杆 {spec.rod_ff}）的"
                          f"实际能力，已按 {spec.rate_mg_s:.1f} mg/s 执行"
                          f"（标定预测 {spec.rod_ff_expected_rate} × "
                          f"{AMP_RATE_CAP_RATIO:g}；振幅再往上推也是这个流量）")
            trusted: list[float] = []
            for item in (workspace.feedforward.get("points") or []):
                try:
                    point_rate = float(item.get("rate_mg_s") or 0)
                    point_cv = float(item.get("cv_pct") or 0)
                except (TypeError, ValueError):
                    continue
                if point_rate > 0 and point_cv <= 40.0:   # 与 _stage_defaults 同一口径：CV>40 不可信
                    trusted.append(point_rate)
            top = max(trusted) if trusted else None
            if top:
                for spec in plan:
                    # 已经钳过的段不再重复警告；只有"手填速率硬超上限"才提醒
                    if spec.rate_capped_from is not None:
                        continue
                    if spec.rate_mg_s > top * 1.2:
                        self._log(f"注意：{STAGE_TEXT[spec.name]}目标速率 "
                                  f"{spec.rate_mg_s:.1f} mg/s 超过标定可信最大 {top:.1f} mg/s，"
                                  f"振幅会一直加到上限（标定+"
                                  f"{AMP_CTRL_RAISE_SPAN_V:g}V）也追不上"
                                  f"（到时日志会写\"加振幅已到极限\"）——"
                                  f"这是手填的值，想按能力执行就把速率留空")

        estimated = 0.0
        prev = 0.0
        for spec in plan:
            end = float(spec.switch_mass_mg) if spec.switch_mass_mg is not None else float(target)
            estimated += max(0.0, end - prev) / max(0.3, spec.rate_mg_s)
            prev = end
        # 超时上限跟着计划走：大目标用 180s 会一刀砍在半路；20000mg 慢速工况要 20 分钟以上。
        # 注意这只是**第一段（快加）期间生效的计划上限**——快加走完后 _loop 会把它
        # 放宽到 MAX_RUN_ABS_S 绝对兜底（2026-10-28 用户定，见常量注释）。
        max_run_s = min(7200.0, max(self.MAX_RUN_S, estimated * 2.2 + 60.0))

        points = []
        if ff_ready:
            for item in (workspace.feedforward.get("points") or []):
                try:
                    rate = float(item.get("rate_mg_s") or 0)
                except (TypeError, ValueError):
                    continue
                if rate <= 0:
                    continue
                try:
                    cv = float(item.get("cv_pct"))
                except (TypeError, ValueError):
                    cv = 0.0
                points.append({"stir_rpm": float(item.get("stir_rpm") or 0),
                               "rod_units": int(item.get("rod_units") or 0),
                               "rate_mg_s": round(rate, 2), "cv_pct": round(cv, 1)})
            points.sort(key=lambda p: p["rate_mg_s"], reverse=True)

        return {"target": target, "workspace": workspace, "ff_ready": ff_ready,
                "freq_hz": freq_hz, "duty_pct": duty_pct, "amp_vpp": amp_vpp,
                "signal_source": signal_source,
                "category": category, "cls": cls, "seed": seed, "seed_id": seed_id,
                "profile": profile, "tail": tail, "plan": plan, "boundaries": boundaries,
                "stage_overrides": stage_overrides, "ff_fit": ff_fit,
                "estimated_s": estimated, "max_run_s": max_run_s, "points": points}


    def stop(self, reason: str = "操作员停止") -> dict:
        """请求停止（线程在下一轮观测时退出并执行停机流程）。"""
        if not self.running:
            return {"stopped": False, "reason": "当前没有运行中的加粉任务"}
        self._internal_stop = reason
        self._stop_event.set()
        self._log(f"闭环加粉收到停止请求：{reason}")
        return {"stopped": True, "reason": reason}

    def abort(self, reason: str) -> None:
        """急停路径调用：只置位，不动硬件（硬件停止由 emergency_stop_all 负责）。"""
        if self.running:
            self._internal_stop = f"急停：{reason}"
            self._stop_event.set()

    # ------------------------------------------------------------------
    # 阶段计划
    # ------------------------------------------------------------------

    @staticmethod
    def _plan_brief(plan: list[StagePlan], target: int) -> str:
        """一句话描述分段计划（日志/结果用）。

        最后一段的 ``switch_mass_mg`` 是 None（跑到判停），格式化时要用目标值兜底——
        直接 ``:g`` 会抛 ``unsupported format string passed to NoneType.__format__``，
        2026-10-21 现场就踩过这个坑（启动直接失败）。
        """
        def end_of(spec: StagePlan) -> float:
            return float(target) if spec.switch_mass_mg is None else float(spec.switch_mass_mg)

        return " → ".join(
            f"{STAGE_TEXT[spec.name]}0~{end_of(spec):g}mg"
            f"({spec.rate_mg_s:.1f}mg/s,搅{spec.stir_rpm:.0f},杆{spec.rod_ff},"
            f"幅{spec.amplitude_vpp:.2f})"
            for spec in plan)

    @staticmethod
    def _load_workspace(powder: str) -> PowderWorkspace | None:
        try:
            return load_powder(powder)
        except ValueError:
            return None

    @classmethod
    def stage_boundaries(cls, target: float) -> dict:
        """三段的切换质量（mg）。值 None 表示这段在这个目标尺寸下不启用。

        target > 5000：快加 0→(目标−800)，迅速把大头加完；粗加 →(目标−100)；慢加收尾
        target 1000..5000：快加 0→75%；粗加 75%→95%（若 5% 已超过 100mg，改成到 目标−100）；
                      慢加收尾（最后 5% 或 100mg）
        300..1000    ：不启用快加；粗加 0→(目标−预留)，慢加收尾；预留 = 15% 夹在 50..150mg
        < 300        ：粗加 0→(目标−40)，保证慢加段至少还有 40mg 可以慢慢调
        """
        target = float(target)
        if target > cls.LARGE_TARGET_MIN_MG:
            # 大目标：快加只认"还剩多少"，不认百分比。20000mg 按 75% 切意味着粗加段
            # 要走 4000mg（约 5 分钟），而快加那 15000mg 本来就能用最快速率一口气冲完。
            fast_end = round(target - cls.LARGE_FAST_TAIL_MG, 1)
            coarse_end = round(min(max(target - 100.0, fast_end + 5.0), target - 5.0), 1)
            return {"fast": fast_end, "coarse": coarse_end, "slow": None}
        if target > cls.FAST_TARGET_MIN_MG:
            fast_end = round(target * 0.75, 1)
            five_pct = target * 0.05
            coarse_end = target - 100.0 if five_pct > 100.0 else target * 0.95
            coarse_end = round(min(max(coarse_end, fast_end + 5.0), target - 5.0), 1)
            return {"fast": fast_end, "coarse": coarse_end, "slow": None}
        if target < 300.0:
            reserve = cls.SLOW_MIN_RESERVE_MG
        else:
            reserve = min(cls.SLOW_MAX_RESERVE_MG, max(50.0, target * 0.15))
        return {"fast": None, "coarse": round(target - reserve, 1), "slow": None}

    @staticmethod
    def _parse_stage_overrides(stages: list[dict] | None, target: int) -> list[dict]:
        """页面传来的三段配置：只认白名单字段，越界一律拒绝（在任何硬件动作之前）。"""
        if not stages:
            return []
        if not isinstance(stages, list):
            raise ValueError("stages 必须是数组")
        fields = (("stir_rpm", 0.0, 1500.0, "搅拌转速 RPM"),
                  ("rod_units", float(ROD_MIN_UNITS), float(ROD_MAX_UNITS), "推杆开度"),
                  ("rate_mg_s", 0.1, 300.0, "目标速率 mg/s"),
                  ("amplitude_vpp", AMP_MIN_VPP, AMP_MAX_VPP, "振幅 Vpp"),
                  ("duty_percent", 5.0, 50.0, "占空比 %"),
                  ("settle_wait_s", 0.0, 5.0, "切段等待 s"),
                  ("switch_mass_mg", 1.0, float(target) - 1.0, "阶段切换质量 mg"))
        parsed: list[dict] = []
        for item in stages:
            if not isinstance(item, dict):
                continue
            name = str(item.get("name") or "").strip().lower()
            if name not in STAGE_ORDER:
                continue
            record: dict[str, Any] = {"name": name}
            if "enabled" in item:
                record["enabled"] = bool(item["enabled"])
            for key, low, high, label in fields:
                raw = item.get(key)
                if raw in (None, ""):
                    continue
                try:
                    value = float(raw)
                except (TypeError, ValueError):
                    raise ValueError(f"{STAGE_TEXT[name]}阶段的{label}不是数字：{raw!r}") from None
                if not low <= value <= high:
                    raise ValueError(f"{STAGE_TEXT[name]}阶段的{label} {value:g} 超出 {low:g}–{high:g}")
                record[key] = value
            parsed.append(record)
        return parsed

    def _stage_defaults(self, workspace: PowderWorkspace | None, profile: dict) -> dict:
        """页面没填时每段的默认配方。

        目标速率固定为 **快加 75 / 粗加 20 / 慢加 5 mg/s**（DEFAULT_STAGE_RATE_MG_S，
        2026-10-22 用户定）。这三个数只是"从标定表里挑工作点"的靶子：
        选定标定点后，**搅拌转速和推杆开度都照抄这个点**，一分不改。

        **推杆开度是硬约束（2026-10-22 用户明确要求）**：绝不允许用推杆调流量。
        速率只由振幅闭环负责（1–4 Vpp）。曾经试过拿 ``solve_rod`` 按靶子反解开度，
        结果 1000mg 封堵剂1 那次粗加用 691、慢加用 617（标定点本来是 600），
        推杆被当成调节变量动来动去 —— 已彻底回退，``_rod_band`` 不再做任何反解。
        """
        points: list[dict] = []
        if workspace and workspace.feedforward:
            for item in workspace.feedforward.get("points") or []:
                try:
                    rate = float(item.get("rate_mg_s") or 0)
                except (TypeError, ValueError):
                    continue
                if rate <= 0:
                    continue
                try:
                    cv = float(item.get("cv_pct"))
                except (TypeError, ValueError):
                    cv = 0.0
                points.append({"stir_rpm": float(item.get("stir_rpm") or 0),
                               "rod_units": int(item.get("rod_units") or 0),
                               "rate_mg_s": rate, "cv": cv})
            # 这里不再先把 CV 差的点删掉：删完只剩一两个点时，选出来的转速会离目标
            # 速率更远（实测就有过这种情况）。改为在 nearest() 里给它们加惩罚分，
            # 有好点就用好点，实在没有才退而用它们。

        def nearest(target: float) -> dict | None:
            """挑"实测速率最贴近靶子"的标定点。

            用对数比例算距离（2.3 vs 5 mg/s 和 30 vs 65 mg/s 都算差一倍，公平）；
            CV>40% 的点（塌料/天平跳数）加 STAGE_POINT_CV_PENALTY 的惩罚，
            只在实在没有好点时才用它们。
            选出的点**同时贡献搅拌转速和推杆开度**（开度照抄，不反解）。
            """
            if not points:
                return None

            def cost(point: dict) -> float:
                ratio = abs(math.log(max(1e-6, point["rate_mg_s"]) / max(1e-6, target)))
                return ratio + (STAGE_POINT_CV_PENALTY if point["cv"] > 40.0 else 0.0)

            return min(points, key=cost)

        defaults: dict[str, dict] = {}
        for name in STAGE_ORDER:
            target = float(DEFAULT_STAGE_RATE_MG_S[name])
            point = nearest(target)
            defaults[name] = {
                "stir_rpm": float(point["stir_rpm"]) if point else 0.0,
                # 推杆开度照抄标定点那一档（硬约束：推杆不是调节变量）
                "rod_units": int(point["rod_units"]) if point else None,
                "rate_mg_s": target,
                # 标定表里离靶子最近那点的实测速率，页面用来提示"这一段可能打不到"
                "matched_rate_mg_s": float(point["rate_mg_s"]) if point else None,
                "matched_cv_pct": float(point["cv"]) if point else None,
            }
        return defaults

    def _build_plan(self, target: int, profile: dict, cls: dict,
                    workspace: PowderWorkspace | None, seed: dict, *,
                    duty_pct: float, amp_vpp: float,
                    overrides: list[dict] | None = None) -> tuple[list[StagePlan], dict]:
        """按"离目标还有多远"切成三段，每段给出完整配方。返回 (plan, 边界表)。"""
        defaults = self._stage_defaults(workspace, profile)
        bounds = self.stage_boundaries(target)
        ov = {item["name"]: item for item in (overrides or [])}

        # 粉级加粉标记（档案 dispense_flags）：只影响慢加的振幅带半宽和"零速率脉冲"许可。
        # 2026-10-29 用户定：封堵剂2 = zero_rate_pulse:false + slow_amp_span_vpp:1.0
        # （不许在速率为 0 时踢脚脉冲；作为补偿，慢加振幅带从 ±0.5V 放宽到 ±1.0V）。
        flags = _powder_flags(workspace)
        zero_rate_pulse = flags.get("zero_rate_pulse")
        zero_rate_pulse = True if zero_rate_pulse is None else bool(zero_rate_pulse)
        try:
            slow_span = (float(flags["slow_amp_span_vpp"])
                         if flags.get("slow_amp_span_vpp") is not None else None)
        except (TypeError, ValueError):
            slow_span = None
        if slow_span is not None and not 0.0 < slow_span <= AMP_MAX_VPP:
            slow_span = None

        # 每段是否启用：默认由边界决定（边界 None = 这个目标尺寸不需要这段）；
        # 慢加是收尾段，不允许关掉（关掉就没人负责最后那一刀）。
        enabled = {name: (bounds.get(name) is not None or name == "slow") for name in STAGE_ORDER}
        for name, item in ov.items():
            if "enabled" in item:
                enabled[name] = bool(item["enabled"])
        enabled["slow"] = True
        if not any(enabled.values()):
            enabled["coarse"] = True

        duty_seed = max(1000, min(5000, int(round(duty_pct * 100))))
        last_enabled = max(index for index, name in enumerate(STAGE_ORDER) if enabled[name])

        plan: list[StagePlan] = []
        prev_switch = 0.0
        for index, name in enumerate(STAGE_ORDER):
            if not enabled[name]:
                continue
            base = defaults[name]
            cfg = ov.get(name, {})
            stir = float(cfg.get("stir_rpm", base["stir_rpm"]) or 0.0)
            rod_override = cfg.get("rod_units", base["rod_units"])
            rate = max(0.2, min(300.0, float(cfg.get("rate_mg_s", base["rate_mg_s"])
                                            or base["rate_mg_s"])))
            # 占空比全段统一（2026-10-21 用户定 22%）；只有页面显式手填才覆盖。
            # 以前这里在"目标速率低于标定最小流量"时按比例压占空比，现在不再压——
            # 低速段靠振幅（1–4V）收，占空比保持一个固定值，信号源参数才不会来回跳。
            stage_duty = duty_seed
            if "duty_percent" in cfg:
                stage_duty = max(1000, min(5000, int(round(float(cfg["duty_percent"]) * 100))))

            if index == last_enabled:
                switch = None
            else:
                switch = bounds.get(name)
                if switch is None:
                    # 小目标也硬开快加这种情况：借后面第一个有边界的段
                    switch = next((bounds[n] for n in STAGE_ORDER[index + 1:]
                                   if enabled[n] and bounds.get(n) is not None), None)
                switch = float(target) * 0.9 if switch is None else float(switch)
                if "switch_mass_mg" in cfg:
                    switch = float(cfg["switch_mass_mg"])
                switch = max(prev_switch + 1.0, min(float(target) - 1.0, switch))
            prev_switch = float(target) if switch is None else switch

            mode = STAGE_MODE[name]
            span = 90 if name == "fast" else (50 if name == "coarse" else 25)
            rod_ff, rod_lo, rod_hi, predicted, achievable = self._rod_band(
                workspace, rate, stir, seed, rod_override=rod_override, span=span)
            # 目标速率钳到"这个工作点的实际能力"（标定实测速率 × 1.15）。
            # 默认靶子 75 而 200rpm/700 标定只有 49.1 时，硬追的结果是振幅从 2.0
            # 一路推到 3.2、白花 25 秒，实测 3.2V 出 43~51 —— 和 2V 的 49.1 没区别，
            # 说明该工况振幅早就饱和了，再推只是让粉流更不匀。
            # 只在"页面没手填速率"时钳：用户手填就是明确要试，尊重他的输入。
            # 只钳快加段：粗加/慢加的靶子本来就贴着标定值，而且慢加用的是
            # "剩余量/18s"的动态目标——把它钳到标定值×1.15 反而会让收尾更慢
            # （封堵剂1 慢加 5 mg/s 被钳成 2.43，100mg 尾巴要多走 20 秒）。
            cap_from = None
            if predicted and name == "fast" and "rate_mg_s" not in cfg:
                capacity = float(predicted) * AMP_RATE_CAP_RATIO
                if rate > capacity:
                    cap_from = rate
                    rate = max(0.2, capacity)
                    achievable = True        # 已在能力范围内执行，不算"够不上"
            duty_lo, duty_hi = self._duty_band(mode, stage_duty)
            # 振幅基准：页面手填了就用填的；没填就用本段默认（快加 3.0 / 粗加 2.5 / 慢加 2.0）。
            # 不再回落到"粉体标定振幅"——用户 2026-10-23 18:36 定的是按段给固定基准，
            # 标定工况（2.0~2.5V）只用来提供转速/开度工作点。
            amp_explicit = "amplitude_vpp" in cfg
            amp = _clamp_amp(float(cfg["amplitude_vpp"]) if amp_explicit
                             else STAGE_DEFAULT_AMP_VPP.get(name, amp_vpp))
            # 切段等待：留空 = 用全局硬上限（SETTLE_GATE_MAX_S），显式 0 = 这一段切出时不停振等
            settle_wait = cfg.get("settle_wait_s")
            plan.append(StagePlan(
                name=name, mode=mode, rate_mg_s=rate, stir_rpm=stir, switch_mass_mg=switch,
                rod_ff=rod_ff, rod_min=rod_lo, rod_max=rod_hi,
                duty_seed_permyriad=stage_duty, duty_min=duty_lo, duty_max=duty_hi,
                # 推杆整段锁死：速率只由振幅调（用户 2026-10-21 要求，别乱动推杆）
                position_feedback=False,
                amplitude_vpp=amp, amp_explicit=amp_explicit,
                amp_span_vpp=(slow_span if name == "slow" else None),
                zero_rate_pulse=zero_rate_pulse,
                settle_wait_s=None if settle_wait is None else float(settle_wait),
                control_note=STAGE_NOTE[name],
                rod_ff_expected_rate=predicted, rod_ff_achievable=achievable,
                matched_rate_mg_s=base.get("matched_rate_mg_s"),
                rate_capped_from=cap_from))
        return plan, bounds

    @staticmethod
    def _duty_band(mode: str, duty_seed: int) -> tuple[int, int]:
        """占空比允许的调整范围：快加允许大幅上抬（追流量），慢加只留很窄的微调带。

        下限 1000（10%）是控制器硬约束，也是信号源还能稳定起振的最小占空比。
        """
        if mode == "coarse":
            return max(1000, duty_seed - 500), min(5000, duty_seed + 900)
        if mode == "fine":
            return max(1000, duty_seed - 700), min(5000, duty_seed + 500)
        return max(1000, duty_seed - 300), min(5000, duty_seed + 300)

    @staticmethod
    def _rod_band(workspace: PowderWorkspace | None, rate: float, stir: float,
                  seed: dict, *, rod_override: int | None = None,
                  span: int = 80) -> tuple[int, int, int, float | None, bool]:
        """推杆开度（**硬约束：整段锁死，绝不用推杆调流量**）。

        开度只有一个来源：
        1) ``rod_override``（页面手填 → 或标定点照抄的 ``rod_units``）；
        2) 都没有才用种子开度（无标定的兜底）。
        **不做任何反解**：以前这里会用 ``solve_rod(目标速率, 转速)`` 算开度，
        靶子一变开度就跟着变（1000mg 封堵剂1 那次粗加 691 / 慢加 617），
        等于把推杆当调节变量——2026-10-22 用户明确禁止，已删除该分支。

        返回值里的 ``rod_lo/rod_hi`` 只作为控制器 settings 的名义窗口（不执行），
        运行时不会去动推杆；``achievable`` 表示"按标定表这个工作点的实测速率
        是否够得上目标速率"，够不上就让页面提示"这一段只能靠振幅拉"。
        """
        if rod_override is not None:
            rod = int(max(ROD_MIN_UNITS, min(ROD_MAX_UNITS, int(rod_override))))
        else:
            rod = int(max(ROD_MIN_UNITS, min(ROD_MAX_UNITS,
                                             int(seed["window_position_units"]))))
        predicted = None
        if workspace and workspace.feedforward:
            predicted = workspace.rate_at(stir, rod)
        # 标定点实测速率不到目标的 80%：标定表这个工作点撑不起目标流量，
        # 页面会显示"标定最近仅 X mg/s"，实际只能靠振幅（最多 4V）往上顶。
        achievable = bool(predicted is not None and predicted >= max(0.2, rate * 0.8))
        return (rod, max(ROD_MIN_UNITS, rod - span),
                min(ROD_MAX_UNITS, rod + span), predicted, achievable)

    def _stage_settings(self, spec: StagePlan, cls: dict, target: int,
                        tail: float, ff_fit: dict) -> FeedbackControllerSettings:
        """每段一套控制器参数——这就是"不同阶段用不同控制方式"的落点。

        按 ``spec.mode``（段名 fast/coarse/slow 映射到控制器分支 coarse/fine/precision）：
        - coarse（快加）：内部 coarse 阈值拉到 0.98×目标，整段一直用快加速率；
        - fine（粗加）：内部阈值压到 0.01（永远进 fine 分支）；
        - precision（慢加）：精修预留量设成 2×目标，强制走 precision 分支并开启预测停车。

        注意：**控制器的 duty / window 输出不执行**（速率由振幅环负责）。
        它在这里的职责只剩三件：预测停车判定、超重兜底、堵料检测。
        因此推杆反馈一律关闭，限速阈值也放宽——否则它会因为"实测速率超了 15%"
        反复进入 emergency 分支，虽然不改设备，但会把内部状态搅乱。
        """
        duty_step = int(ff_fit.get("duty_step_per_update", cls["duty_step_per_update"]))
        emerg = max(duty_step, int(ff_fit.get("emergency_duty_step_per_update",
                                              cls["emergency_duty_step_per_update"])))
        pos_step = int(ff_fit.get("position_step_units", 25))

        if spec.mode == "coarse":
            fractions = (0.97, 0.98, 0.99)
            precision_reserve = 1.0
            duty_step = max(duty_step, 40)
            emerg = max(emerg, duty_step)
            pos_step = max(pos_step, 60)
            duty_interval = 0.4
            pos_interval = 1.5
        elif spec.mode == "fine":
            fractions = (0.01, 0.02, 0.03)
            switch = float(spec.switch_mass_mg) if spec.switch_mass_mg is not None else float(target)
            precision_reserve = max(1.0, float(target) - switch)
            duty_step = max(8, min(duty_step, 40))
            emerg = max(duty_step, min(emerg, 120))
            pos_step = max(8, min(pos_step, 35))
            duty_interval = 0.5
            pos_interval = 2.0
        else:
            fractions = (0.01, 0.02, 0.03)
            precision_reserve = float(target) * 2.0     # 永远落在 precision 分支
            duty_step = max(5, min(duty_step, 12))
            emerg = max(duty_step, min(emerg, 40))
            pos_step = max(5, min(pos_step, 10))
            duty_interval = 0.5
            pos_interval = 4.0

        return FeedbackControllerSettings(
            low_coarse_fraction=fractions[0],
            default_coarse_fraction=fractions[1],
            high_coarse_fraction=fractions[2],
            coarse_rate_mg_s=spec.rate_mg_s,
            fine_rate_mg_s=spec.rate_mg_s,
            precision_rate_mg_s=spec.rate_mg_s,
            min_duty_permyriad=spec.duty_min,
            max_duty_permyriad=spec.duty_max,
            min_window_position_units=spec.rod_min,
            max_window_position_units=spec.rod_max,
            # 校验要求这个上限落在 [min_window, max_window] 之间
            stall_recovery_max_window_position_units=min(
                spec.rod_max, max(spec.rod_min, 400)),
            duty_step_per_update=duty_step,
            emergency_duty_step_per_update=emerg,
            position_step_units=pos_step,
            position_update_interval_s=pos_interval,
            duty_update_interval_s=duty_interval,
            position_feedback_enabled=False,      # 推杆整段锁死，速率只由振幅调
            maximum_flow_rate_mg_s=max(1.0, spec.rate_mg_s * 3.0),
            fixed_tail_mass_mg=tail,
            allowed_overweight_mg=10.0,          # 对应产品规范的 +10mg
            allowed_underweight_mg=0.0,
            precision_start_remaining_mg=precision_reserve,
            force_stop_max_mass_mg=target + cls["force_stop_offset_mg"],
        )

    # ------------------------------------------------------------------
    # 线程主体
    # ------------------------------------------------------------------

    def _thread_main(self, plan: list[StagePlan], target: int,
                     cls: dict, tail: float, ff_fit: dict) -> None:
        reason = "运行结束"
        try:
            states = self._prepare(plan)
            reason = self._loop(plan, states, target, cls, tail, ff_fit)
            self._settle_and_finish(self._baseline, self._rod_start, reason, ok=True)
        except _Abort as abort:
            reason = abort.reason
            self._safe_shutdown_actuators()
            try:
                self._settle_and_finish(self._baseline, self._rod_start, reason, ok=False)
            except Exception as error:      # noqa: BLE001 - 收尾失败也要落结果
                self._log(f"加粉收尾失败：{error}")
                with self._lock:
                    self._state = "aborted"
                    self._detail = f"{reason}（收尾时又出错：{error}）"
        except Exception as error:          # noqa: BLE001 - 线程不能带异常死掉
            reason = f"异常：{error}"
            self._safe_shutdown_actuators()
            with self._lock:
                self._state = "failed"
                self._detail = reason
            self._log(f"闭环加粉异常终止：{error}")
            self._log(traceback.format_exc())   # 堆栈进日志，事后能定位到行

    def _prepare(self, plan: list[StagePlan]) -> dict:
        """起振前的准备：基线 → 振动参数 → 搅拌爬速 → 推杆开到位。"""
        self._t0 = time.monotonic()
        self._set_state("preparing", "读取起始重量基线")
        self._baseline = self._read_baseline()          # 3 次取中位数，纯读不写
        with self._lock:
            self._has_baseline = True
        self._rod_start = int(self._config["rod_start"])

        self._set_state("preparing", "设置振动参数（频率/占空比/振幅）")
        first = plan[0]
        # 先退猝发待命（WO0s+Wc0s）：上一次加粉缓补若用过真脉冲，设备还停在
        # "手动触发模式"——输出通道开着但只认 WP1s 触发，不退出来的话这一次加粉
        # 设了方波也不会连续震动（2026-10-22 18:23 用户指出的坑）。
        try:
            self.hub.sig.burst_off()
        except Exception as error:            # noqa: BLE001 - 没开过猝发也不该挡路
            self._log(f"退猝发待命失败（继续，按连续输出走）：{error}")
        self._sig_write(lambda: (
            self.hub.sig.set_waveform(self.WAVEFORM_SQUARE),
            self.hub.sig.set_frequency(float(self._config["frequency_hz"])),
            self.hub.sig.set_duty(float(first.duty_seed_permyriad) / 100.0),
            self.hub.sig.set_amplitude(float(first.amplitude_vpp)),
        ))
        self._applied_freq = int(round(self._config["frequency_hz"]))
        self._applied_duty = first.duty_seed_permyriad
        self._applied_window = first.rod_ff
        self._applied_amp = _clamp_amp(first.amplitude_vpp)
        self._amp_last_write_s = 0.0
        self._amp_wait_s = AMP_CTRL_MIN_EVAL_S
        self._amp_last_dir = None
        self._amp_last_eval_s = 0.0
        self._amp_probe = None
        self._amp_band = _amp_band_for(first)
        self._amp_lock_reason = ""
        self._amp_kick = None
        self._amp_kick_tries = 0
        self._amp_kick_last_amp = 0.0
        self._amp_kick_m0_total = 0.0
        self._mass_hist = []
        # 第一段的振幅统计（后面切段由 _enter_stage 重置）
        self._amp_changes = 0
        self._amp_saturated = False
        self._amp_min = self._applied_amp
        self._amp_max = self._applied_amp
        self._amp_heartbeat_s = 0.0
        self._stage_start_s = 0.0
        self._stage_start_mass = 0.0
        # ★ 2026-10-27 13:35 修 bug：**所有"以本次加粉 elapsed 为锚"的时间戳必须在开始加粉时复位**。
        # 事故（13:29 那次加粉，失水剂2 2000mg）：快加爬到 2.50V 带顶后零落料，
        # 本该在带顶满 4s、近 4s 速率 <1mg/s 时踢一脚，结果每拍都只写
        # "救济冷却中（等 τ+T = 12s）"，直到操作员 70s 手动停，一脚都没踢出去。
        # 根因：`_stall_wait_s` 只在 __init__ 和 `_enter_stage` 里复位，**第一段不经过
        # `_enter_stage`**（走 `_prepare`），于是它带着上一次加粉的踢脚时刻（13:24 那次加粉
        # 最后一脚在 189s）进了新的一次加粉；新的一次加粉 elapsed 才 41s → `elapsed - _stall_wait_s`
        # 是个负数，永远 < τ+T → 冷却判定恒成立，救济被永久锁死。
        # `_amp_at_cap_since` 同理（陈旧值会让"带顶满 4s"这条也永远不成立）。
        self._stall_wait_s = -99.0       # 卡粉救济上一次动作时刻（-99 = 本次加粉没踢过）
        self._amp_at_cap_since = None    # 本轮"到带顶"的起始时刻
        self._kick_count = 0             # 本次加粉累计踢脚次数（原来跨次加粉累加，CSV 会串账）
        self._stall_abort_s = 0.0        # 断料兜底参考时刻
        self._stall_abort_ref = 0.0      # 断料兜底参考质量
        self._last_drop_s = None         # 上一次天平读数变化的时刻
        self._ramp_last_s = -99.0
        self._prev_eval_rate = None
        self._v7_prev_rate = None
        self._v7_prev_t = None
        self._v7_a_est = None
        self._amp_write_marks = []       # 写幅时刻队列（τ 在线估计用）

        if first.stir_rpm > 0.5:
            self._set_state("preparing", f"搅拌起转到 {first.stir_rpm:.0f} RPM（防搭桥）")
            self._set_stir(first.stir_rpm, wait=True)
        else:
            self._applied_stir = 0.0

        self._set_state("preparing", f"推杆开到前馈开度 {first.rod_ff}")
        self.hub.rod_move(int(first.rod_ff))
        self._wait_rod(int(first.rod_ff))
        # 推杆硬约束：整段锁死；慢加段不追速率（用户 2026-10-22 明确要求）
        self._log(f"推杆锁定 {first.rod_ff}（来自标定点/页面手填，整段不动）；"
                  f"振幅 {float(first.amplitude_vpp):.2f}V，"
                  + (f"慢加只升不降（零落料 +{AMP_SLOW_CLIMB_STEP_V:g}V/"
                     f"{AMP_SLOW_CLIMB_INTERVAL_S:.0f}s）、有出粉绝对不动"
                     if first.name == "slow"
                     else f"速率闭环调幅（基准 −{AMP_RAMP_BAND_V:g}V ~ "
                          f"+{AMP_CTRL_RAISE_SPAN_V:g}V，单步 "
                          f"{AMP_CTRL_STEP_MIN_V:g}–{AMP_CTRL_UP_MAX_V:g}V；"
                          f"步子按缺口÷增益算，"
                          + ("顶到带顶仍不出就踢一脚）" if first.zero_rate_pulse
                             else "该粉不许零速率脉冲，顶到带顶不踢脚）")))

        if self._stop_event.is_set():
            raise _Abort(self._internal_stop)
        self._t0 = time.monotonic()
        self._start_stage_log(first, 0.0, 0.0)
        self._set_state("running",
                        f"起振：{self._config['frequency_hz']:g}Hz / "
                        f"{first.duty_seed_permyriad / 100:.1f}% 占空比 / "
                        f"振幅 {self._applied_amp:.2f}Vpp，"
                        f"搅拌 {first.stir_rpm:.0f} RPM")
        self._open_vibration(f"起振：{STAGE_TEXT[first.name]}，"
                             f"振幅 {self._applied_amp:.2f}Vpp / "
                             f"搅拌 {first.stir_rpm:.0f} RPM")
        return {"stage_index": 0}

    def _loop(self, plan: list[StagePlan], states: dict, target: int,
              cls: dict, tail: float, ff_fit: dict) -> str:
        """主闭环：天平 → 观测器 → 振幅环（调速率）+ 控制器（只做停车判定）。"""
        estimator = MassMotionEstimator()
        stage_index = int(states["stage_index"])
        spec = plan[stage_index]
        settings = self._stage_settings(spec, cls, target, tail, ff_fit)
        controller = ContinuousFeedbackController(
            target_mass_mg=float(target),
            initial=FeedbackInitialParameters(
                frequency_hz=self._applied_freq,
                duty_permyriad=max(settings.min_duty_permyriad,
                                   min(settings.max_duty_permyriad, self._applied_duty)),
                window_position_units=max(settings.min_window_position_units,
                                          min(settings.max_window_position_units,
                                              self._applied_window))),
            settings=settings)

        fails = 0
        last_log = 0.0
        last_curve = 0.0
        cap_released = False   # 快加走完后解除计划估算上限（只留绝对兜底）
        while not self._stop_event.is_set():
            elapsed = time.monotonic() - self._t0
            if elapsed > self._max_run_s:
                if stage_index >= 1 and not cap_released:
                    # 第一段（快加）已经走完：计划估算的时长不再可信（断流磨蹭很正常），
                    # 解除计划上限，只保留 MAX_RUN_ABS_S 绝对兜底防料斗真空转。
                    cap_released = True
                    planned = self._max_run_s
                    self._max_run_s = self.MAX_RUN_ABS_S
                    self._log(f"快加已完成，计划估算超时上限 {planned:.0f}s 解除"
                              f"（放宽到绝对兜底 {self._max_run_s:.0f}s）")
                if elapsed > self._max_run_s:
                    raise _Abort(f"超时保护：已运行 {elapsed:.0f}s（上限 {self._max_run_s:.0f}s）")
            try:
                raw = self._read_balance_raw()
                fails = 0
            except Exception as error:      # noqa: BLE001 - 读不到就是闭环失效
                fails += 1
                if fails >= self.MAX_BALANCE_FAILS:
                    raise _Abort(f"天平失联（连续 {fails} 次读不到：{error}）") from error
                time.sleep(0.05)
                continue

            mass = float(raw) - self._baseline
            self._last_raw = int(raw)

            # ── 阶段切换：达标就换配方（换搅拌 / 换前馈开度 / 换控制器参数）──
            # 三个触发同级：质量到切点 / 预测切点 / **熔断判"这一段撑不住"**。
            # 熔断只负责叫停本段（停振、记事件），切段本身走下面同一套
            # 关段账 → 沉降门 → 进段配方（2026-10-26 18:35 用户："熔断只发生在阶段
            # 切换之间就行了"），所以剩余量永远由下一段接着加，不会跳进缓补。
            force_cut = bool(getattr(self, "_surge_advance", False))
            if force_cut and (stage_index >= len(plan) - 1 or spec.switch_mass_mg is None):
                self._surge_advance = False      # 已经在最后一段，没有下一段可切
                force_cut = False
                # 熔断那边把振动关了，这里必须重新起振——否则最后一段一粒粉都不出，
                # 30s 后会被"断料中止"顶掉（那是自己给自己造事故）。
                if not self._vib_running:
                    self._open_vibration(
                        f"熔断后已是最后一段（{STAGE_TEXT[spec.name]}），没有下一段可切"
                        f"——重新起振接着加（剩 {target - mass:.0f}mg）")
            # **预测切段**（2026-10-26 11:05 用户）：质量还没到切点，但"质量 + 速率×1s"
            # 已经到了，就现在停振——让余料正好落在切点上，而不是越过切点再补一刀。
            cut = {"cut": False, "rate_mg_s": 0.0, "lead_mg": 0.0, "switch_mg": None}
            if (stage_index < len(plan) - 1
                    and spec.switch_mass_mg is not None
                    and mass < spec.switch_mass_mg):
                cut = self._predictive_cut(spec, mass)
            if (stage_index < len(plan) - 1
                    and spec.switch_mass_mg is not None
                    and (mass >= spec.switch_mass_mg or cut["cut"] or force_cut)):
                if force_cut:
                    self._surge_advance = False
                    self._log(f"熔断后提前切段：{STAGE_TEXT[spec.name]} 到 {mass:.1f}mg 就收工"
                              f"（剩 {target - mass:.0f}mg 不再用本段速率硬加），交给"
                              f"{STAGE_TEXT[plan[stage_index + 1].name]}接着加")
                if cut["cut"]:
                    self._log(f"预测切段：{STAGE_TEXT[spec.name]} {mass:.1f}mg + 尾量预测 "
                              f"{cut['lead_mg']:g}mg（速率 {cut['rate_mg_s']:.1f}mg/s ×"
                              f"{SETTLE_GATE_LEAD_S:g}s）= "
                              f"{mass + cut['lead_mg']:.1f}mg ≥ 切点 "
                              f"{float(spec.switch_mass_mg):.0f}mg，提前停振等余料落满")
                self._close_stage_log(mass, elapsed)
                # 沉降门：这一段的料还没落完就换配方，余料会直接顶进下一段
                landed = self._settle_gate(spec, plan[stage_index + 1], mass, elapsed,
                                           force=bool(cut["cut"]))
                if landed["applied"]:
                    mass = landed["mass"]
                    elapsed = landed["elapsed"]
                    self._note_settle(landed)
                # 沉降期间操作员按了停止：就此收手，别再去开下一段的推杆
                if self._stop_event.is_set():
                    raise _Abort(getattr(self, "_internal_stop", "操作员停止"))
                stage_index += 1
                spec = plan[stage_index]
                # ★ 2026-10-23 18:59（用户："沉降门的时候不更新质量，导致很多次最后
                # 粗加加过头了"）：沉降期间秤一直在涨，等判定结束时余料很可能已经把
                # **下一段的切点也顶穿**。旧代码只前进一段就开下一段配方（推杆+基准
                # 振幅全开），粗加一开场就已经在切点之上，只能靠慢加倒着收 → 偏重。
                # 现在：越过几段就跳几段，落到验收带内就直接收尾，不再开新段。
                while (stage_index < len(plan) - 1
                       and spec.switch_mass_mg is not None
                       and mass >= spec.switch_mass_mg):
                    self._log(f"沉降余料越界：{mass:.1f}mg 已到达"
                              f"{STAGE_TEXT[spec.name]}切点 "
                              f"{float(spec.switch_mass_mg):.0f}mg，跳过该段"
                              f"（不再开它的推杆/振幅）")
                    stage_index += 1
                    spec = plan[stage_index]
                accept_lo, _accept_hi = acceptance_limits(target)
                if mass >= accept_lo:
                    self._close_stage_log(mass, elapsed)
                    return (f"沉降余料已到 {mass:.1f}mg（验收下限 {accept_lo:.0f}mg），"
                            f"不再开{STAGE_TEXT[spec.name]}，直接收尾称重")
                self._enter_stage(spec, mass, elapsed)
                settings = self._stage_settings(spec, cls, target, tail, ff_fit)
                controller = ContinuousFeedbackController(
                    target_mass_mg=float(target),
                    initial=FeedbackInitialParameters(
                        frequency_hz=self._applied_freq,
                        duty_permyriad=max(settings.min_duty_permyriad,
                                           min(settings.max_duty_permyriad, self._applied_duty)),
                        window_position_units=max(settings.min_window_position_units,
                                                  min(settings.max_window_position_units,
                                                      self._applied_window))),
                    settings=settings)
                estimator.reset()
                self._taper_done_index = -1      # 新段重新允许"临到切点提前降幅"
                self._taper_hold = False
                self._taper_hold_since = 0.0
                self._taper_prev_amp = None
                self._start_stage_log(spec, mass, elapsed)

            # ★ 段内超速熔断（2026-10-26 11:33）：塌料 0.5s 就能落 100mg，绝不能等
            # 6s 回归窗/等缓补开门，先停振让余料落干净。它只处理"异常快"，正常加粉不触发。
            surge = self._surge_brake(spec, mass, elapsed)
            if surge["applied"]:
                mass = surge["mass"]
                elapsed = surge["elapsed"]
                if surge["finish"]:
                    self._close_stage_log(mass, elapsed)
                    return surge["reason"]
                if surge["advance"]:
                    # 塌得太猛：本段到此为止。振动已经关了，下一拍主循环顶部的切段
                    # 流程会接管（沉降门 + 进下一段），不再往缓补跳。
                    self._surge_advance = True
                    # 立刻进入下一拍去切段：这一拍剩下的控制器/调幅/判停全都基于
                    # "振动已经停了"的旧数据，跑了只会用旧段的振幅再算一遍。
                    continue

            motion = estimator.update(elapsed, mass)
            observation = FeedbackObservation(
                mass_mg=mass,
                predicted_mass_mg=motion.predicted_mass_mg,
                rate_mg_s=motion.rate_mg_s,
                predicted_rate_mg_s=motion.predicted_rate_mg_s,
                acceleration_mg_s2=motion.acceleration_mg_s2,
                jerk_mg_s3=motion.jerk_mg_s3,
                valid=motion.valid,
            )
            decision = controller.step(observation, time.monotonic())

            # 兜底：不依赖控制器预测，质量越过 目标+30mg 立刻停
            if mass > target + self.OVERSHOOT_ABORT_MG:
                raise _Abort(f"超重保护：{mass:.0f} mg 已超过 目标+{self.OVERSHOOT_ABORT_MG:.0f}mg")

            # 兜底：振幅/踢脚都已经到顶还一直不出料 → 别再空转等超时（2026-10-26 11:17）
            if self._amp_saturated:
                if (mass >= float(self._stall_abort_ref) + STALL_ABORT_GAIN_MG
                        or elapsed < float(self._stall_abort_s)):
                    self._stall_abort_ref = mass
                    self._stall_abort_s = elapsed
                elif elapsed - float(self._stall_abort_s) >= STALL_ABORT_S:
                    raise _Abort(
                        f"断料中止：振幅已到顶、踢脚也到顶，连续 {STALL_ABORT_S:.0f}s "
                        f"净重没涨（停在 {mass:.0f}/{target}mg）——请检查料斗是否缺料/"
                        f"搭桥、推杆开度是否够")
            else:
                self._stall_abort_ref = mass
                self._stall_abort_s = elapsed

            # ── 缓补：慢加段剩余量进入最后 20~40mg 区间（开了 trim_pulse 的粉）──
            # 停掉连续振动，改为 ≈10mg/发的脉冲补料，一发一发凑到目标。
            if (spec.name == "slow" and self._trim_on
                    and 0.0 < target - mass <= self._trim_enter_mg):
                self._close_stage_log(mass, elapsed)
                return self._trim_pulse_loop(mass, elapsed, target)

            # 速率只由振幅调：控制器的 duty / 推杆决议一律不执行（它只管停车判定）
            self._maybe_taper(spec, plan, stage_index, mass, elapsed)
            self._step_amplitude(spec, mass, elapsed)
            self._record(elapsed, mass, motion.rate_mg_s, spec.name)
            if elapsed - last_curve >= 0.5:
                last_curve = elapsed
                with self._lock:
                    self._curve.append([round(elapsed, 2), round(mass, 1),
                                        round(motion.rate_mg_s, 2)])
            if elapsed - last_log >= 1.0:
                last_log = elapsed
                # 降幅窗口期间要说清楚"为什么振幅不动"（2026-10-26 11:17 事故：
                # 面板只显示 +0.0mg/s、幅 2.00V，看不出是在等余料还是在卡死）
                hold_note = ("｜切点前降幅窗口：暂停调幅等余料"
                             if self._taper_hold else "")
                self._set_state("running",
                                f"{STAGE_TEXT[spec.name]} {mass:.0f}/{target} mg"
                                f"（{motion.rate_mg_s:+.1f} mg/s，"
                                f"幅 {self._applied_amp:.2f}Vpp，"
                                f"杆 {self._applied_window}，搅 {self._applied_stir:.0f}rpm，"
                                f"占空 {self._applied_duty / 100:.1f}%{hold_note}）")
            if decision.stop_requested:
                self._close_stage_log(mass, elapsed)
                return decision.reason or "控制器判定停止"
            time.sleep(self.SAMPLE_INTERVAL_S)
        self._close_stage_log(self._live.get("mass_mg", 0.0), time.monotonic() - self._t0)
        # 停止请求走"中止"路径：这一次加粉没有跑到控制器判定，结果必须标成未成功
        raise _Abort(getattr(self, "_internal_stop", "操作员停止"))

    # ------------------------------------------------------------------
    # 阶段动作
    # ------------------------------------------------------------------

    def _enter_stage(self, spec: StagePlan, mass: float, elapsed: float) -> None:
        """进入新阶段：**先关闸** → 换搅拌 → 调振幅 → 写占空比 → 开推杆 → **最后开闸**。

        顺序刻意如此。前半句是物理：搅拌决定"料松不松"，振幅决定"振动多强"，
        推杆决定"口开多大"——先把料搅松、振动调到该段的强度，再张口，流量上升是
        平滑的台阶；反过来张口瞬间可能先掉一团结块。
        后半句是时序（2026-10-29 16:14 用户指出"切换粗加的时候，是先振动，再更改
        幅度"）：沉降门不一定会停振（速率低于门槛、或上一段太短测不出速率都会直接
        跳过），所以进段第一件事是确认输出已关，参数全部写完再开——**绝不允许
        "边出料边改幅度"**，那等于拿上一段的振幅先甩一坨料。
        """
        self._log(f"进入{STAGE_TEXT[spec.name]}：{mass:.1f} mg / 目标 "
                  f"{self._config['target_mg']} mg，目标流量 {spec.rate_mg_s:.1f} mg/s，"
                  f"搅拌 {spec.stir_rpm:.0f} RPM，振幅 {spec.amplitude_vpp:.2f} Vpp，"
                  f"前馈开度 {spec.rod_ff}"
                  f"（预期 {spec.rod_ff_expected_rate if spec.rod_ff_expected_rate is not None else '—'} mg/s）")
        # ★ 2026-10-29 16:40（用户："还是不对啊……切到粗加时会先按 2.7V 振动，然后
        # 才变 2V"）：段间必须**先关闸、再改配方、最后开闸**，而且关闸**无条件**执行。
        # 沉降门不是每次都会停振：`_settle_gate` 开头有一道速率门槛
        # （`abs(rate_now) < 本段规定速率×1.5` → 直接 return），而且上一段太短时
        # `_measure_rate(3.0, 4.0)` 根本凑不出样本，rate_now 记 0 → 同样被拒。
        # 那两种情况下振动一直是开着的，于是:
        #   · `_set_amplitude` 变成"边出料边改幅度"，改幅那一瞬间仍按**旧振幅**在落料；
        #   · 新段的搅拌/占空/推杆直接压在上段没断的料流上，等于没有段界。
        # 16:12:59 那次就是活例：粗加只跑 1.8s 到切点（没走沉降门），慢加进段时
        # 日志里既没有"切段沉降"、也没有"振动输出重新打开"——料一直在流。
        # 这里只关**不等**：关→写搅拌/振幅/占空/推杆，串口往返几十毫秒；
        # 等料落定是沉降门的职责，不在进段这里抢（那会把几十 ms 拖成几秒）。
        #
        # 关闸不看 `_vib_running` 这一个标志位：标志位只反映"软件以为的状态"，
        # 上一次 stop 抛异常会被吞掉、页面手动开输出也不改它。只信标志位就等于
        # 允许"标志位说关了、设备其实还在振"——那正是用户看到的现象。
        # `Wc0s` 本身幂等，多关一次零代价。
        was_on = self._vib_running
        t_close = time.monotonic()
        self._safe_stop_vibration()
        if self._vib_running:                 # 关闸真的失败了（串口/设备无应答）
            self._log(f"⚠ 换段关振动失败：{STAGE_TEXT[spec.name]}配方会写在一个仍在"
                      f"振动的料道上（振幅写入后开闸会再复核一次）")
        else:
            self._log(f"① 关闸完成：振动输出已断开"
                      f"（{'上一段沉降门没停振，这里补关' if was_on else '沉降门已关，这里复核一次'}）"
                      f"，接下来改写振幅，最后才开闸 [+{1000 * (time.monotonic() - t_close):.0f}ms]")
        if abs(spec.stir_rpm - self._applied_stir) > 15:
            self._set_stir(spec.stir_rpm, wait=False)
        # 振幅：**进段一律从本段基准起步，基准就是页面填的那个值**。
        # 2026-10-27 11:28 用户定死口径："起振的门槛就是由我来定的，就是我会输入的那个值"。
        # 所以：
        #   · "起振门槛"这个概念已经整个删掉（2026-10-27 11:44）。它当年记的是
        #     "刚才多少伏落得下粉"，而那常常是断料后一脚踢出来的瞬时值（11:19 那次加粉
        #     快加 1139mg 卡桥、3.5V 踢开 → 记成 3.1V），拿它当下一段起点等于让一次
        #     事故决定后面所有段的工作点（粗加基准 1.5V 被顶到 3.1V → 89.8mg/s →
        #     超重 2036mg）；踢完还会把工作振幅上移到"踢脚−0.2V"，于是越踢越高。
        #   · 慢加本来就更严格（设 2V 就跑 2V，不出料靠踢脚救）。
        # 门槛值只由页面「振幅 Vpp」那一行决定：想让它更容易起振，就把它填高。
        base_amp = float(spec.amplitude_vpp)
        new_amp = _clamp_amp(base_amp)
        # 兜底：起步振幅必须在带内（基准±0.5V 的两个端点本来就是起步值，这里是防
        # 以后有人往基准里塞越界值）。
        band_lo, band_hi = _amp_band_for(spec)
        if new_amp > band_hi + 0.001:
            new_amp = band_hi
        elif new_amp < band_lo - 0.001:
            new_amp = band_lo
        new_amp = self._entry_amp_guard(spec, new_amp)
        self._set_amplitude(new_amp, elapsed, f"进入{STAGE_TEXT[spec.name]}")
        self._log(f"② 振幅就位 {self._applied_amp:.2f}Vpp（振动仍关着；"
                  f"推杆/占空写完才开闸）[+{1000 * (time.monotonic() - t_close):.0f}ms]")
        # 质量历史**不能清空**（2026-10-27 11:05 修 bug）：`_measure_rate` 要 6s 窗、
        # ≥4.8s 跨度才出速率，清空后新段前 4.8s 一律测不出 → 再叠加下面
        # `_last_drop_s` 被清成 None，就会被判成"整个窗零落料"（zero_flow），
        # 于是慢加第一次评估必然走"断料涓流 +0.10V"——**方向还是往上升**。
        # 慢加振幅因此永远只从 2.00 爬到 2.10，一次都没降过（10:33 那次加粉实测）。
        # 保留最近 40 拍：跨段质量是连续的，窗口里带一点上一段的余料正好代表真实料流。
        self._mass_hist = self._mass_hist[-40:]
        self._amp_last_eval_s = elapsed
        self._amp_probe = None
        self._amp_band = _amp_band_for(spec)
        self._amp_lock_reason = ""
        # 段内相位复位；τ/T/增益是"这种粉"的性质，跨段保留
        self._amp_write_marks = []
        self._prev_eval_rate = None
        self._last_mass_for_drop = None
        # 落料时刻保留（当作进段瞬间刚落过料）：即便窗口样本还不够，也只会走
        # "拉长窗口再试"，绝不会误判成断料去抬振幅。
        self._last_drop_s = elapsed
        self._stall_wait_s = -99.0
        self._amp_kick = None
        self._amp_kick_tries = 0
        self._amp_kick_last_amp = 0.0
        self._amp_kick_m0_total = 0.0
        self._ramp_below_since = None   # 粗调坡道计时按段重置
        self._ramp_last_s = -99.0
        # v8 速率闭环：响应窗按段从"进段那次写幅"起算
        # 慢加段短（5~15s），等 3s 再评估等于整段只走 1 步 → 慢加用 1.5s（2026-10-27 11:02）
        self._amp_wait_s = (AMP_SLOW_EVAL_S if spec.name == "slow"
                            else AMP_CTRL_MIN_EVAL_S)
        self._amp_last_dir = None
        # v7 自适应状态按段重置
        self._v7_prev_rate = None
        self._v7_prev_t = None
        self._v7_a_est = None
        self._amp_at_cap_since = None
        # 本段的振幅统计（段结束汇总用）
        self._amp_changes = 0
        self._amp_saturated = False
        self._amp_min = self._applied_amp
        self._amp_max = self._applied_amp
        self._amp_heartbeat_s = elapsed
        self._stage_start_s = elapsed
        self._stage_start_mass = mass
        self._stall_abort_ref = mass
        self._stall_abort_s = elapsed

        target_rod = int(spec.rod_ff)
        self._sig_write(lambda: self.hub.sig.set_duty(spec.duty_seed_permyriad / 100.0))
        self._applied_duty = spec.duty_seed_permyriad
        if abs(target_rod - self._applied_window) >= 2:
            self.hub.rod_move(target_rod)         # 只下发不等待：闭环不能被 0.6s 行程卡住
            self._applied_window = target_rod
        # 推杆是硬约束：本段开度就从标定点照抄，全程不再动（用户 2026-10-22 明确要求）。
        # 这句日志是"锁死"的凭证，事后复盘能直接对上"这一段推杆有没有被改过"。
        band_lo, band_hi = self._amp_band
        pulse_note = ("真卡粉才踢一脚 0.5s 脉冲、踢完回落" if spec.zero_rate_pulse
                      else "该粉不许零速率脉冲（不踢脚），只靠这条振幅带逼料")
        amp_note = (f"振幅 {self._applied_amp:.2f}V（慢加不追速率；连续振幅 "
                    f"{band_lo:.2f}–{band_hi:.2f}V = 基准"
                    f"{float(spec.amplitude_vpp):.2f}V±{band_hi - float(spec.amplitude_vpp):.2f}V，"
                    f"{pulse_note}）"
                    if spec.name == "slow" else
                    f"振幅 {band_lo:.2f}–{band_hi:.2f}V"
                    f"（基准 {float(spec.amplitude_vpp):.2f}V：按速率闭环自动加减，"
                    f"慢了加、快了减，追不上就一直加到上界）")
        self._log(f"推杆锁定 {target_rod}（来自标定点/页面手填，整段不动）；{amp_note}")
        # 搅拌 / 振幅 / 占空比 / 推杆全部写完之后，**最后一步才开闸**。
        # 顺序不能反：先开输出再写参数，就等于拿上一段的振幅先甩一坨料
        # （2026-10-29 16:14 用户指出的就是这个）。这里也是沉降门停振后的重开点——
        # 不开的话整段一粒粉都不出，振幅环会把振幅一路顶到 4V 还追不上。
        if not self._vib_running:
            self._open_vibration(
                f"{STAGE_TEXT[spec.name]}，配方已全部就位："
                f"搅拌 {spec.stir_rpm:.0f}RPM / 振幅 {self._applied_amp:.2f}Vpp / "
                f"占空 {spec.duty_seed_permyriad / 100:.1f}% / 推杆 {target_rod}"
                f"；关闸→改幅→开闸 {1000 * (time.monotonic() - t_close):.0f}ms")

    def _prev_stage_rate(self) -> float | None:
        """刚关闭那一段的实测平均速率 mg/s（拿不到就给 None，不做任何压制）。"""
        with self._lock:
            if not self._stage_log:
                return None
            entry = self._stage_log[-1]
        rate = entry.get("actual_rate_mg_s")
        return None if rate is None else float(rate)

    def _entry_amp_guard(self, spec: StagePlan, new_amp: float) -> float:
        """进粗加时，上一段甩得太快就先按 k_est 把起步振幅压下来。

        为什么只管粗加：快加本来就是要流量（压它反而加不动）；慢加是"只恢复出料、
        不追速率"，起步必须锁本段基准（用户 17:29 定的规矩，压下去会空振）。
        粗加夹在中间——它的目标速率比快加低一个量级，却是**接着快加的存量料流**开跑，
        最容易变成"一开段就 6 倍速"。18:04 那次加粉就是这么超的（详见
        ``STAGE_ENTRY_RATE_RATIO`` 的注释）。

        压幅只压到**本段振幅带下界**：再低就该空振了，那种情况交给踢脚和振幅环处理，
        不能为了"看起来安全"把出料压成 0（11:17 那次加粉 2.00V 一粒不出、空转 115s 的教训）。
        """
        if spec.name != "coarse":
            return new_amp
        prev = self._prev_stage_rate()
        if prev is None:
            return new_amp
        target_rate = max(0.1, float(spec.rate_mg_s))
        if prev < STAGE_ENTRY_RATE_RATIO * target_rate:
            return new_amp
        k = float(self._k_est) if self._k_est else AMP_CTRL_K_DEFAULT
        k = max(AMP_CTRL_K_MIN, min(AMP_CTRL_K_MAX, k))
        cut = (prev - target_rate) / k
        floor = _amp_band_for(spec)[0]
        want = round(max(floor, new_amp - cut), 2)
        if want >= new_amp - 0.04:
            return new_amp
        self._log(f"进段压幅：{STAGE_TEXT[spec.name]} 起步 {new_amp:.2f} → {want:.2f} Vpp"
                  f"（上一段实测 {prev:.1f} mg/s = 本段目标 {target_rate:.1f} 的 "
                  f"{prev / target_rate:.1f} 倍，按 k={k:.0f} mg/s/V 压 {cut:.2f}V，"
                  f"下限 {floor:.2f}V）——别让上一段的料流惯性当本段起点")
        return want

    # ------------------------------------------------------------------
    # 切段沉降门（重粉关键：切段那一刻料还没落完）
    # ------------------------------------------------------------------

    def _gate_threshold(self, spec: StagePlan) -> float:
        """本段切出速率的门槛（mg/s）。

        **按段比例**（2026-10-26 11:05 用户定）：门槛 = 本段规定速率 × 1.5，
        粗加 20 → 30mg/s、快加 75 → 112.5mg/s、慢加 5 → 7.5mg/s。同一个绝对值量
        三个段没有物理意义：30mg/s 对慢加是 6 倍超速（早该等），对快加只是 0.4 倍
        （慢得反常反而不等）。页面「触发速率」填了数就用那个绝对值（老口径，覆盖比例）。
        """
        if self._gate_trigger is not None:
            return float(self._gate_trigger)
        return max(SETTLE_GATE_MIN_TRIGGER_MG_S,
                   SETTLE_GATE_RATE_RATIO * max(0.1, float(spec.rate_mg_s)))

    def _predictive_cut(self, prev: StagePlan, mass: float) -> dict:
        """预测切段：质量 + (速率 × 1s) 已经够到切点 → 现在就停振，让余料正好落在切点上。

        用户 2026-10-26 11:05 的原话："预测尾量加当前质量到了 122.6 就需要停了来等待
        尾量或者粉末的沉积，或者切换参数进入下一阶段"。以前是"质量先越过切点、再停振、
        再等余料落完"，停振时余料已经全部记在切点之后了；现在提前一个落料时延停振。

        ``switch_compensation_mg`` 是粉体档案里**已经学到的惯性落料量**（切点已被它
        提前过），所以预测的提前量要减去这部分，免得同一笔余料被补两遍。
        返回 ``{"cut": bool, "rate_mg_s", "lead_mg", "switch_mg"}``。
        """
        out = {"cut": False, "rate_mg_s": 0.0, "lead_mg": 0.0, "switch_mg": None}
        if not self._gate_enabled or prev.switch_mass_mg is None:
            return out
        switch = float(prev.switch_mass_mg)
        out["switch_mg"] = switch
        meas = self._measure_rate(3.0, 4.0)
        rate = float(meas["rate"]) if meas else 0.0
        out["rate_mg_s"] = rate
        if rate <= 0.5:
            return out
        lead = max(0.0, rate * SETTLE_GATE_LEAD_S - float(prev.switch_compensation_mg))
        out["lead_mg"] = round(lead, 1)
        # 只提前，不重复：lead=0 说明学到的补偿已经把这件事做了
        out["cut"] = lead >= 1.0 and mass < switch and mass + lead >= switch
        return out

    def _settle_gate(self, prev: StagePlan, nxt: StagePlan,
                     mass: float, elapsed: float, *, force: bool = False) -> dict:
        """切段沉降门：切出速率高时先停振，等余料落干净再换配方。

        为什么必须做：质量到达切换点 ≠ 粉已经落完。振动一停，料槽里和半空中的粉
        还会继续落到秤上（重粉落差短但单次落料量大）。不等的话这部分余料会直接算进
        下一段——粗加一开场就超过自己的切换点，慢加要倒着补，最终必然偏重。

        动作顺序：**只关振动**，搅拌保持（防搭桥、帮余料走完），推杆不动（不许为了
        "等"去挪机构）。判稳条件：**连续 hold_s 秒没有 ≥2mg 的增重**（hold_s 取自学到的
        落料周期 T_est，钳 2–5s）。硬上限 SETTLE_GATE_MAX_S（8s）只防"永远等下去"；
        到点仍未稳就照切，但日志会明确告警"下一段是踩着余料起振的"。

        （2026-10-27 11:35 用户："切换阶段等待粉末沉降时，沉降未结束，天平质量还在变化就
        进行了下一阶段的振动导致粉末质量不可控"。旧版上限 1.2s + "0.5s 窗极差≤2mg"判稳，
        脉冲式落料两坨之间本来就静，于是每段都 1.2s 到点、stable=False 硬切——
        实测余料 +52/+82/+93/+95/+127mg 全落进了下一段。）

        ``force=True`` 来自 ``_predictive_cut``：已经决定提前停振了，速率门槛不再拦
        （否则"提前停振"会被"速率不够高"否掉，等于没做）。

        返回 ``{"applied", "mass", "elapsed", "gain_mg", "waited_s", "stable", ...}``，
        ``gain_mg`` 就是"停振后还多落了多少"，会被写进粉体档案当下次的提前量。
        """
        out = {"applied": False, "mass": mass, "elapsed": elapsed,
               "gain_mg": 0.0, "waited_s": 0.0, "stable": False, "rate_mg_s": 0.0}
        if not self._gate_enabled:
            return out
        # 判稳需要的静默时间 = 一个落料周期（学到的 T_est），钳在 2.0–5.0s。
        # 页面「沉降等 s」是这一段的等待上限；但它短于判稳所需时会被抬到 hold+1s，
        # 否则就是"门永远来不及判稳"——这正是 1.2s 那版的 bug。
        hold_s = max(SETTLE_GATE_HOLD_MIN_S,
                     min(SETTLE_GATE_HOLD_MAX_S, float(self._period_est)))
        wait_limit = (float(prev.settle_wait_s) if prev.settle_wait_s is not None
                      else SETTLE_GATE_MAX_S)
        if wait_limit <= 0:
            return out                      # 页面填 0 = 这一段显式不等（老行为）
        wait_limit = max(wait_limit, hold_s + 1.0)
        # 触发判定：用上一段最近这几秒的回归速率（和振幅环同一套算法）。
        # 以前读 _rate_hist，那是"瞬时速率平均值"的旧字段，振幅环改用质量回归后
        # 已经没人往里写 —— 不改这里沉降门会永远判 0 速率、一次都不触发。
        meas = self._measure_rate(3.0, 4.0)
        rate_now = float(meas["rate"]) if meas else 0.0
        threshold = self._gate_threshold(prev)
        if not force and abs(rate_now) < threshold:
            return out

        why = ("已提前到达切点" if force else
               f"切出速率 {rate_now:.1f} mg/s ≥ 门槛 {threshold:g}"
               f"（{STAGE_TEXT[prev.name]}规定 {prev.rate_mg_s:g}mg/s"
               f"×{SETTLE_GATE_RATE_RATIO:g}）")
        self._set_state("running",
                        f"切段沉降：{STAGE_TEXT[prev.name]}→{STAGE_TEXT[nxt.name]}，"
                        f"停振等余料落完（≤{wait_limit:g}s）")
        self._log(f"切段沉降：{STAGE_TEXT[prev.name]}→{STAGE_TEXT[nxt.name]}，"
                  f"{why}，停振等余料落完（最多 {wait_limit:g}s），搅拌暂时保持不动")
        self._safe_stop_vibration()
        # ★ 2026-10-29 17:10 **撤销**"停振时预置下一段振幅"（16:50 加的，只活了 20 分钟）。
        # 用户现场判定："没震动，那是搅拌出的粉末"。翻 17:00:19 那次 10000mg 的逐拍：
        # 快加段 ≥2.75V 时平均 80mg/s（振动是有效的），**切段之后粗加整段只有 2–9mg/s，
        # 振幅加到带上界 2.49V 仍不出料**。切段这一刻唯一新增的动作就是"输出关着的时候
        # 写 WC"，所以撤掉它。以后只在两个点写振幅：进段（输出关着写一次）、
        # 开闸后复核一次——不再在沉降等待期间反复写。
        t0 = time.monotonic()
        hist: list[tuple[float, float]] = []
        stable = False
        stir_off = False          # 漏料型粉：等不到判稳就把搅拌也停掉（见常量注释）
        last_tick = 0.0
        last_curve = 0.0
        while True:
            if self._stop_event.is_set():
                break                      # 操作员要停：不再等，直接交给外层中止
            now = time.monotonic()
            if now - t0 >= wait_limit:
                break
            try:
                raw = self._read_balance_raw()
            except Exception as error:      # noqa: BLE001 - 读不到就按超时切，不在这里卡
                self._log(f"沉降等待时天平读不到（{error}），直接切段")
                break
            cur = float(raw) - self._baseline
            hist.append((now, cur))
            # ★ 2026-10-23 18:59（用户指出"沉降门的时候不更新质量"）：
            # 以前整个等待循环只在退出时用最后一个读数，中间的增重既不进页面、
            # 也不进曲线和逐拍 CSV——操作员看到质量冻在切段那一刻，而实际粉还在落。
            # 现在每 0.12s 记一拍（stage 记 gate），页面/曲线/CSV 全程连续。
            elapsed_now = now - self._t0
            if now - last_tick >= self.SAMPLE_INTERVAL_S:
                last_tick = now
                rate = 0.0
                if len(hist) >= 3:
                    span = max(0.05, hist[-1][0] - hist[-3][0])
                    rate = (hist[-1][1] - hist[-3][1]) / span
                self._record(elapsed_now, cur, rate, "gate", learn=False)
                if elapsed_now - last_curve >= 0.5:
                    last_curve = elapsed_now
                    with self._lock:
                        self._curve.append([round(elapsed_now, 2), round(cur, 1),
                                            round(rate, 2)])
                self._set_state("running",
                                f"沉降等待（振动已关）：{STAGE_TEXT[prev.name]}→"
                                f"{STAGE_TEXT[nxt.name]} "
                                f"{cur:.0f}/{self._config['target_mg']} mg"
                                f"（余料 {cur - mass:+.0f}mg，已等 "
                                f"{now - t0:.1f}/{wait_limit:g}s，判稳线 {hold_s:.1f}s）")
            # 漏料型粉：停振 2s 后**还在持续往下走**（近 1.2s ≥3mg/s）→ 问题不在振动，
            # 是搅拌在推料，把搅拌也停掉再多给 4s（下一段 `_enter_stage` 会重新起转）。
            # 只看"持续速率"不看窗口净增重：正常粉停振后那几秒的余料是衰减的尾巴，
            # 不该为它去动搅拌（11:19 那次加粉正常粉在粗加→慢加就被误停过一次）。
            if (not stir_off and self._applied_stir > 1.0
                    and now - t0 >= SETTLE_GATE_STIR_OFF_S):
                recent = [p for p in hist if p[0] >= now - 1.2]
                if len(recent) >= 2:
                    span = max(0.05, recent[-1][0] - recent[0][0])
                    r_recent = (recent[-1][1] - recent[0][1]) / span
                    if r_recent >= SETTLE_GATE_STIR_OFF_RATE_MG_S:
                        stir_off = True
                        wait_limit += SETTLE_GATE_STIR_OFF_EXTRA_S
                        self._log(f"沉降等待 {now - t0:.1f}s 仍以 {r_recent:.1f}mg/s 往下走"
                                  f" → 搅拌减速停（{self._applied_stir:.0f}RPM 还在推料，"
                                  f"光停振停不下来；下一段会按配方重新起转），"
                                  f"等待上限延到 {wait_limit:g}s")
                        self._set_stir(0.0, wait=False)
            # 判稳 = **一个完整落料周期（hold_s）内净增重 ≤ SETTLE_GATE_DROP_MG**。
            # 用"窗口两端净变化"而不是"单拍增重"：天平有 ±1mg 抖动，单拍阈值会被噪声
            # 反复触发（真机上表现为永远判不稳）；窗口净变化对噪声免疫，又对"又落了
            # 一坨"敏感——只要这一整个周期里还多了 2mg 以上，就说明料还在走。
            if now - t0 >= hold_s:
                ref = hist[0][1]
                for ts, ms in reversed(hist):
                    if ts <= now - hold_s:
                        ref = ms
                        break
                net = cur - ref
                if net <= SETTLE_GATE_DROP_MG:
                    stable = True
                    break
            time.sleep(0.1)
        waited = time.monotonic() - t0
        landed = hist[-1][1] if hist else mass
        # "静默"= 末尾这一段净增重 ≤2mg 已经持续了多久（判稳线是 hold_s）
        quiet_s = 0.0
        for ts, ms in reversed(hist):
            if landed - ms <= SETTLE_GATE_DROP_MG:
                quiet_s = time.monotonic() - ts
            else:
                break
        out.update(applied=True, mass=landed, elapsed=time.monotonic() - self._t0,
                   gain_mg=round(landed - mass, 1), waited_s=round(waited, 2),
                   stable=stable, rate_mg_s=round(rate_now, 1),
                   quiet_s=round(quiet_s, 2),
                   from_stage=prev.name, to_stage=nxt.name)
        self._log(f"沉降结束：余料 {landed - mass:+.1f} mg（{STAGE_TEXT[prev.name]}），"
                  f"等 {waited:.1f}s（静默 {quiet_s:.1f}s / 判稳线 {hold_s:.1f}s，"
                  f"{'已判稳' if stable else '到时仍未稳'}），当前 {landed:.1f} mg")
        if not stable:
            # 到硬上限还在落：仍然切，但必须让操作员知道下一段是"踩着余料"起振的
            self._log(f"⚠ 沉降未判稳就切段：{STAGE_TEXT[prev.name]}停振后等满 "
                      f"{wait_limit:g}s 时质量仍在变化（最近 {quiet_s:.1f}s 内有 ≥"
                      f"{SETTLE_GATE_DROP_MG:g}mg 增重），{STAGE_TEXT[nxt.name]}会在余料"
                      f"还没走完时起振——这一段的实际速率会偏高，看紧一点")
        return out

    def _note_settle(self, landed: dict) -> None:
        """把这次沉降结果记到刚结束那一段的日志里（结果页要能看出余料多少）。"""
        with self._lock:
            if self._stage_log:
                entry = self._stage_log[-1]
                entry["settle_gain_mg"] = landed["gain_mg"]
                entry["settle_wait_s"] = landed["waited_s"]
                entry["settle_stable"] = bool(landed.get("stable"))
                entry["settle_rate_mg_s"] = landed.get("rate_mg_s")
        self._settle_events.append({
            "from": landed.get("from_stage"), "to": landed.get("to_stage"),
            "gain_mg": landed["gain_mg"], "waited_s": landed["waited_s"],
            "stable": bool(landed.get("stable")),
            "rate_mg_s": landed.get("rate_mg_s"),
        })

    def _surge_rate(self, window_s: float = 0.7) -> dict | None:
        """超速熔断专用的**短窗**速率：只回答"这 0.7s 到底多快"，不求精度。

        为什么不能复用 `_measure_rate(6.0, …)`：那口窗要攒 6s 才出数，而塌料
        0.5s 就能落 100mg —— 等 6s 出数，粉早就把目标顶穿了（11:30:25 那次加粉
        0→272mg 只用了 2.2s）。这里只要 4 个采样点（≈0.5s）、增重 ≥5mg 就算，
        宁可粗一点，也要在塌料发生的当口就看见。
        """
        with self._lock:
            hist = list(self._mass_hist)
        # ★ 2026-10-26 18:04 现场（用户："慢加之后的缓补好像没有等待粉末沉积"）：
        # 这里原来要求 0.7s 窗内 **≥4 个采样点**，可主循环的实际周期不是标称的
        # `SAMPLE_INTERVAL_S=0.12s` ——读天平 + 写串口 + 控制器计算合起来约 **0.266s**
        # （18:03 那次加粉 CSV 逐拍间隔实测中位 0.266、粗加段 0.25–0.34）。0.7s 只装得下
        # 3 个点，条件永远不成立 → `_surge_rate` 恒返回 None → **超速熔断形同虚设**。
        # 那次加粉粗加 1455→1831mg 只用 3.0s（125.88 mg/s，目标 20 的 6.3 倍），门槛 50mg/s
        # 早就该刹车却一次都没触发，最后 0 发脉冲超验收 48mg。
        # 现在门槛降到 3 个点（跨 ≈0.53s，仍由下面的 span≥0.30 保底，不会拿 2 点判速）。
        if len(hist) < 3:
            return None
        left = hist[-1][0] - float(window_s)
        pts = [p for p in hist if p[0] >= left]
        if len(pts) < 3:
            return None
        span = pts[-1][0] - pts[0][0]
        gain = pts[-1][1] - pts[0][1]
        if span < 0.30 or gain < SURGE_MIN_GAIN_MG:
            return None                     # 增重太小（天平 1mg 噪声）就不判
        return {"rate": gain / span, "gain": gain, "span": span, "n": len(pts)}

    def _surge_brake(self, spec: StagePlan, mass: float, elapsed: float) -> dict:
        """段内超速熔断：一瞬间落得太猛就立刻停振，等余料落完再决定下一步。

        **判据（任一命中）**
          ① 0.7s 短窗速率 ≥ max(12, 本段规定速率 × 倍率)：粗加 20 → 50mg/s；
          ② 剩余量 ≤ 速率 × 1.5s（且速率 ≥12mg/s）：这一瞬间的落料就够把目标顶穿。

        为什么这两条能治"没触发脉冲还超重"：缓补的入口是"剩余 ≤40mg"，而塌料
        是一步从 176 跨到 210（11:30:13 那次加粉），根本不给缓补开门的机会；慢加的
        振幅环还在等 6s 回归窗，等算出速率时已经晚了。熔断只看瞬时速率，先关振，
        让惯性余料自己落干净，再重新起振——宁可多等 2s，也不要一次加粉超重 79mg。

        返回 ``{"applied", "finish", "advance", "mass", "elapsed", "reason"}``：
        ``finish=True`` 表示熔断后已经进了验收带，不必再起振，直接收尾；
        ``advance=True`` 表示塌得太猛、本段到此为止，主循环下一拍**直接切下一段**
        （2026-10-26 18:35 用户："熔断只发生在阶段切换之间就行了"——不再转缓补脉冲）。
        """
        out = {"applied": False, "finish": False, "advance": False, "mass": mass,
               "elapsed": elapsed, "reason": ""}
        if not self._surge_on or self._surge_times >= SURGE_MAX_TIMES:
            return out
        if not self._vib_running:
            return out
        if elapsed - float(self._surge_last_s) < SURGE_COOLDOWN_S:
            return out
        target = float(self._config["target_mg"])
        remain = target - mass
        # 最后这一勺交给缓补脉冲（它本来就是"只开一发就停"，比连续振动安全）
        if (self._trim_on and spec.name == "slow"
                and 0.0 < remain <= self._trim_enter_mg):
            return out
        meas = self._surge_rate()
        if meas is None:
            return out
        rate = float(meas["rate"])
        surge_th = max(SURGE_ABS_MIN_MG_S,
                       float(self._surge_ratio) * float(spec.rate_mg_s))
        brake_remain = max(SURGE_STOP_MIN_REMAIN_MG, rate * SURGE_STOP_HORIZON_S)
        # 慢加不走"超速"这条：慢加规定 5mg/s，踢一脚/塌一阵 15~20mg/s 是常态，
        # 拿它当熔断条件会一路刹车把料刹停（2026-10-26 12:05 那次加粉 400mg 就死在
        # 三次"慢加 17mg/s"上）。慢加只保留"剩余量不够落"这条，那才是真会超重的。
        fast = spec.name != "slow" and rate >= surge_th
        near = (rate >= SURGE_ABS_MIN_MG_S and 0.0 < remain <= brake_remain)
        if not (fast or near):
            return out
        why = (f"0.7s 内 +{meas['gain']:.0f}mg（{rate:.0f}mg/s ≥ 门槛 {surge_th:.0f}mg/s）"
               if fast else
               f"只剩 {remain:.0f}mg，不够这 {rate:.0f}mg/s 落 "
               f"{SURGE_STOP_HORIZON_S:g}s（要 {rate * SURGE_STOP_HORIZON_S:.0f}mg）")
        self._surge_times += 1
        self._surge_last_s = elapsed
        self._log(f"超速熔断（第 {self._surge_times} 次）：{STAGE_TEXT[spec.name]} "
                  f"{mass:.0f}/{target:.0f}mg，{why} → 立刻停振等余料，"
                  f"不让惯性把目标顶穿")
        self._set_state("running",
                        f"超速熔断：{STAGE_TEXT[spec.name]} {mass:.0f}mg、"
                        f"{rate:.0f}mg/s 落得太猛，停振等余料（≤{SURGE_SETTLE_MAX_S:g}s）")
        self._safe_stop_vibration()
        t0 = time.monotonic()
        hist: list[tuple[float, float]] = []
        stable = False
        last_tick = 0.0
        last_curve = 0.0
        while True:
            if self._stop_event.is_set():
                break
            now = time.monotonic()
            if now - t0 >= SURGE_SETTLE_MAX_S:
                break
            try:
                raw = self._read_balance_raw()
            except Exception as error:      # noqa: BLE001 - 读不到就按当前读数继续
                self._log(f"超速熔断等待时天平读不到（{error}），按当前读数继续")
                break
            cur = float(raw) - self._baseline
            hist.append((now, cur))
            elapsed_now = now - self._t0
            # 停振期间秤还在涨，必须实时记（同沉降门：不记就"看不见余料"）
            if now - last_tick >= self.SAMPLE_INTERVAL_S:
                last_tick = now
                tick_rate = 0.0
                if len(hist) >= 3:
                    tick_span = max(0.05, hist[-1][0] - hist[-3][0])
                    tick_rate = (hist[-1][1] - hist[-3][1]) / tick_span
                self._record(elapsed_now, cur, tick_rate, "surge", learn=False)
                if elapsed_now - last_curve >= 0.5:
                    last_curve = elapsed_now
                    with self._lock:
                        self._curve.append([round(elapsed_now, 2), round(cur, 1),
                                            round(tick_rate, 2)])
                self._set_state("running",
                                f"超速熔断：停振等余料 {cur:.0f}/"
                                f"{target:.0f}mg（已多落 {cur - mass:+.0f}mg）")
            if len(hist) >= SURGE_SETTLE_SAMPLES and now - t0 >= 0.5:
                window = hist[-SURGE_SETTLE_SAMPLES:]
                span = max(0.05, window[-1][0] - window[0][0])
                slope = abs((window[-1][1] - window[0][1]) / span)
                spread = max(p[1] for p in window) - min(p[1] for p in window)
                if slope <= SURGE_SETTLE_RATE_EPS_MG_S and spread <= SURGE_SETTLE_STABLE_MG:
                    stable = True
                    break
            time.sleep(0.1)
        waited = time.monotonic() - t0
        landed = hist[-1][1] if hist else mass
        landed_elapsed = time.monotonic() - self._t0
        out.update(applied=True, mass=landed, elapsed=landed_elapsed)
        self._surge_events.append({
            "stage": spec.name, "mass_mg": round(landed, 1),
            "rate_mg_s": round(rate, 1), "gain_mg": round(landed - mass, 1),
            "waited_s": round(waited, 2), "stable": stable, "why": why})
        self._log(f"超速熔断结束：停振期间又落 {landed - mass:+.0f}mg"
                  f"（{mass:.0f} → {landed:.0f}mg，等 {waited:.1f}s，"
                  f"{'已判稳' if stable else '到时未稳'}）")
        accept_lo, _accept_hi = acceptance_limits(int(round(target)))
        if landed >= accept_lo:
            out["finish"] = True
            out["reason"] = (f"超速熔断后已到 {landed:.0f}mg（验收下限 {accept_lo:.0f}mg），"
                             f"不再起振，直接收尾称重")
            self._log(out["reason"])
            return out
        if self._stop_event.is_set():
            # 操作员要停：绝不重新起振（等主循环退出，由收尾统一关输出）
            out["reason"] = "操作员停止"
            return out
        # 塌得够猛 → **本段到此为止，提前切段**（用户 2026-10-26 18:35："熔断只发生在
        # 阶段切换之间就行了"）。不再把剩余量塞给缓补脉冲：熔断就是"提前切段"的另一
        # 种触发方式，跟"质量到切点""预测切点"完全同级——主循环下一拍就走切段流程
        # （关段账 → 沉降门 → 进下一段配方），粗加/慢加照常各跑各的。
        big_gain = (landed - mass) >= SURGE_ADVANCE_MG
        if big_gain:
            out["advance"] = True
            out["reason"] = (f"超速熔断后发现料道还在塌（停振期间又落 "
                             f"{landed - mass:+.0f}mg、触发峰值 {rate:.0f}mg/s），"
                             f"本段提前结束、直接切下一段"
                             f"（剩 {target - landed:.0f}mg 交给更低的速率/振幅，不走脉冲）")
            self._log(out["reason"])
            # 振动保持关闭：下一拍的 `_enter_stage` 会按新段配方重开（它也负责重开振动），
            # 这里先开着只会用旧段的高振幅再灌一坨。
            return out
        # 继续加：**只有真塌料（速率 ≥2 倍门槛）才压振幅**，而且一次加粉最多压 3 档。
        # 2026-10-26 12:00 的教训：`near`（剩余量不够落）这种"正常偏快"也压振幅，
        # 结果 400mg/死区 3.2V 那次加粉被一路压到 1.80V，掉进死区彻底不出料，
        # 最后靠"断料中止"报错——熔断是为了刹车，不是把车拆了。
        if (fast and rate >= 2.0 * surge_th
                and self._surge_amp_cuts < SURGE_AMP_CUT_MAX):
            cut_amp = max(AMP_MIN_VPP, round(self._applied_amp - SURGE_AMP_CUT_V, 2))
            if cut_amp < self._applied_amp - 0.005:
                self._surge_amp_cuts += 1
                self._set_amplitude(cut_amp, landed_elapsed, "超速熔断后降幅")
        self._amp_last_eval_s = landed_elapsed
        self._amp_wait_s = AMP_CTRL_MIN_EVAL_S
        self._amp_probe = None
        self._prev_eval_rate = None
        self._open_vibration(f"超速熔断后重新起振：{self._applied_amp:.2f}V 继续加"
                             f"（剩 {target - landed:.0f}mg，流速回到正常再让振幅环接管）")
        return out

    def _maybe_taper(self, spec: StagePlan, plan: list[StagePlan], index: int,
                     mass: float, elapsed: float) -> None:
        """临到切点提前降幅：把切点前的流速压下来，别让惯性把下一段顶穿。

        距离取 ``max(8mg, 当前速率 × 系数)``：高速时早降（100mg/s × 0.6 = 60mg 前），
        低速时只提前 8mg，避免慢段白白多跑好几秒。每段只降一次。

        速率用回归值（3s 短窗，这里只需要量级）；拿不到就按 0 处理，
        lead 自动退回最小值 8mg —— 宁可按最小距离降，也不要凭一个噪声值提前太早。
        """
        if self._taper_factor <= 0 or index >= len(plan) - 1:
            return
        if spec.switch_mass_mg is None or self._taper_done_index == index:
            return
        dist = float(spec.switch_mass_mg) - mass
        if dist <= 0:
            return
        meas = self._measure_rate(3.0, 4.0)
        rate = float(meas["rate"]) if meas else 0.0
        lead = max(TAIL_TAPER_MIN_MG, max(0.0, rate) * self._taper_factor)
        if dist > lead:
            return
        nxt = plan[index + 1]
        # 提前降幅的目标就是下一段的**标定振幅**（慢加段就是它锁死的那个值）。
        # 不再按速率比例缩放：缩放值会跑到标定工况之外（上一版把 2.0 缩到 2.66）。
        target_amp = _clamp_amp(nxt.amplitude_vpp)
        # 旧版这里还有一道"不许降到实测起振门槛−0.2V 以下"的地板（2026-10-26 11:17 事故加的），
        # **2026-10-27 11:28 已删**：门槛由用户在页面填（用户原话"起振的门槛就是由我来定的，
        # 就是我会输入的那个值"），学到的门槛若高于下一段基准，会把这个提前降幅整个取消，
        # 结果切点前料流照样全速冲过去。降下去真断料，`_taper_watchdog` 会撤销并恢复原振幅。
        self._taper_done_index = index
        if target_amp >= self._applied_amp - 0.03:
            return                          # 下一段本来就要更高幅：没必要先降
        previous = self._applied_amp
        self._set_amplitude(target_amp, elapsed,
                            f"距{STAGE_TEXT[spec.name]}切点 {dist:.0f}mg，提前降幅")
        if abs(self._applied_amp - previous) >= 0.02:
            self._log(f"提前降幅（{STAGE_TEXT[spec.name]}距切点 {dist:.0f}mg）："
                      f"{previous:.2f} → {self._applied_amp:.2f} Vpp，"
                      f"让流速在切段之前先下来，余料少一点")
        # 降幅之后**不再让振幅环把它抬回去**（否则两个环互相打架）：
        # 速率历史保留着（沉降门还要用"切出速率"判断该不该等），只是不再调振幅。
        # 但窗口有看门狗（`_taper_watchdog`）：压到断料就撤销、恢复原振幅继续调幅，
        # 绝不允许像 2026-10-26 11:15 那样"幅 2.00V、+0.0mg/s"空转到人工停止。
        self._taper_hold = True
        self._taper_hold_since = elapsed
        self._taper_prev_amp = previous

    def _taper_watchdog(self, spec: StagePlan, mass: float, elapsed: float) -> bool:
        """降幅窗口的看门狗：返回 True = 继续压着调幅环，False = 已撤销、放手。

        判据只有一条：**降幅是为了把流速压小，不是为了把出料压停**。
        窗口里一旦出现"连续 ``TAIL_TAPER_DRY_S`` 秒几乎没落料"，或者窗口已经顶到
        ``TAIL_TAPER_HOLD_MAX_S``，就立刻把振幅退回降幅前的工作值，让速率环/踢一脚
        重新接管；质量到了切点则由主循环自己走预测切段 + 沉降门。
        """
        if not self._taper_hold:
            return False
        switch = (float(spec.switch_mass_mg)
                  if spec.switch_mass_mg is not None else None)
        if switch is not None and mass >= switch:
            return True          # 已经到切点：下一拍主循环就切段，不用管
        held = elapsed - float(self._taper_hold_since)
        if held < TAIL_TAPER_DRY_S:
            return True
        meas = self._measure_rate(TAIL_TAPER_DRY_S, 2.0)
        rate = float(meas["rate"]) if meas else 0.0
        dry = rate < TAIL_TAPER_DRY_RATE_MG_S
        too_long = held >= TAIL_TAPER_HOLD_MAX_S
        if not (dry or too_long):
            return True          # 还在出料：降幅本来就该让它慢一点走到切点
        prev = float(self._taper_prev_amp if self._taper_prev_amp is not None
                     else self._applied_amp)
        self._taper_hold = False
        self._taper_prev_amp = None
        why = (f"降幅后 {held:.1f}s 速率 {rate:.1f}mg/s（<{TAIL_TAPER_DRY_RATE_MG_S:g}），"
               f"降过头把出料压停了" if dry and not too_long
               else f"降幅窗口已满 {held:.1f}s，质量还差 "
                    f"{(switch - mass) if switch is not None else 0:.1f}mg 到切点")
        if prev > self._applied_amp + 0.02:
            self._set_amplitude(prev, elapsed, "提前降幅后断料，退回工作振幅")
            self._log(f"提前降幅撤销：{why} → 振幅退回 {prev:.2f}V，"
                      f"振幅环/踢一脚重新接管，先让它继续出料再走预测切段")
        else:
            self._log(f"提前降幅窗口放开：{why} → 振幅环重新接管"
                      f"（慢了自己加，顶到带顶还不出就踢一脚）")
        # 恢复后从这一刻重新算响应窗，别拿降幅那几秒的低速去判"缺口很大"一步跨太多
        self._amp_last_eval_s = elapsed
        self._amp_wait_s = AMP_CTRL_MIN_EVAL_S
        self._amp_probe = None
        self._prev_eval_rate = None
        return False

    def _rate_command_for(self, spec: StagePlan, mass: float) -> float:
        """本段此刻该追的速率（mg/s）。

        快加/粗加就是配方的固定靶子；**慢加用动态目标 = 剩余量 / 18s**，
        上限是标定靶子的 1.6 倍。理由（实测）：封堵剂1 慢加段靶子 5 mg/s，
        实际只能出 1.17，硬追的结果是振幅每 6 秒爬 0.03V、124 秒才加 145mg。
        改成"剩余多少就按 18 秒收完"以后：剩余 146mg → 目标 8.1，
        剩余 30mg → 1.7，剩余 12mg → 0.7 —— 天然是个越来越慢的收窄形状，
        最后那几 mg 交给预测停车，不会为了追一个僵硬的 5 mg/s 磨洋工。
        """
        if spec.name != "slow":
            return float(spec.rate_mg_s)
        remain = max(0.0, float(self._config["target_mg"]) - float(mass))
        cmd = remain / AMP_SLOW_TAU_S
        cap = max(0.6, float(spec.rate_mg_s) * AMP_SLOW_CMD_CAP_RATIO)
        return max(0.4, min(cap, cmd))

    def _measure_rate(self, window_s: float, min_gain_mg: float,
                      skip_before: float | None = 0.0) -> dict | None:
        """用质量累积回归算速率——振幅环唯一的速率来源。

        为什么不用 estimate 的瞬时速率：封堵剂1 是脉冲出料，天平只在粉块落下时
        跳一下、中间是平的，差分量算出来 77% 的采样是 0（中位数 0.00）。
        拿这种信号做反馈，振幅就会在两个相邻值之间来回抖（实测粗加 49 秒里
        改了 14 次、全在 1.82–2.06V 之间晃），纯属白折腾信号源。

        返回 None = "现在还没有可用的速率信息"，调用方必须**什么都不做**：
        样本不足、窗口太短、窗口内增重太小（粉还没落下来）都算这种情况。
        ``skip_before`` 用来切掉"刚改完振幅的响应期"（那会儿流量还是旧振幅的）。
        传 None = 不跳过任何样本（v7 小步调幅就是这么调的：效果靠加速度外推补，
        不必全额等 τ）。**None 必须先归一成 0.0 再参与比较**——2026-10-23 10:10
        就是这里直接把 None 拿去比大小，`float >= NoneType` 让整次加粉加粉在起振 2 秒后
        异常中断（hist 攒够 8 个点才走到这行，所以现象是"总是 2 秒后炸"）。
        """
        skip = 0.0 if skip_before is None else float(skip_before)
        with self._lock:
            hist = list(self._mass_hist)
        if len(hist) < 8:
            return None
        left = hist[-1][0] - float(window_s)
        pts = [point for point in hist
               if point[0] >= left and point[0] >= skip]
        if len(pts) < 8:
            return None
        span = pts[-1][0] - pts[0][0]
        if span < float(window_s) * 0.8:
            return None
        gain = pts[-1][1] - pts[0][1]
        if abs(gain) < float(min_gain_mg):
            return None                     # 几乎没落粉：没有速率信息，不是"速率=0"
        n = len(pts)
        mean_t = sum(point[0] for point in pts) / n
        mean_m = sum(point[1] for point in pts) / n
        stt = sum((point[0] - mean_t) ** 2 for point in pts)
        if stt <= 1e-6:
            return None
        slope = sum((point[0] - mean_t) * (point[1] - mean_m) for point in pts) / stt
        sst = sum((point[1] - mean_m) ** 2 for point in pts)
        if sst > 1e-9:
            ssr = sum((point[1] - (mean_m + slope * (point[0] - mean_t))) ** 2
                      for point in pts)
            r2 = max(0.0, 1.0 - ssr / sst)
        else:
            r2 = 1.0
        return {"rate": slope, "span": span, "n": n, "gain": gain, "r2": r2}

    def _set_stir(self, rpm: float, *, wait: bool) -> None:
        try:
            if rpm <= 0.5:
                self.hub.stir_stop("decel", acc=self.STIR_ACC)
                self._applied_stir = 0.0
            else:
                self.hub.stir_start(float(rpm), acc=self.STIR_ACC, direction=0)
                self._applied_stir = float(rpm)
                if wait:
                    time.sleep(self.STIR_SETTLE_S)
        except Exception as error:      # noqa: BLE001 - 搅拌失败不该让整次加粉直接崩
            self._log(f"搅拌切换到 {rpm:.0f} RPM 失败：{error}")

    def _amp_heartbeat(self, spec: StagePlan, mass: float, elapsed: float,
                       cmd: float, detail: str) -> None:
        """振幅环心跳：每 5s 一条，把"现在什么状况、为什么调/不调"写进日志。

        用户 2026-10-22 明确要求：加粉过程中要看得出振幅到底在干什么
        （之前慢加段振幅从 1.00 爬到 1.54 一条日志都没有，事后完全无法复盘）。
        ``detail`` 由调用方拼好（"为什么不动 / 为什么动、动了多少"），
        这里只负责统一格式和限流。
        """
        # 先存现场再限流：逐拍 CSV 每行都要带"这一拍为什么动/不动"，
        # 日志才限流 5s，CSV 不限。
        self._amp_cmd = float(cmd)
        self._amp_detail = detail
        if elapsed - self._amp_heartbeat_s < AMP_HEARTBEAT_S:
            return
        self._amp_heartbeat_s = elapsed
        target = float(self._config["target_mg"])
        # 慢加段的目标是动态的（剩余量/18s），标成"目标(动态)"免得看日志的人以为是常量
        label = "目标(动态)" if spec.name == "slow" else "目标"
        self._log(f"[振幅] {STAGE_TEXT[spec.name]} {mass:.0f}/{target:.0f}mg "
                  f"幅 {self._applied_amp:.2f}V 杆 {self._applied_window}"
                  f"（锁定）搅 {self._applied_stir:.0f}rpm "
                  f"{label} {cmd:.2f} mg/s ｜{detail}")

    def _step_amplitude(self, spec: StagePlan, mass: float, elapsed: float) -> None:
        """振幅调节：增益归一化的增量式 PI（2026-10-23 11:33 定，11:50 收回口径）。

        用户原话："前面粗加阶段长时间 6-12mg/s，但是你也不提升幅度，我计划的粗加
        速率需要 20mg/s，你就一直改动幅度就行了，少了你就加，速度快了你就减"。

        推杆锁死、占空比按段固定，**振幅是快加/粗加唯一的流量执行器**。

        算法：**Δu = e / k_est**，u_next = clamp(u+Δu, 下限, 上限)
          e     = 目标速率 − 预测速率（预测 = 6s 回归均速 + 加速度×min(τ,3s)）
          k_est = 在线学到的 mg/s per V（1.0~80 之间的样本才收，EMA 0.5）
        每次小步累加等于自带积分，稳态误差归零；死区挡住天平噪声引起的抖；
        误差方向翻转（上一步跨多了）→ 本步减半，抑制在目标附近来回撞。

        区间与步长（用户 2026-10-23 11:50 定）：**基准 −0.5V ~ 基准 +0.5V，
        单步 0.05–0.15V（升降同）**。这里"基准"= 本段那一行的振幅（留空则 3.0/2.5/2.0）。
        不再放开到 +2.5V / 5.0V、也不再有 0.50V 升 / 0.60V 降的大步——振幅只在基准
        附近小步走，宁可慢一点不越带。真的顶到 基准+0.5V 还追不上，日志会写
        "加振幅已到极限，料斗可能搭桥/缺料"。

        决策顺序（每拍，命中即停）：
          0. 冻结 / 踢一脚进行中
          1. 踢脚 gate：三段统一——到带顶、停满 4s、这 4s 速率 <1 mg/s 才踢
          2. 响应窗：写幅后等 `_amp_wait_s`（默认 3s），锚在真实写幅时刻
          3. 测速：6s 回归均速；样本不够就拉长一倍；整窗零落料 = 断料 → 按 rate=0 加幅
          4. 预测 + 死区 max(1.5, 目标×8%)
          5. 步长：Δu 钳进 0.05–0.15V；误差反向时本步减半
          6. 慢加：只许升（且速率 <1mg/s 才升），≤0.10V/次，绝不追速率
        """
        cmd = self._rate_command_for(spec, mass)
        base = float(spec.amplitude_vpp)

        if self._taper_hold:
            # 看门狗：压到断料就撤销（见 `_taper_watchdog` 与 2026-10-26 11:17 事故）
            if self._taper_watchdog(spec, mass, elapsed):
                self._amp_heartbeat(
                    spec, mass, elapsed, cmd,
                    "切点前降幅窗口（暂停调幅，等余料走到切点）")
                return
            # 已撤销降幅：这一拍直接继续往下走，让速率环按现状重新评估
        if self._amp_saturated:
            self._amp_heartbeat(spec, mass, elapsed, cmd,
                                f"振幅已停手（{self._amp_lock_reason or '到边界'}）")
            return

        # -- 第零层：踢一脚进行中（卡粉救济的脉冲动作）--------------------------
        # 抬到高幅后什么都不判：这 0.5s 就是专门用来"震开"的，期间落料照常记
        # 采样（阶跃检测会更新 τ/T），但速率判据一律不做——高幅瞬态会把回归算毛。
        kick = self._amp_kick
        if kick is not None:
            if elapsed < float(kick["until"]):
                self._amp_heartbeat(
                    spec, mass, elapsed, cmd,
                    f"踢粉中：{kick['amp']:.2f}V 还剩 "
                    f"{float(kick['until']) - elapsed:.1f}s 后回落 {kick['base']:.2f}V")
                return
            self._amp_kick = None
            self._stall_wait_s = elapsed     # 冷却从回落这一刻起算（τ+T 后才许下一脚）
            self._amp_probe = None           # 踢脚是脉冲动作，不算微调增益样本
            self._prev_eval_rate = None      # 高幅瞬态会污染趋势闸
            # **踢完就回到踢之前的那个工作振幅，之后照旧由速率环管**（2026-10-27 11:44
            # 用户定："触发完回到原来的幅度即可，不需要记忆这个阶跃触发了门槛学习"）。
            # 旧版在这里开一个 τ+T 观察窗，只要窗内落了粉就把工作振幅上移到
            # "踢脚振幅−0.2V"（粗加基准 1.5V 会被抬到 2.8~3.0V），并顺手把踢脚振幅
            # 记成"起振门槛"——结果是踢过一脚以后振幅再也不回来：一次加粉里调 9 次、
            # 从 1.27V 一路跑到 3.0V（11:43 那次加粉粗加 11.93mg/s，目标 20）。
            # 踢脚只是脉冲，不是"这个粉需要多高电压"的证据，所以不学习、不上移。
            self._set_amplitude(
                float(kick["base"]), elapsed,
                f"踢一脚完成（{kick['amp']:.2f}V × {AMP_KICK_HOLD_S:.1f}s）→ "
                f"回到踢前工作振幅 {kick['base']:.2f}V（不做门槛学习，"
                f"后续仍由速率环按基准±带调整）")
            return

        # ── 速率闭环（2026-10-23 11:33 定，11:50 用户改回上限/步长口径）─────────
        # 快加/粗加的上界 = **标定 +0.5V**（同慢加），单步 0.05–0.15V；
        # 慢加保持老规矩（只在断料时往上爬一点点，绝不追速率）。
        slow_stage = spec.name == "slow"
        # 本段真实闭环带：一律从 `_amp_band_for` 取——页面显示的 amplitude_range_vpp
        # 和这里的 cap/floor 是同一个函数算的，永远不会各说各话（2026-10-29 统一）。
        # 慢加基准 2.00V 的默认带是 0.01–2.50V；档案 dispense_flags 给了
        # slow_amp_span_vpp 时（封堵剂2 = 1.0）变成 1.00–3.00V。
        floor_v, cap_v = _amp_band_for(spec)
        ceil_v = AMP_MAX_VPP if slow_stage else AMP_CTRL_CEIL_VPP
        # 下界**只看本段带**，不再被"学到的起振门槛"抬高（2026-10-27 11:28 用户定：
        # "起振的门槛就是由我来定的，就是我会输入的那个值"）。
        # 旧逻辑 `floor_v = max(带下界, 门槛−0.2)` 有两条硬伤：
        #   · 门槛是断料后一脚踢出来的瞬时值（3.5V 踢开 → 记 3.1V），粗加带顶只有
        #     2.0V，会算出 floor > cap，降幅动作 `max(floor_v, applied−du)` 反而比
        #     当前更高 —— 想往下收却往上冲；
        #   · 用户在页面把振幅填低（比如 1.5V），系统却按 3.1−0.2=2.9V 当地板，
        #     等于把页面输入架空。
        # 想让它更容易起振就填高页面那一行；降下去真不出料，由带顶停满 4s 的踢脚救。
        at_cap = float(self._applied_amp) >= cap_v - 0.015
        if at_cap:
            # 陈旧锚点防御（2026-10-27 13:35）：`_amp_at_cap_since` 比本次加粉 elapsed 还大
            # = 上一次加粉漏复位留下的 → 重新起算，否则"带顶满 4s"这条永远不成立、踢脚也
            # 就永远不开（和 `_stall_wait_s` 同一类 bug，一起兜住）。
            if (self._amp_at_cap_since is None
                    or float(self._amp_at_cap_since) > elapsed):
                self._amp_at_cap_since = elapsed
        else:
            self._amp_at_cap_since = None

        # -- 踢脚 gate（三段统一）：振幅已到带顶、顶上停满 4s、这 4s 几乎不落料 -----
        # 上限改回 标定+0.5V 以后，连续段能加的余量只有 0.5V，真断料时很快就会顶死；
        # 这时靠"踢一脚"（0.5s 高幅脉冲，不是连续抬幅）跨过起振门槛。
        # 不到带顶绝不踢——有流量时踢脚只会把速率打乱。
        meas4 = self._measure_rate(AMP_KICK_GATE_WINDOW_S, 1.0, skip_before=None)
        gate_rate = None if meas4 is None else float(meas4["rate"])
        if (at_cap
                and self._amp_at_cap_since is not None
                and elapsed - float(self._amp_at_cap_since) >= AMP_KICK_GATE_WINDOW_S
                and (gate_rate is None or gate_rate < AMP_KICK_GATE_RATE_MG_S)):
            if not spec.zero_rate_pulse:
                # 该粉标记"不许在加粉速率为 0 时脉冲"（2026-10-29 用户定，封堵剂2 专用）：
                # 顶上不踢脚。它起振即成团爆发，0.5s 高幅脉冲甩出来就是几十上百 mg，
                # 宁可顶着放宽后的振幅带（慢加 1.00–3.00V）继续连续振动把料逼出来；
                # 真一直不出料就交给 30s 断料中止，绝不用脉冲赌一把。
                self._amp_heartbeat(
                    spec, mass, elapsed, cmd,
                    f"已在振幅上限 {cap_v:.2f}V 停留 "
                    f"{elapsed - float(self._amp_at_cap_since):.0f}s、近 4s 速率 "
                    + (f"{gate_rate:.1f}" if gate_rate is not None else "≈0")
                    + f" < {AMP_KICK_GATE_RATE_MG_S:g} mg/s —— 该粉标记不许零速率脉冲，"
                    f"不踢脚（振幅带已放宽到 {floor_v:.2f}–{cap_v:.2f}V，再不出料将判断料中止）")
                return
            self._stall_relief(
                spec, mass, elapsed, cmd,
                detail=(f"已在振幅上限 {cap_v:.2f}V 停留 "
                        f"{elapsed - float(self._amp_at_cap_since):.0f}s，近 4s 速率 "
                        + (f"{gate_rate:.1f}" if gate_rate is not None
                           else "≈0（无有效回归）")
                        + f" < {AMP_KICK_GATE_RATE_MG_S:g} mg/s"))
            return

        # -- 响应窗：写幅后等 `_amp_wait_s`（大步 5s / 小步 3s）再评估 ----------
        # 判据锚在 `_amp_last_write_s`——它只在 `_set_amplitude` 真正写成功后才更新，
        # 所以"等响应"和"振幅确实变过"永远同步，不会出现"一直在等、其实没动过"。
        settled = elapsed - float(self._amp_last_write_s)
        if settled < float(self._amp_wait_s):
            self._amp_heartbeat(
                spec, mass, elapsed, cmd,
                f"等上一步生效（{settled:.1f}/{float(self._amp_wait_s):.1f}s；"
                f"τ≈{self._tau_est:.0f}s，缺口由加速度外推补）")
            return

        # -- 测速：6s 回归均速（脉冲式落料也能算；差分量在脉冲粉上是 0）--------
        meas = self._measure_rate(AMP_CTRL_LOOKBACK_S, 2.0, skip_before=None)
        last_drop = self._last_drop_s
        zero_flow = False
        if meas is None:
            # 窗内有落料只是样本不够回归 → 窗口拉长一倍再试（绝不拿短窗凑速率）
            if last_drop is not None and last_drop >= elapsed - AMP_CTRL_LOOKBACK_S:
                meas = self._measure_rate(AMP_CTRL_LOOKBACK_S * 2.0, 1.0,
                                          skip_before=None)
                if meas is None:
                    self._amp_heartbeat(
                        spec, mass, elapsed, cmd,
                        f"{AMP_CTRL_LOOKBACK_S:.0f}s 窗内有落料但样本不足"
                        f"（{elapsed - last_drop:.1f}s 前）—— 再等一拍")
                    return
            else:
                # 整个回看窗一粒粉都没落 = 断料/搭桥。**不能当成"没数据不动"**：
                # 这就是要加振幅的信号（v7 在这里每 3s 只抬 0.12V，等于没动）。
                zero_flow = True

        if zero_flow:
            rate = 0.0
            a_est = 0.0
            detail = f"近 {AMP_CTRL_LOOKBACK_S:.0f}s 零落料"
            self._v7_prev_rate = None
            self._v7_prev_t = None
            self._v7_a_est = 0.0
        else:
            rate = float(meas["rate"])
            detail = (f"均速 {rate:.1f} mg/s（{meas['span']:.1f}s/"
                      f"{meas['gain']:+.0f}mg/R²{meas['r2']:.2f}）")

            # -- 增益学习：上一次"调幅→观察"闭环走完了，算 k_sample -----------
            # k 只用来把缺口换算成步长（Δu = e/k），估小了步长偏大、估大了偏慢，
            # 所以只接受 1.0~80 mg/s per V 的样本，其余当成"这一发没响应"丢掉。
            probe = self._amp_probe
            if probe is not None:
                self._amp_probe = None
                delta_amp = float(probe["step"])
                delta_rate = rate - float(probe["rate"])
                if abs(delta_amp) >= 0.10:
                    k_sample = delta_rate / delta_amp
                    if 1.0 <= k_sample <= AMP_CTRL_K_MAX:
                        self._k_est = (round(k_sample, 2) if self._k_est is None
                                       else round(0.5 * float(self._k_est)
                                                  + 0.5 * k_sample, 2))
                        self._log(f"[振幅] 增益学习：{probe['rate']:.1f}→{rate:.1f} mg/s"
                                  f" / {delta_amp:+.2f}V → k_est={self._k_est:.1f} mg/s/V")

            # -- 加速度估计：相邻两次评估均速的变化率，EMA 平滑、限幅 --------------
            if self._v7_prev_t is not None and self._v7_prev_rate is not None:
                dt_eval = elapsed - float(self._v7_prev_t)
                if 1.0 <= dt_eval <= 15.0:
                    a_raw = (rate - float(self._v7_prev_rate)) / dt_eval
                    prev_a = 0.0 if self._v7_a_est is None else float(self._v7_a_est)
                    a_est = max(-AMP_ACCEL_MAX,
                                min(AMP_ACCEL_MAX, 0.5 * prev_a + 0.5 * a_raw))
                else:
                    a_est = 0.0 if self._v7_a_est is None else float(self._v7_a_est)
            else:
                a_est = 0.0
            self._v7_prev_rate = rate
            self._v7_prev_t = elapsed
            self._v7_a_est = a_est

        # -- 预测：粉末有纯滞后，把速率外推到"这次调幅生效时刻"再比较 ----------
        v_pred = rate + a_est * min(float(self._tau_est), 3.0)
        error = float(cmd) - v_pred
        deadband = max(AMP_CTRL_DEADBAND_ABS, abs(float(cmd)) * AMP_CTRL_DEADBAND_RATIO)
        if abs(error) <= deadband:
            self._amp_heartbeat(
                spec, mass, elapsed, cmd,
                f"{detail}，预测 {v_pred:.1f}/{cmd:.1f} mg/s（加速度 {a_est:+.1f}）"
                f"→ 缺口 {error:+.1f} 在死区 ±{deadband:.1f} 内，不动")
            return

        # -- 步长：Δu = 缺口 ÷ 增益，再钳进 0.05–0.15V（用户 2026-10-23 11:50 改回）---
        # 升降同一个上限 0.15V：振幅只贴着标定工况走（带 = 标定±0.5V），步子大了
        # 在 8s 纯滞后里叠两步就冲过带顶，而且离标定越远速率表越没参考价值。
        # 下限 0.05V 是为了别让"缺口÷增益"算出 0.02V 这种看不出变化的步子。
        k_eff = float(self._k_est) if self._k_est else AMP_CTRL_K_DEFAULT
        k_eff = max(AMP_CTRL_K_MIN, min(AMP_CTRL_K_MAX, k_eff))
        du = error / k_eff
        if du > 0:
            du_cap = (AMP_CTRL_UP_MAX_V if abs(error) > 0.15 * max(1.0, abs(float(cmd)))
                      else AMP_CTRL_UP_SMALL_V)
        else:
            # 慢加降幅步子放大到 0.30V（2026-10-27 11:02）：慢加段只有 5~15s，
            # 0.15V/步根本来不及从 2.0V 收到能压住速率的位置。
            du_cap = AMP_SLOW_DOWN_MAX_V if slow_stage else AMP_CTRL_DOWN_MAX_V
        du = max(-du_cap, min(du_cap, du))
        if du != 0.0 and abs(du) < AMP_CTRL_STEP_MIN_V:
            du = AMP_CTRL_STEP_MIN_V if du > 0 else -AMP_CTRL_STEP_MIN_V
        damp_note = ""
        if self._amp_last_dir is not None and du * float(self._amp_last_dir) < 0.0:
            # 误差方向与上次动作相反 = 上一步跨多了 → 本步减半，抑制在目标附近来回撞
            du *= 0.5
            damp_note = "，超调制动减半"

        if slow_stage:
            # 慢加：**升幅**只许"恢复出料"，绝不追速率（用户铁律，不动）；
            # 但**降幅必须敞开**——2026-10-27 11:02 修 bug。
            # 慢加段存在的全部意义就是把速率压到 5mg/s。实测封堵剂2 在 2.00V 跑
            # 19~24mg/s（超目标 4 倍），旧条件 `du <= 0` 把"降幅"也一起拦了，
            # 于是振幅一动不动、带着 4 倍速冲进验收带 → 偏重报废。
            # 升高会冲速率（所以照旧压住），降低只会断料（踢一脚能救），方向不对称。
            if du > 0:
                if rate >= AMP_KICK_GATE_RATE_MG_S:
                    self._amp_heartbeat(
                        spec, mass, elapsed, cmd,
                        f"{detail}——慢加不追速率（有粉就按实际速率收），振幅不动")
                    return
                du = min(du, AMP_SLOW_CLIMB_STEP_V)

        applied = float(self._applied_amp)
        if du > 0:
            # 升幅：上限 = 段上界；被救济抬到其上就沿当前值，不许拽回
            limit_high = max(cap_v, round(applied, 2))
            target_amp = round(min(limit_high, applied + du), 2)
        else:
            # 降幅：下限 = 本段带下界（基准−0.5；慢加 = 设备下限；封堵剂2 慢加 = 基准−1.0）。
            target_amp = round(max(floor_v, applied - abs(du)), 2)
        if abs(target_amp - applied) < 0.015:
            if error > 0 and applied >= cap_v - 0.015:
                # 到带顶分两种：还在落粉 = 这个基准下就到这儿了；整窗不落 = 真卡粉
                if zero_flow or rate < float(cmd) * AMP_STALL_RATE_RATIO:
                    self._amp_heartbeat(
                        spec, mass, elapsed, cmd,
                        f"{detail} → 已顶到振幅上限 {cap_v:.2f}V（基准+"
                        f"{cap_v - base:.2f}V），速率仍差 {error:.0f}mg/s"
                        f"——加振幅已到极限，料斗可能搭桥/缺料")
                else:
                    self._amp_heartbeat(
                        spec, mass, elapsed, cmd,
                        f"{detail} → 已顶到振幅上限 {cap_v:.2f}V（基准+"
                        f"{cap_v - base:.2f}V），差 {error:.0f}mg/s——"
                        f"振幅不再越带，本段按现有流量跑；要更快得改标定/推杆/转速")
            elif error < 0 and applied <= floor_v + 0.015:
                self._amp_heartbeat(
                    spec, mass, elapsed, cmd,
                    f"{detail} → 已到振幅下限 {floor_v:.2f}V（基准−{base - floor_v:.2f}V），"
                    f"速率仍超目标 {abs(error):.0f}mg/s——起始振幅设高了，本段按现有流量跑")
            else:
                self._amp_heartbeat(
                    spec, mass, elapsed, cmd,
                    f"{detail} → 已在区间边界 {floor_v:.2f}–{cap_v:.2f}V，不动")
            return
        step_v = round(target_amp - applied, 2)
        self._amp_changes += 1
        self._amp_probe = {"amp": applied, "rate": rate,
                           "step": step_v, "elapsed": elapsed}
        self._amp_last_dir = 1 if step_v > 0 else -1
        # 单步已统一钳在 0.05–0.15V，不再有"大步多等"的分支：一律按最小评估间隔走
        self._amp_wait_s = AMP_CTRL_MIN_EVAL_S
        if slow_stage:
            # 慢加段只有 5~15s：按 3s 一评估全程只能走 1~2 步，降幅等于没生效
            # （2026-10-27 实测：慢加 34 拍全程在"等上一步生效"）。慢加只往低走，
            # 步子限 0.30V，1.5s 一判足够。
            self._amp_wait_s = AMP_SLOW_EVAL_S
        if zero_flow:
            self._amp_wait_s = max(float(self._amp_wait_s), AMP_CTRL_ZERO_WAIT_S)
        self._ramp_last_s = elapsed
        self._set_amplitude(
            target_amp, elapsed,
            f"{STAGE_TEXT[spec.name]}速率闭环 {detail}，预测 {v_pred:.1f}/{cmd:.1f} mg/s"
            f"（缺口 {error:+.1f}，k≈{k_eff:.0f} mg/s/V）→ {target_amp:.2f}V"
            f"（{step_v:+.2f}V，第 {self._amp_changes} 次{damp_note}）",
            ceiling=ceil_v)

    def _stall_relief(self, spec: StagePlan, mass: float, elapsed: float,
                      cmd: float, detail: str = "") -> None:
        """卡粉/断料救济：踢一脚——振幅瞬间抬到 标定+1.0V，持续 0.5s 后立刻回落。

        用户 2026-10-22 13:31 定的招："模仿脉冲，突然变为 3V 持续 0.5s 即可回到
        2V，粉体一般就不卡了"。依据是 12:38 真机实测：2Vpp 短脉冲一粒不出、
        3Vpp 0.6s +37mg——一脚正好跨过起振门槛，比 0.5V/步逐级磨快得多；
        高幅只停 0.5s，也不会像连续抬幅那样把速率冲飞。
        节奏：一脚踢完**回到踢之前的那个工作振幅** → 等 τ+T（一个完整生效窗）看落料；
        还没恢复 → 下一脚在此基础上再 +AMP_KICK_ESCALATE_V；硬顶 = 标定+AMP_ESCALATE_V，
        到顶仍不出 → 停手提示查料斗。慢加段同样走这里（只许"恢复出料"，
        不许追速率，速率偏慢的场合在调用方已被拦下）。

        **踢脚只是脉冲，不改变工作点（2026-10-27 11:44 用户定）**："触发完回到原来的
        幅度即可，不需要记忆这个阶跃触发了门槛学习"。旧版踢完会开 τ+T 观察窗，窗内
        一落粉就把工作振幅上移到"踢脚−0.2V"并记成门槛，于是越踢越高（11:43 那次加粉粗加
        从 1.27V 一路跑到 3.0V、调 9 次，实测 11.93mg/s 而目标 20）。

        2026-10-22 13:56 实机抓到的两个坑（修复）：
        ① 锚点不能用 spec 标定值——当前工作振幅可能已经被速率环抬到标定之上
           （2.0→2.5V），按标定算踢脚会**往下踢**（2.50→2.00V），等于亲手把唯一
           能出料的振幅掐灭；必须锚定 max(标定, 当前实际值)。
        ② 救济通道不看 _amp_band——慢加的带退化成一点 (2.0, 2.0)，会把踢脚
           顶死在标定值上（日志"顶 2.00V"）。踢脚上限 = 标定+AMP_ESCALATE_V，
           与微调带无关。

        **粉级封锁（2026-10-29 用户定）**：``spec.zero_rate_pulse = False`` 的粉
        （目前只有封堵剂2）任何情况下都不走这条路径——它的补偿手段是放宽后的
        振幅带，不是脉冲。这里再做一次防御，防止以后有别的分支绕过调用方的判断。
        """
        if not spec.zero_rate_pulse:
            self._amp_heartbeat(
                spec, mass, elapsed, cmd,
                "该粉标记 zero_rate_pulse=false：不走零速率脉冲救济（踢脚）")
            return
        base = float(spec.amplitude_vpp)
        if spec.name == "slow":
            # 慢加踢脚锚点 = 标定值（用户 13:31 原方：2V 标定就踢到 3V×0.5s）。
            # 慢加连续振幅已被钳在 标定+0.5V 以内，标定+1V 起踢保证永远向上踢、
            # 也不会随工作点上移把踢脚越滚越高。
            anchor = base
        else:
            anchor = max(base, float(self._applied_amp))   # 坑①：锚定当前实际振幅
        cap = round(base + AMP_ESCALATE_V, 2)          # 坑②：与微调带无关的硬顶
        # 冷却判定（2026-10-27 13:35 加负值兜底）：正常口径是"上一脚回落后再等 τ+T"。
        # 事故回顾：13:29 那次加粉 `_stall_wait_s` 带着上一次加粉的 189s 进新的一次加粉，新的一次加粉 elapsed
        # 才 41s → `elapsed - 189` 为负、恒小于 τ+T → 每拍都判"冷却中"，一脚都不踢，
        # 振幅卡在带顶、速率 0，只能人工停。所以这里遇到"锚点比现在还晚"一律当
        # "本次加粉还没踢过"处理并清掉旧锚点，绝不允许陈旧时间戳锁死救济。
        cooldown_s = float(self._tau_est) + float(self._period_est)
        stall_since = elapsed - float(self._stall_wait_s)
        if stall_since < 0.0:
            self._stall_wait_s = -99.0
            stall_since = elapsed + 99.0
        if stall_since < cooldown_s:
            self._amp_heartbeat(
                spec, mass, elapsed, cmd,
                f"救济冷却中（等 τ+T = {cooldown_s:.0f}s "
                f"看上一脚有没有踢开；已过 {stall_since:.0f}s）")
            return
        tries = int(self._amp_kick_tries)
        kick_amp = round(min(cap, anchor + AMP_KICK_DELTA_V
                             + tries * AMP_KICK_ESCALATE_V), 2)
        # ★ 2026-10-27 16:25 修 bug（用户："脉冲信号只作用了一次之后再也不会触发"）：
        # 旧判据是"算出来的这一脚振幅没有比上一脚更高 → 判踢粉到顶、永久停手"。
        # 但锚点 anchor 已经被速率环抬到带顶时（2.50V），**第一脚** 2.50+1.00=3.50V
        # 正好等于硬顶 3.50V，于是第二脚算出来还是 3.50V ≤ 上一脚 → 一脚都不再踢。
        # 16:19 那次加粉润滑剂1 就是这样：kick_count 停在 1，后面每拍只写"振幅已停手
        # （踢粉到顶 3.50V 仍不出料）"——可那一脚其实踢出了 9mg，料道是通的。
        # 新口径：硬顶只意味着"振幅不再往上加"，**在硬顶这一档可以反复踢**；
        # 判死改成看"这一脚到底踢出来没有"——连续 AMP_KICK_MAX_EMPTY 脚空才停手。
        capped = tries > 0 and kick_amp <= float(self._amp_kick_last_amp) + 0.015
        if capped:
            gain_total = float(mass) - float(self._amp_kick_m0_total)
            if tries >= AMP_KICK_MAX_TRIES:
                why = f"本段已踢 {tries} 脚仍追不上（振幅到硬顶，无法再高）"
            elif (tries >= AMP_KICK_MIN_TRIES_BEFORE_STOP
                    and gain_total < AMP_KICK_MIN_TOTAL_GAIN_MG):
                why = (f"到顶后踢了 {tries} 脚累计只涨 {gain_total:.0f}mg"
                       f"（< {AMP_KICK_MIN_TOTAL_GAIN_MG:.0f}mg）")
            else:
                why = ""
            if why:
                self._amp_saturated = True
                self._amp_lock_reason = f"踢粉到顶 {cap:.2f}V：{why}"
                self._log(f"踢粉已经到顶 {cap:.2f}V（标定 {base:.2f}V "
                          f"+{AMP_ESCALATE_V:.2f}V）：{why}"
                          + (f"（{detail}）" if detail else "")
                          + "——请检查料斗是否搭桥/缺料、推杆开度是否够。振幅不再往上加")
                return
            kick_amp = cap          # 振幅不再加，就在硬顶这一档继续踢
        self._stall_wait_s = elapsed
        self._amp_kick = {"amp": kick_amp, "base": float(self._applied_amp),
                          "until": elapsed + AMP_KICK_HOLD_S, "fired": elapsed,
                          "m0": float(mass)}
        self._kick_count += 1
        self._amp_kick_tries = tries + 1
        self._amp_kick_last_amp = kick_amp
        if tries == 0:
            # 本段第一脚：记下累计增重的起点（后面判"这几脚到底有没有出料"）
            self._amp_kick_m0_total = float(mass)
        self._amp_changes += 1
        self._amp_probe = None          # 踢脚是脉冲动作，不算微调增益样本
        self._prev_eval_rate = None     # 高幅瞬态会污染趋势闸
        mode = "卡粉救济" if spec.name == "slow" else "断料救济"
        self._set_amplitude(
            kick_amp, elapsed,
            f"{STAGE_TEXT[spec.name]}{mode}：踢一脚 → 瞬间抬到 {kick_amp:.2f}V "
            f"持续 {AMP_KICK_HOLD_S:.1f}s（第 {tries + 1} 脚，顶 {cap:.2f}V"
            f"{'，已在硬顶重复踢' if capped else ''}）"
            + (f"，{detail}" if detail else ""))


    def _trim_stir_on(self) -> None:
        """缓补脉冲起搅拌：粉要靠搅拌才能从腔里推出来，但只允许转脉冲这一下。

        2026-10-23 用户口径："慢加结束后搅拌立即停止，等天平稳定后再缓补"——
        出料窗口（搅拌转 + 振动开）和称重窗口（都停）必须分开，否则称出来的增重里
        永远混着搅拌的持续输送（润滑剂1 关了振动照样 5mg/s），脉冲量既不准也停不下来。
        """
        if self._trim_stir_rpm <= 0.0:
            return
        try:
            self.hub.stir_start(self._trim_stir_rpm, acc=self.STIR_ACC, direction=0)
            self._applied_stir = self._trim_stir_rpm
        except Exception as error:          # noqa: BLE001 - 起不来就只靠振动出料
            self._log(f"缓补起搅拌失败（{error}），本发只靠振动出料")

    def _trim_stir_preroll(self, seconds: float) -> None:
        """起搅拌后先空转一会儿（振动**不开**）再发猝发。

        必须有的原因（2026-10-23 11:23 那次 400mg 的真正死因之一）：Emm42 acc=200
        到 100RPM 约 0.28s 爬升，而缓补脉冲只有 0.15~0.6s——不预转的话大半振动
        落在"转子还没把腔口那层粉推到位"的空档里。实测对照：0.15s 脉冲清一色
        +0mg，0.6s 脉冲能出 5~6mg。预转让整段猝发都发生在额定转速上。

        代价：润滑剂1 这类粉搅拌一转就往下落（~5mg/s），0.35s 约 2mg，
        这段增重会计进本发称重（称重窗口在脉冲之后），所以不影响账目平衡。
        """
        if seconds <= 0.0 or self._applied_stir <= 0.0:
            return
        end = time.monotonic() + float(seconds)
        while not self._stop_event.is_set():
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                return
            if self._stop_event.wait(min(0.05, remaining)):
                return

    def _trim_watch_during_pulse(self, seconds: float, watch_at: float) -> bool:
        """脉冲期间盯秤，读数到 ``watch_at`` 立刻掐输出；返回 True 表示是提前掐的。

        为什么要有刹车：不出粉时我们会把振幅抬到 5V、脉冲拉到 2.5s，一旦料突然下来，
        4V 满流就有 28mg/s，2.5s 能冲出 70mg——一发就把整次加粉冲爆。秤有滞后
        （粉还在空中），所以 watch_at 要留 ``TRIM_PULSE_WATCH_MARGIN_MG`` 余量。
        """
        end = time.monotonic() + max(0.0, float(seconds))
        while not self._stop_event.is_set():
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                return False
            if self._stop_event.wait(min(TRIM_PULSE_WATCH_S, remaining)):
                return False
            try:
                mass = float(self._read_balance_raw()) - self._baseline
            except Exception:                  # noqa: BLE001 - 读不到就继续等周期走完
                continue
            if mass >= watch_at:
                try:
                    self.hub.sig_output(False)      # Wc0s 立即关输出，猝发也认这个
                except Exception as error:          # noqa: BLE001 - 关不掉交给上层兜底
                    self._log(f"脉冲提前掐断时关输出失败：{error}")
                return True
        return False

    def _trim_amp_down_floor(self) -> float:
        """缓补降幅的地板。

        不能降到"已知振不出粉"的死点附近：那次加粉 2.60V 一粒不出、3.10V 才出，
        降到 2.60V 只会白振一发再把振幅加回去（拉锯）。所以地板 =
        ``_trim_amp_dead + TRIM_AMP_DEAD_MARGIN_V``，再兜一个 ``TRIM_AMP_FLOOR_VPP``。
        """
        return round(max(TRIM_AMP_FLOOR_VPP,
                         float(self._trim_amp_dead) + TRIM_AMP_DEAD_MARGIN_V), 2)

    def _trim_escalate_on_no_gain(self, count: int, elapsed: float, no_gain: int,
                                 duration: float, preroll: float) -> tuple[float, float]:
        """一发完全没出粉之后的升级策略，返回下一发的（脉冲时长, 预转时长）。

        顺序（2026-10-23 12:16 用户纠正后的口径）：
        1) 振幅没到顶 → **每发只抬 0.5V**（2.2 → 2.7 → 3.2 …），顶 ``TRIM_AMP_MAX_VPP(5.0V)``。
           曾经是"首发没出直接跳 4.0V"（11:24 那版），现场被否：起振门槛往往只高半档，
           跳到 4V 一次就多冲 20~30mg（4V 满流实测 28mg/s）。用户原话：
           "前两次是 2.2V，最后直接到了 4V，不应该这样，而是调节到 2.7V 即可"。
        2) 振幅已到顶还不出 → 不是"振不动"，是"腔里没料"：**只拉预转**（+0.5s，让搅拌
           多带一会儿下来），脉冲时长仍是恒定的 ``TRIM_PULSE_S(0.40s)`` 一动不动
           （2026-10-26 10:32 用户：全部 0.4s、只变幅度）。**绝不缩短脉冲**——
           旧版这里缩到 0.15s，连转速爬升都没走完，越缩越不出，六发全废。
        3) 抬档只是试探（``_trim_pulse_escalated``）：下一发只要出了粉，就按用户
           2026-10-26 的规矩退回去——**一次退 1.0V**（3.5→2.5、4.0→3.0），
           不许把试探档当工作点；只有振幅 >3.0V 才触发（≤3.0V 视为稳定工作点）。
        """
        cur = float(self._applied_amp)
        # 这个振幅是"已知振不出粉"的死点，给降幅定地板时不能降到它附近去
        self._trim_amp_dead = max(float(self._trim_amp_dead), cur)
        # 下一发是在"刚抬上来的新档位"上发的：只要出粉就必须退回去（2026-10-26）
        self._trim_pulse_escalated = True
        new_amp = cur
        if cur < TRIM_AMP_MAX_VPP - 0.01:
            new_amp = min(TRIM_AMP_MAX_VPP,
                          round(cur + TRIM_AMP_NO_GAIN_STEP_VPP, 2))
            why = (f"缓补第 {count} 发没出粉：振幅 {cur:.2f}→{new_amp:.2f}V"
                   f"（+{TRIM_AMP_NO_GAIN_STEP_VPP:.1f}V 一档，顶 "
                   f"{TRIM_AMP_MAX_VPP:.1f}V）")
            self._set_amplitude(new_amp, elapsed, why, ceiling=TRIM_AMP_MAX_VPP)
        else:
            preroll = min(TRIM_STIR_SPINUP_MAX_S, preroll + TRIM_STIR_SPINUP_GROW_S)
            # 脉冲时长恒定 0.40s，到顶也不拉长（2026-10-26 10:32 用户）：
            # 只让搅拌多带一会儿，振动本身不多给一个周期。
            self._log(f"缓补第 {count} 发没出粉而振幅已到顶 {cur:.2f}V——"
                      f"腔里没料，预转加到 {preroll:.2f}s 再试"
                      f"（脉冲仍固定 {TRIM_PULSE_S:.2f}s）")
        return duration, preroll

    def _trim_stir_off(self, why: str = "") -> None:
        """脉冲一结束立刻停搅拌（**立即停**，不走减速）：多转一秒就是多落几 mg。"""
        if self._applied_stir <= 0.0:
            return
        try:
            self.hub.stir_stop("immediate")
        except Exception as error:          # noqa: BLE001 - 停不下交给收尾兜底
            self._log(f"缓补停搅拌失败（{error}）")
        self._applied_stir = 0.0
        if why:
            self._log(f"搅拌立即停止：{why}")

    def _trim_wait_settled(self, note: str, *,
                           min_s: float = TRIM_SETTLE_MIN_S,
                           max_s: float = TRIM_SETTLE_MAX_S,
                           stage: str = "trim",
                           abort_at: float | None = None) -> tuple[float, float]:
        """脉冲后称重：**至少等 min_s、最多等 max_s**，判稳了也要等满 min_s 才返回。

        2026-10-23 用户口径："缓补每缓补一次等待 2-3s 根据重量再进行缓补"。
        下限 2s 的物理原因：猝发刚停时秤的极差很快进 2mg，但粉还在落料口往下走，
        0.8s 判稳会把"暂时的平台"当成"落完了"——那次加粉 205mg 判稳收工、收尾又涨到 211mg，
        偏重 11mg 出验收。等满 2s 再判，单发到底落了多少才算得准。
        上限 3s：一直不稳（称台在晃/粉流没断）就按当前读数走，卡死比差 1mg 更糟。
        期间持续 _record，页面和 run 文件都能看到缓补段的采样。

        ``abort_at``：跑到这个质量就立刻收，不再等满 max_s（2026-10-23 11:09 那次加粉：
        润滑剂1 关振后搅拌还在以 ~5mg/s 推尾料，入口等 8s 一路从 186 涨到 228，
        页面看着像"一直在缓补"，其实一颗脉冲都没发。已经超了就没必要继续等。）
        """
        start = time.monotonic()
        window: list[float] = []
        mass = 0.0
        elapsed = start - self._t0
        while not self._stop_event.is_set():
            now = time.monotonic()
            waited = now - start
            raw = self._read_balance_raw()
            mass = float(raw) - self._baseline
            elapsed = now - self._t0
            window.append(mass)
            if len(window) > 10:
                window.pop(0)
            self._record(elapsed, mass, 0.0, stage, learn=False)
            if abort_at is not None and mass >= abort_at:
                self._log(f"{note}：等了 {waited:.2f}s 就到 {mass:.1f}mg"
                          f"（≥ {abort_at:.0f}mg 停手线）——这段增重是空中/腔内的存料"
                          f"落下来的，不是脉冲打出来的，提前结束等待")
                return mass, elapsed
            if waited >= max_s:
                self._log(f"{note}：等满 {waited:.1f}s 未判稳，按当前读数 {mass:.1f}mg 继续")
                return mass, elapsed
            if (waited >= min_s and len(window) >= 8
                    and max(window) - min(window) <= TRIM_STABLE_RANGE_MG):
                return mass, elapsed
            self._stop_event.wait(0.1)
        return mass, elapsed

    def _trim_pulse_loop(self, mass: float, elapsed: float, target: int,
                         source: str = "slow", *,
                         hold_amp: bool = False,
                         amp_start: float | None = None) -> str:
        """缓补主循环：开振 pulse_s 秒 → 关振 → 等落定称增重 → 自适应下一发时长。

        ``hold_amp`` / ``amp_start``（2026-10-27 13:46 用户）：**缓补开关关闭时**的
        收尾定幅补差。用户看到失水剂2 那次加粉 1987/2000mg（低验收线 1990）直接判偏轻收场，
        原话："没有开缓补功能的加粉，假如最后停机时质量少了，进行缓补，比如这个就
        0.30V 50RPM 搅拌缓补 0.4s 即可到达要求"。
        这个模式与正常缓补的区别只有两点：
          ① 振幅**锁定**在 ``amp_start``（页面慢加那一行，这次加粉 0.30V），整段不抬档；
          ② 没出粉只拉长搅拌预转，不往上试探（"只补不试探"），出粉也不退档。
        其余（0.40s 恒定时长、每发称重、进验收带就停手）与正常缓补完全一致。
        设 ``hold_amp=False`` 时行为与原来一模一样（慢加尾段缓补、trim 开启的补差）。

        为什么用"短促开振"而不是压低振幅连续出：剩余 20~40mg 时慢加的连续流
        落点误差有 ±几 mg（惯性余料 + 判停滞后），对 ±10mg 的验收太毛；
        脉冲把"出料"和"称重"在时间上完全分开，每一发落多少天平说了算。

        脉冲时长固定、出多少全靠振幅（2026-10-26 10:32 用户："全部设为 0.4 秒，
        不变，只变幅度即可，慢慢出就行"）：每发都是 ``TRIM_PULSE_S = 0.40s``，
        没有 ×1.5 / ×0.65 / 按剩余量等比缩短 / 加多了砍半这套。
        **一发完全没出粉不像旧版那样缩脉冲**（缩到 0.15s 连搅拌转速爬升都没走完，
        越缩越不出），改成抬振幅——**每发只抬 0.5V**（2.2 → 2.7 → 3.2 …，
        2026-10-23 12:16 用户纠正"不许一步跳到 4V"），顶 ``TRIM_AMP_MAX_VPP(5.0V)``；
        振幅到顶还不出，只拉长搅拌预转（脉冲仍是 0.40s）。连续 6 发不出才停手。

        **振幅也会往下走**（2026-10-23 16:05 用户："同一幅度第二次脉冲要根据上次的
        增重来判断，这次同一幅度加多了太多，应该稍微减小一些幅度"）：同一振幅下
        剩的量已经不到上发增重的 1.5 倍 → 发之前先降 0.3V；同一振幅下这一发增重
        达到上一发的 1.8 倍（1200mg 那次加粉 3.10V：+12mg 之后 +38mg）→ 判放量，降 0.3V。
        地板 = 最近一次"完全没出粉"的振幅 + 0.25V，降不到死点上去白振。

        **抬档后一出粉就退档**（2026-10-26 10:18 用户硬规矩）：抬振幅只是"试探能不能
        出料"，试探成功就必须退回低档去加。2000mg 那次就是"4.00V 第一发 +16mg、
        第二发 +38mg"冲出去的，旧版把 4.0V 当成了工作点。退档到某档出料 ≤18mg 就
        停在那档（``_trim_amp_ok``），连退两次都回到同一档（说明退下去那档一粒不出）
        也不再试探。

        **回退触发线 3.0V + 一次退 1.0V**（2026-10-26 10:26 / 10:44 用户）：
        只有 >3.0V 出粉才回退（≤3V 当稳定工作点，不触发）；退的目标 =
        ``max(TRIM_AMP_FLOOR_VPP, 当前 − 1.0V)``，**可以退到 3V 以下**——
        3.5V 出了就回 2.5V、4.0V 出了回 3.0V、4.5V 回 3.5V。
        退下去那档若一粒不出会走"抬 0.5V"回来，由 ``_trim_backoff_target`` 防抖收敛
        （同一次加粉不会退到同一档两次：退过没出 → 直接认定当前档是工作点）。

        **脉冲时长恒 0.40s**（2026-10-26 10:32 用户："全部设为 0.4 秒，不变，只变幅度即可，
        慢慢出就行"）：不再按增重自适应（×1.5 / ×0.65）、不按剩余量比例缩短、加多了也不
        砍半。**唯一的调节量就是振幅**，出得慢就慢慢发几发。只有脉冲中盯秤到
        ``目标−3mg`` 会提前掐断（安全刹车，不是时长规则）。

        节奏（2026-10-23 用户定）：每发之后**等 2~3s 再称重**（见
        ``_trim_wait_settled``），按称出来的重量决定**还要不要发**（时长恒 0.40s，
        不再由称重结果改）；一旦质量进验收窗口（目标 ±10mg）立刻停手，偏轻也不补。

        搅拌开关（2026-10-23 11:16 用户要求："慢加结束后搅拌立即停止，等天平稳定后再缓补"）：
        进缓补先 `stir_stop("immediate")` 等尾料落定，然后**每发只在脉冲那一下转搅拌**
        （`_trim_stir_on` → 预转 → 开振 → `_trim_stir_off` → 称重）。润滑剂1 在 100RPM 下
        关着振动也落 ~5mg/s，搅拌常转就永远等不到稳定，称出来的增重全是假的。
        """
        # 停手判据 = 验收窗口（目标 ±10mg）。2026-10-23 用户："达到 10mg 范围内不再缓补"——
        # 偏轻 10mg 以内也停：单发脉冲本身就是 10mg 量级，为凑最后 3mg 再发一发
        # 反而把结果顶到 +10mg 之外，那是自己把自己搞出验收。
        lower, upper = acceptance_limits(target)
        # 缓补只有两个入口：慢加剩 ≤40mg 正常进、判停偏轻补差（source=topup）。
        # 熔断**不再**走到这里（2026-10-26 18:35 用户："熔断只发生在阶段切换之间"）。
        label = "补差" if hold_amp else STAGE_TEXT["trim"]
        if hold_amp:
            want_amp = float(amp_start) if amp_start is not None else float(self._applied_amp)
            self._log(f"进入{label}：{mass:.1f}/{target} mg（剩 {target - mass:.0f}mg）——"
                      f"停连续振动，每发固定 {TRIM_PULSE_S:.2f}s、"
                      f"搅拌 {float(self._applied_stir):.0f}RPM，"
                      f"**振幅锁定 {want_amp:.2f}V**（页面慢加那一行，"
                      f"没出粉也不抬档、只加长预转），补进验收 "
                      f"{lower:.0f}~{upper:.0f}mg 就停手")
        else:
            self._log(f"进入{label}：{mass:.1f}/{target} mg（剩 {target - mass:.0f}mg）——"
                      f"停连续振动，每发固定 {TRIM_PULSE_S:.2f}s 的脉冲补料（出多少只看振幅）"
                      f"（振幅从 {self._applied_amp:.2f}V 起：没出粉每发只抬 "
                      f"{TRIM_AMP_NO_GAIN_STEP_VPP:.1f}V、顶 {TRIM_AMP_MAX_VPP:.1f}V；"
                      f"振幅 >{TRIM_AMP_BACKOFF_MIN_VPP:.1f}V 出了粉就回退 "
                      f"{TRIM_AMP_BACKOFF_STEP_VPP:.1f}V（3.5→2.5、4.0→3.0，可退到 3V 以下）；"
                      f"≤{TRIM_AMP_BACKOFF_MIN_VPP:.1f}V 不触发回退；"
                      f"单发 >{TRIM_GAIN_TOO_MUCH_MG:.0f}mg 算加多了）")
        if self._vib_running:
            self.hub.sig_output(False)
            self._vib_running = False
        # 慢加结束 → 搅拌**立即停**（2026-10-23 用户要求）：润滑剂1 这种粉在 100RPM 下
        # 振动关着也以 ~5mg/s 往下塌（腔里的料被搅拌推出去），搅拌不转，"等落定"才等得到
        # 真正的稳定。脉冲期间才临时把搅拌带起来（见 `_trim_stir_on`）。
        self._trim_stir_rpm = float(self._applied_stir)
        # 定幅补差：先把振幅摆到调用方给的值（页面慢加那一行）再开始等料。
        # 此时振动是关的，`_set_amplitude` 只改信号源幅度、不输出，安全。
        if amp_start is not None and abs(float(amp_start) - float(self._applied_amp)) > 0.01:
            self._set_amplitude(
                float(amp_start), elapsed,
                f"收尾补差：振幅取慢加那一行 {float(amp_start):.2f}V"
                f"（原 {self._applied_amp:.2f}V，整段锁定不抬档）",
                ceiling=TRIM_AMP_MAX_VPP)
        # ★ 2026-10-26 18:04 现场（用户："慢加之后的缓补好像没有等待粉末沉积"）——
        # 旧顺序是"停振 → 立刻 stir_stop(immediate) → 称重"。`stir_stop` 走串口要
        # ~0.4s，这 0.4s 里秤一直在涨而程序什么都没做；那次加粉 1978mg 进来，停搅拌回来
        # 第一次读数就是 **2048mg（+70mg）**，直接越过 2010 的停手线，缓补 0.2s 结束、
        # 0 发脉冲。所以"等待"在代码里存在，但被夹在两次串口动作中间，等于没等。
        # 现在把它拆成两段，各自都能在页面/CSV 上看见：
        #   ① 停振、**搅拌保持** TRIM_ENTRY_HOLD_S：先看腔内存料会不会自己泄；
        #   ② 再停搅拌，继续等落定 TRIM_TAIL_SETTLE_MAX_S。
        # ①的存在也让"崩落到底是停振引起的、还是停搅拌引起的"有了判据。
        entry_mass = mass
        if self._trim_stir_rpm > 1.0:
            mass, elapsed = self._trim_wait_settled(
                "进缓补：已停振、搅拌保持，先看腔内存料泄不泄",
                min_s=TRIM_ENTRY_HOLD_S, max_s=TRIM_ENTRY_HOLD_S, abort_at=upper)
            hold_gain = mass - entry_mass
            if abs(hold_gain) >= 1.0:
                self._log(f"进缓补：停振后搅拌保持 {TRIM_ENTRY_HOLD_S:g}s，"
                          f"腔内存料泄了 {hold_gain:+.0f}mg"
                          f"（{entry_mass:.0f} → {mass:.0f}mg，还没停搅拌）")
        self._trim_stir_off(why=f"{label}前先停搅拌")
        trim_spec = TrimSpec(stir_rpm=self._trim_stir_rpm, rod_ff=int(self._applied_window),
                             amplitude_vpp=float(self._applied_amp), source=source)
        self._start_stage_log(trim_spec, mass, elapsed)
        # 先把慢加的尾料等干净：尾料落定算进缓补的账，第一发脉冲才量得准。
        # 尾料增重必须显式记一条——慢加冲快时尾料可能有几十 mg（17:27 那次
        # 3V 慢加尾料 67mg），页面只看到"0 发脉冲质量却在涨"就没法对账。
        # 注：`entry_mass` 是**停振那一刻**的质量（上面 ① 之前记的），所以这里的
        # tail_gain 是"停振 + 停搅拌"两段泄料的合计，不是只算最后一段。
        mass, elapsed = self._trim_wait_settled("进缓补：搅拌已停，等尾料落定",
                                                max_s=TRIM_TAIL_SETTLE_MAX_S,
                                                abort_at=upper)
        tail_gain = mass - entry_mass
        if tail_gain >= 1.0:
            self._log(f"缓补入口：慢加尾料落定 +{tail_gain:.0f}mg（停振后含停搅拌全程）"
                      f"（当前 {mass:.1f}/{target}，剩 {max(0.0, target - mass):.0f}mg"
                      f"{'——尾料已够，不再发脉冲' if target - mass <= 0 else ''}）")
        elif tail_gain <= -1.0:
            self._log(f"缓补入口：尾料净减 {tail_gain:.0f}mg"
                      f"（当前 {mass:.1f}/{target}，天平可能被碰过/清零漂移）")
        # ── 存粉保护（2026-10-28 14:30 用户）────────────────────────────────
        # 入口观察窗泄料多 = 料道里挂着存货（抑制剂 3000mg 那次：粗加断流搅拌 150
        # 一直转，判停时 2987 差 3mg，之后任何搅拌都会倾泻，收尾涨到 3052）。
        # 此时脉冲**不起搅拌**，低幅振动试探、慢慢加幅，让存粉细水长流地出。
        stock_mode = (not hold_amp) and tail_gain >= TRIM_STOCK_GAIN_MG
        stock_amp_restored = float(self._applied_amp)   # 转回正常模式时恢复用
        if stock_mode:
            start_amp = max(TRIM_AMP_FLOOR_VPP, TRIM_STOCK_AMP_START_VPP)
            self._log(f"**存粉保护生效**：入口泄料 {tail_gain:.0f}mg ≥ "
                      f"{TRIM_STOCK_GAIN_MG:.0f}mg，料道挂着存货——脉冲不起搅拌"
                      f"（搅拌一丢就倾泻），振幅从 {start_amp:.2f}V 起低幅试探，"
                      f"没出粉每发只 +{TRIM_STOCK_AMP_STEP_VPP:.1f}V 慢慢加，"
                      f"出了粉就停在这档")
            if abs(start_amp - float(self._applied_amp)) > 0.01:
                self._set_amplitude(start_amp, elapsed,
                                    f"存粉保护：振幅降到 {start_amp:.2f}V 起步"
                                    f"（原 {self._applied_amp:.2f}V），"
                                    f"靠振动少量带出存粉",
                                    ceiling=TRIM_AMP_MAX_VPP)
        # 页面顶部那句话到这里必须换掉：旧版整个脉冲段一次 `_set_state` 都没有，顶部会
        # 一直冻在上一个相位的文案（例如"停振等余料 xxx/2000mg"），看着就像程序卡住/跳段。
        self._set_state("running",
                        f"{label}：{mass:.0f}/{target}mg（剩 {max(0.0, target - mass):.0f}mg）"
                        f"——停连续振动，一发一称补足")
        duration = TRIM_PULSE_S
        preroll = TRIM_STIR_SPINUP_S
        count = 0
        no_gain = 0
        # 停手线：正常缓补就挂在下验收线上（一发 10mg 量级，多补一发就顶出去）；
        # 定幅补差多发一档就抬到"下界 +2mg"——收尾 `_settle` 还会再掉 1~2mg，
        # 正好停在 lower 上时一称就掉出带（2026-10-27 仿真复现 199 → 198）。
        stop_at = float(lower) if not hold_amp else min(float(upper), float(lower) + 2.0)
        while not self._stop_event.is_set():
            remaining = target - mass
            if mass >= stop_at:
                if mass > upper:
                    why = f"已超验收上限 {upper:.0f}mg（超目标 {mass - target:.0f}mg）"
                elif remaining <= 0.0:
                    why = f"已超目标 {mass - target:.0f}mg"
                else:
                    why = f"剩 {remaining:.0f}mg 已在验收窗口 {lower:.0f}~{upper:.0f}mg 内"
                self._log(f"缓补停手：{mass:.1f}/{target} mg，{why}——不再发脉冲")
                break
            if count >= TRIM_PULSE_MAX_N:
                self._log(f"缓补脉冲已达 {TRIM_PULSE_MAX_N} 发上限（累计 "
                          f"{mass:.1f}/{target} mg）——停手保护，不再发")
                break
            before = mass
            # ① 基准 = **同一振幅下**上一发的实测增重（换过振幅就作废，回 3mg 保守）。
            #    用户 16:05："同一幅度第二次脉冲要根据上次的增重来判断"。
            same_amp = (self._trim_last_gain >= 1.0
                        and abs(float(self._trim_last_gain_amp)
                                - float(self._applied_amp)) < 0.02)
            ref_gain = float(self._trim_last_gain) if same_amp else 3.0
            # ② 剩余量已经跟"这档振幅上一发的产出"差不多 → **发之前先降一档**。
            #    1200mg 那次加粉就是这么超的：剩 13mg、上一发同振幅 3.10V 出 12mg，
            #    照发一整发 3.10V，结果这一发出了 38mg（同振幅出料量差 3 倍）。
            #    **只在稳定线（3.0V）以上才降**：≤3V 是稳定工况，降下去就空振，
            #    剩量少靠 ③ 的脉冲缩短来压（2026-10-26 10:26 用户）。
            if (not hold_amp and not stock_mode and same_amp
                    and remaining < TRIM_AMP_OVER_RATIO * ref_gain
                    and float(self._applied_amp) > TRIM_AMP_BACKOFF_MIN_VPP + 0.01):
                floor_v = max(self._trim_amp_down_floor(),
                              TRIM_AMP_BACKOFF_MIN_VPP)
                new_amp = round(max(floor_v,
                                    float(self._applied_amp) - TRIM_AMP_DOWN_STEP_VPP), 2)
                if new_amp < float(self._applied_amp) - 0.01:
                    self._set_amplitude(
                        new_amp, elapsed,
                        f"缓补剩 {remaining:.0f}mg 已接近这档振幅上一发的产出 "
                        f"{ref_gain:.0f}mg（{self._applied_amp:.2f}V）——先降 "
                        f"{TRIM_AMP_DOWN_STEP_VPP:.1f}V 再发，防冲过头",
                        ceiling=TRIM_AMP_MAX_VPP)
                    ref_gain = 3.0          # 换了振幅，上一条基准立刻作废
                    same_amp = False
            # ③ 脉冲时长**恒定**：不管剩多少 mg，都是 0.40s 一发（2026-10-26 10:32 用户：
            #    "全部设为 0.4 秒，不变，只变幅度即可，慢慢出就行"）。
            #    旧版这里按 remaining/ref_gain 等比缩短，分母还写错过名义 10mg——
            #    现在剩余量小就靠"少发几发 + 脉冲中盯秤掐断"兜底，不再改时长。
            pulse_s = duration
            fired = False
            # 出料窗口开始：先起搅拌并**预转**到额定转速，脉冲结束立刻停。
            # 存粉保护模式下**不起搅拌**（搅拌一丢存粉就倾泻，2026-10-28 14:30
            # 抑制剂 3000mg 病案：判停只差 3mg，收尾称重却涨 52mg），只靠振动细水长流。
            if not stock_mode:
                self._trim_stir_on()
                self._trim_stir_preroll(preroll)
            # 脉冲期间盯秤的刹车线：宁可少一点多补一发，也不要一发冲过头
            watch_at = min(float(target), float(upper)) - TRIM_PULSE_WATCH_MARGIN_MG
            cut_early = False
            if TRIM_PULSE_MODE == "burst":
                # 真脉冲：把时长换算成猝发个数（80Hz 时粒度 12.5ms），
                # 设备数完自动停——关断时刻不靠轮询精度。
                n = max(1, round(pulse_s * max(1.0, float(self._applied_freq))))
                try:
                    self.hub.sig.burst_setup(n)
                    self.hub.sig.burst_arm()
                    info = self.hub.sig.burst_fire(n)
                    fired = True
                    self._vib_running = True
                    wait_s = float(info.get("estimated_s") or pulse_s) + 0.1
                    cut_early = self._trim_watch_during_pulse(wait_s, watch_at)
                    self._vib_running = False
                except Exception as error:    # noqa: BLE001 - 设备不认就降级
                    self._log(f"猝发指令失败（{error}），本发及后续降级为软件定时脉冲")
                    self._vib_running = False
            if not fired:
                self.hub.sig_output(True)
                self._vib_running = True
                cut_early = self._trim_watch_during_pulse(pulse_s, watch_at)
                self.hub.sig_output(False)
                self._vib_running = False
            count += 1
            # 出料窗口结束：立即停搅拌再称重，称出来的才是这一发真正落了多少。
            # 存粉模式下本来就没起搅拌，不重复下发停搅拌指令。
            if not stock_mode:
                self._trim_stir_off(why=f"第 {count} 发脉冲结束，停搅拌称重")
            mass, elapsed = self._trim_wait_settled(f"第 {count} 发后等落定")
            gain = mass - before
            prev_gain = float(self._trim_last_gain)
            prev_amp = float(self._trim_last_gain_amp)
            self._trim_last_gain = float(gain)
            self._trim_last_gain_amp = float(self._applied_amp)
            if stock_mode:
                self._log(f"{label}第 {count} 发（存粉·无搅拌）：振 {pulse_s:.2f}s "
                          f"@{self._applied_amp:.2f}V → {gain:+.0f}mg，"
                          f"累计 {mass:.1f}/{target} mg（剩 {max(0.0, target - mass):.0f}mg"
                          f"{'，到刹车线提前掐断' if cut_early else ''}）")
            else:
                self._log(f"{label}第 {count} 发：振 {pulse_s:.2f}s @{self._applied_amp:.2f}V"
                          f"（预转 {preroll:.2f}s）→ {gain:+.0f}mg，"
                          f"累计 {mass:.1f}/{target} mg（剩 {max(0.0, target - mass):.0f}mg"
                          f"{'，到刹车线提前掐断' if cut_early else ''}）")
            # 每发都刷一次状态：页面能直接看到"第几发、补了多少、还差多少"，
            # 不会出现"阶段写着缓补、顶部还停在熔断那句话"的错位。
            self._set_state("running",
                            f"{label} 第 {count} 发：{gain:+.0f}mg → {mass:.0f}/{target}mg"
                            f"（剩 {max(0.0, target - mass):.0f}mg）")
            self._record(elapsed, mass, gain / max(pulse_s, 0.05), "trim")
            if gain < 0.5:
                no_gain += 1
                if no_gain >= TRIM_NO_GAIN_LIMIT:
                    self._log(f"{label}连续 {no_gain} 发没出粉（幅"
                              f"{'锁定在' if hold_amp else '已抬到'} "
                              f"{self._applied_amp:.2f}V"
                              f"{'' if hold_amp else f'/{TRIM_AMP_MAX_VPP:.1f}V'}、"
                              f"预转 {preroll:.2f}s、杆 {self._applied_window}）"
                              f"——停手，请检查料斗是否搭桥/缺料")
                    break
                if stock_mode:
                    # 存粉保护：振幅只温柔地 +0.2V/发（不是正常模式的 0.5V），
                    # 连续 TRIM_STOCK_MAX_NOGAIN 发还是一粒不出 → 料道确实空了，
                    # 退出存粉模式：振幅恢复进缓补时的值，转回正常脉冲（起搅拌）。
                    if no_gain >= TRIM_STOCK_MAX_NOGAIN:
                        stock_mode = False
                        no_gain = 0
                        self._log(f"存粉试探 {TRIM_STOCK_MAX_NOGAIN} 发"
                                  f"（幅已爬到 {self._applied_amp:.2f}V）仍一粒不出"
                                  f"——料道应该空了，退出存粉模式：振幅恢复 "
                                  f"{stock_amp_restored:.2f}V，转回正常脉冲（起搅拌）")
                        if abs(stock_amp_restored - float(self._applied_amp)) > 0.01:
                            self._set_amplitude(stock_amp_restored, elapsed,
                                                f"存粉模式退出：振幅恢复 "
                                                f"{stock_amp_restored:.2f}V",
                                                ceiling=TRIM_AMP_MAX_VPP)
                    else:
                        new_amp = round(min(TRIM_AMP_MAX_VPP,
                                            float(self._applied_amp)
                                            + TRIM_STOCK_AMP_STEP_VPP), 2)
                        if new_amp > float(self._applied_amp) + 0.01:
                            self._set_amplitude(new_amp, elapsed,
                                                f"存粉模式第 {no_gain} 发没出粉："
                                                f"振幅 +{TRIM_STOCK_AMP_STEP_VPP:.1f}V → "
                                                f"{new_amp:.2f}V（慢慢加，不搅拌）",
                                                ceiling=TRIM_AMP_MAX_VPP)
                elif hold_amp:
                    # 定幅补差（缓补关闭）：不抬振幅，只让搅拌多带一会儿。
                    # 抬档是"试探能不能出料"，一旦出粉就会多冲——用户要的是慢慢补。
                    preroll = min(TRIM_STIR_SPINUP_MAX_S,
                                  preroll + TRIM_STIR_SPINUP_GROW_S)
                    self._log(f"补差第 {count} 发没出粉：振幅锁定 "
                              f"{float(self._applied_amp):.2f}V 不抬档"
                              f"（缓补关闭只补不试探）——预转加到 {preroll:.2f}s 再试"
                              f"（脉冲仍固定 {TRIM_PULSE_S:.2f}s）")
                else:
                    duration, preroll = self._trim_escalate_on_no_gain(
                        count, elapsed, no_gain, duration, preroll)
            else:
                no_gain = 0
                if stock_mode:
                    # 存粉模式出了粉：**就停在这档**（不退档不抬档、不学"可接受档位"）
                    # ——存粉本来就怕倾泻，出多少算多少，剩下的量靠多发几发收。
                    self._log(f"存粉模式出粉了：{self._applied_amp:.2f}V 能带出料，"
                              f"就停在这档继续出")
                # ── 抬档后一出粉就退档（2026-10-26 用户硬规矩）────────────────
                # 旧版这里只管"没出粉往上抬"，出粉了就认定这个档位是工作点：
                # 2000mg 那次 4.00V 第一发 +16mg、第二发 +38mg，就是这么冲出去的。
                # 现在两条：
                #   ① 本发是在"刚抬上来的档位"上发的 → 只要出粉就退一档（4.0→3.5）；
                #   ② 单发 >18mg（TRIM_GAIN_TOO_MUCH_MG）→ 不管档位高低都退；
                #   ③ 退到某档后出料 ≤18mg 就记成"可接受档位"，以后回到这一档不再退。
                # **回退触发线 3.0V + 一次退 1.0V**（10:26 / 10:44 用户）：
                # 只有 >3.0V 出粉才退，退到 max(设备下限 0.01V, 当前−1.0V)**可退到 3V 以下**
                # （3.5→2.5、4.0→3.0）。但同一个目标档一次加粉只试一次：退下去没出粉、
                # 又被抬回来才出 → 那档以下一粒不出，就此认定当前档为工作点，不再来回。
                amp_used = float(self._applied_amp)
                too_much = gain > TRIM_GAIN_TOO_MUCH_MG
                escalated_pulse = bool(self._trim_pulse_escalated)
                self._trim_pulse_escalated = False
                above_line = amp_used > TRIM_AMP_BACKOFF_MIN_VPP + 0.01
                back_off = False
                if stock_mode:
                    pass    # 存粉模式不退档不抬档（上面已打日志），退档块整段跳过
                elif mass < lower and above_line:
                    want = round(max(TRIM_AMP_FLOOR_VPP,
                                     amp_used - TRIM_AMP_BACKOFF_STEP_VPP), 2)
                    if too_much or (escalated_pulse
                                    and amp_used > self._trim_amp_ok + 0.01):
                        if want <= self._trim_backoff_target + 0.01:
                            # 这档已经退过一次、退下去一粒不出（又抬回 amp_used 才出粉）
                            self._trim_amp_ok = max(float(self._trim_amp_ok), amp_used)
                            self._log(
                                f"缓补：退到 {want:.2f}V 一粒不出、只能靠 "
                                f"{amp_used:.2f}V 出粉——{amp_used:.2f}V 就当工作点，"
                                f"不再来回试探（单发仍盯 "
                                f">{TRIM_GAIN_TOO_MUCH_MG:.0f}mg）")
                        else:
                            back_off = True
                            if too_much:
                                why = (f"这一发 {gain:+.0f}mg@{amp_used:.2f}V 超过 "
                                       f"{TRIM_GAIN_TOO_MUCH_MG:.0f}mg——加多了，"
                                       f"退 {TRIM_AMP_BACKOFF_STEP_VPP:.1f}V")
                            else:
                                why = (f"刚把振幅抬到 {amp_used:.2f}V 这一发就出了 "
                                       f"{gain:+.0f}mg——抬档只是试探，"
                                       f"退 {TRIM_AMP_BACKOFF_STEP_VPP:.1f}V 回去加")
                elif mass < lower and too_much:
                    # ≤3.0V 不触发回退（回退触发线），振幅不动；
                    # 脉冲时长也是恒定的 0.40s 不动（2026-10-26 10:32 用户）——
                    # 这一发就只能是这个量级，剩下的靠"少发几发 + 盯秤掐断"收。
                    self._log(f"缓补这一发 {gain:+.0f}mg 超过 "
                              f"{TRIM_GAIN_TOO_MUCH_MG:.0f}mg，但振幅 {amp_used:.2f}V "
                              f"已在回退触发线 {TRIM_AMP_BACKOFF_MIN_VPP:.1f}V 及以下"
                              f"——振幅与脉冲时长都不动"
                              f"（{TRIM_PULSE_S:.2f}s/发），就这档慢慢出")
                if back_off:
                    # 一次退 1.0V，**可以退到 3V 以下**（用户 10:44：3.5V 出了就回 2.5V）
                    new_amp = round(max(TRIM_AMP_FLOOR_VPP,
                                        amp_used - TRIM_AMP_BACKOFF_STEP_VPP), 2)
                    if new_amp < amp_used - 0.01:
                        self._set_amplitude(new_amp, elapsed,
                                            f"{why}（{amp_used:.2f}→{new_amp:.2f}V）",
                                            ceiling=TRIM_AMP_MAX_VPP)
                        # 记住退到哪一档：同一次加粉里再想退到 ≤ 这一档就直接认定工作点
                        self._trim_backoff_target = new_amp
                        if too_much:
                            # 这个档位已经证明"一发 18mg 以上"，不能再被当成可接受档位
                            self._trim_amp_ok = min(float(self._trim_amp_ok), new_amp)
                    elif too_much:
                        self._log(f"缓补这一发 {gain:+.0f}mg 已到振幅下限 "
                                  f"{amp_used:.2f}V，不能再退——就这档慢慢出")
                else:
                    if amp_used > self._trim_amp_ok + 0.01:
                        # 这一档出料在 18mg 以内 → 记住它能用，下次回到这里就不再退
                        self._trim_amp_ok = amp_used
                # 注：这里原来按"发少了 ×1.5 拉长 / 发多了 ×0.65 缩短"自适应时长，
                # 2026-10-26 10:32 用户定"脉冲固定 0.4s、只变幅度"，已整段删掉。
                # 发少了就靠振幅闭环（没出粉 +0.5V、出粉后按档位收敛），慢慢出。
                # 同一振幅下这一发比上一发**暴涨**（3.10V：+12 → +38，同 0.60s）
                # → 这档振幅对这个粉太凶，降一档留给下一发；已经到位就不再动信号源。
                # **只在回退触发线（3.0V）以上才降**：≤3V 是稳稳出料的工况，降下去就空振，
                # 放量只能靠"少发几发 + 盯秤掐断"收（2026-10-26 10:32 用户：时长不动）。
                if (not stock_mode and not back_off and above_line and mass < lower
                        and prev_gain >= 1.0
                        and abs(prev_amp - float(self._applied_amp)) < 0.02
                        and gain >= TRIM_AMP_BLOWUP_RATIO * prev_gain):
                    floor_v = max(self._trim_amp_down_floor(),
                                  TRIM_AMP_BACKOFF_MIN_VPP)
                    new_amp = round(max(floor_v,
                                        float(self._applied_amp) - TRIM_AMP_DOWN_STEP_VPP), 2)
                    if new_amp < float(self._applied_amp) - 0.01:
                        self._set_amplitude(
                            new_amp, elapsed,
                            f"缓补同振幅 {self._applied_amp:.2f}V 下这一发 {gain:+.0f}mg，"
                            f"是上一发 {prev_gain:+.0f}mg 的 "
                            f"{gain / max(prev_gain, 0.1):.1f} 倍——放量了，降 "
                            f"{TRIM_AMP_DOWN_STEP_VPP:.1f}V",
                            ceiling=TRIM_AMP_MAX_VPP)
        # 兜底停搅拌：正常路径每发结束都停过了，这里只防"脉冲中途异常/停止"把搅拌留在转着
        self._trim_stir_off(why="缓补收尾")
        # 退猝发待命（WO0s+Wc0s）：这次加粉用过真脉冲的话设备还停在"手动触发模式"，
        # 输出通道开着但只认 WP1s 触发——不退出来，下次加粉设了方波也不震
        # （2026-10-22 18:23 用户指出的坑）。放在 stop 判断之前，中止路径也退。
        try:
            self.hub.sig.burst_off()
            self._log("缓补收尾：已退猝发待命，恢复连续方波输出模式")
        except Exception as error:            # noqa: BLE001 - 起振前还有一次兜底
            self._log(f"退猝发待命失败（下次加粉起振前会再退一次）：{error}")
        if self._stop_event.is_set():
            self._close_stage_log(mass, elapsed)
            raise _Abort(getattr(self, "_internal_stop", "操作员停止"))
        self._close_stage_log(mass, elapsed)
        if mass > upper:
            # 2026-10-23 修：原来只要 mass >= lower 就说"落在验收内"，
            # 228mg 也照写"落在 190~210mg 内"，日志自相矛盾。
            return (f"缓补结束（已超验收）：{count} 发脉冲，最终 {mass:.1f} mg / 目标 {target} mg"
                    f"（超上限 {upper:.0f}mg 共 {mass - upper:.0f}mg）——尾料冲过头，结果按偏重")
        if mass >= lower:
            delta = mass - target
            gap = "超出" if delta > 0 else ("差" if delta < 0 else "正好")
            return (f"缓补完成：{count} 发脉冲，最终 {mass:.1f} mg / 目标 {target} mg"
                    f"（{gap} {abs(delta):.1f}mg，落在验收 {lower:.0f}~{upper:.0f}mg 内）")
        return (f"缓补结束但未到目标：{count} 发脉冲，最终 {mass:.1f}/{target} mg"
                f"（差 {target - mass:.1f}mg，验收下限 {lower:.0f}）——结果按未完成处理")

    def _set_amplitude(self, amp: float, elapsed: float, why: str = "",
                       ceiling: float = AMP_MAX_VPP) -> None:
        """把振幅写进信号源（唯一入口，统一钳位 + 留痕 + 重置响应等待窗）。

        ``ceiling``：绝对上限，默认闭环连续段的 4.0V。**只有缓补脉冲**会传
        ``TRIM_AMP_MAX_VPP(5.0)``——脉冲是短促冲击，不受"标定工况 ±0.5V"约束，
        而那种粉的起振门槛实测在 3.5V 以上（见 ``TRIM_AMP_MAX_VPP`` 注释）。
        """
        amp = round(max(AMP_MIN_VPP, min(float(ceiling), float(amp))), 2)
        try:
            self._sig_write(lambda: self.hub.sig.set_amplitude(float(amp)))
        except Exception as error:            # noqa: BLE001 - 写不进就保持原值继续跑
            self._log(f"振幅写入 {amp:.2f} Vpp 失败：{error}")
            return
        previous = self._applied_amp
        self._applied_amp = amp
        self._amp_last_write_s = elapsed
        if abs(amp - previous) >= 0.005:
            # 登记这次写入，等下一次落料阶跃出现时配对出 τ_est（纯滞后在线估计）
            self._amp_write_marks.append((float(elapsed), float(amp)))
            if len(self._amp_write_marks) > 8:
                self._amp_write_marks = self._amp_write_marks[-8:]
        self._amp_min = amp if not self._amp_min else min(self._amp_min, amp)
        self._amp_max = max(self._amp_max, amp)
        # **每次值变化都记一条，不做时间限流**（2026-10-22 用户要求）。
        # 旧版卡了 2s 限流 + 0.01V 阈值：3000mg 那次快加段振幅从 2.00 一路涨到
        # 3.20（4 次写入），日志里一条"振幅 x → y"都没有，只有心跳里的当前值，
        # 事后完全看不出"什么时候为什么改的"。写入频率由 interval（5–7s）约束，
        # 本来就不可能刷屏，限流纯属自找麻烦。
        if abs(amp - previous) >= 0.005:
            self._amp_log_s = elapsed
            self._log(f"振幅 {previous:.2f} → {amp:.2f} Vpp"
                      + (f"（{why}）" if why else ""))

    # ------------------------------------------------------------------
    # 阶段统计
    # ------------------------------------------------------------------

    def _start_stage_log(self, spec: StagePlan, mass: float, elapsed: float) -> None:
        trim_src = getattr(spec, "source", None)
        with self._lock:
            self._stage_log.append({
                "stage": spec.name,
                "stage_text": STAGE_TEXT[spec.name],
                # 缓补段的来源（slow=慢加尾量正常缓补、topup=判停偏轻补差）
                **({"trim_source": trim_src} if trim_src else {}),
                "start_mass_mg": round(mass, 1),
                "start_s": round(elapsed, 1),
                "end_mass_mg": None,
                "end_s": None,
                "rate_target_mg_s": round(spec.rate_mg_s, 2),
                "stir_rpm": round(spec.stir_rpm, 1),
                "rod_ff": int(spec.rod_ff),
                "amp_start_vpp": round(self._applied_amp, 2),
                "amp_target_vpp": round(float(spec.amplitude_vpp), 2),
            })

    def _close_stage_log(self, mass: float, elapsed: float) -> None:
        with self._lock:
            if not self._stage_log:
                return
            entry = self._stage_log[-1]
            if entry.get("end_mass_mg") is not None:
                return
            entry["end_mass_mg"] = round(mass, 1)
            entry["end_s"] = round(elapsed, 1)
            entry["rod_final"] = int(self._applied_window)   # 推杆停在哪就是哪（不复位）
            entry["amp_end_vpp"] = round(self._applied_amp, 2)
            entry["amp_min_vpp"] = round(self._amp_min or self._applied_amp, 2)
            entry["amp_max_vpp"] = round(self._amp_max or self._applied_amp, 2)
            entry["amp_changes"] = int(self._amp_changes)
            gained = mass - entry["start_mass_mg"]
            span = max(0.1, elapsed - entry["start_s"])
            entry["gained_mg"] = round(gained, 1)
            entry["actual_rate_mg_s"] = round(gained / span, 2)
            entry["duration_s"] = round(span, 1)
            # 慢加段的目标速率是动态的（剩余量/18s），记一个收尾时刻的目标值，
            # 复盘时才能看懂"为什么这一段尾巴上目标只有 1 mg/s"
            spec_now = next((item for item in self._plan if item.name == entry["stage"]), None)
            if spec_now is not None:
                entry["rate_target_end_mg_s"] = round(self._rate_command_for(spec_now, mass), 2)
            cmd_end = entry.get("rate_target_end_mg_s")
            cmd_text = (f"{entry['rate_target_mg_s']:.2f}" if cmd_end is None
                        else f"{entry['rate_target_mg_s']:.2f}→{cmd_end:.2f}(末)")
            summary = (f"{entry['stage_text']}结束：{entry['start_mass_mg']:.0f} → "
                       f"{mass:.0f} mg（+{gained:.0f}mg / {span:.1f}s = "
                       f"{gained / span:.2f} mg/s，目标 {cmd_text}）｜振幅 "
                       f"{entry['amp_min_vpp']:.2f}–{entry['amp_max_vpp']:.2f}V"
                       f"（末值 {self._applied_amp:.2f}，共调 {self._amp_changes} 次）"
                       f"｜推杆 {entry['rod_ff']} → {int(self._applied_window)}")
        self._log(summary)

    # ------------------------------------------------------------------
    # 收尾
    # ------------------------------------------------------------------

    def _settle_and_finish(self, baseline: float, rod_start: int, reason: str, *, ok: bool) -> None:
        """停振 → 停搅拌 → 等尾料沉降取终值 → 落盘。

        **推杆停在最终开度，不复位**（2026-10-21 用户要求）：不复位就没有多余的机械动作，
        也不会在称重后再晃一次秤；下次加粉由阶段入口按配方开度自行移动。

        这一阶段**不再响应停止请求**：振动已经关了，唯一还在动的是搅拌减速，
        中途再被中止只会把机构留在半路。
        """
        self._set_state("settling", "已停止振动，搅拌减速，等待尾料沉降称重")
        self._safe_stop_vibration()
        try:
            self.hub.stir_stop("decel", acc=self.STIR_ACC)   # 缓停：转子上挂的粉会继续落
        except Exception as error:      # noqa: BLE001
            self._log(f"收尾停搅拌失败：{error}")
        self._applied_stir = 0.0
        final_raw, stable, estimated = self._settle(baseline)
        achieved = float(final_raw) - baseline
        lower, upper = acceptance_limits(self._config["target_mg"])
        if achieved < lower:
            verdict = "偏轻"
        elif achieved > upper:
            verdict = "偏重"
        else:
            verdict = "合格"

        target = self._config["target_mg"]

        # ── 判停预测失误兜底（2026-10-22 18:33 用户指出"慢加直接结束了没有缓补"）──
        # predictive_stop 用"最近窗口回归速率"估尾料，慢加刚切入时粗加余料还在
        # 输送段里倾泻（瞬态 26 mg/s 被当成稳态），尾料被高估 → 提前停振 →
        # 实际尾料远小于预测 → 收尾偏轻（本次 163/200，差 37mg 却直接结束）。
        # 缓补开启且缺口可补时：重启搅拌，回到缓补脉冲把差的部分一发一发补齐，
        # 补完重新沉降称重再判一次。只兜一次——补完仍偏轻就如实落盘。
        # ★ 2026-10-27 13:46 用户："没有开缓补功能的加粉，假如最后停机时质量少了，
        #   进行缓补"——所以这里**不再要求 `_trim_on`**。两种模式的差别：
        #     · 缓补开 → 自适应脉冲（可抬档/退档），与慢加尾段缓补同一套；
        #     · 缓补关 → 定幅补差：振幅锁在页面慢加那一行（失水剂2 = 0.30V），
        #       50RPM 搅拌、每发 0.40s，不抬档不试探，补进验收带就停手。
        #   触发条件不变：判停成功、终值来自真实称重（非 estimated）、偏轻且在补差上限内。
        if (verdict == "偏轻" and ok and not estimated
                and 0.0 < target - achieved <= TRIM_RESCUE_MAX_MG
                and not getattr(self, "_trim_rescued", False)):
            self._trim_rescued = True
            # ★ 先别动搅拌（2026-10-23 11:09 教训）：判停那一刻"停搅拌后的余料"还没落完。
            #   那次加粉判停 170mg 就立刻重启搅拌，转子上挂的粉被带下来 58mg，
            #   最后 233mg 偏重 33mg。搅拌停着再等 3~6s，质量常常自己就走进了窗口，
            #   这种时候重启搅拌补差纯属自己给自己加油门。
            quiet_mass, _quiet_elapsed = self._trim_wait_settled(
                "判停偏轻：先等停搅拌后的余料落定（搅拌保持停止）",
                min_s=TRIM_RESCUE_QUIET_MIN_S, max_s=TRIM_RESCUE_QUIET_MAX_S,
                stage="settle", abort_at=upper)
            tail = quiet_mass - achieved
            if tail >= 1.0:
                self._log(f"停搅拌后余料又落了 {tail:.0f}mg：{achieved:.1f} → "
                          f"{quiet_mass:.1f} mg / 目标 {target}")
            achieved = quiet_mass
            slow_rpm = next((float(it["stir_rpm"]) for it in reversed(self._stage_log)
                             if it.get("stage") == "slow"), 0.0)
            if achieved >= lower:
                verdict = "偏重" if achieved > upper else "合格"
                self._log(f"判停偏轻是假象：余料自己落进 {lower:.0f}~{upper:.0f}mg 窗口"
                          f"（{achieved:.1f}/{target}mg）——不重启搅拌、不发脉冲")
            elif slow_rpm <= 0.0:
                self._log(f"判停偏轻 {achieved:.1f}/{target} mg，但没有慢加转速记录，"
                          f"无法重启搅拌补差——如实落偏轻")
            else:
                self._log(f"判停预测失误：实际 {achieved:.1f}/{target} mg，差 "
                          f"{target - achieved:.0f}mg 低于下验收线——余料等过了仍不够，"
                          f"转收尾补差（搅拌回 {slow_rpm:.0f} RPM）")
                try:
                    self.hub.stir_start(slow_rpm, acc=self.STIR_ACC, direction=0)
                    self._applied_stir = slow_rpm
                except Exception as error:      # noqa: BLE001
                    self._log(f"补差重启搅拌失败（{error}），脉冲补料照常进行")
                # 不在这里等天平：`_trim_pulse_loop` 一进去就会把搅拌**立即停**再等落定，
                # 现在多等一秒就是让转着的搅拌多送几 mg 下去（10-23 那次加粉白等了 3.1s / +16mg）。
                mass_now = float(self._read_balance_raw()) - self._baseline
                elapsed_now = time.monotonic() - self._t0
                self._close_stage_log(mass_now, elapsed_now)   # 幂等，关掉可能开着的段
                # 缓补关闭 → 定幅补差：振幅就用页面慢加那一行（不再往上试）
                slow_amp = next(
                    (float(getattr(item, "amplitude_vpp", 0.0) or 0.0)
                     for item in (self._plan or []) if item.name == "slow"), 0.0)
                hold = not self._trim_on
                if hold:
                    self._log(f"缓补关闭，但收尾偏轻 {target - achieved:.0f}mg 在补差上限内——"
                              f"按慢加那一行定幅补差：{slow_amp:.2f}V / "
                              f"{slow_rpm:.0f}RPM / 每发 {TRIM_PULSE_S:.2f}s"
                              f"（振幅锁定，不抬档）")
                try:
                    outcome = self._trim_pulse_loop(
                        mass_now, elapsed_now, target, source="topup",
                        hold_amp=hold,
                        amp_start=(slow_amp if hold and slow_amp > 0.0 else None))
                    self._log(f"补差结果：{outcome}")
                finally:
                    mass_now = float(self._read_balance_raw()) - self._baseline
                    elapsed_now = time.monotonic() - self._t0
                    self._close_stage_log(mass_now, elapsed_now)
                    try:
                        self.hub.stir_stop("decel", acc=self.STIR_ACC)
                    except Exception as error:  # noqa: BLE001
                        self._log(f"补差停搅拌失败：{error}")
                    self._applied_stir = 0.0
                final2, _stable2, estimated = self._settle(baseline)
                achieved = float(final2) - baseline
                if achieved < lower:
                    verdict = "偏轻"
                elif achieved > upper:
                    verdict = "偏重"
                else:
                    verdict = "合格"


        # 前馈自修正：把这一次加粉"实测速率 / 标定预测速率"的比值回写粉体表，下一次加粉开度更准。
        # 只在"正常判停 + 合格 + 终值来自真实称重"时更新——中途停机、超重保护、
        # 天平失联这些情况下的实际速率根本不可信，拿来修表只会越修越歪。
        ff_update = None
        if self._workspace is not None and ok and verdict == "合格" and not estimated:
            try:
                ff_update = self._workspace.record_run(list(self._stage_log), accepted=True)
                if ff_update and ff_update.get("updated"):
                    self._workspace.save()
                    self._log(f"前馈自修正：「{self._workspace.name}」系数 → "
                              f"{ff_update['factor']:.3f}"
                              f"（本次加粉实测/标定 = {ff_update['sample']:.3f}）")
            except Exception as error:      # noqa: BLE001 - 写回失败不影响本次加粉结果
                self._log(f"前馈自修正写回失败：{error}")

        # 切段惯性落料量写回粉体档案：下次切段直接把切换点提前这么多。
        # 与"是否合格"无关（它是物理量），但天平失联（estimated）时坚决不写。
        settle_update: dict[str, Any] = {}
        if self._workspace is not None and self._settle_events and not estimated:
            try:
                for event in self._settle_events:
                    if not event.get("from") or not event.get("to"):
                        continue
                    key = f"{event['from']}>{event['to']}"
                    settle_update[key] = self._workspace.record_settle_gain(
                        key, float(event["gain_mg"]))
                if settle_update:
                    self._workspace.save()
                    brief = "、".join(f"{k} {v['gain_mg']:g}mg"
                                     for k, v in settle_update.items())
                    self._log(f"切段惯性已记住：「{self._workspace.name}」{brief}"
                              f"（下次切段点自动提前这么多）")
            except Exception as error:      # noqa: BLE001 - 写回失败不影响本次加粉结果
                self._log(f"切段惯性写回失败：{error}")

        result = {
            "ok": ok and verdict == "合格",
            "verdict": verdict,
            "target_mg": target,
            "achieved_mg": round(achieved, 1),
            "delta_mg": round(achieved - target, 1),
            "acceptance_mg": [lower, upper],
            "baseline_mg": round(baseline, 1),
            "final_mg": round(float(final_raw), 1),
            "rod_start_units": int(rod_start),
            "rod_final_units": int(self._applied_window),   # 停在哪儿就是哪儿，不复位
            "amplitude_vpp": round(self._applied_amp, 2),
            "stable": bool(stable),
            "estimated": bool(estimated),      # 天平读不到时用最后一次已知值，标注为估计
            "duration_s": round(time.monotonic() - self._t0, 1),
            "stop_reason": reason,
            "samples": len(self._samples),
            "curve": list(self._curve),
            "stages": list(self._stage_log),
            "plan": [stage.to_dict() for stage in self._plan],
            "powder": self._config.get("powder"),
            "calibrated": self._config.get("calibrated"),
            "feedforward_update": ff_update,
            "settle_events": list(self._settle_events),
            # 超速熔断的每一次触发（2026-10-26 18:04 补）：以前只进日志、不进结果文件，
            # 复盘时想确认"熔断到底有没有工作"只能去翻 control-*.log。18:04 那次加粉就是
            # 靠翻日志才发现 `_surge_rate` 在真机 0.266s 采样周期下永远凑不够点数、
            # 熔断一次都没触发过。以后 `result.surge_events` 直接能看。
            "surge_events": list(self._surge_events),
            "settle_update": settle_update,
            "settle_gate": (self._config.get("settle_gate") or {}),
            "config": dict(self._config),
            "finished_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        }
        with self._lock:
            self._result = result
            self._state = "done" if ok else "aborted"
            self._detail = (f"{verdict}：{achieved:.1f} mg / 目标 {target} mg"
                            f"（{reason}）")
        self._save_run(result)
        self._log(f"闭环加粉结束：{verdict}，实得 {achieved:.1f} mg"
                  f"（目标 {target} mg，偏差 {achieved - target:+.1f} mg），"
                  f"用时 {result['duration_s']}s，原因：{reason}")

    # ------------------------------------------------------------------
    # 硬件小工具（全部经过互锁/急停已有的实现）
    # ------------------------------------------------------------------

    def _read_balance_raw(self) -> int:
        snapshot = self.hub.bal_refresh()     # 强制读、跳过节流；失败会抛
        if snapshot.get("raw") is None:
            raise RuntimeError("天平没有返回有效计数")
        return int(snapshot["raw"])

    def _read_baseline(self) -> float:
        values: list[int] = []
        for _ in range(3):
            values.append(self._read_balance_raw())
            time.sleep(0.12)
        values.sort()
        self._last_raw = values[1]
        return float(values[1])               # 中位数：抵一次偶发跳数

    def _sig_write(self, action: Callable[[], Any]) -> None:
        """振动参数写入（带重连重试的既有封装）。"""
        self.hub._sig_call(action)

    def _open_vibration(self, why: str) -> None:
        """开振动输出，**并在开闸后立刻复核一次幅度**（唯一开闸入口）。

        为什么开闸后还要再写一遍幅度（2026-10-29 16:40 用户："切到粗加时会先按 2.7V
        振动一下，然后再变 2V"）：段间是"关输出 → 写振幅 → 写占空 → 开推杆 → 开输出"，
        幅度的写入全部落在**输出关闭**的窗口里。信号源如果只在 OUT=1 时才把 WC 应用
        到模拟通道（或开闸瞬间先按上一次的寄存器起振），真出料的那一小段就还是上一段
        的幅度（快加常到 2.88V），下一拍闭环才把它拉回 2.00V——观感就是"先按 2.7V 振
        一下再变 2V"。
        开闸后幂等复写一次，把这个窗口压到一条指令的时间（约 20–40ms，80Hz 下 1–3 个
        周期），无论设备是哪一种语义都成立。

        这里**只做开闸必需的参数写 + 开闸 + 复核幅度**，不发 `WO0s` 一类改变输出
        状态的指令——17:10 曾在这里插过一次 `burst_off()`（`WO0s`+`Wc0s`），结果
        快加段整段不振动。

        **开闸前重设波形/频率/占空**（2026-10-29 17:26 加）：这三个是纯参数写，
        不改变输出通断。之所以每次都要写一遍，是因为 `WO0s`（`_start` 退猝发、
        缓补收尾都会发）会把输出波形复位，之后 `Wc1s` 只开通道不出波；重设一遍
        就与 `_start` 的开场完全同构，无论设备此前停在什么状态都能正常出波。
        """
        try:
            self._sig_write(lambda: (
                self.hub.sig.set_waveform(self.WAVEFORM_SQUARE),
                self.hub.sig.set_frequency(float(self._applied_freq)),
                self.hub.sig.set_duty(self._applied_duty / 100.0),
            ))
        except Exception as error:            # noqa: BLE001 - 参数写失败也继续开闸，靠复核幅度兜底
            self._log(f"开闸前重设波形失败（继续开闸，{why}）：{error}")
        self.hub.sig_output(True)
        self._vib_running = True
        try:
            self._sig_write(lambda: self.hub.sig.set_amplitude(float(self._applied_amp)))
        except Exception as error:            # noqa: BLE001 - 复核失败不影响开闸本身
            self._log(f"开闸后复核幅度失败（{why}）：{error}")
        else:
            self._log(f"③ 开闸：振动输出打开（{why}）｜开闸后复核幅度 "
                      f"{self._applied_amp:.2f}Vpp")

    def _safe_stop_vibration(self) -> None:
        try:
            # ★ 2026-10-29 17:26（用户："慢加依旧不震动"）：**停振只发 Wc0s，不许发 WO0s**。
            #
            # 原来这里先 `burst_off()`（= `WO0s` 关触发功能 + `Wc0s`）再 `Wc0s`。
            # 实测后果：**切段之后的所有段全部哑振**——
            #   · 17:20:48 那发：快加（没走沉降门）78mg/s 正常；走了沉降门的粗加
            #     130s 只 +382mg、慢加 80s 只 +99mg，振幅一路顶到 3.00V 仍不出料；
            #   · 17:18:38 那发：快加 → 粗加没走沉降门，粗加 21mg/s 也是搅拌出的粉。
            # 对照 `_start` 的开场：它是 `burst_off()` 之后**紧接着重设方波/频率/占空/
            # 幅度**，所以第一次开闸能正常震动。`WO0s` 把触发功能关掉的同时也把输出
            # 波形复位了，之后单发 `Wc1s` 只开通道、不出波——这就是"软件说开着、
            # 料却全靠搅拌"的根源（这台设备没有输出状态回读，查不出来）。
            #
            # 退猝发的职责留在 `_start`（每次加粉开头）和缓补收尾，那里后面都有重设
            # 波形的动作；段间停振只关输出，波形状态一直保持，重新开闸立刻能出波。
            self.hub.sig_output(False)
        except Exception as error:            # noqa: BLE001 - 停振动失败必须留痕
            self._log(f"停止振动失败：{error}")
        else:
            self._vib_running = False

    def _safe_shutdown_actuators(self) -> None:
        """异常路径：先关振动（最危险），再停搅拌。

        **不动推杆**（用户 2026-10-21 要求）：异常时把推杆退回去会多一次不可控运动，
        现场人可能正伸手处理；推杆停在原地最安全，需要时用页面手动移。
        """
        self._safe_stop_vibration()
        try:
            self.hub.stir_stop("decel", acc=self.STIR_ACC)
            self._applied_stir = 0.0
        except Exception as error:            # noqa: BLE001
            self._log(f"异常路径停搅拌失败：{error}")

    def _wait_rod(self, target: int, timeout_s: float | None = None,
                  *, honor_stop: bool = True) -> None:
        timeout_s = self.ROD_ARRIVE_TIMEOUT_S if timeout_s is None else timeout_s
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if honor_stop and self._stop_event.is_set():
                raise _Abort(getattr(self, "_internal_stop", "操作员停止"))
            try:
                position = int(self.hub.rod_refresh()["pos"])
            except Exception:                 # noqa: BLE001 - 读不到就再等一轮
                time.sleep(0.1)
                continue
            if abs(position - target) <= 3:
                return
            time.sleep(0.1)
        raise _Abort(f"推杆未在 {timeout_s:.0f}s 内到达 {target}（卡住或阻力过大）")

    def _settle(self, baseline: float) -> tuple[float, bool, bool]:
        """停振后等重量稳定：连续 5 个样本极差 ≤2mg 就算稳，超时取最后 5 个均值。

        返回 (最终原始计数, 是否稳定, 是否估计值)。天平完全读不到时用运行期间
        最后一次已知读数作估计值（并在结果里标 ``estimated=True``），
        绝不因为"读不到"就把结果丢掉——操作员需要知道这一次加粉到底停在哪。
        """
        deadline = time.monotonic() + self.SETTLE_TIMEOUT_S
        earliest = time.monotonic() + self.SETTLE_MIN_STABLE_S
        history: list[int] = []
        while time.monotonic() < deadline:
            try:
                history.append(self._read_balance_raw())
            except Exception:                 # noqa: BLE001
                time.sleep(0.15)
                continue
            # ★ 2026-10-26（用户："沉降门的时候不更新质量"）：收尾称重同样是"振动停了、
            # 粉还在落"，旧版这里只读秤不记数，页面质量在最后 2~4s 冻住，操作员看到
            # 的终值和落盘终值不是一回事。现在每拍都刷新页面/曲线/CSV（learn=False，
            # 不拿去更新 τ）。
            mass_now = float(history[-1]) - float(baseline)
            self._record(time.monotonic() - self._t0, mass_now, 0.0, "settle",
                         learn=False)
            self._set_state("settling",
                            f"收尾称重：{mass_now:.1f}/{self._config['target_mg']} mg"
                            f"（已停振，等尾料落定）")
            if len(history) >= 5 and time.monotonic() >= earliest:
                window = history[-5:]
                if max(window) - min(window) <= 2:
                    return float(round(sum(window) / len(window))), True, False
            time.sleep(0.15)
        window = history[-5:] if len(history) >= 5 else history
        if not window:
            last = getattr(self, "_last_raw", None)
            if last is None:
                return float(baseline), False, True
            return float(last), False, True
        return float(round(sum(window) / len(window))), False, False

    def _record(self, elapsed: float, mass: float, rate: float, stage: str,
                learn: bool = True) -> None:
        """记一拍：页面实时值 + 曲线 + 逐拍 CSV + 速率回归历史。

        ``learn=False``（2026-10-26 加）：**只显示、不参与"落料阶跃"学习**。
        沉降门/收尾称重/缓补等落定这几个场景振动已经关了，秤还在涨是余料在往下走，
        不是"这个振幅落得下粉"的证据——拿去更新纯滞后 ``_tau_est`` 只会把模型带偏
        （起振门槛学习已于 2026-10-27 11:44 整体删除）。所以这几处传 learn=False：
        质量照样实时刷新、CSV 照样留痕，只是不喂给学习器。
        """
        with self._lock:
            # 采样存全量（0.12s 一个），曲线另存 0.5s 抽稀版本给页面画图
            self._samples.append((round(elapsed, 2), round(mass, 1), round(rate, 2), stage))
            # 逐拍 CSV：每个采样一行，带振幅环当时的全部决策现场（见 _csv_tick）
            self._csv_tick(elapsed, mass, rate, stage)
            # 质量历史给振幅环做回归：只留最近 20s（最长窗口 10s + 响应等待 6s 够用），
            # 不清理的话 20 分钟的一次加粉会攒上万点，每轮回归都要遍历一遍。
            self._mass_hist.append((float(elapsed), float(mass)))
            if len(self._mass_hist) > 400:
                self._mass_hist = self._mass_hist[-250:]
            self._live = {
                "mass_mg": round(mass, 1),
                "rate_mg_s": round(rate, 2),
                "elapsed_s": round(elapsed, 1),
                "stage": stage,
                "stage_text": STAGE_TEXT.get(stage, stage),
                "stir_rpm": round(self._applied_stir, 1),
                "rod_units": int(self._applied_window),
                # 输出关着就报 0.00：切段沉降/收尾称重期间料还在往下走，但**振动是停的**。
                # 以前这里照抄 `_applied_amp`，页面在沉降那 5–9s 里一直显示"振幅 2.88V"，
                # 看上去就像"切段后还在按上一段的幅度振"（2026-10-29 16:40 用户的原话）。
                "amp_vpp": round(self._applied_amp, 2) if self._vib_running else 0.0,
                "duty_pct": round(self._applied_duty / 100.0, 1),
                "remaining_mg": round(self._config["target_mg"] - mass, 1),
            }
        # ── 落料阶跃观测（v6 延迟模型的数据源，任何阶段都要看）────────────────
        # 天平上一次 ≥15mg 的增重 = 一次真实落料。它回答两个"什么时候"：
        #   ① 写振幅之后**多久**落的第一坨 → τ_est（纯滞后）
        #   ② 相邻两坨**隔多久** → T_est（落料周期）
        # 这两个量把"改完振幅要等多久才有有效数据"从拍脑袋变成在线测量。
        prev = self._last_mass_for_drop
        self._last_mass_for_drop = float(mass)
        if not learn:
            # 只显示不学习（沉降门/收尾称重）：仍更新"上一次看到的读数"，
            # 免得下一次真学习时把这段时间的余料当成一次巨大阶跃。
            return
        if prev is None or float(mass) - prev < AMP_DROP_EVENT_MG:
            return
        now = float(elapsed)
        if self._last_drop_s is not None:
            gap = now - self._last_drop_s
            if AMP_PERIOD_INIT_S * 0.3 <= gap <= 60.0:
                self._period_est = round(
                    (1 - AMP_TAU_EMA) * self._period_est + AMP_TAU_EMA * gap, 2)
        self._last_drop_s = now
        # 门槛学习已删除（2026-10-27 11:44 用户："不需要记忆这个阶跃触发了门槛学习"）。
        # 旧逻辑会把"这一次落料时的振幅"记成起振门槛，可落料常常是踢脚脉冲
        # （+1.0~1.5V）震下来的、或者余料塌下来的——记下来的门槛比真实门槛高
        # 一大截，之后又被拿去抬工作振幅，形成"踢过一次就再也下不来"。
        # 现在这个粉需要多高电压，只认页面「振幅 Vpp」那一行。
        # τ 估计：把"未关联的振幅写入"和这次落料配对（最早的先配）
        while self._amp_write_marks:
            t_write, amp_at = self._amp_write_marks.pop(0)
            lag = now - t_write
            if lag < 0.5:
                continue                      # 阶跃早于写入（余料），跳过
            if lag <= 60.0:
                self._tau_est = round(
                    (1 - AMP_TAU_EMA) * self._tau_est + AMP_TAU_EMA * lag, 2)
            break                             # 只拿最早那次写入配对，其余作废

    # 逐拍 CSV 列定义：与老系统 feedback_dispense_*_samples.csv 的复盘习惯对齐——
    # 每个天平采样一行，把"当时反馈是多少、振幅环怎么看、动了没有"全记下来。
    _CSV_HEADER = [
        "elapsed_s", "mass_mg", "rate_mg_s", "stage", "target_rate_mg_s",
        "amplitude_vpp", "amp_band_lo_vpp", "amp_band_hi_vpp",
        "kick_active", "kick_count", "kick_amp_vpp",
        "tau_est_s", "period_est_s", "last_write_s_ago",
        "stir_rpm", "rod_units", "frequency_hz", "duty_permyriad",
        "remaining_mg", "reason",
    ]

    def _csv_tick(self, elapsed: float, mass: float, rate: float, stage: str) -> None:
        """往 runs/dispense_<stamp>_<target>mg_<粉>_samples.csv 追加一行逐拍现场。

        首拍惰性建文件、每拍 flush——中途断电/强杀也留得住前面全部数据
        （2026-10-22 17:40 那次加粉就是例子：操作员 23s 手停，光靠 5s 心跳
        复盘不出振幅环每一拍在干什么）。任何落盘异常只记一条日志，
        绝不影响加粉控制本身。
        """
        try:
            if self._samples_csv is None:
                RUN_DIR.mkdir(parents=True, exist_ok=True)
                name = self._config.get("powder") or "nopowder"
                path = RUN_DIR / (f"dispense_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                                  f"_{self._config['target_mg']:g}mg_{name}_samples.csv")
                self._samples_csv = open(path, "w", newline="", encoding="utf-8-sig")
                self._samples_csv_path = str(path)
                csv.writer(self._samples_csv).writerow(self._CSV_HEADER)
            kick = self._amp_kick
            writer = csv.writer(self._samples_csv)
            writer.writerow([
                round(elapsed, 3), round(mass, 1), round(rate, 2), stage,
                round(self._amp_cmd, 2),
                # 实发幅度：输出关着（切段沉降/收尾/缓补间隙）记 0.00，
                # 别把"设定值"当"正在振的幅度"——复盘时这两者混淆会得出错误结论。
                round(self._applied_amp, 2) if self._vib_running else 0.0,
                round(self._amp_band[0], 2), round(self._amp_band[1], 2),
                1 if kick else 0, self._kick_count,
                round(float(kick["amp"]), 2) if kick else "",
                round(self._tau_est, 2), round(self._period_est, 2),
                round(elapsed - self._amp_last_write_s, 2),
                round(self._applied_stir, 1), int(self._applied_window),
                int(self._applied_freq), int(self._applied_duty),
                round(self._config["target_mg"] - mass, 1),
                self._amp_detail,
            ])
            self._samples_csv.flush()
        except Exception as error:            # noqa: BLE001 - 落盘失败不影响控制
            if self._samples_csv is not None:
                try:
                    self._samples_csv.close()
                except Exception:             # noqa: BLE001
                    pass
                self._samples_csv = None
            self._amp_detail = f"逐拍CSV写入失败：{error}"

    def _set_state(self, state: str, detail: str) -> None:
        with self._lock:
            self._state = state
            self._detail = detail

    def _save_run(self, result: dict) -> None:
        """落盘：每次加粉一个 JSON，便于事后复盘（老系统也是这个习惯）。"""
        # 先收逐拍 CSV：无论这一次加粉怎么结束，文件都要完整关闭并把路径报出来
        if self._samples_csv is not None:
            try:
                self._samples_csv.flush()
                self._samples_csv.close()
            except Exception:                 # noqa: BLE001
                pass
            self._samples_csv = None
        if self._samples_csv_path:
            result["samples_csv_path"] = self._samples_csv_path
            self._log(f"逐拍采样 CSV 已保存：{self._samples_csv_path}")
        try:
            RUN_DIR.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            name = result.get("powder") or "nopowder"
            path = RUN_DIR / f"dispense_{stamp}_{result['target_mg']}mg_{name}.json"
            payload = dict(result)
            payload["samples"] = [
                {"t": t, "mass_mg": m, "rate_mg_s": r, "stage": s} for t, m, r, s in self._samples
            ]
            path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            with self._lock:
                self._result["log_path"] = str(path)
        except Exception as error:            # noqa: BLE001 - 落盘失败不影响结果
            self._log(f"加粉记录保存失败：{error}")


class _Abort(Exception):
    """内部中止信号（区别于程序异常）：带一句中文原因。"""

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason
