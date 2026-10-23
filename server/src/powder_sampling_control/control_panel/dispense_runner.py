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
STAGE_TEXT = {"fast": "快加", "coarse": "粗加", "slow": "慢加", "settle": "收尾判停", "trim": "缓补"}
# 段名 → 控制器分支（dispensing_algorithm 里只有 coarse/fine/precision 三档脾气）
STAGE_MODE = {"fast": "coarse", "coarse": "fine", "slow": "precision"}
STAGE_NOTE = {
    "fast": "振幅只许在标定值 ±0.2V 内小步跟，本段最多改 2 次",
    "coarse": "振幅只许在标定值 ±0.2V 内小步跟，本段最多改 2 次",
    "slow": "振幅锁定标定值，整段一动不动（慢加只求慢和稳）",
}
# 三阶段默认目标速率（2026-10-22 用户定）：快加 75 / 粗加 20 / 慢加 5 mg/s。
# 页面留空就是这三个数，标定表按它们反解推杆开度（这张表就是"速率→开度"的查找表）。
DEFAULT_STAGE_RATE_MG_S = {"fast": 75.0, "coarse": 20.0, "slow": 5.0}
# 按速率选标定点时，CV>40%（塌料/天平跳数）的点要额外加多少距离惩罚。
# 0.7 ≈ 相当于"速率再差一倍"——好点够用时永远不会选到脏点。
STAGE_POINT_CV_PENALTY = 0.7
# 页面显示用的目标范围；真正的校验在 dispensing_algorithm.validate_target_mass
TARGET_MIN_MG = 100
TARGET_MAX_MG = 20000

# ── 振幅调节（2026-10-22 用户改口径：基准是标定值，慢加一律不动）─────────────
AMP_MIN_VPP = 1.0
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
#     3V 以上短脉冲就能出料。所以断料救济的上探上限从 标定+0.6V 放宽到 标定+1.5V、
#     步长加大到 0.5V/步（两步就跨过门槛），一旦某档振幅落了粉就把该值记为"起振门槛"，
#     本段后续的微调带以 max(标定值, 门槛−0.2V) 为中心。
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

# ── v7 自适应调幅（2026-10-23 09:52 用户定规矩，取代粗调坡道）────────────────
# 平时调节 = 自适应小步：按"前 5s 平均速率 + 加速度"外推纯滞后后的速率走向，
# 缺口 ÷ 增益(k_est) 换算步长，钳在 0.05–0.15V——丝滑不跳变。
# 脉冲（踢一脚）触发条件**三段统一**：反馈调节已调到带顶（标定+0.5V），
# 且到顶后持续 ≥4s 速率 <1 mg/s——顶上磨满 4s 才许踢，不到顶绝不踢。
AMP_EVAL_INTERVAL_S = 2.0     # 相邻两次调幅的最小间隔（丝滑节奏）
AMP_LOOKBACK_S = 5.0          # 平均速率回看窗（用户举例"前 5 秒平均速率"）
AMP_REEVAL_MIN_S = 4.0        # 写幅后至少等 4s 再评估（效果没到就外推，不全额等 τ）
AMP_STEP_MIN_V = 0.05         # 单步下限（用户举例 0.05–0.15V）
AMP_STEP_MAX_V = 0.15         # 单步上限
AMP_DEFAULT_K = 12.0          # 增益缺省 mg/s per V（k_est 学到值前用）
AMP_ACCEL_MAX = 5.0           # 加速度估计限幅 mg/s²（防噪声把预测吹飞）
AMP_KICK_GATE_WINDOW_S = 4.0  # 踢脚 gate：到顶后观察窗 4s
AMP_KICK_GATE_RATE_MG_S = 1.0 # 踢脚 gate：4s 内速率 <1 mg/s 才算"长时间无速率"
AMP_SLOW_CLIMB_STEP_V = 0.10  # 慢加零落料爬坡每档 +0.10V（涓流）
AMP_SLOW_CLIMB_INTERVAL_S = 2.0


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

# ── 切段沉降门 + 提前降幅（2026-10-21 用户批准，重粉必需）──────────────────
# 快加/粗加以高速率切出时，料槽里的粉还在落：切段那一刻"质量到位"不等于"落完了"。
# 那几 mg～几十 mg 余料会直接顶进下一段的切换点/判停点，重粉尤其明显
# （落差短、单次落料量大），结果是粗加一开场就超标、慢加要倒着补。
# 做法：停振 → 等余料落干净（判稳或超时）→ 再进下一段，并把这段余料记进粉体档案。
SETTLE_GATE_RATE_TRIGGER_MG_S = 30.0   # 上一段实测速率超过它才值得停振等
SETTLE_GATE_MAX_S = 1.2                # 单次等待上限：重粉落料短，超时就用当前质量切
SETTLE_GATE_STABLE_MG = 2.0            # 判稳：窗口内极差
SETTLE_GATE_SAMPLES = 5                # 判稳：窗口样本数（5×0.1s ≈ 0.5s）
SETTLE_GATE_RATE_EPS_MG_S = 0.5        # 判稳：窗口斜率上限 mg/s
# 提前降幅：距切点还有"当前速率 × 系数"mg 时，先把振幅压到下一段的预置值，
# 让流速在切点之前就下来——沉降门只需收最后几 mg，不会站那儿干等。
TAIL_TAPER_FACTOR = 0.6
TAIL_TAPER_MIN_MG = 8.0                # 提前降幅的最小距离（低速段不至于提前太久）

# ── 缓补（trim pulse，2026-10-22 11:30 用户提出）────────────────────────────
# 慢加段最后一段连续出料的落点误差有 ±几 mg 的不确定性（惯性余料 + 判停滞后），
# 对"验收 ±10mg"来说太毛。缓补 = 剩余量进入最后 20~40mg 区间时停掉连续振动，
# 改为**脉冲补料**：短促开一次振（≈10mg 一发）→ 关振等落定称重 → 再发下一发，
# 一发一发凑到目标值。是否启用按粉体档案里的 ``trim_pulse`` 字段（默认关，
# 慢加速率浮动小的粉如膨润土/封堵剂1 不需要开）。
TRIM_ENTER_REMAIN_MG = 40.0   # 慢加段剩余 ≤40mg 进入缓补（用户：剩 30 左右，20~40 区间）
TRIM_PULSE_TARGET_MG = 10.0   # 单发脉冲的目标增重
TRIM_PULSE_START_S = 0.60     # 首发脉冲时长（之后按实际增重自适应）
TRIM_PULSE_MIN_S = 0.15       # 单发最短时长
TRIM_PULSE_MAX_S = 2.50       # 单发最长时长
TRIM_PULSE_MAX_N = 24         # 脉冲发数上限（防死循环；24 发足够凑 240mg）
# 真脉冲模式（2026-10-22 12:38 用户指出：信号发生器自带脉冲输出，不必用方波
# 开-关去模拟）："burst" = 猝发（WM 个数 + WP1s 触发，设备数完 count 个周期
# **自动停**，关断时刻由硬件保证，粒度 = 1/频率 = 12.5ms@80Hz）；
# 猝发指令失败（设备不认/超时）自动降级回 "timed"（软件掐表开-关）。
TRIM_PULSE_MODE = "burst"
TRIM_SETTLE_TIMEOUT_S = 5.0   # 每发之后等落定的超时
TRIM_STABLE_RANGE_MG = 2.0    # 判稳：约 1s 窗口内极差 ≤2mg 视为落定
TRIM_NO_GAIN_LIMIT = 4        # 连续多少发完全没出粉就停手（每发之间振幅上抬试探）
TRIM_PULSE_AMP_RAISE_V = 0.5  # 一发完全没出粉 → 振幅上抬的步长（11:45 实测：2V 完全
                              # 不出粉、4V 出 28mg/s，标定振幅低于起振门槛时脉冲永远发不动；
                              # 抬幅后下一发缩到最短试探，防止突然出大坨）
# 判停预测失误的补差上限（2026-10-22 18:33 用户指出"慢加直接结束了没有缓补"）：
# predictive_stop 高估尾料时提前停振 → 收尾称重偏轻。缓补开启且缺口 ≤ 此值时
# 判停后转缓补脉冲把差的补齐，不直接落"偏轻"结果。缺口更大说明工作点选错，
# 脉冲一颗颗补不现实，如实报偏轻让用户查原因。
TRIM_RESCUE_MAX_MG = 100.0


def _clamp_amp(value: float) -> float:
    """振幅统一钳位（1–4 Vpp）：任何路径写振幅都必须过这一关。"""
    return round(max(AMP_MIN_VPP, min(AMP_MAX_VPP, float(value))), 2)


def _amp_band_for(spec: "StagePlan") -> tuple[float, float]:
    """本段振幅的允许区间 = **标定振幅 ±0.5V**（v7 调幅带，2026-10-23 用户口径）。

    下界为什么是"标定值 −0.5V"：再往下就低于这种粉的起振门槛了（封堵剂1 在
    100rpm/600 下 1.39V 完全不出料、2.0V 才出 2.11 mg/s），降幅过度等于自己断料；
    实际降幅下限还会被"门槛记忆 −0.2V"进一步抬高（见 _step_amplitude）。

    上界 = 标定+0.5V：标定表里每个点的速率都是在标定振幅下测出来的，往上越远
    速率表越不可信；脉冲（踢一脚）是唯一的越带手段——而且必须"已到带顶且
    顶上 4s 速率 <1 mg/s"才许踢（三段统一 gate）。

    慢加段同样 ±0.5V：零落料时允许向上爬到 标定+0.5 恢复出料（不是追速率），
    有出粉时振幅绝对不动。
    """
    base = float(spec.amplitude_vpp)
    return (round(max(AMP_MIN_VPP, base - AMP_RAMP_BAND_V), 2),
            round(min(AMP_MAX_VPP, base + AMP_RAMP_BAND_V), 2))


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
    amplitude_vpp: float = 2.0      # 该段振幅**基准**（标定值；快加/粗加在此 ±0.2V 内调）
    amp_explicit: bool = False      # 振幅是不是页面手填的（手填就不再按上一段继承）
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
            "amplitude_range_vpp": [AMP_MIN_VPP, AMP_MAX_VPP],
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
    amplitude_vpp: float = 2.0      # 记账用：缓补期间振幅保持慢加段标定值不动


class DispenseRunner:
    """单次闭环加粉的后台执行器（同一时间只允许一个实例在跑）。"""

    SAMPLE_INTERVAL_S = 0.12      # 观测周期：比页面 0.5s 快，够用到观测器的 0.75s 预测
    MAX_RUN_S = 180.0             # 基准超时；大目标会按计划估算放大（见 _resolve）
    OVERSHOOT_ABORT_MG = 30.0     # 兜底超重保护（验收是 ±10，留 20mg 给惯性/抖动）
    MAX_BALANCE_FAILS = 5         # 连续读不到天平 ≈ 0.6s 就停，闭环不能"盲跑"
    SETTLE_TIMEOUT_S = 6.0        # 停振后等尾料沉降
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
        # 有没有读过本炉基线：没有基线时页面不显示"净质量"，
        # 免得把秤上的绝对示数（可能还是上一炉的皮重）当成这一炉的增量。
        self._has_baseline = False
        self._rod_start = 0
        self._applied_freq = 0
        self._applied_duty = 0
        self._applied_window = 0
        self._applied_stir = 0.0
        self._applied_amp = 0.0
        self._amp_last_write_s = 0.0    # 上次写振幅的时刻（响应等待窗起点）
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
        self._amp_threshold_est: float | None = None  # 起振门槛：产生落料的最低振幅
        self._stall_wait_s = -99.0                    # 卡粉救济上一次动作时刻
        self._amp_kick: dict | None = None  # 踢一脚进行中：{amp, base, until}
        self._amp_kick_watch: dict | None = None  # 踢后观察窗：{amp, fired, until}
        self._amp_kick_tries = 0      # 本段已踢了几脚（段入口复位）
        self._amp_kick_last_amp = 0.0 # 上一脚的振幅（判断是否已到顶）
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
        self._kick_count = 0                # 本炉累计踢脚次数
        self._t0 = 0.0
        self._max_run_s = self.MAX_RUN_S
        self._internal_stop = ""
        self._last_raw: int | None = None     # 最后一次成功读数：天平失联时当估计终值
        self._plan: list[StagePlan] = []
        self._workspace: PowderWorkspace | None = None
        # 切段沉降门（本次运行的开关与阈值，页面可覆盖；默认值见类常量）
        self._gate_enabled = True
        self._gate_trigger = SETTLE_GATE_RATE_TRIGGER_MG_S
        self._taper_factor = TAIL_TAPER_FACTOR
        self._vib_running = False             # 振动输出当前是否开着（沉降门会临时关掉）
        self._settle_events: list[dict] = []  # 每次切段测到的"停振后还落了多少"
        self._taper_done_index = -1           # 已经提前降过幅的段（每段只降一次）
        self._taper_hold = False              # 切点前降幅窗口内：暂停振幅环

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
        """把天平原始计数换算成「本炉净质量」（计数 − 起炉基线）。

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

        # 切段沉降门参数：页面没传就沿用默认（重粉那套）。校验放在动任何机构之前。
        gate_on = True if settle_gate is None else bool(settle_gate)
        gate_trigger = (SETTLE_GATE_RATE_TRIGGER_MG_S if settle_gate_rate_mg_s is None
                        else float(settle_gate_rate_mg_s))
        if not 0.0 <= gate_trigger <= 300.0:
            raise ValueError(f"沉降门触发速率 {gate_trigger:g} mg/s 超出 0–300")
        taper = TAIL_TAPER_FACTOR if taper_factor is None else float(taper_factor)
        if not 0.0 <= taper <= 3.0:
            raise ValueError(f"提前降幅系数 {taper:g} 超出 0–3（0 = 不提前降幅）")
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
                                "rate_mg_s": round(gate_trigger, 1),
                                "max_s": SETTLE_GATE_MAX_S,
                                "taper_factor": round(taper, 2)},
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
        self._trim_enter_mg = trim_enter
        self._trim_pulse_mg = trim_pulse_mg
        self._trim_rescued = False       # 判停失误补差每炉只允许一次
        # 沉降门参数落到实例上：主循环只在同一个线程里读，不需要再加锁
        self._gate_enabled = gate_on
        self._gate_trigger = gate_trigger
        self._taper_factor = taper
        self._vib_running = False
        self._settle_events = []
        self._taper_done_index = -1
        self._taper_hold = False
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
            f" / 振幅 {r['amp_vpp']:g}Vpp（快加/粗加在标定值 ±{AMP_BAND_V:g}V 内微调、断料时踢一脚最高 +{AMP_ESCALATE_V:g}V，"
            f"慢加连续 ≤ 标定+{AMP_SLOW_RAISE_V:g}V），分段："
            + self._plan_brief(plan, target))
        if gate_on:
            self._log(f"切段沉降门：开（切出速率 ≥{gate_trigger:g} mg/s 时停振等余料落完，"
                      f"最多 {SETTLE_GATE_MAX_S:g}s/段；提前降幅 {taper:g}×速率、"
                      f"最少 {TAIL_TAPER_MIN_MG:g}mg）")
        else:
            self._log("切段沉降门：关（达标即刻切段，余料会顶进下一段）")
        if self._trim_on:
            src = "页面指定" if trim_enabled is not None else "粉体档案"
            self._log(f"缓补：开（{src}；慢加剩 ≤{self._trim_enter_mg:.0f}mg 停连续振动，"
                      f"改为 ≈{self._trim_pulse_mg:.0f}mg/发 的脉冲补料，一发一发凑到目标）")
        else:
            self._log(f"缓补：关（慢加直接判停收尾；需要时在页面「缓补脉冲」勾选开启）")
        for spec in plan:
            if spec.switch_compensation_mg >= 0.5:
                self._log(f"{STAGE_TEXT[spec.name]}切换点按学到的惯性落料提前 "
                          f"{spec.switch_compensation_mg:g} mg（用该粉历史数据）")
        return self.status()

    def preview(self, target_mg: Any, *, powder: str | None = None,
                category: str | None = None, preset_id: str | None = None,
                amplitude_vpp: float | None = None, tail_mass_mg: float | None = None,
                use_feedforward: bool = True,
                stages: list[dict] | None = None,
                settle_gate: bool | None = None,
                settle_gate_rate_mg_s: float | None = None,
                taper_factor: float | None = None) -> dict:
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
            # 沉降门参数原样回显，页面用它确认"这次按什么口径切段"
            "settle_gate": {
                "enabled": True if settle_gate is None else bool(settle_gate),
                "rate_mg_s": round(SETTLE_GATE_RATE_TRIGGER_MG_S if settle_gate_rate_mg_s is None
                                   else float(settle_gate_rate_mg_s), 1),
                "max_s": SETTLE_GATE_MAX_S,
                "taper_factor": round(TAIL_TAPER_FACTOR if taper_factor is None
                                      else float(taper_factor), 2),
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
        # 振幅是本次唯一的速率调节量，只在 1–4 Vpp 内工作（2026-10-21 用户定）。
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
                                  f"振幅会一直顶在 {AMP_MAX_VPP:g}V 也追不上——"
                                  f"这是手填的值，想按能力执行就把速率留空")

        estimated = 0.0
        prev = 0.0
        for spec in plan:
            end = float(spec.switch_mass_mg) if spec.switch_mass_mg is not None else float(target)
            estimated += max(0.0, end - prev) / max(0.3, spec.rate_mg_s)
            prev = end
        # 超时上限跟着计划走：大目标用 180s 会一刀砍在半路；20000mg 慢速工况要 20 分钟以上
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
            # 振幅起点：手填了就用填的；没填就用全局起始振幅，运行时后段会继承前段实际值
            amp_explicit = "amplitude_vpp" in cfg
            amp = _clamp_amp(float(cfg["amplitude_vpp"]) if amp_explicit else amp_vpp)
            # 切段等待：留空 = 用全局上限（重粉 1.2s），显式 0 = 这一段切出时不停振等
            settle_wait = cfg.get("settle_wait_s")
            plan.append(StagePlan(
                name=name, mode=mode, rate_mg_s=rate, stir_rpm=stir, switch_mass_mg=switch,
                rod_ff=rod_ff, rod_min=rod_lo, rod_max=rod_hi,
                duty_seed_permyriad=stage_duty, duty_min=duty_lo, duty_max=duty_hi,
                # 推杆整段锁死：速率只由振幅调（用户 2026-10-21 要求，别乱动推杆）
                position_feedback=False,
                amplitude_vpp=amp, amp_explicit=amp_explicit,
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
        # 先退猝发待命（WO0s+Wc0s）：上一炉缓补若用过真脉冲，设备还停在
        # "手动触发模式"——输出通道开着但只认 WP1s 触发，不退出来的话这一炉
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
        self._amp_last_eval_s = 0.0
        self._amp_probe = None
        self._amp_band = _amp_band_for(first)
        self._amp_lock_reason = ""
        self._amp_kick = None
        self._amp_kick_watch = None
        self._amp_kick_tries = 0
        self._amp_kick_last_amp = 0.0
        self._mass_hist = []
        # 第一段的振幅统计（后面切段由 _enter_stage 重置）
        self._amp_changes = 0
        self._amp_saturated = False
        self._amp_min = self._applied_amp
        self._amp_max = self._applied_amp
        self._amp_heartbeat_s = 0.0
        self._stage_start_s = 0.0
        self._stage_start_mass = 0.0

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
                     else f"自适应调幅（标定±{AMP_RAMP_BAND_V:g}V 带内，"
                          f"步长 {AMP_STEP_MIN_V:g}–{AMP_STEP_MAX_V:g}V 按缺口/增益；"
                          f"踢脚必须到带顶且顶上 {AMP_KICK_GATE_WINDOW_S:.0f}s"
                          f" 速率 <{AMP_KICK_GATE_RATE_MG_S:g} mg/s）"))

        if self._stop_event.is_set():
            raise _Abort(self._internal_stop)
        self._t0 = time.monotonic()
        self._start_stage_log(first, 0.0, 0.0)
        self._set_state("running",
                        f"起振：{self._config['frequency_hz']:g}Hz / "
                        f"{first.duty_seed_permyriad / 100:.1f}% 占空比 / "
                        f"振幅 {self._applied_amp:.2f}Vpp，"
                        f"搅拌 {first.stir_rpm:.0f} RPM")
        self.hub.sig_output(True)
        self._vib_running = True
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
        while not self._stop_event.is_set():
            elapsed = time.monotonic() - self._t0
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
            if (stage_index < len(plan) - 1
                    and spec.switch_mass_mg is not None
                    and mass >= spec.switch_mass_mg):
                self._close_stage_log(mass, elapsed)
                # 沉降门：这一段的料还没落完就换配方，余料会直接顶进下一段
                landed = self._settle_gate(spec, plan[stage_index + 1], mass, elapsed)
                if landed["applied"]:
                    mass = landed["mass"]
                    elapsed = landed["elapsed"]
                    self._note_settle(landed)
                # 沉降期间操作员按了停止：就此收手，别再去开下一段的推杆
                if self._stop_event.is_set():
                    raise _Abort(getattr(self, "_internal_stop", "操作员停止"))
                stage_index += 1
                spec = plan[stage_index]
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
                self._start_stage_log(spec, mass, elapsed)

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
                self._set_state("running",
                                f"{STAGE_TEXT[spec.name]} {mass:.0f}/{target} mg"
                                f"（{motion.rate_mg_s:+.1f} mg/s，"
                                f"幅 {self._applied_amp:.2f}Vpp，"
                                f"杆 {self._applied_window}，搅 {self._applied_stir:.0f}rpm，"
                                f"占空 {self._applied_duty / 100:.1f}%）")
            if decision.stop_requested:
                self._close_stage_log(mass, elapsed)
                return decision.reason or "控制器判定停止"
            time.sleep(self.SAMPLE_INTERVAL_S)
        self._close_stage_log(self._live.get("mass_mg", 0.0), time.monotonic() - self._t0)
        # 停止请求走"中止"路径：这一炉没有跑到控制器判定，结果必须标成未成功
        raise _Abort(getattr(self, "_internal_stop", "操作员停止"))

    # ------------------------------------------------------------------
    # 阶段动作
    # ------------------------------------------------------------------

    def _enter_stage(self, spec: StagePlan, mass: float, elapsed: float) -> None:
        """进入新阶段：先换搅拌，再调振幅、写占空比，最后把推杆开到该段开度。

        顺序刻意如此：搅拌决定"料松不松"，振幅决定"振动多强"，推杆决定"口开多大"。
        先把料搅松、振动调到该段的强度，再张口，流量上升是平滑的台阶；
        反过来的话，张口瞬间可能先掉一团结块。
        """
        self._log(f"进入{STAGE_TEXT[spec.name]}：{mass:.1f} mg / 目标 "
                  f"{self._config['target_mg']} mg，目标流量 {spec.rate_mg_s:.1f} mg/s，"
                  f"搅拌 {spec.stir_rpm:.0f} RPM，振幅 {spec.amplitude_vpp:.2f} Vpp，"
                  f"前馈开度 {spec.rod_ff}"
                  f"（预期 {spec.rod_ff_expected_rate if spec.rod_ff_expected_rate is not None else '—'} mg/s）")
        if abs(spec.stir_rpm - self._applied_stir) > 15:
            self._set_stir(spec.stir_rpm, wait=False)
        # 振幅：**进段取标定振幅；但若上一段实测到"起振门槛"高于标定值**
        # （粉位低/粉种特性，12:38 实测 2V 一粒不出、3V 才出），就从门槛附近起步，
        # 免得每段开头都空转一整个响应窗才发现不出料。
        base_amp = float(spec.amplitude_vpp)
        if spec.name == "slow":
            # 慢加起步**一律用页面/标定振幅**（用户 17:29：设 2V 就跑 2V）。
            # 旧版"门槛−0.5V 起步"（14:44 加的）在润滑剂1 上把慢加顶到 3.0V、
            # 实测 28mg/s，2.7s 冲完慢加段直接超差 +45mg——起振门槛记忆
            # 从此不用于慢加起步；2V 起不出料就靠踢一脚（0.5s 脉冲）救。
            new_amp = _clamp_amp(base_amp)
        elif (self._amp_threshold_est is not None
                and self._amp_threshold_est > base_amp + AMP_BAND_V):
            # 快加/粗加要流量：门槛高于标定+0.2V 就从门槛起步，免得每段开头
            # 空转一整个响应窗才发现不出料。
            new_amp = _clamp_amp(max(base_amp, float(self._amp_threshold_est)))
            self._log(f"起振门槛记忆：本炉实测 {self._amp_threshold_est:.2f}V 才落得下粉"
                      f"（标定 {base_amp:.2f}V 低于当前门槛），{STAGE_TEXT[spec.name]}"
                      f"从 {new_amp:.2f}V 起步")
        else:
            new_amp = _clamp_amp(base_amp)
        self._set_amplitude(new_amp, elapsed, f"进入{STAGE_TEXT[spec.name]}")
        self._mass_hist = []
        self._amp_last_eval_s = elapsed
        self._amp_probe = None
        self._amp_band = _amp_band_for(spec)
        self._amp_lock_reason = ""
        # 段内相位复位；τ/T/门槛/增益是"这种粉"的性质，跨段保留
        self._amp_write_marks = []
        self._prev_eval_rate = None
        self._last_mass_for_drop = None
        self._last_drop_s = None
        self._stall_wait_s = -99.0
        self._amp_kick = None
        self._amp_kick_watch = None
        self._amp_kick_tries = 0
        self._amp_kick_last_amp = 0.0
        self._ramp_below_since = None   # 粗调坡道计时按段重置
        self._ramp_last_s = -99.0
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

        target_rod = int(spec.rod_ff)
        self._sig_write(lambda: self.hub.sig.set_duty(spec.duty_seed_permyriad / 100.0))
        self._applied_duty = spec.duty_seed_permyriad
        if abs(target_rod - self._applied_window) >= 2:
            self.hub.rod_move(target_rod)         # 只下发不等待：闭环不能被 0.6s 行程卡住
            self._applied_window = target_rod
        # 推杆是硬约束：本段开度就从标定点照抄，全程不再动（用户 2026-10-22 明确要求）。
        # 这句日志是"锁死"的凭证，事后复盘能直接对上"这一段推杆有没有被改过"。
        amp_note = (f"振幅 {self._applied_amp:.2f}V（慢加不追速率；连续振幅最高 "
                    f"{float(spec.amplitude_vpp) + AMP_SLOW_RAISE_V:.2f}V = 标定"
                    f"+{AMP_SLOW_RAISE_V:g}V，真卡粉才踢一脚 0.5s 脉冲、踢完回落）"
                    if spec.name == "slow" else                    f"振幅 {self._amp_band[0]:.2f}–{self._amp_band[1]:.2f}V"
                    f"（标定 {float(spec.amplitude_vpp):.2f}V：±{AMP_BAND_V:.2f}V 内微调，"
                    f"断料时踢一脚最高 +{AMP_ESCALATE_V:.2f}V）")
        self._log(f"推杆锁定 {target_rod}（来自标定点/页面手填，整段不动）；{amp_note}")
        # 上一次切段沉降门把振动关了的话，进新段必须重新起振——否则整段一点粉都不出，
        # 振幅环会把振幅一路顶到 4V 还追不上（这是最容易漏的一步）。
        if not self._vib_running:
            self.hub.sig_output(True)
            self._vib_running = True
            self._log(f"振动输出重新打开（{STAGE_TEXT[spec.name]}开始出料）")

    # ------------------------------------------------------------------
    # 切段沉降门（重粉关键：切段那一刻料还没落完）
    # ------------------------------------------------------------------

    def _settle_gate(self, prev: StagePlan, nxt: StagePlan,
                     mass: float, elapsed: float) -> dict:
        """切段沉降门：切出速率高时先停振，等余料落干净再换配方。

        为什么必须做：质量到达切换点 ≠ 粉已经落完。振动一停，料槽里和半空中的粉
        还会继续落到秤上（重粉落差短但单次落料量大）。不等的话这部分余料会直接算进
        下一段——粗加一开场就超过自己的切换点，慢加要倒着补，最终必然偏重。

        动作顺序：**只关振动**，搅拌保持（防搭桥、帮余料走完），推杆不动（不许为了
        "等"去挪机构）。判稳条件：连续 5 个样本窗口极差 ≤2mg 且斜率 ≤0.5 mg/s；
        超时（默认 1.2s，重粉够用）就用当前质量直接切——绝不在这里卡住。

        返回 ``{"applied", "mass", "elapsed", "gain_mg", "waited_s", "stable", ...}``，
        ``gain_mg`` 就是"停振后还多落了多少"，会被写进粉体档案当下次的提前量。
        """
        out = {"applied": False, "mass": mass, "elapsed": elapsed,
               "gain_mg": 0.0, "waited_s": 0.0, "stable": False, "rate_mg_s": 0.0}
        if not self._gate_enabled:
            return out
        wait_limit = (prev.settle_wait_s if prev.settle_wait_s is not None
                      else SETTLE_GATE_MAX_S)
        if wait_limit <= 0:
            return out
        # 触发判定：用上一段最近这几秒的回归速率（和振幅环同一套算法）。
        # 以前读 _rate_hist，那是"瞬时速率平均值"的旧字段，振幅环改用质量回归后
        # 已经没人往里写 —— 不改这里沉降门会永远判 0 速率、一次都不触发。
        meas = self._measure_rate(3.0, 4.0)
        rate_now = float(meas["rate"]) if meas else 0.0
        if abs(rate_now) < self._gate_trigger:
            return out

        self._set_state("running",
                        f"切段沉降：{STAGE_TEXT[prev.name]}→{STAGE_TEXT[nxt.name]}，"
                        f"停振等余料落完（≤{wait_limit:g}s）")
        self._log(f"切段沉降：{STAGE_TEXT[prev.name]}→{STAGE_TEXT[nxt.name]}，"
                  f"切出速率 {rate_now:.1f} mg/s（≥{self._gate_trigger:g}），"
                  f"停振等余料落完（最多 {wait_limit:g}s），搅拌/推杆保持不动")
        self._safe_stop_vibration()
        t0 = time.monotonic()
        hist: list[tuple[float, float]] = []
        stable = False
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
            hist.append((now, float(raw) - self._baseline))
            if len(hist) >= SETTLE_GATE_SAMPLES:
                window = hist[-SETTLE_GATE_SAMPLES:]
                span = max(0.05, window[-1][0] - window[0][0])
                slope = abs((window[-1][1] - window[0][1]) / span)
                spread = max(p[1] for p in window) - min(p[1] for p in window)
                if spread <= SETTLE_GATE_STABLE_MG and slope <= SETTLE_GATE_RATE_EPS_MG_S:
                    stable = True
                    break
            time.sleep(0.1)
        waited = time.monotonic() - t0
        landed = hist[-1][1] if hist else mass
        out.update(applied=True, mass=landed, elapsed=time.monotonic() - self._t0,
                   gain_mg=round(landed - mass, 1), waited_s=round(waited, 2),
                   stable=stable, rate_mg_s=round(rate_now, 1),
                   from_stage=prev.name, to_stage=nxt.name)
        self._log(f"沉降结束：余料 {landed - mass:+.1f} mg（{STAGE_TEXT[prev.name]}），"
                  f"等 {waited:.1f}s（{'已判稳' if stable else '到时未稳，直接切'}），"
                  f"当前 {landed:.1f} mg")
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
        self._taper_hold = True

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
                      skip_before: float = 0.0) -> dict | None:
        """用质量累积回归算速率——振幅环唯一的速率来源。

        为什么不用 estimate 的瞬时速率：封堵剂1 是脉冲出料，天平只在粉块落下时
        跳一下、中间是平的，差分量算出来 77% 的采样是 0（中位数 0.00）。
        拿这种信号做反馈，振幅就会在两个相邻值之间来回抖（实测粗加 49 秒里
        改了 14 次、全在 1.82–2.06V 之间晃），纯属白折腾信号源。

        返回 None = "现在还没有可用的速率信息"，调用方必须**什么都不做**：
        样本不足、窗口太短、窗口内增重太小（粉还没落下来）都算这种情况。
        ``skip_before`` 用来切掉"刚改完振幅的响应期"（那会儿流量还是旧振幅的）。
        """
        with self._lock:
            hist = list(self._mass_hist)
        if len(hist) < 8:
            return None
        left = hist[-1][0] - float(window_s)
        pts = [point for point in hist
               if point[0] >= left and point[0] >= skip_before]
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
        except Exception as error:      # noqa: BLE001 - 搅拌失败不该让整炉直接崩
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
        """振幅调节 v7：自适应小步 + 统一踢脚 gate（2026-10-23 09:52 用户定规矩）。

        用户原话："粗加调节幅度时变化太僵硬……根据实时速率调节，比如输入 2.5V
        启动，每次调整 0.05-0.15V，根据前 5 秒平均速率和加速度、和当前的需求
        速率这些值来进行幅度的增加或者删减"；"慢加和粗加和快加的脉冲触发条件
        必须是：反馈调节已经调节到顶……仍长时间无速率（4s 时间内速率小于 1mg/s）"。

        可得到的值（全部实测，不用任何仿真模型）：
          mass(t)      天平累积质量（单调、±1mg 噪声）
          drop event   单次采样增重 >=AMP_DROP_EVENT_MG = 一次真实落料（时间戳）
          tau_est      写振幅 -> 第一次落料的间隔（在线 EMA；纯滞后）
          T_est        相邻落料间隔（在线 EMA；落料周期）
          k_est        delta_rate/delta_amp（在线学习；缺口÷增益 = 步长）
          5s 均速/加速度  自适应调幅的输入（加速度 = 相邻评估均速变化率，EMA 平滑）

        决策顺序（每拍，命中即停）：
          0. 冻结/踢脚中/踢脚观察窗（第零层，见上方历史注释）
          1. 踢脚 gate（三段统一）：已到带顶（标定+0.5V）且顶上停满 4s
             且 4s 窗速率 <1 mg/s → _stall_relief（不到顶绝不踢）
          2. 滞后闸：写幅后 <4s 不评估（0.15V 小步的效果由加速度外推补）
          3. 5s 均速测不出：有落料等窗口；零落料丝滑爬坡（+0.12V/3s，
             慢加 +0.10V/2s——恢复出料不是追速率）
          4. 加速度外推：v_pred = 5s 均速 + a×min(τ,3s)；缺口 = 目标−v_pred
          5. 死区 max(2, 目标×15%) 内不动；步长 = 缺口÷k_est 钳 0.05–0.15V；
             慢加只升不降、步长 ≤0.10V；升顶 = 带顶、降底 = max(标定−0.5, 门槛−0.2)
        """
        cmd = self._rate_command_for(spec, mass)
        base = float(spec.amplitude_vpp)
        ctrl = AMP_RATE_CTRL.get(spec.name, AMP_RATE_CTRL["coarse"])

        if self._taper_hold:
            self._amp_heartbeat(spec, mass, elapsed, cmd, "切点前降幅窗口，暂停调幅")
            return
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
            # 门槛判定**不在踢完瞬间做**（14:30 实机教训）：粉被踢松后要滞后
            # τ 才落到秤上，踢完立刻看天平必然"没出料"。开观察窗等到 τ+T，
            # 窗内任何落料都算这一脚生效 → 工作振幅上移（在评估循环里做）。
            self._amp_kick_watch = {"amp": float(kick["amp"]),
                                    "fired": float(kick["fired"]),
                                    "m0": float(kick["m0"]),
                                    "until": elapsed + self._tau_est
                                    + self._period_est}
            self._set_amplitude(
                float(kick["base"]), elapsed,
                f"踢一脚完成（{kick['amp']:.2f}V × {AMP_KICK_HOLD_S:.1f}s）→ "
                f"回落 {kick['base']:.2f}V，开 {self._tau_est + self._period_est:.0f}s "
                f"观察窗：落料出现就把工作振幅上移到有效踢脚−0.2V")
            return
        # 观察窗：踢脚后的落料（哪怕滞后 1~2s 才上秤）证明起振门槛 ≤ 踢脚振幅。
        # 门槛记忆更新 + 工作振幅上移到 有效踢脚−0.2V 连续出料——踢脚只点火。
        watch = self._amp_kick_watch
        if watch is not None:
            productive = (
                (self._last_drop_s is not None
                 and self._last_drop_s >= float(watch["fired"]) - 0.2)
                or float(mass) - float(watch["m0"]) >= 3.0)
            if productive:
                cap = round(base + AMP_ESCALATE_V, 2)
                # 慢加的工作点上移也留 0.5V 余量（涓流），其余段 0.2V
                margin = 0.5 if spec.name == "slow" else 0.2
                new_amp = round(min(cap, float(watch["amp"]) - margin), 2)
                if spec.name == "slow":
                    # 慢加连续振幅硬上限 = 标定+0.5V（用户 17:29：不超 2.5V）。
                    # 踢脚哪怕 3.5V 才踢开，连续工作也最多回到 2.5V 涓流。
                    new_amp = min(new_amp, round(base + AMP_SLOW_RAISE_V, 2))
                if (self._amp_threshold_est is None
                        or float(watch["amp"]) > float(self._amp_threshold_est)):
                    self._amp_threshold_est = float(watch["amp"])
                self._amp_kick_watch = None
                self._stall_wait_s = elapsed
                self._amp_probe = None
                self._prev_eval_rate = None
                if new_amp > float(self._applied_amp) + 0.015:
                    self._set_amplitude(
                        new_amp, elapsed,
                        f"踢脚生效（{watch['amp']:.2f}V 踢出了落料）→ 门槛记忆："
                        f"工作振幅上移到 {new_amp:.2f}V 连续出料")
                return
            if elapsed > float(watch["until"]):
                self._amp_kick_watch = None   # 等满 τ+T 也没落料 → 这脚确实没用

        # ── v7 自适应调幅（2026-10-23 09:52 用户定规矩，取代粗调坡道）─────────
        # 调幅带 = 标定 ±0.5V；平时自适应小步（0.05–0.15V）；
        # 脉冲（踢一脚）三段统一触发：已调到带顶 且 到顶后持续 ≥4s 速率 <1 mg/s。
        cap_v = round(min(AMP_MAX_VPP, base + AMP_RAMP_BAND_V), 2)
        floor_v = round(max(AMP_MIN_VPP, base - AMP_RAMP_BAND_V), 2)
        if self._amp_threshold_est is not None:
            floor_v = max(floor_v, round(float(self._amp_threshold_est) - 0.2, 2))
        at_cap = float(self._applied_amp) >= cap_v - 0.015
        if at_cap:
            if self._amp_at_cap_since is None:
                self._amp_at_cap_since = elapsed
        else:
            self._amp_at_cap_since = None

        # -- 踢脚 gate（快加/粗加/慢加统一）：顶上磨满 4s 且 4s 窗速率 <1 才踢 --
        # "反馈调节已经调节到顶……仍长时间无速率（4s 时间内速率小于 1mg/s）"
        meas4 = self._measure_rate(AMP_KICK_GATE_WINDOW_S, 1.0, skip_before=None)
        gate_rate = None if meas4 is None else float(meas4["rate"])
        if (at_cap
                and self._amp_at_cap_since is not None
                and elapsed - float(self._amp_at_cap_since) >= AMP_KICK_GATE_WINDOW_S
                and (gate_rate is None or gate_rate < AMP_KICK_GATE_RATE_MG_S)):
            self._stall_relief(
                spec, mass, elapsed, cmd,
                detail=(f"已在带顶 {cap_v:.2f}V 停留 "
                        f"{elapsed - float(self._amp_at_cap_since):.0f}s，近 4s 速率 "
                        + (f"{gate_rate:.1f}" if gate_rate is not None
                           else "≈0（无有效回归）")
                        + f" < {AMP_KICK_GATE_RATE_MG_S:g} mg/s"))
            return

        # -- 滞后闸：写幅后至少等 4s 再评估（比 τ 短——0.15V 小步的效果由下面的
        #    加速度外推补足，不必每次全额等 τ≈8s，否则节奏僵硬）----------------
        settled = elapsed - float(self._amp_last_write_s)
        if settled < AMP_REEVAL_MIN_S:
            self._amp_heartbeat(
                spec, mass, elapsed, cmd,
                f"等上次调幅的效果（{settled:.0f}/{AMP_REEVAL_MIN_S:.0f}s，"
                f"τ≈{self._tau_est:.0f}s 由预测外推补）")
            return

        # -- 测量：前 5s 平均速率（用户举例口径）--------------------------------
        meas = self._measure_rate(AMP_LOOKBACK_S, 2.0, skip_before=None)
        last_drop = self._last_drop_s
        if meas is None:
            if last_drop is not None and last_drop >= elapsed - AMP_LOOKBACK_S:
                # 窗内有落料只是样本不够回归——等，不猜（旧版拿短窗凑速率的教训）
                self._amp_heartbeat(
                    spec, mass, elapsed, cmd,
                    f"5s 窗样本不足但有落料（{elapsed - last_drop:.0f}s 前）——窗口积累中")
                return
            # 近 5s 一粒粉没落 → 丝滑爬坡（慢加也爬：恢复出料，不是追速率）
            if at_cap:
                self._amp_heartbeat(
                    spec, mass, elapsed, cmd,
                    f"5s 窗零落料，已在带顶 {cap_v:.2f}V——等踢脚 gate"
                    f"（顶上满 {AMP_KICK_GATE_WINDOW_S:.0f}s 且速率"
                    f" <{AMP_KICK_GATE_RATE_MG_S:g}）")
                return
            step = (AMP_SLOW_CLIMB_STEP_V if spec.name == "slow"
                    else AMP_RAMP_STEP_ZERO_V)
            interval = (AMP_SLOW_CLIMB_INTERVAL_S if spec.name == "slow"
                        else AMP_RAMP_INTERVAL_ZERO_S)
            if elapsed - float(self._ramp_last_s) < interval:
                self._amp_heartbeat(
                    spec, mass, elapsed, cmd,
                    f"零落料爬坡冷却（每 {interval:.0f}s 抬 {step:.2f}V → 带顶 {cap_v:.2f}V）")
                return
            self._ramp_last_s = elapsed
            self._amp_changes += 1
            self._amp_probe = {"amp": float(self._applied_amp), "rate": 0.0,
                               "step": step, "elapsed": elapsed}
            self._set_amplitude(
                round(min(cap_v, float(self._applied_amp) + step), 2), elapsed,
                f"{STAGE_TEXT[spec.name]}零落料爬坡：近 5s 一粒粉没落 → 抬 +{step:.2f}V"
                f"（带顶 {cap_v:.2f}V），不脉冲冲击")
            return
        rate = float(meas["rate"])
        detail = (f"5s 均速 {rate:.1f} mg/s（{meas['span']:.1f}s/{meas['gain']:+.0f}mg"
                  f"/R²{meas['r2']:.2f}）")

        # 增益学习：上一次"升幅→观察"闭环走完了，算 k_sample
        probe = self._amp_probe
        if probe is not None and float(probe.get("step", 0)) > 0:
            self._amp_probe = None
            delta_amp = abs(float(probe["step"]))
            delta_rate = rate - float(probe["rate"])
            if delta_amp >= 0.05:
                k_sample = delta_rate / delta_amp
                if -50.0 <= k_sample <= 50.0:
                    self._k_est = (round(k_sample, 2) if self._k_est is None
                                   else round(0.6 * self._k_est + 0.4 * k_sample, 2))
                    self._log(f"[振幅] 增益学习：{probe['rate']:.1f}→{rate:.1f} mg/s / "
                              f"{probe['step']:+.2f}V → k_est={self._k_est:.1f} mg/s/V")

        # -- 卡粉判定（有速率信息但远低于目标也算）-----------------------------
        # 2026-10-23 09:52 起由上面的"踢脚 gate"统一管（三段一致：到顶+4s<1），
        # 这里不再单独判。

        # -- 增益学习：上一次"调幅→观察"闭环走完了，算 k_sample ----------------
        probe = self._amp_probe
        if probe is not None and float(probe.get("step", 0)) > 0:
            self._amp_probe = None
            delta_amp = abs(float(probe["step"]))
            delta_rate = rate - float(probe["rate"])
            if delta_amp >= 0.05:
                k_sample = delta_rate / delta_amp
                if -50.0 <= k_sample <= 50.0:
                    self._k_est = (round(k_sample, 2) if self._k_est is None
                                   else round(0.6 * self._k_est + 0.4 * k_sample, 2))
                    self._log(f"[振幅] 增益学习：{probe['rate']:.1f}→{rate:.1f} mg/s / "
                              f"{probe['step']:+.2f}V → k_est={self._k_est:.1f} mg/s/V")

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
        deadband = max(2.0, abs(float(cmd)) * 0.15)
        if abs(error) <= deadband:
            self._amp_heartbeat(
                spec, mass, elapsed, cmd,
                f"{detail}，加速度 {a_est:+.1f} mg/s² → 预测 {v_pred:.1f}，"
                f"缺口 {error:+.1f} 在死区（±{deadband:.1f}）内，不动")
            return

        # -- 步长：缺口 ÷ 增益，钳 0.05–0.15V（用户举例口径，丝滑不跳变）--------
        k_eff = (float(self._k_est) if self._k_est is not None and self._k_est >= 1.0
                 else AMP_DEFAULT_K)
        step = max(AMP_STEP_MIN_V, min(AMP_STEP_MAX_V, abs(error) / k_eff))
        if spec.name == "slow":
            # 慢加：只许往上爬（恢复出料），不追速率、偏快不压
            if error <= 0 or rate >= AMP_KICK_GATE_RATE_MG_S:
                self._amp_heartbeat(
                    spec, mass, elapsed, cmd,
                    f"{detail}——慢加不追速率（偏快按实际速率收），振幅不动")
                return
            step = min(step, AMP_SLOW_CLIMB_STEP_V)
        # 评估间隔：相邻调幅至少 AMP_EVAL_INTERVAL_S（丝滑节奏）
        if elapsed - float(self._ramp_last_s) < AMP_EVAL_INTERVAL_S:
            self._amp_heartbeat(
                spec, mass, elapsed, cmd,
                f"{detail}，加速度 {a_est:+.1f} → 预测 {v_pred:.1f}，"
                f"备步 {step:+.2f}V，等 {AMP_EVAL_INTERVAL_S:.0f}s 评估间隔")
            return

        applied = float(self._applied_amp)
        if error > 0:
            # 升幅：上限 = 带顶；已被救济抬到带顶之上（恢复态）就沿当前值，不许拽回
            limit_high = max(cap_v, round(applied, 2))
            target_amp = round(min(limit_high, applied + step), 2)
        else:
            # 降幅：下限 = max(标定−0.5, 门槛−0.2)——低于起振门槛会又断料
            target_amp = round(max(floor_v, applied - step), 2)
        if abs(target_amp - applied) < 0.015:
            self._amp_heartbeat(
                spec, mass, elapsed, cmd,
                f"{detail}，已在边界（带 {floor_v:.2f}–{cap_v:.2f}V），不动")
            return
        self._ramp_last_s = elapsed
        self._amp_changes += 1
        self._amp_probe = {"amp": applied, "rate": rate,
                           "step": target_amp - applied, "elapsed": elapsed}
        self._set_amplitude(
            target_amp, elapsed,
            f"{STAGE_TEXT[spec.name]}自适应 {detail}，加速度 {a_est:+.1f} mg/s² → "
            f"预测 {v_pred:.1f}/{cmd:.1f} mg/s → {target_amp:.2f}V"
            f"（步 {target_amp - applied:+.2f}V，第 {self._amp_changes} 次）")

    def _stall_relief(self, spec: StagePlan, mass: float, elapsed: float,
                      cmd: float, detail: str = "") -> None:
        """卡粉/断料救济：踢一脚——振幅瞬间抬到 标定+1.0V，持续 0.5s 后立刻回落。

        用户 2026-10-22 13:31 定的招："模仿脉冲，突然变为 3V 持续 0.5s 即可回到
        2V，粉体一般就不卡了"。依据是 12:38 真机实测：2Vpp 短脉冲一粒不出、
        3Vpp 0.6s +37mg——一脚正好跨过起振门槛，比 0.5V/步逐级磨快得多；
        高幅只停 0.5s，也不会像连续抬幅那样把速率冲飞。
        节奏：一脚踢完回落标定值 → 等 τ+T（一个完整生效窗）看落料；还没恢复
        → 下一脚在此基础上再 +AMP_KICK_ESCALATE_V；硬顶 = 标定+AMP_ESCALATE_V，
        到顶仍不出 → 停手提示查料斗。慢加段同样走这里（只许"恢复出料"，
        不许追速率，速率偏慢的场合在调用方已被拦下）。

        2026-10-22 13:56 实机抓到的两个坑（修复）：
        ① 锚点不能用 spec 标定值——进慢加时"起振门槛记忆"可能已经把振幅抬到
           标定之上（2.0→2.5V），按标定算踢脚会**往下踢**（2.50→2.00V），
           等于亲手把唯一能出料的振幅掐灭；必须锚定 max(标定, 当前实际值)。
        ② 救济通道不看 _amp_band——慢加的带退化成一点 (2.0, 2.0)，会把踢脚
           顶死在标定值上（日志"顶 2.00V"）。踢脚上限 = 标定+AMP_ESCALATE_V，
           与微调带无关。
        """
        base = float(spec.amplitude_vpp)
        if spec.name == "slow":
            # 慢加踢脚锚点 = 标定值（用户 13:31 原方：2V 标定就踢到 3V×0.5s）。
            # 慢加连续振幅已被钳在 标定+0.5V 以内，标定+1V 起踢保证永远向上踢、
            # 也不会随工作点上移把踢脚越滚越高。
            anchor = base
        else:
            anchor = max(base, float(self._applied_amp))   # 坑①：锚定当前实际振幅
        cap = round(base + AMP_ESCALATE_V, 2)          # 坑②：与微调带无关的硬顶
        if elapsed - float(self._stall_wait_s) < self._tau_est + self._period_est:
            self._amp_heartbeat(
                spec, mass, elapsed, cmd,
                f"救济冷却中（等 τ+T = {self._tau_est + self._period_est:.0f}s "
                f"看上一脚有没有踢开）")
            return
        tries = int(self._amp_kick_tries)
        kick_amp = round(min(cap, anchor + AMP_KICK_DELTA_V
                             + tries * AMP_KICK_ESCALATE_V), 2)
        if tries > 0 and kick_amp <= float(self._amp_kick_last_amp) + 0.015:
            self._amp_saturated = True
            self._amp_lock_reason = f"踢粉到顶 {cap:.2f}V 仍不出料"
            self._log(f"踢粉已经到顶 {cap:.2f}V（标定 {base:.2f}V "
                      f"+{AMP_ESCALATE_V:.2f}V）仍不出料"
                      + (f"（{detail}）" if detail else "")
                      + "——请检查料斗是否搭桥/缺料、推杆开度是否够。振幅不再往上加")
            return
        self._stall_wait_s = elapsed
        self._amp_kick_watch = None    # 新的一脚取代旧观察窗
        self._amp_kick = {"amp": kick_amp, "base": float(self._applied_amp),
                          "until": elapsed + AMP_KICK_HOLD_S, "fired": elapsed,
                          "m0": float(mass)}
        self._kick_count += 1
        self._amp_kick_tries = tries + 1
        self._amp_kick_last_amp = kick_amp
        self._amp_changes += 1
        self._amp_probe = None          # 踢脚是脉冲动作，不算微调增益样本
        self._prev_eval_rate = None     # 高幅瞬态会污染趋势闸
        mode = "卡粉救济" if spec.name == "slow" else "断料救济"
        self._set_amplitude(
            kick_amp, elapsed,
            f"{STAGE_TEXT[spec.name]}{mode}：踢一脚 → 瞬间抬到 {kick_amp:.2f}V "
            f"持续 {AMP_KICK_HOLD_S:.1f}s（第 {tries + 1} 脚，顶 {cap:.2f}V）"
            + (f"，{detail}" if detail else ""))


    def _trim_wait_settled(self, note: str) -> tuple[float, float]:
        """等天平落定：约 1s 窗口内极差 ≤TRIM_STABLE_RANGE_MG，或超时。

        超时也用当前读数继续——缓补每发只有 10mg 量级，卡死比落定差 1mg 更糟。
        期间持续 _record，页面和 run 文件都能看到缓补段的采样。
        """
        window: list[float] = []
        mass = 0.0
        elapsed = time.monotonic() - self._t0
        deadline = time.monotonic() + TRIM_SETTLE_TIMEOUT_S
        while not self._stop_event.is_set() and time.monotonic() < deadline:
            raw = self._read_balance_raw()
            mass = float(raw) - self._baseline
            elapsed = time.monotonic() - self._t0
            window.append(mass)
            if len(window) > 10:
                window.pop(0)
            self._record(elapsed, mass, 0.0, "trim")
            if len(window) >= 8 and max(window) - min(window) <= TRIM_STABLE_RANGE_MG:
                return mass, elapsed
            self._stop_event.wait(0.1)
        return mass, elapsed

    def _trim_pulse_loop(self, mass: float, elapsed: float, target: int) -> str:
        """缓补主循环：开振 pulse_s 秒 → 关振 → 等落定称增重 → 自适应下一发时长。

        为什么用"短促开振"而不是压低振幅连续出：剩余 20~40mg 时慢加的连续流
        落点误差有 ±几 mg（惯性余料 + 判停滞后），对 ±10mg 的验收太毛；
        脉冲把"出料"和"称重"在时间上完全分开，每一发落多少天平说了算。

        时长自适应（目标单发 ≈10mg）：增重 <5mg → 时长 ×1.5（上限 2.5s）；
        >14mg → ×0.65（下限 0.15s）。一发完全没出粉 → 振幅 +0.5V 试探
        （2026-10-22 11:45 实测：2V 在杆 680 一粒不出、4V 出 28mg/s——标定振幅
        低于起振门槛时脉冲永远发不动），同时下一发缩到最短防止突然出大坨；
        连续 4 发仍一粒不出就停手并提示查料斗。
        """
        self._log(f"进入缓补：{mass:.1f}/{target} mg（剩 {target - mass:.0f}mg）——"
                  f"停连续振动，改为 ≈{self._trim_pulse_mg:.0f}mg/发 的脉冲补料"
                  f"（振幅从 {self._applied_amp:.2f}V 起，发不出才逐发上抬试探）")
        if self._vib_running:
            self.hub.sig_output(False)
            self._vib_running = False
        trim_spec = TrimSpec(stir_rpm=self._applied_stir, rod_ff=int(self._applied_window),
                             amplitude_vpp=float(self._applied_amp))
        self._start_stage_log(trim_spec, mass, elapsed)
        # 先把慢加的尾料等干净：尾料落定算进缓补的账，第一发脉冲才量得准。
        # 尾料增重必须显式记一条——慢加冲快时尾料可能有几十 mg（17:27 那次
        # 3V 慢加尾料 67mg），页面只看到"0 发脉冲质量却在涨"就没法对账。
        entry_mass = mass
        mass, elapsed = self._trim_wait_settled("进缓补，先等慢加尾料落定")
        tail_gain = mass - entry_mass
        if tail_gain >= 1.0:
            self._log(f"缓补入口：慢加尾料落定 +{tail_gain:.0f}mg"
                      f"（当前 {mass:.1f}/{target}，剩 {max(0.0, target - mass):.0f}mg"
                      f"{'——尾料已够，不再发脉冲' if target - mass <= 0 else ''}）")
        duration = TRIM_PULSE_START_S
        count = 0
        no_gain = 0
        while not self._stop_event.is_set():
            remaining = target - mass
            if remaining <= 0.0:
                break
            if count >= TRIM_PULSE_MAX_N:
                self._log(f"缓补脉冲已达 {TRIM_PULSE_MAX_N} 发上限（累计 "
                          f"{mass:.1f}/{target} mg）——停手保护，不再发")
                break
            before = mass
            # 剩余不足一发时按时长等比例缩短，别为凑 3mg 再灌 7mg 进去
            pulse_s = duration
            if remaining < self._trim_pulse_mg:
                pulse_s = max(TRIM_PULSE_MIN_S, duration * remaining / self._trim_pulse_mg)
            fired = False
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
                    pulse_end = time.monotonic() + wait_s
                    while time.monotonic() < pulse_end:
                        if self._stop_event.wait(0.05):
                            break
                    self._vib_running = False
                except Exception as error:    # noqa: BLE001 - 设备不认就降级
                    self._log(f"猝发指令失败（{error}），本发及后续降级为软件定时脉冲")
                    self._vib_running = False
            if not fired:
                self.hub.sig_output(True)
                self._vib_running = True
                pulse_end = time.monotonic() + pulse_s
                while time.monotonic() < pulse_end:
                    if self._stop_event.wait(0.05):
                        break
                self.hub.sig_output(False)
                self._vib_running = False
            count += 1
            mass, elapsed = self._trim_wait_settled(f"第 {count} 发后等落定")
            gain = mass - before
            self._log(f"缓补第 {count} 发：振 {pulse_s:.2f}s → +{gain:.0f}mg，"
                      f"累计 {mass:.1f}/{target} mg（剩 {max(0.0, target - mass):.0f}mg）")
            self._record(elapsed, mass, gain / max(duration, 0.05), "trim")
            if gain < 0.5:
                no_gain += 1
                if no_gain >= TRIM_NO_GAIN_LIMIT:
                    self._log(f"缓补连续 {no_gain} 发没出粉（幅已抬到 "
                              f"{self._applied_amp:.2f}V、杆 {self._applied_window}）——停手，"
                              f"请检查料斗是否搭桥/缺料")
                    break
                # 一发都没出：大概率是振幅低于这种粉当前的起振门槛（11:45 实测
                # 2V 在杆 680 下一粒不出、4V 出 28mg/s）。抬 0.5V，同时把下一发
                # 缩到最短试探——抬完一旦出粉往往是大流量，长脉冲会直接冲爆。
                new_amp = min(AMP_MAX_VPP, float(self._applied_amp) + TRIM_PULSE_AMP_RAISE_V)
                if new_amp > float(self._applied_amp) + 0.01:
                    self._set_amplitude(new_amp, elapsed,
                                        f"缓补第 {count} 发没出粉，振幅上抬 {TRIM_PULSE_AMP_RAISE_V:.1f}V 试探")
                duration = TRIM_PULSE_MIN_S
            else:
                no_gain = 0
                if gain < 0.5 * self._trim_pulse_mg:
                    duration = min(TRIM_PULSE_MAX_S, duration * 1.5)
                elif gain > 1.4 * self._trim_pulse_mg:
                    duration = max(TRIM_PULSE_MIN_S, duration * 0.65)
        # 退猝发待命（WO0s+Wc0s）：这炉用过真脉冲的话设备还停在"手动触发模式"，
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
        if target - mass <= 0.0:
            return (f"缓补完成：{count} 发脉冲，最终 {mass:.1f} mg / 目标 {target} mg"
                    f"（超出 {mass - target:.1f}mg，验收 ±10mg）")
        return (f"缓补结束但未到目标：{count} 发脉冲，最终 {mass:.1f}/{target} mg"
                f"（差 {target - mass:.1f}mg）——结果按未完成处理")

    def _set_amplitude(self, amp: float, elapsed: float, why: str = "") -> None:
        """把振幅写进信号源（唯一入口，统一钳位 + 留痕 + 重置响应等待窗）。"""
        amp = _clamp_amp(amp)
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
        with self._lock:
            self._stage_log.append({
                "stage": spec.name,
                "stage_text": STAGE_TEXT[spec.name],
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
        if (verdict == "偏轻" and self._trim_on and ok and not estimated
                and 0.0 < target - achieved <= TRIM_RESCUE_MAX_MG
                and not getattr(self, "_trim_rescued", False)):
            self._trim_rescued = True
            slow_rpm = next((float(it["stir_rpm"]) for it in reversed(self._stage_log)
                             if it.get("stage") == "slow"), 0.0)
            if slow_rpm > 0.0:
                self._log(f"判停预测失误：实际 {achieved:.1f}/{target} mg，差 "
                          f"{target - achieved:.0f}mg 低于下验收线——尾料被高估了，"
                          f"转缓补脉冲补差（搅拌回 {slow_rpm:.0f} RPM）")
                try:
                    self.hub.stir_start(slow_rpm, acc=self.STIR_ACC, direction=0)
                    self._applied_stir = slow_rpm
                except Exception as error:      # noqa: BLE001
                    self._log(f"补差重启搅拌失败（{error}），脉冲补料照常进行")
                mass_now, elapsed_now = self._trim_wait_settled("补差前等天平稳")
                self._close_stage_log(mass_now, elapsed_now)   # 幂等，关掉可能开着的段
                try:
                    outcome = self._trim_pulse_loop(mass_now, elapsed_now, target)
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


        # 前馈自修正：把这一炉"实测速率 / 标定预测速率"的比值回写粉体表，下一炉开度更准。
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
                              f"（本炉实测/标定 = {ff_update['sample']:.3f}）")
            except Exception as error:      # noqa: BLE001 - 写回失败不影响本炉结果
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
            except Exception as error:      # noqa: BLE001 - 写回失败不影响本炉结果
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

    def _safe_stop_vibration(self) -> None:
        try:
            # 退猝发待命（WO0s+Wc0s）再关输出：这炉用过缓补真脉冲时设备停在
            # "手动触发模式"，只发 Wc0s 退不出触发模式，下次加粉设了方波也不震。
            try:
                self.hub.sig.burst_off()
            except Exception:                 # noqa: BLE001 - 没进过猝发模式会报错，忽略
                pass
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
        绝不因为"读不到"就把结果丢掉——操作员需要知道这一炉到底停在哪。
        """
        deadline = time.monotonic() + self.SETTLE_TIMEOUT_S
        history: list[int] = []
        while time.monotonic() < deadline:
            try:
                history.append(self._read_balance_raw())
            except Exception:                 # noqa: BLE001
                time.sleep(0.15)
                continue
            if len(history) >= 5:
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

    def _record(self, elapsed: float, mass: float, rate: float, stage: str) -> None:
        with self._lock:
            # 采样存全量（0.12s 一个），曲线另存 0.5s 抽稀版本给页面画图
            self._samples.append((round(elapsed, 2), round(mass, 1), round(rate, 2), stage))
            # 逐拍 CSV：每个采样一行，带振幅环当时的全部决策现场（见 _csv_tick）
            self._csv_tick(elapsed, mass, rate, stage)
            # 质量历史给振幅环做回归：只留最近 20s（最长窗口 10s + 响应等待 6s 够用），
            # 不清理的话 20 分钟的一炉会攒上万点，每轮回归都要遍历一遍。
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
                "amp_vpp": round(self._applied_amp, 2),
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
        if prev is None or float(mass) - prev < AMP_DROP_EVENT_MG:
            return
        now = float(elapsed)
        if self._last_drop_s is not None:
            gap = now - self._last_drop_s
            if AMP_PERIOD_INIT_S * 0.3 <= gap <= 60.0:
                self._period_est = round(
                    (1 - AMP_TAU_EMA) * self._period_est + AMP_TAU_EMA * gap, 2)
        self._last_drop_s = now
        # 门槛记忆：这个振幅落得下粉（若它比已记门槛低）
        amp = float(self._applied_amp)
        if self._amp_threshold_est is None or amp < self._amp_threshold_est - 0.01:
            self._amp_threshold_est = amp
        elif abs(amp - self._amp_threshold_est) <= 0.01:
            self._amp_threshold_est = amp     # 同一档反复确认，保持
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
        （2026-10-22 17:40 那炉就是例子：操作员 23s 手停，光靠 5s 心跳
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
                round(self._applied_amp, 2),
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
        # 先收逐拍 CSV：无论这炉怎么结束，文件都要完整关闭并把路径报出来
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
