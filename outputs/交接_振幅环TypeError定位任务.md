# 交接：粉末闭环加粉振幅环 TypeError 定位任务

> 写给接手的 AI/工程师。代码已提交：分支 `feature/v7-adaptive-amplitude`，commit `210a736`。

## 1. 系统背景（30 秒版）

粉末称重闭环加粉：振动料斗（QLS3600S 信号发生器出方波/脉冲）+ 搅拌电机（防搭桥）+ 推杆（开度，段内锁死）+ 天平（0.12s 采样）。
三阶段：快加/粗加（追速率）→ 慢加（低速率收尾）→ 缓补（设备猝发真脉冲补差）。
振幅是**唯一**闭环调节量。控制面板 `control_panel/app.py`（端口 8765），前端 `index.html`。

## 2. 当前阻塞 bug（优先解决）

**现象**：点「开始加粉」后约 2 秒（推杆 680 定位完成、起振后）抛：
```
TypeError: '>=' not supported between instances of 'float' and 'NoneType'
```
机构自动安全停（异常路径停振+停搅拌，这部分工作正常）。时间线（logs/control-2026-10-23.log 169–173 行）：
```
10:10:07 搅拌启动 150 RPM
10:10:09 推杆定位 → 680
10:10:09 推杆锁定 680 ...振幅 2.50V，自适应调幅...
10:10:11 搅拌停止（decel）        <- 异常触发安全停机
10:10:11 闭环加粉异常终止：'>=' ...
```
**注意**：异常发生在 `_prepare` 完成、主循环（`_run`）开始后 ≤2 秒内，即粗加段第一拍附近。

### 已排除（不要重复查）

- `dispense_runner.py::_step_amplitude` v7 决策层所有比较（AST 全扫描过）：`_amp_at_cap_since`/`_k_est`/`_last_drop_s`/`_amp_threshold_est`/`_v7_prev_*` 均有 None 保护；
- `_csv_tick`（有 try 包裹，落盘失败不影响控制线程）；
- `_rate_command_for`、`_maybe_taper`（比较双方均数值）；
- 已删旧常量（AMP_STALL_RATE_RATIO 等）无残留引用（AST 扫描"常量风格未定义名"= 0）；
- 启动日志文案的 NameError 已修（那是上一个 bug，已确认修复）。

### 异常消息的形态学线索

`float >= NoneType`：**左操作数是 float 实例，右操作数是 None**。
重点怀疑（按优先级）：
1. `core.py`（dispensing_algorithm/controller）：`controller.step()` 每拍调用，接收 `FeedbackObservation`（mass_mg/rate/predicted_mass_mg/acceleration 等）。查其中 `>= / <= / <` 右侧为 `settings.xxx` 或 observation 字段且该字段可能为 None 的（注意 `settings` 由 runner 构造传入，见 `_run` 中 FeedbackSettings(...) 构造处）；
2. `MotionEstimator.update()`（core.py）返回的 `motion.valid` 为 False 时，runner 是否仍有路径拿到 None 参与比较；
3. `_settle_gate` 分支（dispense_runner.py ~1655）：`abs(rate_now) < self._gate_trigger`——`rate_now` 的来源链；
4. 慢加动态目标 `_rate_command_for` 返回值进 `_step_amplitude` 后与 None 比较（返回值恒 float，基本可排除）。

### 复现方法

- 真机：页面 http://127.0.0.1:8765/ 点「开始加粉」（目标 200mg、润滑剂1、粗加速率 20）。**异常已被捕获并记入日志，traceback 会完整打印**（10:10 起 `_run` 的 except 已加 `traceback.format_exc()`）——跑一次直接看 `control_panel/logs/control-2026-10-23.log` 尾部即可拿到精确行号；
- 离线：`DispenseRunner(hub, log)` + 假 hub（需实现 `assert_idle_for_dispense() -> {"rod_position":...,"lift_positions":[...]}`、`bal_refresh() -> {"raw": int, "stable": bool}`、`sig.*`、`sig_output`、`stir_start/stop`、`rod_move/rod_wait`、`_sig_call`），`r.start(200, powder="润滑剂1", amplitude_vpp=2.5, stages=[{"rate_mg_s":20}], trim_enabled=True)`。注意：粉末 JSON 在 `server/data/powder_library/`（沙箱会话可能读不了，真机/用户权限下没问题）。

## 3. 代码地图（关键文件）

| 文件 | 内容 |
|---|---|
| `server/src/powder_sampling_control/control_panel/dispense_runner.py` | 加粉主线程。核心：`_step_amplitude`（v7 调幅决策，~1900–2180 行）、`_stall_relief`（踢一脚，~2177）、`_enter_stage`（进段重置，~1550）、`_run` 主循环（~1430–1490）、`_settle_and_finish`（收尾+判停补差兜底，~2445）、`_trim_pulse_loop`（缓补，~2241）、`_record`（逐拍采样+落料阶跃观测，~2620）、`_csv_tick`（逐拍 CSV，~2745）、v7 常量组 `AMP_*`（~130–210） |
| `server/src/powder_sampling_control/dispensing_algorithm/core.py` | 老闭环控制器（判停/预测尾料）：`_stop_projection`（~2038）、predictive_stop 判定（~1444）。**只管停车判定**，duty/推杆决议被 runner 忽略 |
| `control_panel/index.html` | 前端。「起始振幅」dispAmp 已默认空=按标定 |
| `control_panel/powder_feedforward.py` | 粉体档案/前馈表/自修正 |
| `control_panel/runs/*.json + *_samples.csv` | 每次加粉结果 + 逐拍 20 列现场（含每拍 reason） |

## 4. 交接前的其他已知事项

- 逐拍 CSV：`runs/dispense_<stamp>_<target>mg_<粉>_samples.csv`，每拍一行 20 列（含 `reason`——该拍振幅环为什么动/不动），复盘算法以它为准；
- 判停补差兜底（`_settle_and_finish`）：predictive_stop 被切换瞬态骗过（18:32 那次加粉偏轻 37mg），判停称重后偏轻且缺口 ≤100mg 且开缓补 → 自动转缓补脉冲补差再重判，每次加粉一次；
- 用户铁律（改代码必须遵守）：
  1. **不许在加粉/标定运行中改代码**；改完重启服务前必须确认 `/api/status` 里 dispense.running=False 且 calib.running=False；
  2. 不许动推杆调流量（推杆=标定点照抄或页面手填，段内锁死）；振幅是唯一调节量；
  3. 踢一脚（脉冲）只在"到带顶（标定+0.5V）且顶上 4s 速率<1mg/s"时允许，三段统一；
  4. 慢加不追速率，有出粉绝对不动振幅；
  5. 标定振幅是唯一基准工况，页面值只能显式覆盖；
  6. 用完猝发必须 `sig.burst_off()` 退待命（三处已兜底，新代码路径同改）；
  7. 推杆收尾不复位。

## 5. 环境速查

- 启动：双击 `control_panel/start_server.bat`（系统 Python312）或 `python app.py`（venv: `C:/Users/86182/.workbuddy/binaries/python/envs/powder_control`）；
- 日志：`control_panel/logs/control-YYYY-MM-DD.log`；
- 状态接口：`GET /api/status`（顶层键：lift/rod/stir/bal/dispense/calib/errors）；
- git：分支 `feature/v7-adaptive-amplitude`（基于 `feature/actuator-drivers-and-control-panel`），工作区另有未提交的 `data/calibration/*` 删除与粉体 JSON 变更——**不属于本次改动，不要一起提交**。
