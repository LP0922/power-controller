# 设备适配层（Device Adapters）

天平、运动控制器、振动给料、闸门、静电消除器、传感器与安全联锁的通信适配。

## 当前执行机构角色拓扑（2026-10-20 定案）

| 设备 | 角色 | 串口 | 协议 | 驱动 |
| --- | --- | --- | --- | --- |
| ZDT Emm42 V5.0（从站 0x01） | **旋转搅拌电机** | COM9 | 私有 0x6B @115200 | `zdt_emm42_private.py`（当前固件模式）/ `zdt_emm42.py`（Modbus 模式，需菜单切换） |
| MD30 ×2（从站 1、2） | **升降电机，必须同时运动** | COM10 | 标准 Modbus RTU @115200 | `md30_modbus_rtu.py` + `MD30DualLift`（广播 0x00 同步） |
| 因时 LA（协议 ID 3） | **推杆**（原窗口角色） | COM10 | LA 私有 55AA 帧 @115200 | `la_uart_protocol.py` |
| QLS5800 信号发生器 | 振动给料信号源（未接线） | 待接 | 仿 Modbus 无 CRC @9600 | `qls5800_signal_gen.py` |

⚠️ COM10 一条 485 总线挂了三种设备（Modbus×2 + LA 55AA），实测互不干扰；
上层同时访问必须经同一把锁串行化（见 `control_panel/app.py` 的 `ActuatorHub`）。

## 文件说明

- `at8811c_modbus_rtu.py`：AT8811C 天平，默认 `9600/8-N-1`。
- `la10_modbus_rtu.py`：LA10-D 执行器的 ESP32-S3 网关（Modbus），旧机专用，
  与 LA 电缸本体的 `la_uart_protocol.py` 是完全不同的协议，勿混用。
- `md30_modbus_rtu.py`：MD30 RS-485 伺服电缸（升降），标准 Modbus RTU
  `115200/8-N-1`。实测要点：实际行程 **20..980**；FC04 速度/负载为带符号
  int16；运动标志须块读（0x0002 起 7 个寄存器取第 6 个）；配置寄存器写入
  可能无回显，须读回验证。文件末尾 `MD30DualLift` 为双机同步层。
- `zdt_emm42_private.py`：ZDT 私有协议（0x6B 固定校验）——当前固件模式，
  全部帧经实测验证（含急停必须带 0x98 标志、减速停=速度写 0+acc）。
- `zdt_emm42.py`：ZDT 的 Modbus-RTU 模式驱动（需 OLED 菜单切 Checksum）。
- `la_uart_protocol.py`：因时 LA 电缸（推杆），55AA 帧 + 累加和校验；
  定位 0x21 带反馈 / 0x03 无反馈；目标位置 0..2000；状态 error 字节只看低 4 位。
- `qls5800_signal_gen.py`：QLS5800 寄存器适配器，默认 `9600`；
  实机的波特率与地址需现场确认。
- `feeder_actuator.py`：组合适配层，保留旧版振动与窗口控制的方法名。
  占空比到转速/幅度的映射是显式的标定输入，驱动不做任何猜测。

## 实时控制页面

`control_panel/`：浏览器实时控制三台执行机构（升降同步 / 推杆 / 搅拌）+ 全局急停。

```bash
cd server/src/powder_sampling_control/control_panel
python app.py                 # 默认 COM10/COM9，打开 http://127.0.0.1:8765/
python app.py --lift-port COM10 --stir-port COM9 --http-port 8765
```

各设备波特率与协议不同。COM10 总线为多设备共享，务必通过 `ActuatorHub`
的锁访问；不要在别处并行打开同一串口。
