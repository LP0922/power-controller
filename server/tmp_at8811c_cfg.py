"""临时：读 AT8811C 的分度值/满量程等只读配置，用来确定 4159 的单位。"""
import sys

sys.path.insert(0, "src")
from powder_sampling_control.device_adapters.at8811c_modbus_rtu import (  # noqa: E402
    AT8811CConfig, AT8811CModbusRTU,
)

READS = [
    ("0x0000(未知/可能是当前分度)", 0x0000, 1),
    ("0x0003(模块状态)", 0x0003, 1),
    ("0x000E(分度值)", 0x000E, 1),
    ("0x000F(满量程,32位)", 0x000F, 2),
    ("0x0017(AD频率)", 0x0017, 1),
    ("0x0018(滤波等级)", 0x0018, 1),
    ("0x0028(判稳范围)", 0x0028, 1),
    ("0x001A(波特率码)", 0x001A, 1),
    ("0x001B(协议格式码)", 0x001B, 1),
    ("0x0019(从站地址)", 0x0019, 1),
]

for label, reg, count in READS:
    # 每次新建实例：避免一次失败把熔断计数打满影响后续
    dev = AT8811CModbusRTU(AT8811CConfig(port="COM11", slave_address=1, baudrate=9600,
                                         timeout_s=0.5, retries=1))
    try:
        words = dev.read_holding_registers(reg, count)
        if count == 2:
            raw32 = int.from_bytes(b"".join(w.to_bytes(2, "big") for w in words),
                                   byteorder="big", signed=True)
            print(f"  {label:28} 读 {reg:#06x}×{count} → {words} | 32位值={raw32}")
        else:
            print(f"  {label:28} 读 {reg:#06x}×{count} → {words[0]}")
    except Exception as error:  # noqa: BLE001
        print(f"  {label:28} 失败: {error}")
    finally:
        dev.close()
