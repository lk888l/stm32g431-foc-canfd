# CANopen 与 HexMeow OD 0x08 的兼容范围

协议依据：[HexMeow OD 0x08](https://docs.hexmeow.com/hex-motor/od-08/)。CANopenNode v4.1 负责经典 CANopen 通信，本项目适配驱动状态和运动对象。完整索引以 [生成的对象字典](object-dictionary.md) 和 [EDS](../config/rollercan.eds) 为准；修改 `tools/generate_od.py` 后统一生成 OD.c、OD.h、EDS 与索引表。

## 总线与服务

默认 Node-ID=1、11 位标准 ID、经典 CAN、1 Mbit/s。禁止 RTR、扩展 ID 和 FD 帧。启动发送 `0x700+ID` 的 boot-up（1 字节 0），进入 Pre-operational；生产者心跳默认 1000 ms。NMT Start 不等于驱动使能。

| 服务 | COB-ID / 实现 |
|---|---|
| NMT | 0x000，支持 Start、Stop、Pre-operational、Reset |
| SDO request / response | 0x600+ID / 0x580+ID；expedited 和 segmented server |
| SYNC | 默认 0x080，作为 consumer |
| EMCY | 默认 0x080+ID，驱动故障与通信错误；0x1003 保留 8 条历史 |
| Heartbeat | 0x700+ID；0x1016:01 可配置一个被监测节点 |
| PDO | 4 RPDO + 4 TPDO，可动态映射，支持同步与事件触发 |

所有 PDO 上电默认禁用，防止首次连接意外写入目标。通过 SDO 配置 COB-ID bit31 和映射，遵循“禁用 PDO → 映射数量清零 → 写映射项 → 写数量 → 使能 PDO”的顺序。节点号由 0x2001:01 设置，保存并重启后应用。NMT Reset Node 和 Reset Communication 当前均执行整机复位，重新进入安全启动流程。

## 数据类型与单位

**0x6064、0x606C、0x607A、0x60FF 是 Float32**，分别为实际位置、实际速度、目标位置、目标速度；单位 rev / rev/s。不能用通用 CiA 402 的整数位置模板直接读写。0x6071/0x6072/0x6077 为转矩千分比；1000 对应配置的电流上限与估算 Kt，不代表电机额定 1 Nm。

| 模式 0x6060 | 行为 |
|---|---|
| 0 | 不使能电机 |
| 1 | Profile position；bit4 上升沿采纳目标，bit6 为相对当前位置，bit12 应答；只接受当前目标，没有排队缓冲 |
| 3 | Profile velocity；加减速限制；bit8 Halt 减速到 0 |
| 4 | Profile torque；目标千分比映射到 Iq；Halt 把 Iq 目标置 0 |
| 5 | MIT；位置/速度误差乘 Kp/Kd 再加前馈转矩；Halt 把 Iq 目标置 0 |

位置模式 Halt 暂停轨迹并保持当前位置轨迹参考，清除后继续目标。位置模式的 bit5 排队/立即切换细分、插补缓存和标准同步位置模式不在此实现中。切换运行中的模式会返回 SDO 状态错误。

## 状态与超时

典型使能流程：Pre-operational 下选择模式/限值 → NMT Start → 控制字依次 `0x0006 → 0x0007 → 0x000F`，每步等待状态字。掩码 `status & 0x006F` 应依次为 `0x21 → 0x23 → 0x27`。未校准时保持 Switch-on disabled (`0x40`)。

`0x000B` 请求 Quick stop，按 0x6085 减速，速度低于阈值后进入 disabled；重新运行需要完整使能序列。控制字 bit1 为 0 时优先撤销电压许可。NMT Stop/Pre-operational 同样取消运行许可，不执行软减速。

故障由 0x603F、状态字 fault 位和 EMCY 报告，输出立即禁止。故障清除要求物理输入健康且控制字 bit7 **出现新的上升沿**，清除后仍处于 disabled，不自动恢复运动。0x605A/605B/605C/605E/6007 等选项对象仅提供实际实现的只读策略，不接受与固件行为不符的写入。

运行时，成功写入控制字或运动目标才能刷新 **100 ms 命令期限**；主站只发 NMT、心跳、SYNC 或 SDO 读取，不能维持旧运动命令。通信恢复也不会自动退出已经锁存的驱动故障。Heartbeat consumer 默认关闭，建议主站配置 0x1016:01，例如监测节点 5、100 ms：`0x00050064`，收到该节点第一次心跳后开始超时监测。

非法 PDO 字段（NaN、超限、运行中改模式等）没有 SDO abort 回复，因此固件锁存配置故障并禁止执行本批部分写入结果。修正数据后用 **SDO 写 0x6040=0x0080** 解除协议拒收锁，同时仍需满足驱动故障复位条件。RX/TX 队列溢出视为通信故障；锁存的队列溢出状态通过 NMT Reset 重新初始化。Bus-off 会尝试恢复 CAN 控制器，但电机仍保持故障态。

## 默认 PDO 映射

下表列出使能后的默认 ID，存储值仍带禁用位。多字节字段均为 little endian。

| PDO | ID | 内容 / 长度 |
|---|---|---|
| RPDO1 | 0x200+ID | 60FF:00 f32 + 6040:00 u16 + 6060:00 i8，7 字节 |
| RPDO2 | 0x300+ID | 607A:00 f32 + 6040:00 u16 + 6060:00 i8，7 字节 |
| RPDO3 | 0x400+ID | 6071:00 i16 + 6040:00 u16 + 6060:00 i8，5 字节 |
| RPDO4 | 0x500+ID | 2004:02 low32 + 2004:03 high32，8 字节 |
| TPDO1 | 0x180+ID | 603F fault u16 + 6041 status u16 + 6061 mode i8，5 字节 |
| TPDO2 | 0x280+ID | 6064 position f32 + 606C velocity f32，8 字节 |
| TPDO3 | 0x380+ID | 6077 torque i16 + 2202 bus mV i32，6 字节 |
| TPDO4 | 0x480+ID | 无默认映射 |

RPDO 默认异步类型 255；TPDO 默认类型 1，需要 SYNC。也可以改为 254/255 并配置 event timer。帧长度不足时 CANopenNode 不应用数据；PDO 超时可在 0x140x:05 配置。

## MIT

普通 MIT 使用 0x2003:01 position (rev)、:02 velocity (rev/s)、:03 torque (Nm)、:04 Kp u16、:05 Kd u16、:06 电流/转矩上限千分比。Kp、Kd 乘只读 :07 的 0.001；实际单位为 Nm/rev 与 Nm/(rev/s)，不同于使用 rad 的 MIT 实现。Kd 原始值限制 20000，最终 ≤20。

压缩模式 0x2004:01=1 时，64 位布局：

```text
63                     48 47          36 35          24 23          12 11           0
| position 16 bit        | velocity 12 | Kp 12         | Kd 12         | torque 12     |
```

各无符号值线性映射到 0x2004:04–0D 的 min/max。启用压缩模式之前必须存在且只能存在一个有效 RPDO，严格映射 `0x20040220, 0x20040320`，按低 32 位在前。禁止跨 PDO、重复映射和运行时 SDO 拼接两个半包；接收首个完整帧之前强制控制字为 0。更改映射前先关闭压缩模式。

改变范围只允许停止状态，且必须有限、min<max，并受位置 ±100000 rev、速度 ±40 rev/s、Kp 0–100、Kd 0–20、前馈 ±1 Nm 的上界检查；最终电流仍受硬件项目的 ≤0.5 A 限制。默认压缩范围和默认 PDO 与 Hex 文档不保证完全相同，以此项目 EDS 为准。

## 保存与厂商对象

- 0x2100:01 写 u32 `0x696C6163`（little-endian `cali`）请求电角校准。要求完成电流零点采样、无故障、Pre-operational、控制字 0、disabled。
- 0x2100:02 为 0 required、1 align、2 sweep、3 settle、4 complete、5 failed；:03/:04/:05 只读极对数/方向/电角偏移。
- 0x2100:06 电流上限 A、:07 Kt Nm/A；先写、再保存并重启后生效。
- 0x2100:09 MCU 内温，单位 0.1°C；:0A 历史最长 ADC ISR 周期数，160 cycle=1 µs。
- 0x2200/2203 为相电流/dq 电流 mA；0x2201 为由占空比推算的相电压 mV，不是电压传感器测量；关桥时报告 0。0x2202 为母线 mV。
- 0x1010:01 写 `0x65766173`（`save`）保存；0x1011:01 写 `0x64616F6C`（`load`）把默认参数保存为下一次启动配置。均要求非运行状态、控制字 0、已关桥；恢复默认会使下次启动需要重新校准。

未知对象、只读写入、长度错误、非法范围返回相应 SDO abort。未实现 HexMeow FD 长 PDO、模式 7/8、0x2204 驱动/电机温度、I²C 或 M5 原厂 CAN 私有命令。
