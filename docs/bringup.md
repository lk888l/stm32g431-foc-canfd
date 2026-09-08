# 首次实机调试

已有一次 11.6 V 无负载实机调试记录，见 [测试结果](bench/2026-09-08.md)。新模块仍按以下步骤核对采样、校准和闭环。只有 ST-Link 时可使用 [SWD 台架工具](bench.md)。

## 1. 只观测，保持关桥

固定模块，轴无额外负载，使用可限流电源和正确 CAN 终端。以模块资料允许的供电开始，初次闭环建议约 6–8 V；板载 120 Ω 的实际开关/跳线位置以实物为准，总线只在两端终端。连接 SWD、地和电源，烧录 `build/Debug/canopen_foc.elf`（或 HEX）。避免整片擦除破坏需要保留的校准页。

检查上电 PB2 为低、PB1 唤醒驱动，CAN 上收到 Node 1 的 boot-up / 心跳。至少等待 100 ms，读 0x6041 应为 disabled；0x603F 应为 0。如果卡在启动，SWD 查看 `roller_debug.panic` 与 `roller_debug.reason`。

Linux SocketCAN 接口已配置好时：

```sh
python3 tools/rollercan_cli.py --interface can0 --node 1 inspect
python3 tools/rollercan_cli.py read 0x2200 1 i32
python3 tools/rollercan_cli.py read 0x2200 2 i32
python3 tools/rollercan_cli.py read 0x2200 3 i32
```

核对母线值与万用表一致。空载关桥电流应接近 0；手动转轴，位置变化平滑，单圈约 1 rev，无 0x7340 编码器错误。MCU 内温只用于粗略热保护，不能验证绕组温度。Windows 上可使用支持导入 EDS 的 CANopen 工具；附带 Python 工具需要 Linux SocketCAN，并不直接支持所有 Windows USB-CAN 驱动。

## 2. 时序与功率级验证

示波器查看 PA10/PA9/PA8 对应 A/B/C，PA5 的 SSC 时钟与 PA4 CS。PB2 低时电机应处于高阻；若需要断点或单步，先确认 PB2 已关。Debug 冻结 IWDG，停止 CPU 不会自动停止已经运行的 PWM。

检查 ADC ISR 的 DWT 最大值 0x2100:0A，必须低于 7200 cycles（45 µs），并留足裕量；确认约 20 kHz 的样本增长。观察 `roller_debug.can_stack_free`、`supervisor_stack_free`，单位是 32 位 word；长期负载下不能逼近 0。电流采样必须落在硬件资料所述低侧有效窗口，不能单凭波形频率正确判断采样有效。

## 3. 显式电角校准

以下写入会执行约 7 秒的 **1.2 V 电压矢量对齐和四电周扫描，轴会运动**。保持无负载，监视电流与相序。校准时用 NMT Stop、按钮或断电中止；控制字 0 是校准的启动条件，本身不会取消正在进行的校准。

```sh
python3 tools/rollercan_cli.py nmt preop
python3 tools/rollercan_cli.py write 0x6040 0 u16 0
python3 tools/rollercan_cli.py write 0x2100 1 u32 0x696c6163
python3 tools/rollercan_cli.py read 0x2100 2 u8
# 等待约 7 秒后再读，4 = complete，5 = failed
python3 tools/rollercan_cli.py read 0x2100 3 u16
python3 tools/rollercan_cli.py read 0x2100 4 i8
python3 tools/rollercan_cli.py read 0x2100 5 f32
```

校准期间 CAN 任务仍须健康；校准是单次本地流程，不要求主站反复写校准命令。不要在校准中切 NMT Start。重复校准、故障态或 ADC 零点尚未完成时请求会收到 SDO abort。

检查得到的极对数合理，重复校准的偏移和方向可复现。校准成功只建立电角关系，不代表电流符号、速度 PI 或机械控制已经验证。结果跨重启保留需在关桥状态保存：

```sh
python3 tools/rollercan_cli.py write 0x1010 1 u32 0x65766173
```

## 4. 小电流验证，逐级开启闭环

首次电流环试验可先将 0x2100:06 改为 0.10 A，保存后 NMT Reset，使新限制生效。重新 inspect 确认校准被正确恢复且仍未使能。先用转矩模式和很小的正负目标验证 Iq 正负、Id 接近 0、相电流不自激。再使用速度模式低至 ±0.1 rev/s，最后小幅位置目标。

使用主站循环写 RPDO，建议周期 10–20 ms，不能手工间歇输入目标维持运行。100 ms 没有有效运动写入会故障关桥。默认 RPDO1 需先通过 SDO `0x1400:01=0x00000201` 使能；随后发 7 字节速度帧：

```text
CAN ID 0x201: velocity Float32 LE [4] | controlword LE [2] | mode=3 [1]
速度 0.1 rev/s: CD CC CC 3D
控制字依次 06 00 → 07 00 → 0F 00；每步等待状态字变化。
完整速度运行帧: CD CC CC 3D 0F 00 03
```

选择好模式后 NMT Start（ID 0、data `01 01`），再按状态推进控制字。先以速度 0 建立使能再给小目标。停止优先发控制字 0 或 NMT Stop；验证 Quick stop 时使用 `0x000B`，检查确实减速后 disabled，禁止自动重启。

未调好的电流 PI、相序或电流符号可能造成正反馈，所以不要跳过小电流正负检查直接调速度/位置。当前 PI 参数经过本模块无负载整定，换电机或增加负载后仍需重新验证。软件转矩值为估计，不用它代替外部转矩/电流仪表。

## 5. 故障与耐久验收

逐项记录操作、波形、故障码、最大 ISR 时间、栈余量和复位后的状态：

| 场景 | 预期 |
|---|---|
| 停止主站运动帧，心跳继续 | 100 ms 期限到后通信故障、关桥 |
| 被监测主站心跳丢失 | 通信故障 / NMT 退出运行 |
| NMT Stop 或控制字 bit1 清除 | 取消使能，不继续旧目标 |
| 按钮或 nFAULT 低 | 关桥，锁存故障 |
| 编码器断线 / CRC 错误 | 0x7340，关桥 |
| 超压 / 欠压 / 相电流超限 | 对应故障，需输入恢复与 bit7 新上升沿 |
| 队列溢出 / CAN bus-off | 关桥，恢复总线不自动恢复转动 |
| 保存过程中断电 | 下次启动得到旧完整配置或新完整配置，不接受半条记录 |
| 故障后持续发旧 0x0F | 保持故障，不能自行恢复 |
| 上电 / 看门狗复位 | Pre-operational、控制字 0、功率关闭 |

故障复位：原因排除、关桥、电流降至低于 0.1 A 后，先 SDO 写 0x6040=0，再写 0x80；确认回到 disabled，之后才重新使能。若故障为 ADC 零点校准失败或队列溢出锁存，NMT Reset 完整重新初始化。

完成正反转、启动停止、带载与温升、长时间通信压力试验后，再提高允许电流和速度。实际采样方向、极对数、Kp/Ki、机械限位及可用转矩必须用台架结果确定并记录。
