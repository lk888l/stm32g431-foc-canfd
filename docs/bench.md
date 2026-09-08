# ST-Link 台架测试

`Bench` 运行真实采样、电流环、FreeRTOS 和 CANopenNode，通过 SWD 邮箱注入 CANopen 帧。它替换物理 CAN 传输，不能验证 CAN 引脚、收发器、终端、ACK 或 bus-off。仅在有人值守的台架上使用。

```powershell
cmake --preset Bench
cmake --build --preset Bench
openocd -f tools/rollercan-openocd.cfg -c "adapter serial YOUR_STLINK_SERIAL"
```

在另一个终端或调试器中烧录匹配的 `build/Bench/canopen_foc.elf`，校验并 reset run；保留最后两个参数页。换固件或暂停 CPU 前先关桥，禁止在带电转动时设置断点。工具从 ELF 解析符号，校验邮箱 ABI，不使用固定 SRAM 地址。

```powershell
python tools/stlink_bench.py inspect
python tools/stlink_bench.py fault-reset
python tools/stlink_bench.py --output build/calibration.json calibrate --save
python tools/stlink_bench.py --output build/forward.json velocity 0.5 --seconds 3
python tools/stlink_bench.py --output build/reverse.json velocity -0.5 --seconds 3
python tools/stlink_bench.py stop
```

`inspect` 不开启功率；校准和速度命令会转动电机。速度入口限制在 ±5 rev/s、1–30 s。正常结束或异常退出发送 NMT Stop 并回到 Pre-operational/controlword 0；这是关桥自由滑行。SDO 停止路径失败时，工具尝试通过 SWD 同时拉低 PB2 与 nSLEEP，此后需要复位才能唤醒驱动。

保留全部固件保护和 100 ms 运动命令租约。Windows 调度、USB 或调试器偶发停顿超过此期限时，故障关桥是预期行为；不要据此放宽生产固件的期限。校准本身是约 7 s 的有界流程，NMT Stop 会中止并使当前校准失效；显式保存过的旧校准可在复位后恢复。

快照由 CAN 任务应请求复制后保持不变，便于 SWD 在运行中读取；命令载荷完整写入后才提交序号。任务/ISR 仍通过 SPSC 传值，调试邮箱不作为普通 C++ 线程共享数据方案。

`isr_max` 为完整 ADC 回调的最大 DWT 周期数，除以 160 得到 µs。分段 `sample_max/control_max/apply_max` 不一定来自同一次中断，不能直接相加；分段统计跳过前 100 次启动回调，完整回调上限不跳过。主动 Flash 暂停不计入周期间隔极值，可由 `sampling_epoch` 看出采样重启。`cordic_max_error` 是上电 257 点与软件三角函数对照的最大绝对误差。

完成测试后烧录 Debug/Release 恢复物理 CAN 传输。测试结果见 [2026-09-08 记录](bench/2026-09-08.md)。
