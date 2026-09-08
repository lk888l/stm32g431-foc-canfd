# 验证记录

更新：2026-09-08，STM32G431CBU6 / Unit-RollerCAN V1.0。实机条件与原始遥测见 [台架记录](bench/2026-09-08.md)，操作工具见 [ST-Link 测试](bench.md)。

## 软件检查

| 检查 | 结果 |
|---|---|
| Windows MinGW GCC 16.1，C++23 | PASS，323774 次断言 |
| WSL Ubuntu 24.04，GCC 13.3，ASan + UBSan | PASS，无报告 |
| Arm GNU 15.2.1，Debug / Release / Bench | PASS，自有 C++ `-Werror` |
| ELF 中断入口、禁止分配器及 Bench 隔离 | PASS；生产配置没有 SWD 命令邮箱 |
| CubeMX 6.17 / G4 V1.6.3 | CORDIC 初始化由生成器产生；重生成检查通过 |
| OD.c / OD.h / EDS / 对象索引一致性 | PASS |
| 本地依赖 SHA-256 | PASS；FreeRTOS 36 文件、CANopenNode 70 文件 |
| 官方最新 LTS 查询 | 202604.01-LTS，Kernel V11.3.1，commit `3a22924e0a9ddbbc8b0758881c33b3422a5cc20d` |

构建资源：

| 配置 | Flash 程序映像 | RAM 保留 |
|---|---:|---:|
| Debug | 71496 / 126976 B | 20616 / 32768 B |
| Release | 64804 / 126976 B | 20608 / 32768 B |
| Bench | 70768 / 126976 B | 20832 / 32768 B |

Flash 按 BIN/链接区计算，包含段间对齐；RAM 包含静态对象、TCB、任务栈与 4 KiB MSP 预留，不等于实测栈峰值。最后 4096 B Flash 独立用于参数页。完整 newlib 的空间开销换来了优化的内存拷贝。

主机测试覆盖坐标变换、限幅和极端浮点输入、PI 饱和恢复、RL 电流模型、整圈角度跟踪、SPSC 并发、模块回滚、状态机、校准/中止、有效命令类型、维护后的进度监督，以及真正 CANopenNode 的 SDO/PDO/NMT/SYNC/心跳处理。CRC 查表与逐位实现对照全部 65536 个角度字；Flash 日志模型覆盖写入边界中断及 CRC 损坏。

## 实机已执行

11.6 V、无负载、ST-Link SWD 1800 kHz。验证了电压换算和电流正负、三次多圈电角校准、正反速度至 ±5 rev/s、位置往返、正负小电流目标、快速停止、100 ms 命令租约、NMT Stop 中止校准，以及连续 10 次参数保存和复位恢复。台架测试运行真实功率级，但 CANopen 帧通过 SWD 注入。

最终 Bench 回归测得 ADC 控制回调最大 6929 cycles（43.306 µs），低于原有 7200 cycles 阈值；周期约 8000 cycles。这个数字是有限场景下的观测最大值，不是包含 HAL 入口/出口的完整 IRQ 最坏时间证明。CAN 与监督任务的最小空闲栈分别为 478、227 个 32 位 word，包含保存路径。

已烧录并校验正常 Release，最后 4096 B 参数页逐字节不变，恢复 7 极对、direction=-1、电角偏移 0.4887648 rad。ADC 持续运行，上电 PB2 为低，panic/inhibit 均为 0。物理 CAN 启用后观测到 ACK 错误和发送 FIFO 满，随后进入 0x8130 通信故障并保持关桥；尚无有效对端通信验证。

## 仍待执行

- 物理 CAN 位时序、不同主站互通、总线压力、bus-off 和发送槽取消。
- 示波器验证 PWM/ADC 采样窗口、SSC 电气时序；外部仪表校验电流增益。
- nFAULT/按钮的受控触发与停止延迟；编码器物理断线注入。
- 带载、绕组/驱动温升、长时耐久、EMC 和 CANopen 一致性。
- Flash 真实掉电、欠压及 ECC/NMI 行为。主机内存模型不能证明真实 Flash 在任意掉电下均可恢复。

GitHub 工作流包含 Host 和三个 ARM 配置，尚未在本次会话中推送或执行云端 CI。改变时钟、控制参数、优化选项、RTOS 或 ADC/PWM 配置后，应重新记录对应实机结果。
