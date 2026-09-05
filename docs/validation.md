# 验证记录

日期：2026-09-05。目标为 STM32G431CBU6 / Unit-RollerCAN V1.0。

## 已完成的软件检查

| 检查 | 结果 |
|---|---|
| Windows 主机，MinGW GCC 16.1，C++23 | PASS，177333 次断言检查 |
| WSL2 Ubuntu 24.04，GCC 13.3，AddressSanitizer + UBSan | PASS，无报告 |
| Arm GNU 15.2.1 Debug (`-Og`) | PASS，自有 C++ `-Werror` |
| Arm GNU 15.2.1 Release (`-Os`) | PASS，自有 C++ `-Werror` |
| ELF 中断入口和禁止分配器检查 | PASS；SVC/PendSV/SysTick 由 FreeRTOS 提供 |
| CubeMX 6.17.0 / G4 V1.6.3 正式重生成 | PASS；当次快照的 55 个源码/构建/配置文件逐字节一致 |
| OD.c / OD.h / EDS / 对象索引的一致性 | PASS |
| 依赖逐文件 SHA-256 / 上游压缩包比对 | PASS；FreeRTOS 36 文件，CANopenNode 70 文件 |
| 官方最新 LTS 查询 | 202604.01-LTS → V11.3.1，commit `3a22924e0a9ddbbc8b0758881c33b3422a5cc20d` |
| Linux SocketCAN CLI | 参数解析/帮助可运行；未连接真实 CAN 接口 |
| GitHub 工作流 | 已配置；本地未替用户推送或执行云端 CI |

最终构建资源（`arm-none-eabi-size` 与 linker memory report）：

| 配置 | text | data | bss* | Flash 总占用 | RAM 保留 |
|---|---:|---:|---:|---:|---:|
| Debug | 65864 B | 2312 B | 17400 B | 68176 / 126976 B | 19712 / 32768 B |
| Release | 56900 B | 2312 B | 17400 B | 59212 / 126976 B | 19712 / 32768 B |

\* 包含 NOLOAD 栈预留区。RAM 数字包含静态 TCB、任务栈、协议缓冲和 4 KiB MSP 预算，不代表实测栈峰值。最后 4096 B Flash 不属于程序段，专供参数页。Debug/Release 均生成 ELF、HEX、BIN、MAP。

## 主机测试覆盖

- Clarke/Park 往返、SVPWM 边界、PI 饱和与恢复、角度跨圈和不可信跳变。
- 10000 圈后的细小角增量仍能更新位置，避免单个 float 累加冻结。
- 100000 项 SPSC 并发顺序检查与满/空边界。
- 模块重复注册、启动失败、逆序回滚、清理失败时保留依赖。
- 使能序列、快速停止、过流/过压、命令过期、故障 bit7 边沿，以及非 1 kHz 边界到达的故障复位。
- 独立锁轴 RL 电流模型中正向、反向与零转矩目标收敛；模型为测试用 R=2 Ω、L=1 mH，**不是本电机的辨识参数**。
- 理想角度跟随下正反方向电角校准成功；不动的转子校准失败且不使能。
- 编码器 CRC check vector、损坏数据、状态故障、断线值。
- 真正 CANopenNode 处理 boot-up、SDO 数据类型/权限/NaN/未知索引、动态 PDO、短帧、SYNC、心跳超时、有效运动命令期限。
- 非法 PDO 不执行部分有效命令；压缩 MIT 的同一 PDO 两字段要求、SDO 半包拒绝、量化端点。
- 同步 PDO 取消不丢弃软件队列中的非同步帧；硬件发送槽取消仍待台架验证。
- 参数 CRC 标准向量、每个 erase/double-word 边界模拟断电、元数据/数据/提交字损坏回退。
- 保存后恢复校准与参数，运行模式、控制字和运动目标回到安全默认值。

## 实机验证仍待执行

未连接或操作 ST-Link、CAN 适配器及电机，没有烧录、校准或实际转动。当前结果证明的是软件结构、算法/协议边界及可构建性。

| 项目 | 状态 / 应记录内容 |
|---|---|
| HSI/CAN 位时序与不同主站互通 | 未测；波形、总线负载、错误计数 |
| DRV8311 模式、CSA 正负与比例、PWM 相序 | 未测；示波器与电流表对照 |
| SSC 电气时序、CRC、编码器偏移可重复性 | 未测；逻辑分析与重复校准 |
| 20 kHz ISR 最坏时长和采样窗口 | 未测；DWT 最大值、示波器、IRQ 压力 |
| RTOS / 中断栈峰值 | 未测；栈高水位与 MSP 使用 |
| 电机闭环、PI 整定、可用转矩与带载温升 | 未测；逐级小电流调试 |
| 物理故障脚、按钮、断线、bus-off 恢复 | 未测；停止延迟及复位后输出状态 |
| Flash 真实掉电 / 欠压及 ECC 错误行为 | 未测；主机 RAM 日志模型不模拟 MCU Flash ECC/NMI，不能据此承诺任意掉电都自动恢复 |
| 长时间运行与 EMC/CANopen 一致性 | 未测；无认证结论 |

实机步骤见 [bringup.md](bringup.md)。每次改变时钟、ADC/PWM、RTOS port、优化选项或电机参数后，更新对应硬件测试记录。
