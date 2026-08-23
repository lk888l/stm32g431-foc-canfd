# STM32G431 无刷电机 FOC 驱动工程

这是面向 STM32G431CBU6 的无刷电机控制固件工程基线。工程包含无动态内存的 FOC 数学模块、限幅与抗积分饱和 PI、角度 PLL、PV/PP 轨迹、MIT 阻抗控制计算、CiA 402 PDS/模式管理、厂商 CAN-FD 快速帧编解码、安全管理，以及 STM32G4 HAL 外设参考配置。

> **当前状态不是“接线后即可量产”的成品，也没有任何 CiA、功能安全或 EMC 认证。** 随附的 SimpleFOC Mini 原理图没有相电流采样链路，因此不能在该板上实现真实电流环、真实转矩控制或真实 MIT 控制。默认量产配置会在电流标定系数、电角度零位等硬件参数缺失时保持功率级禁用；不允许用 `Uq` 电压冒充转矩。

## 两种硬件配置

| 配置 | 用途 | 位置传感器 | 相电流反馈 | PV / PP | MIT |
|---|---|---|---|---|---|
| `BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701` | 量产硬件参考，默认 | MT6701 SSI，14 bit | 当前固件同步采 A/B 两相并重构 C 相 | 完成硬件标定后允许 | 只有电流反馈、硬件保护和标定全部有效才允许 |
| `BOARD_PROFILE_LAB_SIMPLEFOC_AS5600` | 低速实验与方向验证 | AS5600 I²C，12 bit | SimpleFOC Mini 上不存在 | 仅限保守带宽实验 | **强制禁用真实 MIT** |

量产板必须至少提供：低侧或在线两/三分流电阻及放大器、TIM1 同步 ADC 采样、独立模拟比较器到 TIM1 BKIN 的硬件过流关断、MT6701 SSI、母线电压与温度采样、可读取的栅极驱动故障、CAN-FD 收发器、正确的终端与时钟设计。详见 [量产硬件契约](docs/02_HARDWARE_CONTRACT.md)。

配置宏名称中的 `3SHUNT` 是现有接口名，不代表当前固件已采三路：本构建只同步采 PA1/A 相和 PA7/B 相，按 `Ic=-(Ia+Ib)` 重构第三相；PB0 只预留可选第三分流。若要真正三路采样，必须重新设计 ADC 队列、触发/采样窗口、校准与测试，不能只接上 PB0。

## 工程能力与边界

| 项目 | 仓库提供 | 放行前仍需完成 |
|---|---|---|
| FOC 算法 | Clarke/Park、两轴事务式 PI、dq 电压圆限幅、SVPWM、最小脉宽，以及按调制器最终 applied voltage 做双轴 tracking 反算抗饱和 | 在目标功率板上校验电流极性、采样窗口、死区、持续饱和/恢复、ADC 时序与 WCET |
| 运动模式 | 独立 mode-instance epoch、PV 加减速限制与新目标锁存、PP 梯形/三角轨迹及 bit 4 目标快照、MIT 转矩到 `Iq` 映射 | 整机惯量/负载下整定；验证模式同值重写/离开重入、丢帧/乱序/重复目标、机械限位与热模型 |
| 驱动状态 | CiA 402 风格 PDS、Controlword/Statusword、门极反馈联锁，并已接入 CANopenNode OD adapter | 目标主站实机互操作、故障注入和适用的一致性测试 |
| CAN | STM32 FDCAN 500 kbit/s 仲裁段、2 Mbit/s 数据段参考；厂商 MIT 快速帧含 CRC/序号/本地超时 | 收发器与布线时序验证；确定整网 ID；压力、错误注入及 bus-off 恢复测试 |
| CANopen | 已绑定 CANopenNode v4 经典 CANopen：NMT、Heartbeat、EMCY/history、SDO server、2 RPDO + 2 TPDO 和 CiA 402 OD adapter；固定 RPDO2 的 0x607A+0x60FF 经全量校验后批量提交 | 目标主站实机互操作、跨 RPDO 时序/丢帧错误注入、EDS/runtime 自动比对和适用的一致性测试；CiA 1301 仍需另接完整协议栈 |
| 安全 | 故障位、锁存、降额、Quick stop、立即关断和命令看门狗框架；BKIN/反馈/deadline/通信路径已绑定 | 过温传感器/热降额和真实 STO 尚未接入；还需 FMEA、硬件失效注入、独立过流链路、STO/急停系统和应用风险评估 |

`App/protocol/motor_vendor_canfd.*` 是厂商自定义 CAN-FD 快速数据通道，**不是 CiA 1301 CANopen FD**。`App/canopen` 使用 CANopenNode 发送 11-bit、0…8-byte 的经典 CAN 帧；仅仅共用 STM32 FDCAN 外设不会让它变成 CANopen FD。若产品要求对外声明 CiA 1301 兼容，需要接入经过授权并通过一致性验证的 CiA 1301 协议栈。详见 [通信契约](docs/05_COMMUNICATION.md)。

每次合法的 0x6060 写入都会推进独立的 32-bit mode-instance epoch，**包括写入与当前数值相同的模式**；不支持/格式非法，或 PDS 仍为 `Operation enabled` / 仍发布 `torque_enable=true` 时被拒绝的写入不会推进。后者也覆盖仍在带转矩受控停机的 Quick-stop/Fault-reaction 阶段；只有已撤销转矩且不再处于 Operation enabled 才允许切换。1 kHz 管理器发现模式数值或实例 epoch 任一变化都会清除 PP/PV 活动目标。PP bit 4 上升沿会冻结此前已提交的 0x607A 值/目标 epoch、bit 5/6、模式及 mode instance；PV 的 0x60FF（包括固定 RPDO2 中的速度成员）也绑定写入时实例，只有与 active instance 一致的命令才可执行。`mode→target→edge` 全部发生在首次 1 ms supervisor 前仍是合法新事务；反之，同 tick 切离再重入、命令后同值重写 0x6060 或实例不匹配都不能重放旧 PP/PV。

PP 的 `New set-point` 也不是“bit 4 和目标大约同时到达”即可：晚到目标不能补救，已经消费或拒绝的目标 generation 不能重放。当前 CANopenNode 在一次 1 ms `CO_process_RPDO()` 中先处理 RPDO1、再处理 RPDO2；因此主站必须先让 RPDO2 至少跨过一次节点 1 ms 处理，再发送产生 bit 4 上升沿的 RPDO1。PV 没有新写入时保持最后一个合法目标，但仍受 NMT/Heartbeat 撤销链约束。完整主站顺序见 [通信契约](docs/05_COMMUNICATION.md#固定-pdo)。

## 目录

```text
App/bsp/       STM32G431 外设、功率级与编码器接口
App/canopen/   CANopenNode v4 经典 CAN 端口、固定 OD/PDO 与应用适配
App/control/   FOC 数学、PI、PLL、轨迹与 MIT 算法
App/drive/     CiA 402 PDS 与模式管理
App/protocol/  对象字典镜像、厂商 CAN-FD 帧
App/safety/    故障锁存、降额和命令看门狗
Core/          启动入口、HAL 中断与 MSP
Tests/unit/    可在主机执行的算法/状态机/协议测试
config/        协议配置模板；不是认证材料
docs/          硬件、控制、通信、调试与安全文档
```

## 文档阅读顺序

1. [资料审计与已知限制](docs/01_MATERIAL_AUDIT.md)
2. [量产硬件契约](docs/02_HARDWARE_CONTRACT.md)
3. [控制架构、频率与单位](docs/03_CONTROL_ARCHITECTURE.md)
4. [PV、PP 与 MIT 模式](docs/04_MOTION_MODES.md)
5. [CiA 402 与 CAN-FD 通信](docs/05_COMMUNICATION.md)
6. [参数辨识、校准与投运](docs/06_COMMISSIONING.md)
7. [安全案例与故障反应](docs/07_SAFETY_CASE.md)
8. [构建、烧录与 CubeMX](docs/08_CUBEMX_BUILD_FLASH.md)
9. [验证与量产放行清单](docs/09_VALIDATION_PLAN.md)

## 核心默认值

- 当前所有固件 profile（包括 Lab）都因启用 CAN-FD 而要求 PF0/PF1 上安装 8 MHz HSE，PLL 到 170 MHz；`BRUSHLESS_USE_8MHZ_HSE=OFF` 会在 CMake configure 阶段失败。PWM 与电流环 20 kHz，PendSV 监督/轨迹环 1 kHz。
- TIM2 提供 1 MHz、32 bit 本地微秒时基；TIM1 最小高/低脉宽窗口当前保留 340 tick（2 µs）。
- `bsp_start_realtime()` 后首次 main background 启动 5 ms 健康宽限；之后用 ISR 完成计数证明 20 kHz 快环持续前进（DWT 新鲜度 ≤500 µs）、1 kHz PendSV 持续前进（≤5 ms），并让 TIM2/DWT 在 2 ms 窗口内双向交叉监测。任一失败先关 gate、锁存 deadline，再停止刷新约 100 ms IWDG。
- CAN-FD：仲裁段 500 kbit/s，数据段 2 Mbit/s，BRS。
- 2804 电机初始建模值：7 极对、`Rs=2.55 Ω`、`Ld=Lq=0.86 mH`、`λ=0.0035 Wb`、`Kt≈0.03675 N·m/A`。
- 保守电流初值：连续 `0.5 A`、峰值 `1.0 A`；母线 nominal `12 V`，软件窗口 `7.5…16 V`。
- 量产电流链使用 A/B 独立 `A/count`；上电各取 `2048` 样本并检查原始值 `[8,4087]`、offset `[512,3584]`、峰峰值 `<=64 count`。任一路失败即禁止 gate；这些保护初值仍须按最终模拟前端和噪声统计审定。
- 上述电机值来自资料整理，只能作为辨识起点，不能替代实物测量和热测试。
- 量产构建必须设置实际板级 `A/count`、`V/count`、电角度零位和编码器方向。默认值故意无效，以阻止未标定使能。

量产 MT6701 路径采用 SPI3 DMA 20 kHz 流水线：每个 ADC 快环消费上一帧并启动下一帧，约一 PWM 周期延迟由带时间戳 PLL 预测。该架构仍须在实物上用逻辑分析仪/示波器确认 SSI 相位、CRC 测试向量、片选、延迟和抖动后才能放行。

近期安全修正已经落盘：MT6701 DMA busy 超过 `500 µs` 时先原子锁存位置故障/关 gate，再执行 HAL abort 并发布无效样本；ADC1/2 使用两个同通道 rank，按芯片 `REV_ID` 选择有效样本（Rev Z `0x2001` 用 JDR1，Rev Y `0x2002` 与 Rev X `0x2003` 用 JDR2，未知 revision 在 GPIO 安全态建立后拒绝启动），并为所有允许 revision 的 ES0431 §2.5.9 无条件丢弃实时启动后的第一整个 dual injected sequence；VBUS regular 只在 JEOS 后启动，非 alternate `ADC_DUALMODE_INJECSIMULT` 避开 §2.5.11 的双 ADC 并发精度限制；机械 OVERSPEED 的 PLL 上限为 `450 rad/s`、trip 为 `300 rad/s`；TIM2/DWT 交叉监测消除了单一 TIM2 停摆让旧命令永久“新鲜”的直接路径。这些都是待测设计措施，不是量产证据。上电必须实读确认 `REV_ID`、dual mode 与 `JSQR.JL=1`，并用示波器/已知注入证明首序列确实未进入 FOC/offset、所选 JDR rank 和采样窗口正确；否则不得把勘误规避视为有效。

Rev Z 的 VBUS regular §2.5.8 路径也已在源码中按同通道连续两个 regular rank 实现：只在 JEOS 后启动，动态处理 EOC/EOS/OVR，并用 `ADC_OVR_DATA_PRESERVED` 保留/发布 rank 1、丢弃 rank 2。这仍必须在目标板实读 regular SQR length=2，并覆盖 rank 1 及时中断与延迟到 rank 2 后的 OVR-preserved 两条路径。任何未来 ADC recalibration、disable/stop 后 restart 也必须重新置位“丢弃首个 dual sequence”，不能只依赖当前唯一的 `bsp_start_realtime()` 路径。

## 开始之前

先不要接电机。先完成源码单元测试，再在断开母线的条件下验证时钟、GPIO 安全态、编码器、ADC 偏置、故障输入和 CAN；随后使用限流电源、小电流和无负载电机按 [投运顺序](docs/06_COMMISSIONING.md) 逐级放行。任何跳步都会让电角度方向错误、采样极性错误或过流保护失效直接变成功率器件损坏风险。

本项目不把软件“Fault”当作人员安全功能。人员防护必须由系统级急停、隔离、机械防护、独立过流/过温链路和经风险评估的 STO 方案实现。
