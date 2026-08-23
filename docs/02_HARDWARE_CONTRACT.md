# 量产硬件契约

本章定义固件能够安全进入 `Operation enabled` 的最低硬件条件。它是软硬件接口契约，不是完整原理图。最终板卡必须经过原理图评审、PCB 电流回路/EMC 评审、热仿真和样机测试。

## 参考引脚分配

| 功能 | STM32G431CBU6 引脚 | 外设 / 方向 | 契约 |
|---|---|---|---|
| A/B/C 高侧 PWM | PA8 / PA9 / PA10 | TIM1_CH1/2/3 | 中心对齐，20 kHz |
| A/B/C 低侧 PWM | PB13 / PB14 / PB15 | TIM1_CH1N/2N/3N | 与高侧互补，硬件死区 |
| 硬件关断 | PA15 | TIM1_BKIN / AF9 | `nFAULT`/比较器汇总，低有效、异步关断；不得配置成普通 GPIO |
| 相 A 电流 | PA1 | ADC1_IN2 / COMP1_INP | 与 TIM1 CH4 同步采样 |
| 相 B 电流 | PA7 | ADC2_IN4 / COMP2_INP | 与 TIM1 CH4 同步采样 |
| 可选相 C 电流 | PB0 | ADC1_IN15 / COMP4_INP | 仅预留；当前双分流构建不采样，启用前需重新规划 ADC 队列 |
| 母线电压 | PA3 | ADC1_IN4 | 分压、RC 和过压钳位 |
| 温度 | PA5 | ADC | 功率级或电机 NTC |
| CAN RX / TX | PA11 / PA12 | FDCAN1 | 必须外接 CAN-FD 收发器 |
| AS5600 SCL / SDA | PB6 / PB7 | I2C1 | 仅实验配置，3.3 V 上拉 |
| MT6701 SCK / MISO | PC10 / PC11 | SPI3/SSI | 量产角度反馈 |
| MT6701 CS | PB4 | GPIO | 软件片选，空闲高 |
| 驱动 EN / RESET / SLEEP | PB10 / PB11 / PB12 | GPIO | 上电默认低/复位，软件放行后再使能 |
| CAN 收发器 standby | PB3 | GPIO | 上电默认待机 |
| SWD | PA13 / PA14 | SWDIO / SWCLK | 量产保留测试点 |
| HSE | PF0 / PF1 | OSC_IN / OSC_OUT | 量产默认要求 8 MHz 晶振及匹配负载网络 |
| 状态 LED | PC13 | GPIO | 不得作为唯一安全指示 |

引脚表是参考设计契约。若实际 PCB 不同，应同时更新原理图、`.ioc`、`Core/Inc/main.h`、BSP、生产测试和本文，不能只靠跳线临时适配。

## 功率级接口

量产 BSP 假设外部栅极驱动接受 6-PWM。应满足：

- TIM1 主输出使能 `MOE` 与独立 `GATE_ENABLE` 构成两级软件许可。
- 上电、复位、HardFault、看门狗复位和调试器断开时，所有栅极处于无转矩安全态。
- EN/RESET/SLEEP、CAN standby 与任何驱动高有效许可脚必须有板级上拉/下拉，使 MCU 尚未配置 GPIO、未知 silicon revision 停在错误处理或 MCU 失电时仍处于安全态；内部 GPIO pull 不能作为唯一防线。
- 死区从栅极驱动、MOSFET 开关时间、温度和器件离散性实测得出；当前 `34` 个 170 MHz 定时器 tick 约 200 ns，只是起始值。
- 当前最小高/低脉宽保留 `340` tick，即 2 µs；它同时为低侧导通、电流放大器建立和 ADC 孔径保留窗口。该值必须由最终 MOSFET、driver 和放大器波形验证。
- 驱动故障输出必须是确定电平，开漏信号有硬件上拉；故障应直接进入 BKIN，不经过普通 GPIO 中断才关 PWM。
- 比较器/DAC 或外部硬件过流阈值必须在 MCU 软件失控时仍能关断。
- DC-link 电容、电源回生吸收、TVS/制动方案必须覆盖 PP 减速和外力回拖的母线能量。

SimpleFOC Mini 常见三输入接口与上述 6-PWM 量产接口并不等价。实验配置必须使用单独的板级适配，禁止把 CHxN 误接到三输入驱动器，也禁止按量产 BSP 直接接线。

## 相电流测量

硬件设计推荐保留三分流能力，但当前固件构建固定同步采 PA1/A 相与 PA7/B 相，并用 `Ic=-(Ia+Ib)` 重构 C 相。配置宏 `BOARD_PROFILE_PRODUCTION_3SHUNT_MT6701` 的名称不能作为“三路已实现”的证据；PB0 只是预留，真正启用第三路前必须重做 ADC 队列、触发/可观测窗口、校准、WCET 和全扇区验证。每个实际采样通道至少定义并测量：

```text
Rshunt                 分流电阻、容差、温漂与额定脉冲功率
Gain                   电流放大器增益和共模范围
Voffset                零电流偏置，通常在 ADC 中点附近
ADC reference          实际参考电压、分辨率与校准误差
  A_per_count[A/B/C]     每个通道结合极性后的安培/ADC count
settling + aperture    PWM 边沿后的建立时间与 ADC 采样孔径
```

转换约定：

```text
Ia[A] = (ADC_A - offset_A) × BOARD_PHASE_A_CURRENT_A_PER_COUNT
Ib[A] = (ADC_B - offset_B) × BOARD_PHASE_B_CURRENT_A_PER_COUNT
Ic    = -(Ia + Ib)                      // 当前双分流构建、有效采样窗口内
```

要求：

1. TIM1_CC4 直接触发 ADC1 master，dual injected event 同步启动 ADC2 slave；ADC2 不另设外部触发。两路使用相同同步时钟 PCLK/4、采样时间和两个同通道 rank。固件按实际硅版本选择 JDR1 或 JDR2，不能把某个 rank 固定写成所有芯片的 dummy；两个 aperture 的时间偏差必须纳入误差预算。
2. TIM1 CH4 使用 PWM2，在每个中心对齐载波周期只产生一次上升沿触发。当前 `CCR4=4164`，即 `BOARD_PWM_PERIOD_TICKS - BOARD_PWM_MIN_PULSE_TICKS/4 - 1`，触发位于 ARR=4249 的 crest 前 85 timer ticks（约 `0.5 µs`），使两个 conversion aperture 分布在 crest 附近，而不是把触发沿本身称为最终采样点。必须在 Rev Z 选择 JDR1、Rev Y/X 选择 JDR2 的两条路径上，分别证明所用 aperture 位于保证的全低侧导通/模拟稳定区，并把死区、二极管恢复、运放建立、触发 jitter 与开关延迟纳入波形测试。改变采样时间、CCR4、最小脉宽或 PWM 极性后必须重新验证。
3. 上电保持驱动禁用。为覆盖 ES0431 §2.5.9，实时 ADC 启动后的第一整个 ADC1/2 dual injected sequence 无条件标为无效，不得送入 FOC 或 offset 累加；当前量产路径随后逐路采集 `2048` 个有效样本。每个原始值必须在闭区间 `[8, 4087]`，均值 offset 必须在 `[512, 3584]`，且 `max-min <= 64 count`。轨到轨、offset 窗口或峰峰噪声任一失败，都会把 current feedback 标记为 invalid，并沿 gate fault 路径禁止使能。
4. 注入已知正/负电流检查增益和符号；三相和应在误差预算内接近零。
5. ADC 饱和、偏置突变、窗口不可观测、两路不一致必须变成故障，不允许继续使用最后值。
6. A/B 通道必须分别使用 `BOARD_PHASE_A_CURRENT_A_PER_COUNT` 与 `BOARD_PHASE_B_CURRENT_A_PER_COUNT` 校准。`BOARD_PHASE_CURRENT_A_PER_COUNT` 只作为两路相同前端的兼容 fallback；量产记录不得因此假定两路增益完全一致。默认校准值无效，必须从实际原理图和逐板/批次测量赋值。

上述 `[8, 4087]`、`[512, 3584]` 和 `64 count` 是当前实现的工程保护初值，不是模拟前端的通用真理。量产放行前必须结合参考电压、运放输出摆幅、增益/offset 公差、开关噪声、温漂和测量系统能力重新审定；若调整，应同步修改需求、代码、测试边界和生产治具 limit，不能仅为“让校准通过”而放宽。

实验 `BOARD_PROFILE_LAB_SIMPLEFOC_AS5600` 明确跳过上述真实电流零偏校准，因为 SimpleFOC Mini 没有这条采样链。跳过只允许电压矢量实验，不能把 current feedback 标为有效，也不能开放真实电流环、转矩或 MIT。

STM32G431 器件勘误是本 ADC 契约的一部分，必须以 [ST ES0431](https://www.st.com/resource/en/errata_sheet/es0431-stm32g431xx441xx-device-errata-stmicroelectronics.pdf) 对应 die revision 审查。当前 BSP 读取 `DBGMCU_IDCODE.REV_ID`：`0x2001` Rev Z 受 §2.5.8 影响，连续两次采同一通道并保留 JDR1；`0x2002` Rev Y 受 §2.5.10 影响，把 JDR1 当 collision dummy、保留 JDR2；`0x2003` Rev X 不受这两项影响，但为保持统一序列而读 JDR2。GPIO 安全输出先于 revision switch 建立；任何未知 revision 随后进入 fail-closed 错误处理并拒绝启动，必须先完成新勘误审查才能加入白名单。§2.5.9 适用于所有这些 revision，因此每次实时 ADC 序列启动后第一整个 dual sequence 都必须丢弃。

针对 §2.5.11，当前 BSP 把 ADC1/2 配成 `ADC_DUALMODE_INJECSIMULT`（不是 alternate trigger），两路使用同一 PCLK/4、相同 rank/采样时间；TIM1_CC4 触发 ADC1 master，同一个 dual injected 事件同步启动 ADC2 slave，不存在第二次独立转换。`.ioc` 不给 slave 配独立 trigger；手写 BSP 的 HAL 配置仍把 ADC2 的 JEXTSEL/JEXTEN 字段写成 T1_CC4/rising，但在该 dual injected slave 模式下硬件忽略这些 slave trigger 位，`HAL_ADCEx_InjectedStart_IT()` 也只使能 slave 而不独立启动它。若以后退出 dual mode，这些位将不再是 don't-care，必须重新设计而不能沿用。本路径是勘误明确排除该并发精度限制的 dual-mode 路径，不再依赖 timer/ADC prescaler workaround 3。量产放行仍须在初始化后记录真实 `REV_ID`，实读 `ADC_CCR.DUAL=0b00101` 与两路 `JSQR.JL=1`，确认 ADC2 从设备先启动、ADC1 主设备后启动，并用目标板数据证明 JDR1/JDR2 都按预期更新、所选 rank 的同步相位及 DNL/增益/串扰满足预算。生成回退到 independent/alternate、未知 revision、`JSQR.JL=0` 或所选 JDR 不变化都必须阻止 gate enable。

## 电压与温度测量

- PA3/VBUS 是 ADC1 regular channel，PA1/A 相是同一 ADC1 的 injected channel。当前 `bsp_bus_voltage_read()` 只发布采样请求；ADC JEOS 路径确认 injected sequence 结束且 ADC idle 后才异步启动 regular，全程不调用会停止 injected group 的 `HAL_ADC_Stop()`。这避免 regular 与 injected 任意相位碰撞并覆盖 Rev Y §2.5.10 的相关路径；也不能把 Rev Y 的 JDR1-dummy 规则错误套到 Rev Z。
- 为覆盖 Rev Z §2.5.8，ADC1 regular SQR 在手写 BSP 中配置为 PA3/VBUS 同通道连续两个 rank。启动时清 EOC/EOS/OVR 并只在转换期间动态打开这三个 IRQ。若 rank 1 的 EOC 及时服务，软件先保存 rank 1，EOS 时读取并丢弃 rank 2；若 IRQ 延迟到 rank 2 完成，`ADC_OVR_DATA_PRESERVED` 让 DR 仍保留 rank 1，EOS/OVR 路径保存它并丢弃第二个结果。最终只发布 rank 1。
- 上述结构必须冻结并通过全相位/超期压力测试：实读 regular `SQR1.L=1`（sequence length=2），证明 regular 在下一个 TIM1_CC4 前完成、JEOS/ADSTART 状态机无竞态、rank 1 及时捕获和延迟 OVR-preserved 两条路径都发布相同的第一个结果，rank 2 永不成为 VBUS，injected 的 revision 选样也不被扰动。这里的预期 OVR-preserved 是取首结果机制的一部分；其他 stale、缺 rank、意外 OVR/flag 顺序必须安全失败。若 ADC 时钟、采样时间、PWM 频率或 rank 改变，必须重新计算并验证；不能仅因勘误注释存在就视为通过。
- `BOARD_BUS_VOLTAGE_V_PER_COUNT` 必须由分压比、ADC 参考和实测校准得到，默认 `0` 时禁止使能。
- 软件欠压/过压初值为 7.5 V / 16 V，仅适用于当前 12 V 电机资料和保守实验；硬件绝对阈值必须考虑电机、驱动、电容和电源中最小额定值。
- 过压需有硬件快速保护或能量吸收，不能只依赖 1 kHz 软件读取。
- 功率级和电机温度应分别测量；断路/短路检测不能把坏传感器解释为“低温”。
- 热降额曲线需要用最坏环境温度、堵转和通风状态验证，不能沿用示例常量。

当前代码只在 pin contract 中预留 PA5，尚未实现功率级/电机温度采集，也没有把 `POWER_STAGE_OVERTEMP`、`MOTOR_OVERTEMP` 接入 `collect_faults()`；传给安全层的 thermal torque limit 仍固定为 100%。因此这两个 fault bit 和降额接口只是框架，不能计作过温保护。量产板必须确定至少两路传感器/前端、ADC 资源、开短路诊断和独立硬件 TSD，并完成固件绑定及热测试。

如果未来增加 ADC 停止/disable、重新校准或重新启动路径，必须在硬件 restart 前重新 arm §2.5.9 的首 dual sequence 丢弃状态，并验证首序列既不进 FOC 也不进 offset。当前 `bsp_start_realtime()` 路径已执行该动作，但这不是自动覆盖所有未来 restart API 的全局机制。

同理，`STO_ACTIVE` 目前只有软件故障位，没有本引脚表中的双通道 STO 输入、诊断或安全认证实现。需要 STO 的产品必须使用独立、经风险分析的硬件安全链与反馈；不得把普通 `GATE_ENABLE` 或 CAN Disable operation 命名为 STO。

## 位置反馈

量产配置要求 MT6701 SSI：

- 3.3 V 电平兼容，磁钢/气隙满足器件要求。
- SPI3 参考配置：MSB first、时钟空闲高、下降沿采样；上板后必须以逻辑分析仪和 CRC 统计确认。
- 每帧验证 CRC、磁场状态、超速位和本地时间戳。
- 角度旧于 2 ms（当前量产默认）视为 stale；阈值应小于失控距离预算。
- 量产路径使用 SPI3 RX/TX DMA1 Channel 1/2 的 20 kHz 流水线：每个 ADC 快环消费上一完整帧并启动下一帧，CRC/磁场/超速状态进入反馈。角度约滞后一 PWM 周期，由带时间戳 PLL 预测到电流采样时刻。必须实测片选、SSI 相位、CRC 测试向量、DMA 完成顺序、延迟、抖动和 ISR WCET；不得消费半更新帧。
- 当前 MT6701 DMA 记录 DWT 启动 cycle；busy 超过 `500 µs` 时 background 先在临界区发布 `POSITION_FEEDBACK` 故障并撤销 gate，随后才调用 `HAL_SPI_Abort()`、释放 CS、发布 bus-error 无效样本并允许流水线重启。必须在目标上验证 abort 最坏阻塞时间、晚到 IRQ、DMA/SPI 标志、缓冲一致性和重复超时；恢复不得清锁存故障或自动恢复 torque。
- `BOARD_ENCODER_DIRECTION` 只能为 `+1` 或 `-1`，并通过低压开环对齐确认。
- `BOARD_ELECTRICAL_ZERO_VALID` 默认是 `0`。只有完成相序、方向和多次零位标定且记录误差后才设为有效。

软件机械 OVERSPEED 已把 PLL 最大估计速度配置为 `450 rad/s`，trip 保持 `300 rad/s`，因而判据不再被同值钳位遮蔽。量产仍必须验证原始角增量/时间戳、PLL 饱和、正反向阈值和迟滞，把 trip 置于电机/机械安全包络内；在目标注入测试通过前，fault bit 8 的存在仍不能单独作为保护证据。

实验 AS5600 的默认 stale 阈值为 5 ms，但这不表示能在 5 ms 延迟下做高带宽控制。其 I²C 调用不得阻塞 20 kHz 电流 ISR；应在较低优先级采样并用时间戳提交。

## CAN-FD 物理层

MCU 的 PA11/PA12 不能直接接 CAN_H/CAN_L。量产板需要：

- 明确支持目标 2 Mbit/s 数据段的 CAN-FD 收发器。
- 可控 standby 输入和已定义的上电状态。
- 120 Ω 终端只安装在总线两端；分支长度、连接器和线束阻抗受控。
- 适当的 TVS、共模抗扰、地参考/隔离策略和 ESD 路径。
- 对最终节点数和线束进行示波器眼图/边沿、采样点与错误计数测试。
- 当前所有固件 profile 都启用 FDCAN 并强制 `BOARD_USE_8MHZ_HSE=1`；实物必须装有 8 MHz 晶振并完成启动裕量、温漂和时钟精度验证。CMake 对 `BRUSHLESS_USE_8MHZ_HSE=OFF` 直接报错，Lab 也没有 HSI 例外；没有 HSE 的板不能使用当前固件基线。

500 kbit/s / 2 Mbit/s 是参考时序，不是所有网络的保证。任何 PCLK、晶振、收发器或线束变化都要重新计算时序与容差。

## PCB 与供电最低要求

- 相电流 Kelvin 采样；功率地与模拟地按器件建议汇聚，避免栅极回流穿过 ADC 参考。
- 栅极环路、DC-link 高频环路面积最小；相节点远离编码器和 ADC 输入。
- MCU、编码器、栅极驱动分别有就地去耦；模拟参考和放大器电源有噪声预算。
- 编码器线缆、CAN、SWD 和相线之间保持间距并考虑屏蔽/共模路径。
- 电机插拔、反接、相间短路、地断开、编码器断线和 CAN 线短路都应有定义的故障反应。
- 看门狗、NRST、BOOT0 和调试接口有确定电平；量产测试可以验证 BKIN 在 MCU 停止执行时仍关断。

## 量产使能门槛

只有以下条件同时满足，PDS 才能允许 `Operation enabled`：

- 板型/固件配置与实物一致。
- 当前采样的偏置、增益、极性和采样窗口有效。
- 母线电压、温度、驱动故障和 BKIN 链路有效。
- 编码器 CRC/状态/新鲜度有效，方向和电角度零位已标定。
- 命令来源已鉴权到系统设计要求，序号和本地看门狗有效。
- 所有锁存故障已消失，并在功率级物理关闭时收到新的 Fault reset 上升沿。
- 参数集合具有版本、CRC、板号和校准记录，且通过当前固件版本的兼容性检查。

不得提供“缺电流反馈时自动改用电压 MIT”的后门。实验板如果需要电压矢量调试，应使用单独命名、限压、限速、限时的调试模式，并且在发行构建中关闭。
