# 参数辨识、校准与投运

投运必须从“功率级永不使能”开始逐级增加能量。每一步都要保存原始波形、参数、固件提交、板号、电机号和通过/失败结论；未通过不得进入下一步。

## 所需设备

- 隔离且可调的直流电源，具有可设置的电流限制和快速关断。
- 示波器：至少四通道，差分电压探头、电流探头；逻辑分析仪用于 SSI/CAN。
- 合适带宽的万用表/LCR 表；已校准的分流或电流源用于 ADC 增益检查。
- CAN-FD 分析仪和第二节点；终端/线束与产品一致。
- 机械固定架、透明防护罩、急停、温度传感器；MIT 测试建议测功机/转矩传感器。
- ST-LINK/SWD；调试器停止 MCU 时也应由硬件链路关闭功率级。

人员不得站在旋转平面内。首次带电时拆除负载、降低母线和电流限幅，并确保物理急停能够切断驱动电源，而不是只发一条 CAN 命令。

## SimpleFOC Mini + AS5600 Lab 路径

当前三输入实验台架不具备相电流反馈，不能按下文量产电流环阶段放行。使用 `BOARD_PROFILE_LAB_SIMPLEFOC_AS5600` 时只连接 PA8/PA9/PA10 到 IN1/IN2/IN3，PB13/PB14/PB15 保持未配置；AS5600 使用 PB6/SCL、PB7/SDA 和 3.3 V 上拉；状态 LED 为 PC6。当前 Lab 固件也不初始化 CAN，因此未连接 PA11/PA12 和收发器不会产生通信故障。

Lab 投运由 ST-Link 一次性邮箱执行：先只读确认 AS5600 `EncoderValid`，再运行限时电角度对齐，最后依次做小电压、低速和小角度位置测试。每条运动命令都有电压上限和最长 10 s 的硬时限，完成或故障后自动回到中性占空比并撤销 gate。详细命令、状态字段和失败判据见 [SimpleFOC Mini + AS5600 的 ST-Link 投运](10_LAB_STLINK_COMMISSIONING.md)。该路径始终保持 `current_feedback_valid=false`；保留的电流采样/PI 接口只为以后更换有采样链的功率板使用。

## 阶段 0：文档与静态门槛

1. 确认实际电机是 2804，而不是资料目录内其他型号；记录极数、绕组连接和批次。
2. 将最终原理图逐项对照 [量产硬件契约](02_HARDWARE_CONTRACT.md)，完成引脚/极性/ERC/PCB 评审。
3. 冻结编译器、HAL/CMSIS/协议栈提交和固件版本；从干净 checkout 构建并保存 map/size/hash。
4. 主机单元测试和静态分析全绿，编译零 warning；在目标上打开栈水位和快环 WCET 测量。
5. 确认量产配置中未启用实验板的电压控制后门，且所有板级校准默认无效时 gate 不会使能。

## 阶段 1：无母线/无电机检查

只给 MCU 低压电源，不连接 DC bus 和电机：

1. 测量复位、启动、调试器暂停和 watchdog 复位期间 `GATE_ENABLE`,`RESET`,`SLEEP`,`MOE` 及 TIM1 相通道 `CCxE/CCxNE` 的安全状态；普通 Idle 允许为 CH4 采样触发保留 MOE，但所有物理相通道必须关闭。
2. 验证 8 MHz HSE 的启动裕量、170 MHz 系统时钟、TIM1 20 kHz 中心对齐、互补极性、约 200 ns 初始死区，以及 340 tick/2 µs 最小高低脉宽；此时只看 MCU/驱动输入。
3. 手动拉动 `nFAULT/BKIN`，确认不依赖 CPU 指令即可关闭 PWM，并锁存正确故障。
4. 验证 ADC 自校准；确认 TIM1 CH4 PWM2 每个中心对齐周期只有一个上升沿，`CCR4=4164` 且两个采样 aperture 位于载波 crest 附近。读取并记录真实 `REV_ID`；Rev Z 验证使用 JDR1，Rev Y/X 验证使用 JDR2，未知 revision 必须在 GPIO 安全态建立后拒绝启动。确认实时启动后的第一整个 dual sequence 仅被清除，未进入 FOC 或 offset 累加；再检查两个 ADC 的相对时序、放大器建立和所选采样孔径。
5. 验证 TIM2 是 1 MHz/32 bit 单调时基，跨回绕的超时测试正确；DWT 用于短时 cycle/deadline、ISR 完成进度和 TIM2 交叉监测。分别冻结 TIM2 CNT 与 DWT CYCCNT，确认另一时基在 2 ms 内检测；再测试两者同时异常的残余共同失效。
6. 验证 MT6701 24-bit 帧、CRC 测试向量、磁场/超速状态和角度单调性；用逻辑分析仪确认 SPI3 RX/TX DMA1 Ch1/Ch2 每个 20 kHz ADC tick 消费上一帧并启动下一帧，没有半帧/覆盖。测量约一 PWM 周期的延迟、抖动和 PLL 预测误差，并统计至少数分钟的 CRC/丢帧。
7. 验证 CAN-FD 仲裁/数据时序、ID 过滤、remote frame 拒绝、错误计数和 standby。
8. 确认 `bsp_start_realtime()` 后首次 main background 才启动 DWT 计时的 5 ms 健康宽限；分别停止 TIM1/ADC 触发和屏蔽 PendSV，验证完成计数在 500 µs/5 ms 内不前进时先关 gate、锁存 `CONTROL_DEADLINE`、随后停止刷新约 100 ms IWDG 并复位。

判定：任何门极毛刺、BKIN 不能异步关断、编码器无 CRC 稳定性或 ADC 触发不确定都必须停在此阶段。

ADC 测试必须验证而不是假定 ES0431 workaround：先将板上 `REV_ID` 与采购批次、芯片丝印和 ES0431 对应，覆盖 Rev Z `0x2001` 的 §2.5.8/JDR1 路径、Rev Y `0x2002` 的 §2.5.10/JDR2 路径以及 Rev X `0x2003` 的 JDR2 路径；未知 revision 必须证明停在功率安全态。针对所有允许 revision 的 §2.5.9，捕获实时启动后的前两个 JEOS，证明第一个完整 dual sequence 不增加 FOC/offset 有效样本计数，第二个才进入正常路径。覆盖 VBUS 请求相对 TIM1_CC4 的全部相位，确认实际 ADSTART 只在 JEOS 后/ADC idle 时发生、regular 在下一个 injected trigger 前完成，且两个 JDR 都会更新、固件只消费 revision 对应的 rank、无 channel-0 零返回。上电实读并记录 `ADC_CCR.DUAL=0b00101`、ADC1/2 的 `JSQR.JL=1`，确认 ADC2 从设备先于 ADC1 主设备启动；用示波器/已知注入证明两个 aperture 围绕 crest、所选 rank 位于稳定低侧窗口。再做长时间 JEOS/JEOC/OVR、raw 边界、DNL/增益/串扰和电流噪声统计。dual mode 是 §2.5.11 的明确排除路径，但在各允许 revision 的目标板数据完成前仍不能签字。

对 Rev Z 还要单独验证 VBUS regular 双 rank/取首结果：实读 `SQR1.L=1`，在受控输入阶跃/斜坡下分别让 rank 1 EOC 及时服务和人为延迟到 rank 2 完成，确认前者先保存 rank 1、后者经 `ADC_OVR_DATA_PRESERVED` 仍读出 rank 1，EOS 后发布值相同且 rank 2 被丢弃；覆盖 EOC/EOS/OVR 同时/分开发生、意外 flag 顺序、下一次 injected trigger 和 code-density/DNL。故障注入还要模拟 ADC disable/restart 和 recalibration/restart，确认每次都重新丢弃首个完整 dual sequence；当前代码只在 `bsp_start_realtime()` 路径置位，新增 restart 路径前必须补齐。

## 阶段 2：电流测量校准

功率级保持禁止，电机可断开。

### 零偏

- 当前量产实现对 A/B 每路采集 `2048` 个同步样本，计算均值、最小值和最大值；每个原始 ADC 值必须在闭区间 `[8, 4087]`，均值必须在 `[512, 3584]`，且 `max-min <= 64 count`。
- 在最低/最高允许板温和不同母线（若放大器受母线影响）重复。
- 任一路轨到轨、offset 窗口或峰峰噪声超限，固件都把 current feedback 标为 invalid，并沿 gate fault 路径锁存 `CURRENT_FEEDBACK`；不得使用失败前的 offset 继续控制。
- 不要在存在相电流时自动重学 offset；这会把真实转矩吞进零点。

这些数值是当前工程初始联锁，不是最终生产 limit。应在最终模拟前端上统计多板、多温度、多母线和不同 PWM 噪声工况，并结合测量系统分析审定；更改 limit 必须有原始数据、风险分析和边界回归测试。

### 增益和极性

用已知双向电流或校准治具逐路测量：

```text
A_per_count[channel] = (I2 - I1) / (ADC2 - ADC1)
offset[channel]      = ADC at I=0
```

至少取零点和正/负各三点拟合，记录线性误差、噪声、温漂和 ADC 饱和余量。确认正 A/B 相电流在定义的正向开关状态下分别产生正确符号的 count 变化；当前双分流构建还要验证各 SVPWM 扇区的重构可观测性及 `Ia+Ib+Ic` 误差。PB0/C 相当前不采样；以后启用第三路时再独立验证其增益、极性和三相和。

只有通过后才分别设置非零 `BOARD_PHASE_A_CURRENT_A_PER_COUNT`、`BOARD_PHASE_B_CURRENT_A_PER_COUNT` 或写入校准存储。通用 `BOARD_PHASE_CURRENT_A_PER_COUNT` 只是兼容相同前端的 fallback，不能替代逐通道增益/极性检查。固件应校验参数 CRC、范围和板号，不接受全零/全 `0xFF` Flash。

`BOARD_PROFILE_LAB_SIMPLEFOC_AS5600` 会明确跳过这一步，因为实验板没有相电流采样链；其结果必须保持 current feedback invalid，仅可进入受限电压矢量实验，禁止进入真实 MIT/转矩投运阶段。

### 母线与温度

- 用校准万用表在至少五个母线点拟合 `V/count`；检查欠压、正常、过压阈值和迟滞。
- 对 NTC/温度 IC 做室温、热板和断线/短路检查；验证降额曲线单调且传感器坏不会显示低温。

当前固件尚未实现温度 ADC/换算/开短路检测，`POWER_STAGE_OVERTEMP`、`MOTOR_OVERTEMP` 不会由实物温度产生，thermal limit 也固定为满量程；因此本小节在完成硬件与代码接入前必定不能签字。若系统要求 STO，也必须在此之前完成独立硬件链、双通道诊断（按风险要求）和反馈测试，普通 gate enable 不算 STO。

## 阶段 3：编码器方向和电角度零位

用限流电源、低母线、极小静态电压矢量固定转子：

1. 手动沿定义正方向旋转，确认机械角和速度为正；不对则只修改 `BOARD_ENCODER_DIRECTION`，记录原因。
2. 确认 A/B/C 相序与软件相序一致。
3. 依次施加若干已知电角度的小幅 `Id` 对齐矢量，等待转子稳定，读取机械角，计算 `electrical_zero`。
4. 从不同初始位置、正反方向重复至少 10 次；保存均值、最大偏差和磁场状态。
5. 以很小 `Iq` 做正/负转矩脉冲，验证正 `Iq` 产生正机械加速度，`Id` 接近 0。

若转子跳动、方向不一致或零位重复性差，先检查极对数、磁钢偏心、SSI 时序、相序和绕组，不要靠增加电流“压住”。标定通过后才设置 `BOARD_ELECTRICAL_ZERO_VALID=1` 和零位值。

还要人工制造一次超过 PLL 最大步长（当前 10 ms）的编码器中断：下一帧有效绝对角应允许 PLL 以零速度重捕获，避免反馈永久失活；但已锁存的 `POSITION_FEEDBACK`、gate off 和 PDS Fault 必须保持，直到低速/安全条件和新的 Fault reset 上升沿全部满足。

屏蔽 DMA completion 或让 SPI 永久 busy：超过 500 µs 后必须先原子锁存 `POSITION_FEEDBACK`、撤销 gate，再进入 `HAL_SPI_Abort()`；确认 CS 释放、DMA/SPI 状态可重新启动、无晚到 callback/旧半帧被消费。恢复出的有效角度只允许诊断，故障/PDS 不自动清除；还要测试连续 timeout 和 HAL abort 失败/阻塞。

机械超速当前使用 300 rad/s trip、450 rad/s PLL 估计上限，判据已可达；仍必须用编码器注入和实物反拖从正/反方向跨越阈值与迟滞，并以钳位前角增量作参考，测量检出与 gate/Quick-stop 反应。只读取 `OVERSPEED` bit 不构成通过。

## 阶段 4：电机参数辨识

资料值只作初值，按实物更新：

### 电阻

转子固定、低电流 DC 注入，稳定后由 `R=V/I` 拟合多个电流点并扣除逆变器压降。若用线间测量，先确认星形/三角形连接再换算到控制模型的相电阻。避免测量时间过长导致铜温变化；记录绕组温度并换算到参考温度。

### 电感

在若干锁定转子角度施加小幅双极电压阶跃/高频注入，用：

```text
L = (V - R·I) / (dI/dt)
```

分别辨识 d/q 轴并统计随电流的饱和变化。采样带宽和死区补偿误差要纳入拟合，不用低质量 LCR 单点值直接替代运行点模型。

### 磁链与转矩常数

- 用外部装置匀速反拖，测线电压/电角速度估计反电动势和磁链；或在已验证电流环下拟合稳态电压模型。
- 用测功机测 `torque/Iq`，得到有效轴端 `Kt`、摩擦和齿槽影响。
- 比较磁链推算值与测功值；差异超过误差预算时不启用 MIT。

### 惯量和摩擦

在电流环已可靠后施加小转矩阶跃，以 `J·dω/dt = τ - Bω - τ_load` 拟合惯量/粘性摩擦。整机装配后的参数不同于裸电机，PP/速度环应使用最终负载数据。

## 阶段 5：电流环

1. 电机固定或无负载，`Iq` 限制从额定的 5–10% 开始，`Id*=0`。
2. 先验证 ADC 窗口、实际/命令符号和硬件过流，再做小方波 `Id`/`Iq` 响应。
3. 按 `Kp=L·ωc`,`Ki=R·ωc` 选保守带宽；加入实际计算/PWM/采样延迟模型。
4. 记录上升时间、超调、稳态误差、d/q 解耦、积分器、dq 电压饱和和 SVPWM scale。
5. 在母线上下限、正反电流、不同电角速度和温度重复；限幅后必须平滑恢复，无 wind-up。
6. 注入 BKIN、ADC 饱和和控制 deadline 故障，确认 PWM 关断时间。
7. 同时制造 SPI/FDCAN/I²C 中断负载，确认优先级 Break=0、ADC=1、SPI/DMA=2、FDCAN=3、I²C=4、PendSV=5 下，20 kHz 电流 ISR 和 1 kHz PendSV 监督器各自不超过放行预算。

首次测试不要同时打开速度 PI、位置环和未验证的前馈，否则无法定位不稳定来源。

## 阶段 6：速度环与 PV

- 电流环固定后，速度环带宽至少低 5–10 倍；从当前保守增益和低 `Iq` 限幅开始。
- 依次测试小正速度、零速、反向、阶跃和斜坡，确认反向先减速到零。
- 验证 0x6083/0x6084 分别控制增速与减速，内部速度限幅置 Statusword bit 11。
- 对 AS5600 实验配置进一步降低带宽，记录滤波设置和实际延迟；不能把高频速度噪声交给 PI 放大。
- 测试堵转、负载突变、再生制动、母线升压和热降额。

## 阶段 7：位置环与 PP

- 先只做小角度、低速度、低加速度绝对运动，再测试相对运动。
- 验证 bit 4 只响应上升沿、bit 12 acknowledge 握手、bit 5 immediate 和单个 pending 队列。
- 测试软限位边界、目标溢出、在途改目标、Halt、Quick stop 和机械硬限位。
- target reached 必须同时满足位置与速度容差；用编码器量化和负载回差确定容差，不能为了“容易到达”无限放宽。
- 检查停止时是否出现一个 count 极限环、积分 wind-up 或母线回生过压。

## 阶段 8：MIT

只有量产电流反馈配置可以进入本阶段：

1. 机械装置固定、防护罩关闭，命令频率与超时策略已通过网络错误注入。
2. `Kp=Kd=0`，极小正/负 `tau_ff`，用测功机确认 `tau/Iq/Kt`。
3. `Kp=0`，逐步加 `Kd`；检查速度估计延迟和噪声引起的电流抖动。
4. `q_des=q_actual`、`tau_ff=0`，从极小 `Kp` 增加；每步复测峰值电流和相位裕度。
5. 测试位置阶跃、外力扰动、饱和恢复、热/母线降额，以及 `Kp/Kd/current` 越界。
6. 强制命令重复、乱序、sequence gap、CRC 错、5 ms stale、20 ms disable、bus-off 和主站掉电。

任何用占空比或 `Uq` 代替测得 `Iq` 的结果都不能进入 MIT 验收记录。

## 阶段 9：通信和整机

- 用目标主站导入最终 EDS/XDD，完成 NMT、SDO、PDO、EMCY、Heartbeat 和状态机互操作。
- 压满计划总线负载，验证优先级、延迟分布、错误帧和反馈 sequence；不只测空闲总线平均延迟。
- 与其他节点同时上线，验证扩展 ID 无冲突，经典节点能容忍 CAN-FD 流量。
- 冷启动、热启动、brownout、瞬断、线缆插拔、终端缺失、CAN_H/L 短路、控制器重启均应进入定义状态。
- 装上最终负载后重新验证轨迹、温升、回生、机械止挡和急停距离。

## 校准记录最小字段

```text
format_version, board_serial, board_revision, motor_serial
firmware_commit, toolchain_version, timestamp, operator
current_offset[3], current_gain[3], current_polarity[3]
bus_voltage_gain, temperature_curve/version
pole_pairs, phase_order, encoder_direction, electrical_zero
Rs, Ld, Lq, flux, Kt, inertia, friction
current/speed/position/MIT gains and approved limits
CRC, approval_state
```

校准数据格式变更必须提升 `format_version`。固件遇到未知版本、CRC 错、板号不匹配或超范围参数时应保持 `Switch on disabled`，不能默默恢复示例默认值。
