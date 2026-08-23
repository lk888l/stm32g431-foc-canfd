# 验证与量产放行清单

## 成熟度定义

本仓库代码可以作为工程基线，但“量产固件”是测试和证据状态，不是代码目录名称。建议用以下门槛管理：

| 等级 | 含义 | 允许用途 |
|---|---|---|
| L0 静态基线 | 可构建、代码评审、单元测试 | 不接功率 |
| L1 板级 bring-up | GPIO/BKIN/ADC/编码器/CAN 无母线验证 | 控制板实验 |
| L2 低能量 FOC | 电流采样/零位/电流环通过 | 防护环境、无负载 |
| L3 运动模式 | PV/PP、故障和通信通过 | 工程样机 |
| L4 MIT/HIL | 真电流 MIT、错误注入、整机负载通过 | 受控试制 |
| L5 量产候选 | 全环境、EMC、热、寿命、制造测试、独立评审完成 | 指定硬件/参数版本的小批量 |
| L6 产品放行 | 法规、系统安全和现场流程完成 | 仅限已批准产品配置 |

任何硬件、功率级、编码器、HAL/编译器、关键控制参数或协议栈变更都可能让成熟度回退，需要影响分析决定回归范围。

## 需求追踪矩阵

| 需求 | 单元测试 | 目标/HIL 证据 | 放行要点 |
|---|---|---|---|
| Clarke/Park 可逆和角度 wrap | `test_control` | 浮点参考向量、目标 FPU 对比 | 误差界、NaN 安全态 |
| dq 电压限幅/SVPWM | `test_control` + `test_motor_foc` | 三扇区、母线变化、最小脉宽示波器 | duty 范围、最终 applied-voltage scale、无窄脉冲 |
| 双轴 PI anti-windup/事务提交 | `test_control` + `test_motor_foc` | 双轴持续饱和、方向变化与解除饱和 | 两轴 tracking 回算、无积分发散/部分提交、时间步超限拒绝 |
| PLL wrap/时间戳 | `test_control` | 编码器扫速、丢帧/抖动 | 最大速度与 stale 反应 |
| PV/PP 轨迹与目标原子性 | `test_control` + `test_drive_protocol` | 正反向、mode-instance 同值重写/离开重入、RPDO 丢帧/乱序、在途改目标、软限位 | RPDO2 批量提交、实例与 target epoch freshness、加减速/容差/握手符合文档 |
| MIT 物理转矩到 `Iq` | `test_control` | 测功机、饱和、5/20 ms 超时 | 真实电流反馈、`Kt` 误差预算 |
| PDS/gate feedback 联锁 | `test_drive_protocol` | 实际 EN/FAULT/BKIN 注入 | 状态码、复位上升沿、卡住 gate |
| 对象范围/只读规则 | `test_drive_protocol` | 目标 CANopen 主站 SDO/PDO | abort code、0x2000 sequence-last 快照提交 |
| 厂商 CAN-FD 编解码 | `test_drive_protocol` | CAN 分析仪/总线负载/错误帧 | 字节 golden vectors、CRC/序号/wrap |
| 安全锁存/降额/watchdog | `test_drive_protocol` | HIL 故障注入 | 每个 bit 的关断时间和恢复路径 |

主机测试通过只是 L0 证据，不能证明 ADC 时序、功率级、实际转矩或总线物理层。

## 主机测试扩展

除现有固定用例外，应增加：

- 属性/随机测试：Park/inverse Park、SVPWM duty 永不越界、Q 格式饱和、CRC bit flip。
- 边界测试：`INT32_MIN/MAX`、NaN/Inf、零/负 timestep、32-bit 时间 wrap、16-bit sequence wrap。
- 状态机穷举：所有 Controlword mask、故障插入时刻、gate feedback 卡高/卡低、复位边沿。
- 轨迹黄金模型：三角/梯形、反向、一步跨目标、pending 满、相对目标溢出。
- 协议 fuzz：所有长度/DLC、保留位、mode、flags、范围、重复/旧/gap 序号。
- 经典 PDO 事务/时序模型：RPDO2 任一字段丢失或非法时两目标和两个 epoch 均不变；随机排列 RPDO1/RPDO2，覆盖同轮 pending、跨 1 ms 屏障、bit 4 持续高、晚到目标、已消费 generation、相对旧增量和 `UINT32_MAX→0`，任何旧目标都不得重放。
- 模式实例 freshness：合法 0x6060（含同值）推进独立 instance epoch；非法、Operation-enabled 或 `torque_enable=true`（含带转矩 Quick-stop/Fault-reaction）拒绝不推进，只有 torque=false 且不在 Operation enabled 才允许。数值或实例变化都清 active PP/PV。PP edge、PV velocity/RPDO2 velocity 只有实例匹配才执行；`mode→target→edge` 首次 supervisor 前合法，同 tick 切离/重入、命令后同值 0x6060、旧实例和 wrap 边界不重放。PV 相同数值的新 target epoch 仍被识别为更新，无新 target epoch 时只保持当前实例最后合法值，Heartbeat/NMT 撤销仍优先。
- zero-position 状态矩阵：gate on/off、PDS 运行/非运行/Quick stop、速度容差内外、bit 3 保持/释放/新上升沿；拒绝后即使条件变安全也不得延迟执行，必须释放再请求。
- 编译器矩阵：开发和冻结的 GNU Arm 版本、主机 GCC/Clang；优化 `-O0/-O2` 结果一致。
- 静态分析：至少 clang-tidy/cppcheck 或项目批准工具；高完整性项目按选定编码规范执行。

测试要固定随机种子并输出可重现的失败输入。对浮点使用有物理意义的绝对/相对误差，不以 bit-identical 代替算法正确性。

## 目标硬件测试

### 定时与资源

- 快环平均、P99.9 和最大 cycle；嵌套 CAN/encoder IRQ 下的 WCET。
- 20 kHz ADC IRQ 与 1 kHz PendSV 监督器分别测量 deadline；确认每 20 个 ADC 样本只 pend 一次监督器，优先级链 Break=0、ADC=1、SPI/DMA=2、FDCAN=3、I²C=4、PendSV=5 符合设计。
- 在 gate 已使能时分别令编码器、母线和量产电流反馈失效，示波器确认同一个 20 kHz 载波中断先写 50% 中性矢量并撤销实际 gate，而不是等待下一次 1 kHz 监督；故障位必须锁存。
- ISR jitter、ADC 两路偏差、sample-to-PWM update 延迟。
- TIM1 CH4 PWM2 每载波单上升沿、`CCR4=4164` 和 340 tick/2 µs 最小脉宽；在全部 SVPWM 扇区和 duty 极值上，分别证明 JDR1/JDR2 aperture 围绕 crest 且所选 rank 位于有效全低侧/模拟稳定窗口。
- 对 VBUS 请求相位做扫描/随机压力测试，证明 ADSTART 只紧随 JEOS/ADC idle，regular 在下一次 TIM1_CC4 前完成；按实物 `REV_ID` 验证 Rev Z (`0x2001`) §2.5.8/JDR1、Rev Y (`0x2002`) §2.5.10/JDR2、Rev X (`0x2003`) JDR2 策略，无 channel-0 零返回且快环无扰动。对所有允许 revision，验证 §2.5.9 要求的首个完整 dual sequence 被丢弃，第二个才进入 FOC/offset。注入未知 revision 或等效启动分支，确认 GPIO 已先到安全态且 gate 始终关闭。
- 单独验证 Rev Z VBUS regular 双 rank/取首结果：实读 `SQR1.L=1`，对已知输入比较 rank 1 EOC 及时服务与延迟到 rank 2 后 `ADC_OVR_DATA_PRESERVED` 两条路径，确认 EOS 最终发布相同 rank 1、rank 2 被丢弃。覆盖 EOC/EOS/OVR 组合、IRQ 延迟/抢占、下一次 injected 触发、慢斜坡/code-density/DNL 与欠/过压阈值。
- 注入 ADC stop/disable/recalibration 后 restart；每次都必须重新丢弃首个完整 dual sequence。若新增任何 restart API，静态审查其先重新 arm 丢弃状态，再允许触发。
- 验证 ES0431 §2.5.11 规避：确认 `ADC_CCR.DUAL=0b00101`，不是 independent/alternate；ADC2 从设备先启动、ADC1 主设备后启动。上电实读两路 `JSQR.JL=1`，用已知注入证明 JDR1/JDR2 每周期变化、revision 选择正确；测量两个 aperture、相对时序、DNL/增益/串扰，覆盖温度、ADC 时钟、采样时间、允许 die revision 与代码生成回归。目标证据完成前仍是投产阻项。
- TIM2 1 MHz 精度、32 bit 回绕与所有本地超时；冻结 TIM2 时 DWT 在 2 ms 内检出，冻结 DWT 时 TIM2 在 2 ms 内检出。覆盖两者同时异常/同主时钟域、HSE/温度/电压极限。
- 栈水位（所有 IRQ 嵌套）、Flash/RAM 余量、无 heap 使用证明。
- IWDG 只在快环/监督器完成计数分别在 DWT 的 ≤500 µs/≤5 ms 窗口持续前进，且 TIM2/DWT 交叉新鲜时刷新；分别停止 TIM1/ADC、屏蔽 PendSV、冻结各时基和卡死 background，验证先关 gate/锁存 `CONTROL_DEADLINE` 再停止喂狗。
- 实测 `bsp_start_realtime()` 后首次 main background 才起算的 5 ms grace、任务 stale 检出到 gate-off 的时间，以及约 100 ms IWDG 在 LSI 最小/最大频率下的复位时间；确认过早/过晚进入 background 都不会形成无期限盲区，复位后仍保持功率安全态。

### 功率与控制

- 每个 PWM 扇区的采样窗口、死区、bootstrap 最小脉宽和 gate 波形。
- A/B 独立 `A/count`、极性、offset、饱和与双分流三相和误差；覆盖温度、母线、PWM duty/扇区和板间差异，不能只验证通用 fallback 增益。
- 对当前校准联锁做精确边界测试：ADC 原始值 `7/8` 与 `4087/4088`、offset 均值 `511/512` 与 `3584/3585`、2048 样本 `max-min=64/65 count`；确认任一路失败都会令反馈 invalid、禁止 gate 并锁存 `CURRENT_FEEDBACK`。
- 用多板、多温度和开关噪声统计证明 `[8,4087]`、`[512,3584]`、`64 count` 的适用性；若调整门限，重新执行边界、噪声、开短路和误接受/误拒绝分析。
- Id/Iq 阶跃、交叉耦合、饱和恢复、正反转和基速附近。
- 构造 d/q 两轴各自未触发独立输出限幅、但合成向量超过母线电压圆的工况；核对 dq 圆限幅与 SVPWM 最终 `voltage_scale` 后的 applied d/q 都被 tracking 回算。持续饱和到稳态，再撤销/反转请求，记录两轴积分器边界、离开饱和时间、过冲和恢复时间。
- 对 d/q 任一轴 PI、inverse Park、dq limiter、SVPWM 和 tracking 注入 NaN/Inf/非法 timestep/范围失败，证明两轴状态均不部分提交、输出进入中性回退且 gate 故障路径生效；在这些分支与最坏嵌套 IRQ 下重新测 20 kHz WCET，不能只引用新增算法前的资源数据。
- PV/PP 全包络、堵转、负载冲击、机械止挡、回生过压和热稳态。
- MIT 测功曲线、刚度/阻尼稳定性、外力扰动和所有限幅组合。

### 故障注入

逐项断开/短接或仿真：

- 分流/放大器输出开路/短路、ADC 轨到轨、offset 跳变、峰峰噪声跨越 64 count，以及 A/B 增益或极性不一致。
- 编码器 CS/SCK/MISO、磁铁移除/偏心、CRC 错/旧时间戳。
- SPI3 RX/TX DMA1 Ch1/Ch2 的未完成、丢完成、双缓冲撕裂和抖动；确认每个 ADC tick 消费上一完整帧并启动下一帧，PLL 对一 PWM 周期延迟的预测误差受限。
- 制造短于/等于/长于 10 ms PLL 最大步长的编码器中断；长中断后的有效绝对角应以零速度重捕获，但 `POSITION_FEEDBACK` 和 gate-off 不得自动清除。覆盖高速下可能跨越 π 的不可判定场景。
- 单独审计机械 OVERSPEED：确认 300 rad/s trip 低于 450 rad/s PLL 上限；分别在钳位前输入阈值下/上、正/反方向、时间戳抖动、PLL 饱和和 wrap 条件验证迟滞、故障位和反应时间。
- 令 SPI/DMA busy 永久不清：验证 500 µs timeout 后先锁存位置故障/关 gate，再 HAL abort；检查 CS、DMA/SPI 标志、晚到 IRQ、缓冲一致性、重复 timeout 与 HAL 失败，无带 gate 或自动清故障恢复。
- nFAULT、BKIN、gate feedback 卡住、驱动过温/欠压。
- 母线欠压/过压、温度传感器断路/短路。
- 快环超时、主任务卡死、HardFault、时钟故障和 watchdog。
- CAN 断线、终端错误、错误帧、bus-off、重复/乱序/CRC/超范围命令。
- 用断开 CAN_H/CAN_L、线间短路及可控错误注入分别触发 bus-off：测量 ErrorStatus 到 gate-off/`CAN_BUS_OFF` 锁存时间，确认清 M_CAN `INIT` 后 PSR 按 ISO 规则退出 bus-off，但软件锁存仍保持。
- 在总线已恢复时依次尝试旧 Controlword、旧 MIT 帧、heartbeat/NMT Operational 和保持为高电平的 Fault reset，均不得重新使能；只有 gate 已关、速度低于容差且新的 Fault reset 上升沿才可清故障，随后还必须重新建立命令序号/heartbeat 并完成 PDS 使能。

每个测试记录“检测时间、转矩撤销时间、MOE/gate 实际关断时间、速度/距离、母线峰值、锁存位、复位条件”。不能只截图软件变量。

## CAN/CANopen 验证

### 厂商 CAN-FD

- 用独立语言实现 golden encoder/decoder，与 C 代码交叉验证 command/feedback 字节。
- 500 k/2 M 在最大/最小时钟误差、最长线束、全部节点下测错误计数和延迟。
- 按计划最大命令/反馈频率计算总线负载并留故障重发余量。
- 验证标准 Classic CAN 节点与 CAN-FD 帧共网行为；不兼容则物理分网。
- 长时间测试 sequence wrap、timestamp wrap 和反馈 echo。
- 验证 bus-off ISO 控制器恢复与 `CAN_BUS_OFF` 软件锁存的双层语义；总线恢复本身不清故障、不恢复旧命令或 torque。
- 并发压测经典 CANopen TPDO/EMCY/SDO 与厂商 48-byte feedback，验证共享 Tx 锁下 message RAM 无重入，记录两通道 lock-busy、FIFO-full、drop/retry 和端到端延迟；证明高优先级路径不会长期饿死另一通道。

### 经典 CANopen

- 使用目标主站与至少一种独立 CANopen 分析工具。
- NMT 全状态、boot-up、Heartbeat producer/consumer、EMCY/history、SDO expedited/segmented/错误和固定 PDO 周期；验证当前没有 SYNC、LSS、SDO client 或参数 storage，0x1016 每次上电必须重配。
- 注入普通驱动 manufacturer fault 时，确认 PDS 去 torque 但 NMT 仍保持受 heartbeat 监督的 Operational，诊断/新 Fault reset 可达；注入 bus-off 或 HB consumer failure 时，确认本地 NMT 按配置退出 Operational。
- 对 0x2000 验证 sub 1…6 单独写不续期 watchdog、sub 7 只有新 sequence 才提交；故意遗漏一个字段，确认提交沿用旧 shadow，并据此验证主站的“写齐后 sequence-last”事务规则。
- 固定 RPDO2 逐项注入 0x607A/0x60FF 缺失、错误长度、位置越界、速度越界、重复/异常对象顺序和延迟；读取应用镜像/epoch/拒绝计数，确认只有两个字段全量有效时才批量提交，任一失败都不留下半更新。
- 验证节点实际的 `CO_process_RPDO()` 顺序为 RPDO1→RPDO2：两帧同轮 pending 时 PP bit 4 快照旧 epoch 并拒绝，晚到 RPDO2 不补救；按“bit 4 低且 ack 清→RPDO2→至少一个 1 ms 处理屏障→RPDO1 bit 4 高”则只接受一次。再覆盖丢任一帧、重复高电平、帧乱序、已消费绝对目标、被拒绝的相对旧增量和 epoch wrap，均不得重放或二次累加。
- 对 0x6060 做精确实例测试：PP/PV/MIT 每次合法写（包括同值）mode-instance epoch `+1`；错误长度、不支持值、`Operation enabled`，以及仍发布 `torque_enable=true` 的 Quick-stop Active/Fault reaction 状态冲突时值/实例均不变。将受控停机推进到 torque=false 后重试才应接受。分别让模式数值变化与仅实例变化进入 1 kHz 管理器，并直接注入 torque-active 防御分支，确认禁止时不应用实例；允许后两类变化都清 PP active/pending/relative base、ack/completion 和 PV active target。
- 验证命令实例绑定：PP bit 4 request 快照 mode instance；独立 0x60FF 与固定 RPDO2 velocity 都记录提交时 instance。构造匹配/不匹配实例，确认只有与 active 相等的命令执行，不匹配 request/target 即使 target epoch 新也被消费后拒绝且不能重放。
- 覆盖首次 supervisor 前的 `mode→target→PP edge` 与 `mode→PV target`，两者应在切换到同一实例后立即接受；再覆盖 supervisor 未见到的 `PP→PV→PP`、`PV→PP→PV`、命令后同值 0x6060、同值 0x6060 后新命令，确认旧命令清除/拒绝而最终实例的新命令可接受。把 mode-instance 置于 `UINT32_MAX` 后合法写到 `0`，确认相等性比较仍识别新实例且不接受 wrap 前命令。
- PV 验证数值相同但 target epoch 新增仍产生一次 update，无新 target epoch 时保持当前实例最后合法目标；退出/重入 PV 或同值 mode 新实例后旧 target 不自动恢复，只有绑定最终 active instance 的新 generation 可更新。随后分别令本地/远端 NMT 非 Operational、Heartbeat 未配置/首包未到/超时，确认保持目标不能绕过强制 `0x6040=0`。
- 对象读写范围，以及 Operation enabled / `torque_enable=true` 模式切换冲突返回确定 abort code；拒绝前后 mode-instance epoch 必须不变。
- CiA 402 PDS 和 PP/PV 行为与主站互操作，而不只在本地状态机测试。
- EDS 与运行时对象逐字段比对，Vendor-ID/Product code 必须是组织合法分配值。

### CiA 1301

只有接入完整协议栈后才建立 XDD、USDO/PDO/NMT 测试，并安排 CANopen FD 一致性测试。在此之前测试报告标题不得出现“CiA 1301 passed”。

## 环境与可靠性

- 最低/最高供电、冷/热启动、温度循环、长时间额定/峰值循环、堵转限制。
- EMC 预兼容与正式辐射/传导/抗扰、EFT/ESD/surge（按产品环境）。
- 电源瞬断/brownout、反接/热插拔策略、回生和急停。
- 编码器磁场、振动、冲击、线束摆动和连接器接触不良。
- Flash 参数掉电写入、磨损、CRC 损坏、固件更新中断与回滚。
- 多板/多电机/多批次统计，不能只以一块“黄金样机”放行。

## 量产测试

每块板至少自动验证：

1. MCU ID/Flash/RAM 基础测试、固件签名/hash、option bytes。
2. 全部电源轨、GPIO 安全默认态、gate EN/RESET/SLEEP。
3. 三相 PWM、互补/死区、BKIN 硬件关断和 gate fault feedback。
4. ADC 零偏/增益治具、母线/温度输入，保存板级校准。
5. MT6701 SSI/CRC/磁场和电角度零位；绑定电机/板序列号。
6. CAN-FD 收发、错误计数和 ID/node 参数。
7. 低能量相序/小电流 FOC，检查方向、空载电流和振动。
8. 校准记录 CRC、结果上传和不可混淆标签。

生产测试的 limit 应由设计验证样本统计和测量系统分析确定，不应直接复制开发板的一次读数。

## 发布包和签字

量产候选发布包至少包含：

- 源码提交、submodule 提交、无未跟踪修改证明。
- 可重现构建说明、工具链容器/安装包 hash、完整构建日志。
- ELF/HEX/BIN/map/size/stack usage、符号与许可证清单。
- 最终原理图/PCB/BOM、`.ioc` 与 pin/clock report 的一致性证明。
- 参数 schema、默认/允许范围、每块板校准记录。
- 需求—测试追踪、单元/静态/HIL/目标/环境/EMC 报告。
- 安全分析、已知限制、现场升级/回滚/故障诊断流程。
- 控制、硬件、安全、制造和产品责任人的评审签字。

## 当前不得勾选的项目

在拿到最终两/三分流功率板并完成测试前，以下声明必须保持未完成：

- [ ] 真实相电流环在目标板通过。
- [ ] MIT 转矩精度和稳定性在测功机通过。
- [ ] 硬件 BKIN/OCP 最坏关断时间通过。
- [ ] 功率级/电机温度两条检测链已接入，开短路、热降额和 bits 3/4 反应通过；当前代码未实现。
- [ ] 若产品要求 STO，独立硬件安全链、反馈/诊断和系统验证完成；当前 `STO_ACTIVE` 只是软件位。
- [ ] 300 rad/s OVERSPEED trip 在 450 rad/s PLL 上限内可达，并通过正反向阈值/迟滞/反应测试。
- [ ] 已实现的 MT6701 500 µs timeout → 原子故障/gate off → HAL abort 流程通过晚到 IRQ、重复 timeout 与 abort 失败测试。
- [ ] 已实现的 ES0431 §2.5.10 JEOS 后 regular 路径通过全相位压力测试；Rev Y injected JDR1 dummy/JDR2 选择无 channel-0 零返回。
- [ ] ES0431 §2.5.8/§2.5.10 revision 白名单与 JDR1/JDR2 选择、§2.5.9 首 dual sequence 丢弃、§2.5.11 dual injected simultaneous、`ADC_CCR.DUAL=0b00101`、`JSQR.JL=1`、`CCR4=4164` 采样窗及目标板同步/精度证据完成。
- [ ] Rev Z VBUS regular length=2、rank 1 capture/OVR-preserved/EOS 路径及 DNL/阈值测试通过；所有 ADC restart 路径重新 arm §2.5.9 丢弃。
- [ ] 完整经典 CANopen 协议栈互操作通过。
- [ ] CiA 1301 CANopen FD 一致性通过。
- [ ] EMC、热、寿命和系统安全放行。
- [ ] SimpleFOC Mini 具备真实 MIT 能力（按现有原理图此项不能通过）。

完成代码不是完成产品。只有这些证据与指定硬件、参数和版本绑定后，才可以把该组合称作量产候选。
