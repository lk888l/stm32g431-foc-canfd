# 安全案例与故障反应

## 声明范围

本工程提供的是电机驱动的诊断和故障反应框架，不是经 IEC 61508、ISO 13849、IEC 61800-5-2 或其他功能安全标准认证的安全功能。单路 MCU、单个 BKIN、软件状态机和 CAN 命令都不能自行构成 STO、SS1 或人员保护。

系统集成者必须对机械、人员、电源和使用场景做风险评估，并在软件之外提供急停、隔离、机械防护、过流/过温和必要的安全转矩关闭链路。

## 安全目标

1. 关键测量或计算证据缺失时不产生非预期转矩。
2. 硬件过流/驱动故障在 CPU 不执行代码时仍能关闭 PWM。
3. 通信中断、旧包、坏包或模式错误不能维持旧转矩。
4. 故障清除必须要求原因消失、功率级关闭和新的复位边沿。
5. 所有饱和、降额和故障可以从反馈/事件记录追踪。
6. 任何实验降级配置都不能进入量产 MIT 构建。

## 防护层

```text
器件级保护       gate driver OCP/UVLO/TSD、MOSFET/保险/电源限流
        ↓
异步硬件关断     current comparator / nFAULT → TIM1 BKIN → MOE off
        ↓
实时软件保护     ADC/encoder/deadline 检测、dq/current/voltage 限幅
        ↓
运动状态保护     PDS、Quick stop、命令 watchdog、门极反馈联锁
        ↓
系统级安全       急停、STO、接触器、机械防护、上位机监督
```

上层不能补偿下层缺失。例如软件 ADC 过流不能替代异步比较器；CAN heartbeat 不能替代本地 MIT command watchdog；写零转矩不能替代撤销 gate。

## 故障位与默认策略

| bit | 名称 | 默认锁存 | 默认主要反应 |
|---:|---|:---:|---|
| 0 | PHASE_OVERCURRENT | 是 | 立即关 gate |
| 1 | DC_OVERVOLTAGE | 是 | 立即关 gate；系统另需处理回生能量 |
| 2 | DC_UNDERVOLTAGE | 否 | warning + Quick stop 请求 + gate inhibit |
| 3 | POWER_STAGE_OVERTEMP | 是 | warning + Quick stop 请求 + gate inhibit |
| 4 | MOTOR_OVERTEMP | 是 | warning + Quick stop 请求 + gate inhibit |
| 5 | GATE_DRIVER | 是 | 立即关 gate |
| 6 | CURRENT_FEEDBACK | 是 | 立即关 gate；MIT 不允许降级 |
| 7 | POSITION_FEEDBACK | 是 | Quick stop 请求 + gate inhibit |
| 8 | OVERSPEED | 是 | Quick stop 请求 + gate inhibit；trip 300 rad/s、PLL 上限 450 rad/s，仍待目标阈值/反应验证 |
| 9 | CONTROL_DEADLINE | 是 | 立即关 gate |
| 10 | GATE_INTERLOCK | 是 | 立即关 gate；表示命令与物理反馈不一致 |
| 11 | CAN_BUS_OFF | 是 | Quick stop 请求 + gate inhibit |
| 12 | COMMAND_SOFT_TIMEOUT | 否 | warning + controlled/Quick stop，转矩上限降到 Q15 8192（25%） |
| 13 | COMMAND_HARD_TIMEOUT | 是 | 立即关 gate |
| 14 | PROTOCOL | 是 | Quick stop 请求 + gate inhibit |
| 15 | STO_ACTIVE | 是（当前软件视图） | 立即关 gate；实际 STO 状态必须来自独立安全电路 |

便携安全模块的通用缺省超时是 soft 100 ms、hard 500 ms；当前 `App/app.c` 在初始化时将其覆盖为板级 MIT 的 5 ms stale、20 ms disable。任何重构都必须保留或进一步收紧这一覆盖，并由失控距离/能量分析验证。

表中存在 fault bit 不等于检测链存在。当前 `collect_faults()` 没有温度输入，bits 3/4 不会由实物过温置位，thermal torque limit 固定为满量程；bit 15 也没有物理 STO 输入或双通道诊断。这三项是明确投产阻项，不能计入安全覆盖率。栅极驱动器自身 TSD/nFAULT 若存在，只能作为独立硬件故障路径，不能替代电机绕组温度模型或合格 STO。

### 当前默认掩码的优先级说明

`gate_inhibit` 的优先级高于 `controlled_stop_request`。因此欠压、过温、位置反馈、超速、bus-off 和 protocol 等故障虽然置 Quick-stop/controlled-stop 请求，但默认掩码同时使 `gate_enable_allowed=false`，实际结果可能是撤销门极而不能主动制动。只有 soft command timeout 默认允许短暂保留 gate 并做受控停机。

这不是对所有机械系统都正确的最终策略：

- 垂直轴可能需要制动器和受控停机后再 STO。
- 回生能力不足时主动减速可能加剧 DC 过压。
- 位置传感器失效时闭环主动制动本身可能危险。
- 过温可能适合先降额，也可能要求立即断能。

量产项目必须在 FMEA 中逐故障选择“立即断能 / 有限转矩受控停机 / 保持功率等待机械制动器”，并据此修改和测试掩码。不能仅根据函数名推断已经实现 SS1。

## 故障检测要求

### 相电流/过流

- 第一通道：模拟比较器或驱动器 OCP 直接 BKIN，关断延迟用示波器测量。
- 第二通道：ADC 每周期检查绝对电流、三相一致性、offset、饱和和采样窗口；当前上电校准还要求 A/B 各 `2048` 个样本全部位于 `[8, 4087]`、均值位于 `[512, 3584]`、峰峰值不超过 `64 count`。
- 任一路轨到轨、offset 或噪声检查失败即把 current feedback 标为 invalid，并沿 `CURRENT_FEEDBACK` gate fault 路径关断；不得冻结最后一次有效电流继续运行。
- 这些 ADC 门限必须按量产模拟前端的输出摆幅、公差、开关噪声、温漂和样本统计审定。放宽门限属于安全相关参数变更，需要故障注入和边界回归。
- SimpleFOC Mini 实验配置因物理上没有相电流链而显式跳过校准；“跳过”不等于“校准通过”，该配置始终不得放行真实电流环、转矩或 MIT。
- ES0431 §2.5.8/§2.5.10 规避已实现为 ADC1/2 两个同通道 injected rank 和 revision-aware 选样：Rev Z `0x2001` 保留 JDR1，Rev Y `0x2002` 保留 JDR2，Rev X `0x2003` 使用 JDR2，未知 revision 在 GPIO 安全态建立后 fail-closed。§2.5.9 由每次实时启动后无条件丢弃第一整个 dual injected sequence 覆盖，该序列不得进入 FOC/offset。VBUS regular 只在 JEOS 后确认 ADC idle 再启动；不从低速任务任意时相直接启动，也不软件 stop injected。
- ES0431 §2.5.11 已选择勘误明确排除的非 alternate `ADC_DUALMODE_INJECSIMULT`，不再依赖有疑问的 timer/ADC prescaler 条件。但这仍只是设计规避：启动时必须验证 `REV_ID`、首 sequence 丢弃、`ADC_CCR.DUAL=0b00101`、两路 `JSQR.JL=1`、ADC2→ADC1 启动次序及两个 JDR 更新，并以各允许 revision 的所选 aperture、同步相位、DNL/增益/串扰数据放行；任何配置或硅版本回退都必须保持 gate inhibited。`CCR4=4164` 让两个 aperture 围绕 crest，但没有目标波形与模拟链建立证据前同样不能放行。
- Rev Z 的 ADC1 regular VBUS 已按 §2.5.8 配成同通道双 rank，动态 EOC/EOS/OVR IRQ 与 `ADC_OVR_DATA_PRESERVED` 保留/发布 rank 1、丢弃 rank 2。该设计措施仍须实读 `SQR1.L=1`，并覆盖 rank 1 及时服务、延迟 OVR-preserved、异常 flag 顺序与 injected 抢占；目标证据完成前不能把母线监测链计为已验证覆盖。
- §2.5.9 丢弃状态必须在每次 ADC stop/disable/recalibration 后 restart 时重新 arm。当前唯一 `bsp_start_realtime()` 路径已做；未来新增 restart 路径若遗漏，会让可能错误的首转换进入 FOC/offset，必须作为代码审查和故障注入项。
- 比较器阈值高于正常峰值、低于功率器件安全边界，并考虑 blanking、噪声和最坏公差。
- BKIN 测试必须包括 CPU halt、全局中断关闭和代码卡死。

### 电压/温度

- 过压阈值要低于最小额定器件并留瞬态余量；软件 1 kHz 检测不能捕获所有尖峰。
- 欠压需要迟滞，避免反复 enable/disable；brownout 后必须重新走 PDS。
- 温度传感器断路/短路是故障；热模型必须限制绕组温度而不仅是 PCB 温度。

### 编码器

- MT6701 每帧 CRC、磁场强弱、超速、时间戳；连续错误阈值按失控距离设置。
- 角度增量与最大速度一致性检查；跳变不能进入 PLL。
- 有效样本间隔超过 PLL 最大步长时允许用绝对角、零估计速度重捕获，目的是恢复诊断能力，不是自动恢复 torque；先前的位置反馈故障继续锁存，gate/PDS 必须保持安全态直到新的合法 Fault reset。
- PLL 最大估计速度现为 `450 rad/s`，软件 OVERSPEED trip 为 `300 rad/s`，判据已经可达；但仍必须用钳位前/独立参考验证别名、时间戳抖动、正反向阈值、迟滞和反应。完成目标/HIL 注入前不能把它计为已验证保护覆盖。
- MT6701 DMA busy 超过 `500 µs` 时先在临界区锁存位置反馈故障并关 gate，再由 background `HAL_SPI_Abort()`、释放 CS、发布无效样本并允许流水线重启。锁存故障不会随 abort 清除；仍须验证晚到 IRQ、HAL 卡死/失败、重复 timeout 和缓冲撕裂，不能把调用 abort 本身算作恢复证据。
- AS5600 实验板的总线成功不等于低延迟健康，高速时仍须限制模式/速度。

功率已使能且必需反馈失效时，20 kHz 快环在同一载波中断写中性矢量、撤销 gate 并锁存故障，不能等 1 kHz 监督器。测试必须测量从坏样本/反馈 stale 到物理 gate-off 的时间；软件中性 duty 本身不计作关断完成。

### 控制 deadline

- 使用 DWT cycle counter 测快环进入、退出和最大嵌套时间。
- 20 kHz ADC IRQ 只执行快环，每 20 个样本 pend 一次优先级 5 的 PendSV；CiA 402、轨迹和安全监督在 PendSV 中运行。快环和监督器分别计算 deadline，外环不得搬回 ADC IRQ。
- 单次超期是否立即关断、允许几个连续丢周期必须由风险分析决定；默认保守锁存。
- IWDG 当前约 100 ms（实际受 LSI 容差影响）。`bsp_start_realtime()` 后首次进入 main background 时按 DWT 开始 5 ms grace；此后只有 20 kHz/1 kHz ISR 的**完成计数**分别在 DWT 的 500 µs/5 ms 窗口持续前进，且 TIM2/DWT 在 2 ms 内互相观察到进度时才刷新。任一 stale，main 立即撤销 gate、锁存 `CONTROL_DEADLINE` 并停止喂狗。
- 交叉监测覆盖单个计数器停摆，不证明两个同主时钟域计数器的独立性。仍须分别及同时冻结 TIM2/DWT、停 TIM1、屏蔽 PendSV、卡死 background，并实测 gate-off 与 LSI IWDG 复位时间。

### 控制数值与饱和

- d/q PI 在两个状态副本上形成候选；dq 圆限幅和 SVPWM 最小脉宽得到最终 `voltage_scale` 后，再把 `u_dq_applied = u_dq_limited × scale` 同时 tracking 回算到两轴积分器。这样可以覆盖“每轴均未到独立限幅、但向量合成后超过逆变器电压圆”的耦合饱和。
- 两轴 PI、变换、限幅、SVPWM 和两轴 tracking 是一个软件提交事务。任一轴非有限、timestep/范围非法或后级计算失败，都不得保留另一轴已经前进一步的积分器；输出回退中性值，功率已使能时由数值故障路径撤销 gate。中性值本身仍不是安全关断。
- 上述结构是算法措施，不是稳定性证据。必须在母线/速度/温度全包络内验证持续双轴饱和、饱和方向变化、解除饱和后的恢复、参数误差及 20 kHz 最坏执行时间；没有目标实测前不能据此声明抗饱和已经量产放行。

### 通信

- 只有完整通过长度、ID、CRC、保留位、范围、mode 和 sequence 的命令才能刷新 watchdog。
- 旧包/重复包/sequence gap/bus-off 均有计数和事件记录。
- 固定 RPDO2 只在 0x607A/0x60FF 全量校验后批量更新控制值和两个 target epoch，并让 velocity generation 绑定提交时的 mode instance；PP bit 4 上升沿冻结此前已提交的位置 generation、bit 5/6、mode 与 instance。每次合法 0x6060 写（含同值）推进新 mode instance；非法、PDS Operation-enabled 或已发布 `torque_enable=true` 时拒绝且不推进。后者覆盖仍带受控转矩的 Quick-stop/Fault-reaction，必须等 torque=false 且不处于 Operation enabled 才能切换。模式数值或实例任一变化都清 PP/PV 活动目标，只允许 instance 与 active 匹配的命令。目标晚到不能补救，旧实例或已消费/拒绝的 generation（包括相对旧增量）不能重放。由于一次 1 ms RPDO 处理先 RPDO1 后 RPDO2，主站必须让 RPDO2 跨过处理屏障后再产生 RPDO1 bit 4 上升沿；同轮 pending 按拒绝处理。
- 经典 CANopen 的非零 Controlword 要求本地 NMT Operational、0x1016 已配置、已收到首个 heartbeat、未超时且远端 NMT Operational；默认 0x1016=0 时不能使能。任一条件失效或 communication reset 都强制 `0x6040=0`。该反应依赖 `CO_process()` 后台持续运行，必须纳入任务活性和 IWDG 健康票据。
- 1 kHz 监督器发布的有效故障掩码已实际接入 CANopen：通过 EMCY status bit 0x40、0x1001 manufacturer bit 0x80、与 0x603F 一致的 error code 和 32-bit fault mask infoCode 聚合，并进入 0x1003 history；故障变化 reset/report、清零 reset。EMCY 是诊断，不是关断执行器。
- 驱动 manufacturer fault bit `0x40` 不触发 NMT error-register 自动停机：PDS 已撤销 torque，节点保持受 heartbeat 监督的 Operational 以便读取诊断和提交 CiA 402 Fault reset。只有 bus-off/HB consumer failure 通过 NMT 错误策略退出 Operational；不得把“仍是 Operational”解释为功率级仍在运行。
- NMT reset-node 先执行 fatal gate-off 再 NVIC reset；不得在复位窗口保留 MOE。
- 恢复通信不自动恢复 torque；需要重新初始化序号、重新发模式/目标并走 PDS 使能。

Bus-off 分成两层恢复：HAL ErrorStatus 先锁存 `CAN_BUS_OFF`，再清 M_CAN `INIT` 让控制器按 ISO 规则恢复总线参与；软件故障不会随控制器恢复自动清除。只有 PSR 已非 bus-off、gate 物理关闭、速度处于复位容差且收到新的 Fault reset 上升沿，才允许清锁存，随后仍停在 PDS 安全状态等待完整使能。CAN 断线/短路解除、收到 heartbeat 或 NMT Operational 都不能替代该复位边沿。

## 故障反应状态

定义三种反应，实际项目应对每个故障冻结：

1. **Derate/warning**：保持闭环但限制 `Iq/torque`，用于仍有可靠反馈的渐进热/电源限制。
2. **Controlled stop / Quick stop**：在可靠角度、电流和母线能量路径存在时，用批准的减速度把速度/转矩降到窗口，再撤销 gate。
3. **Immediate gate off**：立刻清物理相通道 `CCxE/CCxNE` 和 `GATE_ENABLE`；BKIN/fatal 路径同时清 `MOE`。普通软件关断保留 CH4/MOE 仅用于继续监督 ADC 快环，不能让任何相输出保持启用。适合过流、过压、驱动故障、当前反馈失效、deadline、gate interlock、hard timeout 和 STO。

“立即关 gate”不保证电机立即停止；惯性负载会自由滑行，永磁电机也可能向母线回馈。需要机械制动或接触器时必须由系统状态机协调。

## 锁存与复位

- `active_faults` 反映当前检测原因；`latched_faults` 保存需要人工/主站确认的历史原因；反馈同时发布两者。
- Fault reset 是 Controlword bit 7 或厂商 clear-fault 的新上升沿，电平一直为 1 不重复复位。
- 复位前必须确认：所有相关 active 原因已消失、gate feedback 为 off、速度/母线/温度在安全窗口、急停/STO 已按系统流程复归。
- 复位只清已经不活跃的锁存位；仍活跃的故障在同一个 tick 重新锁存。
- 复位后保持 Switch on disabled，不恢复目标、积分器或旧 MIT 命令。
- 厂商 zero-position 是独立危险动作：只在新上升沿、gate 已失能、PDS 非运行或 Quick stop、速度处于零位容差时执行。被拒绝的边沿必须释放后重发，不能在条件稍后满足时自动补执行。

## 上电、调试和更新

- GPIO 在复位默认态就应使驱动关闭；软件初始化顺序不能是唯一防线。
- 调试器 halt 时，由 BKIN/独立 watchdog/安全电路保证关断。禁止在带负载时用断点停住电流 ISR。
- Bootloader、固件校验失败、Flash 参数无效、时钟异常都保持 gate off。
- 固件升级需要签名/完整性、回滚和掉电恢复策略（根据产品威胁模型）；升级模式不得使能功率级。
- 生产测试应实测故障引脚而不是只修改软件变量。

## 最小安全证据

量产放行包至少包含：

- 系统危害分析、FMEA/FMEDA（若适用）和每个故障的反应理由。
- 原理图/PCB 安全评审，硬件过流/BKIN/急停测试波形。
- 最坏关断时间、停止距离、回生母线峰值和热试验。
- 软件需求到单元/HIL/目标测试的追踪矩阵、覆盖率和静态分析报告。
- 编译器/依赖/参数/校准可追溯性和独立评审签字。
- 现场错误记录与安全相关变更的再验证策略。

在这些证据完成前，应将工程称作“开发/验证基线”，不能称作功能安全或量产认证固件。
