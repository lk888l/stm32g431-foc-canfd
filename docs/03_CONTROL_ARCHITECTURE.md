# 控制架构、频率与单位

## 分层结构

```text
CANopen / 厂商 CAN-FD / 本地命令
                 │  范围、CRC、序号、本地到达时间
                 ▼
        CiA 402 PDS + 安全管理（1 kHz）
                 │  模式、门极许可、Quick stop、转矩降额
        ┌────────┼──────────┐
        ▼        ▼          ▼
       PP       PV         MIT             （1 kHz）
  位置轨迹   速度轨迹   阻抗→转矩→Iq
        └────────┼──────────┘
                 ▼
        位置环 → 速度环 → Iq*            （1 kHz）
                         Id* = 0
                 │
                 ▼
   电流采样 → Clarke/Park → dq PI → 电压圆限幅
                 │                         （20 kHz）
       编码器/PLL→电角度 → inverse Park → SVPWM
                                           │
                                           ▼
                                  TIM1 + gate driver
```

安全路径旁路普通控制流程：模拟比较器/驱动故障直接进入 TIM1 BKIN；软件 Break ISR、HardFault 和命令硬超时只做补充关断和故障记录，不能代替硬件链路。

## 调度与优先级

| 任务 | 参考频率 | 允许内容 | 禁止内容 |
|---|---:|---|---|
| TIM1/ADC 快环 | 20 kHz，50 µs | 取同步电流、角度预测、Clarke/Park、dq PI、限幅、SVPWM、更新 CCR、deadline 监测；每 20 次只 pend 一次 PendSV | 阻塞 HAL、外环、I²C、printf、动态分配、Flash 写入 |
| MT6701 采样 | 20 kHz，SPI3 DMA | 每个 ADC 快环消费上一完整 SSI 帧并启动下一帧；CRC/状态/时间戳提交，PLL 预测约一 PWM 周期延迟 | 阻塞 SPI、消费半帧、DMA 未完成时覆盖缓冲区 |
| 运动/安全监督 | 1 kHz，PendSV | PDS、PV/PP/MIT、位置/速度环、故障合并、降额、状态快照及独立 deadline | 在 20 kHz ADC ISR 内运行外环；直接无联锁开启 MOE |
| CAN RX/TX | 事件驱动 + 1 kHz 服务 | 验 CRC/长度/ID、序号守卫、无锁邮箱、PDO/反馈调度 | 在 ISR 内运行轨迹或电流控制 |
| 诊断 | 100 Hz–1 kHz | 统计最大执行时间、错误率、限幅、温度、电压 | 清除仍活跃的锁存故障 |
| Heartbeat | 10 Hz 参考 | 节点健康/状态 | 作为 MIT 唯一的命令新鲜度来源 |

当前 NVIC 优先级约定为：Break `0`、ADC/电流快环 `1`、SPI/编码器 `2`、FDCAN `3`、I²C `4`、PendSV 监督器 `5`，SysTick 保持最低 `15`。20 kHz ISR 每 20 个周期只设置 PendSV pending 位；外环不会在电流 ISR 栈上执行。每层均有独立 deadline 检测，任何更改都要重新测最坏嵌套时间和栈水位。

功率已使能时，20 kHz 快环若发现当前模式必需的编码器、母线或量产电流反馈失效，会在该载波中断先输出三相 50% 中性矢量并立即撤销 gate，同时锁存对应故障；不会等待下一次 1 kHz 监督。中性矢量只是撤销前的受控数值回退，真正安全动作仍是硬件 gate off/BKIN。

ADC1/2 每次 injected 都执行相同通道的两个 rank，快环按芯片 revision 选样：Rev Z (`REV_ID=0x2001`) 为 ES0431 §2.5.8 保留 JDR1；Rev Y (`0x2002`) 为 §2.5.10 丢弃 JDR1、保留 JDR2；Rev X (`0x2003`) 使用 JDR2；未知 revision 在 GPIO 安全态建立后拒绝。为满足适用于所有这些 revision 的 §2.5.9，`bsp_start_realtime()` 后第一整个 dual injected sequence 只清标志、不调用 FOC/offset 路径，第二个完整 sequence 才成为首个可用样本。低速 VBUS 任务只置请求，regular conversion 仅紧随 injected JEOS、确认 ADC idle 后启动。针对 §2.5.11，BSP 使用非 alternate 的 `ADC_DUALMODE_INJECSIMULT`，这是勘误明确不受该双 ADC 并发精度限制的模式；两路仍须保持同一 PCLK/4 与相同 rank/采样时间，TIM1_CC4 触发 ADC1 master，dual event 同步启动 ADC2 slave。`.ioc` 不给 ADC2 独立 trigger；手写 HAL 配置虽写入相同的 slave JEXTSEL/JEXTEN，但 dual injected 硬件忽略这些位，HAL start 也只使能 slave，因此不会产生第二次触发。初始化/产测必须记录 `REV_ID`，实读 `ADC_CCR.DUAL=0b00101` 与 `JSQR.JL=1`，证明 ADC2→ADC1 启动次序、首序列丢弃、两个 JDR 更新、revision 对应 rank、相对相位和精度，不能只凭初始化 API 返回成功放行。`CCR4=4164` 把两个采样 aperture 布置在载波 crest 附近；两条 rank 路径仍必须分别用目标波形证明处于有效低侧窗口。

这个首序列丢弃是 restart 状态，不是“一生只做一次”的常量。当前只有 `bsp_start_realtime()` 启动实时 ADC；若以后加入 stop/disable/recalibration/restart，必须在每次 restart 前重新 arm 丢弃标志。regular VBUS 也已为 Rev Z §2.5.8 使用两个同通道 rank：动态 EOC/EOS/OVR IRQ 配合 `ADC_OVR_DATA_PRESERVED`，无论 rank 1 是否在 rank 2 前及时服务，都只发布 rank 1、丢弃 rank 2。该逻辑仍须在目标硬件验证 regular `SQR1.L=1`、两种 IRQ 延迟路径和 injected/regular 互不干扰，不能只从源码分支推断正确。

20 kHz 快环的理论周期是 50 µs，PendSV 监督器周期是 1 ms。量产门槛建议把各自最坏执行时间（包括更高优先级嵌套）控制在周期的 70% 以下，并分别记录最大值；当前代码以各自完整周期作为硬 deadline，达到即锁存 `CONTROL_DEADLINE` 并撤销 gate。70% 是放行裕量，不能等到触发硬 deadline 才认为性能不足。

`bsp_start_realtime()` 后首次进入 main background 时启动 IWDG 健康聚合，并从该时刻按 DWT 给 5 ms 启动宽限。20 kHz current ISR 和 1 kHz PendSV 只在完整执行到末尾时各自递增完成计数；background 观察计数变化并用 DWT 分别要求 `<=500 µs`、`<=5 ms` 的进度。同时，TIM2 的数值变化必须在 DWT 的 `2 ms` 内出现，DWT 的数值变化也必须在 TIM2 的 `2 ms` 内出现。任一条件失败，main 立即撤销 gate、锁存 `CONTROL_DEADLINE` 并停止刷新约 100 ms 的 IWDG。

这能检测 TIM1/ADC、PendSV、单独 TIM2 或单独 DWT 停摆，避免命令/编码器年龄与 ISR 票据共享 TIM2 时形成共同盲区；但 DWT 与 TIM2 仍来自同一主时钟域，不是功能安全意义上的独立双通道。必须注入单独/同时计数器停摆、时钟切换、调试 halt 和寄存器破坏，并由独立 LSI IWDG 的实测复位作为最后证据。

## 单位和数值格式

控制内部全部使用单精度 SI 单位：

| 物理量 | 内部单位 | 协议格式 |
|---|---|---|
| 机械/电角度、位置 | rad | 有符号 Q16.16 rad |
| 机械/电角速度 | rad/s | 有符号 Q16.16 rad/s |
| 加速度/减速度 | rad/s² | 无符号 Q16.16 rad/s²（轮廓值） |
| 相电流、`Id/Iq` | A | 厂商反馈有符号 Q16.16 A；MIT 电流限幅 UQ8.8 A |
| `Vd/Vq`、母线电压 | V | 母线 UQ16.16 V |
| 转矩 | N·m | 厂商帧有符号 Q16.16 N·m；0x6071/0x6077 是额定转矩千分比 |
| MIT `Kp` | N·m/rad | UQ8.8 |
| MIT `Kd` | N·m·s/rad | UQ8.8 |
| 时间 | s / µs | 本地单调 32 bit µs 时间戳 |

所有多字节线上字段都显式小端序列化，不能把 C 结构体直接 `memcpy` 到 CAN 帧。TIM2 以 1 MHz 自由运行，提供 32 bit 本地微秒时基，约每 71.6 min 回绕；时间差使用无符号减法，可跨一次 wrap，所有超时必须小于 `2^31 µs`（约 35.8 min）。DWT 170 MHz cycle counter 用于 WCET、ISR 完成计数的新鲜度和 TIM2 交叉监测，不作为协议长时间戳。

Q16.16 的转换必须做饱和和有限数检查：

```text
physical = signed_value / 65536
signed_value = saturate(round(physical × 65536))

UQ8.8 physical = unsigned_value / 256
```

## 角度和方向约定

推荐统一定义：

```text
theta_m  = 解卷后的机械角，正方向由机械接口定义
omega_m  = d(theta_m)/dt
theta_e  = wrap_0_2pi(pole_pairs × direction × theta_m - electrical_zero)
omega_e  = pole_pairs × direction × omega_m
```

正 `Iq` 必须产生正机械转矩。`Ia/Ib/Ic`、PWM A/B/C、编码器方向和 `electrical_zero` 四者必须通过低压/限流对齐一次性确认；不能靠同时反转多个符号“调到能转”，那会掩盖保护和坐标错误。

角度 PLL 是二型环，接收 `[0,2π)` 的带时间戳样本并输出连续角度/速度。正常更新要求相邻真实角位移小于 π；无效/非有限输入不会更新。若一次有效样本与上次的间隔超过 PLL 最大步长（当前 10 ms），实现会以该绝对角重新初始化并把估计速度清零，避免此后所有样本永久被旧时间戳拒绝；此前产生的 `POSITION_FEEDBACK` 安全故障仍保持锁存、gate 保持关闭，必须按安全复位流程恢复。高速时可按已估计速度预测到 ADC 采样时刻，但预测时域必须受限，并在源样本 stale 时立即失效。

机械超速 trip 是 `300 rad/s`，PLL 最大估计速度已提高到 `450 rad/s`，所以该判据现在可达。仍要用钳位前角增量参考与目标注入验证正反向阈值、时间戳抖动、PLL 饱和和反应时间；可达不等于已经取得安全覆盖证据。

## 电流 FOC

快环的参考过程：

1. 从同步 ADC 获取 `Ia`,`Ib`，验证窗口并重构 `Ic`。
2. 幅值不变 Clarke 得到 `Iα,Iβ`。
3. 用 ADC 样本对应时刻的 `theta_e` 做 Park，得到 `Id,Iq`。
4. `Id*=0`（基速以下）；外环或 MIT 给出受热/电源/安全降额后的 `Iq*`。
5. dq PI 先在两个控制器副本上计算本周期候选状态。每轴保留自身输出边界的 back-calculation 抗积分饱和：

   ```text
   u = Kp·e + integrator + feedforward
   d(integrator)/dt = Ki·e + Kaw·(u_limited-u_unlimited)
   ```

6. 可加入经电机参数验证的交叉耦合/反电动势前馈：

   ```text
   Vd_ff = -omega_e · Lq · Iq
   Vq_ff =  omega_e · (Ld · Id + flux)
   ```

7. 将 dq 电压向量按可用母线电压做圆限幅；inverse Park 后由 SVPWM 再施加母线与最小脉宽约束。控制器用于 tracking 的最终命令电压不是每轴 PI 的独立限幅值，也不是仅做圆限幅后的值，而是：

   ```text
   u_dq_applied = u_dq_circle_limited × svpwm.voltage_scale
   integrator_dq += Kaw × (u_dq_applied - u_dq_pi_limited) × dt
   ```

   这里的 `applied` 是固件根据最终调制 scale 得到的执行器命令电压估计；死区、器件压降等实物非理想仍须另行建模/测量。
8. 两轴 tracking 候选都通过有限数、范围和 timestep 校验后，才同时提交 d/q 积分器与输出。任一 PI、坐标变换、dq 圆限幅、SVPWM 或任一轴 tracking 失败，都不允许只提交一轴或半个周期的状态；本周期输出中性值并由安全路径处理数值故障。
9. 用中心对齐 SVPWM 更新三相 CCR。无效输入返回三相 50% 中性占空比，但安全层仍应撤销 gate enable；中性占空比不是安全关断。

当前初值 `Kp=0.60 V/A`、`Ki=1800 V/(A·s)` 与资料中的 `L≈0.86 mH`、`R≈2.55 Ω` 对应约 700 rad/s（约 110 Hz）的保守电流环带宽。理论起点可用 `Kp=L·ωc`,`Ki=R·ωc`，但最终必须由实测采样延迟、运放带宽和 PWM 延迟约束，不能机械提高到 `fs/10`。

## 位置与速度环

- 速度 PI 输出 `Iq*`；默认 `Kp=0.005 A/(rad/s)`、`Ki=0.10 A/rad` 只作为首次低电流辨识起点。
- PP 的位置 P 环输出速度参考；默认 `Kp=15 s⁻¹` 同样不是整机整定结果。
- 限幅顺序是：用户/协议范围 → 轨迹速度/加速度 → 位置/速度环 → 电流/转矩 → 热/电源降额 → 硬件保护。
- 外环限幅时必须实施积分条件化或反算，且切换模式/使能时重置到当前反馈以实现无扰切换。
- 电流环带宽应显著高于速度环，速度环显著高于位置环。AS5600 实验配置还需按其延迟把速度/位置带宽进一步降低。

## 数据一致性

- ISR 与 1 kHz 任务之间使用单写者邮箱、短临界区或序列锁快照；禁止共享半更新结构体。
- 每份命令携带本地接收时间和序号；发送方时间只用于诊断，不能直接替代本地 watchdog。
- 固定 CANopen RPDO2 的 0x607A/0x60FF 先 staging、全量校验，再同时更新值与各自 target epoch；任一字段失败都不能向控制任务发布半个目标。其 velocity generation 同时记录当前模式和 mode-instance epoch。PP bit 4 上升沿另行冻结当时已经提交的位置值/target epoch、选项位、模式和 mode instance，目标晚到不能回填已经形成的请求。
- 每次通过长度、支持模式和 PDS 状态校验的 0x6060 写入都会推进 32-bit `mode_instance_epoch`，同值重写也不例外；格式/模式非法，或 PDS 为 `Operation enabled` / 仍发布 `torque_enable=true` 时被拒绝，模式值和实例都不变。该保护覆盖仍带转矩的 Quick-stop/Fault-reaction 受控阶段；只有 torque 已撤销且 PDS 不在 Operation enabled 才允许。1 kHz 管理器检测模式数值或实例任一变化时清空 PP active/pending/relative base 和 PV active target。保留 consumed target epochs 仍阻止目标重复消费，但 mode freshness barrier 的权威依据是独立实例 epoch，不能再把“只保留 consumed epoch”描述为充分条件。
- PP request 和 PV velocity 只在其快照/标签的 mode instance 与 active instance 相等时可执行；实例比较只用相等/不等，`UINT32_MAX→0` 仍是新实例。允许主站在首次 supervisor 前完成 `mode→target→PP edge`（或 `mode→PV target`），因为这些字段携带同一个新实例；同 tick 切离/重入、在命令后同值重写 0x6060，或使用旧实例命令都必须清活动目标并拒绝重放。
- 参数更新采用“校验到 shadow → 在安全状态原子提交 → 记录版本/CRC”，不能边运行边逐字段修改控制器。
- 实时路径不分配内存、不递归、不记录文本；只增加有界计数器或写固定大小事件记录。

## 启停顺序

上电后保持 RESET/SLEEP/EN 和物理相通道 CCxE/CCxNE 为安全态，依次完成：HAL 时钟、ADC 自校准、传感器健康、零电流偏置、总线/温度检查、编码器方向/零位数据校验、CAN 初始化、PDS 进入 `Switch on disabled`。无引脚 CH4/MOE 可在 gate 关闭时继续产生 ADC 快环触发。只有收到合法 Controlword/命令且安全层允许，才能请求相通道与 gate enable；必须由物理 gate feedback 确认后，PDS 才进入 `Switched on`/`Operation enabled`。

关断时先根据故障类别执行受控减速或立即 BKIN，随后撤销物理相通道 `CCxE/CCxNE` 和 `GATE_ENABLE`；BKIN/fatal 路径同时清 `MOE`。普通 Idle 为保持 CH4 ADC 触发可以保留 MOE，但不能保留任何物理相通道。软件命令值归零不等同于硬件关断。
