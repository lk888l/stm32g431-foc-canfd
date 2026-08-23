# PV、PP 与 MIT 模式

## 通用状态约束

支持的模式值：

| 模式 | 0x6060 / 0x6061 值 | 标准性 |
|---|---:|---|
| Profile Position (PP) | `1` | CiA 402 标准模式 |
| Profile Velocity (PV) | `3` | CiA 402 标准模式 |
| MIT impedance | `-128` (`0x80`) | 厂商自定义，不是 CiA 402 标准模式 |

模式切换只允许在 `mode_change_inhibited=false` 时完成；当前条件是 PDS **不处于** `Operation enabled`，并且上一份发布状态的 `torque_enable=false`。因此仍在输出受控转矩的 Quick-stop Active 或 Fault reaction 阶段也拒绝 0x6060，不能靠状态名称已离开 Operation enabled 就提前换模式；必须等受控停机撤销 torque。被禁止的写入返回状态冲突，模式数值和 mode-instance epoch 都不改变；长度错误或不支持的模式同样不推进实例。每次真正合法的 0x6060 写入都会产生新的 32-bit mode instance，**即使写入值与当前 0x6060 相同**。1 kHz 管理器还会以当前 torque-active 状态做第二层保护；只有允许转换时，才把数值/实例任一变化的新实例设为 active，清除 PP 活动目标/队列/相对基准和 PV 活动目标，并将轨迹状态对齐到反馈。

PP bit 4 request 快照模式数值和 mode instance；PV 的 0x60FF 写入（固定 RPDO2 的速度成员也一样）记录写入时的模式/实例。命令只有在该实例与 active instance 完全相等时才可执行。保留已消费的 target/request epochs 仍防止同一目标重复使用，但它只是第二层防重放，不足以替代 mode-instance barrier。这样既允许 `0x6060 mode → target → PP bit 4 edge` 或 `mode → PV target` 全部在第一次 1 ms supervisor 前完成并立即生效，也能拒绝 supervisor 未见到的同 tick `PP→PV→PP`/`PV→PP→PV` 旧命令、命令之后的同值 0x6060 重写，以及 epoch wrap 前实例的命令。实例用相等性判断，`UINT32_MAX→0` 被视为新实例。

`Halt`（Controlword bit 8）、Quick stop 和安全故障的优先级高于任何模式命令。目标超范围时先拒绝或钳位并置 `Internal limit active`，不得整数回绕。

## PV：Profile Velocity

PV 以 0x60FF 的 Q16.16 rad/s 为目标，使用独立的加速和减速限制：

1. 目标先钳位到配置的最大绝对速度。
2. 同方向增速使用 0x6083 加速度。
3. 减速、反向或降低速度使用 0x6084 减速度；反向必须先穿过零速。
4. 每个 1 kHz tick 用梯形积分更新连续位置参考，避免速度阶跃产生位置不连续。
5. 速度进入容差窗口后可置 target reached；该位表示轨迹达到目标速度，不表示电机已经停止。

PV 的正常信号链是 `速度轨迹 → 速度 PI → Iq* → 电流环`。在实验 AS5600 配置下只能采用显著降低的速度环带宽和小电压/电流等效限制；如果没有实际相电流，就不能给出可验证的转矩限幅。

每次成功写 0x60FF 都增加 velocity target epoch，即使数值与上次相同也代表一次新的刷新；PV 只消费尚未消费、写入时模式为 PV，且 `target_velocity_mode_instance_epoch == active_mode_instance_epoch` 的 generation。没有新 epoch 时继续保持当前实例最后一个合法 PV 目标，不把旧 OD 值重复解释为新命令。模式数值或实例变化会先清活动目标；因此退出再进入 PV、同值重写 0x6060，或同 tick 切离/重入后，都必须在最终 active instance 下重新写 0x60FF/RPDO2，旧速度不能恢复。这个“保持最后目标”不绕过通信安全：经典 CANopen 的非零 Controlword 仍受本地/远端 NMT Operational、已配置且新鲜的 Heartbeat consumer 约束，条件失效会强制 `0x6040=0`。

Quick stop 时不得继续跟踪原 0x60FF。按配置的安全减速度把速度参考降为零，确认实际速度和转矩进入窗口后再撤销 gate；若故障属于立即关断类，跳过受控减速。

## PP：Profile Position

PP 以 0x607A 的 Q16.16 rad 为目标，轨迹生成器每个 tick 根据剩余距离重新计算制动速度：

```text
v_brake = sqrt(2 × deceleration × remaining_distance)
v_ref   = direction × min(profile_velocity, v_brake)
```

因此自动形成梯形或三角速度轨迹，也允许运动期间改变目标。近目标并且速度足够低时，状态精确贴合目标，避免编码器量化造成一个 count 左右的极限环。

### Controlword 握手

| bit | 名称 | 本工程行为 |
|---:|---|---|
| 4 | New set-point | 只在 `0→1` 写入上升沿形成一次请求；该时刻冻结此前已提交的 0x607A 值/target epoch、bit 5/6、mode 和 mode-instance epoch，持续为 1 不会重复触发 |
| 5 | Change immediately | 置 1 时立即替换当前目标并清空 pending；否则运动中最多排队一个目标 |
| 6 | Relative | 相对最后一次已接受的命令目标；首次命令相对实际位置 |
| 8 | Halt | 用当前配置的减速度把速度轨迹降到 0，优先于继续 PP/PV/MIT 输出；不是立即硬关断 |

成功接收后 Statusword bit 12 作为 set-point acknowledge。主站应看到确认后把 bit 4 清零，等待 bit 12 清零，再发下一个上升沿。目标超软限位、pending 已满、模式快照不是 PP、请求的 mode instance 不是 active instance、整数运算将溢出，或该目标 epoch 已经消费时，都会拒绝命令且不改变当前有效目标。

每个 bit 4 请求和它捕获的位置 epoch 在排队/范围判断前就被标为已消费。因此目标在上升沿之后才到达时不能“补救”该请求；持续高电平不能重新尝试；已经接受或拒绝的 generation 不能由后续新边沿重放。相对目标也遵守同一规则，旧增量不能因为基准位置变化而再次相加。要重试只能先提交一个新的 0x607A generation，再完成新的 bit 4 低/高握手。

如果 bit 4 edge 形成后又合法写了一次 0x6060——即使仍写 PP——新 mode instance 也会清活动目标并让此前 request 的实例失配；它不会在下一 tick 执行。离开再重入 PP 也是同一规则。只有在最终 mode instance 下形成的新 edge 才有效；可以先预装一个尚未消费的 0x607A，但 edge 本身必须快照最终 active instance。

固定 PDO 下还存在明确的跨 RPDO 顺序：节点在一次 1 ms `CO_process_RPDO()` 中先处理 RPDO1(Controlword)、再处理 RPDO2(0x607A+0x60FF)。若两帧同时 pending，bit 4 会先快照旧位置 epoch，稍后提交的 RPDO2 不会回填它，PP 请求按 fail-closed 语义拒绝。安全主站顺序是：先确保 bit 4 已低且 acknowledge 已清；发送 RPDO2；让节点至少完成一次 1 ms RPDO 处理；再发送带所需 bit 5/6 和 bit 4 上升沿的 RPDO1。不能依赖 CAN 仲裁先后或“同一毫秒内发送”来替代这个处理屏障。

Statusword bit 10 仅在位置误差和速度都进入容差，且没有 active/pending 目标时表示 target reached；bit 11 表示内部限制。机械硬限位必须独立于软件位置窗口。

Halt bit 8 走与受控 Quick stop 相同的零速度轨迹路径，使用已校验的 profile deceleration（0x6084）逐 tick 减速；它不替代 BKIN、立即关断故障或系统急停。主站释放 Halt 后是否继续原目标必须按 PP/PV 状态和握手重新确认，不能假定旧命令一定安全恢复。

## MIT：阻抗/转矩模式

MIT 算法使用真实反馈计算：

```text
tau_request = Kp × (q_des - q) + Kd × (dq_des - dq) + tau_ff
Iq_request  = clamp(tau_request, tau_limit) / Kt
Iq_command  = clamp(Iq_request, current_limit, thermal_limit, supply_limit)
```

其中 `q` 是机械位置 rad，`dq` 是机械速度 rad/s，`tau` 是 N·m，`Kt` 是经过测功/电流标定的有效轴端 N·m/A。输出必须进入闭环 `Iq` 控制器，才具有物理转矩意义。

处理顺序是：

1. 验证帧长度、magic、版本、类型、保留位和 CRC16。
2. 验证序号单调和最大允许前跳，使用接收端本地时间戳。
3. 验证模式、使能标志及所有固定点范围。
4. 对 `q_des`,`dq_des`,`Kp`,`Kd`,`tau_ff` 分别限幅并记录限制位。
5. 计算请求转矩，再做总转矩、`Iq`、热和供电限制。
6. 命令 stale 时输出严格为零并请求受控停止；hard timeout 时立即撤销 gate。

板级快速通道定义 5 ms stale / 20 ms disable。便携安全模块自身的通用缺省值是 100 ms soft / 500 ms hard，但 `App/app.c` 初始化时已经显式覆盖成 5/20 ms；修改初始化或复用安全模块时，不能误用较慢的通用缺省值覆盖 MIT 阈值。

### MIT 非零转矩条件

除通用 PDS 条件外，MIT 要产生非零转矩还要求：

- 当前板型支持同步相电流测量，ADC 增益/偏置/符号有效。
- `Kt`、编码器方向和电角度零位均有有效校准记录。
- 电流环已在更低风险的固定 `Iq`/速度测试中通过稳定性与过流测试。
- 最新命令通过协议校验并在本地新鲜度窗口内。当前 PDS 可在尚无 fresh MIT command 时短暂进入 armed-neutral，但 `control_mit_compute()` 只会给零输出；5 ms soft timeout 请求受控停止，20 ms hard timeout 撤销 gate。不得把 armed-neutral 误记为有效 MIT 转矩使能。
- `Kp/Kd/tau/current` 均低于当前已批准的安全包络。

`BOARD_PROFILE_LAB_SIMPLEFOC_AS5600` 不满足第一项，必须让 `current_feedback_valid=false`，PDS 因而拒绝 MIT Operation enabled。禁止通过伪造该标志绕过联锁。

### 增益投运方法

首次 MIT 测试固定 `Kp=0`,`Kd=0`，只给很小的 `tau_ff`，确认 `Iq` 与测功方向/幅值。随后：

1. 保持 `Kp=0`，从很小的 `Kd` 开始验证阻尼和速度噪声。
2. 在机械固定装置和低电流上限下逐步增加 `Kp`，观察相位裕度、抖动和峰值电流。
3. 检查命令丢帧、重复、乱序、CRC 错、主站停止和 CAN bus-off 时的软/硬超时。
4. 在整个位置、速度、温度和母线范围复测；增益上限由最坏工况决定。

严禁在悬空电机、人员可接触机构或未验证机械限位上直接使用高 `Kp`。高刚度命令会把位置误差瞬间放大为饱和转矩。

## 无扰切换

- Disable operation 时，当前实现把位置轨迹重置到实际位置、速度参考重置到实际速度，并把速度 PI 的积分器/输出清零；它不会保持先前转矩。若机构依赖持续保持力，必须先由系统完成受控卸载/制动，不能把该切换描述为任意负载下的无扰保持。
- 从 PV/PP 进入 MIT 前必须关闭转矩；MIT 的 `q_des` 初值应等于当前 `q`、`dq_des=0`、`Kp/Kd/tau_ff=0`。
- 从 MIT 退出先将转矩平滑降到零，确认后再切模式。
- Fault reset 只清故障状态，不自动恢复上次目标；主站必须重新选择模式、重发参数和执行完整使能序列。
