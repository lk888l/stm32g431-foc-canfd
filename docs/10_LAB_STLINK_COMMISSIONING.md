# SimpleFOC Mini + AS5600 的 ST-Link 投运

本文只适用于 `BOARD_PROFILE_LAB_SIMPLEFOC_AS5600`：2804 电机、AS5600、三输入 SimpleFOC Mini 和 STM32G431。它不具备相电流采样，所有“Voltage/Velocity/Position”均是限幅电压控制，不是电流或转矩控制。

## 固定接线和固件行为

| 功能 | STM32 引脚 | Lab 行为 |
|---|---|---|
| IN1 / IN2 / IN3 | PA8 / PA9 / PA10 | TIM1 CH1/2/3，20 kHz 中心对齐 3-PWM |
| 互补输出 | PB13 / PB14 / PB15 | 不配置、不启动 CH1N/2N/3N |
| AS5600 SCL / SDA | PB6 / PB7 | GPIO 开漏软件 I2C；半周期按 200 kHz 配置，写阶段实效约 133 kHz；SDA 三点多数采样；模块 4.7 kΩ 与 MCU 弱上拉到 3.3 V |
| 驱动总使能 | PB10 | 只有编码器、母线、标定和安全状态全部允许时拉高 |
| RESET / SLEEP | PB11 / PB12 | 实际载板引出时连接；启动后先 RESET、再 SLEEP |
| nFAULT / BKIN | PA15 | 低有效，硬件异步关闭 TIM1 MOE |
| 状态 LED | PC6 | 高有效 |
| CAN | PA11 / PA12 | 当前 Lab 不初始化，可不连接收发器 |

Lab 只启动 PA8/PA9/PA10 三路主 PWM。`current_feedback_valid` 始终为 false，MIT/真实转矩路径保持禁止；量产电流采样入口、d/q PI 和有效性联锁仍保留，供以后更换带分流采样的功率板使用。

## 构建、烧录和只读检查

```powershell
.\tools\build.ps1 -Target host -Configuration Debug
.\tools\build.ps1 -Target firmware -Configuration Debug -BoardProfile Lab
.\tools\lab_stlink.ps1 -Action Flash
.\tools\lab_stlink.ps1 -Action Status
```

旧版 ST-LINK Utility CLI 不支持直接烧录 ELF，脚本会使用同目录 HEX 并执行读回校验。它从 ELF 符号表解析状态/命令邮箱地址，不能把某次 map 中的 RAM 地址写死到外部脚本。烧录始终清除 NOLOAD 命令提交字和轨迹，避免重放旧运动；默认也清除校准。只有已经在同一角度约定下验证过零位、且此次仅修改增益等无关代码时，才可显式使用 `-PreserveCalibration` 保留带 guard 的校准记录；若链接后 NOLOAD 符号地址或记录布局改变，旧记录仍会按安全设计失效，必须以刷后 `Status` 的 `ElectricalCalibrated` 为准。

脚本等待状态稳定后会让 GPIO-I2C 进入静默状态，因此正常只读结果至少应满足：

- `Flags` 含 `EncoderBusQuiesced, ThreePwmProfile, CurrentLoopInterfaceAvailable, EncoderSclHigh, EncoderSdaHigh`，不含 `CanEnabled`、`GateArmed` 或 `OutputEnabled`。
- `EncoderHealth` 理想值为 `OK`；`EncoderRaw`、`RawAngleRad`、`EncoderStatusRaw`、`EncoderAgc` 和 `EncoderMagnitude` 保留最后一次有效读数。
- `ActiveFaults` 为 `None`；`DutyA/B/C` 在未使能时均为 `0.5`。
- `CurrentLoopCount / SupervisorLoopCount` 的增长速率约为 20:1，对应 20 kHz/1 kHz。

旧 ST-Link Utility 写 RAM 时会断言复位。为避免在 GPIO 软件 I2C 半字节中途复位，Lab 在取得至少一个有效样本且 gate 关闭、命令空闲/完成后，主动停止发起新的编码器事务并报告 `EncoderBusQuiesced`。此时 `EncoderValid` 会因 8 ms 新鲜度超时而退出，但最后的 `EncoderRaw/EncoderHealth` 仍保留，且不形成位置故障；下一条命令通过 Connect-Under-Reset 启动新实例，必须重新读到有效编码器后才允许 gate armed。运行期间绝不使用静默窗口或缓存样本代替实时角度。

`CurrentLoopInterfaceAvailable` 只表示软件接口被保留；只有 `CurrentFeedbackValid` 才表示真实电流采样可用，而该标志在本 Lab 板上必须保持关闭。

## AS5600 故障判定

| 状态 | 含义和检查 |
|---|---|
| `EncoderHealth: Bus` | I2C 读写失败；检查 AS5600 3.3 V、共地、SCL→PB6、SDA→PB7、上拉和短路。空闲时 SCL/SDA 都应为高。 |
| `NoMagnet` | STATUS.MD 未置位；检查磁钢、轴向气隙、同心度和磁极方向。 |
| `MagnetWeak` / `MagnetStrong` | 磁场超出推荐窗口。3.3 V 下 AGC 范围是 0…128；调气隙/同心度时应让 AGC 离开端点并尽量接近中部约 64，同时让 ML/MH 清零。当前台架仅在健康位恰好只有 `MagnetWeak` 时允许受限 Lab 标定，并显式报告 `WeakMagnetAccepted`。`MagnetStrong` 和任何组合故障仍禁止运行。 |
| `PositionFeedback` 故障 | 编码器无效或超时；gate 保持关闭，必须先恢复有效样本再 `Clear`。 |

Lab 的正常配置是不初始化 I2C1：PB6/PB7 为默认拉高的低速 GPIO 开漏输出，由软件 I2C 完成起始、停止、ACK、时钟延展等待和最多 32 个恢复脉冲；半周期按 200 kHz 配置，写阶段因额外数据保持实效约 133 kHz。SDA 在一个半周期内取三点多数值，降低 PWM 边沿毛刺被误判为位翻转的概率。中断可以拉长 SCL 高电平，不影响协议。AS5600-S 模块原理图上的 R1/R2 为 4.7 kΩ 上拉，MCU 同时启用弱内部上拉。初始化会把易失 CONF 设置为 `SF=11`、`FTH=001`，随后读回相应位验证；设备不应答时以 100 ms 周期重新探测，避免无休止阻塞快速控制。Production 的硬件 I2C1 后端仍保留，并在 HAL 超时后复位外设以清除可能遗留的 `CR2.START`。

带 PWM 的台架会出现瞬态软件 I2C 事务丢失。当前读取路径会立即恢复总线并重读；若仍失败，最多允许后续两个 1 kHz 周期继续重试，期间不刷新样本时间戳，因此 8 ms 编码器 stale watchdog 仍独立生效。连续第三个周期失败才发布 `Bus` 并撤销 gate；NoMagnet/MagnetStrong 等非总线健康故障不使用该容错窗口。`EncoderBusErrorCount/RecoveryCount/FailureCount` 和 `EncoderMaximumAgeUs` 用于量化该过程；8 ms 阈值仍低于 PLL 的 10 ms 重捕获上限，禁止继续放宽。

状态 LED：有效故障时每 100 ms 翻转（5 Hz 完整闪烁）；gate armed 时常亮；编码器有效且未 armed 时每秒亮 100 ms；编码器无效且尚未形成有效故障输出时按 250 ms 明暗（2 Hz）提示。

## 电角度对齐

确认 `EncoderValid` 后执行；本套 2804 实机的已验证电压为 1.2 V：

```powershell
.\tools\lab_stlink.ps1 -Action Calibrate -VoltageLimit 1.2 -TraceCsv tmp\lab-results\calibrate.csv
```

固件执行两遍相互独立的对齐。每遍先保持 0° 电角度 1 s，再用前 60% 时间缓慢扫到 +90°、后 40% 静置，最后以同样方式返回 0°。7 极对电机的预期机械位移为 `π/(2×7)=0.2244 rad=12.86°`；单遍有效窗口是理论值的 55%…130%，方向由位移符号确定。每遍零位取“扫动前 0° 角度”和“回程后 0° 角度”的环形中点，以抵消一阶齿槽/静摩擦回差；回程误差还必须小于理论位移的一半，末端绝对速度必须小于 0.5 rad/s。只有两遍方向一致、两个电角零位的环形差不超过 0.05 rad 时，才取二者环形平均并提交校准。

全过程电压只允许 `0.3…1.2 V`。对本实物，1.0 V 仍可能只移动理论值约 52%，而 1.2 V 已完成双遍校准；按 `Rs=2.55 Ω` 只能粗略估算 1.2 V 静止矢量约为 0.47 A，它不是测得的相电流或转矩。

成功结果应为：

- `State: Complete (8)`、`Result: OK (2)`。
- `Flags` 增加 `ElectricalCalibrated`，结束后不含 `GateArmed/OutputEnabled`。
- `AlignmentDeltaRad` 落入上述窗口，`EncoderDirection` 为 `+1` 或 `-1`。
- 三路 duty 返回 `0.5`，Active/Latched fault 均为 `None`。

`AlignmentNoMotion` 表示单遍位移不足或过大，`AlignmentReturnError` 表示未可靠回到起点，`AlignmentInconsistent` 表示两遍方向或零位不一致。遇到这些结果应先检查三相顺序、驱动 EN/RESET/SLEEP、nFAULT、供电和机械卡滞，不能超过 1.2 V 限制硬顶。若已有合法校准，新校准在全部检查通过之前不会覆盖它；失败后旧的方向/零位和 `ElectricalCalibrated` 标志保持不变。本台架已用 0.3 V 故意触发 `AlignmentNoMotion`，实测旧零位完整保留。

标定值带 magic/guard 保存在 NOLOAD RAM 中，允许跨调试器产生的 PIN/SW reset 恢复，以便旧工具先复位、后写下一条邮箱命令时不丢零位；重新烧录默认清除校准 magic，只有 `Flash -PreserveCalibration` 显式保留。上电/BOR、看门狗、低功耗或 option-byte 复位也会使其失效。命令提交字与校准保留独立校验，并在动作前清零，因此后续复位不会重放运动命令。

## 逐级运动测试

标定成功后按能量从低到高执行，每步先观察状态再继续：

```powershell
# 小 q 轴电压，500 ms
.\tools\lab_stlink.ps1 -Action Voltage -Value 0.15 -VoltageLimit 0.3 -DurationMs 500 -TraceCsv tmp\lab-results\voltage.csv

# 5 rad/s，2 s
.\tools\lab_stlink.ps1 -Action Velocity -Value 5.0 -VoltageLimit 0.8 -DurationMs 2000 -TraceCsv tmp\lab-results\velocity.csv

# 以本次调试复位后的首个有效角度为位置零点，移动 0.25 rad
.\tools\lab_stlink.ps1 -Action Position -Value 0.25 -VoltageLimit 0.3 -DurationMs 3000 -TraceCsv tmp\lab-results\position.csv

# 随时提交停止；也可依赖每条命令的硬时限自动撤销 gate
.\tools\lab_stlink.ps1 -Action Stop
```

脚本会在 Calibrate/Voltage/Velocity/Position/OpenLoop 前先提交一次 `Clear`，并在动作结束且 gate 已撤销后自动转储轨迹。ABI v3 以标称 3 ms 周期保存 384 个样本；缓冲区满后循环覆盖最旧数据，因此长命令保留的是最后约 1.2…1.5 s，而不是启动瞬态。每个样本含时间戳、状态、原始角度、位置、PLL 速度、参考、电压、编码器年龄、角度读取次数和总线错误/失败累计值；头部另存最终状态/结果和最旧样本索引。Fault reset 只有在 nFAULT 已释放、gate 未 armed、编码器恢复且速度接近零时才会成功。电压上限最大 1.2 V、速度目标最大 20 rad/s、位置目标绝对值最大 100 rad、单次运行最长 10 s；越界请求进入 `InvalidRequest`，不会钳位成一个看似合法的运动。命令状态的 `Result: OK` 仅表示硬时限内未触发联锁，不等于目标误差合格；速度稳态应优先用末段位置差/时间计算，不能只看量化噪声较大的单点 PLL 速度。

调试版为兼容旧 ST-Link 工具会在暂停内核时冻结 TIM2/IWDG，但 TIM1 继续切换，避免把已 armed 的相输出冻结成直流电平。仍禁止在 gate armed 时打断点或手工读取 `Status`；脚本的默认等待覆盖最慢的合法对齐路径。若使用 `-WaitForCompletion $false`，必须等待硬时限结束或先执行 `Stop`，再读取状态。

## 2026-08-25 台架记录

- 更换并升级后的 ST-Link 以 SWD 1.8 MHz 稳定完成连接、烧录、读回校验和 RAM 邮箱命令；其 `Target voltage=2.3…2.5 V` 是已知错误上报，本次只采信 SWD 功能结果。每次脚本终态均确认 gate/output 关闭、三相 duty 为 0.5。
- AS5600 始终报告 `MagnetWeak`、`AGC=128`，本轮 `MAGNITUDE` 约为 `1370…1391`。SCL/SDA 空闲高、角度持续可读；PWM 运行中出现过可恢复的软件 I2C 事务错误。最终成功校准的约 5500 次角度读取中有 9 个监督周期耗尽单周期三次事务重试，但后续周期均在 8 ms stale 联锁前恢复，未产生无效轨迹样本或位置故障；后续闭环命令的最终失败增量均为 0，最大样本年龄约 `2.5…3.7 ms`。实机还捕获并立即重读恢复过不可能的角度帧；坏帧没有进入换相或位置跟踪。
- Lab FOC 已改为最新有效、定向后的 AS5600 角度直接换相，PLL 只用于测速；Production MT6701 的预测角路径未改变。低速 PLL 保持 `Kp/Ki=80/1600`，速度评估使用环形轨迹末段的位置差，而不是单个 PLL 瞬时值。
- 旧的单遍、阶跃式对齐受齿槽平衡点影响，电角零位重复性很差。最终双遍慢扫、往返中点和 0.05 rad 一致性检查在 1.2 V 下得到 `EncoderDirection=-1`、`ElectricalZeroRad=-2.867010 rad`。第一次 1.2 V 尝试因单遍位移只有理论值 50.6% 而安全拒绝，第二次通过；0.3 V 故意失败后，已验证旧方向和零位仍完整保留。
- `+2/-2 rad/s`、0.8 V 双向速度测试均完成且无无效样本/最终总线失败。完整 5 s 测试的末段平均曾为 `+2.016/-1.992 rad/s`；最终映像重新校准后的 3 s 冒烟测试为 `+2.029/-1.953 rad/s`，误差约 `+1.4%/-2.3%`，峰值 q 轴电压分别为 `0.651/0.434 V`。不同转子区域的齿槽会改变所需电压，但方向和闭环稳定性一致。
- 位置环加入“转子已朝目标脱离齿槽后清除破静摩擦积分”，抑制破静摩擦后的大幅越过。最终映像的双向约 1 rad、0.8 V、10 s 测试，正/反向终点误差分别为 `0.00169/0.00767 rad`，两者末约 1.50 s 的位置范围均为 `0.00307 rad`（2 个 AS5600 计数），且没有无效样本或最终总线失败。最强齿槽位置的一次 5 s 命令曾停在目标另一侧、误差 `0.0416 rad`，同目标 10 s 命令随后可靠回调并稳定；因此不要把 5 s 硬时限当作本套件全角度范围的位置到达保证。
- 以上结果通过本套件的受限实验电压/速度/位置路径，但不解除硬件边界：SimpleFOC Mini 没有相电流反馈，`MagnetWeak/AGC=128` 仍是磁场裕量告警。它不能据此放行为真实转矩控制、MIT、量产电流环或安全功能。

## 2026-08-24 台架记录

- STM32G431 Rev X (`REV_ID=0x2003`) 已由 ST-Link 识别；Lab HEX 已烧录、读回校验通过。
- 20 kHz 快速循环和 1 kHz 监督循环持续运行；3-PWM 标志有效，PA8/PA9/PA10 为主输出，CAN 未初始化，未 armed 时三相 duty 为 0.5。
- 目标寄存器实读：PA8/PA9/PA10 为 AF，PA11/PA12 与 PB13/PB14/PB15 为模拟态，PC6 为输出；Idle 时 `TIM1_CCER=0x1000`，只有无引脚 CH4 采样触发保持，CH1/2/3 和全部 CHxN 均关闭，PB10 `GATE_ENABLE` 为低。BKIN/fatal 仍可清 MOE。
- `Stop` 命令、ST-Link 写后复位兼容、一次性防重放邮箱、失败标定超时路径均已在目标板验证；编码器无效时标定返回 `Fault (9) / EncoderInvalid (4)`，gate 和输出保持关闭。
- 修正前 AS5600 状态为 `EncoderHealth: Bus`、`PositionFeedback`。暂停内核、关闭 I2C1 并把 PB6/PB7 改为输入后，即使不启用 MCU 内部上拉，两线也都被模块的外部上拉拉高；这排除了 MCU 引脚损坏和持续外部短路。旧 `TIMINGR=0x00F02B86` 在 170 MHz 下接近 Fast-mode Plus 而非注释所述 400 kHz，且 HAL 超时后会遗留 `CR2.START`；即使改用 HSI16/400 kHz，当前目标板的硬件 I2C 路径仍不能可靠释放 SCL。最终 Lab 固件停用 I2C1，改用 PB6/PB7 的开漏软件 I2C；2026-08-25 又根据带 PWM 的恢复测试降至当前保守时序。经台架操作者明确确认供电、机械和现场安全后，当前构建启用仅限 Lab 的弱磁降级许可：健康位必须恰好只有 `MagnetWeak`，状态同时显示 `WeakMagnetAccepted`；其余编码器健康故障不放行。
