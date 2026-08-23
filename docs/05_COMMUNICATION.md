# CiA 402 与 CAN-FD 通信契约

## 三个必须区分的层次

| 层次 | 本工程状态 | 能否称为 CANopen FD |
|---|---|---|
| CiA 402 对象与 PDS 行为 | `App/drive` 和 `App/protocol/cia402_od.*` 提供可移植状态机/对象镜像 | 不能；CiA 402 是设备配置文件，不是总线传输栈 |
| 经典 CANopen | 已集成 CANopenNode v4 的 STM32 FDCAN 经典帧端口、NMT/Heartbeat/EMCY/SDO/PDO、固定 OD 与 CiA 402 回调；仍待目标主站实机互操作和适用的一致性测试 | 不能；它是 CiA 301 经典 CANopen，即使底层控制器名称叫 FDCAN |
| CiA 1301 CANopen FD | 当前仓库未实现 USDO 和完整 CiA 1301 服务 | 只有接入合法、完整、经验证的 CiA 1301 栈后才能声明 |
| 厂商 MIT 快速通道 | 已实现固定 32/48-byte CAN-FD 编解码、CRC、范围和序号守卫 | **不能；明确为 NOT CiA 1301** |

FDCAN 参考时序是 500 kbit/s 仲裁段、2 Mbit/s 数据段、BRS。经典 CANopen 帧仍必须以 Classic CAN 格式发送，最大 8 byte；厂商快速通道使用 CAN-FD+BRS。两者共用一个物理网络前，必须确认所有节点都能容忍 CAN-FD 帧。

## CiA 402 对象镜像

所有对象边界读写显式使用小端序。下表已经通过 `App/canopen` 的 OD extension 接到经典 CANopen SDO；只有标明并列在固定映射中的对象可走 PDO，且访问仍受 NMT、heartbeat、PDS 状态和应用范围检查约束。

| Index | Sub | 类型 | 访问 | 本工程单位/语义 |
|---:|---:|---|---|---|
| 0x603F | 0 | UNSIGNED16 | RO | Error code；需由故障映射层发布 |
| 0x6040 | 0 | UNSIGNED16 | RW | Controlword |
| 0x6041 | 0 | UNSIGNED16 | RO | Statusword |
| 0x6060 | 0 | INTEGER8 | RW | Modes of operation；每次合法写（含同值）推进 mode-instance epoch；Operation enabled、`torque_enable=true` 或非法写拒绝且不推进 |
| 0x6061 | 0 | INTEGER8 | RO | Modes of operation display |
| 0x6064 | 0 | INTEGER32 | RO | Position actual，Q16.16 rad |
| 0x606C | 0 | INTEGER32 | RO | Velocity actual，Q16.16 rad/s |
| 0x6071 | 0 | INTEGER16 | RW | Target torque，额定转矩千分比 |
| 0x6077 | 0 | INTEGER16 | RO | Torque actual，额定转矩千分比 |
| 0x607A | 0 | INTEGER32 | RW | Target position，Q16.16 rad |
| 0x6081 | 0 | UNSIGNED32 | RW | Profile velocity，Q16.16 rad/s，必须大于 0 |
| 0x6083 | 0 | UNSIGNED32 | RW | Profile acceleration，Q16.16 rad/s²，必须大于 0 |
| 0x6084 | 0 | UNSIGNED32 | RW | Profile deceleration，Q16.16 rad/s²，必须大于 0 |
| 0x60FF | 0 | INTEGER32 | RW | Target velocity，Q16.16 rad/s |
| 0x6502 | 0 | UNSIGNED32 | RO | bit 0=PP、bit 2=PV、bit 31=厂商 MIT；bit 31 不是标准模式位 |
| 0x2000 | 0…7 | RECORD | RW/RO | 厂商 MIT 命令记录，见下表 |

0x2000：

| Sub | 名称 | 类型 | 单位 |
|---:|---|---|---|
| 0 | Highest sub-index | UNSIGNED8, RO | 固定 7 |
| 1 | Desired position | INTEGER32 | Q16.16 rad |
| 2 | Desired velocity | INTEGER32 | Q16.16 rad/s |
| 3 | Feed-forward torque | INTEGER32 | Q16.16 N·m |
| 4 | Kp | UNSIGNED16 | UQ8.8 N·m/rad |
| 5 | Kd | UNSIGNED16 | UQ8.8 N·m·s/rad |
| 6 | Current limit | UNSIGNED16 | UQ8.8 A |
| 7 | Sequence | UNSIGNED16 | 单调递增提交序号 |

0x2000 的逐字段 SDO 写入不是原子命令。当前适配器把 sub 1…6 分别做范围检查并写入 shadow，这些写入本身不刷新 MIT watchdog；sub 7 写入一个变化的 sequence 才把当时的完整 shadow 快照作为命令提交并续期 watchdog。固件不会判断主站是否在本批次重写了全部 sub 1…6，遗漏字段会沿用上次 shadow 值，因此主站必须按顺序写齐所需字段并最后写 sequence。更适合硬实时 MIT 的方式是使用后述厂商 32-byte 单帧。

### Controlword / Statusword

常用 Controlword 模式：

| 命令 | `(cw & mask)` 期望 | 常见值 |
|---|---|---:|
| Shutdown | bit 1=1, bit 2=1, bit 0=0 | 0x0006 |
| Switch on | bit 0/1/2=1, bit 3=0 | 0x0007 |
| Enable operation | bit 0/1/2/3=1 | 0x000F |
| Quick stop | 清 bit 2 | 例如 0x000B |
| Fault reset | bit 7 新的上升沿 | 0x0080 |

基础 Statusword 状态码：

| PDS 状态 | 基础 Statusword |
|---|---:|
| Not ready to switch on | 0x0000 |
| Switch on disabled | 0x0040 |
| Ready to switch on | 0x0021 |
| Switched on | 0x0033 |
| Operation enabled | 0x0037 |
| Quick stop active | 0x0017 |
| Fault reaction active | 0x000F |
| Fault | 0x0008 |

bit 4 `Voltage enabled` 来自物理 gate feedback，而不是软件刚写出的 EN 值；bit 7 warning、bit 9 remote、bit 10 target reached、bit 11 internal limit、bit 12/13 模式相关。若软件请求关闭而 gate feedback 仍为开，状态机锁存 gate interlock fault。

Fault reset 只接受新的 bit 7 上升沿，并且只有故障原因已消失、功率级物理关闭且安全层判定可复位时才清锁存。复位不会自动恢复运动。

## 厂商 CAN-FD MIT 快速通道

### CAN ID

当前参考使用 29-bit 扩展 ID：

```text
command  = 0x18FF1000 | node_id
feedback = 0x18FF1800 | node_id
node_id  = 1…127（默认 1）
```

默认节点 1 即 `0x18FF1001` / `0x18FF1801`。这是项目内临时分配，不是 CiA 预定义 COB-ID，也不是 J1939 PGN 声明；进入现有网络前必须做全网 ID 冲突评审。接收过滤器只应接受本节点 command ID，所有 remote frame 拒绝。

### 32-byte command（CAN-FD DLC=13）

| Byte | 长度 | 字段 | 格式 |
|---:|---:|---|---|
| 0 | 1 | magic | 固定 `0x4D` (`M`) |
| 1 | 1 | version | 固定 `1` |
| 2 | 1 | type | 固定 `0x10` |
| 3 | 1 | flags | bit0 enable、bit1 quick stop、bit2 clear fault、bit3 zero position；其余必须 0 |
| 4 | 2 | sequence | uint16 little-endian |
| 6 | 4 | sender timestamp | uint32 µs，仅诊断 |
| 10 | 4 | desired position | int32 Q16.16 rad |
| 14 | 4 | desired velocity | int32 Q16.16 rad/s |
| 18 | 4 | feed-forward torque | int32 Q16.16 N·m |
| 22 | 2 | Kp | uint16 UQ8.8 N·m/rad |
| 24 | 2 | Kd | uint16 UQ8.8 N·m·s/rad |
| 26 | 2 | current limit | uint16 UQ8.8 A |
| 28 | 1 | mode | int8：codec 识别 1=PP、3=PV、-128=MIT；当前产品应用层只接受 -128=MIT |
| 29 | 1 | reserved | 必须 0 |
| 30 | 2 | CRC | CRC-16/CCITT over byte 0…29，小端存放 |

CRC 参数：初值 `0xFFFF`，多项式 `0x1021`，逐字节 MSB-first，不反射，无额外 final xor。

接收端只有在 magic/version/type/flags/reserved/CRC、mode、全部范围和 sequence 检查得到可提交结果后，才用 1 MHz TIM2 记录 `local_rx_timestamp_us`。当前应用层随后只提交 MIT：PP/PV 即使通过公共 codec，也会锁存 protocol fault；PV/PP 必须走上面的经典 CANopen OD/PDO/SDO 路径。发送方时间戳不能决定新鲜度，因为两端时钟未必同步。TIM2 又由 DWT 以 2 ms 窗口交叉监测；时基停摆会关 gate/停喂 IWDG，而不是让旧命令无限保持新鲜。

sequence 的差值按 uint16 模运算：`delta=0` 为重复，`delta>=0x8000` 为旧包，超出配置的最大向前跳为 gap；正常 `0xFFFF→0x0000` 被接受。重复、旧包和 CRC/字段错误会被拒绝且不刷新 watchdog。gap 被记录为诊断结果，但因为该帧本身已经完整通过 CRC/范围校验，当前实现会提交这条最新命令、更新 sequence 并刷新 watchdog；主站不得把 gap 当作无损通信，量产测试必须统计并对持续 gap/bus error 设系统级告警策略。

### 48-byte feedback（CAN-FD DLC=14）

| Byte | 长度 | 字段 | 格式 |
|---:|---:|---|---|
| 0 | 1 | magic | `0x4D` |
| 1 | 1 | version | `1` |
| 2 | 1 | type | `0x11` |
| 3 | 1 | flags | bit0 operation enabled、bit1 warning、bit2 target reached、bit3 command fresh、bit4 derated |
| 4 | 2 | feedback sequence | uint16 |
| 6 | 2 | accepted command echo | uint16 |
| 8 | 4 | local timestamp | uint32 µs |
| 12 | 4 | position | int32 Q16.16 rad |
| 16 | 4 | velocity | int32 Q16.16 rad/s |
| 20 | 4 | applied torque | int32 Q16.16 N·m |
| 24 | 4 | measured `Iq` | int32 Q16.16 A |
| 28 | 4 | bus voltage | uint32 UQ16.16 V |
| 32 | 2 | Statusword | uint16 |
| 34 | 1 | mode display | int8 |
| 35 | 1 | PDS state | 0…7 |
| 36 | 4 | active faults | uint32 bitmask |
| 40 | 4 | latched faults | uint32 bitmask |
| 44 | 2 | torque limit | uint16 Q15；32767=100% |
| 46 | 2 | CRC | CRC-16/CCITT over byte 0…45 |

反馈频率必须计入总线负载。主站用 command echo 检测闭环延迟和丢包，但不能因为收到反馈就假定执行了 enable；仍须同时检查 Statusword、PDS state、fresh、faults 和 operation-enabled flag。

### 并发与超时

- FDCAN ISR 完成帧验证后写入单槽邮箱；1 kHz 任务原子取得一致快照。
- 经典 CANopen 与厂商 CAN-FD 共用 FDCAN1 的 Tx message RAM，并通过同一非阻塞发送锁串行化 HAL 入队；拿不到锁或 FIFO 已满时不得并发写 message RAM。两通道的 busy/drop/retry 与最坏延迟仍须在目标负载下分别统计，发送锁不等于带宽已经充足。
- 收到 quick-stop 标志立即使运动任务进入受控停止；硬件故障路径不等待 CAN 任务。
- MIT 默认 5 ms 未收到有效 command 进入软失效/零转矩；20 ms 进入硬失效并撤销 gate。具体数值必须按系统失控距离验证。
- HAL ErrorStatus 检测到 bus-off 后立即锁存 `CAN_BUS_OFF`，并清 M_CAN `INIT` 以启动控制器的 ISO bus-off 恢复过程；这一步只恢复物理控制器参与总线的能力，不会清除软件安全故障，也不会继续执行旧命令。
- 只有控制器 PSR 已确认不再处于 bus-off、gate 反馈为关闭、转速进入复位容差，并收到**新的** Fault reset 上升沿时，才允许清除 bus-off 软件锁存。之后仍须重建命令序号/heartbeat 并完整走 PDS 使能序列；仅仅总线重新出现有效帧不得恢复转矩。
- clear-fault 只在功率关闭、故障原因已消失并检测到新的请求上升沿时接受。
- zero-position 同样只响应 bit 3 的新上升沿，并且必须同时满足 gate 已失能、PDS 不在运行态或正处于 Quick stop、速度绝对值不大于零位容差。若某个上升沿因条件不满足被拒绝，主站必须先把 bit 3 释放为 0，再产生新的上升沿；保持请求为 1 不会在条件后来满足时延迟执行。

## 经典 CANopen 当前实现

`App/canopen` 将 CANopenNode v4 绑定到 FDCAN1，但只接收/发送 11-bit、0…8-byte 的 Classic CAN 帧，固定 500 kbit/s。对象和栈实例使用静态存储，不在实时路径分配内存。默认 node-ID 是 `1`，由 `BOARD_CANOPEN_NODE_ID` 在 1…127 范围内覆盖。

已编入的服务：

- NMT slave、boot-up 和 NMT 状态回调。
- Heartbeat producer：0x1017 默认 100 ms；首次 heartbeat 延迟配置为 250 ms。
- 一个 Heartbeat consumer：0x1016:1 默认 `0`，即禁用。经典 CANopen 要允许非零 Controlword，主站必须先把它配置为监督节点/超时，并让端口收到该远端的第一个 Operational heartbeat。
- EMCY producer 和 8 项 pre-defined error history。
- 一个 segmented SDO server，内部 buffer 32 byte。
- 两个固定 RPDO、两个固定异步 TPDO；未启用 SYNC、LSS、SDO client 和参数 storage。
- OD extension 将驱动对象读写交给 `cia402_od_read/write` 做长度、范围、只读和状态检查。

调度契约：`app_init()` 以 node-ID/500 kbit/s 初始化端口；1 kHz PendSV 在运动监督前处理 RPDO/TPDO；main background 用 TIM2 计算的真实 `elapsed_us` 持续调用 `CO_process()`，处理 NMT、SDO、Heartbeat 和 EMCY。background 不是可选任务：若它停止，Heartbeat consumer 超时和 NMT 状态变化将不能按期处理，IWDG 健康聚合必须覆盖这一活性。

非零 Controlword 只有在以下条件全部成立时才接受：本地 NMT 为 Operational；0x1016:1 已配置；已收到受监督节点的第一个 heartbeat；heartbeat 未超时；远端上报的 NMT 状态也是 Operational。`0x6040=0` 始终允许用于安全撤销。因为 0x1016 默认禁用且当前没有参数 storage，主站每次启动都必须先配置 consumer（32-bit 值为 `node_id<<16 | timeout_ms`），再启动/确认远端 Operational，之后才能走 CiA 402 使能序列。

本地或远端 NMT 离开 Operational、Heartbeat consumer 超时或 communication reset 都通过验证过的应用路径强制写 `0x6040=0`；NMT reset-node 先执行 fatal gate-off，再请求 NVIC 系统复位。恢复到 Operational 不会自动恢复旧转矩命令。

上述 heartbeat/NMT 联锁只约束经典 CANopen OD 路径；厂商 CAN-FD MIT 快速通道使用自己的 CRC/序号及 5 ms/20 ms 本地 watchdog。0x1016 默认未配置时不会周期性清除一个仍由厂商通道合法维持的命令，但 heartbeat 也不能替代厂商命令新鲜度。系统若要求两条通道互斥或共同授权，必须在上层明确命令所有权并验证切换，不能把当前共存机制解释为仲裁协议。

安全故障聚合到经典 CANopen 时，端口使用 EMCY error status bit `0x40`，并置 0x1001 manufacturer-specific bit `0x80`；EMCY error code 与应用发布的 0x603F 相同，若存在非零故障 mask 但 code 为 0，则回退 `0xFF00`。EMCY `infoCode` 携带完整 32-bit fault mask。mask/code 改变时先 reset 旧报文再 report 新报文，故障清零时 reset；主站仍应读取 active/latched 诊断，不能只凭单个 EMCY 判断机械已安全。

驱动 manufacturer fault 的 error status bit `0x40` 故意不纳入 NMT error-register 自动停机掩码：PDS Fault 已经撤销转矩，节点保持在受 heartbeat 监督的 NMT Operational，主站才能继续读取诊断并写 CiA 402 Fault reset。只有 bus-off 或 Heartbeat consumer failure 通过 `CO_NMT_ERR_ON_BUSOFF_HB` 使本地 NMT 退出 Operational；这不改变任何驱动故障必须先安全清除、再接受新 reset 边沿的规则。

### 固定 PDO

| PDO | COB-ID | 长度 | 固定映射 | 发送条件 |
|---|---|---:|---|---|
| RPDO1 | `0x200 + node` | 2 byte | 0x6040:00 Controlword, 16 bit | 异步收到即处理 |
| RPDO2 | `0x300 + node` | 8 byte | 0x607A:00 target position, 32 bit；0x60FF:00 target velocity, 32 bit | 异步收到即处理 |
| TPDO1 | `0x180 + node` | 5 byte | 0x6041:00 Statusword, 16 bit；0x6061:00 mode display, 8 bit；0x603F:00 error code, 16 bit | transmission type 254，event 10 ms |
| TPDO2 | `0x280 + node` | 8 byte | 0x6064:00 position actual, 32 bit；0x606C:00 velocity actual, 32 bit | transmission type 254，event 20 ms |

0x6060 mode 与 0x6071 target torque 都是 SDO-only。0x6060 由应用状态检查，只在模式受支持、PDS 不为 `Operation enabled` 且已发布 `torque_enable=false` 时允许修改；每次成功写都会递增 32-bit mode-instance epoch，同值重写也代表显式新实例。带转矩的 Quick-stop Active/Fault reaction 仍会返回状态冲突，必须等受控停机真正撤销 torque；长度/值非法或上述状态冲突时，模式和实例都不改变。1 kHz mode manager 对 torque-active 再做防御性拒绝，避免尚未应用的实例在转矩阶段切换。0x6071 保留运行时范围检查，但不额外要求失能，也没有映射为 MIT 命令。RPDO1 故意只放 Controlword：CANopenNode 对多对象 RPDO 逐项调用 OD extension，若后一个对象被状态检查拒绝，无法回滚前一个已经提交的对象；因此不得把“同一个 PDO”泛化为事务原子性。

固定 RPDO2 是针对此固定映射实现的受控例外。`CO_process_RPDO()` 期间，适配器先把 0x607A staging；只有随后收到同一完整 RPDO2 的 0x60FF，且长度、位置范围和速度范围全部有效，才在短临界区批量发布两个值，并同时递增 position/velocity target epoch。速度 generation 同时绑定提交时的模式数值与 mode-instance epoch；位置值在以后 bit 4 edge 形成 PP request 时再与 edge 的模式实例一起冻结。缺字段、重复/顺序异常、长度错误或任一范围错误都会拒绝该 record、记录 protocol/rejection 诊断，且控制应用镜像和两个 target epoch 都保持原值。这个保证仅适用于固定 RPDO2；通过 SDO 分别写 0x607A/0x60FF 仍是两次独立事务，每次成功写各自增加 target epoch，0x60FF 同样绑定当时 mode instance。

0x6040 bit 4 的 `0→1` 写入边沿会在 OD 层立即建立 PP request epoch，并冻结当时已提交的 0x607A value/position target epoch、bit 5/6 选项、mode 与 mode-instance epoch。1 kHz 模式管理器只接受 request instance 与 active instance 相等的 PP 命令，并在任何队列/范围判断前消费 request epoch 及其尚未消费的位置 epoch，所以晚到目标不能补救，持续高 bit 4 不能重试，接受或拒绝过的目标也不能重放；相对旧增量不会在新的基准位置上再次执行。

1 kHz 管理器把“模式数值变化 **或** mode-instance epoch 变化”都当作模式实例转换，清 PP active/pending/relative base 与 PV active target。保留 consumed target epochs 只是附加防重放，不能单独构成模式屏障。PV 只锁存新 velocity target epoch，并要求其 mode=PV 且 target mode instance 等于 active instance；数值相同的新 0x60FF 仍是一次刷新，没有新写入时只保持当前实例最后合法的 PV 目标。保持目标始终受前述 Heartbeat/NMT permission 与强制 `0x6040=0` 路径约束。

这个实例模型仍允许 `0x6060→target→bit 4 edge`（PV 无 edge）全部在第一次 1 ms supervisor 前完成：管理器先切到该实例、清旧状态，再消费同一实例的新命令。相反，同 tick 切离再重入后只保留最终数值/新实例；旧 PP request 或 PV target 因实例不匹配而拒绝。命令形成后再同值写 0x6060 也会产生新实例并使旧命令失配。实例采用相等/不等判断，`UINT32_MAX→0` 正常形成新实例。

当前 CANopenNode 实例按创建顺序在一次 `CO_process_RPDO()` 调用中处理 RPDO1 后处理 RPDO2。两帧若同时 pending，RPDO1 的 bit 4 会先快照旧 position epoch；随后原子提交的 RPDO2 不会追溯修改已经形成的请求，监督器会 fail closed 拒绝该 PP set-point。主站的安全序列是：

1. 先用 RPDO1 令 bit 4 为 0，并等待 Statusword bit 12 清零；若本来已低可省略重复写。
2. 发送完整 RPDO2，并至少跨过一次节点 1 ms RPDO 处理周期。
3. 再发送含所需 bit 5/6 且 bit 4 为 1 的 RPDO1，等待 bit 12 acknowledge。

CAN 帧的线上先后、仲裁优先级或同一主站循环内的发送顺序，都不能替代步骤 2 的节点处理屏障。

厂商 0x2000 MIT record 不能在一个经典 8-byte PDO 中原子承载，也未映射进经典 PDO。其 sub 1…6 作为持久到下次覆盖的 shadow，sub 7 新 sequence 才提交当前快照并续期 MIT watchdog；实时 MIT 推荐使用厂商 32-byte CAN-FD 单帧。

### 上线前仍需的协议证据

实现完成不等于互操作或认证完成。产品上线前仍须：

1. 用目标主站导入 EDS，逐项比对运行时 OD 的类型、访问、默认值、PDO 和 abort code。
2. 验证所有 NMT 状态、boot-up、100 ms producer、0x1016 未配置时拒绝使能、首个远端 heartbeat/远端 Operational 门槛、consumer timeout、EMCY/history、segmented SDO、固定 PDO 周期和 reset-communication/reset-node；确认普通驱动故障保持 NMT Operational 以便诊断/复位，而 bus-off/HB consumer failure 才按配置退出 Operational。
3. 在总线压力、bus-off、错误被动、丢 heartbeat 和 background 延迟下测量 gate-off 时间；验证控制器的 ISO 恢复与软件锁存相互独立，并证明没有新的 Fault reset 上升沿时不能再次使能。不能只观察 CANopen 状态变量。
4. 对固定 RPDO2 做字段丢失/错误长度/任一字段越界/重复与处理延迟注入，证明失败时位置、速度及两个 target epoch 都不变；再覆盖 RPDO1/RPDO2 同轮 pending、RPDO2→处理屏障→RPDO1、晚到目标、重复高 bit 4、已消费目标和相对旧增量，确认只执行明确的新 generation。
5. 验证每次合法 0x6060 写（含同值）只推进一次 mode-instance epoch；非法长度/模式、`Operation enabled`，以及仍发布 `torque_enable=true` 的 Quick-stop Active/Fault reaction 都拒绝且不推进。待 torque=false 且不处于 Operation enabled 后才应接受。再覆盖模式数值变化、同值实例变化、同 tick 切离/重入、命令后同值重写、mode/target/edge 首次 supervisor 前同 tick 合法序列，以及 mode-instance `UINT32_MAX→0`。PP request、PV/固定 RPDO2 velocity 必须只在实例匹配时执行，旧实例 generation 不重放。
6. 覆盖 position/velocity/request target epoch `UINT32_MAX→0`、相同数值的新 velocity epoch 与无更新保持；证明旧目标 generation 不重放，且 Heartbeat/NMT 失效即使 PV 目标保持也会撤销 Controlword。
7. 用组织合法获得的 Vendor-ID/产品身份替换开发占位值，并决定 node-ID、持久化和现场配置策略。当前端口未启用 LSS/storage。
8. 执行适用的 CiA 301/CiA 402 一致性和目标主站互操作测试，并归档 CAN trace。

`config/brushless_control_classic_canopen.eds` 是与当前固定 OD/PDO 对齐的开发模板，不是测试证书。

## CiA 1301 集成边界

如果需求明确是标准 CANopen FD，而不仅是“使用 CAN-FD 硬件”，应替换/扩展通信层：

- 选择具有 CiA 1301、USDO、PDO 和网络管理完整实现的协议栈并确认许可证。
- 用其 XDD/对象生成工具绑定 CiA 402 对象，保留厂商 MIT 对象的明确命名空间。
- 由栈控制 CAN-FD 帧、服务状态和超时，不能把经典 CANopenNode 端口改成 64-byte 帧后自行命名为 CiA 1301。
- 按 CiA 测试中心方法做 CANopen FD 设备一致性测试，并另做 CiA 402 驱动互操作测试。

任何产品手册、EDS/XDD 和 CAN 抓包说明都必须如实标明：经典 CANopen、CiA 1301 或厂商 CAN-FD，三者不能混称。
