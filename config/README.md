# 配置文件说明

`brushless_control_classic_canopen.eds` 是开发期的**经典 CANopen（CiA 301）EDS 绑定模板**，对应 `App/canopen` 的 CANopenNode v4 固定对象字典/PDO，并把驱动对象绑定到 `App/protocol/cia402_od.*`。端口代码存在不等于目标主站互操作或一致性已经通过，更不表示 CiA 1301 CANopen FD 兼容。

该模板描述量产 MT6701 配置，因此 0x6502 bit 31 和 0x2000 MIT record 可见。现有 profile 宏名虽然含 `3SHUNT`，当前固件实际同步采 A/B 两相并重构 C 相；EDS 不能证明电流硬件或校准已经合格。AS5600/SimpleFOC Mini 实验配置必须使用单独 EDS 变体，清除 MIT 支持并移除 0x2000，不能让主站看到一个硬件上无法安全执行的模式。

使用前必须：

1. 用最终固件和目标主站实测 NMT、SDO、PDO、EMCY、Heartbeat 及 FDCAN 错误恢复。
2. 用组织合法获得的 Vendor-ID、Product code 和 revision 替换开发占位值；不能借用第三方编号。
3. 把默认 node-ID、Heartbeat、COB-ID、PDO 映射和范围与产品配置冻结。
4. 逐项比对 EDS 与运行时对象，并用目标主站做互操作测试。
5. 根据正式发布工具更新文件版本、日期和 checksum（若流程要求）。

MIT 记录 0x2000 的多次经典 SDO 写入不是原子的：sub 1…6 写 shadow 且不续期 watchdog，sub 7 的新 sequence 提交当时的完整 shadow。固件不跟踪本批次是否重写了每个字段，遗漏字段会沿用旧 shadow，因此主站必须按序写齐并最后写 sequence。硬实时 MIT 推荐使用文档定义的 32-byte 厂商 CAN-FD 单帧；该单帧明确 **NOT CiA 1301**。

0x1016:1 默认是 0（consumer 禁用），而端口的非零 Controlword 联锁要求 consumer 已配置、收到首个 heartbeat、未超时且远端 NMT 为 Operational。当前未启用参数 storage，因此主站每次上电都要先写 `node_id<<16 | timeout_ms`，再执行 NMT/CiA 402 使能。

驱动 manufacturer fault 保持受 heartbeat 监督的 NMT Operational，以便读取诊断并提交 CiA 402 Fault reset；bus-off/HB consumer failure 才会按 NMT 错误策略退出 Operational。控制器从 bus-off 恢复不自动清 `CAN_BUS_OFF` 软件锁存，必须满足 gate off/低速并收到新的 Fault reset 上升沿，之后仍完整重走 PDS。
