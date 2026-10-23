# DMA 实机验证（2026-10-22）

设备：STM32G431CB / Unit-RollerCAN，160 MHz；ST-Link SWD 1800 kHz。全部验证保持 PB2 低、电机关闭。ST-Link 报告约 0.58 V，但连接、Flash 编程及校验均成功；ADC 测得电机母线约 11.65 V。

## 实现范围

| DMA1 通道 | 用途 |
|---|---|
| 1 | ADC1 四路规则序列，半字循环双帧缓冲，20 kHz |
| 2 / 3 | SPI1 SSC 接收 / 发送，命令一字、应答两字 |
| 4 | TIM1 COM 触发三字 DMAR burst，CCR1/2/3 对应 C/B/A |
| 5 / 6 | CORDIC 两字结果 / 两字参数 |

TIM1 下溢启动本周期编码器事务，ADC 数据就绪后只汇合同周期结果。温度和 VREFINT 注入组没有原生 DMA 请求；FDCAN 没有原生 DMA 请求。初次验证时使用了额外的 PWM 更新窗口门限（向下计数且 CNT≥640），后续按用户要求移除此门限及 DMA 完成后的计数方向检查；以下初次实测数据保留为历史记录。

## 检查结果

- Debug、Release、Bench 编译及 ELF 约束检查通过；Host 的两个 CTest 通过；28 个 Python 工具测试通过。
- 正常 Bench 运行累计超过 150 万帧，SSC 错误计数 0、PWM DMA 错误计数 0、CORDIC DMA 错误计数 0。三组 DMA 成功计数均随采样增加。
- 最终 Bench 空闲测量：ADC 回调间隔 7938–8058 cycles（49.61–50.36 µs）；ADC 回调入口到计数/遥测处理结束的历史最大值为 3477 cycles（21.73 µs）。从回调入口到 `apply()` 返回的最大值为 3119 cycles（19.49 µs）。这是关桥状态的观测值，不是带载最坏情况上限。
- CORDIC 上电 257 角度自检通过，与软件三角函数的最大误差约 `1.13e-6`。
- 参数保存通过正常 CANopen `0x1010:01` 执行，仅保存当时现有设置。采样 epoch 从 1 增至 2，样本继续增加，故障码保持 0；验证了 Flash 暂停、DMA 取消和重新启动。
- 在关桥状态短暂屏蔽 SPI RX DMA 中断，下一周期检测到未完成事务，锁存 `0x5300` 时序故障。恢复中断后，经正常 CANopen 故障复位恢复到故障码 0。此次故障注入产生的 SSC 取消计数属于预期结果。
- 暂停 CPU 并保持关桥，向三字 DMA 源提供 `2200/2000/1800`，触发 COM burst。传输剩余数变为 0，CCR1/2/3 读回对应数值，CCR4 保持 3680；随后复位恢复固件运行。

## 交付状态与边界

设备最终烧录并校验 `build/Debug/canopen_foc.elf`，保留最后 4 KiB 参数区。普通 Debug 固件的样本数持续增长，panic 为 0，SSC/PWM/CORDIC DMA 错误计数为 0，最长 ADC 路径观测值为 3342 cycles（20.89 µs）。

恢复物理 CAN 后，FDCAN 协议状态显示 ACK 错误、Error Passive，控制器报告 `0x8130` 通信故障，PB2 保持低。此次未验证实际 CAN 对端或总线收发；Bench 的 SWD 邮箱不覆盖物理 CAN。此次也未进行电机使能、旋转或带载测试。编码器角度在本周期电流采样之前读取，当前没有角度外推。

原始本地记录：`.work/dma-bench-validation.json`、`.work/dma-pwm-transfer.json`、`.work/dma-final-bench.json`、`.work/dma-final-debug.json`。`.work` 不作为仓库交付文件。

## 后续移除 PWM 更新窗口门限

按用户要求删除 `Board::apply()` 中提交前的向下计数/CNT≥640 检查，以及 DMA 完成后的计数方向检查。Debug、Release、Bench 重新构建及 ELF 约束检查通过。此次未完成烧录：系统未枚举到 ST-Link，OpenOCD 返回 `open failed`；上述初次实测记录不代表移除门限后已完成实机复测。
