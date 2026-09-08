# 硬件约定

目标板：Unit-RollerCAN V1.0，MCU STM32G431CBU6（128 KiB Flash / 32 KiB SRAM）。这里的引脚与板级电路来自原理图，并交叉检查本地 `m5-rollercan` 的引脚定义；没有沿用其电机控制或通信逻辑。

## 引脚

| 功能 | 引脚 | 配置 / 语义 |
|---|---|---|
| 相 A PWM | **PA10 / TIM1_CH3** | DRV8311 INA |
| 相 B PWM | **PA9 / TIM1_CH2** | DRV8311 INB |
| 相 C PWM | **PA8 / TIM1_CH1** | DRV8311 INC；不能按 CH1=A 接线 |
| 三相桥使能 | PB2 | 三个 INL 共用；低电平使桥为高阻 |
| 驱动唤醒 | PB1 | nSLEEP；高电平唤醒 |
| 驱动故障 | PB0 / EXTI0 | nFAULT，低有效，中断优先级 1 |
| 相 A/B/C 电流 | PA1 / PA2 / PA3 | ADC1_IN2 / IN3 / IN4，注入 rank 1/2/3 |
| 母线电压 | PA0 | ADC1_IN1，注入 rank 4 |
| 编码器 CS | PA4 | 低有效 GPIO，空闲高 |
| 编码器时钟 | PA5 / SPI1_SCK | 5 MHz，CPOL=0、CPHA=1 |
| 编码器 DATA 读 | PA6 / SPI1_MISO | SSC 共享 DATA 线 |
| 编码器 DATA 写 | PA7 / SPI1_MOSI | 经 2.2 kΩ 接 DATA，读阶段改成输入高阻 |
| CAN RX/TX | PA11 / PA12 | FDCAN1，经典帧 1 Mbit/s |
| CAN 收发器待机 | PB4 | 高电平待机，启动 CAN 后拉低 |
| 用户按钮 | PC6 | 低有效，按下关桥并触发故障 |
| SWD | PA13 / PA14 | 保留调试接口 |

板上没有外部晶振，使用 HSI + PLL 得到 160 MHz。I²C、OLED、RGB 相关引脚没有应用驱动，未使用的引脚由 CubeMX 设为模拟输入。

## 功率级与采样

DRV8311H 的 MODE 悬空，使用 **3PWM**；GAIN 悬空（R11 未装），电流输出尺度为 **1 V/A**。母线分压 **R18=12 kΩ、R19=2.2 kΩ，比例 71/11≈6.454545**。2026-09-08 实机核对修正了此前将 R18 误读为 22 kΩ 的错误。

```text
I_phase[A] = (current_adc - offset_adc) × VDDA / 4095 / (1 V/A)
Vbus[V]   = bus_adc × VDDA / 4095 × (12000 + 2200) / 2200
```

上电保持 PB2 低，采集 1024 个样本确定三个零点，约 51.2 ms。零点必须在 1700–2400 count 内，峰峰值不超过 100 count；超出后锁存配置故障。VDDA 由 VREFINT 工厂校准值计算，并周期更新。实机静止正 α 电压矢量下，A 相 ADC 高于零点、B/C 低于零点，确认以上电流方向；正负 Iq 跟踪进一步验证了闭环符号。电流增益的绝对精度仍需外部电流表校验。

TIM1 为 20 kHz 中心对齐 PWM，ARR=4000、PSC=0、RCR=1。CH4 是内部 PWM2 比较输出，不占外部引脚，CCR4=3680，OC4REF 上升沿触发 ADC 注入序列。ADC 使用 PCLK/4=40 MHz，四路各采样 24.5 周期、转换 12.5 周期。

占空比限于 10%–90%。极限 90% 时，三个低侧同时导通的窗口约为周期内 22.5–27.5 µs；触发点约 23.0 µs，第三相采样保持结束约 25.46 µs，四路转换完成约 26.7 µs。这个计算为驱动传播和模拟建立留下余量，实际仍应示波器确认。更改 PWM 频率、占空比限制、ADC 时钟或采样时间时必须一起重新计算。

PB2 是最终关桥手段。驱动内部完成 MOSFET 死区，本板不是六路互补 PWM 接法；TIM1 BKIN 未接 nFAULT，因此不能声称 MCU 具有由 BKIN 实现的硬件周期限流。nFAULT 依靠 DRV8311 自身保护以及 EXTI 软件快速关桥。

## 编码器

SSC 按 TLI5012B E1000 处理：16 位 SPI word、模式 1、5 MHz。命令 `0x8021` 读取角度，再读 safety word；15 位角度换算为弧度。使用 CRC-8，poly=0x1D、init=0xFF、xorout=0xFF，覆盖命令和角度的高字节优先序列；同时检查状态位及全 1 断线值。

发送命令后 PA7 转输入，PA6 接收应答；完成后恢复 PA7 AF。DATA turnaround 为 200 ns，满足手册的 130 ns 最小值；保留 CS setup 和下一帧间隔。CRC 使用编译期生成的 256 字节查表，仍完整检查命令、角度和状态。每个 word 轮询最多 10 µs；超时关桥并进入故障。启动及显式故障复位时读 `0x8001` STAT 应答编码器的复位标志，下一次角度读取仍必须通过 CRC。

角度为单圈绝对值；运行时软件展开多圈，掉电后不能恢复先前圈数。编码器跳变故障复位只恢复到当前可测角度，丢失的完整圈数无法推断，位置主站需重新建立位置参考。

## 限值与待标定项

软件默认电流目标上限 0.25 A，参数最多 0.5 A；瞬时相电流故障阈值 0.75 A。母线允许 4.5–16 V，MCU 内部温度达到 85°C 关桥。电流传感器并不等于母线电流测量；MCU 内温不等于电机或驱动芯片温度。

电机按产品资料的 3504 / 200 KV 配置。`Kt=60/(2π×200)≈0.04775 Nm/A` 只是 KV 换算估值，不能视为测得的轴端转矩系数。极对数默认候选 7，但 `calibrated=false`，必须运行校准。校准不负责辨识电阻、电感、惯量或速度环增益。

## 资料

- [M5Stack Unit-RollerCAN](https://docs.m5stack.com/en/unit/Unit-RollerCAN)：模块与电机规格。
- [TLI5012B E1000 数据手册](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/products/unit/Unit-Roller485/TLI5012BE1000.pdf)：SSC、电气时序和 safety word。
- [DRV8311H 数据手册](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/products/unit/Unit-Roller485/DRV8311HRRWR.pdf)：3PWM 模式及 CSA。
- 用户提供的 `sch_Unit-RollerCAN_V1.0.pdf`，第 1 页：相序、共用 INL、GAIN/MODE、ADC 分压、DATA 电阻连接。
- 用户提供的 `Unit-RollerCAN-CAN-Protocol-CN.pdf`：用于确认原厂私有协议与本项目 CANopen 的区别。原始 NAS 文档不重新分发到开源仓库。
