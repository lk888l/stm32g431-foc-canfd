# STM32G431 · DRV8316C · MT6701 FOC / CAN-FD

面向 simple-foc v2.0 驱动板和 4015 电机的 C++23 固件。MCU 为 STM32G431CBU6，主频 160 MHz，20 kHz 电流环、1 kHz 运动环、FreeRTOS 静态任务和 CANopenNode。支持单圈/多圈位置、速度、力矩和 MIT 模式。

已实现速度、单圈/多圈位置、力矩及 MIT 模式。2026-11-17 增加有期限的电流零点检查、三方向电流响应检查、完整机械圈双向电角校准和 SWD 验收脚本，并修复驱动唤醒期间 PI 积累及 DWT 回绕问题。通信超时、单帧编码器错误、采样窗口不足或一次控制延期仍保持原有诊断策略。

SVPWM 使用完整线性范围 `|Vdq| <= Vbus / sqrt(3)`，占空比 0～100%，不再固定限制为 4 V 或 10%～90%。DRV8316C 是低侧电流检测；近饱和区采用有效相重建和模型预测，**满电压利用率不等于每周期都能直接测到三相电流，也不等于已经实测验证的带载性能**。

## 当前板上固件

2026-11-18 的 Bench 固件按整数编码器计数求位置误差，并用双向低速轨迹细调了周期电流表。实机净转 10007.11 圈后完成单计数相对步进测试。低速波动、MIT 静差、换向响应和 ±900 rpm 同周期电流峰值见[第三轮台架记录](docs/motor-response.md)。

2026-11-17 第二轮实机优化采用 1 ms 差分窗 + 0.25 ms 低通测速、随速度调整的速度 PI、20 kHz 周期电流补偿及高速淡出，并将电流 PI 标称整定频率提高至 800 Hz。已完成 ±840 rpm（0.30 A 电流目标限幅）和 ±900 rpm（0.50 A 限幅）的短时空载测试；900 rpm 接近当前约 19.3 V 供电的电压余量边界，实际电流峰值不等于目标限幅。低速、整圈位置和各模式的对比及限制见 [第二轮性能记录](docs/motor-performance.md)。

本台电机的 1024 点周期补偿表默认启用，≤1 rev/s 全量、1～6 rev/s 渐退、≥6 rev/s 为零，总电流目标始终限幅；更换电机或编码器安装后须重新测量。电角校准和 Flash 参数保持原值。当前板上保留 Bench，测试结束后电机停止、PB10 低、三相 PWM 关闭。首轮校准及补偿记录见 [电机调试记录](docs/motor-commissioning.md)，此前 Bringup 记录见 [v2 验证记录](docs/v2-validation.md)。

## 构建

```powershell
./tools/build.ps1 -Preset Host
./tools/build.ps1 -Preset Bringup
./tools/build.ps1 -Preset Bench
./tools/build.ps1 -Preset Debug
./tools/build.ps1 -Preset Release
```

| 构建 | 传输 | 电机输出 |
|---|---|---|
| Bringup | SWD 邮箱；另有启动时 64 B FD+BRS 内部回环 | 编译时禁用，PWM 引脚为低电平 GPIO |
| Bench | SWD 邮箱 | 显式校准/使能命令后允许 |
| Debug / Release | 物理 CAN-FD | 显式校准/使能命令后允许 |
| Host | 假 CAN、便携域逻辑 | 无 MCU |

构建不自动烧录。固件位于 `build/<Preset>/canopen_foc.elf/.hex/.bin`；最后 4 KiB 保留为参数日志。v3 参数记录与新硬件绑定，读取旧 v1/v2 时保留通信设置、重置旧电机校准。

## 不转动电机的读取

启动 OpenOCD 后，用与板上固件匹配的 ELF：

```powershell
openocd -f tools/rollercan-openocd.cfg
python tools/hardware_probe.py --elf build/Bench/canopen_foc.elf --samples 100 --output .work/probe.json
python tools/stlink_bench.py --elf build/Bench/canopen_foc.elf inspect
```

`hardware_probe.py` 只写诊断快照请求，不发送运动命令、不暂停 MCU。读数包含原始 ADC、编码器 CRC/状态、nSLEEP、PWM 使能和运行计数。Bringup 下 CSA 未唤醒、零点未建立；其电流模拟输入不能解释为有效相电流。

- [硬件与采样](docs/hardware.md)
- [4015 直流无刷电机参数](docs/motor-4015.md)
- [电机调试、SWD 验收与补偿结果](docs/motor-commissioning.md)
- [架构与运行异常策略](docs/architecture.md)
- [构建和 CubeMX 维护](docs/build.md)
- [CANopen 使用协议](docs/protocol.md)、[对象字典](docs/object-dictionary.md)、[EDS](config/rollercan.eds)

自有代码采用 MIT，HAL/CMSIS、FreeRTOS、CANopenNode 保留各自许可，见 [第三方依赖](ThirdParty/README.md)。
