# STM32G431 · DRV8316C · MT6701 FOC / CAN-FD

面向 simple-foc v2.0 驱动板和 4015 电机的 C++23 固件。MCU 为 STM32G431CBU6，主频 160 MHz，20 kHz 电流环、1 kHz 运动环、FreeRTOS 静态任务和 CANopenNode。支持单圈/多圈位置、速度、力矩和 MIT 模式。

本轮已迁移新引脚、DRV8316CT 电流采样及 MT6701 SSI，修复上次审查的 PWM 更新、校准、反向斜坡、NMT/PDO 边界和非有限值故障复位问题。驱动器不会因为通信超时、单帧编码器错误、采样窗口不足或一次控制延期而直接失能。显式控制字/NMT 停止、芯片自身保护和真正执行停滞的看门狗仍具有原本语义。

SVPWM 使用完整线性范围 `|Vdq| <= Vbus / sqrt(3)`，占空比 0～100%，不再固定限制为 4 V 或 10%～90%。DRV8316C 是低侧电流检测；近饱和区采用有效相重建和模型预测，**满电压利用率不等于每周期都能直接测到三相电流，也不等于已经实测验证的带载性能**。

## 当前板上固件

2026-11-16 的实机工作仅为不转动电机的 Bringup 测试。该构建将 PB10/nSLEEP 固定为低、PA8/9/10 固定为低电平 GPIO；没有进行电角校准或使能握手。磁编码器、双 ADC、内部 CAN-FD 回环、SWD 上的 SDO 读回和计数器已验证，详情见 [v2 验证记录](docs/v2-validation.md)。

## 构建

```powershell
./tools/build.ps1 -Preset Host
./tools/build.ps1 -Preset Bringup
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
python tools/hardware_probe.py --elf build/Bringup/canopen_foc.elf --samples 100 --output .work/probe.json
python tools/stlink_bench.py --elf build/Bringup/canopen_foc.elf inspect
```

`hardware_probe.py` 只写诊断快照请求，不发送运动命令、不暂停 MCU。读数包含原始 ADC、编码器 CRC/状态、nSLEEP、PWM 使能和运行计数。Bringup 下 CSA 未唤醒、零点未建立；其电流模拟输入不能解释为有效相电流。

- [硬件与采样](docs/hardware.md)
- [架构与运行异常策略](docs/architecture.md)
- [构建和 CubeMX 维护](docs/build.md)
- [CANopen 使用协议](docs/protocol.md)、[对象字典](docs/object-dictionary.md)、[EDS](config/rollercan.eds)

自有代码采用 MIT，HAL/CMSIS、FreeRTOS、CANopenNode 保留各自许可，见 [第三方依赖](ThirdParty/README.md)。
