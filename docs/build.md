# 构建与维护

需要支持 C++23 的 Arm GNU `arm-none-eabi` 工具链、CMake ≥3.25、Ninja、Python ≥3.10。Host 使用本机 GCC。HAL/CMSIS、FreeRTOS、CANopenNode 均在仓库中，不在构建时下载。

```powershell
./tools/build.ps1 -Preset Host -Fresh
./tools/build.ps1 -Preset Bringup -Fresh
./tools/build.ps1 -Preset Debug -Fresh
./tools/build.ps1 -Preset Release -Fresh
python -m unittest discover -s tools -p 'test_*.py'
```

`build.ps1` 检查 OD 生成一致性、第三方哈希、引脚/外设配置约束，构建并运行 CTest 或检查 ELF 静态分配和中断归属。更换工具链或复制旧缓存后用 `-Fresh`。Bringup 使用 SWD 邮箱且编译时禁止电机输出；Bench 保留 SWD 传输并允许显式运动；Debug/Release 使用物理 CAN。

## 烧录与只读验证

构建不会自动烧录。使用目标 preset 的 ELF/HEX，仅擦写映像覆盖的页，保留 `0x0801F000..0x0801FFFF` 参数区。首次迁移先备份原 Flash；本次备份位于 `.work/hardware-v2/original-flash.bin`，未自动重写参数。

```powershell
openocd -f tools/rollercan-openocd.cfg
python tools/hardware_probe.py --elf build/Bringup/canopen_foc.elf --samples 100 --output .work/probe.json
python tools/stlink_bench.py --elf build/Bringup/canopen_foc.elf inspect
```

诊断工具仅请求快照；不暂停 MCU、不发送校准或使能命令。与固件不匹配的 ELF 会提供错误地址，必须使用同一构建产物。Bringup 通过内部 64 B CAN-FD+BRS 回环检查 FDCAN，`can_loopback=1` 表示通过，2 为失败；不验证外部收发器、终端电阻或总线 ACK。

## CubeMX 状态

工程保留 CubeMX **6.17.0 / STM32CubeG4 V1.6.3** 配置。2026-11-16 本机配置路径中的 6.17.0 不存在，实际安装可找到的版本为 6.16.1；**本轮未宣称已验证 CubeMX 重生成**，没有通过降低 `.ioc` 版本绕过检查。

新硬件的 `.ioc` 与 `Core` 初始化代码已同步修改，并通过静态配置检查和实际编译、实机外设测试。`Core/Src/adc.c` 的双 ADC 初始化和 Bringup 的 GPIO 低电平分支当前为手动维护。重新生成前应安装匹配版本，在隔离副本运行生成并复核差异，尤其保留 Bringup 不输出的逻辑；不能直接覆盖后默认认为语义相同。

```powershell
./tools/cubemx.ps1 -Check -CubeMxHome '<实际安装的 6.17.0 或更新版本目录>'
python tools/check_project.py
```

配置约束：ADC1/ADC2 同步三 rank、CDR 32 位循环 DMA、SPI3 mode1/8bit/5MHz、TIM1 CCR4=3920、PWM A/B/C=CH1/2/3、CAN=PB8/9、nSLEEP=PB10。DMA1 通道 2/3 由 SSI 驱动自主管理。FreeRTOS 不通过 CubeMX middleware 生成，SysTick/SVC/PendSV 由静态内核拥有。

实际链接脚本为 `linker/STM32G431CB_FLASH.ld`，不是 CubeMX 根目录备用脚本。禁止动态分配，20 kHz 域代码在 Debug/Release 均使用 -O2，保留浮点有限性检查。

## 验证边界

Host 覆盖 MT6701 全部 18 位负载 CRC 和单比特翻转、完整 SVPWM 圆周、重建/预测、速度换向、校准失步、非有限值故障复位、NMT 同批状态边界、CAN-FD 和 Flash 掉电日志。硬件 Bringup 覆盖静止编码器、ADC 母线、内部 CAN 回环、协议只读访问。

带电 PWM 采样建立、真实 R/L/Kt、旋转角度跟踪、负载电流阶跃和高速饱和区仍须在后续允许转动电机的阶段验证。旧板历史性能不迁移为新板的验证证据。
