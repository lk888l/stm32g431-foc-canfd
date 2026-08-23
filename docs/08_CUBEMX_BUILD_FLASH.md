# 构建、烧录与 CubeMX

## 工具与依赖

工程面向：

- STM32G431CBU6（Cortex-M4F，硬浮点）。
- CMake 3.22 或更新版本。
- GNU Arm Embedded `arm-none-eabi-gcc`；当前环境为 14.2.1（14.2.Rel1），正式项目应冻结并记录安装包、版本和 hash。
- Windows 主机测试可使用 MinGW-w64 + `mingw32-make`；CMake Preset 则默认使用 Ninja。
- STM32G4 HAL、CMSIS Device/Core 和 CANopenNode 目录由 `.gitmodules` 描述。

从 Git checkout 获取依赖：

```powershell
git submodule update --init --recursive
```

如果拿到的是不含 `.git` 元数据的文件包，上述命令不可用；必须确认 `Drivers/STM32G4xx_HAL_Driver`、`Drivers/CMSIS/Device/ST/STM32G4xx`、`Drivers/CMSIS/Core` 和 `Middlewares/Third_Party/CANopenNode` 已完整提供，并用发布清单校验提交/hash。

## 一键构建脚本

`tools/build.ps1` 使用 MinGW Makefiles，可分别构建主机测试和目标固件：

```powershell
# GNU Arm 工具链根目录，即内部含 bin\arm-none-eabi-gcc.exe 的上一级
$env:ARM_GNU_TOOLCHAIN_ROOT = 'C:\Toolchains\arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi'

# 主机单元测试
.\tools\build.ps1 -Target host -Configuration Debug

# STM32 Debug 固件
.\tools\build.ps1 -Target firmware -Configuration Debug

# SimpleFOC Mini + AS5600 Lab 固件；输出到 build/firmware-lab-debug
.\tools\build.ps1 -Target firmware -Configuration Debug -BoardProfile Lab

# 两者都构建；量产候选用 Release 并另保存测试构建
.\tools\build.ps1 -Target all -Configuration Release
```

脚本要求 `cmake`、`mingw32-make` 在 `PATH` 中；找不到时应显式修正开发环境，不要把个人绝对路径写进仓库。

产物：

```text
build/host/                         主机测试
build/firmware-debug/brushless_control.elf/.hex/.bin
build/firmware-debug/brushless_control.map
build/firmware-lab-debug/           Lab profile 对应产物
build/firmware-release/             Release 对应产物
```

链接脚本为 `STM32G431CBUX_FLASH.ld`：128 KiB Flash、32 KiB SRAM 视图。每次构建会打印 Flash/RAM 使用量并生成 `.su` stack-usage 文件；量产放行必须检查 map、未解析弱符号、栈水位和最坏中断嵌套，不能只看“链接成功”。

2026-08-24 在最新 PP/RPDO 原子提交、mode-instance/带转矩模式切换屏障与 d/q 双轴 tracking anti-windup 合入后的全量 `--clean-first` 构建快照为：Release Production `83516 B Flash / 7784 B RAM`，Debug Production `85184 B / 7768 B`，Debug Lab `82912 B / 7768 B`；主机 CTest `3/3` 通过。`arm-none-eabi-size` 对应的 `text/data/bss` 分别为 Release `82780/736/7056 B`、Debug `84448/736/7040 B`、Lab `82176/736/7040 B`；链接器 region 统计是判断 MCU Flash/RAM 占用的权威值。三份 ELF 各有 3 个 LOAD segment（flash R/E、data R/W、bss/noinit R/W），未发现 RWX segment。Release `.su` 的最大单函数静态栈帧为 `app_supervisor_irq = 368 B`，`motor_foc_fast_step = 232 B`；这不是最坏中断嵌套或完整调用链栈水位的替代品。

本次 Release 发布产物 SHA-256：

```text
ELF  A30D3213C4685F0055CF6E79AA3F38E8C73B3B7142D4D98F7C3F6E2FC5450FBA
HEX  513BAE2563FBC2ABAAD6D09E45042530291B9A68F327522FC0CFCE1E52F98DED
BIN  392A8FAAFFD9C4CA238F4180EF1D17771B5FB07AD6E5375B9B6C6CF4FAF5ACC2
```

这些哈希和资源数据记录了当前固件源码、构建配置与工具链的本地验证结果。正式发布时仍必须从干净的候选提交重新构建，并把实际 Git commit、dirty 状态和生成日志与产物一起归档。代码、工具链或 linker script 改变后必须重新生成；文档记录也不能代替目标板运行、栈水位、HIL、功率和安全测试。

## CMake Presets

安装 Ninja 并保证 ARM GCC 可发现后，也可使用：

```powershell
cmake --preset host-debug
cmake --build --preset host-debug
ctest --preset host-debug

$env:ARM_GNU_TOOLCHAIN_ROOT = 'C:\Toolchains\arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi'
cmake --preset firmware-debug
cmake --build --preset firmware-debug

cmake --preset firmware-release
cmake --build --preset firmware-release

# SimpleFOC Mini + AS5600 实验固件；仍默认使用 8 MHz HSE
cmake --preset firmware-lab-debug
cmake --build --preset firmware-lab-debug
```

如果同一个 build 目录已经用不同生成器或工具链配置过，应使用一个新的 build 目录；不要手工篡改 `CMakeCache.txt` 让不兼容配置“继续跑”。

## 构建选项

| CMake 选项 | 主机默认 | 交叉编译默认 | 说明 |
|---|---:|---:|---|
| `BRUSHLESS_BUILD_TESTS` | ON | OFF | 构建硬件无关单元测试；交叉编译时禁止 |
| `BRUSHLESS_BUILD_FIRMWARE` | OFF | ON | 构建 STM32 ELF/HEX/BIN；必须使用 ARM toolchain file |
| `BRUSHLESS_BOARD_PROFILE` | `PRODUCTION` | `PRODUCTION` | `PRODUCTION` → MT6701/真实电流链；`LAB` → SimpleFOC Mini/AS5600 受限电压实验 |
| `BRUSHLESS_USE_8MHZ_HSE` | ON | ON | 使用 8 MHz HSE；所有 firmware profile 都启用 CAN-FD，设为 `OFF` 一律在 configure 阶段失败 |
| `ARM_GNU_TOOLCHAIN_ROOT` | 空 | 环境/缓存 | GNU Arm 安装根目录 |

主机 `brushless_portable` 使用 C11，并对 GCC/MinGW 开启 `-Wall -Wextra -Wpedantic -Werror -Wconversion -Wdouble-promotion -Wshadow`。固件构建使用 Cortex-M4F hard-float、section GC、newlib-nano/nosys。任何新增的硬件无关算法都应先进入 portable library 和主机测试。

固件 target 默认同时编译 `App/canopen` 和 CANopenNode v4 所需的 NMT、Heartbeat consumer、EMCY、SDO server、PDO 与 FIFO 源文件；缺少 CANopenNode submodule 会在 CMake configure 阶段失败。该端口使用静态对象并固定为 500 kbit/s Classic CAN，不是可关闭后仍能宣称“有 CANopen”的占位选项。

`tools/build.ps1 -Target firmware` 使用默认 `PRODUCTION`/HSE 配置；传 `-BoardProfile Lab` 会使用独立的 `build/firmware-lab-*` 目录。实验板也可使用 `firmware-lab-debug` preset，或在新的 build 目录显式传 `-DBRUSHLESS_BOARD_PROFILE=LAB`。不要通过全局 `CMAKE_C_FLAGS` 注入 `BOARD_PROFILE`，否则 CMake 缓存、IDE 与发布记录无法可靠表达硬件配置。当前任何 firmware profile 都不能关闭 HSE；若将来要支持纯 HSI 板，必须作为新的受控时钟/通信配置实现和验证，不能绕过 CMake 检查。

## 烧录

先连接 ST-LINK，仅给控制板低压电源且保持 DC bus 断开。若安装 STM32CubeProgrammer，可按实际安装路径执行：

```powershell
& 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe' `
  -c port=SWD freq=4000 `
  -w '.\build\firmware-debug\brushless_control.elf' `
  -v -rst
```

如果该路径不存在，用系统实际的 CubeProgrammer 路径。首次板卡建议降低 SWD 频率。烧录后仍保持母线断开，先验证 GPIO 安全态、BKIN、编码器、ADC 和 CAN；不要因为程序能启动就直接使能功率级。

量产烧录记录应包含：ELF/HEX 的 SHA-256、Git 提交和 dirty 状态、编译器/依赖版本、板号、校准区版本、读回校验、option bytes 和最终锁定策略。不要只保存 `.bin` 文件名。

## CubeMX 实时调用

用户提供的安装目录可用。当前主机实际调用的 STM32CubeMX CLI 为 `6.16.1`，器件数据库为 `6.0.161`；`.ioc` 当前选择 `STM32Cube FW_G4 V1.6.3`。但本机 Repository 中尚未发现该版本已解包的 `STM32Cube_FW_G4_V1.6.3` 目录，因此这里的成功结论只覆盖配置 reload/check，不表示 CubeMX 已具备离线代码生成所需的完整 G4 包。包装器使用安装目录内自带 Java。常用命令：

```powershell
# 打开 .ioc 图形界面
.\tools\cubemx.ps1 -Action open

# 不修改 .ioc：静态检查关键契约，再让 CubeMX 重新 load/check
powershell -ExecutionPolicy Bypass -File tools\cubemx.ps1 -Action validate

# 有意重新施加完整量产外设基线；会规范化并改写 .ioc
powershell -ExecutionPolicy Bypass -File tools\cubemx.ps1 -Action configure-ioc

# 交互安装 STM32CubeG4 软件包；可能出现 ST 登录/许可证界面
.\tools\cubemx.ps1 -Action install-g4
```

包装器实际使用 CubeMX 自带 Java：

```powershell
& 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeMX\jre\bin\java.exe' `
  -jar 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeMX\STM32CubeMX.exe' `
  -q '<script-file>'
```

`install-g4` 故意使用交互模式，脚本不会代表用户接受许可证。此前静默安装留下了下载归档，但没有形成可确认的已解包 G4 包目录；请在 GUI 中登录、阅读并接受适用条款后完成安装，再把 `generate` 的 staging 结果作为独立步骤验证。固件当前从仓库内固定的 HAL/CMSIS 源目录构建，不要求为了编译而绕过许可证。

## `.ioc` 的当前状态与边界

`brushless_control.ioc` 已不再是旧的 HSI/Classic-CAN 引脚脚手架。`configure-ioc` 已实际运行完成，依次报告 peripheral configuration completed、static `.ioc` contract PASS 和 `CubeMX reload/check validation completed`；独立 `-Action validate` 也通过静态时序/引脚契约与 CubeMX `config load` + `check mcu modes` 重新加载检查。当前基线包含：

- 准确料号/封装 STM32G431CBU6 / UFQFPN48，8 MHz HSE、PLLM=2/N=85/R=2 到 170 MHz。
- TIM1 中心对齐 20 kHz，ARR=4249，CH1/2/3 互补输出，deadtime=34 tick；PA15 TIM1_BKIN；CH4 PWM2=`4164`，按 `period - min_pulse/4 - 1` 放在 crest 前 85 tick，使两个 injected aperture 分布在 crest 附近，每载波只有一个上升沿。340 tick/2 µs 最小脉宽仍是必须实测的约束。
- ADC1_IN2 与 ADC2_IN4 各两个同通道 injected rank。两路 scan 已设 enable、injected length 为 2（硬件目标 `JSQR.JL=1`）、EOC 选择 sequence，common mode 是非 alternate `ADC_DUALMODE_INJECSIMULT`（硬件目标 `ADC_CCR.DUAL=0b00101`）；TIM1_CC4 rising 触发 ADC1 master，ADC2 由 dual event 同步启动。`.ioc` 不给 ADC2 独立 trigger；手写 BSP 虽让 HAL 写入同样的 slave JEXTSEL/JEXTEN，但这些位在 dual injected slave 模式中被硬件忽略，HAL start 只使能 slave，不会形成第二次触发。这是 §2.5.11 的明确排除路径。手写 BSP 再按 `REV_ID` 选择 JDR1/JDR2：Rev Z `0x2001` 用 JDR1，Rev Y `0x2002` 与 Rev X `0x2003` 用 JDR2；GPIO 安全态先建立，未知 revision 随后拒绝启动。为覆盖 §2.5.9，手写 IRQ/BSP 还会丢弃实时启动后的第一整个 dual sequence；这不是 `.ioc` 能表达或 CubeMX check 能验证的行为。
- PA3 ADC1_IN4 的 VBUS regular 在 `.ioc` 与 BSP 中都设为 software-start/no-edge、非 continuous、同通道 Rank1+Rank2（47.5 cycles、single-ended、no offset），`NbrOfConversion=2`、`ADC_OVR_DATA_PRESERVED`；ADC2 regular 保持 disabled/length=1。它只在 injected JEOS 后由手写 BSP 启动。生成/移植后必须实读 ADC1 `SQR1.L=1` 且两个 SQ 均为 IN4，再验证动态 EOC/EOS/OVR IRQ 只发布 rank 1、丢弃 rank 2，以及 rank1-captured 与延迟 OVR-preserved 两条路径；CubeMX reload/check 不能证明运行时 IRQ 语义。任何未来 ADC restart/recalibration 路径也必须手工重新 arm §2.5.9 的首 dual sequence 丢弃。
- TIM2 prescaler=169、32-bit ARR，得到 1 MHz 微秒时基；IWDG prescaler/reload 初值约 100 ms。
- FDCAN1 FD+BRS：170 MHz kernel，nominal prescaler/segments `10/27/6` = 500 kbit/s，data `5/13/3` = 2 Mbit/s；经典 CANopen 仍逐帧选择 Classic 格式。
- SPI3 + DMA1 Channel 1/2、I2C1、GPIO 上电安全态；NVIC Break=0、ADC=1、SPI/DMA=2、FDCAN=3、I²C=4、PendSV=5、SysTick=15。

该结果证明“配置文件能由当前 CubeMX 重新加载且关键字段符合仓库契约”，不证明目标 PCB、模拟时序、器件勘误、功率安全或生成代码已经通过量产验证。包装器会在 CubeMX CLI 配置后执行 deterministic metadata postprocess，因为当前 headless 模式对部分派生时钟/DMA/NVIC 元数据不能稳定保存；随后再做严格字段/算术检查和 CubeMX reload/check。升级 CubeMX 或改外设时必须审查这个后处理，不能把它当作绕过配置冲突的通用工具。

`create-ioc` 是 `configure-ioc` 的兼容别名。两者都会改写 `.ioc`，只应在有意重建基线时执行；日常 CI/复核使用只读的 `validate`。包装器从自身路径解析项目根目录，不含当前工作区绝对路径。

### 安全的生成工作流

CubeMX 输出只生成到 `staging/cubemx`，再人工 diff/移植：

```powershell
# 只有在 .ioc 已经 validate、经过评审且 STM32CubeG4 包安装完成后执行
.\tools\cubemx.ps1 -Action generate
```

`tools/cubemx_generate.txt.in` 设置 CMake/GCC、Advanced structure 和 `copy only`，输出到 staging。评审差异时重点检查：时钟、FDCAN timing、ADC trigger/rank、TIM1 polarity/deadtime/BKIN、GPIO 初值、IRQ 优先级、HAL 包版本和 linker memory。

CubeMX “check modes” 通过不代表芯片勘误已规避。生成差异评审必须逐条核对 ES0431 §2.5.8/§2.5.10 的 revision-aware rank 策略、§2.5.9 首 dual sequence 丢弃与 JEOS 后 VBUS 状态机，并保持 §2.5.11 的 non-alternate dual injected simultaneous。启动调试应记录 `REV_ID`，读出 `ADC_CCR.DUAL=0b00101` 与 ADC1/2 `JSQR.JL=1`，观察 JDR1/JDR2 每周期变化，确认首序列未进入控制、固件按 revision 取样，并验证 `CCR4=4164` 下所选 aperture 的窗口和同步/精度。只看到 `.ioc` 字段或 HAL 返回成功不算通过。

不要把 staging 目录整体覆盖现有 `App/control`、`App/drive`、`App/protocol`、`App/safety` 或手写 BSP。推荐流程：

```text
修改并评审 .ioc → 生成到 staging → 编译 staging 原始输出
→ 对比外设初始化 → 选择性移植到 BSP → 主机/目标/HIL 回归
```

## CubeMX 与 CMake 的权威关系

- CMake、linker、portable control、安全和协议代码由仓库维护，是构建权威。
- 当前 `.ioc` 是经过自动 reload/check 的引脚、时钟和外设意图基线；只有再与最终原理图/PCB、BSP 和目标波形逐项签字后，才能成为指定硬件版本的权威配置。
- 生成 C 初始化代码只是候选实现；手写实时/BKIN/安全细节必须通过 diff 审查保留。
- 更换 CubeMX/HAL 版本属于受控变更，需要全量编译、目标时序和故障回归。

CubeMX 官方用户手册可用于核对 CLI 和代码生成行为：[STM32CubeMX User Manual UM1718](https://www.st.com/resource/en/user_manual/dm00104712-stm32cubemx-for-stm32-configuration-and-initialization-c-code-generation-stmicroelectronics.pdf)。
