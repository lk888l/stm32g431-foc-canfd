# 构建、重生成与维护

## 环境与构建

固件使用 C++23，工具链为 Arm GNU `arm-none-eabi`，CMake ≥3.25、Ninja、Python ≥3.10。主机测试需要支持 C++23 的本机 GCC。`arm-none-linux-gnueabihf` 是 Linux 交叉编译器，不能替代裸机工具链。FreeRTOS、CANopenNode、HAL 和 CMSIS 以普通源码文件收录，构建不需要下载依赖。

将工具链的 `bin`、CMake、Ninja 和 Python 加入 PATH，并重新打开终端。在工程根目录执行：

```powershell
./tools/build.ps1 -Preset Debug
./tools/build.ps1 -Preset Release
./tools/build.ps1 -Preset Host
```

`build.ps1` 执行 OD 一致性、依赖哈希、CubeMX 约束、编译及 ELF/主机测试；`-Python` 可指定解释器，`-Jobs` 控制并行数。复制过旧构建目录或更换编译器时使用 `-Fresh`，例如 `./tools/build.ps1 -Preset Debug -Fresh`。普通 CMake 构建命令见 [README](../README.md)。

| Preset | 用途 |
|---|---|
| Debug | 物理 CAN 固件，保留调试信息，冻结调试暂停期间的 IWDG |
| Release | 物理 CAN 固件，优化代码体积 |
| Host | 本机控制、协议和参数存储测试，不使用 ARM 工具链 |
| Bench | 使用 SWD 邮箱替换物理 CAN 的专用固件，邮箱最多 8 B |

Debug/Release 的输出为 `build/<Preset>/canopen_foc.elf/.hex/.bin/.map`，构建不自动烧录。烧录时使用匹配的 ELF 或 HEX，保留最后 4 KiB 参数区，不执行整片擦除。普通通信、初始化和运动控制见 [协议说明](protocol.md)。

Bench 工具命令可用 `python tools/stlink_bench.py --help` 查看；运行时必须使用匹配的 Bench ELF。它不能验证物理 CAN 收发器、ACK 或 CAN-FD 长帧；恢复正常 CAN 通信需烧录 Debug/Release。

## 主机检查

CTest 包含 `roller_domain_and_protocol` 和 `roller_canfd_protocol`。Python 工具测试另行运行：

```sh
python3 -m unittest discover -s tools -p 'test_*.py'
```

Linux sanitizer 使用独立构建目录，避免与 Windows 缓存混用：

```sh
cmake -S . -B build/Host-sanitize -G Ninja -DROLLER_HOST_TESTS=ON -DROLLER_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Host-sanitize --parallel
ctest --test-dir build/Host-sanitize --output-on-failure
```

没有 Ninja 时可选择 `-G 'Unix Makefiles'` 并使用另一个构建目录。主机测试验证逻辑和协议，不能替代电机、采样窗口和实时路径的硬件验证。

## 迁移与 VS Code 排错

迁移复制源码，构建缓存由新机器重建。排查顺序：

1. `python tools/vendor.py --check` 检查依赖完整性，拒绝 gitlink 或嵌套 Git 仓库。`--fresh` 只能清缓存，不能补齐丢失的 HAL/CMSIS 源码。
2. `./tools/build.ps1 -Preset Debug -Fresh` 使用 PATH 中工具链重新配置、构建。更新 PATH 后重新打开终端或 VS Code。
3. 在 VS Code 中打开工程根目录，选择 Debug/Release configure preset；迁移缓存后执行 **CMake: Delete Cache and Reconfigure**。

`.vscode/settings.json` 的 `cube-cmake`、`cube` 和 `CUBE_BUNDLE_PATH` 由 STM32Cube 扩展提供，使用安装了该扩展的 Profile。普通 PowerShell 使用标准 `cmake` 或 `build.ps1`。仅使用 Microsoft CMake Tools 时，可将 `cmake.cmakePath` 改为 `cmake` 并删除 `cmake.configureArgs` 中的 `-DCMAKE_COMMAND=cube-cmake`。

CubeMX 应与 `.ioc` 所需的 **6.17.0** 版本兼容，并安装 **STM32CubeG4 V1.6.3** 软件包。旧版加载新版工程可能出现对话框或停在命令行生成阶段；不要降低 `.ioc` 版本字段绕过检查。先运行 `./tools/cubemx.ps1 -Check` 检查实际安装版本。VS Code 的 `stm32cube-ide-core.configuration.productSTM32CubeMX.executablePath` 应指向所需版本；路径变更后可执行 **Developer: Reload Window**。仅有仓库中的 Drivers 不等于安装了 CubeMX 软件包。

## CubeMX 所有权

| 文件 / 目录 | 维护者 |
|---|---|
| `canopen_foc.ioc` | CubeMX 管理的时钟、引脚、外设静态配置与 ADC DMA 配置来源，用 CubeMX 保存 |
| `Core/Inc`、`Core/Src` | CubeMX；仅 USER CODE 中接入应用 |
| `Drivers`、`cmake/stm32cubemx`、启动汇编、根生成 linker | CubeMX，使用 G4 V1.6.3 |
| 根 CMake、Presets、`cmake/gcc-arm-none-eabi.cmake` | 自有构建入口 |
| `linker/STM32G431CB_FLASH.ld` | 实际链接脚本，124 KiB 程序 + 4 KiB 参数 |
| `Application`、`Platform`、`Protocol`、`RTOS` | 自有实现，生成器不处理 |

使用 CubeMX 打开 `.ioc`，确认 G4 V1.6.3、Keep User Code 和 CMake toolchain，再 Generate Code。FreeRTOS middleware 保持关闭；自有 FreeRTOS 接管 SVC/PendSV/SysTick，HAL 时基为 TIM6，不混入 CMSIS-RTOS 层。

Windows 命令行生成：

```powershell
./tools/cubemx.ps1 -Check
./tools/cubemx.ps1
./tools/build.ps1 -Preset Debug
./tools/build.ps1 -Preset Release
```

脚本通过 `-CubeMxHome`、`STM32CUBEMX_HOME` 或默认安装位置找到生成器，拒绝低于工程要求的版本，检查根构建文件未被覆盖，并验证引脚、采样、PWM、CAN、IRQ 和应用入口。日志位于 `.work/cubemx-generate.log`。CubeMX 会使用名为 `tmp` 的临时文件，工程临时目录使用 `.work/`，不要创建根 `tmp/` 目录。

重生成后检查：

- ADC1 规则组四路电流/母线序列、TIM1 TRGO 上升沿、24.5 周期采样、循环 DMA；内部温度/VREF 为软件触发的两路注入序列、247.5 周期采样。
- `MX_DMA_Init()` 在 ADC 初始化前调用，DMA1_Channel1 为半字、内存递增、Very High，半满/全满和 ADC 错误中断处理保留；overrun 使用保留数据并报告错误的配置。
- TIM1 CH3=A / CH2=B / CH1=C、CENTERALIGNED1、ARR=4000、RCR=1、CH4 CCR=3680、OC4REF TRGO；TIM1_UP_TIM16 中断优先级 2，在自然下溢启动编码器 DMA。
- HSI 160 MHz、SPI1 16-bit mode 1、FDCAN 默认 1M/5M 和 FD+BRS、nFAULT IRQ。
- CORDIC 初始化和时钟由 CubeMX 管理，算法模式与输入/结果 DMA 由板级适配器管理。
- `main.c` USER CODE 2 中的应用入口、异常中的 `roller_panic` 调用。
- FreeRTOS 异常处理没有重复定义，实际 linker 保留 Flash 页 62/63。

HAL/CMSIS 集合及哈希由 `ThirdParty/vendor-lock.json` 维护，哈希归一化 CRLF/LF。更改外设或升级软件包导致驱动变化时，应审阅并更新锁文件。运行时 ADC/DMA 启停、PWM burst、SSC DATA 方向和保存的 CAN 位时序由 Board 及 `Platform/STM32` 适配器管理。

DMA1_Channel1 的 ADC 配置由 CubeMX 生成；Channel2/3 的 SPI1 RX/TX、Channel4 的 TIM1 COM burst、Channel5/6 的 CORDIC READ/WRITE 由板级代码配置对应 DMAMUX 通道。这六个通道为当前控制路径保留，在 CubeMX 新增 DMA 请求时必须避免冲突。SPI DMA IRQ 在 `ssc_dma.cpp` 实现；TIM1 update IRQ 的 USER CODE 清除 update 标志并调用编码器触发入口，随后直接返回。重生成和链接后由 ELF 检查确认 update/ADC/SPI DMA 中断实际使用强处理函数，防止落入启动文件的默认弱中断。完整通道分配见 [架构](architecture.md#dma-数据路径与所有权)。

DMA 路径的实机验证应检查连续帧、传输错误、采样重启、栈余量和输出关闭状态。首个编码器事务在 PWM 定时器启动前准备，后续由下溢中断每 50 µs 启动；ADC 在同周期约 23 µs 开始扫描，与该周期编码器结果汇合后才执行控制。`0x2100:0A` 与 Bench 的完整路径耗时从 ADC DMA 帧回调入口计至控制和输出结束，只包含 ADC 回调之后尚未完成的 SSC 等待与中断延迟；之前的编码器传输不在该计时区间内。单个 DMA ISR 的执行时间不足以代表这段延迟。静态 Bench 检查不触发使能或运动，也不能替代带功率输出时的采样窗口和 PWM 更新验证。

## CAN 时序与参数布局

默认仲裁段 1 Mbit/s、数据段 5 Mbit/s，`FDCAN_FRAME_FD_BRS`。运行时位时序见 `Platform/STM32/fdcan_timing.hpp`，基于 160 MHz FDCAN 时钟。发送延迟补偿 TDC 仅允许数据预分频为 1 或 2；1 Mbit/s 数据档使用 prescaler=5，因此关闭 TDC，其余当前档位可开启。用户可配置的速率与保存流程见 [协议说明](protocol.md)。

参数布局 v2 为 1044 B，冻结的 `Protocol/CANopen/OD_v1.h` 对应旧 v1 的 788 B。迁移保留旧校准、Node-ID、PDO 和持久化参数，旧只读通信常量采用新默认 1M/5M/BRS=1。首次加载只在 RAM 迁移，显式保存才写 v2；双页日志在长度变化时仍保留旧有效记录，直到新页完成 CRC 和提交。不要擦除最后 4096 B 参数区。

## 对象字典与依赖更新

```sh
python3 tools/generate_od.py         # 更新 OD、EDS 和索引说明
python3 tools/generate_od.py --check # 只验证生成结果一致
python3 tools/vendor.py --check      # 离线检查依赖与锁文件
python3 tools/vendor.py --check-lts   # 联网检查官方 LTS manifest
python3 tools/vendor.py --update-freertos-lts
```

FreeRTOS 更新以官方 LTS manifest 为准。脚本确认本地库未被修改后才替换文件，并记录版本、commit、归档和逐文件 SHA-256。提交库与锁文件后重跑构建、sanitizer 和必要的时序验证。GitHub Actions 提供构建测试和周期性 LTS 检查，不自动发布固件。

## 格式与代码约定

自有 C/C++ 遵循 `.clang-format`，不格式化 Core、Drivers、ThirdParty 或生成 OD。使用显式所有权、固定容量、`noexcept`、跨上下文值快照和结果检查；热路径不使用 printf、锁、动态分配或系统调用。

20 kHz 热路径在 Debug/Release 都使用 `-O2`，保留浮点有限性检查，不使用 `-ffast-math`；使用完整 newlib。Board 运行时启用 Flash prefetch，控制路径仍从 Flash 执行，没有迁入 RamFunc。改变 HAL 时钟、ADC/PWM/DMA、RTOS 优先级、优化或内核后，应重新检查 50 µs 控制周期、完整采样至输出延迟、栈余量和功率关闭行为。
