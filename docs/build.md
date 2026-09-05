# 构建、重生成与维护

## 环境

固件使用 C++23，测试过 Arm GNU Toolchain 15.2.1。推荐主机也使用 GCC：MinGW GCC 16.1 或 Ubuntu 24.04 GCC 13.3 均已通过测试。CMake 最低 3.25，Ninja，Python 3.10+。工具链不随源代码复制；FreeRTOS、CANopenNode、HAL 和 CMSIS 均已复制，构建无网络依赖。

`tools/build.ps1 -Preset Debug|Release|Host` 运行 OD 一致性、依赖哈希、CubeMX 约束、构建及 ELF/主机测试；Python 不在 PATH 时可传 `-Python /path/to/python.exe`。普通 CMake 命令见根 README。`CMakePresets.json` 中 Debug/Release 为 ARM，Host 为本机，不要对 Host 使用 ARM toolchain。

Linux sanitizer：

```sh
cmake --preset Host -DROLLER_SANITIZE=ON
cmake --build --preset Host
ctest --preset Host
```

如果没有 Ninja，可另用 `cmake -S . -B build/host-make -G 'Unix Makefiles' -DROLLER_HOST_TESTS=ON -DROLLER_SANITIZE=ON`。

## CubeMX 所有权

| 文件 / 目录 | 维护者 |
|---|---|
| `canopen_foc.ioc` | 外设配置唯一来源，使用 CubeMX 保存 |
| `Core/Inc`、`Core/Src` | CubeMX；只在 USER CODE 中接入应用 |
| `Drivers`、`cmake/stm32cubemx`、启动汇编、根生成 linker | CubeMX，使用 G4 V1.6.3 |
| 根 `CMakeLists.txt`、`CMakePresets.json`、`cmake/gcc-arm-none-eabi.cmake` | 自有构建入口；CubeMX 发现现有文件时保留 |
| `linker/STM32G431CB_FLASH.ld` | 实际链接脚本；124 KiB 程序 + 4 KiB 参数区 |
| `Application`、`Platform`、`Protocol`、`RTOS` | 自有实现，生成器不处理 |

打开 `.ioc`，确认 G4 **V1.6.3**、Keep User Code、CMake toolchain，然后 Generate Code。FreeRTOS middleware 必须保持关闭；FreeRTOS 自行接管 SVC/PendSV/SysTick，HAL 时基为 TIM6。不能把 CubeMX 的 CMSIS-RTOS 层再混入当前调度体系。

Windows 命令行调用安装的 CubeMX Java 前端，与图形界面使用同一生成器：

```powershell
./tools/cubemx.ps1 -CubeMxHome 'C:/kk_software/stm_Cubemx'
./tools/build.ps1 -Preset Debug
./tools/build.ps1 -Preset Release
```

脚本检查根构建文件未被覆盖，并验证引脚、ADC 注入序列、PWM、CAN、IRQ 所有权和 C++ 入口。日志位于 `.work/cubemx-generate.log`。CubeMX 会使用根目录名为 `tmp` 的临时文件，故工程临时资料放在 `.work/`，不要建立根 `tmp/` 目录。

重生成后审阅差异，重点检查：

- ADC JEOS、4 路注入序列、内部温度/VREF 虚拟通道、规则组不连续单 rank。
- TIM1 CH3=A / CH2=B / CH1=C、ARR 4000、CH4 CCR 3680、OC4REF TRGO。
- HSI 160 MHz、SPI1 16-bit mode 1、CAN 1 Mbit/s；nFAULT IRQ 及所有 HAL IRQ 调用仍存在。
- 应用入口仍位于 `main.c` 的 USER CODE 2；异常处理仍调用 `roller_panic`。
- FreeRTOS 自有三种异常处理函数没有 CubeMX 的重复定义；实际 linker 仍预留 62/63 页。

本次已使用 CubeMX 真正重新生成，不是在 Platform 层悄悄重配一套与 `.ioc` 不同的外设初始化。运行时必要的 PWM compare、ADC 启停和 SSC DATA 方向切换属于驱动行为，仍由 Board 实现。

现有 `.vscode` 为 CubeMX 生成的 STM32Cube 插件设置，可能引用本机 `cube-cmake` 工具。命令行构建仅依赖 PATH 中的标准 CMake/Arm GCC；`.clangd` 指向 `build/Debug/compile_commands.json`。调试器配置按实际探针选择，不强制假定 ST-Link 序列号或自动烧录。

## 对象字典与依赖更新

```sh
python3 tools/generate_od.py         # 更新 OD、EDS 和说明
python3 tools/generate_od.py --check # 验证完全一致，不修改文件
python3 tools/vendor.py --check      # 离线检查本地库与锁文件
python3 tools/vendor.py --check-lts   # 联网检查官方最新 LTS manifest
python3 tools/vendor.py --update-freertos-lts
```

更新脚本以官方 **LTS manifest** 中的内核版本为准，不是直接追踪内核主分支或非 LTS 新版本。下载解析完成并确认现有库未被本地修改后，才更新固定目录；记录 LTS tag、内核 tag、commit、zip SHA-256 和每个文件的 SHA-256。应把库源码和锁文件一起提交，然后重跑构建、sanitizer、ISR 与栈测试。

GitHub Actions 的普通 CI 编译 Host+sanitizer 和 Debug/Release；每周工作流检查 LTS 是否过期，发现更新时失败提醒维护者，不自动发布未验证固件。创建/修改工作流文件没有替用户启用仓库、推送提交或运行云端作业；要推送后 GitHub 才会执行。

## 格式与代码约定

自有 C/C++ 遵循 `.clang-format`（本次用 21.1.8）。不格式化 `Core`、`Drivers`、`ThirdParty` 或生成 OD，以免重生成/上游更新形成噪声 diff。坚持显式所有权、有限容量、`noexcept`、按值传递跨上下文快照、结果类型检查。性能关键路径不使用 printf、锁、动态分配或系统调用。

改变 HAL 时钟/ADC/PWM、RTOS 中断阈值、编译优化或内核版本后，编译通过并不等于满足 50 µs 的实际预算，须重新实测。硬件首次验证记录填写到 `docs/validation.md`，不要把纯主机测试写成实机结果。
