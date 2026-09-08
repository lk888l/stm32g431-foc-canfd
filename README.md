# RollerCAN C++23 / STM32G431

面向 **M5Stack Unit-RollerCAN V1.0 / STM32G431CBU6** 的可离线构建电机固件。业务层使用 C++23，20 kHz FOC 与 1 kHz 运动控制运行在 ADC 中断中；FreeRTOS 承担 CANopen 和监督任务。

已在 11.6 V、无负载的 Unit-RollerCAN 上完成校准、正反转、位置往返、电流跟踪、快速停止和参数保存测试，记录见 [实机结果](docs/bench/2026-09-08.md)。默认上电关桥，未校准时不接受闭环使能。物理 CAN 互通、示波器采样窗口、带载与耐久测试仍待完成。

## 依赖与工具

| 项目 | 版本 / 选择 |
|---|---|
| C++ | C++23；`std::expected`、`std::span`、静态容量容器、无异常、无 RTTI |
| FreeRTOS | **202604.01-LTS / Kernel V11.3.1**，按 2026-09-05 官方 LTS manifest 核实 |
| CANopen | CANopenNode **v4.1**；经典 CAN，1 Mbit/s，默认 Node-ID 1 |
| HAL / CMSIS | STM32CubeG4 **V1.6.3**，由 CubeMX 复制到本地 |
| CubeMX | **6.17.0**，CMake 工程，HAL 时基 TIM6 |
| 编译器 | 已验证 Arm GNU **15.2.1**；主机 GCC 16.1 / Ubuntu GCC 13.3 |
| 构建工具 | CMake ≥3.25、Ninja、Python ≥3.10；测试需要支持 C++23 的主机 GCC |

外部库完整保存在 `ThirdParty/` 与 `Drivers/`，没有 Git submodule、FetchContent 或构建时下载。ETL 暂未引入：现有静态队列、数组和非拥有接口已满足容量及生命周期要求。

## 构建

将 `arm-none-eabi-g++`、`cmake`、`ninja` 和 Python 加入 PATH。在工程根目录执行：

```powershell
./tools/build.ps1 -Preset Host
./tools/build.ps1 -Preset Debug
./tools/build.ps1 -Preset Release
```

Linux 或直接使用 CMake：

```sh
python3 tools/vendor.py --check
python3 tools/generate_od.py --check
python3 tools/check_project.py
cmake --preset Host
cmake --build --preset Host
ctest --preset Host
cmake --preset Debug
cmake --build --preset Debug
python3 tools/check_project.py --elf build/Debug/canopen_foc.elf
```

输出：`build/Debug/` 或 `build/Release/` 内的 `.elf`、`.hex`、`.bin`、`.map`。移植旧目录后若 CMake 报缓存路径不一致，执行 `cmake --fresh --preset Debug`。未提供自动烧录动作。

## 生成与源码边界

直接打开 `canopen_foc.ioc`，在 CubeMX 点击 Generate Code 即可。自定义代码位于 `Application/`、`Platform/`、`Protocol/`、`RTOS/`；`Core/` 仅在 `USER CODE` 块保留 C/C++ 入口和紧急关闭调用。根 CMake、工具链和独立 linker script 由本项目维护。

命令行生成及完整约束见 [构建与重生成](docs/build.md)。不要在 CubeMX 中启用 FreeRTOS middleware，也不要覆盖现有自有 CMake 文件。使用已锁定的 G4 软件包生成；升级 CubeMX/HAL 后需要重跑检查。

## 功能与协议边界

- 三相电流采样、TLI5012B SSC 编码器、母线采样、Clarke/Park、带限幅与抗积分饱和的 PI、SVPWM。
- CiA 402 风格状态机，位置模式 1、速度模式 3、转矩模式 4、MIT 模式 5；快速停止、故障锁存与显式复位。
- NMT、Heartbeat producer/consumer、EMCY、SDO、SYNC、4 RPDO / 4 TPDO 与动态映射。
- 采用 HexMeow OD 0x08 的 Float32 转数 / 转每秒单位，以及 64 位压缩 MIT 布局。详见 [协议兼容性](docs/protocol.md) 和 [EDS](config/rollercan.eds)。
- 100 ms 有效运动命令超时、硬件故障脚直接关桥、过流/过压/欠压/温度/编码器/时序故障、任务健康监督与 IWDG。
- 显式电角校准；双 Flash 页参数日志，元数据与数据 CRC、最后提交标记；上电不恢复运行命令。

这里实现的是 **经典 CANopen 的明确子集**。未实现 HexMeow 的 CAN-FD 长 PDO、模式 7/8、插补缓冲区，也不兼容 M5 原厂私有 29 位 CAN 协议。I²C、OLED 和 RGB 功能暂未启用；不提供缺少传感器的驱动器/电机温度读数。尚未进行 CiA 一致性认证。

## 阅读顺序

1. [硬件与引脚](docs/hardware.md)：原理图连接、采样窗口和电流尺度。
2. [架构](docs/architecture.md)：模块生命周期、并发所有权、实时路径和持久化。
3. [CANopen 协议](docs/protocol.md) / [对象字典](docs/object-dictionary.md)。
4. [实机调试](docs/bringup.md)：先观测与校准，再低电流闭环。
5. [构建与验证](docs/build.md) / [本次验证记录](docs/validation.md)。

`tools/rollercan_cli.py` 是无额外 Python 依赖的 Linux SocketCAN SDO 工具，`inspect` 只读取状态。`tools/vendor.py --check-lts` 检查最新 LTS；`--update-freertos-lts` 显式更新本地内核及锁文件，之后需重新测试。GitHub 工作流提供构建测试及每周 LTS 检查，更新不会绕过版本锁悄悄进入固件。

自有代码采用 MIT，第三方及 ST 生成代码保留各自许可，见 [依赖说明](ThirdParty/README.md)。
