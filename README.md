# RollerCAN C++23 / STM32G431

面向 **M5Stack Unit-RollerCAN V1.0 / STM32G431CBU6** 的可离线构建电机固件。业务层使用 C++23，TIM1 每个 PWM 周期启动编码器 DMA，ADC DMA 帧与同周期角度都就绪后执行 20 kHz FOC，每 20 帧执行 1 kHz 运动控制；FreeRTOS 承担 CANopen 和监督任务。

支持速度、单圈位置、多圈位置、力矩和 MIT 五种运动模式；MIT 提供非压缩与压缩两种指令格式。默认上电关桥，未校准时不接受闭环使能。完整的上电检查、校准、参数保存、PDO 配置、使能握手和停止流程见 [通信与运动控制协议](docs/protocol.md)。

CAN 默认速率为 **仲裁段 1 Mbit/s、数据段 5 Mbit/s**。PDO 按映射长度自动选择经典 CAN（≤8 B）或 CAN-FD+BRS（9–64 B）；SDO、NMT、SYNC、Heartbeat、EMCY 继续使用经典帧。速率可经 SDO 暂存、显式保存后重启应用。

## 依赖与工具

| 项目 | 版本 / 选择 |
|---|---|
| C++ | C++23；`std::expected`、`std::span`、静态容量容器、无异常、无 RTTI |
| FreeRTOS | **202604.01-LTS / Kernel V11.3.1**，按 2026-09-05 官方 LTS manifest 核实 |
| CANopen | CANopenNode **v4.1**；默认仲裁段 **1 Mbit/s**、FD 数据段 **5 Mbit/s**、Node-ID 1；长 PDO 为本项目 FD 扩展 |
| HAL / CMSIS | STM32CubeG4 **V1.6.3**，由 CubeMX 复制到本地 |
| CubeMX | **6.17.0**，仅修改外设/重新生成时需要；CMake 工程，HAL 时基 TIM6 |
| 编译器 | 已验证 Arm GNU **14.2.1 / 15.2.1**、STM32 插件 GCC **14.3.1**；主机 GCC 16.1 / Ubuntu GCC 13.3 |
| 构建工具 | CMake ≥3.25、Ninja、Python ≥3.10；测试需要支持 C++23 的主机 GCC |

外部库完整保存在 `ThirdParty/` 与 `Drivers/`，没有 Git submodule、FetchContent 或构建时下载。ETL 暂未引入：现有静态队列、数组和非拥有接口已满足容量及生命周期要求。

## 构建

将 Arm GNU **arm-none-eabi** 工具链的 `bin`、CMake、Ninja 和 Python 加入 PATH，再打开终端。`arm-none-linux-gnueabihf` 是 Linux 交叉编译器，不能替代这里的裸机工具链。构建已有源码不需要安装或运行 CubeMX。在工程根目录执行：

```powershell
./tools/build.ps1 -Preset Debug
./tools/build.ps1 -Preset Release
# 可选：本机逻辑测试，另需支持 C++23 的本机 GCC/g++
./tools/build.ps1 -Preset Host
```

Linux 或直接使用 CMake：

```sh
python3 tools/vendor.py --check
python3 tools/generate_od.py --check
python3 tools/check_project.py
cmake --preset Host
cmake --build --preset Host
ctest --preset Host
python3 -m unittest discover -s tools -p 'test_*.py'
cmake --preset Debug
cmake --build --preset Debug
python3 tools/check_project.py --elf build/Debug/canopen_foc.elf
```

输出：`build/Debug/` 或 `build/Release/` 内的 `.elf`、`.hex`、`.bin`、`.map`。复制了旧 `build/` 或切换编译器后，执行 `./tools/build.ps1 -Preset Debug -Fresh`（等价于先 `cmake --fresh --preset Debug`）。未提供自动烧录动作。

VS Code 使用安装了 STM32Cube 扩展的配置文件（Profile），打开工程根目录后选择 `Debug` 或 `Release` preset，再执行 CMake 配置/构建。仓库的 `cube-cmake` 设置依赖 STM32Cube 扩展提供的环境；普通 PowerShell 使用上面的 `build.ps1` 或标准 `cmake`。详见 [迁移与 VS Code 排错](docs/build.md#迁移与-vs-code-排错)。

## 生成与源码边界

使用 **CubeMX 6.17.0** 打开 `canopen_foc.ioc`，安装 **STM32CubeG4 V1.6.3** 软件包后再 Generate Code；旧版 CubeMX 可能拒绝这个新版 `.ioc`，不要通过降低文件头版本号绕过检查。自定义代码位于 `Application/`、`Platform/`、`Protocol/`、`RTOS/`；`Core/` 仅在 `USER CODE` 块保留 C/C++ 入口和紧急关闭调用。根 CMake、工具链和独立 linker script 由本项目维护。

命令行生成及完整约束见 [构建与重生成](docs/build.md)。不要在 CubeMX 中启用 FreeRTOS middleware，也不要覆盖现有自有 CMake 文件。使用已锁定的 G4 软件包生成；升级 CubeMX/HAL 后需要重跑检查。

## 功能与协议边界

- ADC DMA 采集三相电流与母线电压，异步 SPI DMA 读取 TLI5012B SSC 编码器；Clarke/Park、带限幅与抗积分饱和的 PI、SVPWM，TIM1 DMA burst 更新三相占空比，CORDIC 输入与结果也使用 DMA。
- CiA 402 风格状态机，单圈最短路径位置模式 -1（线上 `0xFF`）、多圈累计位置模式 1、速度模式 3、转矩模式 4、MIT 模式 5；快速停止、故障锁存与显式复位。原模式 1 的累计位置行为保持兼容。
- NMT、Heartbeat producer/consumer、EMCY、SDO、SYNC、4 RPDO / 4 TPDO 与动态映射；每个 PDO 最多 16 个映射项、64 B，自动选择经典帧或 FD+BRS 并填充到合法 DLC。
- 位置、速度使用 Float32 转数 / 转每秒；非压缩 MIT 支持完整长 PDO，压缩 MIT 使用 64 位布局。详见 [协议说明](docs/protocol.md) 和 [EDS](config/rollercan.eds)。
- 100 ms 有效运动命令超时、硬件故障脚直接关桥、过流/过压/欠压/温度/编码器/时序故障、任务健康监督与 IWDG。
- 显式电角校准；双 Flash 页参数日志，元数据与数据 CRC、最后提交标记；上电不恢复运行命令。
- 通信速率 `0x2001:02/:03` 可写，要求 Pre-operational、关桥和控制字 0；`0x1010:01` 保存后 NMT Reset 应用。旧 v1 参数保留校准和原 PDO 配置，以新默认速率迁移，显式保存后生成 v2 记录。

这里实现的是 **CANopen 经典服务子集与本项目的 CAN-FD 长 PDO 扩展**，不是完整 CiA 1301 CANopen FD。未实现模式 7/8、插补缓冲区，也不兼容 M5 原厂私有 29 位 CAN 协议。I²C、OLED 和 RGB 功能暂未启用；不提供缺少传感器的驱动器/电机温度读数。尚未进行 CiA 一致性认证。

## 阅读顺序

1. [硬件与引脚](docs/hardware.md)：原理图连接、采样窗口和电流尺度。
2. [架构](docs/architecture.md)：模块生命周期、并发所有权、实时路径和持久化。
3. [通信与运动控制协议](docs/protocol.md)：从初始化、校准到各模式运行、停止与故障恢复。
4. [对象字典](docs/object-dictionary.md)：完整对象索引、类型、默认值和 PDO 权限。
5. [构建与维护](docs/build.md)：工具配置、CubeMX 重生成、主机测试和依赖管理。

`tools/rollercan_cli.py` 是无额外 Python 依赖的 Linux SocketCAN 工具，提供经典 SDO、PDO 映射与自动 FD 发送、帧监听和通信参数配置；`inspect` 只读取状态。`tools/vendor.py --check-lts` 检查最新 LTS；`--update-freertos-lts` 显式更新本地内核及锁文件，之后需重新测试。GitHub 工作流提供构建测试及每周 LTS 检查，更新不会绕过版本锁悄悄进入固件。

`Bench` 构建提供 SWD 邮箱，可用 `python tools/stlink_bench.py --help` 查看工具命令；它代替物理 CAN 传输，不能验证 CAN-FD 长帧。正式 CAN 通信使用 Debug/Release。多圈位置只在本次上电期间累计；力矩系数默认仍为 KV 换算估值。

自有代码采用 MIT，第三方及 ST 生成代码保留各自许可，见 [依赖说明](ThirdParty/README.md)。
