# 本地依赖

| 组件 | 来源 / 版本 | 许可 | 本地内容 |
|---|---|---|---|
| FreeRTOS-Kernel | [V11.3.1](https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/V11.3.1)，[202604.01-LTS manifest](https://github.com/FreeRTOS/FreeRTOS-LTS/blob/202604.01-LTS/manifest.yml) | MIT，`FreeRTOS-Kernel/LICENSE.md` | 原始 kernel C 文件、include、GCC ARM_CM4F port；不包含 heap 实现 |
| CANopenNode | [v4.1](https://github.com/CANopenNode/CANopenNode/tree/v4.1) | Apache-2.0，`CANopenNode/LICENSE` | 上游源码与许可；只编译本项目需要的 301 服务 |
| STM32 HAL / CMSIS Device | [STM32CubeG4 V1.6.3](https://github.com/STMicroelectronics/STM32CubeG4/tree/v1.6.3) | 见原组件 LICENSE 与 `STM32CubeG4-Package_license.md` | CubeMX 复制到 `Drivers/` |
| CMSIS Core | 同一 G4 软件包 | Apache-2.0，`Drivers/CMSIS/LICENSE.txt` | 本地 Include，不使用 gitlink |

`vendor-lock.json` 记录 FreeRTOS 与 CANopenNode 的来源、tag、不可变 commit、原始压缩包哈希、逐文件哈希。所有收录文件与下载的对应上游文件逐字节比对过；裁剪只移除未使用目录，没有修改内核或协议栈源码。FreeRTOS 的完整 LTS manifest 另存为 `freertos-lts-manifest.yml`，其中其他组件只是上游套件信息，并未因此被引入本工程。

适配和配置均位于 `RTOS/`、`Protocol/CANopen/`，不要直接修改上游库。普通构建不会联网，也不会执行 `git submodule update`。

维护命令：`python tools/vendor.py --check`、`--check-lts`、`--update-freertos-lts`，说明见 `docs/build.md`。HAL/CMSIS 通过锁定的 CubeMX 软件包更新，保留原许可和生成器记录。

自有目录的 MIT 许可不覆盖 ST、Arm、FreeRTOS 或 CANopenNode 的原版权。根 README 中的许可只适用于本项目新增的业务、适配、工具与文档；生成文件和启动文件按原版权声明及上述组件许可使用。
