# 本地依赖

| 组件 | 来源 / 版本 | 许可 | 本地内容 |
|---|---|---|---|
| FreeRTOS-Kernel | [V11.3.1](https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/V11.3.1)，[202604.01-LTS manifest](https://github.com/FreeRTOS/FreeRTOS-LTS/blob/202604.01-LTS/manifest.yml) | MIT，`FreeRTOS-Kernel/LICENSE.md` | 原始 kernel C 文件、include、GCC ARM_CM4F port；不包含 heap 实现 |
| CANopenNode | [v4.1](https://github.com/CANopenNode/CANopenNode/tree/v4.1) | Apache-2.0，`CANopenNode/LICENSE` | 上游源码与许可；只编译本项目需要的 301 服务 |
| STM32 HAL / CMSIS Device | [STM32CubeG4 V1.6.3](https://github.com/STMicroelectronics/STM32CubeG4/tree/v1.6.3)：HAL v1.2.7、CMSIS Device v1.2.6 | 见 `Drivers/STM32G4xx_HAL_Driver/LICENSE.md`、`Drivers/CMSIS/Device/ST/STM32G4xx/LICENSE.md` 与 `STM32CubeG4-Package_license.md` | 与 CubeMX 的“仅复制所需库”生成集合一致，保留上游许可和说明；普通源码文件 |
| CMSIS Core | 同一 G4 软件包 | Apache-2.0，`Drivers/CMSIS/LICENSE.txt` | 本地 Include，不使用 gitlink |

`vendor-lock.json` 记录 FreeRTOS、CANopenNode、HAL 与 CMSIS Device 的来源、版本、不可变 commit、原始压缩包哈希和逐文件哈希；现有 CMSIS Core Include 也记录来源 commit 与逐文件哈希。FreeRTOS/CANopenNode 保持原始字节哈希。ST 三个组件显式使用 `hash_mode: text-lf`：计算文件哈希时只将 CRLF 转成 LF，以兼容官方安装包、GitHub 和 CubeMX 生成器的换行差异，其他任何字节变化、缺失或新增文件仍会失败。所有原始压缩包 SHA256 继续按原始字节计算。裁剪只移除 CubeMX 不复制的未使用源码，没有修改库实现。FreeRTOS 的完整 LTS manifest 另存为 `freertos-lts-manifest.yml`，其中其他组件只是上游套件信息，并未因此被引入本工程。

迁移修复时发现 HAL、CMSIS Device 曾被误存成无 `.gitmodules` 的 gitlink，换机克隆只有空目录。原 gitlink 指向晚于发布包的开发提交（HAL `a6001282`、CMSIS Device `626ee412`），与 `.ioc` 锁定的 V1.6.3 不一致。现统一恢复为该官方发布包的 HAL [`f5929f43`](https://github.com/STMicroelectronics/stm32g4xx-hal-driver/tree/f5929f431f9effe45fbe18f5337e4753ced9ac92) 和 CMSIS Device [`25664ddc`](https://github.com/STMicroelectronics/cmsis-device-g4/tree/25664ddc3a7624ae9627ae8c4c672073dc5b2539)。两组提交的差异仅涉及本项目未编译的 SMARTCARD/UART 实现及 CMSIS 公共头文件注释；现有 CMSIS Core Include 全部与 V1.6.3 相同。

适配和配置均位于 `RTOS/`、`Protocol/CANopen/`，不要直接修改上游库。普通构建不会联网，也不会执行 `git submodule update`。

维护命令：`python tools/vendor.py --check`、`--check-lts`、`--update-freertos-lts`，说明见 `docs/build.md`。离线检查覆盖 `Drivers/` 与 `ThirdParty/`，并拒绝嵌套 Git 仓库和 Git 索引中的 gitlink，防止本机可编译而新克隆缺文件。HAL/CMSIS 通过锁定的 CubeMX 软件包更新；更改外设导致生成集合变化或升级库版本时，需同步审核锁文件，保留原许可和生成器记录。`.gitattributes` 对 `Drivers/` 使用 LF 文本，对 `ThirdParty/` 保留原始字节，避免正常 CubeMX 重生成出现整文件换行差异。

自有目录的 MIT 许可不覆盖 ST、Arm、FreeRTOS 或 CANopenNode 的原版权。根 README 中的许可只适用于本项目新增的业务、适配、工具与文档；生成文件和启动文件按原版权声明及上述组件许可使用。
