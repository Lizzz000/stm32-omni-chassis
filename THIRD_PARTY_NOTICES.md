# 第三方声明

本仓库同时包含项目自编代码和第三方组件。根目录 [LICENSE](LICENSE) 的 Apache License 2.0 **只覆盖下面列出的自编部分**，不把 HAL、CMSIS 或 CubeMX 生成模板重新许可为 Apache-2.0。

CubeMX 工程对应固件包 STM32Cube FW_F4 V1.28.3。各组件以本树中实际附带的许可文件和文件头为准。

## 项目自编代码

以下内容采用 Apache License 2.0：

- `Core/APP/`
- `Core/Inc/chassis_config.h`
- `Core/Inc/control_timing.h`
- 根目录 `CMakeLists.txt`、`CMakePresets.json`
- `README.md`、`docs/`
- `.gitignore`、`.github/`、本文件

`Core/Src/main.c` 和 `Core/Src/stm32f4xx_it.c` 等生成文件里，`USER CODE` 区域内的控制逻辑也属于项目自编代码。这些文件的其余部分仍是下面的 CubeMX 生成模板。

## STM32Cube HAL

路径：`Drivers/STM32F4xx_HAL_Driver/`

版权：STMicroelectronics。

`Drivers/STM32F4xx_HAL_Driver/LICENSE.txt` 写明：若组件在软件包之外、或没有随包提供许可条款，则适用 BSD-3-Clause。

https://opensource.org/licenses/BSD-3-Clause

单个 HAL 源文件头仍写着 ST 版权，并指向“该软件组件根目录中的 LICENSE 文件”；找不到该文件时按 AS-IS 提供。这里的组件许可是 HAL 目录中的 `LICENSE.txt`，不是本仓库根目录的 Apache 许可证。

## Arm CMSIS

路径：`Drivers/CMSIS/Include/`

版权：Arm Limited。

核心头文件带有 `SPDX-License-Identifier: Apache-2.0`。目录许可见 `Drivers/CMSIS/LICENSE.txt`，为 Apache License 2.0。

## ST CMSIS 器件文件

路径：`Drivers/CMSIS/Device/ST/STM32F4xx/`

版权：STMicroelectronics。

`Drivers/CMSIS/Device/ST/STM32F4xx/LICENSE.txt` 写明：若组件在软件包之外、或没有随包提供许可条款，则适用 Apache-2.0。器件头文件自身的文件头同时保留 ST 版权声明。

## CubeMX 生成文件与启动文件

版权：STMicroelectronics。

包括：

- `Core/Inc` 与 `Core/Src` 中由 CubeMX 生成的外设、中断和系统文件（`chassis_config.h`、`control_timing.h` 除外）
- `startup_stm32f407xx.s`
- `Core/Src/system_stm32f4xx.c`
- `Core/Src/syscalls.c`、`Core/Src/sysmem.c`
- `cmake/gcc-arm-none-eabi.cmake`、`cmake/starm-clang.cmake`、`cmake/stm32cubemx/`

这些文件头写明版权归 ST，并指向该软件组件根目录的 LICENSE；若没有随组件提供 LICENSE，则按 AS-IS 提供。该声明不把文件改到本仓库的 Apache-2.0 之下。

`STM32F407xx_FLASH.ld` 的文件头自带 BSD-3-Clause 全文，以该文件头为准。

`stm32_omni_chassis.ioc` 是本项目的 CubeMX 配置，随自编工程文件一并提供。
