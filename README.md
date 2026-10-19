# stm32-omni-chassis

基于 **STM32F407VET6** 的三轮全向底盘控制固件，采用 STM32 HAL、C11 和 CMake。上位机通过串口发送速度帧，即可控制底盘前进、横移和旋转。

本项目经过 AI 辅助修改，可能仍存在潜在问题。上车前请自行核对轮向、编码器和保护逻辑。

[构建固件](#构建固件) · [串口控制](#串口控制) · [目录结构](#目录结构) · [详细使用指南](docs/control-guide.md)

## 功能

- 三轮全向运动学、编码器反馈、增量式 PI 与前馈控制。
- IMU 航向保持、单次相对角度转向。
- 串口速度指令解析、失联停车、超速与失控锁存保护。
- 可配置的上电单次动作；默认关闭，初始化后保持停止。

## 硬件配置

| 项目 | 配置 |
| --- | --- |
| 主控 | STM32F407VET6，168 MHz |
| 底盘 | 三轮全向：A 后轮、B 右前轮、C 左前轮 |
| 电机驱动 | AT8236，双 PWM，20 kHz |
| 编码器 | TIM4 / TIM5 / TIM8 |
| IMU | JY901P，USART2 |
| 控制周期 | TIM7，10 ms / 100 Hz |

完整引脚映射及方向约定见 [使用指南](docs/control-guide.md#硬件映射)。更换电机、编码器或轮系后，需重新配置机械参数、方向和控制增益。

## 构建固件

准备 CMake 3.22+、Ninja 和匹配主机架构的 Arm GNU Toolchain，并把 `arm-none-eabi-gcc` 所在目录加入 `PATH`。在仓库根目录执行：

```sh
cmake --preset Debug
cmake --build --preset Debug --parallel
```

生成文件：

```text
build/Debug/stm32_omni_chassis.elf
build/Debug/stm32_omni_chassis.map
```

优化构建使用 `Release` 预设。CubeMX 配置入口为 `stm32_omni_chassis.ioc`。其它构建选项见 [使用指南](docs/control-guide.md#构建选项)。推送到 GitHub 后，Actions 用同一 Debug 预设编译固件。

## 串口控制

| 用途 | 接口 | 参数 |
| --- | --- | --- |
| 速度控制 | USART1：PA9 TX / PA10 RX | 115200，8N1 |
| IMU | USART2：PA2 TX / PA3 RX | 115200，8N1 |
| 调试输出 | USART3：PD8 TX / PD9 RX | 115200，8N1 |

上位机 TX 接 MCU RX，并共地。发送 ASCII `x,y,w`，**每帧以实际 LF 换行结束**，也支持 CRLF。

| 发送内容 | 动作 |
| --- | --- |
| `0.15,0,0` | 前进 0.15 m/s |
| `0,0.15,0` | 向左横移 0.15 m/s |
| `0,0,0.5` | 逆时针旋转 0.5 rad/s |
| `0,0,0` | 停车 |

`x/y` 单位为 m/s，正方向分别为前方和左侧；`w` 单位为 rad/s，逆时针为正。默认范围为 x/y 各 ±0.5 m/s、w ±2 rad/s。建议每 10 ms 连续发送，500 ms 未收到完整有效新帧则停车；异常数值和字段数量错误整帧拒绝。

串口速度模式不依赖 IMU。超时或故障恢复后，先发送一个 LF 同步边界，再发送完整速度帧；连续发送时，恢复后的首帧可能仅用于同步。保持航向、相对转向、完整协议与故障恢复说明见 [使用指南](docs/control-guide.md)。

## 目录结构

```text
Core/
  APP/                     Encoder、JY901P、Motor、Omni、PID、Remote、Safety
  Inc/chassis_config.h     速度环与航向环参数
  Inc/control_timing.h     控制周期
  Src/main.c               启动、命令仲裁和控制调度
  Src/stm32f4xx_it.c        中断转发与故障刹车
Drivers/                   HAL / CMSIS
cmake/                     工具链与 CubeMX 构建配置
docs/control-guide.md      控制流程、参数与接口使用说明
stm32_omni_chassis.ioc      CubeMX 工程
```

控制链路为：**TIM7 采样编码器 → 保护检查与指令仲裁 → 运动模式 → 逆运动学 → 三轮 PI 与前馈 → PWM 输出**。主循环处理串口解析、接收恢复和日志，控制计算由 TIM7 调度。

## 使用说明

控制参数集中在 `chassis_config.h`，机械尺寸、轮向和前馈在 `Omni.h`，遥控范围与超时在 `remote_control.h`。修改周期时需同步换算离散控制参数。

保护锁存后取消旧指令；航向模式在 IMU 失效时停止。当前未实现堵转、编码器断线/极性异常的完整检测，也未启用独立看门狗。接入实车后需确认实际轮向、编码器极性、PI 收敛与转向效果。串口速度指令没有鉴权，接到 USART1 的设备即可驱动底盘。

## 许可证

`Core/APP`、`chassis_config.h`、`control_timing.h`、文档和本仓库的 CMake 预设采用 [Apache License 2.0](LICENSE)。HAL、CMSIS、启动文件和 CubeMX 生成代码保留各自原许可，范围见 [第三方声明](THIRD_PARTY_NOTICES.md)。
