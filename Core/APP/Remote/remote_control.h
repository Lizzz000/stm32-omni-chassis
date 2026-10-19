#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 串口发送 ASCII 文本：x,y,w\n；也允许空格、Tab 和 CRLF。
 * x：前进速度，y：向左平移速度，单位 m/s；w：角速度，单位 rad/s。
 * 每帧必须以换行结束，例如 0.15,0,0\n；0,0,0\n 表示停车。
 * 持续运动建议每 10 ms（100Hz）发一帧，主控制逻辑按下面的超时参数停车。
 * 各参数可以在本文件修改，也可以由编译器 -D 参数覆盖。
 */
#ifndef REMOTE_CONTROL_TIMEOUT_MS
#define REMOTE_CONTROL_TIMEOUT_MS 500U
#endif

#ifndef REMOTE_CONTROL_MAX_LINEAR_SPEED_MPS
#define REMOTE_CONTROL_MAX_LINEAR_SPEED_MPS 0.5f
#endif

#ifndef REMOTE_CONTROL_MAX_OMEGA_RAD_S
#define REMOTE_CONTROL_MAX_OMEGA_RAD_S 2.0f
#endif

#ifndef REMOTE_CONTROL_DMA_BUFFER_SIZE
#define REMOTE_CONTROL_DMA_BUFFER_SIZE 128U
#endif

#ifndef REMOTE_CONTROL_RING_BUFFER_SIZE
#define REMOTE_CONTROL_RING_BUFFER_SIZE 512U
#endif

#ifndef REMOTE_CONTROL_LINE_BUFFER_SIZE
#define REMOTE_CONTROL_LINE_BUFFER_SIZE 96U
#endif

typedef struct
{
    float x;
    float y;
    float w;
    /* 换行所在 DMA 接收事件的 HAL tick，不能用主循环解析时间刷新超时。 */
    uint32_t received_tick;
} RemoteControlCommand_t;

/* 在 UART 和 DMA 初始化完成后调用；RX DMA 必须配置成 DMA_NORMAL。 */
void RemoteControl_Init(UART_HandleTypeDef *huart);

/* 主循环调用：解析完整行，并恢复 UART 接收异常。 */
void RemoteControl_Process(void);

/* 取消旧接收流：清除排队/待发布命令与半帧，并请求主循环重启 DMA。
 * 可从控制中断调用，不执行阻塞 Abort；恢复后先丢弃至 LF 边界，
 * 此后的完整新帧才可接管。超时停车、保护进入及解除时各调用一次。 */
void RemoteControl_InvalidateStream(void);

/* 从 HAL UART 回调转发；仅匹配 Init 传入的 UART，不影响 IMU/日志串口。 */
void RemoteControl_UartRxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void RemoteControl_UartErrorCallback(UART_HandleTypeDef *huart);

/* 控制入口必须先检查 TakeFault 并停车，再获取新命令；可从控制定时器调用。
 * 多个完整有效帧到达时只返回最新帧；取出后该帧不会重复返回。
 * 缓冲溢出、超长行或 UART 异常会清除旧命令，并丢弃至下一个换行边界；
 * 之后的完整新有效帧才允许恢复遥控。TakeFault 返回 1 表示需要停车。
 */
uint8_t RemoteControl_TakeCommand(RemoteControlCommand_t *command);
uint8_t RemoteControl_TakeFault(void);

/* 纯解析接口：line 不包含结尾 LF；成功返回 1，received_tick 置零。
 * 必须恰好包含三个有限的十进制浮点数，可用科学计数法；
 * 十六进制浮点、超范围、缺字段、额外字段均拒绝。
 * 普通坏帧被忽略，不刷新有效指令时间；此前运动最多保持至遥控超时。
 */
uint8_t RemoteControl_ParseLine(const char *line, RemoteControlCommand_t *command);

#ifdef __cplusplus
}
#endif

#endif
