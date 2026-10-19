#include "remote_control.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#if (REMOTE_CONTROL_DMA_BUFFER_SIZE < 2U) || (REMOTE_CONTROL_DMA_BUFFER_SIZE > 65535U)
#error "REMOTE_CONTROL_DMA_BUFFER_SIZE must be in 2..65535"
#endif
#if (REMOTE_CONTROL_RING_BUFFER_SIZE < 2U) || (REMOTE_CONTROL_RING_BUFFER_SIZE > 65535U)
#error "REMOTE_CONTROL_RING_BUFFER_SIZE must be in 2..65535"
#endif
#if (REMOTE_CONTROL_LINE_BUFFER_SIZE < 8U)
#error "REMOTE_CONTROL_LINE_BUFFER_SIZE must be at least 8"
#endif

static UART_HandleTypeDef *s_uart;
static uint8_t s_dma_buffer[REMOTE_CONTROL_DMA_BUFFER_SIZE];
static uint8_t s_ring_data[REMOTE_CONTROL_RING_BUFFER_SIZE];
static uint32_t s_ring_tick[REMOTE_CONTROL_RING_BUFFER_SIZE];
static volatile uint16_t s_ring_head;
static volatile uint16_t s_ring_tail;
static volatile uint32_t s_stream_generation;
static volatile uint8_t s_fault_pending;
static volatile uint8_t s_parser_reset;
static volatile uint8_t s_restart_pending;
static volatile uint8_t s_recovering;
static volatile uint8_t s_command_pending;
static RemoteControlCommand_t s_latest_command;

/* 行组装和浮点解析仅在主循环进行，中断中不调用 strtof。 */
static char s_line[REMOTE_CONTROL_LINE_BUFFER_SIZE];
static size_t s_line_length;
static uint8_t s_line_bad;
static uint8_t s_discard_until_newline;

static uint32_t RemoteControl_Lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void RemoteControl_Unlock(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static uint16_t RemoteControl_NextIndex(uint16_t index)
{
    index++;
    if (index >= REMOTE_CONTROL_RING_BUFFER_SIZE)
    {
        index = 0U;
    }
    return index;
}

/* 调用时须处于临界区：故障立即失效旧队列和旧命令。
 * generation 防止解析期间发生 UART 错误后，主循环又发布旧帧。
 */
static void RemoteControl_MarkFaultLocked(void)
{
    s_fault_pending = 1U;
    s_command_pending = 0U;
    s_ring_tail = s_ring_head;
    s_stream_generation++;
    s_parser_reset = 1U;
}

/* ReceiveToIdle 的 NORMAL 模式在 IDLE/TC 事件后停止接收，必须重新启动。
 * HT 不会停止接收，而且 Size 是累计值；关闭 HT 可以防止重复解析。
 * 本函数不执行阻塞式 Abort，可在接收完成中断中调用。
 */
static HAL_StatusTypeDef RemoteControl_StartReceiveLocked(void)
{
    HAL_StatusTypeDef status;

    if ((s_uart == NULL) || (s_uart->hdmarx == NULL) ||
        (s_uart->hdmarx->Init.Mode != DMA_NORMAL))
    {
        return HAL_ERROR;
    }

    status = HAL_UARTEx_ReceiveToIdle_DMA(s_uart, s_dma_buffer,
                                        (uint16_t)sizeof(s_dma_buffer));
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(s_uart->hdmarx, DMA_IT_HT);
    }
    return status;
}

static const char *RemoteControl_SkipSpace(const char *text)
{
    while ((*text == ' ') || (*text == '\t') || (*text == '\r'))
    {
        text++;
    }
    return text;
}

uint8_t RemoteControl_ParseLine(const char *line, RemoteControlCommand_t *command)
{
    const char *cursor;
    char *end;
    float values[3];
    uint8_t i;

    if ((line == NULL) || (command == NULL))
    {
        return 0U;
    }

    cursor = line;
    for (i = 0U; i < 3U; i++)
    {
        const char *number_start;
        cursor = RemoteControl_SkipSpace(cursor);
        /* strtof 会接受 LF 等空白，协议在这里限制为本行的空格/Tab/CR。 */
        if ((*cursor == '\0') || (*cursor == '\n') || (*cursor == '\v') ||
            (*cursor == '\f'))
        {
            return 0U;
        }
        /* 协议只接受十进制/科学计数法，排除 strtof 支持的十六进制浮点。 */
        number_start = cursor;
        if ((*number_start == '+') || (*number_start == '-'))
        {
            number_start++;
        }
        if ((number_start[0] == '0') &&
            ((number_start[1] == 'x') || (number_start[1] == 'X')))
        {
            return 0U;
        }
        errno = 0;
        values[i] = strtof(cursor, &end);
        if ((end == cursor) || (errno == ERANGE) || !isfinite(values[i]))
        {
            return 0U;
        }
        cursor = RemoteControl_SkipSpace(end);
        if (i < 2U)
        {
            if (*cursor != ',')
            {
                return 0U;
            }
            cursor++;
        }
        else if (*cursor != '\0')
        {
            return 0U;
        }
    }

    /* 超范围整帧拒绝，避免输入错误被限幅后继续运动。 */
    if ((fabsf(values[0]) > REMOTE_CONTROL_MAX_LINEAR_SPEED_MPS) ||
        (fabsf(values[1]) > REMOTE_CONTROL_MAX_LINEAR_SPEED_MPS) ||
        (fabsf(values[2]) > REMOTE_CONTROL_MAX_OMEGA_RAD_S))
    {
        return 0U;
    }

    command->x = values[0];
    command->y = values[1];
    command->w = values[2];
    command->received_tick = 0U;
    return 1U;
}

void RemoteControl_Init(UART_HandleTypeDef *huart)
{
    uint32_t primask = RemoteControl_Lock();

    s_uart = huart;
    s_ring_head = 0U;
    s_ring_tail = 0U;
    s_stream_generation = 0U;
    s_fault_pending = 0U;
    s_parser_reset = 0U;
    s_restart_pending = 0U;
    s_recovering = 0U;
    s_command_pending = 0U;
    s_line_length = 0U;
    s_line_bad = 0U;
    s_discard_until_newline = 0U;

    if (RemoteControl_StartReceiveLocked() != HAL_OK)
    {
        RemoteControl_MarkFaultLocked();
        s_restart_pending = 1U;
    }
    RemoteControl_Unlock(primask);
}

void RemoteControl_UartRxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint32_t tick;
    uint32_t primask;
    uint16_t i;

    if ((huart == NULL) || (huart != s_uart))
    {
        return;
    }
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
    {
        return;
    }

    tick = HAL_GetTick();
    primask = RemoteControl_Lock();
    if ((s_recovering != 0U) || (s_restart_pending != 0U))
    {
        RemoteControl_Unlock(primask);
        return;
    }

    if ((size == 0U) || (size > sizeof(s_dma_buffer)))
    {
        RemoteControl_MarkFaultLocked();
    }
    else
    {
        for (i = 0U; i < size; i++)
        {
            uint16_t next = RemoteControl_NextIndex(s_ring_head);
            if (next == s_ring_tail)
            {
                /* 丢弃本次剩余数据，下次换行后重新同步，旧帧不得恢复运动。 */
                RemoteControl_MarkFaultLocked();
                break;
            }
            s_ring_data[s_ring_head] = s_dma_buffer[i];
            s_ring_tick[s_ring_head] = tick;
            s_ring_head = next;
        }
    }

    if (RemoteControl_StartReceiveLocked() != HAL_OK)
    {
        RemoteControl_MarkFaultLocked();
        s_restart_pending = 1U;
    }
    RemoteControl_Unlock(primask);
}

void RemoteControl_UartErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if ((huart == NULL) || (huart != s_uart))
    {
        return;
    }

    primask = RemoteControl_Lock();
    RemoteControl_MarkFaultLocked();
    s_restart_pending = 1U;
    /* 中断中只记录故障，阻塞式 AbortReceive 放到主循环处理。 */
    RemoteControl_Unlock(primask);
}

void RemoteControl_InvalidateStream(void)
{
    uint32_t primask = RemoteControl_Lock();

    /* 停车屏障也必须覆盖尚在 DMA 内的旧字节，禁止旧回调重新入队。
     * 此接口可由控制中断调用；阻塞 Abort 和硬件清理留给主循环处理。 */
    s_command_pending = 0U;
    s_ring_tail = s_ring_head;
    s_stream_generation++;
    s_parser_reset = 1U;
    s_restart_pending = 1U;
    RemoteControl_Unlock(primask);
}

static void RemoteControl_RecoverReceive(void)
{
    UART_HandleTypeDef *huart;
    HAL_StatusTypeDef status;
    uint32_t primask = RemoteControl_Lock();

    if ((s_restart_pending == 0U) || (s_uart == NULL) || (s_recovering != 0U))
    {
        RemoteControl_Unlock(primask);
        return;
    }
    huart = s_uart;
    s_recovering = 1U;
    RemoteControl_Unlock(primask);

    /* HAL DMA 错误不一定清 DMAR；AbortReceive 统一关闭旧接收。
     * 保持全局中断开启，HAL 的 DMA 超时仍可依赖 SysTick 正常退出。
     */
    status = HAL_UART_AbortReceive(huart);

    primask = RemoteControl_Lock();
    if (status == HAL_OK)
    {
        /* 读取 SR/DR 清除 ORE、FE、NE、PE、IDLE，丢弃错误残留字节。 */
        __HAL_UART_CLEAR_OREFLAG(huart);
        s_restart_pending = 0U;
        status = RemoteControl_StartReceiveLocked();
    }
    if (status != HAL_OK)
    {
        RemoteControl_MarkFaultLocked();
        s_restart_pending = 1U;
    }
    s_recovering = 0U;
    RemoteControl_Unlock(primask);
}

void RemoteControl_Process(void)
{
    uint32_t processed;

    RemoteControl_RecoverReceive();

    /* 一次最多处理一圈队列，连续输入时仍给其它主循环任务运行机会。 */
    for (processed = 0U; processed < REMOTE_CONTROL_RING_BUFFER_SIZE; processed++)
    {
        uint8_t byte;
        uint32_t tick;
        uint32_t generation;
        uint32_t primask = RemoteControl_Lock();

        if (s_parser_reset != 0U)
        {
            s_line_length = 0U;
            s_line_bad = 0U;
            s_discard_until_newline = 1U;
            s_parser_reset = 0U;
        }
        if ((s_restart_pending != 0U) || (s_recovering != 0U) ||
            (s_ring_head == s_ring_tail))
        {
            RemoteControl_Unlock(primask);
            break;
        }
        byte = s_ring_data[s_ring_tail];
        tick = s_ring_tick[s_ring_tail];
        generation = s_stream_generation;
        s_ring_tail = RemoteControl_NextIndex(s_ring_tail);
        RemoteControl_Unlock(primask);

        if (s_discard_until_newline != 0U)
        {
            if (byte == '\n')
            {
                s_discard_until_newline = 0U;
            }
            continue;
        }

        if (byte == '\n')
        {
            RemoteControlCommand_t command;
            uint8_t valid;
            s_line[s_line_length] = '\0';
            valid = (s_line_bad == 0U) ? RemoteControl_ParseLine(s_line, &command) : 0U;
            s_line_length = 0U;
            s_line_bad = 0U;
            if (valid != 0U)
            {
                command.received_tick = tick;
                primask = RemoteControl_Lock();
                /* 解析过程中若中断报告故障，本帧属于旧流，禁止发布。 */
                if ((generation == s_stream_generation) && (s_parser_reset == 0U) &&
                    (s_restart_pending == 0U))
                {
                    s_latest_command = command;
                    s_command_pending = 1U;
                }
                RemoteControl_Unlock(primask);
            }
            continue;
        }

        if (s_line_length >= sizeof(s_line) - 1U)
        {
            primask = RemoteControl_Lock();
            RemoteControl_MarkFaultLocked();
            RemoteControl_Unlock(primask);
            continue;
        }
        /* 二进制 NUL/控制字符不能截断成一个看似有效的 ASCII 指令。 */
        if (((byte < 0x20U) && (byte != '\t') && (byte != '\r')) || (byte > 0x7EU))
        {
            s_line_bad = 1U;
        }
        s_line[s_line_length++] = (char)byte;
    }
}

uint8_t RemoteControl_TakeCommand(RemoteControlCommand_t *command)
{
    uint8_t available = 0U;
    uint32_t primask;

    if (command == NULL)
    {
        return 0U;
    }
    primask = RemoteControl_Lock();
    if ((s_command_pending != 0U) && (s_fault_pending == 0U) &&
        (s_restart_pending == 0U) && (s_parser_reset == 0U))
    {
        *command = s_latest_command;
        s_command_pending = 0U;
        available = 1U;
    }
    RemoteControl_Unlock(primask);
    return available;
}

uint8_t RemoteControl_TakeFault(void)
{
    uint8_t fault;
    uint32_t primask = RemoteControl_Lock();
    fault = s_fault_pending;
    s_fault_pending = 0U;
    RemoteControl_Unlock(primask);
    return fault;
}
