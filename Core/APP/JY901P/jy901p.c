/**
  ******************************************************************************
  * @file    jy901p.c
  * @brief   JY901P 九轴姿态传感器驱动
  *          USART2 DMA + 空闲中断接收, WIT 私有协议解析
  ******************************************************************************
  * 接收流程:
  *   1. JY901P_Init 启动 HAL_UARTEx_ReceiveToIdle_DMA
  *   2. 传感器每轮数据输出结束后产生空闲(IDLE)中断
  *   3. HAL 在 USART2_IRQHandler 中调用 HAL_UARTEx_RxEventCallback
  *   4. 回调先复制本批数据，再重启 DMA，并逐字节解析副本
  *   5. 0x53 角度帧校验通过后更新 roll/pitch/yaw
  *
  * 数据帧格式(固定 11 字节, 无长度字段):
  *   0x55 | TYPE | D1L D1H D2L D2H D3L D3H D4L D4H | SUM
  *   SUM = (0x55 + TYPE + 8 个数据字节) 的低 8 位
  *   16 位数据为小端带符号: DATA = (int16_t)(高字节<<8 | 低字节)
  *
  * 写指令(5 字节, 无校验和, 修改前必须先解锁):
  *   FF AA [ADDR] [DATAL] [DATAH]
  * 解锁: FF AA 69 88 B5      保存: FF AA 00 00 00
  * 注: 解锁后所有指令须在 10S 内完成, 否则模块自动上锁
  *
  * 注: 解析在中断上下文执行; roll/pitch/yaw 为 32 位浮点,
  *     Cortex-M4 单次读写原子, 主循环可直接读取。
  ******************************************************************************
  */
#include <string.h>
#include "jy901p.h"

/* 当前注册的驱动实例(单实例, 中断回调转发用) */
static JY901P_t *s_imu = NULL;
static uint8_t s_rx_recovering = 0U;

/* 串口错误会使字节流不连续，残帧和旧角度都不能继续作为有效输入。 */
static void JY901P_InvalidateReceive(JY901P_t *imu)
{
    imu->has_valid_angle = 0U;
    imu->last_update_ms = 0U;
    imu->rx_state = JY901P_RX_WAIT_HEAD;
    imu->frame_id = 0U;
    imu->frame_sum = 0U;
    imu->frame_cnt_data = 0U;
    memset(imu->frame_data, 0, sizeof(imu->frame_data));
}

/* 解锁指令: 修改配置寄存器前必须先发送 (FF AA 69 88 B5) */
static void JY901P_Unlock(JY901P_t *imu)
{
    const uint8_t unlock[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};

    HAL_UART_Transmit(imu->huart, (uint8_t *)unlock, sizeof(unlock), 100);
}

/* 写寄存器: FF AA [REG] [D0] [D1], 无校验和 */
static void JY901P_WriteReg(JY901P_t *imu, uint8_t reg, uint8_t d0, uint8_t d1)
{
    uint8_t cmd[5] = {0xFF, 0xAA, reg, d0, d1};

    HAL_UART_Transmit(imu->huart, cmd, sizeof(cmd), 100);
}

/* 处理一帧完整数据(校验和已通过) */
static void JY901P_HandleFrame(JY901P_t *imu)
{
    /* 角度帧 0x53: Roll/Pitch/Yaw 小端 int16(带符号), 原始值 32768 对应 180° */
    if (imu->frame_id == JY901P_ID_ANGLE)
    {
        imu->roll  = (float)(int16_t)((uint16_t)imu->frame_data[1] << 8 | imu->frame_data[0]) / 32768.0f * 180.0f;
        imu->pitch = (float)(int16_t)((uint16_t)imu->frame_data[3] << 8 | imu->frame_data[2]) / 32768.0f * 180.0f;
        imu->yaw   = (float)(int16_t)((uint16_t)imu->frame_data[5] << 8 | imu->frame_data[4]) / 32768.0f * 180.0f;
        imu->last_update_ms = HAL_GetTick();
        imu->frame_count++;
        imu->has_valid_angle = 1U;
    }
}

/* 逐字节帧解析状态机 */
static void JY901P_ParseByte(JY901P_t *imu, uint8_t byte)
{
    switch (imu->rx_state)
    {
    case JY901P_RX_WAIT_HEAD:               /* 等待帧头 0x55 */
        if (byte == 0x55)
            imu->rx_state = JY901P_RX_WAIT_TYPE;
        break;

    case JY901P_RX_WAIT_TYPE:               /* 帧类型, 校验和从帧头开始累计 */
        imu->frame_id = byte;
        imu->frame_sum = 0x55 + byte;
        imu->frame_cnt_data = 0;
        imu->rx_state = JY901P_RX_WAIT_DATA;
        break;

    case JY901P_RX_WAIT_DATA:               /* 数据区: 固定 8 字节 */
        imu->frame_data[imu->frame_cnt_data++] = byte;
        imu->frame_sum += byte;
        if (imu->frame_cnt_data >= JY901P_FRAME_DATA_LEN)
            imu->rx_state = JY901P_RX_WAIT_SUM;
        break;

    case JY901P_RX_WAIT_SUM:                /* 校验和: 所有字节和的低 8 位 */
        if ((uint8_t)imu->frame_sum == byte)
            JY901P_HandleFrame(imu);
        imu->rx_state = JY901P_RX_WAIT_HEAD;   /* 字节流连续, 回到帧头状态 */
        break;

    default:
        imu->rx_state = JY901P_RX_WAIT_HEAD;
        break;
    }
}

/* 启动 NORMAL DMA 空闲接收；失败时清错误标志并重试一次。
 * 只处理 IDLE/TC：HT 的 Size 是累计半缓冲长度，不能作为新一批数据重放。 */
static void JY901P_RestartReceive(JY901P_t *imu)
{
    HAL_StatusTypeDef status;

    /* 未在 CubeMX 配置 RX DMA 时 hdmarx 为空, 直接返回,
     * 防止 ReceiveToIdle_DMA 解引用空指针导致 HardFault */
    if (imu->huart->hdmarx == NULL)
        return;

    status = HAL_UARTEx_ReceiveToIdle_DMA(imu->huart, imu->dma_rx_buf, JY901P_DMA_BUF_SIZE);
    if (status != HAL_OK)
    {
        __HAL_UART_CLEAR_OREFLAG(imu->huart);
        __HAL_UART_CLEAR_NEFLAG(imu->huart);
        __HAL_UART_CLEAR_FEFLAG(imu->huart);
        __HAL_UART_CLEAR_PEFLAG(imu->huart);
        status = HAL_UARTEx_ReceiveToIdle_DMA(imu->huart, imu->dma_rx_buf, JY901P_DMA_BUF_SIZE);
    }
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(imu->huart->hdmarx, DMA_IT_HT);
    }
}

/**
  * @brief  UART 接收事件回调(空闲/传输完成触发)
  * @note   由应用层 HAL_UARTEx_RxEventCallback 转发；在注册 UART 的
  *         全局中断或 RX DMA 中断上下文中执行，仅处理该 UART。
  */
void JY901P_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    static uint8_t parse_buf[JY901P_DMA_BUF_SIZE];
    uint16_t i;

    /* 与 JY901P_Init 注册的 UART(现为 USART2)比对, 不再硬编码 USART3 */
    if ((s_imu == NULL) || (huart != s_imu->huart))
        return;

    /* Abort 完成前丢弃旧接收事件；HT 的 Size 是累计长度，不作为新数据解析。 */
    if ((s_rx_recovering != 0U) ||
        (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT))
        return;

    if (Size > JY901P_DMA_BUF_SIZE)
        Size = JY901P_DMA_BUF_SIZE;

    /* 先复制本批次数据，再重启DMA。不能直接重启到dma_rx_buf后再解析，
     * 否则新到的字节会覆盖正在解析的旧数据。 */
    memcpy(parse_buf, s_imu->dma_rx_buf, Size);
    JY901P_RestartReceive(s_imu);

    for (i = 0; i < Size; i++)
        JY901P_ParseByte(s_imu, parse_buf[i]);
}

/**
  * @brief  UART 错误回调(帧错误/噪声/溢出等)
  * @note   先失效残帧和旧角度，再异步终止接收；完成回调中重新启动 DMA。
  *         DMA FE/DME 错误可能仍保留活动流，不能只清标志后直接重启。
  */
void JY901P_UartErrorCallback(UART_HandleTypeDef *huart)
{
    /* 仅恢复注册的 JY901P UART（本项目为 USART2）。 */
    if ((s_imu == NULL) || (huart != s_imu->huart))
        return;

    JY901P_InvalidateReceive(s_imu);
    if (s_rx_recovering != 0U)
        return;

    s_rx_recovering = 1U;
    /* USART2 与 RX DMA 同级中断；阻塞式 Abort 可能等不到 SysTick 超时。
     * 此 API 可同步或异步调用完成回调，因此必须先设置恢复标志。
     * 若启动 Abort 失败，保持数据无效并忽略旧事件，等待重新初始化。
     */
    if (HAL_UART_AbortReceive_IT(huart) != HAL_OK)
        return;
}

/* 由应用层唯一的 HAL_UART_AbortReceiveCpltCallback 转发。 */
void JY901P_UartAbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
    if ((s_imu == NULL) || (huart != s_imu->huart) ||
        (s_rx_recovering == 0U))
        return;

    /* 读取 SR/DR 清除 ORE、FE、NE、PE、IDLE，并丢弃故障期间的残留字节。 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    s_rx_recovering = 0U;
    JY901P_RestartReceive(s_imu);
}

/**
  * @brief  初始化 JY901P
  * @param  imu   驱动实例
  * @param  huart USART2 句柄
  * @note   必须在 MX_USART2_UART_Init 之后调用;
  *         空闲中断依赖 USART2 全局中断, 需在 CubeMX 中使能
  */
void JY901P_Init(JY901P_t *imu, UART_HandleTypeDef *huart)
{
    memset(imu, 0, sizeof(JY901P_t));
    imu->huart = huart;
    s_imu = imu;
    s_rx_recovering = 0U;

    /* 启动 DMA + 空闲中断接收。
     * 未配置 RX DMA 时跳过启动(不 HardFault), 应用层可用
     * JY901P_IsFresh() 判断数据是否真正到达。 */
    JY901P_RestartReceive(imu);
}

/* 获取姿态角(单位: °) */
float JY901P_GetRoll(const JY901P_t *imu)
{
    return imu->roll;
}

float JY901P_GetPitch(const JY901P_t *imu)
{
    return imu->pitch;
}

float JY901P_GetYaw(const JY901P_t *imu)
{
    float yaw = imu->yaw - imu->yaw_offset;

    if (yaw > 180.0f)
        yaw -= 360.0f;
    else if (yaw < -180.0f)
        yaw += 360.0f;
    return yaw;
}

/* 数据新鲜度检查 */
uint8_t JY901P_IsFresh(const JY901P_t *imu, uint32_t timeout_ms)
{
    /* 首帧未到返回 0: 初始化后 last_update_ms=0 与 HAL tick 同步起步,
     * 开机前 500ms 会被 (tick-0)<=timeout 误判为"新鲜", 破坏
     * "上电首帧未到不启动转向/不锁存航向"的契约 */
    if (!imu->has_valid_angle)
        return 0U;
    return (HAL_GetTick() - imu->last_update_ms) <= timeout_ms;
}

/* 将当前偏航角设为软件零位 */
void JY901P_ResetYaw(JY901P_t *imu)
{
    imu->yaw_offset = imu->yaw;
}

/**
  * @brief  开始校准(自动发送解锁指令并延时)
  * @param  type 校准类型, 见 JY901P_CalibType
  * @note   校准模式写入 CALSW(0x01) 寄存器;
  *         解锁后延时 200ms 再发校准指令(见协议文档校准流程)
  */
void JY901P_CaliStart(JY901P_t *imu, JY901P_CalibType type)
{
    JY901P_Unlock(imu);                                     /* 校准前必须先解锁 */
    HAL_Delay(200);                                         /* 解锁后延时 200ms */
    JY901P_WriteReg(imu, JY901P_REG_CALSW, (uint8_t)type, 0x00);
}

/**
  * @brief  结束校准并保存到 Flash
  * @param  type 校准类型, 与 JY901P_CaliStart 保持一致
  * @note   陀螺仪手动校准(JY901P_CALIB_GYRO_START)结束时写入 0x0D,
  *         其余类型写入 0x00 退出校准; 磁场校准等耗时长于 10S,
  *         需重新解锁后再退出
  */
void JY901P_CaliEnd(JY901P_t *imu, JY901P_CalibType type)
{
    uint8_t end_mode = (type == JY901P_CALIB_GYRO_START) ? JY901P_CALIB_GYRO_END : 0x00;

    JY901P_Unlock(imu);                                     /* 校准过程可能超过 10S, 重新解锁 */
    HAL_Delay(200);
    JY901P_WriteReg(imu, JY901P_REG_CALSW, end_mode, 0x00); /* 退出校准 */
    HAL_Delay(100);                                         /* 退出校准后延时 100ms */
    JY901P_WriteReg(imu, JY901P_REG_SAVE, 0x00, 0x00);      /* 保存到 Flash */
}

/**
  * @brief  执行一次完整的陀螺仪零偏校准
  * @param  sample_ms 静止采样时间(ms), 建议不小于 5000ms
  * @note   校准期间模块必须保持完全静止; 校准结果保存到模块 Flash。
  *         校准完成后会重新打开陀螺仪自动校准(0x61=0)。
  */
void JY901P_CalibrateGyro(JY901P_t *imu, uint32_t sample_ms)
{
    if ((imu == NULL) || (imu->huart == NULL))
        return;

    /* 手动校准前关闭自动校准，防止两种校准同时修改零偏。 */
    JY901P_SetGyroAutoCal(imu, 0);
    HAL_Delay(200);

    JY901P_CaliStart(imu, JY901P_CALIB_GYRO_START);
    HAL_Delay(sample_ms);
    JY901P_CaliEnd(imu, JY901P_CALIB_GYRO_START);

    /* 校准完成后恢复自动校准并保存。此前漏掉这一步: 0x61=1 被永久
     * 保存, 之后每次上电陀螺零偏不再自动采集, 零偏被积分 → yaw 持续
     * 自增(磁力计受干扰时尤其明显)。 */
    JY901P_SetGyroAutoCal(imu, 1);
}

/**
  * @brief  通用配置: 解锁 + 写寄存器 + 保存到 Flash
  * @param  reg    寄存器地址
  * @param  value  寄存器值(低 8 位)
  * @note   所有配置指令须在解锁后 10S 内完成
  */
void JY901P_SetConfigReg(JY901P_t *imu, uint8_t reg, uint8_t value)
{
    JY901P_Unlock(imu);
    HAL_Delay(200);                                         /* 解锁后延时 200ms */
    JY901P_WriteReg(imu, reg, value, 0x00);
    JY901P_WriteReg(imu, JY901P_REG_SAVE, 0x00, 0x00);      /* 保存到 Flash */
}

/**
  * @brief  设置姿态解算算法(九轴/六轴)并保存
  * @param  algo 算法类型, 见 JY901P_Algorithm
  */
void JY901P_SetAlgorithm(JY901P_t *imu, JY901P_Algorithm algo)
{
    JY901P_SetConfigReg(imu, JY901P_REG_ALGORITHM, (uint8_t)algo);
}

/**
  * @brief  设置串口波特率为 115200 并保存
  * @note   按协议文档"修改波特率"流程:
  *         解锁 -> 写波特率(模块立即切换) -> 重新解锁 -> 保存;
  *         前提是 USART2 已按 115200 配置(见 jy901p.h)
  */
void JY901P_SetBaud115200(JY901P_t *imu)
{
    JY901P_Unlock(imu);
    HAL_Delay(200);
    JY901P_WriteReg(imu, JY901P_REG_BAUD, JY901P_BAUD_115200, 0x00);
    HAL_Delay(50);                                          /* 等待模块切换波特率 */

    /* 模块切换波特率后重新解锁并保存(USART2 已按 115200 配置) */
    JY901P_Unlock(imu);
    HAL_Delay(200);
    JY901P_WriteReg(imu, JY901P_REG_SAVE, 0x00, 0x00);
}

/**
  * @brief  陀螺仪自动校准开关
  * @param  enable  1 = 开启自动校准(默认), 0 = 关闭
  * @note   0x61 寄存器: 0x00 = 开启(默认), 0x01 = 关闭;
  *         手动校准陀螺仪前需先关闭自动校准
  */
void JY901P_SetGyroAutoCal(JY901P_t *imu, uint8_t enable)
{
    JY901P_SetConfigReg(imu, JY901P_REG_GYROAUTOCAL, enable ? 0x00 : 0x01);
}

/**
  * @brief  设置输出速率(回传频率)并保存
  * @param  rate  速率取值, 见 JY901P_RATE_* 宏
  * @note   RRATE(0x03) 寄存器; 默认 10Hz
  */
void JY901P_SetOutputRate(JY901P_t *imu, uint8_t rate)
{
    JY901P_SetConfigReg(imu, JY901P_REG_RRATE, rate);
}

/**
  * @brief  设置输出内容(各帧开关)并保存
  * @param  mask  位掩码, 用 JY901P_RSW_* 宏按位或组合
  * @note   RSW(0x02) 为 16 位寄存器, 低 8 位在前、高 8 位在后
  */
void JY901P_SetOutputContent(JY901P_t *imu, uint16_t mask)
{
    JY901P_Unlock(imu);
    HAL_Delay(200);                                             /* 解锁后延时 200ms */
    JY901P_WriteReg(imu, JY901P_REG_RSW,
                    (uint8_t)(mask & 0xFF), (uint8_t)(mask >> 8));
    JY901P_WriteReg(imu, JY901P_REG_SAVE, 0x00, 0x00);          /* 保存到 Flash */
}

/**
  * @brief  设置安装方向并保存
  * @param  vertical  0 = 水平安装(默认), 1 = 垂直安装(Y 轴箭头朝上)
  * @note   ORIENT(0x23) 寄存器
  */
void JY901P_SetOrientation(JY901P_t *imu, uint8_t vertical)
{
    JY901P_SetConfigReg(imu, JY901P_REG_ORIENT, vertical ? 0x01 : 0x00);
}

/**
  * @brief  设置陀螺仪量程并保存
  * @param  range  量程取值, 见 JY901P_GYRORANGE_* 宏
  * @note   GYRORANGE(0x20) 寄存器; 默认 2000°/s, 一般不可改
  */
void JY901P_SetGyroRange(JY901P_t *imu, uint8_t range)
{
    JY901P_SetConfigReg(imu, JY901P_REG_GYRORANGE, range);
}

/**
  * @brief  设置加速度量程并保存
  * @param  range  量程取值, 见 JY901P_ACCRANGE_* 宏
  * @note   ACCRANGE(0x21) 寄存器; 默认 ±2g, 一般不可改
  */
void JY901P_SetAccRange(JY901P_t *imu, uint8_t range)
{
    JY901P_SetConfigReg(imu, JY901P_REG_ACCRANGE, range);
}
