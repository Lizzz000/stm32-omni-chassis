/**
  ******************************************************************************
  * @file    jy901p.h
  * @brief   JY901P 九轴姿态传感器驱动头文件
  ******************************************************************************
  * 通信方式: USART2 + DMA(空闲中断接收), WIT 私有协议
  * 协议文档: 《WIT私有协议》 https://wit-motion.yuque.com/wumwnr/ltst03/vl3tpy
  *
  * 接收数据帧(固定 11 字节, 无长度字段):
  *   0x55 | TYPE | D1L D1H D2L D2H D3L D3H D4L D4H | SUM
  *   SUM = (0x55 + TYPE + 8 个数据字节) 的低 8 位
  *   16 位数据为小端带符号: DATA = (int16_t)(高字节<<8 | 低字节)
  * 角度帧(0x53): Roll/Pitch/Yaw 各 2 字节 + 版本号(2 字节)
  *   角度(°) = 原始值 / 32768 * 180
  *
  * 写指令(5 字节, 无校验和, 修改前必须先解锁):
  *   FF AA [ADDR] [DATAL] [DATAH]
  * 解锁: FF AA 69 88 B5      保存: FF AA 00 00 00
  * 注: 解锁后所有指令须在 10S 内完成, 否则模块自动上锁;
  *     读取指令不需要解锁。
  *
  * 使用示例(在 MX_USART2_UART_Init 之后):
  *   JY901P_t imu;
  *   JY901P_Init(&imu, &huart2);
  *   float roll  = JY901P_GetRoll(&imu);
  *   float pitch = JY901P_GetPitch(&imu);
  *   float yaw   = JY901P_GetYaw(&imu);
  *
  * 前提: CubeMX 已使能 USART2 全局中断(生成 USART2_IRQHandler),
  *       USART2 已配置 DMA 收发(DMA1_Stream5 RX / DMA1_Stream6 TX)。
  ******************************************************************************
  */
#ifndef __JY901P_H
#define __JY901P_H

#include "stm32f4xx_hal.h"

/* DMA 接收缓冲大小(空闲中断每次搬运的最大字节数) */
#define JY901P_DMA_BUF_SIZE   256
/* 数据帧数据区固定字节数(4 个 16 位数据) */
#define JY901P_FRAME_DATA_LEN 8

/* 输出帧 TYPE */
#define JY901P_ID_TIME         0x50    /* 时间 */
#define JY901P_ID_ACC          0x51    /* 加速度 */
#define JY901P_ID_GYRO         0x52    /* 角速度 */
#define JY901P_ID_ANGLE        0x53    /* 角度 */
#define JY901P_ID_MAG          0x54    /* 磁场(个别型号支持) */
#define JY901P_ID_PORT         0x55    /* 端口状态(个别型号支持) */
#define JY901P_ID_PRESSURE     0x56    /* 气压高度(个别型号支持) */
#define JY901P_ID_GPS          0x57    /* 经纬度(个别型号支持) */
#define JY901P_ID_GPSVEL       0x58    /* GPS地速(个别型号支持) */
#define JY901P_ID_QUAT         0x59    /* 四元数(个别型号支持) */
#define JY901P_ID_GPSACC       0x5A    /* GPS定位精度(个别型号支持) */
#define JY901P_ID_READREG      0x5F    /* 读取寄存器返回 */

/* 配置寄存器地址(见协议寄存器表) */
#define JY901P_REG_SAVE        0x00    /* 保存(0x00)/重启(0xFF)/恢复出厂(0x01) */
#define JY901P_REG_CALSW       0x01    /* 校准模式(取值见 JY901P_CalibType) */
#define JY901P_REG_RSW         0x02    /* 输出内容开关(16 位位掩码, 见 JY901P_RSW_*) */
#define JY901P_REG_RRATE       0x03    /* 输出速率(取值见 JY901P_RATE_*) */
#define JY901P_REG_BAUD        0x04    /* 串口波特率(取值见 JY901P_BAUD_*) */
#define JY901P_REG_GYRORANGE   0x20    /* 陀螺仪量程(取值见 JY901P_GYRORANGE_*) */
#define JY901P_REG_ACCRANGE    0x21    /* 加速度量程(取值见 JY901P_ACCRANGE_*) */
#define JY901P_REG_ORIENT      0x23    /* 安装方向(0 水平 / 1 垂直, Y 轴朝上) */
#define JY901P_REG_ALGORITHM   0x24    /* AXIS6 算法(见 JY901P_Algorithm) */
#define JY901P_REG_READADDR    0x27    /* 读寄存器: 发送 FF AA 27 ADDR 00, 返回 0x5F 帧 */
#define JY901P_REG_GYROAUTOCAL 0x61    /* 陀螺仪自动校准: 0=开启(默认), 1=关闭 */
#define JY901P_REG_KEY         0x69    /* 解锁: 写 0xB588 解锁 */

/* 波特率取值 */
#define JY901P_BAUD_4800       0x01    /* 4800bps */
#define JY901P_BAUD_9600       0x02    /* 9600bps */
#define JY901P_BAUD_19200      0x03    /* 19200bps */
#define JY901P_BAUD_38400      0x04    /* 38400bps */
#define JY901P_BAUD_57600      0x05    /* 57600bps */
#define JY901P_BAUD_115200     0x06    /* 115200bps */
#define JY901P_BAUD_230400     0x07    /* 230400bps */

/* 输出速率取值(RRATE) */
#define JY901P_RATE_0_2HZ      0x01    /* 0.2Hz */
#define JY901P_RATE_0_5HZ      0x02    /* 0.5Hz */
#define JY901P_RATE_1HZ        0x03    /* 1Hz */
#define JY901P_RATE_2HZ        0x04    /* 2Hz */
#define JY901P_RATE_5HZ        0x05    /* 5Hz */
#define JY901P_RATE_10HZ       0x06    /* 10Hz */
#define JY901P_RATE_20HZ       0x07    /* 20Hz */
#define JY901P_RATE_50HZ       0x08    /* 50Hz */
#define JY901P_RATE_100HZ      0x09    /* 100Hz */
#define JY901P_RATE_200HZ      0x0B    /* 200Hz */
#define JY901P_RATE_SINGLE     0x0C    /* 单次回传(不回传时发送 FF AA 27 00 00 触发一次) */
#define JY901P_RATE_OFF        0x0D    /* 不回传(上电无输出, 用 0x27 指令逐次读取) */

/* 输出内容位掩码(RSW, 对应 16 位寄存器, 默认 0x001E) */
#define JY901P_RSW_TIME        (1u << 0)   /* 时间帧 0x50 */
#define JY901P_RSW_ACC         (1u << 1)   /* 加速度帧 0x51 */
#define JY901P_RSW_GYRO        (1u << 2)   /* 角速度帧 0x52 */
#define JY901P_RSW_ANGLE       (1u << 3)   /* 角度帧 0x53 */
#define JY901P_RSW_MAG         (1u << 4)   /* 磁场帧 0x54 */
#define JY901P_RSW_PORT        (1u << 5)   /* 端口状态帧 0x55 */
#define JY901P_RSW_PRESS       (1u << 6)   /* 气压高度帧 0x56 */
#define JY901P_RSW_GPS         (1u << 7)   /* 经纬度帧 0x57 */
#define JY901P_RSW_VELOCITY    (1u << 8)   /* GPS地速帧 0x58 */
#define JY901P_RSW_QUAT        (1u << 9)   /* 四元数帧 0x59 */
#define JY901P_RSW_GSA         (1u << 10)  /* GPS定位精度帧 0x5A */

/* 陀螺仪量程取值(GYRORANGE, 默认 0x03, 一般不可改) */
#define JY901P_GYRORANGE_200   0x00    /* 200°/s */
#define JY901P_GYRORANGE_500   0x01    /* 500°/s */
#define JY901P_GYRORANGE_1000  0x02    /* 1000°/s */
#define JY901P_GYRORANGE_2000  0x03    /* 2000°/s(默认) */

/* 加速度量程取值(ACCRANGE, 默认 0x00, 一般不可改) */
#define JY901P_ACCRANGE_2G     0x00    /* ±2g(默认) */
#define JY901P_ACCRANGE_4G     0x01    /* ±4g */
#define JY901P_ACCRANGE_8G     0x02    /* ±8g */
#define JY901P_ACCRANGE_16G    0x03    /* ±16g */

/* 姿态解算算法(寄存器 AXIS6 Bit0) */
typedef enum
{
    JY901P_ALGO_NINE_AXIS = 0x00,     /* 九轴: 磁力计解算航向角, 绝对航向角(默认) */
    JY901P_ALGO_SIX_AXIS  = 0x01,     /* 六轴: 积分解算航向角, 相对航向角 */
} JY901P_Algorithm;

/* 校准类型(写入 CALSW 寄存器的取值, 校准前必须先解锁) */
typedef enum
{
    JY901P_CALIB_ACC        = 0x01,   /* 自动加计校准: 模块正面水平放置 */
    JY901P_CALIB_YAW        = 0x04,   /* 航向角置零: 仅六轴算法下可用 */
    JY901P_CALIB_MAG        = 0x07,   /* 磁场校准(球型拟合法): 三轴各旋转 1~2 圈 */
    JY901P_CALIB_ANGLE_REF  = 0x08,   /* 设置角度参考: 当前姿态置为相对零位 */
    JY901P_CALIB_GYRO_START = 0x0C,   /* 陀螺仪手动校准开始: 静止状态下采集零偏 */
    JY901P_CALIB_GYRO_END   = 0x0D,   /* 陀螺仪手动校准结束: 运动前结束采集 */
} JY901P_CalibType;

/* 帧解析状态机 */
typedef enum
{
    JY901P_RX_WAIT_HEAD = 0,          /* 等待 0x55 */
    JY901P_RX_WAIT_TYPE,              /* 帧类型 TYPE */
    JY901P_RX_WAIT_DATA,              /* 8 字节数据区 */
    JY901P_RX_WAIT_SUM,               /* 校验和 */
} JY901P_RxState;

typedef struct
{
    UART_HandleTypeDef *huart;    /* 注册的 UART 句柄(现为 USART2) */

    /* DMA 接收缓冲(中断回调用)
       注意: 不可放置在 CCM RAM(F407 的 CCM 区 DMA 无法访问) */
    uint8_t dma_rx_buf[JY901P_DMA_BUF_SIZE];

    /* 帧解析状态 */
    JY901P_RxState rx_state;
    uint8_t frame_id;             /* 当前帧 TYPE */
    uint8_t frame_sum;            /* 校验和累计 */
    uint8_t frame_cnt_data;       /* 已收数据字节数 */
    uint8_t frame_data[JY901P_FRAME_DATA_LEN];

    /* 姿态角(单位: °), 由中断更新、主循环读取, 故加 volatile */
    volatile float roll;          /* 横滚角 ±180° */
    volatile float pitch;         /* 俯仰角 ±90° */
    volatile float yaw;           /* 航向角 ±180° */
    float yaw_offset;             /* 软件零位偏移(仅主循环访问) */
    volatile uint32_t last_update_ms;  /* 最近一次角度帧的时间戳(HAL_GetTick) */
    volatile uint8_t has_valid_angle; /* Published after angle data/timestamp; not a counter. */
    volatile uint16_t frame_count;     /* 已解析的有效角度帧数 */
} JY901P_t;

/**
  * @brief  初始化 JY901P
  * @param  imu   驱动实例
  * @param  huart USART2 句柄
  * @note   必须在 MX_USART2_UART_Init 之后调用;
  *         启动后由空闲中断+DMA 自动接收解析, 主循环无需轮询
  */
void JY901P_Init(JY901P_t *imu, UART_HandleTypeDef *huart);

/* 获取姿态角(单位: °) */
float JY901P_GetRoll(const JY901P_t *imu);
float JY901P_GetPitch(const JY901P_t *imu);
float JY901P_GetYaw(const JY901P_t *imu);

/**
  * @brief  JY901P UART接收回调(由应用层 HAL_UARTEx_RxEventCallback 分发调用)
  * @param  huart UART句柄
  * @param  Size  本次接收字节数
  * @note   仅处理注册的 UART(现为 USART2); 需在 main.c 的 HAL_UARTEx_RxEventCallback 中调用
  */
void JY901P_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);

/* 由应用唯一的HAL_UART_ErrorCallback转发，避免多个模块重复定义HAL回调。 */
void JY901P_UartErrorCallback(UART_HandleTypeDef *huart);

/* 由应用唯一的 HAL_UART_AbortReceiveCpltCallback 转发，完成异步错误恢复。 */
void JY901P_UartAbortReceiveCpltCallback(UART_HandleTypeDef *huart);

/**
  * @brief  数据新鲜度检查
  * @param  imu         驱动实例
  * @param  timeout_ms  超时时间
  * @retval 1 = 距最近一次角度帧不超过 timeout_ms, 0 = 超时(传感器异常/断线)
  */
uint8_t JY901P_IsFresh(const JY901P_t *imu, uint32_t timeout_ms);

/* 将当前偏航角设为软件零位(立即生效, 无需写模块寄存器) */
void JY901P_ResetYaw(JY901P_t *imu);

/**
  * @brief  开始校准(自动发送解锁指令并延时)
  * @param  type 校准类型, 见 JY901P_CalibType
  * @note   自动加计校准: 先正面水平放置模块再调用, 约 4S 后结束;
  *         磁场校准:   在远离铁磁物质(>20cm)处调用, 之后三轴各旋转 1~2 圈
  *                     (JY901P 约 5 分钟后自动退出校准);
  *         陀螺仪手动校准: 调用后设备保持静止, 采集完成前调用
  *                     JY901P_CaliEnd 结束采集
  */
void JY901P_CaliStart(JY901P_t *imu, JY901P_CalibType type);

/**
  * @brief  结束校准并保存到 Flash
  * @param  type 校准类型, 与 JY901P_CaliStart 保持一致
  * @note   陀螺仪手动校准(JY901P_CALIB_GYRO_START)结束为写入 0x0D,
  *         其余类型写入 0x00 退出校准; 校准时长超过 10S 时本函数会重新解锁
  */
void JY901P_CaliEnd(JY901P_t *imu, JY901P_CalibType type);

/**
  * @brief  执行一次完整的陀螺仪零偏校准
  * @param  imu       驱动实例
  * @param  sample_ms 静止采样时间(ms), 建议不小于 5000ms
  * @note   调用期间必须让 JY901P 和整车完全静止;
  *         本函数会关闭陀螺仪自动校准、开始手动校准、等待采样、
  *         结束校准并把结果保存到模块 Flash。
  */
void JY901P_CalibrateGyro(JY901P_t *imu, uint32_t sample_ms);

/**
  * @brief  设置姿态解算算法(九轴/六轴)并保存到 Flash
  * @param  algo 算法类型, 见 JY901P_Algorithm
  * @note   切换算法后建议重新执行陀螺仪校准并重新上电
  */
void JY901P_SetAlgorithm(JY901P_t *imu, JY901P_Algorithm algo);

/**
  * @brief  设置串口波特率为 115200 并保存到 Flash
  * @note   模块收到指令后立即切换波特率;
  *         本驱动假设 USART2 已按 115200 配置, MCU 侧无需改动;
  *         若模块当前波特率非 115200, 需在发送本指令后由用户重新配置
  *         串口波特率, 再发送解锁+保存指令(见协议文档"修改波特率"流程)
  */
void JY901P_SetBaud115200(JY901P_t *imu);

/**
  * @brief  通用配置: 解锁 + 写寄存器 + 保存到 Flash
  * @param  reg    寄存器地址(见上方 JY901P_REG_* 宏)
  * @param  value  寄存器值(低 8 位)
  * @note   用于扩展其他配置项(如量程 0x20/0x21、回传速率 0x03 等)
  */
void JY901P_SetConfigReg(JY901P_t *imu, uint8_t reg, uint8_t value);

/**
  * @brief  陀螺仪自动校准开关
  * @param  imu     驱动实例
  * @param  enable  1 = 开启自动校准(默认), 0 = 关闭
  * @note   手动校准陀螺仪前需先关闭自动校准
  */
void JY901P_SetGyroAutoCal(JY901P_t *imu, uint8_t enable);

/**
  * @brief  设置输出速率(回传频率)并保存
  * @param  imu   驱动实例
  * @param  rate  速率取值, 见 JY901P_RATE_* 宏
  * @note   默认 10Hz; 设为 JY901P_RATE_OFF 后上电无输出,
  *         需发 FF AA 27 00 00 逐次触发回传
  */
void JY901P_SetOutputRate(JY901P_t *imu, uint8_t rate);

/**
  * @brief  设置输出内容(各帧开关)并保存
  * @param  imu   驱动实例
  * @param  mask  位掩码, 用 JY901P_RSW_* 宏按位或组合
  * @note   默认 0x001E(时间+加速度+角速度+角度+磁场);
  *         例: JY901P_SetOutputContent(&imu, JY901P_RSW_ANGLE) 只输出角度帧
  */
void JY901P_SetOutputContent(JY901P_t *imu, uint16_t mask);

/**
  * @brief  设置安装方向并保存
  * @param  imu       驱动实例
  * @param  vertical  0 = 水平安装(默认), 1 = 垂直安装(必须坐标轴 Y 轴箭头朝上)
  */
void JY901P_SetOrientation(JY901P_t *imu, uint8_t vertical);

/**
  * @brief  设置陀螺仪量程并保存
  * @param  imu   驱动实例
  * @param  range 量程取值, 见 JY901P_GYRORANGE_* 宏
  * @note   默认 2000°/s; 除特殊型号外固定不可设置, 改前先确认型号支持
  */
void JY901P_SetGyroRange(JY901P_t *imu, uint8_t range);

/**
  * @brief  设置加速度量程并保存
  * @param  imu   驱动实例
  * @param  range 量程取值, 见 JY901P_ACCRANGE_* 宏
  * @note   默认 ±2g; 除特殊型号外固定不可设置, 改前先确认型号支持
  */
void JY901P_SetAccRange(JY901P_t *imu, uint8_t range);

#endif /* __JY901P_H */
