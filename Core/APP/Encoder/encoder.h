/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   增量式编码器速度与累计计数接口
  ******************************************************************************
  * 当前参数：13 PPR × 4倍频 × 34.014减速比 = 输出轴每圈1768.728计数。
  * A/B/C 轮编码器分别固定使用 TIM4/TIM5/TIM8。TIM4/TIM8 为16位，TIM5为32位；
  * 本模块不再每次清零CNT，
  * 而是比较相邻采样值并按各定时器 ARR 修正回绕，因此正转和反转都能正确计算。
  */
#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "tim.h"

#define ENCODER_A_TIM htim4
#define ENCODER_B_TIM htim5
#define ENCODER_C_TIM htim8

#define ENCODER_MOTOR_PPR               13.0f
#define ENCODER_QUADRATURE_MULTIPLIER     4.0f
#define ENCODER_GEAR_RATIO               34.014f
#define ENCODER_PULSES_PER_REV \
    (ENCODER_GEAR_RATIO * ENCODER_MOTOR_PPR * ENCODER_QUADRATURE_MULTIPLIER)

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t last_count;   /**< 上一次读取的硬件CNT */
    int32_t last_delta;    /**< 最近一次经过回绕修正和方向修正的增量 */
    int64_t total_count;   /**< 自初始化/复位以来的累计有符号计数 */
    float rpm;             /**< 最近一次计算的输出轴转速 */
    int8_t direction;      /**< +1=原始方向，-1=软件反向 */
    uint8_t initialized;
} Encoder_t;

/**
  * @brief 启动编码器定时器并建立第一次采样基准。
  * @return 1=成功，0=参数无效或HAL启动失败。
  */
int Encoder_Init(Encoder_t *encoder, TIM_HandleTypeDef *htim);

/** 按 A=TIM4、B=TIM5、C=TIM8 一次启动三轮编码器。1=全部成功。 */
int Encoder_InitOmni(Encoder_t *encoder_a, Encoder_t *encoder_b,
                     Encoder_t *encoder_c);

/** @brief 停止编码器定时器并把实例标记为未初始化。 */
void Encoder_Stop(Encoder_t *encoder);

/** @brief 清零硬件CNT、RPM、最近增量和累计计数，保持定时器运行。 */
int Encoder_Reset(Encoder_t *encoder);

/**
  * @brief 根据本次与上次CNT差值更新RPM。
  * @param period_s 两次调用的实际秒数，必须有限且大于0。
  * @return 1=更新成功，0=参数/状态无效；失败时RPM置0。
  * @note 相邻采样位移必须小于计数周期的一半。16位ARR=65535时即小于32768计数。
  */
int Encoder_UpdateRPM(Encoder_t *encoder, float period_s);

/**
  * @brief 计算定时器的更新周期（秒），供 Encoder_UpdateRPM 换算转速。
  *        依据 PSC/ARR 与所在总线的定时器时钟计算：
  *        APB 预分频≠1 时定时器时钟为总线时钟×2。
  *        采样定时器配置改动后无需再修改调用代码。
  * @param htim 采样定时器句柄（本项目为产生控制周期中断的 TIM7）。
  * @return 周期秒数；句柄无效或时钟未知时返回 0（UpdateRPM 会拒绝）。
  */
float Encoder_GetTimerPeriodS(const TIM_HandleTypeDef *htim);

/** @brief 设置软件方向；direction<0反向，其余值使用正向。 */
int Encoder_SetDirection(Encoder_t *encoder, int direction);

float Encoder_GetRPM(const Encoder_t *encoder);
int32_t Encoder_GetLastDelta(const Encoder_t *encoder);
int64_t Encoder_GetTotalCount(const Encoder_t *encoder);

/**
  * @brief 纯数学接口：计算自由运行计数器两次采样间的最短有符号增量。
  * @param auto_reload 定时器ARR；计数模数为ARR+1。
  * @return 1=成功，0=输入超范围/结果不能表示为int32。
  * @note 公开此函数便于无硬件单元测试，业务代码通常直接调用UpdateRPM。
  */
int Encoder_CalculateWrappedDelta(uint32_t current, uint32_t previous,
                                  uint32_t auto_reload, int32_t *delta);

#endif /* __ENCODER_H */
