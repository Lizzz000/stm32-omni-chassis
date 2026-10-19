/**
  ******************************************************************************
  * @file    motor_safety.c
  * @brief   电机超速保护: 任一车轮实测转速超过限值 → 立即刹停并锁定
  ******************************************************************************
  */
#include "motor_safety.h"
#include "motor.h"
#include <math.h>

static volatile uint8_t s_tripped;                 /* 保护锁定标志 */
static volatile uint8_t s_over_cnt[3];             /* 三路超速连续计数 */
static volatile uint8_t s_enabled = 1U;            /* 保护使能标志(默认开启) */

void MotorSafety_Init(void)
{
    s_tripped = 0U;
    s_enabled = 1U;
    s_over_cnt[0] = s_over_cnt[1] = s_over_cnt[2] = 0U;
}

void MotorSafety_Check(float rpm_a, float rpm_b, float rpm_c)
{
    /* 保护被临时禁用(自检/标定期间): 不做任何检测与刹车 */
    if (!s_enabled)
        return;

    /* 已触发: 保持锁定, 每次调用强制刹车,
     * 主循环即使继续发 PWM 指令, 电机也不会重新转动 */
    if (s_tripped) {
        Motor_A_Brake();
        Motor_B_Brake();
        Motor_C_Brake();
        return;
    }

    /* 单轮超速计数: 连续 N 个周期 |RPM| > 限值 才触发, 滤除毛刺/瞬态过冲 */
    const float rpm[3] = {rpm_a, rpm_b, rpm_c};
    uint8_t trip = 0U;
    uint8_t i;

    for (i = 0U; i < 3U; i++) {
        if (fabsf(rpm[i]) > MOTOR_SAFETY_RPM_LIMIT) {
            if (++s_over_cnt[i] >= MOTOR_SAFETY_TRIP_CYCLES)
                trip = 1U;
        } else {
            s_over_cnt[i] = 0U;
        }
    }

    if (trip) {
        s_tripped = 1U;
        /* 立即停机: 三路 AT8236 快速刹车(比 PWM 清零的滑行停得更快) */
        Motor_A_Brake();
        Motor_B_Brake();
        Motor_C_Brake();
    }
}

uint8_t MotorSafety_Tripped(void)
{
    return s_tripped;
}

void MotorSafety_Clear(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_tripped = 0U;
    s_over_cnt[0] = s_over_cnt[1] = s_over_cnt[2] = 0U;
    __DMB();
    if (primask == 0U)
        __enable_irq();
}

void MotorSafety_SetEnabled(uint8_t enable)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_enabled = enable ? 1U : 0U;
    __DMB();
    if (primask == 0U)
        __enable_irq();
}
