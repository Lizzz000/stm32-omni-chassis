#include "motor.h"

#include <stddef.h>

static uint32_t  Motor_ClampMagnitude(const TIM_HandleTypeDef *htim, int32_t pwm)
{
    uint32_t magnitude = (pwm < 0) ? (uint32_t)(-pwm) : (uint32_t)pwm;
    uint32_t period = __HAL_TIM_GET_AUTORELOAD((TIM_HandleTypeDef *)htim);

    return (magnitude > period) ? period : magnitude;
}

static void Motor_SetPWM(TIM_HandleTypeDef *htim, uint32_t in1_channel,
                         uint32_t in2_channel, int16_t pwm)
{
    uint32_t duty;
    uint32_t period;

    if (htim == NULL || htim->Instance == NULL)
        return;

    period = __HAL_TIM_GET_AUTORELOAD(htim);
    duty   = Motor_ClampMagnitude(htim, (int32_t)pwm);

    /* 死区补偿: 非零指令低于最小有效占空比时提升到最小值，
     * 使低速目标能够真正起转；明确的pwm=0仍保持滑行停止。 */
    if (pwm != 0) {
        /* 64 位乘法：32 位 ARR 定时器时 period*percent 会溢出。 */
        uint32_t min_duty = (uint32_t)(((uint64_t)period * MOTOR_MIN_DUTY_PERCENT) / 100U);
        if (duty < min_duty)
            duty = min_duty;
    }

    /* 互补驱动: 方向通道给满(ARR), 速度通道写 ARR - |pwm|,
     * 使速度通道低电平时间 = |pwm|/ARR, 即设定占空比:
     *   正转: IN1=满, IN2=ARR-pwm → IN2 低电平期间电机正转通电,
     *         其余时间 IN1=IN2=1 进入刹车态, 调速更硬、电流纹波更小
     *   反转: IN2=满, IN1=ARR-pwm → 同理反向
     *   停止: 两路清零(滑行), 与旧行为一致; 需刹车时用 Motor_X_Brake() */
    if (pwm > 0) {
        __HAL_TIM_SET_COMPARE(htim, in1_channel, period);
        __HAL_TIM_SET_COMPARE(htim, in2_channel, period - duty);
    } else if (pwm < 0) {
        __HAL_TIM_SET_COMPARE(htim, in2_channel, period);
        __HAL_TIM_SET_COMPARE(htim, in1_channel, period - duty);
    } else {
        __HAL_TIM_SET_COMPARE(htim, in1_channel, 0U);
        __HAL_TIM_SET_COMPARE(htim, in2_channel, 0U);
    }
}

static void Motor_Brake(TIM_HandleTypeDef *htim, uint32_t in1_channel,
                        uint32_t in2_channel)
{
    uint32_t period;

    if (htim == NULL || htim->Instance == NULL)
        return;
    period = __HAL_TIM_GET_AUTORELOAD(htim);
    __HAL_TIM_SET_COMPARE(htim, in1_channel, period);
    __HAL_TIM_SET_COMPARE(htim, in2_channel, period);
}

static void Motor_StopPWM(TIM_HandleTypeDef *htim, uint32_t in1_channel,
                          uint32_t in2_channel)
{
    if (htim == NULL || htim->Instance == NULL)
        return;

    __HAL_TIM_SET_COMPARE(htim, in1_channel, 0U);
    __HAL_TIM_SET_COMPARE(htim, in2_channel, 0U);
    (void)HAL_TIM_PWM_Stop(htim, in1_channel);
    (void)HAL_TIM_PWM_Stop(htim, in2_channel);
}

int Motor_Init(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    __HAL_TIM_SET_COUNTER(&MOTOR_A_TIM, 0U);
    __HAL_TIM_SET_COUNTER(&MOTOR_B_TIM, 0U);
    __HAL_TIM_SET_COUNTER(&MOTOR_C_TIM, 0U);

    Motor_A_SetPWM(0);
    Motor_B_SetPWM(0);
    Motor_C_SetPWM(0);

    if (HAL_TIM_PWM_Start(&MOTOR_A_TIM, MOTOR_A_IN1_CH) != HAL_OK) status = HAL_ERROR;
    if (HAL_TIM_PWM_Start(&MOTOR_A_TIM, MOTOR_A_IN2_CH) != HAL_OK) status = HAL_ERROR;
    if (HAL_TIM_PWM_Start(&MOTOR_B_TIM, MOTOR_B_IN1_CH) != HAL_OK) status = HAL_ERROR;
    if (HAL_TIM_PWM_Start(&MOTOR_B_TIM, MOTOR_B_IN2_CH) != HAL_OK) status = HAL_ERROR;
    if (HAL_TIM_PWM_Start(&MOTOR_C_TIM, MOTOR_C_IN1_CH) != HAL_OK) status = HAL_ERROR;
    if (HAL_TIM_PWM_Start(&MOTOR_C_TIM, MOTOR_C_IN2_CH) != HAL_OK) status = HAL_ERROR;

    if (status != HAL_OK) {
        /* 任一通道失败时回滚全部输出，避免留下部分电机仍启用。 */
        Motor_StopPWM(&MOTOR_A_TIM, MOTOR_A_IN1_CH, MOTOR_A_IN2_CH);
        Motor_StopPWM(&MOTOR_B_TIM, MOTOR_B_IN1_CH, MOTOR_B_IN2_CH);
        Motor_StopPWM(&MOTOR_C_TIM, MOTOR_C_IN1_CH, MOTOR_C_IN2_CH);
    }

    return status == HAL_OK;
}

void Motor_A_SetPWM(int16_t pwm)
{
    Motor_SetPWM(&MOTOR_A_TIM, MOTOR_A_IN1_CH, MOTOR_A_IN2_CH, pwm);
}

void Motor_B_SetPWM(int16_t pwm)
{
    Motor_SetPWM(&MOTOR_B_TIM, MOTOR_B_IN1_CH, MOTOR_B_IN2_CH, pwm);
}

void Motor_C_SetPWM(int16_t pwm)
{
    Motor_SetPWM(&MOTOR_C_TIM, MOTOR_C_IN1_CH, MOTOR_C_IN2_CH, pwm);
}

void Motor_A_Brake(void)
{
    Motor_Brake(&MOTOR_A_TIM, MOTOR_A_IN1_CH, MOTOR_A_IN2_CH);
}

void Motor_B_Brake(void)
{
    Motor_Brake(&MOTOR_B_TIM, MOTOR_B_IN1_CH, MOTOR_B_IN2_CH);
}

void Motor_C_Brake(void)
{
    Motor_Brake(&MOTOR_C_TIM, MOTOR_C_IN1_CH, MOTOR_C_IN2_CH);
}

uint16_t Motor_GetMaxPWM(void)
{
    uint32_t max_pwm = __HAL_TIM_GET_AUTORELOAD(&MOTOR_A_TIM);
    uint32_t period_b = __HAL_TIM_GET_AUTORELOAD(&MOTOR_B_TIM);
    uint32_t period_c = __HAL_TIM_GET_AUTORELOAD(&MOTOR_C_TIM);

    if (period_b < max_pwm) max_pwm = period_b;
    if (period_c < max_pwm) max_pwm = period_c;
    if (max_pwm > UINT16_MAX) max_pwm = UINT16_MAX;
    return (uint16_t)max_pwm;
}
