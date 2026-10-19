/**
  ******************************************************************************
  * @file    motor.h
  * @brief   AT8236 直流电机驱动（每电机双 PWM 输入）
  ******************************************************************************
  * 电机 A/B/C 分别使用 TIM1/TIM2/TIM3 的两个 PWM 通道驱动 AT8236 的
  * IN1/IN2 输入：
  *   A: TIM1 CH1(PE9)/CH2(PE11)    B: TIM2 CH3(PB10)/CH4(PB11)
  *   C: TIM3 CH1(PA6)/CH2(PA7)
  * 正 PWM 和负 PWM 表示两个方向，绝对值按各自定时器 ARR 限幅后换算：
 *   正向：IN1 = ARR（恒高），IN2 = ARR-|v|（倒置 PWM）
 *   反向：IN2 = ARR（恒高），IN1 = ARR-|v|
 *   零速/滑行：IN1=IN2=0；主动刹车：IN1=IN2=ARR。
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "tim.h"

/* 三轮分别使用独立定时器的两路PWM。若CubeMX修改通道，只需调整这里。 */
#define MOTOR_A_TIM        htim1
#define MOTOR_A_IN1_CH     TIM_CHANNEL_1
#define MOTOR_A_IN2_CH     TIM_CHANNEL_2

#define MOTOR_B_TIM        htim2
#define MOTOR_B_IN1_CH     TIM_CHANNEL_3
#define MOTOR_B_IN2_CH     TIM_CHANNEL_4

#define MOTOR_C_TIM        htim3
#define MOTOR_C_IN1_CH     TIM_CHANNEL_1
#define MOTOR_C_IN2_CH     TIM_CHANNEL_2

/* 非零PWM低于该占空比时提升到此值，克服电机起步死区；明确的0不提升。 */
#define MOTOR_MIN_DUTY_PERCENT  5U

/** @return 1=三个电机的六个PWM通道全部启动；0=配置失败且所有通道已清零并停止输出。 */
int Motor_Init(void);

/** 设置有符号PWM，内部按对应定时器ARR安全限幅；0表示滑行停止。 */
void Motor_A_SetPWM(int16_t pwm);
void Motor_B_SetPWM(int16_t pwm);
void Motor_C_SetPWM(int16_t pwm);

/** 两输入均置为ARR，使用AT8236主动刹车；仅短时停止或故障保护使用。 */
void Motor_A_Brake(void);
void Motor_B_Brake(void);
void Motor_C_Brake(void);

/** 返回三路电机共同可用的最大PWM比较值。 */
uint16_t Motor_GetMaxPWM(void);

#endif /* __MOTOR_H */
