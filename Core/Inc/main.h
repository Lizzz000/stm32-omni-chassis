/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control_timing.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
    CHASSIS_CONTROL_STOP = 0,  /* 0：停止，默认上电状态 */
    CHASSIS_CONTROL_VELOCITY,  /* 1：vx/vy/omega 速度闭环，不保持航向 */
    CHASSIS_CONTROL_STRAIGHT,  /* 2：前后直行，保持进入模式时的航向 */
    CHASSIS_CONTROL_LATERAL,   /* 3：左右平移，使用独立航向环 */
    CHASSIS_CONTROL_COMBINED,  /* 4：前后与横向组合运动，保持航向 */
    CHASSIS_CONTROL_TURN       /* 5：单次相对转向，完成或失败后自动停止 */
} ChassisControlMode_t;

/* 上电测试只执行一次；状态用于日志/调试查看，不作为新的运动命令入口。 */
typedef enum
{
    POWER_ON_TEST_DISABLED = 0, /* 0：开关关闭，使用普通手动控制 */
    POWER_ON_TEST_WAITING,      /* 1：等待启动延迟结束和 IMU 数据有效 */
    POWER_ON_TEST_RUNNING,      /* 2：正在执行所选动作 */
    POWER_ON_TEST_DONE,         /* 3：定时结束或 180° 转向到位 */
    POWER_ON_TEST_ABORTED       /* 4：配置无效、等待超时、故障或手动切换取消 */
} PowerOnTestState_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* 基本控制命令：先让 STOP 执行至少一个 10ms 周期，再写参数并切换模式。
 * 同时更新多个字段时使用短临界区；手动直行/平移测试结束需主动切回 STOP。 */
extern volatile ChassisControlMode_t g_control_mode; /* 当前命令模式，取值 0～5 */
extern volatile float g_target_vx;          /* 前后速度 m/s，向前为正 */
extern volatile float g_target_vy;          /* 横移速度 m/s，向左为正 */
extern volatile float g_target_omega;       /* 速度模式的角速度 rad/s，逆时针为正 */
extern volatile float g_turn_angle_deg;     /* 相对转角 °，进入 TURN 时锁存，180° 两侧皆可 */
extern volatile uint8_t g_turn_state;       /* 最近转向状态：0 空闲、1 转向中、2 到位、3 超时、4 越限 */
extern volatile PowerOnTestState_t g_power_on_test_state; /* 上电测试状态，取值 0～4 */
extern volatile uint8_t g_remote_active;   /* 1：串口正在控制持续运动 */
extern volatile uint8_t g_remote_timeout;  /* 1：遥控超时/过期帧/串口故障 */
extern volatile float RPM_A;                /* A 后轮实测转速，单位 RPM */
extern volatile float RPM_B;                /* B 右前轮实测转速，单位 RPM */
extern volatile float RPM_C;                /* C 左前轮实测转速，单位 RPM */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
