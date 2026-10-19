/**
  ******************************************************************************
  * @file    motor_safety.h
  * @brief   电机超速保护: 任一车轮实测转速超过限值 → 立即刹停并锁定
  ******************************************************************************
  * 保护策略:
  *   1. 每控制周期(10ms, TIM7 中断)调用 MotorSafety_Check(RPM_A, RPM_B, RPM_C);
  *   2. 任一车轮 |实测RPM| > MOTOR_SAFETY_RPM_LIMIT 且连续
  *      MOTOR_SAFETY_TRIP_CYCLES 个周期满足 → 触发保护;
  *   3. 触发瞬间三路电机快速刹车, 并锁定停机状态;
  *   4. 锁定期间每次 Check 都会强制刹车, 即使主循环继续发 PWM 指令
  *      电机也不会转动, 必须调用 MotorSafety_Clear() 才恢复输出;
  *   5. 连续 N 周期才触发是为了滤除编码器毛刺和正常闭环的瞬态过冲,
  *      避免正常调速时误触发。
  *
  * 使用示例:
  *   MotorSafety_Init();
  *   // TIM7 回调里(更新完 RPM 之后):
  *   MotorSafety_Check(RPM_A, RPM_B, RPM_C);
  *   if (MotorSafety_Tripped()) { ... 提示/上报 ... }
  *   // 故障排除后:
  *   MotorSafety_Clear();
  ******************************************************************************
  */
#ifndef __MOTOR_SAFETY_H
#define __MOTOR_SAFETY_H

#include <stdint.h>
#include "control_timing.h"

/* 超速阈值: 任一车轮实测|RPM|超过该值判定超速。
 * 电机额定转速约 260RPM, 取 300 留出余量; 可按实际测试调整 */
#define MOTOR_SAFETY_RPM_LIMIT    300.0f

/* 连续满足超速条件的控制周期数(10ms×10=100ms)才触发, 防误报 */
#define MOTOR_SAFETY_TRIP_CYCLES  (100U / CHASSIS_CONTROL_PERIOD_MS)

/** @brief 初始化/复位超速保护(清除锁定与超速计数) */
void MotorSafety_Init(void);

/**
  * @brief  超速检测(每控制周期调用一次, 建议放在 TIM7 10ms 回调里)
  * @param  rpm_a  A轮实测转速 (RPM, 带符号)
  * @param  rpm_b  B轮实测转速 (RPM, 带符号)
  * @param  rpm_c  C轮实测转速 (RPM, 带符号)
  * @note   触发时立即三路快速刹车; 锁定期间每次调用都会强制刹车,
  *         防止其他代码的 PWM 指令让电机重新转动
  */
void MotorSafety_Check(float rpm_a, float rpm_b, float rpm_c);

/** @brief 查询保护是否已触发, 1=已触发锁定(电机被强制停机) */
uint8_t MotorSafety_Tripped(void);

/** @brief 手动清除锁定, 恢复电机输出 */
void MotorSafety_Clear(void);

/**
  * @brief  使能/禁用超速保护
  * @param  enable  1=使能(默认), 0=禁用
  * @note   禁用期间 MotorSafety_Check 不做任何检测与刹车;
  *         用于上电自检、单轮标定等需要短时高转速的场合, 用完后恢复使能
  */
void MotorSafety_SetEnabled(uint8_t enable);

#endif /* __MOTOR_SAFETY_H */
