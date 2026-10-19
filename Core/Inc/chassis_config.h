/* SPDX-License-Identifier: Apache-2.0 */
#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* 底盘控制调参入口，数值沿用当前标定。
 * 速度 Ki、航向 Ki/Kd 和单周期增量限幅均对应 10 ms 离散周期。
 * 修改 control_timing.h 的周期时，须重新换算这些参数并验证闭环效果。
 * 机械尺寸、编码器方向、前馈及运动限幅见 Omni.h/encoder.h。 */

/* 三轮增量式速度 PI：输出及增量单位均为 PWM 比较值。 */
#define CHASSIS_SPEED_KP                1.0f
#define CHASSIS_SPEED_A_KI              0.08f
#define CHASSIS_SPEED_B_KI              0.09f
#define CHASSIS_SPEED_C_KI              0.09f
#define CHASSIS_SPEED_KD                0.0f
#define CHASSIS_SPEED_OUTPUT_LIMIT      2000.0f
#define CHASSIS_SPEED_INCREMENT_LIMIT   40.0f

/* 直行/组合运动与平移分别使用独立的航向环。 */
#define CHASSIS_STRAIGHT_YAW_KP         0.08f
#define CHASSIS_STRAIGHT_YAW_KI         0.0005f
#define CHASSIS_STRAIGHT_YAW_KD         0.0005f
#define CHASSIS_LATERAL_YAW_KP          0.1f
#define CHASSIS_LATERAL_YAW_KI          0.0f
#define CHASSIS_LATERAL_YAW_KD          0.0f

/* 启动初始化和每次进入 TURN 均使用这一组参数。 */
#define CHASSIS_TURN_YAW_KP             0.035f
#define CHASSIS_TURN_YAW_KI             0.0f
#define CHASSIS_TURN_YAW_KD             0.0f

#endif /* CHASSIS_CONFIG_H */
