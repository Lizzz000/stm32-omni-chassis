/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_config.h"
#include "encoder.h"
#include "jy901p.h"
#include "motor.h"
#include "motor_safety.h"
#include "Omni.h"
#include "pid.h"
#include "remote_control.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_FRESH_TIMEOUT_MS 500U   /* IMU 角度帧超过 500ms 未更新，视为失效 */
#define DEBUG_PRINT_PERIOD_MS 100U  /* USART3 调试日志输出间隔，单位 ms */

/* ==================== 上电单次动作测试参数 ====================
 * 修改下面的宏后重新编译/烧录，也可通过编译器 -D 覆盖。
 * ENABLE=0：关闭上电测试，车辆保持原有手动控制；ENABLE=1：上电执行一次。
 * MODE 选择：CHASSIS_CONTROL_STRAIGHT（2，直行）、
 *            CHASSIS_CONTROL_LATERAL（3，平移）、CHASSIS_CONTROL_TURN（5，180°）。
 * 直行/平移按各自速度和时长执行，到时自动输出零 PWM 并停止；
 * 转向固定为相对 180°，沿用底盘到位/8 秒超时/越限保护，不使用平移时长。
 * 完成或取消后不自动重试；需要再次上电测试时复位 MCU。 */
#ifndef POWER_ON_TEST_ENABLE
#define POWER_ON_TEST_ENABLE 0U                  /* 总开关：0 关闭，1 启用 */
#endif
#ifndef POWER_ON_TEST_MODE
#define POWER_ON_TEST_MODE CHASSIS_CONTROL_STRAIGHT /* 动作选择：2 直行、3 平移、5 掉头 */
#endif
#ifndef POWER_ON_TEST_STRAIGHT_SPEED_MPS
#define POWER_ON_TEST_STRAIGHT_SPEED_MPS 0.15f    /* 直行速度 m/s，正值前进，负值后退 */
#endif
#ifndef POWER_ON_TEST_LATERAL_SPEED_MPS
#define POWER_ON_TEST_LATERAL_SPEED_MPS 0.15f     /* 平移速度 m/s，正值向左，负值向右 */
#endif
#ifndef POWER_ON_TEST_STRAIGHT_DURATION_MS
#define POWER_ON_TEST_STRAIGHT_DURATION_MS 2000U  /* 直行测试时长 ms，从实际开始驱动时计时 */
#endif
#ifndef POWER_ON_TEST_LATERAL_DURATION_MS
#define POWER_ON_TEST_LATERAL_DURATION_MS 2000U   /* 平移测试时长 ms，从实际开始驱动时计时 */
#endif
#ifndef POWER_ON_TEST_START_DELAY_MS
#define POWER_ON_TEST_START_DELAY_MS 2000U        /* 初始化完成后的启动等待时间 ms */
#endif
#ifndef POWER_ON_TEST_IMU_WAIT_TIMEOUT_MS
#define POWER_ON_TEST_IMU_WAIT_TIMEOUT_MS 5000U   /* 启动延迟结束后，等待 IMU 有效的最长时间 ms */
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Omni_t omni;           /* 三轮全向底盘，保存轮速目标和航向控制状态 */
Encoder_t encoder1;    /* A 后轮编码器，TIM4 */
Encoder_t encoder2;    /* B 右前轮编码器，TIM5 */
Encoder_t encoder3;    /* C 左前轮编码器，TIM8 */
JY901P_t imu;          /* JY901P 姿态传感器，USART2 */

/* 三轮独立增量式速度 PID；由 Omni 模块每个控制周期计算一次。
 * 直行/平移使能为 1 仅表示允许执行，实际运动由 g_control_mode 选择。 */
inc_pid_t motor_A_pid;
inc_pid_t motor_B_pid;
inc_pid_t motor_C_pid;
volatile uint8_t g_drive_straight_enable = 1U; /* 直行使能，组合运动也检查此标志 */
volatile uint8_t g_drive_lateral_enable = 1U;  /* 平移使能，组合运动也检查此标志 */

volatile float RPM_A;  /* A 轮实测转速，单位 RPM，保留编码器原始方向符号 */
volatile float RPM_B;  /* B 轮实测转速，TIM7 每 10ms 更新 */
volatile float RPM_C;  /* C 轮实测转速，TIM7 每 10ms 更新 */

/* 基本动作测试入口，可通过调试器 Watch 修改下面的全局变量：
 * 1. 先设 g_control_mode=0 并运行至少 10ms，让停止指令执行。
 * 2. 确认日志 fresh=1、trip=0/0，再写目标参数，最后设置运动模式。
 *    直行：g_target_vx=0.15f，g_control_mode=2；负速度表示后退。
 *    平移：g_target_vy=0.15f，g_control_mode=3；正值向左，负值向右。
 *    掉头：g_turn_angle_deg=180.0f，g_control_mode=5。
 * 3. 手动直行和平移持续执行，测试结束写 g_control_mode=0；转向结束自动停止。
 *    若要上电自动测试并定时停止，使用上面的 POWER_ON_TEST_* 配置宏。
 *
 * 上电默认停止。若用代码启动测试，应在外设初始化完成、IMU 数据有效后
 * 再设置运动模式；首帧未到时直接设置航向模式，会被失效检查清回停止。
 * 同时更新多个命令字段时使用短临界区，避免中断读取到不完整命令。 */
volatile ChassisControlMode_t g_control_mode = CHASSIS_CONTROL_STOP;
volatile float g_target_vx = 0.0f;       /* 前后速度，单位 m/s，正值向前 */
volatile float g_target_vy = 0.0f;       /* 横移速度，单位 m/s，正值向左 */
volatile float g_target_omega = 0.0f;    /* 仅速度模式使用，单位 rad/s，逆时针为正 */
volatile float g_turn_angle_deg = 90.0f; /* 相对转角，单位度，进入转向模式时锁存 */
/* 转向状态：0 空闲，1 转向中，2 到位，3 超时，4 越限。
 * 停止或保护触发时可能保留上次状态，判断是否运动需同时查看 mode/trip。
 * 普通正转角为逆时针、负转角为顺时针，超过 180° 时自动取最短路径。
 * 恰好 180° 的两侧路径等长，当前控制不保证固定掉头方向。 */
volatile uint8_t g_turn_state = OMNI_TURN_IDLE;
volatile PowerOnTestState_t g_power_on_test_state = POWER_ON_TEST_DISABLED;
volatile uint8_t g_remote_active;  /* 1：当前持续运动由串口速度帧控制 */
volatile uint8_t g_remote_timeout; /* 1：遥控超时、过期帧或串口接收故障 */

static char dma_uart3_tx_buffer[256]; /* DMA 发送未完成前不能改写该缓冲区 */
static ChassisControlMode_t last_control_mode = CHASSIS_CONTROL_STOP;
static float active_turn_angle_deg;   /* 当前转向锁存值，途中修改命令不影响本次动作 */

/* 上电测试与底盘控制都由 TIM7 调度，避免主循环与中断争用运动参数。 */
static uint32_t power_on_test_wait_start_tick;
static uint32_t power_on_test_motion_start_tick;
static ChassisControlMode_t power_on_test_mode;
static float power_on_test_speed;
static uint32_t power_on_test_duration_ms;

static uint32_t remote_last_frame_tick; /* 最近采用的完整有效帧接收时刻 */
static uint32_t remote_rearm_after_tick;
static uint8_t remote_rearm_required;  /* 故障后只接受停车屏障之后的新帧 */
static uint8_t remote_protection_seen; /* 记录保护状态，解除时丢弃旧队列指令 */

/* 给 Omni 转向控制提供航向角读取回调，单位度。 */
float Imu_GetYaw(void)
{
    return JY901P_GetYaw(&imu);
}

/* 返回 1 表示 IMU 已收到有效角度帧，且最近更新时间未超过 500ms。 */
uint8_t Imu_YawFresh(void)
{
    return JY901P_IsFresh(&imu, IMU_FRESH_TIMEOUT_MS);
}

/* 初始化遥控状态，不改变上电测试/手动控制的默认行为。 */
static void SerialRemote_Init(void)
{
    g_remote_active = 0U;
    g_remote_timeout = 0U;
    remote_last_frame_tick = 0U;
    remote_rearm_after_tick = 0U;
    remote_rearm_required = 0U;
    remote_protection_seen = 0U;
}

/* 有效遥控帧主动接管，包含 0,0,0 停车帧，避免等待中的上电测试后来启动。 */
static void SerialRemote_CancelPowerOnTest(void)
{
    if (g_power_on_test_state == POWER_ON_TEST_WAITING
        || g_power_on_test_state == POWER_ON_TEST_RUNNING)
        g_power_on_test_state = POWER_ON_TEST_ABORTED;
}

static void SerialRemote_Stop(uint32_t now)
{
    RemoteControl_InvalidateStream();
    g_target_vx = 0.0f;
    g_target_vy = 0.0f;
    g_target_omega = 0.0f;
    g_control_mode = CHASSIS_CONTROL_STOP;
    g_remote_active = 0U;
    g_remote_timeout = 1U;
    remote_rearm_required = 1U;
    remote_rearm_after_tick = now;
    SerialRemote_CancelPowerOnTest();
}

/* TIM7 独占运动参数更新；主循环只负责解析，不能直接给电机发控制量。
 * 数据格式 x,y,w\n，x/y 为 m/s，w 为 rad/s。0,0,0\n 立即请求停车。
 * 全部速度帧采用三轮速度闭环，不锁定 IMU 航向；航向仍可由手动模式控制。
 * 连续运动建议每 10ms（100Hz）更新一次，500ms 未收到有效帧则停车。 */
static void SerialRemote_BeforeControl(void)
{
    RemoteControlCommand_t command;
    uint32_t now = HAL_GetTick();
    uint32_t timeout_ms = (uint32_t)REMOTE_CONTROL_TIMEOUT_MS;
    uint8_t fault = RemoteControl_TakeFault();
    uint8_t received = RemoteControl_TakeCommand(&command);

    /* 保护不能由串口数据解除；保护期间消费并丢弃命令，底盘控制负责刹车。 */
    if (MotorSafety_Tripped() || Omni_SafetyTripped()) {
        if (!remote_protection_seen)
            RemoteControl_InvalidateStream();
        g_control_mode = CHASSIS_CONTROL_STOP;
        g_target_vx = g_target_vy = g_target_omega = 0.0f;
        g_remote_active = 0U;
        remote_protection_seen = 1U;
        remote_rearm_required = 1U;
        remote_rearm_after_tick = now;
        return;
    }
    if (remote_protection_seen) {
        RemoteControl_InvalidateStream();
        /* 保护刚解除：丢弃本周期命令，避免先前接收、延迟解析的帧恢复旧运动。 */
        remote_protection_seen = 0U;
        remote_rearm_required = 1U;
        remote_rearm_after_tick = now;
        return;
    }

    if (fault) {
        /* 已接管运动时故障停车；空闲串口故障不接管上电测试/手动运动。
         * 本周期获取到的命令也丢弃，恢复后必须发送新的完整有效帧。 */
        if (g_remote_active)
            SerialRemote_Stop(now);
        else {
            RemoteControl_InvalidateStream();
            g_remote_timeout = 1U;
            remote_rearm_required = 1U;
            remote_rearm_after_tick = now;
        }
        return;
    }

    /* 先执行已到期的看门狗，不能让刚补齐的旧半帧覆盖本次停车屏障。 */
    if (g_remote_active && g_control_mode == CHASSIS_CONTROL_VELOCITY
        && now - remote_last_frame_tick >= timeout_ms) {
        SerialRemote_Stop(now);
        return;
    }

    if (received) {
        uint32_t age = now - command.received_tick;
        /* 使用真实接收时间，堵住主循环积压后把旧帧当成新命令的路径。 */
        if (age >= timeout_ms) {
            g_remote_timeout = 1U;
            if (g_remote_active)
                SerialRemote_Stop(now);
            else
                RemoteControl_InvalidateStream();
            return;
        }
        /* 比较无符号时间差，兼容 tick 回绕及长时间断线后的重新连接。 */
        if (remote_rearm_required && age >= now - remote_rearm_after_tick)
            return;

        SerialRemote_CancelPowerOnTest();
        g_target_vx = command.x;
        g_target_vy = command.y;
        g_target_omega = command.w;
        g_turn_angle_deg = 0.0f; /* 速度遥控不使用相对转角，清理之前的命令值 */
        remote_last_frame_tick = command.received_tick;
        remote_rearm_required = 0U;
        g_remote_timeout = 0U;
        g_remote_active = command.x != 0.0f || command.y != 0.0f || command.w != 0.0f;
        g_control_mode = g_remote_active ? CHASSIS_CONTROL_VELOCITY : CHASSIS_CONTROL_STOP;
    }

    if (g_remote_active) {
        if (g_control_mode != CHASSIS_CONTROL_VELOCITY)
            g_remote_active = 0U; /* 调试器切换了手动模式，退出当前遥控看门狗 */
        else if (now - remote_last_frame_tick >= timeout_ms)
            SerialRemote_Stop(now);
    }
}

/* 底盘控制可能在本周期触发保护；失效后不保留遥控运动的活动状态。 */
static void SerialRemote_AfterControl(void)
{
    if (g_remote_active && g_control_mode != CHASSIS_CONTROL_VELOCITY)
        g_remote_active = 0U;
    if (MotorSafety_Tripped() || Omni_SafetyTripped()) {
        if (!remote_protection_seen)
            RemoteControl_InvalidateStream();
        g_control_mode = CHASSIS_CONTROL_STOP;
        g_target_vx = g_target_vy = g_target_omega = 0.0f;
        g_remote_active = 0U;
        remote_protection_seen = 1U;
        remote_rearm_required = 1U;
        remote_rearm_after_tick = HAL_GetTick();
    }
}

/* 在 TIM7 启动前装载配置。禁用时不接管手动命令；配置错误时保持停止。 */
static void PowerOnTest_Init(void)
{
    uint32_t delay_ms = (uint32_t)POWER_ON_TEST_START_DELAY_MS;
    uint32_t wait_ms = (uint32_t)POWER_ON_TEST_IMU_WAIT_TIMEOUT_MS;

    g_power_on_test_state = POWER_ON_TEST_DISABLED;
    if (POWER_ON_TEST_ENABLE == 0U)
        return;

    g_control_mode = CHASSIS_CONTROL_STOP;
    power_on_test_mode = (ChassisControlMode_t)POWER_ON_TEST_MODE;
    power_on_test_speed = 0.0f;
    power_on_test_duration_ms = 0U;

    /* 每段计时限制在 uint32_t 的半个周期以内，排除负时长转换及无效配置。
     * 用 C 枚举比较动作，不能用 #if 比较枚举名。 */
    if (POWER_ON_TEST_ENABLE != 1U || delay_ms > 0x7FFFFFFFU
        || wait_ms == 0U || wait_ms > 0x7FFFFFFFU) {
        g_power_on_test_state = POWER_ON_TEST_ABORTED;
        return;
    }

    switch (power_on_test_mode) {
    case CHASSIS_CONTROL_STRAIGHT:
        power_on_test_speed = POWER_ON_TEST_STRAIGHT_SPEED_MPS;
        power_on_test_duration_ms = (uint32_t)POWER_ON_TEST_STRAIGHT_DURATION_MS;
        break;
    case CHASSIS_CONTROL_LATERAL:
        power_on_test_speed = POWER_ON_TEST_LATERAL_SPEED_MPS;
        power_on_test_duration_ms = (uint32_t)POWER_ON_TEST_LATERAL_DURATION_MS;
        break;
    case CHASSIS_CONTROL_TURN:
        break; /* 固定 180°，完成判定和运动超时由 Omni 转向控制负责 */
    default:
        g_power_on_test_state = POWER_ON_TEST_ABORTED;
        return;
    }

    if (power_on_test_mode != CHASSIS_CONTROL_TURN
        && (!isfinite(power_on_test_speed) || power_on_test_speed == 0.0f
            || power_on_test_duration_ms == 0U
            || power_on_test_duration_ms > 0x7FFFFFFFU)) {
        g_power_on_test_state = POWER_ON_TEST_ABORTED;
        return;
    }

    power_on_test_wait_start_tick = HAL_GetTick();
    g_power_on_test_state = POWER_ON_TEST_WAITING;
}

/* 每个控制周期先处理启动/停止条件，再让 Chassis_ControlTick 输出一次控制。
 * 使用无符号时间差，HAL 毫秒计数器回绕时仍能正确计时。 */
static void PowerOnTest_BeforeControl(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t delay_ms = (uint32_t)POWER_ON_TEST_START_DELAY_MS;
    uint32_t wait_ms = (uint32_t)POWER_ON_TEST_IMU_WAIT_TIMEOUT_MS;

    if (g_power_on_test_state != POWER_ON_TEST_WAITING
        && g_power_on_test_state != POWER_ON_TEST_RUNNING)
        return;

    /* 保护优先于启动及正常结束；触发后取消测试，不能误报完成。 */
    if (MotorSafety_Tripped() || Omni_SafetyTripped()) {
        g_control_mode = CHASSIS_CONTROL_STOP;
        g_power_on_test_state = POWER_ON_TEST_ABORTED;
        return;
    }

    /* 手动切换模式立即退出自动测试，保留新命令交给基础控制处理。
     * 运行中设 STOP 会停车且不重启；等待阶段已是 STOP，重复写 STOP 不算切换。 */
    if ((g_power_on_test_state == POWER_ON_TEST_WAITING
         && g_control_mode != CHASSIS_CONTROL_STOP)
        || (g_power_on_test_state == POWER_ON_TEST_RUNNING
            && g_control_mode != power_on_test_mode)) {
        g_power_on_test_state = POWER_ON_TEST_ABORTED;
        return;
    }

    /* 直行/平移使能关闭也取消测试，防止后来恢复使能时突然继续执行。 */
    if ((power_on_test_mode == CHASSIS_CONTROL_STRAIGHT && !g_drive_straight_enable)
        || (power_on_test_mode == CHASSIS_CONTROL_LATERAL && !g_drive_lateral_enable)) {
        g_control_mode = CHASSIS_CONTROL_STOP;
        g_power_on_test_state = POWER_ON_TEST_ABORTED;
        return;
    }

    if (g_power_on_test_state == POWER_ON_TEST_WAITING) {
        uint32_t elapsed = now - power_on_test_wait_start_tick;
        if (elapsed < delay_ms)
            return;
        /* 到等待截止时刻即取消；IMU 此后恢复也不自动重试。 */
        if (elapsed - delay_ms >= wait_ms) {
            g_power_on_test_state = POWER_ON_TEST_ABORTED;
            return;
        }
        if (!Imu_YawFresh())
            return;

        g_target_vx = 0.0f;
        g_target_vy = 0.0f;
        g_target_omega = 0.0f;
        if (power_on_test_mode == CHASSIS_CONTROL_STRAIGHT)
            g_target_vx = power_on_test_speed;
        else if (power_on_test_mode == CHASSIS_CONTROL_LATERAL)
            g_target_vy = power_on_test_speed;
        else
            g_turn_angle_deg = 180.0f;

        /* 本周期紧接着执行底盘控制，直行/平移的时长从首次驱动周期开始。 */
        power_on_test_motion_start_tick = now;
        g_control_mode = power_on_test_mode;
        g_power_on_test_state = POWER_ON_TEST_RUNNING;
        return;
    }

    if (!Imu_YawFresh()) {
        g_control_mode = CHASSIS_CONTROL_STOP;
        g_power_on_test_state = POWER_ON_TEST_ABORTED;
    } else if (power_on_test_mode != CHASSIS_CONTROL_TURN
               && now - power_on_test_motion_start_tick >= power_on_test_duration_ms) {
        /* 到时先切 STOP，再执行本周期控制；最多有一个 10ms 周期的量化误差。 */
        g_control_mode = CHASSIS_CONTROL_STOP;
        g_power_on_test_state = POWER_ON_TEST_DONE;
    }
}

/* 执行底盘控制后同步结果，区分转向到位与超时/越限/失效取消。 */
static void PowerOnTest_AfterControl(void)
{
    if (g_power_on_test_state != POWER_ON_TEST_RUNNING)
        return;
    if (MotorSafety_Tripped() || Omni_SafetyTripped()) {
        g_control_mode = CHASSIS_CONTROL_STOP;
        g_power_on_test_state = POWER_ON_TEST_ABORTED;
    } else if (g_control_mode != power_on_test_mode) {
        g_power_on_test_state = power_on_test_mode == CHASSIS_CONTROL_TURN
            && g_turn_state == OMNI_TURN_DONE
            ? POWER_ON_TEST_DONE : POWER_ON_TEST_ABORTED;
    }
}

/* 每 10ms 执行一次；每周期只调用一种运动 API，避免速度 PID 重复计算。 */
static void Chassis_ControlTick(void)
{
    ChassisControlMode_t mode = g_control_mode;
    float vx = g_target_vx;
    float vy = g_target_vy;
    float omega = g_target_omega;

    /* 超速或失控保护锁存时取消当前命令并保持刹车。
     * 排除故障、清除保护后，必须重新下发模式，旧运动不会自动恢复。 */
    if (MotorSafety_Tripped() || Omni_SafetyTripped()) {
        g_control_mode = CHASSIS_CONTROL_STOP;
        last_control_mode = CHASSIS_CONTROL_STOP;
        Omni_BrakeStop(&omni);
        return;
    }

    /* 非法模式或 NaN/无穷大参数直接转为停止，避免无效值进入运动学计算。 */
    if (mode < CHASSIS_CONTROL_STOP || mode > CHASSIS_CONTROL_TURN
        || !isfinite(vx) || !isfinite(vy) || !isfinite(omega)
        || !isfinite(g_turn_angle_deg)) {
        g_control_mode = CHASSIS_CONTROL_STOP;
        mode = CHASSIS_CONTROL_STOP;
    }

    /* 切换模式时停止上一动作，并清除速度/航向 PID 历史，重新锁定航向。 */
    if (mode != last_control_mode) {
        Omni_TurnStop(&omni);
        if (mode == CHASSIS_CONTROL_TURN) {
            /* 新的一次转向只在进入模式时初始化，不能每个周期重新初始化。
             * 转向参数与启动初始化共用 chassis_config.h 中的配置。 */
            Omni_TurnInit(&omni, CHASSIS_TURN_YAW_KP,
                          CHASSIS_TURN_YAW_KI, CHASSIS_TURN_YAW_KD);
            /* 先取余，避免极大有限转角使后续角度归一化循环无法收敛。 */
            active_turn_angle_deg = fmodf(g_turn_angle_deg, 360.0f);
            g_turn_state = OMNI_TURN_IDLE;
        }
        last_control_mode = mode;
    }

    /* 速度模式使用编码器闭环，不依赖 IMU。
     * 直行、平移、组合运动和转向要求 IMU 数据有效；失效后取消动作，
     * IMU 恢复也不会自动续跑，需要重新设置运动模式。 */
    if (mode >= CHASSIS_CONTROL_STRAIGHT && !Imu_YawFresh()) {
        g_control_mode = CHASSIS_CONTROL_STOP;
        last_control_mode = CHASSIS_CONTROL_STOP;
        Omni_TurnStop(&omni);
        g_turn_state = OMNI_TURN_IDLE;
        return;
    }

    switch (mode) {
    case CHASSIS_CONTROL_VELOCITY:
        /* 模式 1：按 vx/vy/omega 合成轮速，执行速度闭环，不保持航向。 */
        Omni_SetVelocity(&omni, vx, vy, omega);
        break;
    case CHASSIS_CONTROL_STRAIGHT:
        /* 模式 2：前后直行，首次执行时锁定当前航向，并持续修正偏航。 */
        Omni_DriveStraight(&omni, vx, Imu_GetYaw());
        break;
    case CHASSIS_CONTROL_LATERAL:
        /* 模式 3：左右平移，使用独立的平移航向 PID 保持车头方向。 */
        Omni_DriveLateral(&omni, vy, Imu_GetYaw());
        break;
    case CHASSIS_CONTROL_COMBINED:
        /* 模式 4：同时执行前后和横向运动，与直行共用航向 PID。 */
        Omni_DriveCombined(&omni, vx, vy, Imu_GetYaw());
        break;
    case CHASSIS_CONTROL_TURN:
        /* 模式 5：单次相对转向；角度、超时和越限判定在 Omni 模块完成。 */
        Omni_TurnBy(&omni, active_turn_angle_deg);
        g_turn_state = (uint8_t)Omni_TurnGetState(&omni);
        if (g_turn_state == OMNI_TURN_DONE
            || g_turn_state == OMNI_TURN_TIMEOUT
            || g_turn_state == OMNI_TURN_OVERSHOOT) {
            /* 保存本次终态供日志查看，同时回到停止，允许下一次转向重新初始化。
             * 越限会锁存保护，必须排除故障并清除保护或复位后才能再次运动。 */
            g_control_mode = CHASSIS_CONTROL_STOP;
            last_control_mode = CHASSIS_CONTROL_STOP;
        }
        break;
    case CHASSIS_CONTROL_STOP:
    default:
        /* 模式 0：输出零 PWM，滑行停止，并清除速度和行驶航向环历史。 */
        Omni_Stop(&omni);
        break;
    }
}
/* 外设 MX_* 初始化后调用，集中保持初始化顺序。 */
static void Chassis_Init(void)
{
    if (!Encoder_InitOmni(&encoder1, &encoder2, &encoder3) || !Motor_Init())
        Error_Handler();

    MotorSafety_Init();
    inc_pid_init(&motor_A_pid, CHASSIS_SPEED_KP, CHASSIS_SPEED_A_KI,
                 CHASSIS_SPEED_KD, CHASSIS_SPEED_OUTPUT_LIMIT,
                 CHASSIS_SPEED_INCREMENT_LIMIT);
    inc_pid_init(&motor_B_pid, CHASSIS_SPEED_KP, CHASSIS_SPEED_B_KI,
                 CHASSIS_SPEED_KD, CHASSIS_SPEED_OUTPUT_LIMIT,
                 CHASSIS_SPEED_INCREMENT_LIMIT);
    inc_pid_init(&motor_C_pid, CHASSIS_SPEED_KP, CHASSIS_SPEED_C_KI,
                 CHASSIS_SPEED_KD, CHASSIS_SPEED_OUTPUT_LIMIT,
                 CHASSIS_SPEED_INCREMENT_LIMIT);
    Omni_Init(&omni, &encoder1, &encoder2, &encoder3);
    Omni_StraightInit(&omni, CHASSIS_STRAIGHT_YAW_KP,
                      CHASSIS_STRAIGHT_YAW_KI, CHASSIS_STRAIGHT_YAW_KD);
    Omni_LateralInit(&omni, CHASSIS_LATERAL_YAW_KP,
                     CHASSIS_LATERAL_YAW_KI, CHASSIS_LATERAL_YAW_KD);
    Omni_TurnInit(&omni, CHASSIS_TURN_YAW_KP,
                  CHASSIS_TURN_YAW_KI, CHASSIS_TURN_YAW_KD);
    JY901P_Init(&imu, &huart2);
    SerialRemote_Init();
    RemoteControl_Init(&huart1);
    PowerOnTest_Init();

    /* 所有模块就绪后才启动编码器采样与运动控制。 */
    if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
        Error_Handler();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  Chassis_Init();
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    RemoteControl_Process(); /* 只解析串口数据及恢复接收，运动命令由 TIM7 消费 */
    /* 在主循环格式化并发送日志，避免控制中断中执行耗时的浮点格式化。
     * 仅在上一帧 DMA 发送完成后写缓冲区，防止覆盖正在发送的数据。
     * 字段：mode 模式；x/y/w 当前速度目标（m/s、m/s、rad/s）；
     * A/B/C 实测 RPM；yaw 航向角；fresh IMU 有效标志；
     * turn 转向状态；ptest 上电测试状态（0 关闭/1 等待/2 运行/3 完成/4 取消）；
     * rc 遥控持续运动标志；rto 遥控超时/故障标志；
     * trip 两个值分别表示超速保护/失控保护是否锁存。 */
    static uint32_t last_debug_tick;
    uint32_t now = HAL_GetTick();
    if (now - last_debug_tick >= DEBUG_PRINT_PERIOD_MS
        && HAL_UART_GetState(&huart3) == HAL_UART_STATE_READY) {
      last_debug_tick = now;
      /* TIM7 会同时更新这些字段；只在短临界区复制，不在关中断时格式化。 */
      const uint32_t log_primask = __get_PRIMASK();
      __disable_irq();
      const unsigned log_mode = (unsigned)g_control_mode;
      const float log_x = g_target_vx;
      const float log_y = g_target_vy;
      const float log_w = g_target_omega;
      const float log_rpm_a = RPM_A;
      const float log_rpm_b = RPM_B;
      const float log_rpm_c = RPM_C;
      const unsigned log_turn = (unsigned)g_turn_state;
      const unsigned log_ptest = (unsigned)g_power_on_test_state;
      const unsigned log_rc = (unsigned)g_remote_active;
      const unsigned log_rto = (unsigned)g_remote_timeout;
      __set_PRIMASK(log_primask);
      snprintf(dma_uart3_tx_buffer, sizeof(dma_uart3_tx_buffer),
               "mode=%u x=%.2f y=%.2f w=%.2f A=%.1f B=%.1f C=%.1f yaw=%.2f fresh=%u turn=%u ptest=%u rc=%u rto=%u trip=%u/%u\r\n",
               log_mode, (double)log_x, (double)log_y, (double)log_w,
               (double)log_rpm_a, (double)log_rpm_b, (double)log_rpm_c,
               (double)Imu_GetYaw(), (unsigned)Imu_YawFresh(), log_turn, log_ptest,
               log_rc, log_rto,
               (unsigned)MotorSafety_Tripped(),
               (unsigned)Omni_SafetyTripped());
      HAL_UART_Transmit_DMA(&huart3, (uint8_t *)dma_uart3_tx_buffer,
                            (uint16_t)strlen(dma_uart3_tx_buffer));
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM7)
        return;

    /* TIM7：84MHz / (84 × 10000)，每 10ms 采样一次编码器并执行控制。
     * 从定时器实际配置计算周期，保证编码器 RPM 换算使用正确采样时间。 */
    float period_s = Encoder_GetTimerPeriodS(htim);
    int valid_a = Encoder_UpdateRPM(&encoder1, period_s);
    int valid_b = Encoder_UpdateRPM(&encoder2, period_s);
    int valid_c = Encoder_UpdateRPM(&encoder3, period_s);
    /* 保存轮速供控制和主循环日志读取；任一采样失败均进入故障停机。 */
    RPM_A = Encoder_GetRPM(&encoder1);
    RPM_B = Encoder_GetRPM(&encoder2);
    RPM_C = Encoder_GetRPM(&encoder3);
    if (!valid_a || !valid_b || !valid_c)
        Error_Handler();

    /* 先检查超速，再执行运动，保证本周期发现的故障立即阻止继续驱动。 */
    MotorSafety_Check(RPM_A, RPM_B, RPM_C);
    SerialRemote_BeforeControl(); /* 遥控优先接管/停车；不清除保护 */
    PowerOnTest_BeforeControl(); /* 自动测试启动/定时结束/取消，均在中断内更新命令 */
    Chassis_ControlTick();
    PowerOnTest_AfterControl();  /* 底盘控制执行后读取转向结果，测试只运行一次 */
    SerialRemote_AfterControl(); /* 同步本周期保护引起的遥控取消 */
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* DMA/空闲接收事件交给 IMU 驱动；驱动内部只处理注册的 USART2。 */
    JY901P_RxCallback(huart, Size);
    RemoteControl_UartRxEventCallback(huart, Size); /* 仅处理注册的 USART1 */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* UART 出错时交给 IMU 驱动恢复接收。 */
    JY901P_UartErrorCallback(huart);
    RemoteControl_UartErrorCallback(huart); /* USART1 异常上报故障并由主循环恢复 */
}

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
    /* IMU 使用异步 Abort 恢复接收；完成后由驱动重启 USART2 DMA。 */
    JY901P_UartAbortReceiveCpltCallback(huart);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  /* 运行中发生HAL/参数错误时，先把电机置于硬件刹车态，再停机等待复位。 */
  Motor_A_Brake();
  Motor_B_Brake();
  Motor_C_Brake();
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
