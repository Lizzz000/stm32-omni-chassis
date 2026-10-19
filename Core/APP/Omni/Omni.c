#include "Omni.h"
#include "motor.h"
#include "motor_safety.h"
#include "pid.h"
#include "tim.h"
#include <math.h>
#include <string.h>

/* 120° 布局三角函数常量 */
#define OMNI_SIN60   0.8660254f  /* sin 60° */
#define OMNI_COS60   0.5f        /* cos 60° */
#define OMNI_COS180 -1.0f        /* cos 180° */
#define OMNI_1_SQRT3 0.5773503f  /* 1/√3 */

/* 轮线速度(m/s) → 电机转速(RPM): RPM = v / (2πR) × 60 */
#define OMNI_MS_TO_RPM  (60.0f / (2.0f * 3.14159265f * OMNI_WHEEL_RADIUS))

extern inc_pid_t motor_A_pid;
extern inc_pid_t motor_B_pid;
extern inc_pid_t motor_C_pid;

/* main.c 定义的行驶使能标志(直线行驶与平移分开): 1=可以行驶, 0=不运动 */
extern volatile uint8_t g_drive_straight_enable;
extern volatile uint8_t g_drive_lateral_enable;

/* ---- 防疯转保护参数 ---- */
#define OMNI_SAFETY_RPM_ABS_MIN    50.0f  /* 判定超速的绝对下限(RPM), 防止低速目标误报 */
#define OMNI_SAFETY_TARGET_RPM_MIN 10.0f  /* 低速减速/停车阶段不做疯转判定; 原50使低速接近段完全失保 */
#define OMNI_SAFETY_RPM_OVERSHOOT  2.0f   /* 实测|RPM| > |目标|×2 + ABS_MIN 视为超速 */
#define OMNI_SAFETY_TRIP_CYCLES    (250U / CHASSIS_CONTROL_PERIOD_MS) /* 210ms */
#define OMNI_SAFETY_PWM_DEADBAND   100.0f /* |PWM| 距 ARR 限幅小于该值视为饱和 */

static volatile uint8_t s_runaway_cnt[3];
static volatile uint8_t s_safety_tripped;

/* 速度环抗积分饱和(条件积分): 前馈+PID 已超出 PWM 限幅且本周期增量仍朝
 * 饱和方向时, 撤销本周期增量。否则轮子跟不上目标期间增量式 PID 累计输出
 * 持续增长, 目标回落后需要很多周期才能退饱和, 表现为长时间超速。 */
static void _speed_antiwindup(Omni_Wheel_t *wheel, inc_pid_t *pid, float ff_gain,
                              float limit, float previous_output)
{
    const float u = wheel->target_rpm * ff_gain + wheel->speed_output;

    if (!isfinite(u))
        return;
    if ((u > limit && wheel->speed_output > previous_output)
        || (u < -limit && wheel->speed_output < previous_output)) {
        wheel->speed_output = previous_output;
        pid->output = previous_output;
    }
}

/* 对称限幅: [-limit, +limit] */
static float _clamp_f(float value, float limit)
{
    /* NaN 不满足任何比较，若直接透传再转 int16 属未定义行为，按停转处理。 */
    if (!isfinite(value)) return 0.0f;
    if (value > limit)  return limit;
    if (value < -limit) return -limit;
    return value;
}

/* 角度归一化到[-180°, +180°]。JY901P从+180°跳到-180°只是数值环绕，
 * 不是反向转了360°；使用while也能防御意外超过一圈的输入。 */
static float _wrap_deg180(float deg)
{
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* 只清PID运行历史，保留已经整定好的kp/ki/kd和限幅参数。
 * 定点转向前必须清掉直行/横移航向环的积分和上次误差，否则恢复基本行驶
 * 的第一周期可能带入转向前的控制量。 */
static void _pid_clear_runtime(pid_struct_t *pid)
{
    pid->get = 0.0f;
    pid->set = 0.0f;
    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->output = 0.0f;
}

static void _speed_pid_clear_runtime(inc_pid_t *pid)
{
    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
    pid->err[2] = 0.0f;
    pid->output = 0.0f;
}

void Omni_Init(Omni_t *omni, Encoder_t *enc_a, Encoder_t *enc_b, Encoder_t *enc_c)
{
    /* 全结构清零兜底: 若调用方漏调 Straight/Lateral/TurnInit,
     * yaw PID 等字段不会残留垃圾值(此前依赖 BSS 零初始化) */
    memset(omni, 0, sizeof(*omni));

    omni->wheel_a.encoder = enc_a;
    omni->wheel_b.encoder = enc_b;
    omni->wheel_c.encoder = enc_c;

    omni->wheel_a.dir = OMNI_A_DIR;
    omni->wheel_b.dir = OMNI_B_DIR;
    omni->wheel_c.dir = OMNI_C_DIR;

    omni->wheel_a.target_rpm = 0.0f;
    omni->wheel_b.target_rpm = 0.0f;
    omni->wheel_c.target_rpm = 0.0f;

    /* PWM 输出上限取电机 PWM 定时器(TIM1/2/3)的最小 ARR。
     * 注意: 本工程的 htim5 是编码器定时器(32位 ARR=0xFFFFFFFF),
     * 读它截断成 int16_t 会得到 -1, 导致限幅把所有输出压成 ±1 */
    {
        uint16_t max_pwm = Motor_GetMaxPWM();
        /* ARR>32767 时直接截断为 int16_t 会变成负数，饱和到 INT16_MAX。 */
        omni->pwm_max = (max_pwm > (uint16_t)INT16_MAX) ? INT16_MAX : (int16_t)max_pwm;
    }

    Motor_A_SetPWM(0);
    Motor_B_SetPWM(0);
    Motor_C_SetPWM(0);
}

/* 单轮疯转检测: 实测远超目标 且 输出PWM饱和, 连续N周期判为疯转 */
static int Omni_CheckRunaway(const Omni_t *omni, const Omni_Wheel_t *wheel,
                             float rpm, uint8_t idx)
{
    const float margin = fabsf(wheel->target_rpm) * OMNI_SAFETY_RPM_OVERSHOOT
                       + OMNI_SAFETY_RPM_ABS_MIN;
    const float sat_level = (float)omni->pwm_max - OMNI_SAFETY_PWM_DEADBAND;

    if (fabsf(wheel->target_rpm) >= OMNI_SAFETY_TARGET_RPM_MIN
        && fabsf(rpm) > margin
        && fabsf(wheel->ctr_input) >= sat_level) {
        if (++s_runaway_cnt[idx] >= OMNI_SAFETY_TRIP_CYCLES)
            return 1;
    } else {
        s_runaway_cnt[idx] = 0U;
    }
    return 0;
}

/* 触发保护: 三路全停 + 清零增量PID状态, 防止清除后输出跳变 */
static void Omni_SafetyTrip(Omni_t *omni)
{
    /* Use the same active-brake path as an already-latched safety trip. */
    Omni_BrakeStop(omni);
    motor_A_pid.output = 0.0f;
    motor_A_pid.err[0] = motor_A_pid.err[1] = motor_A_pid.err[2] = 0.0f;
    motor_B_pid.output = 0.0f;
    motor_B_pid.err[0] = motor_B_pid.err[1] = motor_B_pid.err[2] = 0.0f;
    motor_C_pid.output = 0.0f;
    motor_C_pid.err[0] = motor_C_pid.err[1] = motor_C_pid.err[2] = 0.0f;
    s_safety_tripped = 1U;
}

uint8_t Omni_SafetyTripped(void)
{
    return s_safety_tripped;
}

void Omni_SafetyClear(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_safety_tripped = 0U;
    s_runaway_cnt[0] = s_runaway_cnt[1] = s_runaway_cnt[2] = 0U;
    __DMB();
    if (primask == 0U)
        __enable_irq();
}

void Omni_SetVelocity(Omni_t *omni, float vx, float vy, float omega)//直走: b顺时针, c逆时针
{
    float previous_target_a;
    float previous_target_b;
    float previous_target_c;

    if (omni == NULL)
        return;

    /* 防疯转保护已触发: 保持停机, 直到 Omni_SafetyClear() 清除 */
    if (s_safety_tripped || MotorSafety_Tripped()) {
        Omni_BrakeStop(omni);
        return;
    }

    /* 逆运动学: 目标正 = 轮子逆时针(CCW) = 滚向 -t_i。
     * 三轮标定: 正PWM=顺时针, 编码器顺时针=正计数, dir 全部 -1。
     * 左移(vy>0): A 逆时针, B/C 顺时针, A 速度为 B/C 的 2 倍。 */
    float va = vy                                       - omega * OMNI_WHEEL_BASE; /* 后轮 */
    float vb = -OMNI_SIN60 * vx - OMNI_COS60 * vy       - omega * OMNI_WHEEL_BASE; /* 右前 */
    float vc =  OMNI_SIN60 * vx - OMNI_COS60 * vy       - omega * OMNI_WHEEL_BASE; /* 左前 */

    /* 线速度 → 目标 RPM, 限幅防电机超速 */
    previous_target_a = omni->wheel_a.target_rpm;
    previous_target_b = omni->wheel_b.target_rpm;
    previous_target_c = omni->wheel_c.target_rpm;
    {
        /* 任一轮超出 OMNI_MAX_RPM 时三轮按同一比例缩放, 保持合成速度方向
         * (vx:vy:omega 比例)不变; 逐轮单独限幅会使运动方向偏转。 */
        float rpm_a = va * OMNI_MS_TO_RPM;
        float rpm_b = vb * OMNI_MS_TO_RPM;
        float rpm_c = vc * OMNI_MS_TO_RPM;
        float peak = fmaxf(fabsf(rpm_a), fmaxf(fabsf(rpm_b), fabsf(rpm_c)));
        if (isfinite(peak) && peak > OMNI_MAX_RPM) {
            const float scale = OMNI_MAX_RPM / peak;
            rpm_a *= scale;
            rpm_b *= scale;
            rpm_c *= scale;
        }
        /* 保留逐轮限幅兜底(含 NaN → 0)。 */
        omni->wheel_a.target_rpm = _clamp_f(rpm_a, OMNI_MAX_RPM);
        omni->wheel_b.target_rpm = _clamp_f(rpm_b, OMNI_MAX_RPM);
        omni->wheel_c.target_rpm = _clamp_f(rpm_c, OMNI_MAX_RPM);
    }

    /* 目标换向或某一路归零时清除该路速度环历史，避免旧方向的累积
     * 输出在下一次调用中瞬间反向释放。 */
    if ((previous_target_a != 0.0f && omni->wheel_a.target_rpm != 0.0f
         && ((previous_target_a < 0.0f) != (omni->wheel_a.target_rpm < 0.0f)))
        || (previous_target_a != 0.0f && omni->wheel_a.target_rpm == 0.0f))
        _speed_pid_clear_runtime(&motor_A_pid);
    if ((previous_target_b != 0.0f && omni->wheel_b.target_rpm != 0.0f
         && ((previous_target_b < 0.0f) != (omni->wheel_b.target_rpm < 0.0f)))
        || (previous_target_b != 0.0f && omni->wheel_b.target_rpm == 0.0f))
        _speed_pid_clear_runtime(&motor_B_pid);
    if ((previous_target_c != 0.0f && omni->wheel_c.target_rpm != 0.0f
         && ((previous_target_c < 0.0f) != (omni->wheel_c.target_rpm < 0.0f)))
        || (previous_target_c != 0.0f && omni->wheel_c.target_rpm == 0.0f))
        _speed_pid_clear_runtime(&motor_C_pid);

    /* 三轮目标同时归零是明确停车请求，不让增量PID继续积累。 */
    if (omni->wheel_a.target_rpm == 0.0f
        && omni->wheel_b.target_rpm == 0.0f
        && omni->wheel_c.target_rpm == 0.0f) {
        Omni_Stop(omni);
        return;
    }

    // /* 开环输出: 目标 RPM → PWM(含方向修正) */
    // Omni_SetPWM(omni,
    //             Omni_WheelToPWM(&omni->wheel_a),
    //             Omni_WheelToPWM(&omni->wheel_b),
    //             Omni_WheelToPWM(&omni->wheel_c));

    /* 闭环PID: 实测RPM(含方向修正) vs 目标RPM → 速度环修正量 */
    float rpm_a = Encoder_GetRPM(omni->wheel_a.encoder) * omni->wheel_a.dir;
    float rpm_b = Encoder_GetRPM(omni->wheel_b.encoder) * omni->wheel_b.dir;
    float rpm_c = Encoder_GetRPM(omni->wheel_c.encoder) * omni->wheel_c.dir;

    const float previous_a = motor_A_pid.output;
    const float previous_b = motor_B_pid.output;
    const float previous_c = motor_C_pid.output;
    omni->wheel_a.speed_output = inc_pid_calc(&motor_A_pid, rpm_a, omni->wheel_a.target_rpm);
    omni->wheel_b.speed_output = inc_pid_calc(&motor_B_pid, rpm_b, omni->wheel_b.target_rpm);
    omni->wheel_c.speed_output = inc_pid_calc(&motor_C_pid, rpm_c, omni->wheel_c.target_rpm);

    /* 执行器饱和时回算 PID 累计量, 防止积分饱和(ctr_input 仍为饱和值, 不影响疯转检测)。 */
    _speed_antiwindup(&omni->wheel_a, &motor_A_pid, OMNI_A_FF_GAIN, (float)omni->pwm_max, previous_a);
    _speed_antiwindup(&omni->wheel_b, &motor_B_pid, OMNI_B_FF_GAIN, (float)omni->pwm_max, previous_b);
    _speed_antiwindup(&omni->wheel_c, &motor_C_pid, OMNI_C_FF_GAIN, (float)omni->pwm_max, previous_c);

    /* 前馈 + PID修正 = 最终 PWM，限幅后输出 */
    Omni_CalcCtrlInput(omni);

    /* 防疯转检测: 任一车轮超速且PWM饱和持续5周期 → 三路紧急停机 */
    if (Omni_CheckRunaway(omni, &omni->wheel_a, rpm_a, 0U)
        || Omni_CheckRunaway(omni, &omni->wheel_b, rpm_b, 1U)
        || Omni_CheckRunaway(omni, &omni->wheel_c, rpm_c, 2U)) {
        Omni_SafetyTrip(omni);
        return;
    }

    /* 防止TIM7在本函数计算期间刚好锁存超速保护后仍写出PWM。 */
    if (s_safety_tripped || MotorSafety_Tripped()) {
        Omni_BrakeStop(omni);
        return;
    }

    Omni_SetPWM(omni,
                (int16_t)omni->wheel_a.ctr_input,
                (int16_t)omni->wheel_b.ctr_input,
                (int16_t)omni->wheel_c.ctr_input);

}

void Omni_SetPWM(Omni_t *omni, int16_t pwm_a, int16_t pwm_b, int16_t pwm_c)
{
    if (omni == NULL || s_safety_tripped || MotorSafety_Tripped()) {
        if (omni != NULL)
            Omni_BrakeStop(omni);
        return;
    }
    Motor_A_SetPWM(pwm_a);
    Motor_B_SetPWM(pwm_b);
    Motor_C_SetPWM(pwm_c);
}

/**
  * @brief  闭环控制量计算: ctr_input = -(target_rpm × FF_GAIN + speed_output)
  * @note   前馈分量 = target_rpm × FF_GAIN（RPM → PWM 经验映射）
  *         PID修正量 = speed_output（速度环输出）
  *         本板实测: 正 PWM → 顺时针, 与运动学约定(逆时针为正)相反且三轮一致,
  *         故输出整体取反(等效于三路电机线同时对调);
  *         编码器约定(顺时针=正计数)不变, OMNI_*_DIR=-1 仍然正确
  */
void Omni_CalcCtrlInput(Omni_t *omni)
{
    /* 前馈 + PID修正 = 最终 PWM(整体取反适配本板电机方向) */
    omni->wheel_a.ctr_input = -(omni->wheel_a.target_rpm * OMNI_A_FF_GAIN + omni->wheel_a.speed_output);
    omni->wheel_b.ctr_input = -(omni->wheel_b.target_rpm * OMNI_B_FF_GAIN + omni->wheel_b.speed_output);
    omni->wheel_c.ctr_input = -(omni->wheel_c.target_rpm * OMNI_C_FF_GAIN + omni->wheel_c.speed_output);

    /* 限幅: [-pwm_max, pwm_max] */
    omni->wheel_a.ctr_input = _clamp_f(omni->wheel_a.ctr_input, (float)omni->pwm_max);
    omni->wheel_b.ctr_input = _clamp_f(omni->wheel_b.ctr_input, (float)omni->pwm_max);
    omni->wheel_c.ctr_input = _clamp_f(omni->wheel_c.ctr_input, (float)omni->pwm_max);
}

void Omni_Stop(Omni_t *omni)
{
    if (omni == NULL)
        return;

    omni->wheel_a.target_rpm = 0.0f;
    omni->wheel_b.target_rpm = 0.0f;
    omni->wheel_c.target_rpm = 0.0f;

    /* 停车时同步清零轮速环与控制量缓存，避免调试读取旧运动输出。 */
    omni->wheel_a.speed_output = 0.0f;
    omni->wheel_b.speed_output = 0.0f;
    omni->wheel_c.speed_output = 0.0f;
    omni->wheel_a.ctr_input = 0.0f;
    omni->wheel_b.ctr_input = 0.0f;
    omni->wheel_c.ctr_input = 0.0f;

    Motor_A_SetPWM(0);
    Motor_B_SetPWM(0);
    Motor_C_SetPWM(0);

    _speed_pid_clear_runtime(&motor_A_pid);
    _speed_pid_clear_runtime(&motor_B_pid);
    _speed_pid_clear_runtime(&motor_C_pid);
    /* 停止直行/平移后清除航向环历史，下一次启动以新锁存的Yaw为基准，
     * 避免上一段运动的积分项在新任务中产生瞬时旋转修正。 */
    _pid_clear_runtime(&omni->yaw_pid);
    _pid_clear_runtime(&omni->lateral_yaw_pid);
    omni->yaw_locked = 0U;
    omni->lateral_yaw_locked = 0U;

    /* 正常停车会中断“连续N周期”条件，不能把停车前的未完成计数带到
     * 下一次运动/转向，否则新动作第一周期可能被误判为第N周期。 */
    s_runaway_cnt[0] = s_runaway_cnt[1] = s_runaway_cnt[2] = 0U;
}

void Omni_BrakeStop(Omni_t *omni)
{
    if (omni == NULL)
        return;

    omni->wheel_a.target_rpm = 0.0f;
    omni->wheel_b.target_rpm = 0.0f;
    omni->wheel_c.target_rpm = 0.0f;

    /* 停车时同步清零轮速环与控制量缓存，避免调试读取旧运动输出。 */
    omni->wheel_a.speed_output = 0.0f;
    omni->wheel_b.speed_output = 0.0f;
    omni->wheel_c.speed_output = 0.0f;
    omni->wheel_a.ctr_input = 0.0f;
    omni->wheel_b.ctr_input = 0.0f;
    omni->wheel_c.ctr_input = 0.0f;

    Motor_A_Brake();
    Motor_B_Brake();
    Motor_C_Brake();

    _speed_pid_clear_runtime(&motor_A_pid);
    _speed_pid_clear_runtime(&motor_B_pid);
    _speed_pid_clear_runtime(&motor_C_pid);
    _pid_clear_runtime(&omni->yaw_pid);
    _pid_clear_runtime(&omni->lateral_yaw_pid);
    omni->yaw_locked = 0U;
    omni->lateral_yaw_locked = 0U;
    s_runaway_cnt[0] = s_runaway_cnt[1] = s_runaway_cnt[2] = 0U;
}

void Omni_GetVelocity(Omni_t *omni, float *vx, float *vy, float *omega)
{
    /* 编码器原始值 → 滚向 +t_i 的轮线速度 (m/s)。
     * 三轮均为: 顺时针=正计数, 且顺时针=滚向 +t_i, 原始值直接使用。 */
    float va = Encoder_GetRPM(omni->wheel_a.encoder) / OMNI_MS_TO_RPM;
    float vb = Encoder_GetRPM(omni->wheel_b.encoder) / OMNI_MS_TO_RPM;
    float vc = Encoder_GetRPM(omni->wheel_c.encoder) / OMNI_MS_TO_RPM;

    /* 正运动学(逆运动学矩阵求逆), B=右前(φ=-60°), C=左前(φ=+60°) */
    *vx = (vb - vc) * OMNI_1_SQRT3;
    *vy = (vb + vc - 2.0f * va) / 3.0f;
    *omega = (va + vb + vc) / (3.0f * OMNI_WHEEL_BASE);
}

float Omni_GetWheelRPM(const Omni_Wheel_t *wheel)
{
    return Encoder_GetRPM(wheel->encoder);
}

/**
  * @brief  单轮目标转速 → PWM 比较值(含方向修正)
  * @param  wheel  轮子实例指针
  * @return int16_t  PWM 比较值 (±, 符号决定转向)
  * @note   公式: PWM = target_rpm × OMNI_FF_GAIN × dir
  *         其中 dir = ±1, 由标定决定:
  *         正 PWM 的转向与运动学正方向一致为 +1, 相反为 -1
  */
int16_t Omni_WheelToPWM(const Omni_Wheel_t *wheel)
{
    /* 浮点转 int16 超范围是未定义行为，先做有限性检查和饱和。 */
    float pwm = wheel->target_rpm * OMNI_FF_GAIN * (float)wheel->dir;
    if (!isfinite(pwm))
        return 0;
    if (pwm > (float)INT16_MAX)
        return INT16_MAX;
    if (pwm < (float)INT16_MIN)
        return INT16_MIN;
    return (int16_t)pwm;
}

/* ==================== 直线行驶（串级PID） ==================== */

/**
  * @brief  初始化直线行驶角度环 PID
  * @param  kp/ki/kd  角度环 PID 参数
  * @note   i_max=2.0 (积分限幅), out_max=3.0 rad/s (最大角速度)
  */
void Omni_StraightInit(Omni_t *omni, float kp, float ki, float kd)
{
    pid_init(&omni->yaw_pid, kp, ki, kd, 2.0f, OMNI_DRIVE_MAX_OMEGA);
    omni->target_yaw  = 0.0f;
    omni->yaw_locked  = 0;
}

/**
  * @brief  初始化平移角度环 PID(与直线行驶分开, 不共用)
  * @param  kp/ki/kd  角度环 PID 参数
  * @note   i_max=2.0 (积分限幅), out_max=3.0 rad/s (最大角速度);
  *         使用独立的 lateral_yaw_pid/lateral_target_yaw/lateral_yaw_locked
  */
void Omni_LateralInit(Omni_t *omni, float kp, float ki, float kd)
{
    pid_init(&omni->lateral_yaw_pid, kp, ki, kd, 2.0f, OMNI_DRIVE_MAX_OMEGA);
    omni->lateral_target_yaw   = 0.0f;
    omni->lateral_yaw_locked   = 0;
}

/**
  * @brief  直线行驶控制（每控制周期调用）
  * @param  vx          前进速度 (m/s)
  * @param  current_yaw 当前航向角 (°, 来自 IMU)
  * @note   首次调用自动锁定当前 yaw；之后角度环持续修正偏航
  */
void Omni_DriveStraight(Omni_t *omni, float vx, float current_yaw)
{
    /* 直线行驶使能标志判断: 未收到"可以行驶"(标志为 0)时不动作 */
    if (!g_drive_straight_enable) {
        Omni_Stop(omni);
        omni->yaw_locked = 0; /* 解除锁定, 重新使能后按当前航向重新锁定 */
        return;
    }

    /* 首次锁定目标航向 */
    if (!omni->yaw_locked) {
        omni->target_yaw = current_yaw;
        omni->yaw_locked = 1;
    }

    /* 计算最短路径角度误差（处理 ±180° 环绕） */
    float yaw_err = _wrap_deg180(omni->target_yaw - current_yaw);

    /* 角度环: yaw误差 → omega角速度修正 */
    float omega = pid_calc_by_error(&omni->yaw_pid, yaw_err);

    /* 调用全向运动学: vx前进 + omega旋转修正 → 内部速度环自动执行 */
    Omni_SetVelocity(omni, vx, 0.0f, omega);
}

/**
  * @brief  停止直线行驶并清除锁定
  */
void Omni_StraightStop(Omni_t *omni)
{
    omni->yaw_locked = 0;
    Omni_Stop(omni);
}

/**
  * @brief  停止平移并清除锁定
  */
void Omni_LateralStop(Omni_t *omni)
{
    omni->lateral_yaw_locked = 0;
    Omni_Stop(omni);
}

/* ==================== 定点转向（陀螺仪 yaw 闭环） ==================== */

/**
  * @brief  初始化定点转向角度环 PID
  * @note   i_max=2.0 限制积分项, out_max=OMNI_TURN_MAX_OMEGA 限制最大角速度
  */
void Omni_TurnInit(Omni_t *omni, float kp, float ki, float kd)
{
    /* 组合行驶首次运行会锁定当时的航向。若这里不清锁，完成掉头后恢复
     * Omni_DriveCombined() 时会继续追踪掉头前的target_yaw，产生约180°
     * 误差并把车再次转回去。所有定点转向都经过本函数，因此在这里统一
     * 清除直行/横移航向锁和PID历史。下一次组合行驶会按转向后的当前航向
     * 重新锁定，不依赖g_drive_*_enable临时置0。 */
    omni->yaw_locked = 0U;
    omni->lateral_yaw_locked = 0U;
    omni->target_yaw = 0.0f;
    omni->lateral_target_yaw = 0.0f;
    _pid_clear_runtime(&omni->yaw_pid);
    _pid_clear_runtime(&omni->lateral_yaw_pid);

    pid_init(&omni->turn.yaw_pid, kp, ki, kd, 2.0f, OMNI_TURN_MAX_OMEGA);
    omni->turn.target_yaw = 0.0f;
    omni->turn.delta      = 0.0f;
    omni->turn.start_yaw  = 0.0f;
    omni->turn.rotated    = 0.0f;
    omni->turn.last_yaw   = 0.0f;
    omni->turn.state      = OMNI_TURN_IDLE;
    omni->turn.settle_cnt = 0U;
    omni->turn.start_tick = 0U;
    omni->turn.done_flag  = 0U;  /* 单次转向完成标志: 上电初始化清零 */
}

/**
  * @brief  启动一次相对转向(内部函数, 由 Omni_TurnBy 调用)
  * @note   重新初始化角度环, 清除上次转向残留的积分/微分项, 防止输出跳变;
  *         相对转角先归一化到最短路径(±180°), 目标航向 = 当前航向 + delta;
  *         delta 记录在状态机里供越限保护判断方向
  */
static void Omni_TurnStart(Omni_t *omni, float current_yaw, float delta_deg)
{
    /* 用原参数重初始化 PID: 状态清零, 保留调好的 kp/ki/kd */
    pid_init(&omni->turn.yaw_pid,
             omni->turn.yaw_pid.kp, omni->turn.yaw_pid.ki, omni->turn.yaw_pid.kd,
             omni->turn.yaw_pid.i_max, omni->turn.yaw_pid.out_max);

    /* 相对转角归一化到最短路径(±180°): 如 270° → -90° */
    omni->turn.delta      = _wrap_deg180(delta_deg);
    omni->turn.target_yaw = _wrap_deg180(current_yaw + omni->turn.delta);
    omni->turn.rotated    = 0.0f;       /* 累计转角清零(越限保护用) */
    omni->turn.last_yaw   = current_yaw;
    omni->turn.settle_cnt = 0U;
    omni->turn.start_tick = HAL_GetTick();
    omni->turn.state      = OMNI_TURN_RUNNING;
}

/**
  * @brief  转向闭环轮询(内部函数, 由 Omni_TurnBy 每控制周期调用)
  * @note   状态机: RUNNING → 角度环输出 omega → Omni_SetVelocity(0,0,omega);
  *         误差连续达标 OMNI_TURN_SETTLE_CYCLES 周期 → DONE + 停机;
  *         超时 OMNI_TURN_TIMEOUT_MS → TIMEOUT + 停机;
  *         累计转角超过 |delta|+OMNI_TURN_OVERSHOOT_DEG(转过了)或
  *         反向旋转超过阈值 → OVERSHOOT + 保护锁存停机(立即执行)
  */
static void Omni_TurnPoll(Omni_t *omni, float current_yaw)
{
    if (omni->turn.state != OMNI_TURN_RUNNING)
        return;

    /* 超时保护(堵转/传感器异常时保证停机) */
    if ((HAL_GetTick() - omni->turn.start_tick) > OMNI_TURN_TIMEOUT_MS) {
        omni->turn.state = OMNI_TURN_TIMEOUT;
        Omni_BrakeStop(omni);
        return;
    }

    /* 最短路径角度误差（处理 ±180° 环绕, 掉头 180° 两侧皆可） */
    float yaw_err = _wrap_deg180(omni->turn.target_yaw - current_yaw);

    /* 累计转过的角度(解绕)。不用误差符号判越限: 误差在 ±180° 边界处
     * 会翻转符号, 掉头 180° 时两侧都是最短路径, 用误差判会误报。
     * 单周期增量超过 OMNI_TURN_MAX_DYAW_DEG(absolute IMU sample jump guard; sensor rate is independent)
     * 视为传感器跳变(上电首帧/断线重连/磁干扰), 只更新参考不累计,
     * 防止跳变被当成"转过了"误触发保护 */
    float d = _wrap_deg180(current_yaw - omni->turn.last_yaw);
    omni->turn.last_yaw = current_yaw;
    if (fabsf(d) <= OMNI_TURN_MAX_DYAW_DEG)
        omni->turn.rotated += d;

    /* 越限保护(转过了触发保护):
     *   掉头(|delta|≈180°): 两侧皆可, 累计转角超过 |delta|+阈值 → 转过了
     *   其他角度: 沿指令方向的有符号累计转角 r = rotated × dir,
     *     r < -阈值        → 反向旋转(电机/IMU 方向异常)
     *     r > |delta|+阈值 → 转过了(越过目标超过阈值)
     * 触发后锁存停机(Omni_SafetyTrip), 重新上电后 Omni_TurnBy 重新读取当前航向角 */
    {
        float margin = OMNI_TURN_OVERSHOOT_DEG;
        float limit  = fabsf(omni->turn.delta) + margin;
        float dir    = (omni->turn.delta >= 0.0f) ? 1.0f : -1.0f;
        float r      = omni->turn.rotated * dir;
        int   trip   = (fabsf(omni->turn.delta) > 179.0f)
                         ? (fabsf(omni->turn.rotated) > limit)
                         : ((r < -margin) || (r > limit));

        if (trip) {
            omni->turn.state = OMNI_TURN_OVERSHOOT;
            Omni_SafetyTrip(omni);
            return;
        }
    }

    /* 到位判定: 误差连续达标 N 个周期才算完成, 滤除单周期抖动 */
    if (fabsf(yaw_err) <= OMNI_TURN_TOLERANCE_DEG) {
        /* 到位窗口内: 停止输出并滑行, 防止 P 增益 + 最小占空比死区
         * 造成的"到位后左右抖动"极限环; 滑行漂出窗口则重新纠偏 */
        Omni_BrakeStop(omni);
        if (++omni->turn.settle_cnt >= OMNI_TURN_SETTLE_CYCLES) {
            omni->turn.state = OMNI_TURN_DONE;
            /* main.c会在本周期看到DONE并停止再次调用Omni_TurnBy，因此完成
             * 标志必须在产生DONE的同一周期置位，不能等下一周期补写。 */
            omni->turn.done_flag = 1U;
        }
        return;
    }
    omni->turn.settle_cnt = 0U;

    /* 角度环: yaw误差 → omega角速度, 原地旋转(纯角速度) */
    float omega = pid_calc_by_error(&omni->turn.yaw_pid, yaw_err);
    Omni_SetVelocity(omni, 0.0f, 0.0f, omega);
}

void Omni_TurnStop(Omni_t *omni)
{
    omni->turn.state = OMNI_TURN_IDLE;
    Omni_Stop(omni);
}

Omni_TurnState_t Omni_TurnGetState(const Omni_t *omni)
{
    return (Omni_TurnState_t)omni->turn.state;
}

uint8_t Omni_TurnIsDone(const Omni_t *omni)
{
    return omni->turn.state == OMNI_TURN_DONE;
}

/* ---- 单次任意角度转向: 每控制周期调用这一个函数即可 ---- */

/* main.c 提供的航向角读取/新鲜度回调(封装 JY901P, 见 main.c) */
extern float Imu_GetYaw(void);
extern uint8_t Imu_YawFresh(void);

/* 航向角读取回调(在 Omni_TurnBy 内注册, 绑定 main.c 的 Imu_GetYaw) */
static float (*s_get_yaw)(void) = NULL;
/* 航向角数据新鲜度回调(500ms 内收到角度帧视为有效) */
static uint8_t (*s_yaw_fresh)(void) = NULL;

/**
  * @brief  注册航向角读取/新鲜度回调(内部函数, 在 Omni_TurnBy 内调用)
  * @param  get_yaw    回调: 返回当前航向角(°, ±180)
  * @param  yaw_fresh  回调: 返回航向角数据是否新鲜(1=新鲜, 0=超时), 可传 NULL
  */
static void Omni_AttachYawSource(float (*get_yaw)(void), uint8_t (*yaw_fresh)(void))
{
    s_get_yaw   = get_yaw;
    s_yaw_fresh = yaw_fresh;
}

/**
  * @brief  单次任意角度转向(合并启动/闭环/完成校验, 每控制周期调用)
  * @param  omni       底盘实例
  * @param  delta_deg  相对转角(°, 正=逆时针, 负=顺时针, 任意值, 自动取最短路径)
  * @note   建议放在 TIM7 10ms 回调中每周期调用一次, 中断回调只需调用本函数:
  *         - 首次进入: 函数内注册航向角回调 Omni_AttachYawSource(Imu_GetYaw, ...)
  *         - 标志位判断(if): done_flag 上电时在 Omni 初始化(Omni_TurnInit)里清零,
  *           置位后本函数不再执行转向
  *         - 标志位通过: 读取一次当前航向角并记录(start_yaw), 启动转向;
  *           等待 IMU 数据新鲜后才启动, 防止读到上电默认值 0
  *         - RUNNING: 每周期执行角度环闭环, 转过了立即触发保护停机并锁存
  *         - DONE: 连续到位的同一周期锁存done_flag，不按原始delta重新启动
  *         保护锁存后需重新上电(重新测量当前航向角)才能再次转向
  */
void Omni_TurnBy(Omni_t *omni, float delta_deg)
{
    if (omni == NULL)
        return;

    /* 航向角回调注册放在函数内: 首次调用时绑定 main.c 的 Imu_GetYaw */
    if (s_get_yaw == NULL)
        Omni_AttachYawSource(Imu_GetYaw, Imu_YawFresh);

    /* 标志位判断: 转向完成校验通过后 done_flag=1, 不再执行转向 */
    if (omni->turn.done_flag)
        return;

    /* 保护已锁存(转过了触发): 保持停机, 重新上电前不再动作 */
    if (s_safety_tripped)
        return;

    switch (omni->turn.state)
    {
    case OMNI_TURN_IDLE:
        /* 等待航向角数据新鲜(上电首帧未到时不启动, 防止读到默认值 0) */
        if ((s_yaw_fresh != NULL) && !s_yaw_fresh())
            return;
        /* 标志位通过: 读取一次当前航向角并记录, 启动转向 */
        omni->turn.start_yaw = s_get_yaw();
        Omni_TurnStart(omni, omni->turn.start_yaw, delta_deg);
        break;

    case OMNI_TURN_RUNNING:
        /* 角度环闭环(内部含超时/越限保护: 转过了立即执行保护停机并锁存) */
        Omni_TurnPoll(omni, s_get_yaw());
        break;

    case OMNI_TURN_DONE:
        /* DONE在Omni_TurnPoll到位的同一周期已经锁存。禁止退回IDLE后再按
         * 原始delta重新武装；否则航向抖动时可能从新当前位置再转整整180°。 */
        omni->turn.done_flag = 1U;
        Omni_BrakeStop(omni);
        break;

    default:
        /* TIMEOUT / OVERSHOOT: 保持停机不再动作 */
        break;
    }
}

/* ==================== 平移（陀螺仪 yaw 闭环保持航向） ==================== */

/**
  * @brief  横移控制: 与 Omni_DriveStraight 共用 yaw 角度环,
  *         首次调用锁定当前航向, 之后边横移边修正偏航
  */
void Omni_DriveLateral(Omni_t *omni, float vy, float current_yaw)
{
    /* 平移使能标志判断: 未收到"可以行驶"(标志为 0)时不动作 */
    if (!g_drive_lateral_enable) {
        Omni_Stop(omni);
        omni->lateral_yaw_locked = 0; /* 解除锁定, 重新使能后按当前航向重新锁定 */
        return;
    }

    /* 首次锁定目标航向(独立于直线行驶的锁定) */
    if (!omni->lateral_yaw_locked) {
        omni->lateral_target_yaw = current_yaw;
        omni->lateral_yaw_locked = 1;
    }

    /* 计算最短路径角度误差（处理 ±180° 环绕） */
    float yaw_err = _wrap_deg180(omni->lateral_target_yaw - current_yaw);

    /* 平移角度环: yaw误差 → omega角速度修正 */
    float omega = pid_calc_by_error(&omni->lateral_yaw_pid, yaw_err);

    /* 调用全向运动学: vy横移 + omega旋转修正 → 内部速度环自动执行 */
    Omni_SetVelocity(omni, 0.0f, vy, omega);
}

/**
  * @brief  斜线行驶控制(直线+平移合成): 与直线行驶共用 yaw 角度环,
  *         首次调用锁定当前航向, vx+vy 一次合成进运动学
  */
void Omni_DriveCombined(Omni_t *omni, float vx, float vy, float current_yaw)
{
    /* 两个使能标志任一为 0 时不动作 */
    if (!g_drive_straight_enable || !g_drive_lateral_enable) {
        Omni_Stop(omni);
        omni->yaw_locked = 0; /* 解除锁定, 重新使能后按当前航向重新锁定 */
        return;
    }

    /* 首次锁定目标航向(与直线行驶共用锁定) */
    if (!omni->yaw_locked) {
        omni->target_yaw = current_yaw;
        omni->yaw_locked = 1;
    }

    /* 计算最短路径角度误差（处理 ±180° 环绕） */
    float yaw_err = _wrap_deg180(omni->target_yaw - current_yaw);

    /* 角度环: yaw误差 → omega角速度修正 */
    float omega = pid_calc_by_error(&omni->yaw_pid, yaw_err);

    /* vx+vy 合成: 运动学一次解算, 三路速度环每周期只执行一次 */
    Omni_SetVelocity(omni, vx, vy, omega);
}

/**
  * @brief  斜线行驶并持续跟踪指定目标航向
  * @note   指定绝对航向时不能使用 Omni_DriveCombined 的“首次调用锁存当前
  *         yaw”行为，否则转向后产生的残余角度会被永久当成正确航向。
  */
void Omni_DriveCombinedToYaw(Omni_t *omni, float vx, float vy,
                             float current_yaw, float target_yaw)
{
    if (omni == NULL)
        return;

    /* 两个使能标志任一为 0 时不动作。 */
    if (!g_drive_straight_enable || !g_drive_lateral_enable) {
        Omni_Stop(omni);
        omni->yaw_locked = 0U;
        return;
    }

    /* 目标航向由上层明确给出，不能改写成当前航向。 */
    omni->target_yaw = _wrap_deg180(target_yaw);
    omni->yaw_locked = 1U;

    /* 角度环持续把车身拉回目标航向，处理±180°环绕。 */
    float yaw_err = _wrap_deg180(omni->target_yaw - current_yaw);
    float omega = pid_calc_by_error(&omni->yaw_pid, yaw_err);

    /* vx+vy 合成: 运动学一次解算, 三路速度环每周期只执行一次。 */
    Omni_SetVelocity(omni, vx, vy, omega);
}
