#include "pid.h"
#include "main.h"

#include <math.h>

/**
 * @brief  初始化PID控制器参数
 * @param  pid      指向PID结构体的指针
 * @param  kp       比例系数
 * @param  ki       积分系数
 * @param  kd       微分系数
 * @param  i_max    积分限幅上限
 * @param  out_max  输出限幅上限
 * @note   会对所有参数做合法性检查，负数将触发 Error_Handler()。
 *         调用后所有历史误差、积分累加量和输出均清零。
 */
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
    /* --- 参数合法性检查：所有PID参数不允许为负数 --- */
    if (!(kp >= 0.0f) || !(ki >= 0.0f) || !(kd >= 0.0f) || !(i_max >= 0.0f)
        || !(out_max >= 0.0f))
    {
        Error_Handler();
    }

    /* --- 写入用户配置参数 --- */
    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->i_max   = i_max;
    pid->out_max = out_max;

    /* --- 清零内部状态变量 --- */
    pid->i_out   = 0.0f;   // 积分累加量
    pid->err[0]  = 0.0f;   // 上一次误差
    pid->err[1]  = 0.0f;   // 当前误差
    pid->output  = 0.0f;   // PID输出
    pid->p_out   = 0.0f;
    pid->d_out   = 0.0f;
}

/**
 * @brief  直接传入误差的PID计算（核心函数）
 * @param  pid   指向PID结构体的指针
 * @param  error 已处理好的误差值
 * @return float  限幅后的PID输出值
 * @note   位置式PID核心算法，供 pid_calc 和外部直接调用：
 *         - 比例项：Kp * error
 *         - 积分项：Ki * error 的累加，带积分限幅防饱和
 *         - 微分项：Kd * (error - err_prev)
 *         - 输出限幅：防止输出超出执行机构允许范围
 */
float pid_calc_by_error(pid_struct_t *pid, float error)
{
    /* 非有限误差(编码器/指令异常)会永久污染积分和历史误差：清零状态并输出 0。 */
    if (!isfinite(error))
    {
        pid->i_out  = 0.0f;
        pid->err[0] = 0.0f;
        pid->err[1] = 0.0f;
        pid->p_out  = 0.0f;
        pid->d_out  = 0.0f;
        pid->output = 0.0f;
        return 0.0f;
    }

    /* --- 写入当前误差 --- */
    pid->err[1] = error;

    /* ==================== 比例项 ==================== */
    pid->p_out = pid->kp * pid->err[1];

    /* ==================== 积分项 ==================== */
    pid->i_out += pid->ki * pid->err[1];

    /* 积分限幅 */
    if (pid->i_out > pid->i_max)
    {
        pid->i_out = pid->i_max;
    }
    else if (pid->i_out < -pid->i_max)
    {
        pid->i_out = -pid->i_max;
    }

    /* ==================== 微分项 ==================== */
    float derivative = (pid->err[1] - pid->err[0]);
    pid->d_out = pid->kd * derivative;

    /* ==================== 输出合成 ==================== */
    pid->output = pid->p_out + pid->i_out + pid->d_out;

    /* 输出限幅 */
    if (pid->output > pid->out_max)
    {
        pid->output = pid->out_max;
    }
    else if (pid->output < -pid->out_max)
    {
        pid->output = -pid->out_max;
    }

    /* --- 更新历史误差 --- */
    pid->err[0] = pid->err[1];

    return pid->output;
}

/**
 * @brief  PID计算函数（位置式PID）
 * @param  pid  指向PID结构体的指针
 * @param  get  当前反馈值
 * @param  set  目标设定值
 * @return float  限幅后的PID输出值
 * @note   内部调用 pid_calc_by_error(error=set-get)
 */
float pid_calc(pid_struct_t *pid, float get, float set)
{
    /* --- 保存当前反馈值和目标值（便于外部调试查看） --- */
    pid->get = get;
    pid->set = set;

    return pid_calc_by_error(pid, set - get);
}

/* ==================== 增量式PID ==================== */

/**
 * @brief  初始化增量式PID控制器参数
 * @param  pid      指向增量式PID结构体的指针
 * @param  kp       比例系数
 * @param  ki       积分系数
 * @param  kd       微分系数
 * @param  out_max  输出限幅上限（PWM最终限幅）
 * @param  inc_max  单次增量限幅上限（抑制突变跳变）
 * @note   所有误差缓存和输出均清零。
 *         增量式PID公式:
 *           Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)]
 *           output += Δu(k)
 */
void inc_pid_init(inc_pid_t *pid,
                  float kp,
                  float ki,
                  float kd,
                  float out_max,
                  float inc_max)
{
    /* 与 pid_init 一致：负数/非有限参数视为配置错误。 */
    if (!(kp >= 0.0f) || !(ki >= 0.0f) || !(kd >= 0.0f) || !(out_max >= 0.0f)
        || !(inc_max >= 0.0f) || !isfinite(kp) || !isfinite(ki) || !isfinite(kd)
        || !isfinite(out_max) || !isfinite(inc_max))
    {
        Error_Handler();
    }

    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->out_max = out_max;
    pid->inc_max = inc_max;

    pid->err[0]  = 0.0f;   // e(k-2)
    pid->err[1]  = 0.0f;   // e(k-1)
    pid->err[2]  = 0.0f;   // e(k)
    pid->output  = 0.0f;
}

/**
 * @brief  增量式PID计算函数
 * @param  pid  指向增量式PID结构体的指针
 * @param  get  当前反馈值（带方向的实际速度 RPM）
 * @param  set  目标设定值（带方向的目标速度 RPM）
 * @return float  限幅后的累加输出值（可用于PWM占空比设定）
 * @note   算法流程：
 *         1. 计算当前误差 Err = set - get
 *         2. 增量公式: add = Kp*(Err-err[1]) + Ki*Err + Kd*(Err-2*err[1]+err[0])
 *         3. 单次增量限幅（抑制突变跳变）
 *         4. 累加到输出 output += add
 *         5. 输出限幅（确保在执行机构允许范围内）
 *         6. 滚动误差缓存
 */
float inc_pid_calc(inc_pid_t *pid, float get, float set)
{
    /* --- 计算当前误差 --- */
    float Err = set - get;

    /* 增量式输出是累加量，NaN/Inf 一旦进入将永久锁死输出：清零并停转。 */
    if (!isfinite(Err))
    {
        pid->err[0] = 0.0f;
        pid->err[1] = 0.0f;
        pid->err[2] = 0.0f;
        pid->output = 0.0f;
        return 0.0f;
    }

    /* ==================== 增量式PID核心公式 ====================
     * Δu = Kp * [e(k) - e(k-1)]
     *    + Ki * e(k)
     *    + Kd * [e(k) - 2*e(k-1) + e(k-2)]
     *
     * 其中: e(k)=Err, e(k-1)=err[1], e(k-2)=err[0]
     */
    float add = pid->kp * (Err - pid->err[1])
              + pid->ki * Err
              + pid->kd * (Err - 2.0f * pid->err[1] + pid->err[0]);

    /* --- 单次增量限幅，抑制突变跳变 --- */
    if (add > pid->inc_max)  add = pid->inc_max;
    if (add < -pid->inc_max) add = -pid->inc_max;

    /* --- 累加增量到输出 --- */
    pid->output += add;

    /* --- 最终输出限幅 --- */
    if (pid->output > pid->out_max)  pid->output = pid->out_max;
    if (pid->output < -pid->out_max) pid->output = -pid->out_max;

    /* --- 滚动更新误差缓存 --- */
    pid->err[0] = pid->err[1];   // e(k-2) ← e(k-1)
    pid->err[1] = Err;           // e(k-1) ← e(k)

    return pid->output;
}
