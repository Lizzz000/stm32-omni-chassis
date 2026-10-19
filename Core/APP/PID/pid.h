#ifndef __PID_H
#define __PID_H

/**
 * @brief PID控制器结构体
 *
 * @note 该结构体封装了位置式PID控制器的所有状态变量。
 *       使用时先调用 pid_init() 初始化参数，
 *       然后在控制循环中周期性调用 pid_calc() 计算输出。
 */
typedef struct
{
    /* --- PID 参数（用户配置）--- */
    float kp;        /**< 比例系数 */
    float ki;        /**< 积分系数 */
    float kd;        /**< 微分系数 */
    float i_max;     /**< 积分限幅最大值（对称限幅，即 [-i_max, +i_max]） */
    float out_max;   /**< 输出限幅最大值（对称限幅，即 [-out_max, +out_max]） */

    /* --- 内部状态（运行时更新，无需用户干预）--- */
    float get;       /**< 当前反馈值 */
    float set;       /**< 当前目标值 */
    float err[2];    /**< 误差缓存: err[0] 上一次误差, err[1] 当前误差 */
    float p_out;     /**< 比例项输出 */
    float i_out;     /**< 积分项输出（带累加记忆） */
    float d_out;     /**< 微分项输出 */
    float output;    /**< PID 最终输出（含限幅） */
} pid_struct_t;

/**
 * @brief  初始化PID控制器参数
 * @param  pid      指向PID结构体的指针
 * @param  kp       比例系数（必须 >= 0）
 * @param  ki       积分系数（必须 >= 0）
 * @param  kd       微分系数（必须 >= 0）
 * @param  i_max    积分限幅上限（必须 >= 0）
 * @param  out_max  输出限幅上限（必须 >= 0）
 * @note   若任一参数为负值，将调用 Error_Handler() 进入错误处理。
 *         初始化后所有内部状态清零。
 */
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);

/**
 * @brief  PID计算函数（位置式PID）
 * @param  pid  指向PID结构体的指针
 * @param  get  当前反馈值（传感器实测值）
 * @param  set  目标设定值
 * @return float  PID计算后的输出值（已限幅到 [-out_max, +out_max]）
 * @note   本函数应在定时控制循环中周期性调用。
 *         调用频率应保持一致，以保证积分和微分的计算精度。
 *
 *         位置式PID公式:
 *           P_out = Kp * err
 *           I_out += Ki * err   （累加，并进行积分限幅）
 *           D_out = Kd * (err - err_prev)  （微分先行：误差的微分）
 *           Output = P_out + I_out + D_out  （输出限幅）
 */
float pid_calc(pid_struct_t *pid, float get, float set);

/**
 * @brief  直接传入误差的PID计算（用于需预处理误差的场景，如角度环绕）
 * @param  pid   指向PID结构体的指针
 * @param  error 已处理好的误差值（set - get 的等效值）
 * @return float  PID计算后的输出值（已限幅到 [-out_max, +out_max]）
 * @note   与 pid_calc 共享同一结构体和内部状态，可混用。
 *         典型场景：航向角控制中先将 ±180° 包裹为最短路径误差再传入
 */
float pid_calc_by_error(pid_struct_t *pid, float error);

/* ==================== 增量式PID ==================== */

/**
 * @brief 增量式PID控制器结构体
 *
 * @note  增量式PID输出的是控制量的变化量 Δu，
 *        累加到上一次输出上，天然抗积分饱和。
 *        公式: Δu = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)]
 */
typedef struct
{
    float kp;        /**< 比例系数 */
    float ki;        /**< 积分系数 */
    float kd;        /**< 微分系数 */
    float err[3];    /**< 误差缓存: err[0]=e(k-2), err[1]=e(k-1), err[2]=e(k) */
    float output;    /**< 累加输出值（如PWM占空比） */
    float out_max;   /**< 输出限幅最大值（对称限幅） */
    float inc_max;   /**< 单次增量限幅最大值（防止突变跳变） */
} inc_pid_t;

void inc_pid_init(inc_pid_t *pid,
                  float kp,
                  float ki,
                  float kd,
                  float out_max,
                  float inc_max);

float inc_pid_calc(inc_pid_t *pid, float get, float set);

#endif /* __PID_H */
