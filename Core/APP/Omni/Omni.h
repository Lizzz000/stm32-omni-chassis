/**
  ******************************************************************************
  * @file    Omni.h
  * @brief   三轮全向轮(福来轮)底盘运动学库头文件
  ******************************************************************************
  * 轮系布局(俯视图, 车头为 +x 方向, +y 指向车身左侧):
  *
  *              +x (车头)
  *               ↑
  *         C ●       ● B      C: 左前轮, 位置角 φ=+60°
  *           \       /        B: 右前轮, 位置角 φ=-60°
  *            \     /         A: 后轮,   位置角 φ=180°
  *             \   /
  *              ● A
  *
  * 三轮绕底盘中心均匀分布(间隔 120°), 标准全向轮:
  * 轮子滚动方向为位置矢量的切线方向 t = (-sinφ, cosφ),
  * 滚子轴向与轮轴平行, 滚子方向自由滑动。
  *
  * 逆运动学(底盘速度 → 轮线速度, 单位 m/s):
  *   v_i = -vx·sinφ_i + vy·cosφ_i + ω·L
  *   vx: 前进速度(+), vy: 左侧速度(+), ω: 旋转角速度(rad/s, 逆时针+)
  *   L:  底盘中心到轮心距离(OMNI_WHEEL_BASE)
  *
  * 展开(φ_A=180°, φ_B=-60°, φ_C=60°; 目标正 = 轮子逆时针 = 滚向 -t_i;
  *       左移(vy>0): A 逆时针, B/C 顺时针, A 速度为 B/C 的 2 倍):
  *   v_A = +vy - ω·L
  *   v_B = -0.866·vx - 0.5·vy - ω·L
  *   v_C =  0.866·vx - 0.5·vy - ω·L
  *
  * 正运动学(实测轮速 → 底盘速度, 用于里程计/反馈;
  *         v_i 为滚向 +t_i 的轮速, 编码器原始值直接使用):
  *   vx = (v_B - v_C) / √3
  *   vy = (v_B + v_C - 2·v_A) / 3
  *   ω  = (v_A + v_B + v_C) / (3·L)
  *
  * 控制流程(当前为三轮速度闭环, 每控制周期调用):
  *   Omni_SetVelocity(vx, vy, ω) → 逆运动学 → 目标 RPM(限幅)
  *         ↓
  *   编码器RPM × dir → 速度环 PID → speed_output
  *   PWM = -(目标RPM × 对应轮 FF_GAIN + speed_output) → 限幅(±ARR) → Motor_X_SetPWM
  *
  * 使用示例(MX_* 外设初始化、Encoder_Init、Motor_Init 之后):
  *   Omni_t omni;
  *   Omni_Init(&omni, &encoder1, &encoder2, &encoder3);  // A 后轮, B 右前, C 左前
  *   Omni_SetVelocity(&omni, 0.5f, 0.0f, 0.0f);          // 前进 0.5 m/s
  *   Omni_SetVelocity(&omni, 0.0f, 0.0f, 1.0f);          // 逆时针自转 1 rad/s
  *   float vx, vy, w;
  *   Omni_GetVelocity(&omni, &vx, &vy, &w);              // 正运动学(里程计)
  *   Omni_Stop(&omni);                                   // 全停
  *
  * 上电必做标定(本板实测: 正 PWM=顺时针、编码器顺时针=正计数):
  *   1. OMNI_*_DIR: 单轮让轮子逆时针转, 看该轮编码器计数符号,
  *      逆时针→计数为正则 dir=+1, 为负则 dir=-1(本板均为 -1)。
  *      注: 正 PWM 的转向与运动学正方向相反, 输出已在 CalcCtrlInput 整体取反。
  *   2. 整机验证(编码器: 顺时针=正计数): 前进 B 顺时针/C 逆时针/A 停;
  *      左移 A 逆时针/B,C 顺时针(A 转速为 B/C 的 2 倍); 自转三轮同向(均顺时针)。
  *   3. 前馈增益: 开环给固定 PWM(如 500), 用 Omni_GetWheelRPM
  *      测转速(如 250RPM), 取 CCR/RPM = 500/250 = 2.0 填入。
  ******************************************************************************
  */
#ifndef __OMNI_H
#define __OMNI_H

#include "stm32f4xx_hal.h"
#include "encoder.h"
#include "control_timing.h"
#include "pid.h"

/* 机械参数 */
#define OMNI_WHEEL_RADIUS  0.035f /* 轮半径 R (m) = 35mm */
#define OMNI_WHEEL_BASE    0.17f  /* 底盘中心到轮心距离 L (m) = 170mm */

/* 闭环速度环里 dir 乘在反馈端, 把编码器符号换算到运动学符号(逆时针为正)。
 * 标定方法: 单轮让轮子逆时针转, 串口看该轮编码器计数:
 *   逆时针 → 计数为正 → dir=+1; 逆时针 → 计数为负 → dir=-1。
 * 标错会导致速度环正反馈发散(该轮全速疯转)。
 * 本板实测: 编码器顺时针=正计数(即逆时针=负), 故 dir 均为 -1。
 * 另: 本板正 PWM=顺时针, 与运动学正方向(逆时针)相反, 输出已在
 *     Omni_CalcCtrlInput 里整体取反(见 Omni.c)。 */
#define OMNI_A_DIR  -1
#define OMNI_B_DIR  -1
#define OMNI_C_DIR  -1

/* 目标转速上限(RPM), 防电机超速， 电机额度转速在260RPM左右
   对应底盘最大线速度 ≈ OMNI_MAX_RPM × 2πR / 60 (m/s) */
#define OMNI_MAX_RPM  250.0f

/* 开环转速→PWM 标定系数: 每 1 RPM 对应的 PWM 比较值,  空载时测量，需负载校准 */
/* 前馈增益: 闭环 ctr_input = -(target_rpm × 对应轮 FF_GAIN + speed_output); OMNI_FF_GAIN 供开环辅助 API 使用 */
#define OMNI_FF_GAIN  7.14f //空载值，负载需重新测定
#define OMNI_A_FF_GAIN 6.8f  /* A轮实测略强 */
#define OMNI_B_FF_GAIN 9.2f  /* B轮负载下偏弱 */
#define OMNI_C_FF_GAIN 9.0f  /* C轮负载下偏弱 */
#define OMNI_DRIVE_MAX_OMEGA 0.8f /* 行驶锁航向修正上限(rad/s) */

typedef struct
{
    Encoder_t *encoder;   /* 轮速反馈编码器 */
    int8_t dir;           /* 编码器反馈方向修正 ±1; 当前速度闭环只用于反馈端 */
    float target_rpm;     /* 运动学解算的目标转速(RPM, 运动学正方向) */
    float speed_output;  /* 速度环 PID 累计修正量(PWM), 停车时清零 */
    float ctr_input;     /* 最终运动 PWM 控制量(含前馈/方向/限幅), 停车时清零 */
} Omni_Wheel_t;

/* ---- 定点转向（陀螺仪 yaw 闭环: 任意角度） ---- */

/* 转向控制参数 */
#define OMNI_TURN_TOLERANCE_DEG  3.0f   /* 转向到位角度误差阈值(°) */
#define OMNI_TURN_SETTLE_CYCLES  (200U / CHASSIS_CONTROL_PERIOD_MS) /* 200ms */
#define OMNI_TURN_TIMEOUT_MS     8000U  /* 转向超时保护: 超过该时间未到位则停机(ms) */
#define OMNI_TURN_MAX_OMEGA      2.0f   /* 转向角速度限幅(rad/s), 降低惯性过冲 */
#define OMNI_TURN_OVERSHOOT_DEG  20.0f  /* 转向越限保护阈值(°): 累计转角超过 |delta|+该值
                                           (转过了)或反向旋转超过该值 → 触发保护停机 */
#define OMNI_TURN_MAX_DYAW_DEG   20.0f  /* Absolute IMU sample jump guard.
 * IMU output rate is independent of the 100Hz controller: do not scale
 * this limit by control dt without measuring the IMU frame interval. */
/* 转向状态 */
typedef enum
{
    OMNI_TURN_IDLE = 0,       /* 空闲 */
    OMNI_TURN_RUNNING,        /* 转向中 */
    OMNI_TURN_DONE,           /* 已到位(自动停机) */
    OMNI_TURN_TIMEOUT,        /* 超时(自动停机) */
    OMNI_TURN_OVERSHOOT       /* 转过了: 越限保护已触发(锁存停机，清除保护或复位后恢复) */
} Omni_TurnState_t;

typedef struct
{
    pid_struct_t yaw_pid;     /* 转向角度环 PID */
    float  target_yaw;        /* 目标航向 (°, 已归一化到 ±180°) */
    float  delta;             /* 本次转向角 (°, 带符号, 已归一化到 ±180°), 越限保护用 */
    float  start_yaw;         /* 启动时记录的当前航向角 (°), 完成校验用 */
    float  rotated;           /* 累计转过的角度 (°, 解绕), 越限保护用 */
    float  last_yaw;          /* 上一周期航向角 (°), 累计转角用 */
    uint8_t state;            /* 状态, 见 Omni_TurnState_t */
    uint8_t settle_cnt;       /* 误差连续达标周期计数 */
    uint8_t done_flag;        /* 单次转向完成标志: 上电初始化(Omni_TurnInit)清零,
                                 完成校验通过后置 1, 置 1 后 Omni_TurnBy 不再执行转向 */
    uint32_t start_tick;      /* 转向起始时刻 (HAL_GetTick) */
} Omni_Turn_t;

typedef struct
{
    Omni_Wheel_t wheel_a; /* 后轮 */
    Omni_Wheel_t wheel_b; /* 右前轮 */
    Omni_Wheel_t wheel_c; /* 左前轮 */
    int16_t pwm_max;      /* PWM 输出上限(Init 时由 Motor_GetMaxPWM 读取) */

    /* 直线行驶 — yaw 角度环 */
    pid_struct_t yaw_pid;     /* 直线行驶航向角 PID */
    float  target_yaw;        /* 锁定后的目标航向 (°) */
    uint8_t yaw_locked;       /* 首次锁定标志 */

    /* 平移 — yaw 角度环(与直线行驶分开, 不共用) */
    pid_struct_t lateral_yaw_pid;   /* 平移航向角 PID */
    float  lateral_target_yaw;      /* 平移锁定后的目标航向 (°) */
    uint8_t lateral_yaw_locked;     /* 平移首次锁定标志 */

    /* 定点转向 — yaw 角度环状态机 */
    Omni_Turn_t turn;         /* 转向状态机(任意角度, Omni_TurnBy 启动) */
} Omni_t;

/**
  * @brief  初始化全向底盘
  * @param  omni  底盘实例
  * @param  enc_a A 轮编码器(后轮)
  * @param  enc_b B 轮编码器(右前轮)
  * @param  enc_c C 轮编码器(左前轮)
  * @note   必须在 Encoder_Init、Motor_Init 之后调用;
  *         输出钳位到定时器 ARR, 上电输出全零
  */
void Omni_Init(Omni_t *omni, Encoder_t *enc_a, Encoder_t *enc_b, Encoder_t *enc_c);

/**
  * @brief  设置底盘速度并执行三轮速度闭环, 输出 PWM
  * @param  omni   底盘实例
  * @param  vx     前进速度 (m/s, +向前)
  * @param  vy     横移速度 (m/s, +向左)
  * @param  omega  自转角速度 (rad/s, +逆时针)
  * @note   PWM = -(目标RPM × 对应轮 FF_GAIN + speed_output),
  *         编码器反馈乘 dir 后参与速度环计算, dir 不再乘到闭环输出端;
  *         目标 RPM 先经 OMNI_MAX_RPM 限幅, 输出再限幅到 ±ARR
  */
void Omni_SetVelocity(Omni_t *omni, float vx, float vy, float omega);

/**
  * @brief  开环调试: 直接输出指定 PWM(经 ±ARR 限幅)
  * @param  omni   底盘实例
  * @param  pwm_a  A 轮 PWM(±, 符号为方向)
  * @param  pwm_b  B 轮 PWM
  * @param  pwm_c  C 轮 PWM
  * @note   用于标定: 固定 PWM 测编码器转速, 确定 OMNI_FF_GAIN 与 OMNI_*_DIR
  */
void Omni_SetPWM(Omni_t *omni, int16_t pwm_a, int16_t pwm_b, int16_t pwm_c);

/** @brief 停止全部电机(清目标/PID/运动输出缓存, 输出全零并滑行) */
void Omni_Stop(Omni_t *omni);

/**
  * @brief  主动刹车停止: 清零目标/PID/运动输出缓存并让AT8236进入快速刹车
  * @note   只用于到位/故障后的短时制动；长时间停车应随后调用Omni_Stop转为滑行。
  */
void Omni_BrakeStop(Omni_t *omni);

/**
  * @brief  正运动学: 实测轮速 → 底盘速度(里程计/速度反馈)
  * @param  omni   底盘实例
  * @param  vx     输出: 前进速度 (m/s, +向前)
  * @param  vy     输出: 横移速度 (m/s, +向左)
  * @param  omega  输出: 自转角速度 (rad/s, +逆时针)
  * @note   输入为编码器原始实测值(滚向 +t_i 为正), 本函数不乘 dir;
  *         积分三轴速度即得位姿(x, y, θ)
  */
void Omni_GetVelocity(Omni_t *omni, float *vx, float *vy, float *omega);

/** @brief 读取单轮实际转速 (RPM, 编码器原始值, 未含方向修正) */
float Omni_GetWheelRPM(const Omni_Wheel_t *wheel);

/**
  * @brief  开环辅助: 单轮目标转速 → PWM 比较值(含方向修正)
  * @param  wheel  轮子实例指针
  * @return int16_t  PWM 比较值 (±, 符号决定转向)
  * @note   PWM = target_rpm × OMNI_FF_GAIN × dir
  *         dir = +1 时正 RPM → 正 PWM; dir = -1 时正 RPM → 负 PWM;
  *         这是开环辅助公式, 当前 Omni_SetVelocity 不调用此 API。
  */
int16_t Omni_WheelToPWM(const Omni_Wheel_t *wheel);

/**
  * @brief  闭环控制量计算: ctr_input = -(target_rpm × 对应轮 FF_GAIN + speed_output)
  * @param  omni  底盘实例
  * @note   前馈 + PID修正 = 最终 PWM 值; 本板正 PWM=顺时针,
  *         与运动学约定(逆时针为正)相反且三轮一致, 故输出整体取反
  *         计算结果存入各轮 ctr_input 字段，之后可直接用于 Motor_X_SetPWM
  */
void Omni_CalcCtrlInput(Omni_t *omni);

/* ---- 防疯转保护（正反馈检测 + 紧急停机）---- */

/**
  * @brief 查询防疯转保护是否已触发
  * @return 1=已触发(三路电机锁定停机), 0=正常
  * @note  触发条件: 任一车轮实测|RPM| > |目标|×2+50 且 PWM 饱和, 连续25个控制周期（250ms）
  */
uint8_t Omni_SafetyTripped(void);

/**
  * @brief 清除防疯转保护锁定, 恢复速度环输出
  * @note  触发后 Omni_SetVelocity 会保持停机, 必须调用本函数(或复位)才恢复;
  *         触发时三个增量PID的输出与误差缓存已清零, 清除后从零开始重建输出
  */
void Omni_SafetyClear(void);

/* ---- 直线行驶（串级PID: yaw角度外环 + 速度内环）---- */

/**
  * @brief  初始化直线行驶角度环 PID
  * @param  omni  底盘实例
  * @param  kp    角度环比例系数 (如 1.5)
  * @param  ki    角度环积分系数 (如 0.1)
  * @param  kd    角度环微分系数 (如 0.0)
  * @note   在 Omni_Init 之后调用；内部固定 i_max=2.0, out_max=3.0 rad/s
  */
void Omni_StraightInit(Omni_t *omni, float kp, float ki, float kd);

/**
  * @brief  初始化平移角度环 PID(与直线行驶分开, 不共用)
  * @param  omni  底盘实例
  * @param  kp    角度环比例系数 (如 1.5)
  * @param  ki    角度环积分系数 (如 0.1)
  * @param  kd    角度环微分系数 (如 0.0)
  * @note   在 Omni_Init 之后调用；内部固定 i_max=2.0, out_max=3.0 rad/s;
  *         使用独立的 lateral_yaw_pid/lateral_target_yaw/lateral_yaw_locked,
  *         与直线行驶的角度环互不影响
  */
void Omni_LateralInit(Omni_t *omni, float kp, float ki, float kd);

/**
  * @brief  直线行驶控制（每控制周期调用一次）
  * @param  omni        底盘实例
  * @param  vx          前进速度 (m/s, +向前)
  * @param  current_yaw 当前航向角 (°, 来自 IMU, ±180°)
  * @note   首次调用自动锁定当前 yaw 为 target_yaw；
  *         角度环输出 omega → Omni_SetVelocity(vx, 0, omega);
  *         直线行驶使能标志 g_drive_straight_enable(main.c 定义)为 0 时不动作,
  *         收到"可以行驶"写 1 后小车才会动(与平移标志分开, 不共用)
  */
void Omni_DriveStraight(Omni_t *omni, float vx, float current_yaw);

/**
  * @brief  停止直线行驶并清除锁定
  * @param  omni  底盘实例
  * @note   清除 yaw_locked 标志并调用 Omni_Stop 全停；
  *         再次调用 Omni_DriveStraight 会重新锁定当前航向
  */
void Omni_StraightStop(Omni_t *omni);

/**
  * @brief  停止平移并清除锁定
  * @param  omni  底盘实例
  * @note   清除 lateral_yaw_locked 标志并调用 Omni_Stop 全停；
  *         再次调用 Omni_DriveLateral 会重新锁定当前航向
  */
void Omni_LateralStop(Omni_t *omni);

/* ---- 定点转向（陀螺仪 yaw 闭环: 任意角度） ---- */

/**
  * @brief  初始化定点转向角度环 PID
  * @param  omni  底盘实例
  * @param  kp    角度环比例系数 (建议 4~6)
  * @param  ki    角度环积分系数 (建议先用 0, 避免大角度下积分饱和过冲)
  * @param  kd    角度环微分系数 (建议 0)
  * @note   在 Omni_Init 之后调用；内部固定 i_max=2.0, out_max=OMNI_TURN_MAX_OMEGA;
  *         同时清除直行/横移航向锁及PID历史；转向连续到位后在同一周期
  *         将done_flag置1，恢复追踪时按转向后的当前航向重新锁定
  */
void Omni_TurnInit(Omni_t *omni, float kp, float ki, float kd);

/** @brief 中止转向并停机(状态回到 IDLE) */
void Omni_TurnStop(Omni_t *omni);

/** @brief 查询转向状态, 返回 Omni_TurnState_t */
Omni_TurnState_t Omni_TurnGetState(const Omni_t *omni);

/** @brief 查询转向是否已到位(DONE), 1=到位 */
uint8_t Omni_TurnIsDone(const Omni_t *omni);

/**
  * @brief  单次任意角度转向(合并启动/闭环/完成校验, 每控制周期调用)
  * @param  omni       底盘实例
  * @param  delta_deg  相对转角(°, 正=逆时针, 负=顺时针, 任意值, 自动取最短路径)
  * @note   建议放在 TIM7 10ms 回调中每周期调用一次, 中断回调只需调用本函数:
  *         - 函数内注册航向角读取回调(Omni_AttachYawSource(Imu_GetYaw, ...))
  *         - if 标志位判断: done_flag 上电时在 Omni_TurnInit 里清零, 置位后
  *           不再执行转向
  *         - 标志位通过后读取一次当前航向角并记录(start_yaw), 启动转向
  *           (等待 IMU 数据新鲜, 防止读到上电默认值 0)
  *         - 转向中每周期执行角度环闭环; 转过了立即触发保护停机并锁存
  *         - 误差连续满足门限后在同一周期锁存done_flag，不按原始转角重新启动
  *         保护锁存/标志位置位后需重新上电(重新测量当前航向角)才能再次转向
  */
void Omni_TurnBy(Omni_t *omni, float delta_deg);

/* ---- 平移（陀螺仪 yaw 闭环保持航向） ---- */

/**
  * @brief  横移控制（每控制周期调用一次, 陀螺仪锁定航向）
  * @param  omni        底盘实例
  * @param  vy          横移速度 (m/s, +向左)
  * @param  current_yaw 当前航向角 (°, 来自 IMU, ±180°)
  * @note   使用独立的平移 yaw 角度环与锁定机制(Omni_LateralInit 初始化),
  *         与直线行驶互不影响; 首次调用自动锁定当前航向,
  *         之后边横移边修正偏航; 停止平移调用 Omni_LateralStop(清除锁定并停机);
  *         平移使能标志 g_drive_lateral_enable(main.c 定义)为 0 时不动作
  *         (与直线行驶标志分开, 不共用)
  */
void Omni_DriveLateral(Omni_t *omni, float vy, float current_yaw);

/**
  * @brief  斜线行驶控制(直线+平移合成, 每控制周期调用一次)
  * @param  omni        底盘实例
  * @param  vx          前进速度 (m/s, +向前)
  * @param  vy          横移速度 (m/s, +向左)
  * @param  current_yaw 当前航向角 (°, 来自 IMU, ±180°)
  * @note   与直线行驶共用 yaw 角度环与锁定(yaw_pid/target_yaw/yaw_locked);
  *         vx+vy 一次合成进运动学, 三路速度环每周期只执行一次。
  *         直线/平移分开调用会互相覆盖目标并造成速度环双重积分,
  *         双使能时请用本函数而不是依次调用 DriveStraight+DriveLateral。
  *         两个使能标志(g_drive_straight_enable/g_drive_lateral_enable)
  *         任一为 0 时不动作
  */
void Omni_DriveCombined(Omni_t *omni, float vx, float vy, float current_yaw);

/**
  * @brief  斜线行驶并持续跟踪指定目标航向
  * @param  omni        底盘实例
  * @param  vx          前进速度 (m/s, +向前)
  * @param  vy          横移速度 (m/s, +向左)
  * @param  current_yaw 当前航向角 (°, 来自 IMU, ±180°)
  * @param  target_yaw  目标航向角 (°, ±180°)
  * @note   与 Omni_DriveCombined 不同，本函数不会把当前航向重新锁存为目标；
  *         适用于有明确绝对航向目标的基本行驶控制。
  */
void Omni_DriveCombinedToYaw(Omni_t *omni, float vx, float vy,
                             float current_yaw, float target_yaw);

#endif /* __OMNI_H */
