//
// Created by JiangyaF on 2026/10/11.
//
// 内环速度 PID 实现
//
// 输出结构（前馈 + PID）：
//
//   占空比 = KFF * 目标速度        ← 前馈：负责"大体上该给多少"
//          + Kp * 误差             ← 比例：负责"现在差多少"
//          + Ki * Σ(误差 * dt)     ← 积分：负责"一直差一点就补上"
//          + Kd * d(误差)/dt       ← 微分：负责"预判要冲过头"
//
// 前馈项是整个设计的关键：它让 PID 只需要修正小偏差，而不是从零凑出全部输出。
// 没有前馈的话，直行时输出必须靠积分项慢慢攒出来，必然积分饱和、一转弯就失控。
//

#include "SpeedPID.h"
#include <stdio.h>
#include <string.h>

/* 串口句柄，自检时用于打印 */
extern UART_HandleTypeDef huart2;

/* ------------------------------------------------------------------------- */
/* 模块内部状态                                                                */
/* ------------------------------------------------------------------------- */

#define SPEED_PID_COUNT     2U      /* 两个轮子各一份 */

/* PID 参数与运行状态 */
static Speed_PID_t s_pid[SPEED_PID_COUNT];

/* 上一次的误差，用于计算微分项。静态变量默认初始化为 0 */
static float   s_last_error[SPEED_PID_COUNT];
static uint8_t s_last_error_valid[SPEED_PID_COUNT];

/* 采样周期，单位秒。用于积分和微分的量纲换算 */
#define SPEED_PID_DT_SEC    ((float)SPEED_PID_PERIOD_MS / 1000.0f)

/* 把编号映射到控制器实例。索引与 PWM.h 的 MOTOR_A / MOTOR_B 一致。 */
static Speed_PID_t *Speed_PID_Get(uint32_t motor)
{
    if (motor >= SPEED_PID_COUNT)
    {
        return NULL;
    }
    return &s_pid[motor];
}

/* ------------------------------------------------------------------------- */
/* 参数与状态管理                                                              */
/* ------------------------------------------------------------------------- */

void Speed_PID_Init(void)
{
    uint32_t i;

    for (i = 0U; i < SPEED_PID_COUNT; i++)
    {
        Speed_PID_t *pid = &s_pid[i];

        /* 编译期默认参数 */
        pid->kp  = SPEED_PID_DFLT_KP;
        pid->ki  = SPEED_PID_DFLT_KI;
        pid->kd  = SPEED_PID_DFLT_KD;
        pid->kff = SPEED_PID_DFLT_KFF;

        /* 输出限幅：TB6612 的占空比行程是 ±MOTOR_DUTY_MAX */
        pid->output_min = -SPEED_OUTPUT_LIMIT;
        pid->output_max = SPEED_OUTPUT_LIMIT;

        /* 目标速度限幅，避免外环异常时给出离谱值 */
        pid->target_limit = SPEED_LIMIT_CMS;

        /*
         * 积分限幅 = 输出限幅，这是抗饱和（anti-windup）的第一道防线：
         * 积分项自己最多只能把输出顶到满量程，不会无限累积。
         * 第二道防线是下面的"条件积分"。
         */
        pid->integral_min = pid->output_min;
        pid->integral_max = pid->output_max;

        /* 默认关闭微分滤波（kd 默认为 0，暂时用不上） */
        pid->d_filter = 1.0f;

        Speed_PID_Reset(i);
    }
}

void Speed_PID_SetGains(uint32_t motor, float kp, float ki, float kd, float kff)
{
    Speed_PID_t *pid = Speed_PID_Get(motor);

    if (pid == NULL)
    {
        return;
    }

    pid->kp  = kp;
    pid->ki  = ki;
    pid->kd  = kd;
    pid->kff = kff;

    Speed_PID_Reset(motor);
}

void Speed_PID_Reset(uint32_t motor)
{
    Speed_PID_t *pid = Speed_PID_Get(motor);

    if (pid == NULL)
    {
        return;
    }

    pid->target       = 0.0f;
    pid->measure      = 0.0f;
    pid->integral     = 0.0f;
    pid->d_filt_state = 0.0f;
    pid->output       = 0.0f;
    pid->saturated    = 0U;

    /* 清掉微分的历史，避免复位后第一拍产生虚假的巨大微分 */
    s_last_error[motor]       = 0.0f;
    s_last_error_valid[motor] = 0U;
}

/* ------------------------------------------------------------------------- */
/* PID 核心                                                                    */
/* ------------------------------------------------------------------------- */

float Speed_PID_GetGain(uint32_t motor, uint8_t which)
{
    Speed_PID_t *pid = Speed_PID_Get(motor);

    if (pid == NULL)
    {
        return 0.0f;
    }

    switch (which)
    {
    case SPEED_PID_GAIN_KP:  return pid->kp;
    case SPEED_PID_GAIN_KI:  return pid->ki;
    case SPEED_PID_GAIN_KD:  return pid->kd;
    case SPEED_PID_GAIN_KFF: return pid->kff;
    default:                 return 0.0f;
    }
}

void Speed_PID_SetGain(uint32_t motor, uint8_t which, float value)
{
    Speed_PID_t *pid = Speed_PID_Get(motor);

    if (pid == NULL)
    {
        return;
    }

    switch (which)
    {
    case SPEED_PID_GAIN_KP:  pid->kp  = value; break;
    case SPEED_PID_GAIN_KI:  pid->ki  = value; break;
    case SPEED_PID_GAIN_KD:  pid->kd  = value; break;
    case SPEED_PID_GAIN_KFF: pid->kff = value; break;
    default: return;
    }

    /* 参数变了，积分和微分滤波状态都失去意义，清掉重新收敛 */
    Speed_PID_Reset(motor);
}

float Speed_PID_Update(uint32_t motor, float target)
{
    Speed_PID_t *pid = Speed_PID_Get(motor);
    float error;
    float derivative;
    float integral_candidate;
    float out;

    if (pid == NULL)
    {
        return 0.0f;
    }

    /* ---------- 1. 取实测值（编码器模块内部已做低通滤波） ---------- */
    pid->measure = Encoder_GetSpeed(motor);

    /* ---------- 2. 目标值限幅 ---------- */
    if (target > pid->target_limit)
    {
        target = pid->target_limit;
    }
    else if (target < -pid->target_limit)
    {
        target = -pid->target_limit;
    }
    pid->target = target;

    /* ---------- 3. 偏差 ---------- */
    error = target - pid->measure;

    /* ---------- 4. 微分项：误差的变化率，带低通滤波 ---------- */
    if (s_last_error_valid[motor] == 0U)
    {
        /* 首次运行：把上次误差对齐到当前值，微分记 0，避免起跳冲击 */
        s_last_error[motor]       = error;
        s_last_error_valid[motor] = 1U;
        derivative = 0.0f;
    }
    else
    {
        derivative = (error - s_last_error[motor]) / SPEED_PID_DT_SEC;
        s_last_error[motor] = error;
    }

    /* 一阶低通：d_filter 越小越平滑但越滞后。等于 1.0f 时相当于不滤波 */
    pid->d_filt_state += pid->d_filter * (derivative - pid->d_filt_state);
    derivative = pid->d_filt_state;

    /* ---------- 5. 积分项：带条件积分抗饱和 ---------- */
    integral_candidate = pid->integral + pid->ki * error * SPEED_PID_DT_SEC;

    {
        /*
         * 条件积分：若上一拍输出已经正向饱和、而当前误差还想把它推得更正，
         * 就停止累加。反向同理。这是防止积分饱和最有效的一招。
         */
        uint8_t pushing_positive = (pid->saturated == 1U) && (error > 0.0f);
        uint8_t pushing_negative = (pid->saturated == 2U) && (error < 0.0f);

        if ((pushing_positive == 0U) && (pushing_negative == 0U))
        {
            if (integral_candidate > pid->integral_max)
            {
                integral_candidate = pid->integral_max;
            }
            else if (integral_candidate < pid->integral_min)
            {
                integral_candidate = pid->integral_min;
            }
            pid->integral = integral_candidate;
        }
    }

    /* ---------- 6. 合成输出：前馈 + P + I + D ---------- */
    out = pid->kff * target
        + pid->kp  * error
        + pid->integral
        + pid->kd  * derivative;

    /* ---------- 7. 输出限幅，并记录饱和方向供下一拍使用 ---------- */
    if (out > pid->output_max)
    {
        out = pid->output_max;
        pid->saturated = 1U;
    }
    else if (out < pid->output_min)
    {
        out = pid->output_min;
        pid->saturated = 2U;
    }
    else
    {
        pid->saturated = 0U;
    }

    pid->output = out;

    /* ---------- 8. 驱动电机 ---------- */
    Motor_SetOne(motor, (int16_t)out);

    return out;
}

/* ------------------------------------------------------------------------- */
/* 状态查询与停止                                                              */
/* ------------------------------------------------------------------------- */

float Speed_PID_GetTarget(uint32_t motor)
{
    Speed_PID_t *pid = Speed_PID_Get(motor);
    return (pid == NULL) ? 0.0f : pid->target;
}

float Speed_PID_GetMeasure(uint32_t motor)
{
    Speed_PID_t *pid = Speed_PID_Get(motor);
    return (pid == NULL) ? 0.0f : pid->measure;
}

float Speed_PID_GetOutput(uint32_t motor)
{
    Speed_PID_t *pid = Speed_PID_Get(motor);
    return (pid == NULL) ? 0.0f : pid->output;
}

void Speed_PID_Stop(uint32_t motor)
{
    if (Speed_PID_Get(motor) == NULL)
    {
        return;
    }

    Speed_PID_Reset(motor);
    Motor_SetOne(motor, 0);     /* 滑行停止 */
}

void Speed_PID_StopAll(void)
{
    Speed_PID_Stop(MOTOR_A);
    Speed_PID_Stop(MOTOR_B);
}

/* ------------------------------------------------------------------------- */
/* 自检：阶跃响应                                                              */
/* ------------------------------------------------------------------------- */

void Speed_PID_TestSequence(void)
{
    /* 每档目标速度保持的 tick 数：20ms x 50 = 1000ms */
    #define TEST_TICKS_PER_STEP     50U
    #define TEST_PRINT_EVERY        5U      /* 每 5 拍打印一行，约 100ms 一行 */

    static const float test_targets[] = { 20.0f, 30.0f, 40.0f, 0.0f };
    const uint32_t step_count = (uint32_t)(sizeof(test_targets) / sizeof(test_targets[0]));

    char     buf[192];      /* 需容纳表头与每行数据，留足余量 */
    uint32_t step;
    uint32_t tick;

    {
        int len = snprintf(buf, sizeof(buf),
                           "\r\n[SpeedPID] step response test, please lift the wheels\r\n"
                           "  target(cm/s) |  A:meas  A:out  |  B:meas  B:out\r\n"
                           "  ---------------------------------------------------\r\n");
        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
        }
    }

    for (step = 0U; step < step_count; step++)
    {
        /* 换档时复位积分，避免上一档的积分残留干扰本档观察 */
        Speed_PID_Reset(MOTOR_A);
        Speed_PID_Reset(MOTOR_B);

        for (tick = 0U; tick < TEST_TICKS_PER_STEP; tick++)
        {
            HAL_Delay(SPEED_PID_PERIOD_MS);

            /*
             * 顺序不能反：先采编码器刷新速度，再跑 PID。
             * 反过来的话 PID 用的就是上一拍的旧速度，凭空引入一个周期的延迟。
             */
            Encoder_UpdateSpeed();
            Speed_PID_Update(MOTOR_A, test_targets[step]);
            Speed_PID_Update(MOTOR_B, test_targets[step]);

            if ((tick % TEST_PRINT_EVERY) == 0U)
            {
                int len = snprintf(buf, sizeof(buf),
                                   "      %5.1f     |  %6.1f %6.1f  |  %6.1f %6.1f\r\n",
                                   (double)test_targets[step],
                                   (double)Speed_PID_GetMeasure(MOTOR_A),
                                   (double)Speed_PID_GetOutput(MOTOR_A),
                                   (double)Speed_PID_GetMeasure(MOTOR_B),
                                   (double)Speed_PID_GetOutput(MOTOR_B));
                if (len > 0)
                {
                    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
                }
            }
        }
    }

    Speed_PID_StopAll();

    {
        const char *done = "  ---------------------------------------------------\r\n"
                           "[SpeedPID] test done, motors stopped\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)done, (uint16_t)strlen(done), 100U);
    }
}

//
// Created by JiangyaF on 2026/10/11.
//
