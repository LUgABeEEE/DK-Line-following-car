//
// Created by JiangyaF on 2026/10/11.
//
// 外环循迹 PD 实现
//
// 控制律（无积分项，原因见头文件）：
//
//   差速量 = Kp * error + Kd * d(error)/dt
//   基础速度 = base_speed * 弯道减速系数
//   左轮目标速度 = 基础速度 + 差速量
//   右轮目标速度 = 基础速度 - 差速量
//
// 符号链（三者必须自洽，否则闭环变正反馈）：
//   误差 > 0（线偏左）
//     -> 差速量 > 0
//     -> 左轮减速、右轮加速
//     -> 车向左转（车头朝左）
//     -> 线相对传感器中心向右移动，误差减小   <-- 负反馈成立
//

#include "LinePD.h"
#include <stdio.h>
#include <string.h>

/* 串口句柄，自检时用于打印 */
extern UART_HandleTypeDef huart2;

/* ------------------------------------------------------------------------- */
/* 内部辅助                                                                    */
/* ------------------------------------------------------------------------- */

static float LinePD_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

/* 误差限幅值 = 阵列半跨度（转成 cm）。超出这个范围灰度模块已经饱和了 */
static float LinePD_ErrorLimit(void)
{
    return GRAY_HALF_SPAN_MM * 0.1f;
}

/* 把两个轮子的目标速度交给内环，并立即执行一次速度环 */
static void LinePD_ApplyWheelSpeeds(const Line_PD_t *pd)
{
    float left_speed  = pd->speed + pd->steer;
    float right_speed = pd->speed - pd->steer;

    /*
     * 1. 先按整体速度上限夹一次，防止"基础速度 + 差速量"顶破内环量程
     * 2. 再按差速限幅夹一次。因为 |steer| <= diff_limit，夹完必然满足
     *    |左右之差| <= diff_limit，内侧轮不会被反向拖成倒转。
     *    默认 diff_limit(25) < base_speed(30)，所以内侧轮最慢只会到 0，不会反转。
     */
    left_speed  = LinePD_Clamp(left_speed,  pd->speed_limit);
    right_speed = LinePD_Clamp(right_speed, pd->speed_limit);
    left_speed  = LinePD_Clamp(left_speed,  pd->diff_limit);
    right_speed = LinePD_Clamp(right_speed, pd->diff_limit);

    Speed_PID_Update(MOTOR_A, left_speed);
    Speed_PID_Update(MOTOR_B, right_speed);
}

/* ------------------------------------------------------------------------- */
/* 初始化与参数                                                                */
/* ------------------------------------------------------------------------- */

void LinePD_Init(Line_PD_t *pd)
{
    if (pd == NULL)
    {
        return;
    }

    pd->kp          = LINE_PD_DFLT_KP;
    pd->kd          = LINE_PD_DFLT_KD;
    pd->base_speed  = LINE_PD_DFLT_BASE_SPEED;
    pd->diff_limit  = LINE_DIFF_LIMIT;
    pd->speed_limit = LINE_SPEED_LIMIT;
    pd->d_filter    = LINE_PD_DFLT_D_FILTER;

    LinePD_Reset(pd);
}

void LinePD_Reset(Line_PD_t *pd)
{
    if (pd == NULL)
    {
        return;
    }

    pd->error          = 0.0f;
    pd->last_error     = 0.0f;
    pd->d_filter_state = 0.0f;
    pd->steer          = 0.0f;
    pd->speed          = 0.0f;
    pd->lost_count     = 0U;
    pd->line_lost      = 0U;
    pd->first_run      = 1U;
}

void LinePD_SetGains(Line_PD_t *pd, float kp, float kd, float base_speed)
{
    if (pd == NULL)
    {
        return;
    }

    pd->kp         = kp;
    pd->kd         = kd;
    pd->base_speed = base_speed;

    LinePD_Reset(pd);
}

void LinePD_Stop(Line_PD_t *pd)
{
    if (pd == NULL)
    {
        return;
    }

    Speed_PID_StopAll();
    LinePD_Reset(pd);
}

/* ------------------------------------------------------------------------- */
/* 核心                                                                        */
/* ------------------------------------------------------------------------- */

uint8_t LinePD_Update(Line_PD_t *pd, Gray_Sensor_t *gray, I2C_HandleTypeDef *hi2c)
{
    uint8_t gray_result;
    float   error;
    float   derivative;
    float   steer;
    float   slow_factor;
    float   error_limit;

    if ((pd == NULL) || (gray == NULL))
    {
        return LINE_PD_ERR_PERIOD;
    }

    /* ---------- 0. 周期合理性检查（配置错误就拒绝运行，避免整车振荡） ---------- */
    if ((SPEED_PID_PERIOD_MS * LINE_MIN_PERIOD_RATIO) > LINE_PD_PERIOD_MS)
    {
        Speed_PID_StopAll();
        return LINE_PD_ERR_PERIOD;
    }

    /* ---------- 1. 读传感器（内部已算好加权质心位置） ---------- */
    gray_result = Gray_Update(gray, hi2c);

    if ((gray_result == 1U) || (gray_result == 2U))
    {
        /* I2C 通信失败：保持上一拍的电机指令不变，不拿脏数据去开车 */
        pd->line_lost = 1U;
        return LINE_PD_ERR_I2C;
    }

    /* ---------- 2. 取误差并按线性区限幅 ---------- */
    error_limit = LinePD_ErrorLimit();
    error = LinePD_Clamp(Gray_GetPositionCm(gray), error_limit);
    pd->error = error;

    /* ---------- 3. 丢线计数 ---------- */
    if (Gray_IsLineLost(gray) != 0U)
    {
        if (pd->lost_count < 0xFFU)
        {
            pd->lost_count++;
        }
    }
    else
    {
        pd->lost_count = 0U;
    }

    /* ---------- 4. 微分项：带低通滤波，首拍不做微分 ---------- */
    if (pd->first_run != 0U)
    {
        pd->first_run  = 0U;
        pd->last_error = error;
        derivative     = 0.0f;
    }
    else
    {
        float dt_sec = (float)LINE_PD_PERIOD_MS / 1000.0f;
        derivative = (error - pd->last_error) / dt_sec;
        pd->last_error = error;
    }

    pd->d_filter_state += pd->d_filter * (derivative - pd->d_filter_state);
    derivative = pd->d_filter_state;

    /* ---------- 5. PD 运算 + 差速限幅 ---------- */
    steer = (pd->kp * error) + (pd->kd * derivative);
    steer = LinePD_Clamp(steer, pd->diff_limit);
    pd->steer = steer;

    /* ---------- 6. 弯道减速：偏离越大跑得越慢，给转向留时间 ---------- */
    slow_factor = 1.0f;
    if ((LINE_CURVE_SLOW_MIN < 1.0f) && (error_limit > 0.001f))
    {
        float normalized = (error < 0.0f) ? -error : error;   /* 取绝对值 */
        normalized /= error_limit;                            /* 0 ~ 1      */
        if (normalized > 1.0f)
        {
            normalized = 1.0f;
        }
        /* 线性插值：正中时为 1.0，偏到极限时为 LINE_CURVE_SLOW_MIN */
        slow_factor = 1.0f - (normalized * (1.0f - LINE_CURVE_SLOW_MIN));
    }
    pd->speed = pd->base_speed * slow_factor;

    /* ---------- 7. 丢线超时处理 ---------- */
    if (pd->lost_count > LINE_LOST_TOLERANCE)
    {
        pd->line_lost = 1U;

#if (LINE_STOP_IF_LINE_LOST != 0)
        {
            const char *msg = "[LinePD] line lost, stop\r\n";
            Speed_PID_StopAll();
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 20U);
        }
#else
        /*
         * 搜索模式：保持转向（Gray 模块在丢线时会给出"最后偏离方向"的满幅误差），
         * 但把前进速度压到最低，让车尽量在原地附近找回线。
         */
        {
            float low_speed = pd->base_speed * LINE_CURVE_SLOW_MIN * 0.5f;
            float left_speed  = low_speed + steer;
            float right_speed = low_speed - steer;

            left_speed  = LinePD_Clamp(left_speed,  pd->speed_limit);
            right_speed = LinePD_Clamp(right_speed, pd->speed_limit);
            Speed_PID_Update(MOTOR_A, left_speed);
            Speed_PID_Update(MOTOR_B, right_speed);
        }
#endif

        return LINE_PD_ERR_LOST;
    }

    /* ---------- 8. 正常，输出到内环 ---------- */
    pd->line_lost = 0U;
    LinePD_ApplyWheelSpeeds(pd);

    return LINE_PD_OK;
}

/* ------------------------------------------------------------------------- */
/* 查询接口                                                                    */
/* ------------------------------------------------------------------------- */

float LinePD_GetError(const Line_PD_t *pd)
{
    return (pd == NULL) ? 0.0f : pd->error;
}

float LinePD_GetSteer(const Line_PD_t *pd)
{
    return (pd == NULL) ? 0.0f : pd->steer;
}

float LinePD_GetSpeed(const Line_PD_t *pd)
{
    return (pd == NULL) ? 0.0f : pd->speed;
}

uint8_t LinePD_IsLineLost(const Line_PD_t *pd)
{
    return (pd == NULL) ? 1U : pd->line_lost;
}

/* ------------------------------------------------------------------------- */
/* 符号与转向方向验证                                                          */
/* ------------------------------------------------------------------------- */

void LinePD_VerifySigns(Line_PD_t *pd, Gray_Sensor_t *gray,
                        I2C_HandleTypeDef *hi2c, uint8_t interactive)
{
    char     buf[320];   /* 需容纳多行表头，留足余量 */
    uint32_t i;

    if ((pd == NULL) || (gray == NULL))
    {
        return;
    }

    {
        int len = snprintf(buf, sizeof(buf),
                           "\r\n=====================================================\r\n"
                           "[LinePD] sign verification\r\n"
                           "  expected: error > 0 (line on the LEFT)\r\n"
                           "            error < 0 (line on the RIGHT)\r\n"
                           "            error ~ 0 (line centered)\r\n"
                           "=====================================================\r\n");
        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
        }
    }

    /* ---------- 第 1 段：只读传感器，电机全程不动 ---------- */
    {
        const char *msg = "STEP 1/2  motors OFF -- move the line: CENTER, LEFT, RIGHT\r\n"
                          "  time   pos(cm)  error  state  active  sum    verdict\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 100U);
    }

    for (i = 0U; i < 200U; i++)     /* 200 x 100ms = 20 秒 */
    {
        float   pos_cm;
        float   error;
        uint8_t gray_result;
        const char *verdict;

        HAL_Delay(LINE_PD_PERIOD_MS);

        gray_result = Gray_Update(gray, hi2c);

        if ((gray_result == 1U) || (gray_result == 2U))
        {
            int len = snprintf(buf, sizeof(buf),
                               "  %4u   ---- I2C FAIL (result=%u) ----\r\n",
                               (unsigned int)i, (unsigned int)gray_result);
            if (len > 0)
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
            }
            continue;
        }

        pos_cm = Gray_GetPositionCm(gray);
        error  = LinePD_Clamp(pos_cm, LinePD_ErrorLimit());

        if (Gray_IsLineLost(gray) != 0U)
        {
            verdict = "LINE LOST";
        }
        else if (error > 0.5f)
        {
            verdict = "LEFT  (sign ok if line is left)";
        }
        else if (error < -0.5f)
        {
            verdict = "RIGHT (sign ok if line is right)";
        }
        else
        {
            verdict = "CENTER";
        }

        {
            int len = snprintf(buf, sizeof(buf),
                               "  %4u   %+6.2f  %+6.2f  0x%02X    %u    %5.2f  %s\r\n",
                               (unsigned int)i, (double)pos_cm, (double)error,
                               (unsigned int)gray->state_byte,
                               (unsigned int)gray->active_count,
                               (double)gray->strength, verdict);
            if (len > 0)
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
            }
        }
    }

    /* ---------- 第 2 段：转向方向验证（会转电机） ---------- */
    if (interactive == 0U)
    {
        const char *msg = "STEP 2/2  skipped (interactive = 0), motors stay OFF\r\n"
                          "[LinePD] sign check done\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 100U);
        return;
    }

    {
        const char *msg = "\r\nSTEP 2/2  steering check -- the car WILL move, keep it clear\r\n"
                          "  A) left wheel 0%%, right wheel 25%%  -> expect: turn LEFT  (counter-clockwise)\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 100U);
    }

    Motor_SetSpeed(0, 25);
    HAL_Delay(2000U);
    Motor_SetSpeed(0, 0);
    HAL_Delay(800U);

    {
        const char *msg = "  B) left wheel 25%%, right wheel 0%  -> expect: turn RIGHT (clockwise)\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 100U);
    }

    Motor_SetSpeed(25, 0);
    HAL_Delay(2000U);
    Motor_SetSpeed(0, 0);
    HAL_Delay(800U);

    {
        const char *msg = "[LinePD] sign check done, motors stopped\r\n"
                          "  if the turns were reversed, swap MOTOR_A / MOTOR_B wiring\r\n"
                          "  or invert the two channels of Motor_SetSpeed()\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 100U);
    }
}

/* ------------------------------------------------------------------------- */
/* 循迹自检                                                                    */
/* ------------------------------------------------------------------------- */

void LinePD_TestSequence(Line_PD_t *pd, Gray_Sensor_t *gray,
                         I2C_HandleTypeDef *hi2c, uint32_t duration_sec)
{
    char     buf[320];   /* 需容纳多行表头，留足余量 */
    uint32_t elapsed_ms = 0U;
    uint32_t total_ms;

    if ((pd == NULL) || (gray == NULL))
    {
        return;
    }

    total_ms = duration_sec * 1000U;

    {
        int len = snprintf(buf, sizeof(buf),
                           "\r\n[LinePD] line following test for %u s\r\n"
                           "  kp=%.2f  kd=%.2f  base=%.1fcm/s  diff_limit=%.1f  error_limit=%.2fcm\r\n"
                           "  period: outer=%ums  inner=%ums  ratio=%u\r\n"
                           "  time   err(cm)  steer   base  L:tgt  R:tgt  L:meas R:meas\r\n",
                           (unsigned int)duration_sec,
                           (double)pd->kp, (double)pd->kd,
                           (double)pd->base_speed, (double)pd->diff_limit,
                           (double)LinePD_ErrorLimit(),
                           (unsigned int)LINE_PD_PERIOD_MS,
                           (unsigned int)SPEED_PID_PERIOD_MS,
                           (unsigned int)(LINE_PD_PERIOD_MS / SPEED_PID_PERIOD_MS));
        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
        }
    }

    LinePD_Reset(pd);
    Encoder_Reset();

    while (elapsed_ms < total_ms)
    {
        uint8_t result;

        HAL_Delay(LINE_PD_PERIOD_MS);
        elapsed_ms += LINE_PD_PERIOD_MS;

        result = LinePD_Update(pd, gray, hi2c);

        {
            float left_target  = pd->speed + pd->steer;
            float right_target = pd->speed - pd->steer;
            const char *tag = "";

            if (result == LINE_PD_ERR_I2C)
            {
                tag = " I2C-FAIL";
            }
            else if (result == LINE_PD_ERR_LOST)
            {
                tag = " LINE-LOST";
            }
            else if (result == LINE_PD_ERR_PERIOD)
            {
                tag = " BAD-PERIOD";
            }

            {
                int len = snprintf(buf, sizeof(buf),
                                   "  %5u  %+6.2f  %+6.2f  %5.1f  %+6.1f %+6.1f  %+6.1f %+6.1f%s\r\n",
                                   (unsigned int)(elapsed_ms / 1U),
                                   (double)pd->error, (double)pd->steer,
                                   (double)pd->speed,
                                   (double)left_target, (double)right_target,
                                   (double)Encoder_GetSpeed(MOTOR_A),
                                   (double)Encoder_GetSpeed(MOTOR_B),
                                   tag);
                if (len > 0)
                {
                    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
                }
            }
        }

        /* 遇到严重错误就别再继续跑了 */
        if ((result == LINE_PD_ERR_PERIOD) || (result == LINE_PD_ERR_I2C))
        {
            break;
        }
    }

    LinePD_Stop(pd);

    {
        const char *done = "[LinePD] test done, stopped\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)done, (uint16_t)strlen(done), 100U);
    }
}

//
// Created by JiangyaF on 2026/10/11.
//
