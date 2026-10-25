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
static void LinePD_ApplyWheelSpeeds(Line_PD_t *pd)
{
    /*
     * ⚠️ 差速分配的方向 —— 这里曾经写反过，导致车一偏就朝更偏的方向冲出去。
     *
     * 【物理铁律】
     *   差速驱动要往哪边转，【哪边的轮子就要慢下来】：
     *       想左转 -> 左轮慢、右轮快
     *       想右转 -> 左轮快、右轮慢
     *
     * 【符号推导】
     *   error > 0 表示线在传感器【左侧】（Gray 的约定是左正右负）。
     *   线在左边，说明车往右偏了，需要【向左转】。
     *   向左转 = 左轮慢、右轮快，所以：
     *       left_speed  = base - steer
     *       right_speed = base + steer
     *
     * 【原来的错误写法】
     *       left_speed  = base + steer
     *   steer 为正时左轮反而加速，车向右转，越偏越远 —— 正反馈。
     *   实测表现：车放上赛道一开始就朝一个方向冲出去。
     */
    float left_speed  = pd->speed + LINE_STEER_DIR * pd->steer;
    float right_speed = pd->speed - LINE_STEER_DIR * pd->steer;

    /*
     * 只按【内环允许的目标速度上限】夹一次。
     *
     * ⚠️ 原来的代码这里还多夹了一次 diff_limit，那是错的：
     *    diff_limit 限制的是"差速量 steer"（在第 6 步已经夹过了），
     *    不是车轮的绝对速度。
     *
     *    以 base_speed=30、diff_limit=25 为例：
     *      直行 steer=0  ->  left=right=30  会被夹成 25，车速直接掉 17%
     *      转弯 steer=23 ->  left=53 夹成 25、right=7
     *                        内侧/外侧都跑不到该有的速度，平均速度只剩 16
     *
     *    表现就是：车能走，但明显比设定慢，转弯也不利索。
     *
     * 现在的做法：
     *    steer 已经限幅到 ±diff_limit，所以 |left-right| <= 2*diff_limit，
     *    差速本身是安全的；再按 speed_limit 夹一次就够了。
     */
    left_speed  = LinePD_Clamp(left_speed,  pd->speed_limit);
    right_speed = LinePD_Clamp(right_speed, pd->speed_limit);

    /*
     * 只保存目标，不在这里驱动速度环！
     *
     * 内环由 ControlTask 的 20ms 节拍单独驱动，
     * 这样速度环保持 50Hz、循迹环保持 10Hz，各自独立。
     * 详见 LinePD.h 里 left_target 字段的说明。
     */
    pd->left_target  = left_speed;
    pd->right_target = right_speed;
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
    pd->search_ms      = 0U;
    pd->left_target    = 0.0f;
    pd->right_target   = 0.0f;
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

uint8_t LinePD_Update(Line_PD_t *pd, Gray_Sensor_t *gray)
{
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

    /* ---------- 1. 检查传感器数据是否新鲜 ---------- */
    /*
     * 传感器由调用者在 20ms 节拍里读好（比控制律的 40ms 快一倍），
     * 这里只检查最近一次读取有没有连续失败。
     * fail_count 在 Gray_Update 里成功一次就清零，
     * 所以它反映的是"当前是否处于通信故障状态"。
     */
    if (gray->fail_count >= GRAY_FAIL_LIMIT)
    {
        /* I2C 通信失败：保持上一拍的电机指令不变，不拿脏数据去开车 */
        pd->line_lost = 1U;
        return LINE_PD_ERR_I2C;
    }

    /* ---------- 2. 取误差并按线性区限幅 ---------- */
    error_limit = LinePD_ErrorLimit();
    error = LinePD_Clamp(Gray_GetPositionCm(gray), error_limit);
    pd->error = error;

    /*
     * ---- 位置趋势 ----
     * pos_slow 是过去十几拍的位置平均值，error 是当前值。
     * 两者之差就是"线正在往哪边移动"：
     *   正 -> 位置在变大 -> 线往左移 -> 弯往左
     *   负 -> 线往右移 -> 弯往右
     *
     * 只在【真的看到线】时更新，丢线时的满幅回退值不能进滤波器，
     * 否则会把趋势值带偏。
     */
    if (Gray_IsLineLost(gray) == 0U)
    {
        pd->pos_trend = error - pd->pos_slow;
        pd->pos_slow += LINE_TREND_ALPHA * (error - pd->pos_slow);
    }

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
    /*
     * 基础速度带起步斜坡。
     *
     * 目标值 = base_speed × 弯道减速系数；
     * 实际值 pd->speed 每拍最多变化 (斜率 × 外环周期)，
     * 这样从 0 起步或者中途调大 LBAS 都不会出现阶跃。
     */
    {
        float target_speed = pd->base_speed * slow_factor;
        float step = LINE_SPEED_RAMP_CMS_PER_SEC
                     * ((float)LINE_PD_PERIOD_MS / 1000.0f);

        if (pd->speed < target_speed)
        {
            pd->speed += step;
            if (pd->speed > target_speed)
            {
                pd->speed = target_speed;
            }
        }
        else if (pd->speed > target_speed)
        {
            pd->speed -= step;
            if (pd->speed < target_speed)
            {
                pd->speed = target_speed;
            }
        }
    }

    /* ---------- 7. 丢线超时处理 ---------- */
    if (pd->lost_count > LINE_LOST_TOLERANCE)
    {
        pd->line_lost = 1U;

        /* 累计搜索时间（外环节拍固定，累加即可） */
        if (pd->search_ms < 0xFFFFU)
        {
            pd->search_ms = (uint16_t)(pd->search_ms + LINE_PD_PERIOD_MS);
        }

#if (LINE_SEARCH_ENABLE != 0)
        /*
         * ---- 刚进入搜索：锁定方向 ----
         *
         * 只在 search_ms 还是 0 的那一拍锁存，之后不再改变。
         * 这样即使斑块扫过、偶尔又看到一点线，也不会把方向翻掉。
         */
        if (pd->search_ms == 0U)
        {
            float trend = pd->pos_trend;
            float abs_trend = (trend < 0.0f) ? -trend : trend;

            if (abs_trend > LINE_TREND_THRESHOLD_CM)
            {
                /* 趋势明显：跟着线移动的方向转 */
                pd->search_dir = (trend > 0.0f) ? 1 : -1;
            }
            else
            {
                /* 趋势不明显（直线丢线）：退回用最后一刻的位置 */
                pd->search_dir = (pd->error >= 0.0f) ? 1 : -1;
            }
        }

#if (LINE_SEARCH_BIDIRECTIONAL != 0)
        if (pd->search_ms <= (uint16_t)(LINE_SEARCH_TIMEOUT_MS * 2U))
#else
        if (pd->search_ms <= LINE_SEARCH_TIMEOUT_MS)
#endif
        {
            /*
             * ---- 搜索模式：原地慢转找线 ----
             *
             * 转向方向由【最后看到线的方向】决定：
             *   Gray 模块丢线时会把位置固定成"最后偏离方向的满幅值"，
             *   所以 pd->error 的符号就是线最后所在的一侧。
             *
             * error > 0 表示线在左 -> 要向左转 -> 右轮前进、左轮反转。
             * 这里直接按"左转/右转"来分配，不走 steer 公式，
             * 因为搜索需要的是【确定的方向和幅度】，不是比例控制。
             */
            float v_outer = LINE_SEARCH_SPEED_CMS;
            float v_inner = -LINE_SEARCH_SPEED_CMS * LINE_SEARCH_INNER_RATIO;

            /*
             * 决定这一拍的方向。
             *
             * 单向模式：一直用进入搜索时锁定的方向。
             * 双向模式：前半程用锁存方向，后半程自动反方向。
             *           这样即使初始方向猜错，也能在第二阶段扫到线。
             */
            int8_t  dir_now = pd->search_dir;

#if (LINE_SEARCH_BIDIRECTIONAL != 0)
            if (pd->search_ms > LINE_SEARCH_TIMEOUT_MS)
            {
                dir_now = (int8_t)(-pd->search_dir);
            }
#endif

            uint8_t turn_left = (dir_now >= 0) ? 1U : 0U;

            if (turn_left != 0U)
            {
                /* 向左转：右轮前进，左轮反转 */
                pd->left_target  = v_inner;
                pd->right_target = v_outer;
            }
            else
            {
                /* 向右转：左轮前进，右轮反转 */
                pd->left_target  = v_outer;
                pd->right_target = v_inner;
            }

            pd->steer = 0.0f;
            pd->speed = 0.0f;

            return LINE_PD_ERR_LOST;
        }
#endif

        /*
         * ---- 搜索超时（或搜索被禁用）：停车 ----
         *
         * 已经转了 LINE_SEARCH_TIMEOUT_MS 还没找到线，
         * 说明线真的不在附近了。继续转下去只会让车跑出赛道或撞墙。
         */
        {
            const char *msg = "[LinePD] line lost, stop\r\n";
            Speed_PID_StopAll();

            /*
             * 必须把目标速度也清零！
             * Speed_PID_StopAll() 只复位 PID 内部状态，
             * 内环节拍每 20ms 会拿 pd->left_target/right_target 重新驱动，
             * 不清的话判了丢线车却还在跑。
             */
            pd->left_target  = 0.0f;
            pd->right_target = 0.0f;
            pd->steer        = 0.0f;
            pd->speed        = 0.0f;

            HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 20U);
        }

        return LINE_PD_ERR_LOST;
    }

    /*
     * ---- 确认"真的找回来了" ----
     *
     * 要求连续 LINE_RECOVER_CYCLES 拍都看到线才清空搜索状态。
     * 锐角弯里斑块扫过时经常只看到一拍，如果立刻退出搜索，
     * 下一拍又丢 -> 搜索里程重置 -> 方向重新锁存 -> 反复抽搐。
     */
    if (pd->recover_count < 0xFFU)
    {
        pd->recover_count++;
    }

    if (pd->recover_count >= LINE_RECOVER_CYCLES)
    {
        pd->search_ms   = 0U;
        pd->search_dir  = 0;
        pd->recover_count = LINE_RECOVER_CYCLES;
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
        uint8_t gray_result;
        float   pos_cm;
        float   error;
        const char *verdict;

        HAL_Delay(LINE_PD_PERIOD_MS);

        /* 这是独立的自检函数，自己读传感器，与 20ms 采样节拍无关 */
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

        result = LinePD_Update(pd, gray);

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
