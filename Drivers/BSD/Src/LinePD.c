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
#include "Encoder.h"        /* 自动停车要用累计里程 */
#include "MPU6050.h"        /* 陀螺航向 / yaw 阻尼 */
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
    pd->straight_ticks = 0U;
    pd->speed_boost    = 0.0f;
    pd->pos_filter     = 0.0f;
    pd->theta_rad      = 0.0f;
    pd->last_pos_f     = 0.0f;
    pd->last_dist_cm   = 0.0f;
    pd->hist_count     = 0U;
    pd->theta_ff       = 0.0f;
    memset(pd->pos_hist,  0, sizeof(pd->pos_hist));
    memset(pd->dist_hist, 0, sizeof(pd->dist_hist));
    pd->commit_target_deg = 0.0f;
    pd->race_enabled   = 0U;
    pd->race_phase     = 0U;
    pd->race_pulse     = 0U;
    pd->race_lap_cm    = 0.0f;
    pd->race_start_cm  = 0.0f;
    pd->race_detect_cm = 0.0f;
    pd->lost_count     = 0U;
    pd->line_lost      = 0U;
    pd->first_run      = 1U;
    pd->search_ms      = 0U;
    pd->left_target    = 0.0f;
    pd->right_target   = 0.0f;

    /* 搜索与角点事件状态一并清掉：重置后必须从"不在任何事件里"开始 */
    pd->search_dir     = 0;
    pd->recover_count  = 0U;
    pd->corner_active  = 0U;
    pd->corner_confirm = 0U;
    pd->corner_release = 0U;
    pd->corner_dir     = 0;
    pd->corner_dir_src = 0U;
    pd->corner_dir_alt = 0U;
    pd->corner_dir_flip_tick = 0U;
    pd->corner_dir_hold = 0U;
    pd->corner_acute   = 0U;
    pd->commit_ms      = 0U;
    pd->commit_active  = 0U;
    pd->commit_done    = 0U;
    pd->commit_dir     = 0;
    pd->last_commit_turn = 0.0f;
    pd->yaw_deg        = 0.0f;
    pd->corner_yaw0    = 0.0f;
    pd->last_yaw_tick  = 0U;
    pd->all_assist     = 0U;
    pd->all_count      = 0U;
    pd->scene_last     = 0U;
    pd->lost_announced = 0U;
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

/*
 * 自适应模式学到的"一圈周长"。
 *
 * 只要有一次成功识别到启停线，就用"识别时的里程 + 17cm"估出周长，
 * 之后即使高速漏检，也能按里程兜底停车。
 * 断电丢失（放 RAM）——重启后先低速跑一圈即可重新学到。
 */
#if (LINE_RACE_LEARN_LAP != 0)
static float s_learned_lap_cm = 0.0f;
#endif

/* 两轮累计里程的平均值，cm（编码器量的是轮子转过的距离） */
static float LinePD_RaceTravel(const Line_PD_t *pd)
{
    (void)pd;
    return (Encoder_GetDistance(ENCODER_A) + Encoder_GetDistance(ENCODER_B)) * 0.5f;
}

uint8_t LinePD_Update(Line_PD_t *pd, Gray_Sensor_t *gray)
{
    float   error;
    float   derivative;
    float   steer;
    float   slow_factor;
    float   error_limit;
    uint8_t assist;

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

    /* ---------- 1b. 偏航角：优先用陀螺，没有模块时退回编码器 ---------- */
    /*
     * yaw += (v_right - v_left) / 轮距 × dt
     *   左转 -> 右轮快 -> 为正，与 corner_dir 的符号约定一致。
     * dt 用 HAL_GetTick 的真实间隔，不用名义 40ms：
     *   外环偶尔会被高优先级任务推迟，按名义周期积分会系统性少算角度。
     */
    if ((LINE_USE_IMU_YAW != 0) && (MPU6050_IsReady() != 0U))
    {
        /*
         * B) 陀螺航向：直接取 MPU6050 积分好的角度。
         *    打滑时轮子转得比车身多，编码器差分算出的角度偏大；
         *    陀螺测的是车身真实转动，直角/锐角的定角转弯因此更准。
         */
        pd->yaw_deg = MPU6050_GetYawDeg();
        pd->last_yaw_tick = HAL_GetTick();
    }
    else
    {
        uint32_t now_tick = HAL_GetTick();
        uint32_t dt_ms    = now_tick - pd->last_yaw_tick;

        if ((dt_ms > 0U) && (dt_ms < 200U))
        {
            float v_left  = Speed_PID_GetMeasure(MOTOR_A);
            float v_right = Speed_PID_GetMeasure(MOTOR_B);

            pd->yaw_deg += (v_right - v_left) / LINE_TRACK_WIDTH_CM
                           * ((float)dt_ms / 1000.0f)
                           * 57.2957795f;      /* rad -> deg */
        }
        pd->last_yaw_tick = now_tick;
    }

    /* ---------- 1c. 比赛自动停车：启停线识别 + 里程 ---------- */
    /*
     * 阶段机：
     *   0 等待   —— 还没跑过半圈（防止起跑线/杂线误触发）
     *   1 巡航   —— 已武装，只等启停线的"短全黑脉冲"
     *   2 接近   —— 已识别，按 12cm/s 直行到"灰度条到测试点"的距离
     *   3 到位   —— 通知 ControlTask 硬刹停车
     */
    if (pd->race_enabled != 0U)
    {
        float traveled = LinePD_RaceTravel(pd) - pd->race_start_cm;

        if (pd->race_phase == 0U)
        {
            float arm_cm = (pd->race_lap_cm > 0.0f)
                           ? (pd->race_lap_cm * LINE_RACE_ARM_FRAC)
                           : LINE_RACE_ARM_MIN_CM;

            if (traveled >= arm_cm)
            {
                pd->race_phase = 1U;
            }
        }
        else if (pd->race_phase == 1U)
        {
            /*
             * 启停线识别。
             *
             * 判定在 Gray_Update（20ms 一拍）里做：5cm 横线在 50cm/s 时
             * 只黑 36ms，外环 40ms 一拍可能整拍漏掉（实测就是这样丢的）。
             * 这里只消费 Gray 锁存的"短全黑脉冲"标志。
             */
            if (gray->stop_bar_seen != 0U)
            {
                /*
                 * 判定（宁漏不误）：
                 *   - 必须【没有方向锁存】：真角点一定有 SIDE/FORK 方向，
                 *     启停线只是短横线，方向保持 0；
                 *   - 横向误差 <1.2cm；
                 *   - 脉冲宽度由 Gray 限制在 <=3 拍（角点全黑会持续更久）。
                 * 高速漏检由"学到的周长"兜底，不在弯道冒误停的风险。
                 */
                if ((pd->corner_dir == 0) &&
                    (gray->position_cm > -LINE_RACE_DETECT_ERR_CM) &&
                    (gray->position_cm <  LINE_RACE_DETECT_ERR_CM) &&
                    ((pd->race_lap_cm > 0.0f) ||
                     (traveled >= LINE_RACE_DETECT_MIN_CM)))
                {
                    pd->race_phase     = 2U;
                    pd->race_detect_cm = traveled;

#if (LINE_RACE_LEARN_LAP != 0)
                    /* 学习周长：识别点 + 17cm（只做自适应模式） */
                    if ((pd->race_lap_cm <= 0.0f) &&
                        ((traveled + LINE_RACE_STOP_OFFSET_CM) >= LINE_RACE_DETECT_MIN_CM))
                    {
                        float lap_est = traveled + LINE_RACE_STOP_OFFSET_CM;

                        if (s_learned_lap_cm > 0.0f)
                        {
                            s_learned_lap_cm = (s_learned_lap_cm * 0.7f) + (lap_est * 0.3f);
                        }
                        else
                        {
                            s_learned_lap_cm = lap_est;
                        }
                    }
#endif
                }
                gray->stop_bar_seen = 0U;
            }

            /* 兜底：有周长按周长到位，自适应模式按最大里程强制停 */
            if (pd->race_phase == 1U)
            {
                if ((pd->race_lap_cm > 0.0f) &&
                    (traveled >= (pd->race_lap_cm - LINE_RACE_FALLBACK_CM)))
                {
                    pd->race_phase = 3U;
                }
#if (LINE_RACE_LEARN_LAP != 0)
                else if ((pd->race_lap_cm <= 0.0f) &&
                         (s_learned_lap_cm > 0.0f) &&
                         (traveled >= (s_learned_lap_cm - LINE_RACE_FALLBACK_CM)))
                {
                    /* 自适应模式：用上次学到的周长兜底（高速漏检时靠它） */
                    pd->race_phase = 3U;
                }
#endif
                else if ((pd->race_lap_cm <= 0.0f) &&
                         (traveled >= LINE_RACE_MAX_CM))
                {
                    pd->race_phase = 3U;
                }
            }
        }
        else if (pd->race_phase == 2U)
        {
            float approach = traveled - pd->race_detect_cm;

            if (approach >= (LINE_RACE_STOP_OFFSET_CM - LINE_RACE_BRAKE_MARGIN_CM))
            {
                pd->race_phase = 3U;
            }
        }
        else
        {
            /* 阶段 3：已经到位，等 ControlTask 停车 */
        }
    }

    /* ---------- 2. 角点事件状态更新（SIDE / FORK 统一处理） ---------- */
    /*
     * 判据全部来自 gray->scene。它已经在 Gray_Update 里做过时间防抖，
     * 所以这里看到的是"稳定场景"，不会一帧 SIDE、一帧 FORK。
     *
     * 事件生命周期：
     *   进入：SIDE_L / SIDE_R / FORK 连续 LINE_CORNER_CONFIRM_CYCLES 拍
     *   保持：方向锁存，不再重判（这是"不来回跳"的关键）
     *   退出：连续 LINE_CORNER_RELEASE_CYCLES 拍都是 LINE
     *
     * 丢线（NONE）不释放事件：角点里丢线恰恰要保留锁存方向去搜索。
     */
    pd->scene_last = (uint8_t)gray->scene;

    if (gray->scene == GRAY_SCENE_ALL)
    {
        if (pd->all_count < 0xFFU)
        {
            pd->all_count++;
        }
    }
    else
    {
        pd->all_count = 0U;
    }

#if (LINE_STOP_ON_ALL_BLACK != 0)
    if ((pd->corner_active == 0U) &&
        (pd->all_count >= LINE_ALL_CONFIRM_CYCLES))
    {
        /*
         * 几乎全部通道都激活 —— 横向黑线 / 终点标记 / 传感器离开赛道。
         * 停车是唯一安全的处理方式：此时位置解算没有意义
         * （整片都是"线"），继续按质心开车必然乱跑。
         *
         * 角点事件内不触发：锐角弯的弯心同样会全亮，
         * 那时需要的是"继续转"，不是"停"。
         */
        const char *msg = "[LinePD] all-black, stop\r\n";

        Speed_PID_StopAll();
        pd->left_target  = 0.0f;
        pd->right_target = 0.0f;
        pd->steer        = 0.0f;
        pd->speed        = 0.0f;
        pd->line_lost    = 1U;
        pd->search_ms    = 0U;
        pd->search_dir   = 0;

        HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 20U);
        return LINE_PD_ERR_LOST;
    }
#endif

    {
        uint8_t is_corner = 0U;
        int8_t  evidence  = 0;
        uint8_t src       = 0U;   /* 1 = SIDE（直接看到哪半边全黑） 2 = FORK（另一条腿） */

        if (gray->scene == GRAY_SCENE_SIDE_L)
        {
            is_corner = 1U;
            evidence  = 1;       /* 左半边全黑 -> 向左转 */
            src       = 1U;
        }
        else if (gray->scene == GRAY_SCENE_SIDE_R)
        {
            is_corner = 1U;
            evidence  = -1;      /* 右半边全黑 -> 向右转 */
            src       = 1U;
        }
        else if (gray->scene == GRAY_SCENE_FORK)
        {
            is_corner = 1U;
            evidence  = (int8_t)gray->corner_dir;   /* 另一条腿所在的一侧 */
            src       = 2U;
        }
        else
        {
            is_corner = 0U;
        }

        /*
         * SIDE 只在横向误差足够大时才算角点证据。
         * 1cm 多的正常偏移也会让半边全黑，不设门限会被误当成弯。
         */
        if ((src == 1U) && (is_corner != 0U))
        {
            float pos = gray->position_cm;

            if ((pos > -LINE_CORNER_TRIGGER_ERR_CM) &&
                (pos <  LINE_CORNER_TRIGGER_ERR_CM))
            {
                /*
                 * 只取消"触发角点事件"，但【保留方向证据】。
                 *
                 * 实测：真 90° 角点处 SIDE 出现时 |e| 只有 1.2cm，
                 * 如果连方向一起丢掉，随后 ALL（全黑、e≈0）到来时
                 * 没有方向可用，车会按直道处理、全速冲出赛道。
                 */
                is_corner = 0U;
            }
        }

            /*
             * 方向锁存 + 优先级。
             *
             * 以前的写法是"第一帧证据说了算"，实测（logs/capture.log）
             * 会出现 SIDE_L（左半边全黑，明显该左转）却锁存了 cd=-1：
             * 因为先出现了一帧 FORK，它的"另一条腿在哪侧"先把方向占住了。
             *
             * 现在按证据强度分级：
             *   SIDE（哪半边全黑）= 1，最直接，可以纠正之前由 FORK 得到的方向
             *   FORK（另一条腿在哪侧）= 2，只在还没有任何方向时采纳
             * 一旦由 SIDE 锁存，后续 FORK 不会再改它 —— 仍然不会来回跳。
             */
            if (evidence != 0)
            {
                if (pd->corner_dir == 0)
                {
                    /* 还没有方向：直接采纳 */
                    pd->corner_dir           = evidence;
                    pd->corner_dir_src       = src;
                    pd->corner_dir_alt       = 0U;
                    pd->corner_dir_flip_tick = HAL_GetTick();
                }
                else if (evidence == pd->corner_dir)
                {
                    /* 同向：清反向计数；SIDE 可以升级 FORK 的来源等级 */
                    pd->corner_dir_alt = 0U;

                    if ((src == 1U) && (pd->corner_dir_src == 2U))
                    {
                        pd->corner_dir_src = 1U;
                    }
                }
                else if (pd->commit_active == 0U)
                {
                    /*
                     * 反向证据：迟滞。必须连续 N 拍，
                     * 且距上次锁定 >= HOLD_MS 才允许翻转。
                     * commit 期间一律禁止（否则承诺转向会中途改向）。
                     */
                    if (pd->corner_dir_alt < 0xFFU)
                    {
                        pd->corner_dir_alt++;
                    }

                    if ((pd->corner_dir_alt >= LINE_CORNER_DIR_FLIP_CYCLES) &&
                        ((HAL_GetTick() - pd->corner_dir_flip_tick) >=
                         LINE_CORNER_DIR_FLIP_HOLD_MS))
                    {
                        pd->corner_dir           = evidence;
                        pd->corner_dir_src       = src;
                        pd->corner_dir_alt       = 0U;
                        pd->corner_dir_flip_tick = HAL_GetTick();
                    }
                }

                /* 只要这一拍还有方向证据，就把"出弯后记忆"续上 */
                pd->corner_dir_hold = LINE_CORNER_DIR_HOLD_CYCLES;
            }

        if (is_corner != 0U)
        {
            if (pd->corner_confirm < 0xFFU)
            {
                pd->corner_confirm++;
            }

            /*
             * 新事件开始：先清掉上一段的方向，再采纳本帧证据。
             * （corner_dir_hold 不清 —— 它是给"出弯后立刻丢线"兜底用的）
             */
            if ((pd->corner_active == 0U) &&
                (pd->corner_confirm >= LINE_CORNER_CONFIRM_CYCLES))
            {
                /*
                 * 注意：这里【不再清零】corner_dir。
                 * 窄/短赛道的事件会被频繁重建，每次清零就等于每 200ms
                 * 重新投票一次方向，方向必然乱跳（实测 cd 0.19s 翻一次）。
                 * 现在沿用上一段方向，翻向交给下面的迟滞逻辑。
                 */
                pd->corner_dir_alt = 0U;
                pd->corner_acute   = 0U;
                pd->corner_active  = 1U;
                pd->corner_release = 0U;

                /* 方案 C：记下"入弯时的车头朝向"，后面按 yaw 增量做定角转弯 */
                pd->corner_yaw0    = pd->yaw_deg;
                pd->commit_active  = 0U;
                pd->commit_done    = 0U;
                pd->last_commit_turn = 0.0f;
            }


            /*
             * 锐角证据：FORK（两条腿同时可见）或 ALL（整片全黑）。
             * 出现过的角点事件，丢线后要做"承诺转向"，见 LINE_ACUTE_TURN_DEG。
             */
            if ((gray->scene == GRAY_SCENE_FORK) ||
                (gray->scene == GRAY_SCENE_ALL))
            {
                pd->corner_acute = 1U;
            }
        }
        else
        {
            pd->corner_confirm = 0U;
        }

        if (pd->corner_active != 0U)
        {
            if (gray->scene == GRAY_SCENE_LINE)
            {
                if (pd->corner_release < 0xFFU)
                {
                    pd->corner_release++;
                }
                if (pd->corner_release >= LINE_CORNER_RELEASE_CYCLES)
                {
                    /*
                     * 出弯：不立刻清方向。保留 LINE_CORNER_DIR_HOLD_CYCLES 拍
                     * （600ms），让"刚出弯就丢线"还能用上刚才的转弯方向；
                     * 真正的清除在下面的 hold 衰减里做。
                     */
                    pd->corner_active   = 0U;
                    pd->corner_release  = 0U;
                    pd->corner_dir_hold = LINE_CORNER_DIR_HOLD_CYCLES;
                }
            }
            else
            {
                pd->corner_release = 0U;
            }
        }
        else if (pd->corner_dir_hold > 0U)
        {
            /* 不在角点事件里：方向记忆按拍衰减，到 0 才真正清除 */
            pd->corner_dir_hold--;
            if (pd->corner_dir_hold == 0U)
            {
                pd->corner_dir     = 0;
                pd->corner_dir_src = 0U;
            }
        }
    }

    /* ---------- 2b. 取误差；角点内遇到 ALL 时用锁存方向兜底 ---------- */
    error_limit = LinePD_ErrorLimit();
    error = LinePD_Clamp(Gray_GetPositionCm(gray), error_limit);

    /*
     * B) 位置低通：丢线时直接把滤波器拉到当前值，避免拖尾
     *    （丢线时 Gray 给的是满幅回退值，平滑它没有意义）。
     */
    if (Gray_IsLineLost(gray) != 0U)
    {
        pd->pos_filter = error;
    }
    else
    {
        pd->pos_filter += LINE_POS_LPF_ALPHA * (error - pd->pos_filter);
    }
    error = pd->pos_filter;

    assist = 0U;
#if (LINE_ALL_AS_CORNER != 0)
    if (((pd->corner_active != 0U) || (pd->all_count >= LINE_ALL_ASSIST_CYCLES)) &&
        (gray->scene == GRAY_SCENE_ALL) &&
        (pd->corner_dir != 0))
    {
        /*
         * 8 路全黑时加权质心必然落在阵列正中，error ≈ 0，
         * PD 会输出 0 差速 —— 车直直地冲出弯心。
         * 此时位置信息已经没有意义，改用事件开始时锁存的方向满舵。
         */
        error  = (pd->corner_dir > 0) ? error_limit : -error_limit;
        assist = 1U;
    }
#endif
    pd->error = error;

    /*
     * A) 线角估计：θ = Δ位置 / Δ里程。
     *
     *   θ > 0   线朝左斜 -> 车要向左转（与 error 的符号约定一致）
     *   丢线/assist/车没动时不计，θ 慢慢回零
     *
     * 小角度近似 tanθ≈θ，整段不需要 atan，避免引入 libm。
     */
    {
        float dist_now = LinePD_RaceTravel(pd);

        if ((Gray_IsLineLost(gray) != 0U) || (assist != 0U))
        {
            pd->hist_count   = 0U;
            pd->last_dist_cm = dist_now;
            pd->last_pos_f   = pd->pos_filter;
            pd->theta_rad   += 0.5f * (0.0f - pd->theta_rad);
        }
        else
        {
            /*
             * 维护最近 3 个 (位置, 里程) 样本，用最小二乘斜率估线角。
             *
             * 两点差分（Δpos/Δs）本质是对含噪声的位置做微分：8 路量化 0.7cm，
             * 40ms 内只走 2cm，噪声能把 θ 顶到 ±0.3rad，前馈直接变成抖动源。
             * 用 3 点回归把基线拉长到 80ms/4cm，斜率噪声大约降到 1/3。
             */
            if (pd->hist_count < 3U)
            {
                pd->pos_hist[pd->hist_count]  = pd->pos_filter;
                pd->dist_hist[pd->hist_count] = dist_now;
                pd->hist_count++;
            }
            else
            {
                pd->pos_hist[0]  = pd->pos_hist[1];
                pd->pos_hist[1]  = pd->pos_hist[2];
                pd->dist_hist[0] = pd->dist_hist[1];
                pd->dist_hist[1] = pd->dist_hist[2];
                pd->pos_hist[2]  = pd->pos_filter;
                pd->dist_hist[2] = dist_now;
            }

            if (pd->hist_count >= 3U)
            {
                float sm = 0.0f, sp = 0.0f, ssd = 0.0f, spd = 0.0f;
                float ds_total = pd->dist_hist[2] - pd->dist_hist[0];
                uint8_t k;

                for (k = 0U; k < 3U; k++)
                {
                    sm  += pd->dist_hist[k];
                    sp  += pd->pos_hist[k];
                }
                sm /= 3.0f;
                sp /= 3.0f;

                for (k = 0U; k < 3U; k++)
                {
                    float dd = pd->dist_hist[k] - sm;
                    float dp = pd->pos_hist[k]  - sp;

                    ssd += dd * dd;
                    spd += dd * dp;
                }

                if ((ds_total > 1.0f) && (ssd > 0.01f))
                {
                    float th = spd / ssd;

                    th = LinePD_Clamp(th, LINE_THETA_MAX_RAD);

                    /* 死区：接近直线时不加前馈，避免放大噪声 */
                    if ((th > -LINE_THETA_DEADBAND) && (th < LINE_THETA_DEADBAND))
                    {
                        th = 0.0f;
                    }

                    pd->theta_rad += LINE_THETA_LPF * (th - pd->theta_rad);
                }
                else
                {
                    pd->theta_rad += 0.5f * (0.0f - pd->theta_rad);
                }
            }

            pd->last_dist_cm = dist_now;
            pd->last_pos_f   = pd->pos_filter;
        }
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

    /* ---------- 3b. 锐角"承诺转向"（角度制） ---------- */
    /*
     * 转到 LINE_ACUTE_TURN_DEG 再开始找线，而不是按固定时间。
     * 这几拍即使看到线也不认（否则会把刚走过的那条腿当成"找回来了"）。
     *
     * ⚠️ 必须放在第 3 步"丢线计数"之后：那一步在没丢线时会把 lost_count 清零，
     *    放前面会被它覆盖掉。
     */
    if (pd->commit_active != 0U)
    {
        float turned_deg;

        if (pd->commit_ms > LINE_PD_PERIOD_MS)
        {
            pd->commit_ms = (uint16_t)(pd->commit_ms - LINE_PD_PERIOD_MS);
        }
        else
        {
            pd->commit_ms = 0U;
        }

        /* 已经转过的角度（朝锁存方向为正） */
        turned_deg = (pd->yaw_deg - pd->corner_yaw0) * (float)pd->commit_dir;

        if ((turned_deg >= pd->commit_target_deg) || (pd->commit_ms == 0U))
        {
            /*
             * 转够了（或兜底超时）：结束承诺转向，把搜索计时清零，
             * 让后面的双向扫描重新获得完整的搜索时间。
             * commit_done 防止同一段搜索里再次触发承诺转向。
             */
            pd->commit_active = 0U;
            pd->commit_done   = 1U;
            pd->commit_dir    = 0;
            pd->search_ms     = 0U;
            /* 记录本次实际转过的角度，供 `[PD] ok` 输出、方便标定 */
            pd->last_commit_turn = turned_deg;
        }
        else
        {
            pd->lost_count    = (uint8_t)(LINE_LOST_TOLERANCE + 1U);
            pd->recover_count = 0U;
        }
    }

    /* ---------- 4. 微分项：带低通滤波，首拍/不可导的边沿不做微分 ---------- */
    if ((pd->first_run != 0U) || (assist != 0U) || (pd->all_assist != 0U))
    {
        /*
         * 三种情况本拍不做微分：
         *   首拍            —— 没有上一拍误差
         *   进入 assist     —— error 被强行拉到 ±error_limit，是阶跃
         *   退出 assist     —— error 从 ±error_limit 跳回实测值，也是阶跃
         * 不冻结的话，Kd 会在这个边沿上打出一个假的大差速，车身会抽一下。
         */
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
    pd->all_assist = assist;

    pd->d_filter_state += pd->d_filter * (derivative - pd->d_filter_state);
    derivative = pd->d_filter_state;

    /* ---------- 5. PD 运算 + 差速限幅 ---------- */
    {
        /*
         * Kp 随速度缩放：速度越高，同样误差对应的差速越小。
         * 不缩放的话 50cm/s 时 LKP=20 会形成 ~3Hz 极限环（见头文件说明）。
         */
        float kp_eff = pd->kp;

        if (pd->speed > LINE_KP_SPEED_REF_CMS)
        {
            float scale = LINE_KP_SPEED_REF_CMS / pd->speed;
            if (scale < LINE_KP_MIN_SCALE)
            {
                scale = LINE_KP_MIN_SCALE;
            }
            kp_eff *= scale;
        }

        steer = (kp_eff * error) + (pd->kd * derivative);

        /*
         * 线角前馈：只在线、不在角点事件、不在 assist 时叠加，
         * 避免直角/全黑时把角度估计的毛刺放大。
         */
        if ((assist == 0U) && (pd->all_assist == 0U) && (pd->corner_active == 0U))
        {
            float ff_target = LINE_THETA_FF_GAIN * pd->speed * pd->theta_rad;
            float ff_limit  = pd->diff_limit * LINE_THETA_FF_CLAMP_RATIO;
            float delta;

            ff_target = LinePD_Clamp(ff_target, ff_limit);

            /* 斜坡限幅：一拍最多变 LINE_THETA_FF_SLEW_CMS，杜绝"一拍打满" */
            delta = ff_target - pd->theta_ff;
            delta = LinePD_Clamp(delta, LINE_THETA_FF_SLEW_CMS);
            pd->theta_ff += delta;
        }
        else
        {
            /* 角点/assist 时前馈快速退回 0 */
            float delta = 0.0f - pd->theta_ff;

            delta = LinePD_Clamp(delta, LINE_THETA_FF_SLEW_CMS);
            pd->theta_ff += delta;
        }

        steer += pd->theta_ff;

        /*
         * A) yaw 阻尼：车身正在转（ω_yaw）就反向给差速，
         *    把"过弯后半段/出弯"的摆动压下去。
         *    只在正常循迹时生效，直角事件/assist 内不加，免得转向发肉。
         */
        if ((assist == 0U) && (pd->all_assist == 0U) &&
            (pd->corner_active == 0U) && (MPU6050_IsReady() != 0U))
        {
            float damp = LINE_YAW_DAMP_GAIN * MPU6050_GetYawRateDps();

            damp = LinePD_Clamp(damp, LINE_YAW_DAMP_MAX_CMS);
            steer -= damp;
        }
    }
    {
        /*
         * 角点事件期间放宽差速限幅：
         * 整半边全是线（或两条腿同时可见），说明车头已经严重偏出，
         * 需要比平时更大的差速才能把车身拧过来。
         *
         * 用事件状态（corner_active）而不是"本帧是不是 SIDE"来开关，
         * 这样巡航时偶尔一帧误分类不会把差速放大 35% 又立刻收回，
         * 转向输出在时间上是连续的。
         */
        float steer_limit = pd->diff_limit;

        if (pd->corner_active != 0U)
        {
            steer_limit *= LINE_CORNER_DIFF_BOOST;
        }

        steer = LinePD_Clamp(steer, steer_limit);
    }

    /*
     * 差速变化速率限制：只在正常循迹时生效。
     * assist（全黑满舵）和角点事件需要快速打满，跳过限制。
     */
    if ((assist == 0U) && (pd->all_assist == 0U) && (pd->corner_active == 0U))
    {
        float d_steer = steer - pd->steer;

        if (d_steer > LINE_STEER_SLEW_CMS)
        {
            steer = pd->steer + LINE_STEER_SLEW_CMS;
        }
        else if (d_steer < -LINE_STEER_SLEW_CMS)
        {
            steer = pd->steer - LINE_STEER_SLEW_CMS;
        }
        else
        {
            /* 在限制范围内 */
        }
    }

    pd->steer = steer;

    /* ---------- 6. 速度规划：直道加速 / 弧线按曲率减速 / 直角降速 ---------- */
    /*
     * 简单赛道（只有弧线和直角）的优化目标：
     *   直道尽量快、弯道按"曲率深度"提前减速、直角/锐角稳。
     *
     * 曲率深度由三部分组成：
     *   1) |error|    偏离越大越慢  —— 最直观的弯道深度
     *   2) |steer|    转向需求越大越慢 —— 差速会先于误差饱和，相当于提前量
     *   3) |d(error)| 误差变化率 —— 进弯前就开始减，不等误差长大
     *
     * 直道加速：连续若干拍"误差小 + 转向小 + 深度低"才逐步加，最多 +25%；
     * 一旦进弯立刻按 LINE_BOOST_DECAY_STEP 收回，避免把速度带进弯里。
     */
    {
        float e_norm = 0.0f;
        float s_norm = 0.0f;
        float d_norm;
        float depth;

        if (error_limit > 0.001f)
        {
            e_norm = (error < 0.0f) ? -error : error;
            e_norm /= error_limit;
            if (e_norm > 1.0f)
            {
                e_norm = 1.0f;
            }
        }

        if (pd->diff_limit > 0.001f)
        {
            s_norm = (steer < 0.0f) ? -steer : steer;
            s_norm /= pd->diff_limit;
            if (s_norm > 1.0f)
            {
                s_norm = 1.0f;
            }
        }

        d_norm = (derivative < 0.0f) ? -derivative : derivative;
        d_norm /= LINE_LEAD_DERR_MAX;
        if (d_norm > 1.0f)
        {
            d_norm = 1.0f;
        }

        depth = (LINE_DEPTH_W_ERR * e_norm) + (LINE_DEPTH_W_STEER * s_norm);
        if ((LINE_LEAD_WEIGHT * d_norm) > depth)
        {
            depth = LINE_LEAD_WEIGHT * d_norm;
        }
        if (depth > 1.0f)
        {
            depth = 1.0f;
        }

        slow_factor = 1.0f - (depth * (1.0f - LINE_CURVE_SLOW_MIN));

        /* 角点事件期间再压一档：直角/锐角宁可慢一点，也不要冲过头 */
        if ((pd->corner_active != 0U) && (slow_factor > LINE_CORNER_SLOW_MAX))
        {
            slow_factor = LINE_CORNER_SLOW_MAX;
        }

        /* 直道加速：仅在线、且三个量都小时才给 */
        if ((pd->corner_active == 0U) &&
            (((error < 0.0f) ? -error : error) < LINE_STRAIGHT_EPS_CM) &&
            (((steer < 0.0f) ? -steer : steer) < LINE_STRAIGHT_STEER_CM) &&
            (depth < 0.30f))
        {
            if (pd->straight_ticks < LINE_STRAIGHT_TICKS)
            {
                pd->straight_ticks++;
            }
            if (pd->straight_ticks >= LINE_STRAIGHT_TICKS)
            {
                pd->speed_boost += LINE_STRAIGHT_BOOST_STEP;
                if (pd->speed_boost > LINE_STRAIGHT_BOOST_MAX)
                {
                    pd->speed_boost = LINE_STRAIGHT_BOOST_MAX;
                }
            }
        }
        else
        {
            pd->straight_ticks = 0U;
            pd->speed_boost -= LINE_BOOST_DECAY_STEP;
            if (pd->speed_boost < 0.0f)
            {
                pd->speed_boost = 0.0f;
            }
        }

        slow_factor *= (1.0f + pd->speed_boost);

        /*
         * 丢线降速：进入搜索前（lost_count 1~容差）先减速，
         * 给车时间冲过断口/遮挡，也避免高速丢线直接甩出去。
         */
        if (pd->lost_count > 0U)
        {
            slow_factor *= LINE_LOST_SLOW_FACTOR;
        }

        /*
         * 基础速度带斜坡。
         *
         * 目标值 = base_speed × 速度系数（弯道减速 × 直道加速）；
         * 实际值 pd->speed 每拍最多变化一定量，避免阶跃。
         * 减速斜坡是加速的两倍：进弯要果断，出弯可以慢慢来。
         */
        {
            float target_speed = pd->base_speed * slow_factor;
            float dt_sec  = (float)LINE_PD_PERIOD_MS / 1000.0f;
            float step_up = LINE_SPEED_RAMP_CMS_PER_SEC * dt_sec;
            float step_dn = step_up * LINE_DOWN_RAMP_MULT;

            /* 自动停车接近段：限速，17cm 内要停得准 */
            if ((pd->race_phase == 2U) && (target_speed > LINE_RACE_APPROACH_CMS))
            {
                target_speed = LINE_RACE_APPROACH_CMS;
            }

            if (target_speed > pd->speed_limit)
            {
                target_speed = pd->speed_limit;
            }

            if (pd->speed < target_speed)
            {
                pd->speed += step_up;
                if (pd->speed > target_speed)
                {
                    pd->speed = target_speed;
                }
            }
            else if (pd->speed > target_speed)
            {
                pd->speed -= step_dn;
                if (pd->speed < target_speed)
                {
                    pd->speed = target_speed;
                }
            }
        }
    }

    /* ---------- 7. 丢线超时处理 ---------- */
    if (pd->lost_count > LINE_LOST_TOLERANCE)
    {
        pd->line_lost = 1U;

        /*
         * 【注意顺序：必须先判断"是否刚进入"，再累加计时】
         *
         * 之前的写法是"先累加 search_ms，再判断 search_ms==0"，
         * 结果 search_ms 在判断前就变成了 40，条件永远不成立，
         * 方向锁存代码从来没执行过 —— search_dir 一直是 0，
         * 于是每次都走进停车分支，表现为"一丢线就停"。
         */
        {
            uint8_t just_entered = (pd->search_ms == 0U) ? 1U : 0U;

        /* 累计搜索时间（外环节拍固定，累加即可） */
        if (pd->search_ms < 0xFFFFU)
        {
            pd->search_ms = (uint16_t)(pd->search_ms + LINE_PD_PERIOD_MS);
        }

#if (LINE_SEARCH_ENABLE != 0)
        /*
         * ---- 刚进入搜索：锁定方向 ----
         *
         * 只在刚进入的那一拍锁存，之后不再改变。
         * 这样即使斑块扫过、偶尔又看到一点线，也不会把方向翻掉。
         */
        if (just_entered != 0U)
        {
            /*
             * 搜索方向的选择，按可靠度排序：
             *
             *   1. 角点事件锁存的方向（corner_dir）
             *      进入角点事件时由 SIDE（哪半边全黑）或 FORK（另一条腿
             *      在哪一侧）直接观测得到，之后锁存不再改。
             *      它不受丢线前的最后位置被过冲污染的影响。
             *
             *   2. 最后有效位置的符号
             *      常规丢线（不在角点事件里）的依据。急弯过冲时可能被误导，
             *      但它是实测值，不是推断。
             *
             * 两者都失效时由双向扫描兜底。
             */
            int8_t dir = 0;
            float  last_cm  = gray->last_valid_mm * 0.1f;
            float  abs_last = (last_cm < 0.0f) ? -last_cm : last_cm;

            if (((pd->corner_active != 0U) || (pd->corner_dir_hold > 0U)) &&
                (pd->corner_dir != 0))
            {
                /*
                 * 来源1：角点事件（或刚出弯的记忆）锁存的方向。
                 *
                 * 【为什么不再和实测位置做"冲突检查"】
                 *   车在锐角弯转过头之后，实测位置会指回刚走过的那条腿；
                 *   按它修方向，车就顺着走过的赛道往回跑
                 *   （实车日志 17:36:54：cd=+1，丢线后却向右搜，随后沿旧线返回）。
                 *   角点方向来自 SIDE/FORK 的直接观测，这里信它。
                 */
                dir = pd->corner_dir;

                /*
                 * 锐角事件：进入搜索后先"闭眼"朝这个方向转到目标角度，
                 * 把回来那条腿扫过去，避免刚丢线就把它当成找回来了。
                 * commit_done 保证一次搜索里只做一次承诺转向。
                 */
                if ((pd->commit_done == 0U) &&
                    ((pd->corner_acute != 0U) ||
                     ((pd->corner_active != 0U) &&
                     (abs_last >= LINE_RIGHT_ANGLE_MIN_ERR_CM))))
                {
                    pd->commit_active = 1U;
                    pd->commit_dir    = pd->corner_dir;   /* 固定方向，防止中途被清 */

                    /*
                     * 锐角事件（FORK/ALL）：按 V 形角度转；
                     * 纯 SIDE 且误差已经拉大：按直角转。
                     */
                    pd->commit_target_deg = (pd->corner_acute != 0U)
                                            ? LINE_ACUTE_TURN_DEG
                                            : LINE_RIGHT_ANGLE_TURN_DEG;
                    pd->commit_ms = LINE_ACUTE_COMMIT_TIMEOUT_MS;
                }
            }
            else if (abs_last >= LINE_SEARCH_MIN_INFO_CM)
            {
                /* 来源2：最后一次有效位置，但要求偏离足够明显 */
                dir = (last_cm > 0.0f) ? 1 : -1;
            }
            else
            {
                /*
                 * 来源3：没有任何方向信息。
                 *
                 * 注意这里【不是放弃搜索】—— 双向扫描是系统性的，
                 * 会依次覆盖两个方向，起点方向不影响最终能否找到。
                 * 所以这里只决定"先扫哪边"，带一个明确的默认值，
                 * 而不是随机的或者隐含的。
                 */
                dir = (int8_t)LINE_SEARCH_DEFAULT_DIR;
            }

            pd->search_dir = dir;
        }

#if (LINE_SEARCH_BIDIRECTIONAL != 0)
        if ((pd->search_dir != 0) &&
            (pd->search_ms <= (uint16_t)(LINE_SEARCH_TIMEOUT_MS * 2U)))
#else
        if ((pd->search_dir != 0) &&
            (pd->search_ms <= LINE_SEARCH_TIMEOUT_MS))
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
        }

        /*
         * ---- 搜索超时（或搜索被禁用）：停车 ----
         *
         * 已经转了 LINE_SEARCH_TIMEOUT_MS 还没找到线，
         * 说明线真的不在附近了。继续转下去只会让车跑出赛道或撞墙。
         */
        {
            /*
             * ⚠️ 9600 波特率下，从控制任务里打印"[LinePD] line lost, stop"
             * （27 字节 ≈ 28ms）会吃掉 40ms 控制周期的一大半；
             * 而且 20ms 的发送超时会让这行在 "s" 处被截断。
             * 所以：
             *   ① 只在本次丢线的第一拍打印（lost_announced 标志）
             *   ② 消息压到 11 字节 ≈ 12ms
             *   ③ 超时给到 40ms，保证整行发完
             * 需要更详细的日志时，看 [L] 行或把 LINE_DEBUG_PRINT 打开。
             */
            if (pd->lost_announced == 0U)
            {
                const char *msg = "[PD] lost\r\n";

                pd->lost_announced = 1U;
                (void)HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                                        (uint16_t)strlen(msg), 40U);
            }

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
        /*
         * 如果刚才确实在搜索（search_ms > 0），说明是靠搜索找回来的。
         * 打印搜索用掉的毫秒数 —— 用它就能知道锐角弯实际需要转多少度，
         * 从而把 LINE_SEARCH_TIMEOUT_MS 定在有依据的数值上。
         *
         * 换算：转角 = search_ms / 1000 × 183 度
         */
        if (pd->search_ms > 0U)
        {
            char msg[40];
            int  len = snprintf(msg, sizeof(msg),
                                "[PD] ok %ums y%+d\r\n",
                                (unsigned int)pd->search_ms,
                                (int)pd->last_commit_turn);

            if (len > 0)
            {
                (void)HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                                        (uint16_t)len, 40U);
            }
        }

        pd->search_ms   = 0U;
        pd->search_dir  = 0;
        pd->recover_count = LINE_RECOVER_CYCLES;
        pd->lost_announced = 0U;
        pd->commit_active  = 0U;
        pd->commit_done    = 0U;
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


/* ------------------------------------------------------------------------- */
/* 比赛自动停车接口                                                            */
/* ------------------------------------------------------------------------- */

void LinePD_RaceStart(Line_PD_t *pd, float lap_cm)
{
    if (pd == NULL)
    {
        return;
    }

    /*
     * LAP 语义（OLED 步长是 10cm，所以自适应档留了一整段区间）：
     *   0        -> 关闭自动停车
     *   1~49     -> 自适应模式：只靠启停线识别，不需要周长
     *               （从 0 按一下就是 10，直接进入自适应）
     *   >= 50cm  -> 周长模式：识别为主，里程兜底
     */
    if (lap_cm <= 0.0f)
    {
        pd->race_enabled = 0U;
        pd->race_lap_cm  = 0.0f;
    }
    else if (lap_cm < 50.0f)
    {
        pd->race_enabled = 1U;
        pd->race_lap_cm  = 0.0f;     /* 自适应：没有周长 */
    }
    else
    {
        pd->race_enabled = 1U;
        pd->race_lap_cm  = lap_cm;
    }

    pd->race_start_cm  = LinePD_RaceTravel(pd);
    pd->race_detect_cm = 0.0f;
    pd->race_pulse     = 0U;
    pd->race_phase     = 0U;
}

void LinePD_RaceCancel(Line_PD_t *pd)
{
    if (pd == NULL)
    {
        return;
    }

    pd->race_enabled   = 0U;
    pd->race_lap_cm    = 0.0f;
    pd->race_phase     = 0U;
    pd->race_pulse     = 0U;
    pd->race_detect_cm = 0.0f;
}

float LinePD_RaceLearnedLapCm(void)
{
#if (LINE_RACE_LEARN_LAP != 0)
    return s_learned_lap_cm;
#else
    return 0.0f;
#endif
}

uint8_t LinePD_RaceDone(const Line_PD_t *pd)
{
    if (pd == NULL)
    {
        return 0U;
    }
    return ((pd->race_enabled != 0U) && (pd->race_phase == 3U)) ? 1U : 0U;
}

uint8_t LinePD_RacePhase(const Line_PD_t *pd)
{
    return (pd == NULL) ? 0U : pd->race_phase;
}
