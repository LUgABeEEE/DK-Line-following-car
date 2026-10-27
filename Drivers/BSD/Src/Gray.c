//
// Created by JiangyaF on 2026/10/11.
//
// 幻尔 8 路 RGB 灰度巡线传感器驱动实现
//
// 解算思路：加权质心
//
//   每路的数字状态只有 0/1 两种取值，直接用会得到 8 个离散的误差值，
//   误差一跳变，PD 的微分项就会炸掉。所以这里用 16 位的灰度模拟值 +
//   模块自带的阈值做归一化，得到近似连续的误差，精度远高于 8 路本身。
//
//   归一化：(analog - threshold) / |threshold|
//           线比背景暗时该值为负，乘以 -1 让它变成正，这样"看到线的通道"
//           永远得到正贡献，与线条明暗无关，后面求质心就不必再区分黑白。
//

#include "Gray.h"
#include <stdio.h>
#include <string.h>

/* 串口句柄，自检时用于打印 */
extern UART_HandleTypeDef huart2;

/* ------------------------------------------------------------------------- */
/* 模块内部状态                                                                */
/* ------------------------------------------------------------------------- */

/*
 * 线条明暗关系。1 = 线比背景亮（黑底白线），0 = 线比背景暗（白底黑线）
 *
 * 【当前赛道：白底黑线 -> GRAY_LINE_IS_DARK】
 *
 * 注意：本模块的【模拟值方向】和手册描述相反！
 *   手册写"收到光越强、灰度值越高、越接近白色"，
 *   但实测（8 路模拟值）是：
 *     白色背景   -> A ≈ 2950（低）
 *     黑色线条   -> A ≈ 3900（高）
 *   也就是说这个模块输出的是"暗度"：越黑数值越大。
 *
 *   状态位 channel[i] 的判定也是 A > 阈值 时置 1，
 *   所以"线 = 数值高 = 状态位为 1"是自洽的。
 *
 * 【但位置解算已经不再依赖这个配置】
 *   第 5 步的质心计算现在用【状态位做门控 + |模拟值-阈值| 做权重】，
 *   取了绝对值，所以白底黑线 / 黑底白线都能正确工作。
 *   这个宏目前只影响自检打印里的文字描述。
 */
static uint8_t s_line_is_bright = GRAY_LINE_IS_DARK;

/* 上一次有效的线位置，用于丢线时保持方向 */
static float s_last_position_mm = 0.0f;

/* ------------------------------------------------------------------------- */
/* 底层 I2C 读写                                                               */
/* ------------------------------------------------------------------------- */

/*
 * 从模块的连续寄存器读 len 个字节。
 * 地址是 7 位，HAL 要求左移一位后传入。
 */
static uint8_t Gray_ReadRegisters(I2C_HandleTypeDef *hi2c, uint8_t reg,
                                 uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status;

    if ((hi2c == NULL) || (buf == NULL) || (len == 0U))
    {
        return 0U;
    }

    status = HAL_I2C_Mem_Read(hi2c,
                              (uint16_t)(GRAY_I2C_ADDR_7BIT << 1),
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              buf,
                              len,
                              GRAY_I2C_TIMEOUT_MS);

    return (status == HAL_OK) ? 1U : 0U;
}

/* 把 16 字节的"低字节在前"数据解析成 8 个 uint16 */
static void Gray_ParseU16Array(const uint8_t *raw, uint16_t *out)
{
    uint32_t i;

    for (i = 0U; i < GRAY_CHANNEL_COUNT; i++)
    {
        out[i] = (uint16_t)raw[i * 2U] | ((uint16_t)raw[i * 2U + 1U] << 8);
    }
}

/* ------------------------------------------------------------------------- */
/* 初始化                                                                      */
/* ------------------------------------------------------------------------- */

void Gray_Init(Gray_Sensor_t *sensor)
{
    if (sensor == NULL)
    {
        return;
    }

    memset(sensor, 0, sizeof(Gray_Sensor_t));
    s_last_position_mm = 0.0f;
}

void Gray_SetLinePolarity(uint8_t line_is_bright)
{
    s_line_is_bright = (line_is_bright != 0U) ? 1U : 0U;
}

uint8_t Gray_RefreshThreshold(Gray_Sensor_t *sensor, I2C_HandleTypeDef *hi2c)
{
    uint8_t raw[GRAY_CHANNEL_COUNT * 2U];

    if (sensor == NULL)
    {
        return 0U;
    }

    /* 阈值在寄存器 22~37 连续排列，一次读完 */
    if (Gray_ReadRegisters(hi2c, GRAY_REG_THRESHOLD_CH1, raw, sizeof(raw)) == 0U)
    {
        return 0U;
    }

    Gray_ParseU16Array(raw, sensor->threshold);
    sensor->threshold_loaded = 1U;

    return 1U;
}

/* ------------------------------------------------------------------------- */
/* 场景防抖                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * 场景族：把 6 个细分场景归成 3 类。
 *
 *   0 = 丢线 NONE
 *   1 = 正常单线 LINE
 *   2 = 角点 ALL / SIDE_L / SIDE_R / FORK
 *
 * 为什么防抖要以"族"为单位：
 *   直角（SIDE）和锐角（FORK）不是互斥类别，同一个锐角弯里
 *   SIDE / FORK 会逐帧交替出现。如果要求"同一个子类型连续 N 帧"，
 *   交替抖动时永远凑不够 N 帧，角点事件反而永远进不去 ——
 *   车会直直地冲过弯心。以族为单位确认，交替也能在 2 帧内进入角点。
 */
static uint8_t Gray_SceneFamily(uint8_t scene)
{
    if (scene == GRAY_SCENE_NONE)
    {
        return 0U;
    }
    if (scene == GRAY_SCENE_LINE)
    {
        return 1U;
    }
    return 2U;      /* ALL / SIDE_L / SIDE_R / FORK 都算角点 */
}

/*
 * 场景族切换需要连续确认多少帧。
 *
 *   - 任意 -> 丢线      : 立即生效（安全事件，晚 40ms 就可能冲出去）
 *   - 丢线 -> 任意      : 正常确认
 *   - 角点 -> 正常单线  : 多确认一次，防止弯心残留的宽黑带被判成"出弯"
 *   - 其它（单线 <-> 角点）: 正常确认
 */
static uint8_t Gray_SceneFamilyConfirmNeed(uint8_t prev_fam, uint8_t next_fam)
{
    if (next_fam == 0U)
    {
        return GRAY_SCENE_LOST_SAMPLES;
    }
    if (prev_fam == 0U)
    {
        return GRAY_SCENE_CONFIRM_SAMPLES;
    }
    if ((prev_fam == 2U) && (next_fam == 1U))
    {
        return GRAY_SCENE_RELEASE_SAMPLES;
    }
    return GRAY_SCENE_CONFIRM_SAMPLES;
}

/* ------------------------------------------------------------------------- */
/* 核心：读一次并解算                                                          */
/* ------------------------------------------------------------------------- */

uint8_t Gray_Update(Gray_Sensor_t *sensor, I2C_HandleTypeDef *hi2c)
{
    uint8_t raw_analog[GRAY_CHANNEL_COUNT * 2U];
    uint8_t raw_state = 0U;
    float   weighted = 0.0f;
    float   weight_sum = 0.0f;
    uint32_t i;

    if (sensor == NULL)
    {
        return 1U;
    }

    /* 累计计数：每次进入都算一次读取尝试，用于统计长期错误率 */
    sensor->total_reads++;

    /* ---------- 1. 读数字状态（寄存器 5，1 字节） ---------- */
    if (Gray_ReadRegisters(hi2c, GRAY_REG_STATE, &raw_state, 1U) == 0U)
    {
        if (sensor->fail_count < 0xFFFFU)
        {
            sensor->fail_count++;       /* 限幅，避免回绕成"假恢复" */
        }
        sensor->total_fails++;
        sensor->line_valid = 0U;
        return (sensor->fail_count >= GRAY_FAIL_LIMIT) ? 2U : 1U;
    }

    /* ---------- 2. 读 8 路模拟值（寄存器 6~21，16 字节，一次事务） ---------- */
    if (Gray_ReadRegisters(hi2c, GRAY_REG_ANALOG_CH1, raw_analog, sizeof(raw_analog)) == 0U)
    {
        if (sensor->fail_count < 0xFFFFU)
        {
            sensor->fail_count++;       /* 限幅，避免回绕成"假恢复" */
        }
        sensor->total_fails++;
        sensor->line_valid = 0U;
        return (sensor->fail_count >= GRAY_FAIL_LIMIT) ? 2U : 1U;
    }

    /* 通信成功，清除失败计数 */
    sensor->fail_count  = 0U;
    sensor->state_byte  = raw_state;
    sensor->state_mask  = raw_state;
    Gray_ParseU16Array(raw_analog, sensor->analog);

    /* ---------- 3. 首次运行时自动把阈值读回来 ---------- */
    if (sensor->threshold_loaded == 0U)
    {
        (void)Gray_RefreshThreshold(sensor, hi2c);
    }

    /* ---------- 4. 解析数字状态，并统计看到线的通道数 ---------- */
    sensor->active_count = 0U;
    for (i = 0U; i < GRAY_CHANNEL_COUNT; i++)
    {
        uint8_t on = (uint8_t)((raw_state >> i) & 0x01U);
        sensor->channel[i] = on;
        if (on != 0U)
        {
            sensor->active_count++;
        }
    }

    /* ---------- 4b. 启停线短脉冲检测 ---------- */
    /*
     * 5cm 横线：8 路同时黑 1~3 拍；角点/宽黑带持续更久。
     * 结果锁存在 stop_bar_seen，由 LinePD 消费（外环 40ms 可能整拍漏掉）。
     */
    if (sensor->active_count >= GRAY_STOP_BAR_CHANNELS)
    {
        if (sensor->stop_pulse < 0xFFU)
        {
            sensor->stop_pulse++;
        }
    }
    else if (sensor->stop_pulse > 0U)
    {
        if (sensor->stop_pulse <= GRAY_STOP_BAR_PULSE_MAX)
        {
            sensor->stop_bar_seen = 1U;
            sensor->stop_bar_len  = sensor->stop_pulse;
        }
        sensor->stop_pulse = 0U;
    }
    else
    {
        /* 保持 */
    }

    /* ---------- 5. 逐通道求权重，累加求加权质心 ---------- */
    /*
     * 权重的来源分两层：
     *
     *   【门控】用传感器自己的数字状态位 channel[i] 判断"这一路是否看到线"。
     *
     *     模块内部有完整的自学习阈值和滤波，它给出的状态位比我们
     *     拿模拟值和阈值硬比较要可靠得多（实测：5 个位置全部判断正确）。
     *     而且阈值放得偏高偏低时，模块自己仍然能正确判定。
     *
     *   【加权】用"模拟值偏离阈值多远"做亚像素权重。
     *
     *     线正压在某一路上时偏差大，压在两个通道中间时两边偏差都小，
     *     加权平均就能得到比 8 路更细的位置分辨率。
     *     取绝对值，这样白底黑线和黑底白线都能用，不必再区分极性。
     *
     * 之前用 (A-T)/T × 极性的做法有两个问题：
     *   1. 极性容易搞反（曾经反过一次，导致把背景当线）
     *   2. 阈值一旦偏得厉害，归一化值只剩 0.05 量级，
     *      两路加起来刚到丢线门限，稍微一动就误判丢线
     */
    /*
     * ---- 5a. 场景分类 ----
     *
     * 先找出所有"连续激活"的通道簇，再据此判断场景。
     * 全部依据可观测的通道状态，不做任何推断。
     */
    {
        uint8_t cl_first[GRAY_CHANNEL_COUNT];
        uint8_t cl_last[GRAY_CHANNEL_COUNT];
        uint8_t cl_count = 0U;
        uint8_t raw_scene = GRAY_SCENE_NONE;
        /* 分叉时选中的那条"腿"，用于禁用另一条腿的权重 */
        uint8_t gate_first = 0U;
        uint8_t gate_last  = 0U;
        uint8_t gate_valid = 0U;

        i = 0U;
        while (i < GRAY_CHANNEL_COUNT)
        {
            if (sensor->channel[i] != 0U)
            {
                uint8_t start = (uint8_t)i;

                while ((i < GRAY_CHANNEL_COUNT) && (sensor->channel[i] != 0U))
                {
                    i++;
                }
                if (cl_count < GRAY_CHANNEL_COUNT)
                {
                    cl_first[cl_count] = start;
                    cl_last[cl_count]  = (uint8_t)(i - 1U);
                    cl_count++;
                }
            }
            else
            {
                i++;
            }
        }

        /*
         * ---- 簇合并（间隔 ≤ GRAY_CLUSTER_MERGE_GAP）----
         *
         * 为什么必须合并：
         *   模块的状态位是逐路自学习阈值判出来的，一条连续的黑线在
         *   临界点上可能出现"亮-灭-亮"的单个空洞。如果不合并，
         *   这条线会被算成两个簇 —— 直接判成 FORK（锐角），
         *   而实际上只是阈值抖动。锐角判断"乱跳"的很大一部分来源就在这里。
         *
         *   真正的锐角弯，两条腿之间隔着大片白底（间隔远大于 1 路），
         *   不会被这条规则合并掉。
         */
        if (cl_count > 1U)
        {
            uint8_t m = 0U;
            uint8_t c;

            for (c = 0U; c < cl_count; c++)
            {
                if ((m > 0U) &&
                    ((int)(cl_first[c] - cl_last[m - 1U] - 1) <= (int)GRAY_CLUSTER_MERGE_GAP))
                {
                    cl_last[m - 1U] = cl_last[c];
                }
                else
                {
                    cl_first[m] = cl_first[c];
                    cl_last[m]  = cl_last[c];
                    m++;
                }
            }
            cl_count = m;
        }

        sensor->corner_dir    = 0;
        sensor->scene_left    = 0U;
        sensor->scene_right   = 0U;
        sensor->scene_clusters = cl_count;
        sensor->scene_span    = 0U;

        /* 最大簇的跨度 */
        {
            uint8_t c3;
            uint8_t max_span = 0U;

            for (c3 = 0U; c3 < cl_count; c3++)
            {
                uint8_t span = (uint8_t)(cl_last[c3] - cl_first[c3] + 1U);
                if (span > max_span)
                {
                    max_span = span;
                }
            }
            sensor->scene_span = max_span;
        }

        if (cl_count == 0U)
        {
            raw_scene = GRAY_SCENE_NONE;
        }
        else if (cl_count >= 2U)
        {
            /*
             * ---- 分叉 = 锐角弯 ----
             *
             * 两条腿同时可见。记录"另一条腿"在哪一侧 ——
             * 弯就往那一侧拐，丢线后搜索也用这个方向。
             *
             * 选簇规则：选离上一次位置最近的那条作为"当前腿"
             * （保持连续性，不被另一条腿的质心拉偏）。
             */
            float last_ch = (GRAY_CHANNEL_COUNT - 1U) * 0.5f;
            float sel_center;
            float other_center;
            float best_dist = 1.0e9f;
            uint8_t chosen  = 0U;
            uint8_t c;

            /* 上一次位置换算回通道下标 */
            if ((GRAY_CHANNEL_PITCH_MM > 0.001f) ||
                (GRAY_CHANNEL_PITCH_MM < -0.001f))
            {
                last_ch += s_last_position_mm
                           / (GRAY_CHANNEL_PITCH_MM * GRAY_DIR_SIGN);
            }

            for (c = 0U; c < cl_count; c++)
            {
                float center = ((float)cl_first[c] + (float)cl_last[c]) * 0.5f;
                float dist   = center - last_ch;

                if (dist < 0.0f)
                {
                    dist = -dist;
                }
                if (dist < best_dist)
                {
                    best_dist = dist;
                    chosen    = c;
                }
            }

            sel_center   = ((float)cl_first[chosen] + (float)cl_last[chosen]) * 0.5f;
            other_center = sel_center;
            best_dist    = 1.0e9f;

            for (c = 0U; c < cl_count; c++)
            {
                float center;
                float dist;

                if (c == chosen)
                {
                    continue;
                }
                center = ((float)cl_first[c] + (float)cl_last[c]) * 0.5f;
                dist   = center - sel_center;
                if (dist < 0.0f)
                {
                    dist = -dist;
                }
                if (dist < best_dist)
                {
                    best_dist    = dist;
                    other_center = center;
                }
            }

            /*
             * 弯的方向 = 从当前腿指向另一条腿的方向，用位置的正负约定。
             *   delta = (other - sel) × 间距 × DIR
             *   delta > 0 -> 另一条腿在正方向（左）
             */
            {
                float delta = (other_center - sel_center)
                              * GRAY_CHANNEL_PITCH_MM * GRAY_DIR_SIGN;

                if (delta > 0.01f)
                {
                    sensor->corner_dir = 1;
                }
                else if (delta < -0.01f)
                {
                    sensor->corner_dir = -1;
                }
            }

            raw_scene = GRAY_SCENE_FORK;

            /*
             * 记录选中的那条腿。真正的门控（把其余通道权重置 0）
             * 要等防抖结果出来以后再做 —— 只有防抖后的场景仍然是 FORK，
             * 才说明这两条腿稳定存在，这时才值得只跟一条腿。
             * 否则每帧都按瞬时分类门控一次，质心会在"两条腿中间"
             * 和"单条腿"之间来回跳，转向自然也跟着抖。
             */
            gate_first = cl_first[chosen];
            gate_last  = cl_last[chosen];
            gate_valid = 1U;
        }
        else
        {
            /* ---- 单簇：区分 LINE / ALL / SIDE ---- */
            uint8_t half     = (uint8_t)(GRAY_CHANNEL_COUNT / 2U);
            uint8_t left_on  = 0U;
            uint8_t right_on = 0U;
            uint8_t c;

            for (c = 0U; c < GRAY_CHANNEL_COUNT; c++)
            {
                if (sensor->channel[c] == 0U)
                {
                    continue;
                }

                /*
                 * CH1（下标0）在物理最左侧，所以下标 0~half-1 是左半边。
                 * 这里按【下标】分左右，和位置的正负号无关
                 * （GRAY_DIR_SIGN 影响位置符号，但不影响物理排布）。
                 */
                if (c < half)
                {
                    left_on++;
                }
                else
                {
                    right_on++;
                }
            }

            sensor->scene_left  = left_on;
            sensor->scene_right = right_on;

            if (sensor->active_count >= GRAY_ALL_MIN_CHANNELS)
            {
                /* 几乎全亮：横向黑线 / 终点标记 / 传感器悬空 */
                raw_scene = GRAY_SCENE_ALL;
            }
            else if ((left_on >= GRAY_SIDE_DOMINANT_MIN) &&
                     (right_on <= GRAY_SIDE_OTHER_MAX))
            {
                /* 左半边占优：直角弯向左 */
                raw_scene = GRAY_SCENE_SIDE_L;
            }
            else if ((right_on >= GRAY_SIDE_DOMINANT_MIN) &&
                     (left_on <= GRAY_SIDE_OTHER_MAX))
            {
                /* 右半边占优：直角弯向右 */
                raw_scene = GRAY_SCENE_SIDE_R;
            }
            else
            {
                raw_scene = GRAY_SCENE_LINE;
            }
        }

        /* ---- 5a-2. 场景防抖（见 Gray.h 的 GRAY_SCENE_*_SAMPLES 说明） ---- */
        sensor->scene_raw = raw_scene;

        {
            uint8_t raw_fam = Gray_SceneFamily(raw_scene);
            uint8_t st_fam  = Gray_SceneFamily(sensor->scene);

            if (sensor->scene_init == 0U)
            {
                /* 首次上电：直接采纳，不要求连续确认 */
                sensor->scene          = raw_scene;
                sensor->scene_fam_cand = raw_fam;
                sensor->scene_hold     = 0U;
                sensor->scene_sub_cand = raw_scene;
                sensor->scene_sub_hold = 0U;
                sensor->scene_init     = 1U;
            }
            else if (raw_fam == st_fam)
            {
                /* 场景族没变：清掉族切换计数 */
                sensor->scene_hold = 0U;

                if (raw_scene == sensor->scene)
                {
                    sensor->scene_sub_cand = raw_scene;
                    sensor->scene_sub_hold = 0U;
                }
                else if (st_fam == 2U)
                {
                    /*
                     * 角点内部换子类型（SIDE <-> FORK <-> ALL）：
                     * 必须连续 GRAY_SCENE_SUBTYPE_SAMPLES 帧都是同一个新子类型
                     * 才改。这样"一帧 SIDE、一帧 FORK"的抖动不会输出出去，
                     * 串口 sc 字段在锐角弯里也保持稳定。
                     */
                    if (raw_scene == sensor->scene_sub_cand)
                    {
                        if (sensor->scene_sub_hold < 0xFFU)
                        {
                            sensor->scene_sub_hold++;
                        }
                    }
                    else
                    {
                        sensor->scene_sub_cand = raw_scene;
                        sensor->scene_sub_hold = 1U;
                    }

                    if (sensor->scene_sub_hold >= GRAY_SCENE_SUBTYPE_SAMPLES)
                    {
                        sensor->scene_sub_hold = 0U;
                        sensor->scene          = raw_scene;
                        if (sensor->scene_flips < 0xFFFFU)
                        {
                            sensor->scene_flips++;
                        }
                    }
                }
                else
                {
                    /* 丢线族和单线族各只有一个成员，理论到不了这里 */
                    sensor->scene = raw_scene;
                }
            }
            else
            {
                uint8_t need;

                /* 累计候选场景族的连续帧数 */
                if (raw_fam == sensor->scene_fam_cand)
                {
                    if (sensor->scene_hold < 0xFFU)
                    {
                        sensor->scene_hold++;
                    }
                }
                else
                {
                    sensor->scene_fam_cand = raw_fam;
                    sensor->scene_hold     = 1U;
                }

                need = Gray_SceneFamilyConfirmNeed(st_fam, raw_fam);
                if (sensor->scene_hold >= need)
                {
                    /*
                     * 采用这一帧的瞬时分类作为新场景。
                     * 进入角点时，这一帧是 SIDE 还是 FORK 都无所谓 ——
                     * 外环只把它当"角点事件"，方向由各自的分支给出。
                     */
                    sensor->scene          = raw_scene;
                    sensor->scene_hold     = 0U;
                    sensor->scene_sub_cand = raw_scene;
                    sensor->scene_sub_hold = 0U;
                    if (sensor->scene_flips < 0xFFFFU)
                    {
                        sensor->scene_flips++;
                    }
                }
            }
        }

        /*
         * ---- 5a-3. 分叉门控（只在防抖后仍为 FORK 时生效）----
         *
         * 只跟选中的那条腿，避免质心落在两条腿中间。
         * 放到防抖之后做：瞬时分类偶尔出现双簇时不再直接改质心，
         * 否则位置解算会跟着场景一起跳。
         */
        if ((sensor->scene == GRAY_SCENE_FORK) && (gate_valid != 0U))
        {
            uint8_t c2;

            for (c2 = 0U; c2 < GRAY_CHANNEL_COUNT; c2++)
            {
                if ((c2 < gate_first) || (c2 > gate_last))
                {
                    sensor->channel[c2] = 0U;
                }
            }
        }
    }

    /* ---------- 5b. 加权质心 ---------- */
    for (i = 0U; i < GRAY_CHANNEL_COUNT; i++)
    {
        float v = 0.0f;
        float pos_mm;

        if (sensor->channel[i] != 0U)
        {
            float diff = (float)sensor->analog[i] - (float)sensor->threshold[i];

            /* 取绝对值：线比背景亮或暗都适用，不再依赖极性配置 */
            if (diff < 0.0f)
            {
                diff = -diff;
            }

            /*
             * 除以 1000 只是为了让 strength 的数值落在 0~3 这种好读的范围：
             * 实测偏差约 90~340，除以 1000 后单路权重 0.09~0.34，
             * 两路在线时总和约 0.4，远高于 GRAY_STRENGTH_MIN(0.10)。
             */
            v = diff / 1000.0f;
        }

        pos_mm  = ((float)i - (float)(GRAY_CHANNEL_COUNT - 1U) * 0.5f)
                  * GRAY_CHANNEL_PITCH_MM;
        pos_mm *= GRAY_DIR_SIGN;

        weighted   += v * pos_mm;
        weight_sum += v;
    }

    sensor->strength      = weight_sum;
    sensor->last_valid_mm = s_last_position_mm;

    /* ---------- 6. 判定位置有效性 / 丢线 ---------- */
    if (weight_sum < GRAY_STRENGTH_MIN)
    {
        /*
         * 所有通道都看不到线 = 丢线。
         * 此时质心没有意义，改为朝"最后一次偏离的方向"继续打满，
         * 让车有最大机会转回线上；如果之前是正的，就保持正方向。
         */
        sensor->line_lost   = 1U;
        sensor->line_valid  = 1U;
        sensor->position_mm = (s_last_position_mm >= 0.0f)
                              ? (GRAY_HALF_SPAN_MM * GRAY_LOST_ERROR_FACTOR)
                              : (-GRAY_HALF_SPAN_MM * GRAY_LOST_ERROR_FACTOR);
    }
    else
    {
        sensor->line_lost   = 0U;
        sensor->line_valid  = 1U;
        sensor->position_mm = weighted / weight_sum;
        s_last_position_mm  = sensor->position_mm;
    }

    sensor->position_cm = sensor->position_mm * 0.1f;

    return 0U;
}

/* ------------------------------------------------------------------------- */
/* 结果查询                                                                    */
/* ------------------------------------------------------------------------- */

float Gray_GetPositionCm(const Gray_Sensor_t *sensor)
{
    return (sensor == NULL) ? 0.0f : sensor->position_cm;
}

float Gray_GetPositionMm(const Gray_Sensor_t *sensor)
{
    return (sensor == NULL) ? 0.0f : sensor->position_mm;
}

uint8_t Gray_IsLineLost(const Gray_Sensor_t *sensor)
{
    return (sensor == NULL) ? 1U : sensor->line_lost;
}

uint8_t Gray_IsCentered(const Gray_Sensor_t *sensor, float tolerance_cm)
{
    if (sensor == NULL)
    {
        return 0U;
    }

    if (sensor->line_lost != 0U)
    {
        return 0U;
    }

    return ((sensor->position_cm <= tolerance_cm) &&
            (sensor->position_cm >= -tolerance_cm)) ? 1U : 0U;
}

/* ------------------------------------------------------------------------- */
/* 自检                                                                        */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* I2C 总线恢复                                                                */
/* ------------------------------------------------------------------------- */

void Gray_BusRecovery(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t i;

    if (hi2c == NULL)
    {
        return;
    }

    /*
     * 1. 释放 I2C 外设。
     *    DeInit 会把 PB8/PB9 从 AF 模式解绑（走 HAL_I2C_MspDeInit），
     *    同时关掉 I2C1 时钟。
     */
    (void)HAL_I2C_DeInit(hi2c);

    /*
     * 2. 把 SCL、SDA 都配成【开漏输出 + 上拉】的普通 GPIO。
     *    开漏是关键：这样我们既能主动拉低，也能在读的时候被
     *    外部上拉拉高，不会和从机抢总线。
     */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin   = GS_SCL_Pin | GS_SDA_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GS_SCL_GPIO_Port, &gpio);

    /* 3. 先都置高，让总线回到空闲电平 */
    HAL_GPIO_WritePin(GS_SCL_GPIO_Port, GS_SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GS_SDA_GPIO_Port, GS_SDA_Pin, GPIO_PIN_SET);
    HAL_Delay(2U);

    /*
     * 4. 打 9 个 SCL 时钟。
     *    从机若卡在某个数据位，最多 8 个时钟就能把它送完；
     *    第 9 个是保险，确保它进入"等待 STOP"的状态。
     */
    for (i = 0U; i < 9U; i++)
    {
        HAL_GPIO_WritePin(GS_SCL_GPIO_Port, GS_SCL_Pin, GPIO_PIN_RESET);
        HAL_Delay(1U);
        HAL_GPIO_WritePin(GS_SCL_GPIO_Port, GS_SCL_Pin, GPIO_PIN_SET);
        HAL_Delay(1U);
    }

    /*
     * 5. 手工产生 STOP 条件：SCL 为高时，SDA 由低变高。
     *    从机看到 STOP 就会释放 SDA 线。
     */
    HAL_GPIO_WritePin(GS_SCL_GPIO_Port, GS_SCL_Pin, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GS_SDA_GPIO_Port, GS_SDA_Pin, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GS_SCL_GPIO_Port, GS_SCL_Pin, GPIO_PIN_SET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GS_SDA_GPIO_Port, GS_SDA_Pin, GPIO_PIN_SET);
    HAL_Delay(1U);

    /*
     * 6. 重新初始化 I2C。
     *    HAL_I2C_Init 会调用 MspInit，把 PB8/PB9 重新配成 AF_OD，
     *    并打开 I2C1 时钟。
     */
    (void)HAL_I2C_Init(hi2c);
}

void Gray_TestSequence(Gray_Sensor_t *sensor, I2C_HandleTypeDef *hi2c)
{
    static const char *channel_name[GRAY_CHANNEL_COUNT] =
    {
        "CH1", "CH2", "CH3", "CH4", "CH5", "CH6", "CH7", "CH8"
    };

    char     buf[192];
    char     row1[192];
    char     row2[192];
    uint32_t i;
    uint32_t t;
    uint8_t  first_print = 1U;

    if (sensor == NULL)
    {
        return;
    }

    {
        int len = snprintf(buf, sizeof(buf),
                           "\r\n[Gray] sensor self test, move the sensor over the line\r\n"
                           "  sensor: 0x%02X  pitch: %.1fmm  dir: %+.1f  polarity: %s\r\n",
                           (unsigned int)GRAY_I2C_ADDR_7BIT,
                           (double)GRAY_CHANNEL_PITCH_MM,
                           (double)GRAY_DIR_SIGN,
                           (s_line_is_bright != 0U) ? "BRIGHT line" : "DARK line");
        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
        }
    }

    for (t = 0U; t < 30U; t++)      /* 30 x 200ms = 6 秒 */
    {
        uint8_t result;

        HAL_Delay(200U);

        result = Gray_Update(sensor, hi2c);

        if (result != 0U)
        {
            int len = snprintf(buf, sizeof(buf),
                               "  [%02u] I2C FAIL (result=%u, fail_count=%u)\r\n",
                               (unsigned int)t, (unsigned int)result,
                               (unsigned int)sensor->fail_count);
            if (len > 0)
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
            }
            continue;
        }

        /* 只在第一次成功读取时打印表头，避免刷屏 */
        if (first_print != 0U)
        {
            first_print = 0U;
            {
                int len = snprintf(buf, sizeof(buf),
                                   "  ------------- threshold -------------   "
                                   "------------- analog ----------------\r\n");
                if (len > 0)
                {
                    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
                }
            }
        }

        /* 拼两行：阈值行 + 模拟值行 */
        {
            uint16_t n1 = 0U;
            uint16_t n2 = 0U;

            n1 += (uint16_t)snprintf(row1 + n1, sizeof(row1) - n1, "  ");
            n2 += (uint16_t)snprintf(row2 + n2, sizeof(row2) - n2, "  ");

            for (i = 0U; i < GRAY_CHANNEL_COUNT; i++)
            {
                n1 += (uint16_t)snprintf(row1 + n1, sizeof(row1) - n1, "%5u ",
                                         (unsigned int)sensor->threshold[i]);
                n2 += (uint16_t)snprintf(row2 + n2, sizeof(row2) - n2, "%5u ",
                                         (unsigned int)sensor->analog[i]);
            }
        }

        {
            int len = snprintf(buf, sizeof(buf), "%s\r\n%s\r\n", row1, row2);
            if (len > 0)
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
            }
        }

        /* 状态位图 + 解算结果 */
        {
            char bits[GRAY_CHANNEL_COUNT + 1U];

            for (i = 0U; i < GRAY_CHANNEL_COUNT; i++)
            {
                bits[i] = (sensor->channel[i] != 0U) ? '1' : '0';
            }
            bits[GRAY_CHANNEL_COUNT] = '\0';

            {
                int len = snprintf(buf, sizeof(buf),
                                   "  state=0x%02X [%s]  active=%u  sum=%5.2f  pos=%+6.2fmm (%+5.2fcm) %s\r\n",
                                   (unsigned int)sensor->state_byte, bits,
                                   (unsigned int)sensor->active_count,
                                   (double)sensor->strength,
                                   (double)sensor->position_mm,
                                   (double)sensor->position_cm,
                                   (sensor->line_lost != 0U) ? "LINE LOST" : "");
                if (len > 0)
                {
                    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
                }
            }
        }
    }

    {
        /* 顺便把 channel_name 用上，打印一次映射关系便于对照 */
        int len = snprintf(buf, sizeof(buf), "  channel order: %s(左,最负) ... %s\r\n",
                           channel_name[0], channel_name[GRAY_CHANNEL_COUNT - 1U]);
        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
        }
    }

    {
        const char *done = "[Gray] test done\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)done, (uint16_t)strlen(done), 100U);
    }
}

//
// Created by JiangyaF on 2026/10/11.
//
