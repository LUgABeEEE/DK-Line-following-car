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

/* 线条明暗关系。1 = 线比背景亮（黑底白线），0 = 线比背景暗（白底黑线） */
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
/* 核心：读一次并解算                                                          */
/* ------------------------------------------------------------------------- */

uint8_t Gray_Update(Gray_Sensor_t *sensor, I2C_HandleTypeDef *hi2c)
{
    uint8_t raw_analog[GRAY_CHANNEL_COUNT * 2U];
    uint8_t raw_state = 0U;
    float   weighted = 0.0f;
    float   weight_sum = 0.0f;
    float   polarity;
    uint32_t i;

    if (sensor == NULL)
    {
        return 1U;
    }

    /* ---------- 1. 读数字状态（寄存器 5，1 字节） ---------- */
    if (Gray_ReadRegisters(hi2c, GRAY_REG_STATE, &raw_state, 1U) == 0U)
    {
        sensor->fail_count++;
        sensor->line_valid = 0U;
        return (sensor->fail_count >= GRAY_FAIL_LIMIT) ? 2U : 1U;
    }

    /* ---------- 2. 读 8 路模拟值（寄存器 6~21，16 字节，一次事务） ---------- */
    if (Gray_ReadRegisters(hi2c, GRAY_REG_ANALOG_CH1, raw_analog, sizeof(raw_analog)) == 0U)
    {
        sensor->fail_count++;
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

    /* ---------- 5. 逐个通道归一化并累加，求加权质心 ---------- */
    polarity  = (s_line_is_bright != 0U) ? -1.0f : 1.0f;

    for (i = 0U; i < GRAY_CHANNEL_COUNT; i++)
    {
        float v = 0.0f;
        float pos_mm;

        if (sensor->threshold_loaded != 0U)
        {
            float threshold = (float)sensor->threshold[i];
            float denominator;

            if (threshold < 1.0f)
            {
                threshold = 1.0f;       /* 防止除零 */
            }

            /*
             * 分母取绝对值，这样无论线条比背景暗还是亮，
             * 归一化结果都被统一到"看到线 -> 正值"这个约定上。
             */
            denominator = (threshold > 0.0f) ? threshold : 1.0f;
            v = ((float)sensor->analog[i] - threshold) / denominator * polarity;

            /* 置负：没看到线的通道不参与质心，避免背景噪声把位置拉偏 */
            if (v < 0.0f)
            {
                v = 0.0f;
            }
        }

        pos_mm  = ((float)i - (float)(GRAY_CHANNEL_COUNT - 1U) * 0.5f) * GRAY_CHANNEL_PITCH_MM;
        pos_mm *= GRAY_DIR_SIGN;

        weighted   += v * pos_mm;
        weight_sum += v;
    }

    sensor->strength = weight_sum;

    /* ---------- 6. 判定位置有效性 / 丢线 ---------- */
    if (weight_sum < 0.5f)
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
