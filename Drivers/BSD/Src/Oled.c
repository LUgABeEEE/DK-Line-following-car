//
// Created by JiangyaF on 2026/10/11.
//
// SSD1306 OLED 驱动实现（128x64，I2C）
//
// 设计要点：
//
//   1. 本地显存 + 整屏刷新。
//      直接写屏的话，画一屏要几千次 I2C 事务；本驱动在 RAM 里改显存，
//      最后 OLED_Refresh() 用 16 次事务推完（8 页 x (1 命令 + 1 数据)）。
//
//   2. 页寻址模式下，一页内的 128 列地址会自动回卷到 0。
//      所以刷新时不需要每字节重设列地址——但为了兼容性，每页仍然显式设一次。
//
//   3. 所有坐标变换（翻转）集中在 OLED_Tx/Ty 两个内联函数里，
//      绘制函数一律只认"逻辑坐标"，翻转功能就不会漏改某处。
//
//   4. 坐标越界一律静默忽略，不做断言。
//      因为显示是"辅助功能"，不该因为一个坐标算错就把整车搞崩。
//

#include "Oled.h"
#include "OledFont.h"
#include "Encoder.h"
#include "SpeedPID.h"
#include "LinePD.h"
#include "Gray.h"
#include "Ble.h"
#include <stdio.h>
#include <string.h>

/* I2C 句柄定义在 main.c 中 */
extern I2C_HandleTypeDef hi2c2;

/* 灰度传感器实例，定义在 main.c 中。只读它的位置/丢线状态 */
extern Gray_Sensor_t g_gray;

/* ------------------------------------------------------------------------- */
/* 显存                                                                        */
/* ------------------------------------------------------------------------- */

/*
 * 显存布局：[页][列]，共 8 页 x 128 列 = 1024 字节。
 * 页 p 的 bit0 是第 p*8 行的像素（上方），bit7 是第 p*8+7 行（下方）。
 */
static uint8_t s_gram[OLED_PAGE_COUNT][OLED_WIDTH];

/* 当前是否已做过 180 度翻转 */
static uint8_t s_flipped = 0U;

/* 是否处于反色显示 */
static uint8_t s_inverted = 0U;

/* ------------------------------------------------------------------------- */
/* 坐标变换                                                                    */
/* ------------------------------------------------------------------------- */

static uint8_t OLED_Tx(uint8_t x)
{
    return (s_flipped != 0U) ? (uint8_t)(OLED_WIDTH - 1U - x) : x;
}

static uint8_t OLED_Ty(uint8_t y)
{
    return (s_flipped != 0U) ? (uint8_t)(OLED_HEIGHT - 1U - y) : y;
}

/* ------------------------------------------------------------------------- */
/* 底层 I2C                                                                    */
/* ------------------------------------------------------------------------- */

/* 发送若干【命令】字节 */
static uint8_t OLED_WriteCmd(const uint8_t *cmds, uint16_t len)
{
    uint8_t buf[32];

    if ((cmds == NULL) || (len == 0U) || (len > (sizeof(buf) - 1U)))
    {
        return 0U;
    }

    buf[0] = 0x00;      /* 控制字节：后面是命令 */
    memcpy(&buf[1], cmds, len);

    return (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)(OLED_I2C_ADDR_7BIT << 1),
                                    buf, (uint16_t)(len + 1U),
                                    OLED_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

/* 发送单条【命令】 */
static uint8_t OLED_Cmd1(uint8_t cmd)
{
    return OLED_WriteCmd(&cmd, 1U);
}

/* 发送两字节命令（SSD1306 里很多命令需要一个参数） */
static uint8_t OLED_Cmd2(uint8_t cmd, uint8_t arg)
{
    uint8_t buf[2];

    buf[0] = cmd;
    buf[1] = arg;

    return OLED_WriteCmd(buf, 2U);
}

/*
 * 发送一页【数据】（128 字节）。
 * 控制字节 0x40 之后跟 128 字节显存，一次事务推完。
 */
static uint8_t OLED_WritePageData(const uint8_t *data, uint16_t len)
{
    uint8_t buf[OLED_WIDTH + 1U];

    if ((data == NULL) || (len == 0U) || (len > OLED_WIDTH))
    {
        return 0U;
    }

    buf[0] = 0x40;      /* 控制字节：后面是数据 */
    memcpy(&buf[1], data, len);

    return (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)(OLED_I2C_ADDR_7BIT << 1),
                                    buf, (uint16_t)(len + 1U),
                                    OLED_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

/* ------------------------------------------------------------------------- */
/* 初始化                                                                      */
/* ------------------------------------------------------------------------- */

uint8_t OLED_Init(void)
{
    /* SSD1306 上电后内部电荷泵需要稳定时间，手册要求约 100ms。
       这里给 100ms，避免刚上电就发命令被忽略。 */
    HAL_Delay(100U);

    /* 依次发送初始化命令。命令值来自厂商例程的 OLED_Init()，逐条核对过。 */
    if (OLED_Cmd1(0xAE) == 0U) { return 0U; }    /* 关显示（配置期间保持熄灭） */
    OLED_Cmd1(0x00);                             /* 列低 4 位地址 = 0            */
    OLED_Cmd1(0x10);                             /* 列高 4 位地址 = 0            */
    OLED_Cmd1(0x40);                             /* 显示起始行 = 0               */
    OLED_Cmd2(0x81, 0xCF);                       /* 对比度 = 0xCF                */
    OLED_Cmd1(0xA1);                             /* 段重映射：左右正常            */
    OLED_Cmd1(0xC8);                             /* 扫描方向：上下正常            */
    OLED_Cmd1(0xA6);                             /* 正常显示（非反色）            */
    OLED_Cmd2(0xA8, 0x3F);                       /* 多路复用比 = 1/64             */
    OLED_Cmd2(0xD3, 0x00);                       /* 显示偏移 = 0                  */
    OLED_Cmd2(0xD5, 0x80);                       /* 时钟分频 / 振荡频率           */
    OLED_Cmd2(0xD9, 0xF1);                       /* 预充电周期                    */
    OLED_Cmd2(0xDA, 0x12);                       /* COM 引脚硬件配置              */
    OLED_Cmd2(0xDB, 0x30);                       /* VCOMH 取消选择电平            */
    OLED_Cmd2(0x20, 0x02);                       /* 页寻址模式（0x02 = PAGE）     */
    OLED_Cmd2(0x8D, 0x14);                       /* 使能内部电荷泵                */

    OLED_Clear();

    if (OLED_Refresh() == 0U)
    {
        return 0U;      /* 清屏失败说明 I2C 不通，直接返回，不必再开显示 */
    }

    if (OLED_Cmd1(0xAF) == 0U) { return 0U; }    /* 开显示 */
    OLED_Cmd1(0xA4);                             /* 显示内容跟随显存（非全亮）    */

    s_flipped  = 0U;
    s_inverted = 0U;

    return 1U;
}

/* ------------------------------------------------------------------------- */
/* 显示控制                                                                    */
/* ------------------------------------------------------------------------- */

uint8_t OLED_DisplayOn(void)
{
    return OLED_Cmd1(0xAF);
}

uint8_t OLED_DisplayOff(void)
{
    return OLED_Cmd1(0xAE);
}

uint8_t OLED_SetContrast(uint8_t contrast)
{
    return OLED_Cmd2(0x81, contrast);
}

uint8_t OLED_SetInvert(uint8_t invert)
{
    s_inverted = (invert != 0U) ? 1U : 0U;
    return OLED_Cmd1((s_inverted != 0U) ? 0xA7 : 0xA6);
}

uint8_t OLED_SetFlip(uint8_t flip)
{
    s_flipped = (flip != 0U) ? 1U : 0U;

    /*
     * 翻转有两个层面，必须同时处理：
     *   1. 硬件层面：段重映射(0xA0/0xA1) 和 扫描方向(0xC0/0xC8)
     *   2. 软件层面：OLED_Tx/OLED_Ty 里的坐标镜像
     *
     * 只改硬件的话，显存里的内容方向对了但绘制坐标没变，结果还是反的；
     * 只改软件的话，绘制坐标对了但屏的扫描方向没变，也是反的。
     */
    if (s_flipped != 0U)
    {
        OLED_Cmd1(0xA0);
        OLED_Cmd1(0xC0);
    }
    else
    {
        OLED_Cmd1(0xA1);
        OLED_Cmd1(0xC8);
    }

    /* 方向变了，把已有内容重新推一遍 */
    return OLED_Refresh();
}

/* ------------------------------------------------------------------------- */
/* 显存操作                                                                    */
/* ------------------------------------------------------------------------- */

void OLED_Clear(void)
{
    memset(s_gram, 0x00, sizeof(s_gram));
}

uint8_t OLED_Refresh(void)
{
    uint8_t page;

    for (page = 0U; page < OLED_PAGE_COUNT; page++)
    {
        /*
         * 页寻址模式下必须先设页地址和列地址，再连续写 128 字节。
         * 列地址会自动递增并在 127 处回卷，所以 128 字节正好覆盖一整页。
         */
        if (OLED_Cmd1((uint8_t)(0xB0U + page)) == 0U) { return 0U; }  /* 页地址 */
        if (OLED_Cmd1(0x00) == 0U) { return 0U; }                     /* 列低 4 位 */
        if (OLED_Cmd1(0x10) == 0U) { return 0U; }                     /* 列高 4 位 */

        if (OLED_WritePageData(s_gram[page], OLED_WIDTH) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t color)
{
    uint8_t px;
    uint8_t py;
    uint8_t page;
    uint8_t bit;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;     /* 越界静默忽略 */
    }

    px   = OLED_Tx(x);
    py   = OLED_Ty(y);
    page = (uint8_t)(py / 8U);
    bit  = (uint8_t)(py % 8U);

    if (color != 0U)
    {
        s_gram[page][px] |= (uint8_t)(1U << bit);
    }
    else
    {
        s_gram[page][px] &= (uint8_t)(~(1U << bit));
    }
}

uint8_t OLED_GetPoint(uint8_t x, uint8_t y)
{
    uint8_t px;
    uint8_t py;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return 0U;
    }

    px = OLED_Tx(x);
    py = OLED_Ty(y);

    return (uint8_t)((s_gram[py / 8U][px] >> (py % 8U)) & 0x01U);
}

/* ------------------------------------------------------------------------- */
/* 图形                                                                        */
/* ------------------------------------------------------------------------- */

void OLED_DrawHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color)
{
    uint8_t i;

    for (i = 0U; i < w; i++)
    {
        OLED_DrawPoint((uint8_t)(x + i), y, color);
    }
}

void OLED_DrawVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color)
{
    uint8_t i;

    for (i = 0U; i < h; i++)
    {
        OLED_DrawPoint(x, (uint8_t)(y + i), color);
    }
}

void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t fill, uint8_t color)
{
    uint8_t i;

    if ((w == 0U) || (h == 0U))
    {
        return;
    }

    if (fill != 0U)
    {
        for (i = 0U; i < h; i++)
        {
            OLED_DrawHLine(x, (uint8_t)(y + i), w, color);
        }
        return;
    }

    OLED_DrawHLine(x, y, w, color);                          /* 上边 */
    OLED_DrawHLine(x, (uint8_t)(y + h - 1U), w, color);      /* 下边 */
    OLED_DrawVLine(x, y, h, color);                          /* 左边 */
    OLED_DrawVLine((uint8_t)(x + w - 1U), y, h, color);      /* 右边 */
}

void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
{
    /*
     * Bresenham 直线算法：只用整数加减和比较，不用浮点、不用除法。
     * 在 M3 上比"按斜率插值"快得多，也不会因为浮点误差画歪。
     */
    int16_t dx  = (int16_t)x1 - (int16_t)x0;
    int16_t dy  = (int16_t)y1 - (int16_t)y0;
    int16_t sx  = (dx >= 0) ? 1 : -1;
    int16_t sy  = (dy >= 0) ? 1 : -1;
    int16_t err;
    int16_t x = (int16_t)x0;
    int16_t y = (int16_t)y0;

    if (dx < 0) { dx = -dx; }
    if (dy < 0) { dy = -dy; }

    err = (int16_t)(dx - dy);

    for (;;)
    {
        OLED_DrawPoint((uint8_t)x, (uint8_t)y, color);

        if ((x == (int16_t)x1) && (y == (int16_t)y1))
        {
            break;
        }

        {
            int16_t e2 = (int16_t)(err * 2);

            if (e2 > -dy)
            {
                err -= dy;
                x += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y += sy;
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* 文字                                                                        */
/* ------------------------------------------------------------------------- */

uint8_t OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size, uint8_t color)
{
    uint8_t idx;
    uint8_t col;
    uint8_t width;

    /* 不可打印字符统一显示成空格，避免越界访问字体表 */
    if ((ch < 0x20) || (ch > 0x7E))
    {
        ch = ' ';
    }
    idx = (uint8_t)(ch - 0x20);

    if (size == OLED_FONT_8X16)
    {
        width = 8U;

        /*
         * 8x16 字体：每字符 16 字节。
         * 前 8 字节是上半页（对应 y ~ y+7），后 8 字节是下半页（y+8 ~ y+15）。
         * 字体的点阵本来就是"每列一字节、bit0 在上"，和显存的页内位序一致，
         * 所以可以直接按位或进去，不需要做位翻转。
         */
        for (col = 0U; col < 6U; col++)
        {
            uint8_t upper = oled_font_8x16[idx][col];
            uint8_t lower = oled_font_8x16[idx][col + 8U];
            uint8_t bit;

            for (bit = 0U; bit < 8U; bit++)
            {
                uint8_t on_up = (uint8_t)((upper >> bit) & 0x01U);
                uint8_t on_lo = (uint8_t)((lower >> bit) & 0x01U);

                /* 颜色为黑时反过来写，这样黑字可以"抠"在实心背景上 */
                OLED_DrawPoint((uint8_t)(x + col), (uint8_t)(y + bit),
                               (color != 0U) ? on_up : (uint8_t)(on_up ^ 0x01U));
                OLED_DrawPoint((uint8_t)(x + col), (uint8_t)(y + 8U + bit),
                               (color != 0U) ? on_lo : (uint8_t)(on_lo ^ 0x01U));
            }
        }
        return (uint8_t)(x + width);
    }

    /* 默认按 6x8 处理 */
    width = 6U;
    for (col = 0U; col < 6U; col++)
    {
        uint8_t data = oled_font_6x8[idx][col];
        uint8_t bit;

        for (bit = 0U; bit < 8U; bit++)
        {
            uint8_t on = (uint8_t)((data >> bit) & 0x01U);

            OLED_DrawPoint((uint8_t)(x + col), (uint8_t)(y + bit),
                           (color != 0U) ? on : (uint8_t)(on ^ 0x01U));
        }
    }

    return (uint8_t)(x + width);
}

uint8_t OLED_ShowString(uint8_t x, uint8_t y, const char *text, uint8_t size, uint8_t color)
{
    uint8_t cx = x;
    uint8_t cy = y;
    uint8_t width;
    uint8_t height;

    if (text == NULL)
    {
        return y;
    }

    width  = (size == OLED_FONT_8X16) ? 8U : 6U;
    height = (size == OLED_FONT_8X16) ? 16U : 8U;

    while (*text != '\0')
    {
        /* 自动换行：右边放不下就回到最左并下移一行 */
        if ((uint16_t)cx + width > OLED_WIDTH)
        {
            cx = 0U;
            cy = (uint8_t)(cy + height);
        }

        /* 底部放不下就停止，避免越界写 */
        if ((uint16_t)cy + height > OLED_HEIGHT)
        {
            break;
        }

        cx = OLED_ShowChar(cx, cy, *text, size, color);
        text++;
    }

    return cy;
}

uint8_t OLED_ShowNumber(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t size, uint8_t color)
{
    char    buf[16];
    int     n;

    /* 用 snprintf 统一处理符号位和补零，避免手写取模逻辑出错 */
    n = snprintf(buf, sizeof(buf), "%*ld", (int)len, (long)num);

    if (n <= 0)
    {
        return x;
    }

    return OLED_ShowString(x, y, buf, size, color);
}

uint8_t OLED_ShowFloat(uint8_t x, uint8_t y, float value, uint8_t decimals, uint8_t len,
                       uint8_t size, uint8_t color)
{
    char  buf[24];
    int   n = 0;
    long  scale = 1;
    uint8_t i;

    if (decimals > 4U)
    {
        decimals = 4U;      /* 再多就没有显示价值了，且容易溢出 long */
    }

    for (i = 0U; i < decimals; i++)
    {
        scale *= 10;
    }

    /*
     * 先按精度四舍五入到整数，再拆出整数部分和小数部分。
     * 直接对负数做 (int) 截断会得到错误结果（-30.567 会变成 -30.56 而不是 -30.57），
     * 所以这里有 +0.5 / -0.5 的处理。
     */
    {
        float scaled = value * (float)scale;

        scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;

        {
            long fixed = (long)scaled;      /* 已含小数位的整数表示 */
            long ipart = fixed / scale;
            long fpart = fixed % scale;

            if (fpart < 0)
            {
                fpart = -fpart;             /* 小数部分一律取正，符号由 ipart 带 */
            }

            if (decimals == 0U)
            {
                n = snprintf(buf, sizeof(buf), "%*ld", (int)len, ipart);
            }
            else
            {
                n = snprintf(buf, sizeof(buf), "%*ld.%0*ld",
                             (int)len, ipart, (int)decimals, fpart);
            }
        }
    }

    if (n <= 0)
    {
        return x;
    }

    return OLED_ShowString(x, y, buf, size, color);
}

/* ------------------------------------------------------------------------- */
/* 整车状态面板                                                                */
/* ------------------------------------------------------------------------- */

void OLED_ShowDashboard(const Line_PD_t *pd)
{
    char line[24];

    OLED_Clear();

    /* 行0：蓝牙连接状态 */
    (void)snprintf(line, sizeof(line), "BT:%-3s  %s",
                   (BLE_IsConnected() != 0U) ? "ON" : "OFF",
                   (Gray_IsLineLost(&g_gray) != 0U) ? "LINE LOST" : "tracking");
    OLED_ShowString(0U, 0U, line, OLED_FONT_6X8, OLED_WHITE);

    /* 行1：灰度位置 */
    (void)snprintf(line, sizeof(line), "pos:%+6.2fcm s:%4.1f",
                   (double)Gray_GetPositionCm(&g_gray),
                   (double)g_gray.strength);
    OLED_ShowString(0U, 8U, line, OLED_FONT_6X8, OLED_WHITE);

    if (pd != NULL)
    {
        /* 行2：循迹误差与差速量 */
        (void)snprintf(line, sizeof(line), "err:%+6.2f st:%+6.2f",
                       (double)pd->error, (double)pd->steer);
        OLED_ShowString(0U, 16U, line, OLED_FONT_6X8, OLED_WHITE);

        /* 行3、4：左右轮的目标速度与实测速度 */
        (void)snprintf(line, sizeof(line), "L t:%5.1f m:%5.1f",
                       (double)(pd->speed + pd->steer),
                       (double)Encoder_GetSpeed(MOTOR_A));
        OLED_ShowString(0U, 24U, line, OLED_FONT_6X8, OLED_WHITE);

        (void)snprintf(line, sizeof(line), "R t:%5.1f m:%5.1f",
                       (double)(pd->speed - pd->steer),
                       (double)Encoder_GetSpeed(MOTOR_B));
        OLED_ShowString(0U, 32U, line, OLED_FONT_6X8, OLED_WHITE);

        /* 行5：外环 PD 参数与基础速度 */
        (void)snprintf(line, sizeof(line), "PD kp%.0f kd%.1f b%.0f",
                       (double)pd->kp, (double)pd->kd, (double)pd->base_speed);
        OLED_ShowString(0U, 40U, line, OLED_FONT_6X8, OLED_WHITE);
    }
    else
    {
        OLED_ShowString(0U, 16U, "(no LinePD instance)", OLED_FONT_6X8, OLED_WHITE);
        OLED_ShowString(0U, 24U, "L m:", OLED_FONT_6X8, OLED_WHITE);
        OLED_ShowFloat(24U, 24U, Encoder_GetSpeed(MOTOR_A), 1U, 5U, OLED_FONT_6X8, OLED_WHITE);
        OLED_ShowString(0U, 32U, "R m:", OLED_FONT_6X8, OLED_WHITE);
        OLED_ShowFloat(24U, 32U, Encoder_GetSpeed(MOTOR_B), 1U, 5U, OLED_FONT_6X8, OLED_WHITE);
    }

    /* 行7：内环 PID 参数（从只读查询接口拿不到增益，这里显示编码器原始计数代替） */
    (void)snprintf(line, sizeof(line), "enc %5u / %5u",
                   (unsigned int)Encoder_GetCount(ENCODER_A),
                   (unsigned int)Encoder_GetCount(ENCODER_B));
    OLED_ShowString(0U, 56U, line, OLED_FONT_6X8, OLED_WHITE);

    (void)OLED_Refresh();
}

/* ------------------------------------------------------------------------- */
/* 自检                                                                        */
/* ------------------------------------------------------------------------- */

void OLED_TestSequence(void)
{
    uint8_t i;

    /* ---- 1. 全屏点亮：检查坏点和 I2C 是否通 ---- */
    memset(s_gram, 0xFF, sizeof(s_gram));
    if (OLED_Refresh() == 0U)
    {
        /* 屏没应答，后面就不用试了 */
        return;
    }
    HAL_Delay(1200U);

    /* ---- 2. 边框 + 对角线 + 中心十字：检查坐标方向 ---- */
    OLED_Clear();
    OLED_DrawRect(0U, 0U, OLED_WIDTH, OLED_HEIGHT, 0U, OLED_WHITE);
    OLED_DrawLine(0U, 0U, (uint8_t)(OLED_WIDTH - 1U), (uint8_t)(OLED_HEIGHT - 1U), OLED_WHITE);
    OLED_DrawLine((uint8_t)(OLED_WIDTH - 1U), 0U, 0U, (uint8_t)(OLED_HEIGHT - 1U), OLED_WHITE);
    OLED_DrawHLine(0U, (uint8_t)(OLED_HEIGHT / 2U), OLED_WIDTH, OLED_WHITE);
    OLED_DrawVLine((uint8_t)(OLED_WIDTH / 2U), 0U, OLED_HEIGHT, OLED_WHITE);
    OLED_Refresh();
    HAL_Delay(1500U);

    /* ---- 3. ASCII 字符表：检查字体数据 ---- */
    OLED_Clear();
    for (i = 0U; i < 40U; i++)
    {
        uint8_t col = (uint8_t)(i % 20U);
        uint8_t row = (uint8_t)(i / 20U);

        OLED_ShowChar((uint8_t)(col * 6U), (uint8_t)(row * 10U),
                      (char)(0x21 + i), OLED_FONT_6X8, OLED_WHITE);
    }
    OLED_ShowString(0U, 24U, "0.96 OLED 128x64", OLED_FONT_6X8, OLED_WHITE);
    OLED_ShowString(0U, 34U, "SSD1306 / I2C2", OLED_FONT_6X8, OLED_WHITE);
    OLED_ShowString(0U, 44U, "abcdefghijklm", OLED_FONT_6X8, OLED_WHITE);
    OLED_ShowString(0U, 54U, "NOPQRSTUVWXYZ", OLED_FONT_6X8, OLED_WHITE);
    OLED_Refresh();
    HAL_Delay(1800U);

    /* ---- 4. 两种字体 + 数字格式：检查大小字体和数值显示 ---- */
    OLED_Clear();
    OLED_ShowString(0U, 0U, "BIG 8x16 FONT", OLED_FONT_8X16, OLED_WHITE);
    OLED_ShowString(0U, 18U, "int:  ", OLED_FONT_6X8, OLED_WHITE);
    OLED_ShowNumber(42U, 18U, -1234, 6U, OLED_FONT_6X8, OLED_WHITE);
    OLED_ShowString(0U, 28U, "float:", OLED_FONT_6X8, OLED_WHITE);
    OLED_ShowFloat(42U, 28U, -30.567f, 2U, 6U, OLED_FONT_6X8, OLED_WHITE);
    OLED_ShowString(0U, 38U, "zero :", OLED_FONT_6X8, OLED_WHITE);
    OLED_ShowFloat(42U, 38U, 0.0f, 2U, 6U, OLED_FONT_6X8, OLED_WHITE);
    OLED_ShowString(0U, 48U, "pos  :", OLED_FONT_6X8, OLED_WHITE);
    OLED_ShowFloat(42U, 48U, 123.456f, 1U, 6U, OLED_FONT_6X8, OLED_WHITE);
    OLED_ShowString(0U, 56U, "test done", OLED_FONT_6X8, OLED_WHITE);
    OLED_Refresh();
    HAL_Delay(1500U);

    OLED_Clear();
    OLED_Refresh();
}

//
// Created by JiangyaF on 2026/10/11.
//
