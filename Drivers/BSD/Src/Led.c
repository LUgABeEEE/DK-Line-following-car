//
// Created by JiangyaF on 2026/10/11.
//
// 板载 LED 驱动实现
//
// 设计要点：
//   1. 闪烁用 HAL_GetTick() 时间戳计算，不用 HAL_Delay——不阻塞主循环
//   2. 亮/灭/闪烁三种"意图"分开存：想点亮就无条件点亮，想闪烁才走时间逻辑
//   3. 极性用一个宏集中控制，硬件接法变了只改一处
//

#include "Led.h"

/* ------------------------------------------------------------------------- */
/* 硬件映射                                                                    */
/* ------------------------------------------------------------------------- */

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} Led_Hw_t;

static const Led_Hw_t s_led_hw[LED_COUNT] =
{
    { LED1_GPIO_Port, LED1_Pin },      /* LED_1 : PB5 */
    { LED2_GPIO_Port, LED2_Pin },      /* LED_2 : PB4 */
};

/* ------------------------------------------------------------------------- */
/* 运行状态                                                                    */
/* ------------------------------------------------------------------------- */

/* 0 = 常亮/常灭（由 on/off 决定），1 = 闪烁 */
static uint8_t  s_blinking[LED_COUNT];
static uint8_t  s_is_on[LED_COUNT];         /* 最近一次实际输出的状态       */
static uint32_t s_blink_last_ms[LED_COUNT]; /* 本次闪烁周期的起点时间戳     */
static uint32_t s_blink_on_ms[LED_COUNT];   /* 一个周期内"亮"持续多久       */
static uint32_t s_blink_period_ms[LED_COUNT];/* 完整周期长度                */
static uint32_t s_blink_remain[LED_COUNT];  /* 剩余闪烁次数，0 = 无限       */

/* ------------------------------------------------------------------------- */
/* 内部辅助                                                                    */
/* ------------------------------------------------------------------------- */

static uint8_t Led_IsValid(uint32_t led)
{
    return (led < LED_COUNT) ? 1U : 0U;
}

/* 按极性把"逻辑亮/灭"翻译成实际电平写出去 */
static void Led_Write(uint32_t led, uint8_t on)
{
#if (LED_ACTIVE_LOW != 0)
    GPIO_PinState level = (on != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET;
#else
    GPIO_PinState level = (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
#endif
    HAL_GPIO_WritePin(s_led_hw[led].port, s_led_hw[led].pin, level);
    s_is_on[led] = (on != 0U) ? 1U : 0U;
}

/* 把某个灯切回"非闪烁"模式。任何直接操作（On/Off/Toggle）都要先取消闪烁，
   否则闪烁逻辑在下一拍 Refresh 里会把状态又改回去，表现为"手动控制失灵"。 */
static void Led_CancelBlink(uint32_t led)
{
    s_blinking[led]       = 0U;
    s_blink_remain[led]   = 0U;
    s_blink_last_ms[led]  = HAL_GetTick();
}

/* ------------------------------------------------------------------------- */
/* 初始化                                                                      */
/* ------------------------------------------------------------------------- */

void Led_Init(void)
{
    uint32_t i;

    for (i = 0U; i < LED_COUNT; i++)
    {
        s_blinking[i]        = 0U;
        s_blink_remain[i]    = 0U;
        s_blink_on_ms[i]     = 0U;
        s_blink_period_ms[i] = 0U;
        s_blink_last_ms[i]   = HAL_GetTick();

        Led_Write(i, 0U);           /* 上电先全灭 */
    }
}

/* ------------------------------------------------------------------------- */
/* 基本操作                                                                    */
/* ------------------------------------------------------------------------- */

void Led_On(uint32_t led)
{
    if (Led_IsValid(led) == 0U)
    {
        return;
    }
    Led_CancelBlink(led);
    Led_Write(led, 1U);
}

void Led_Off(uint32_t led)
{
    if (Led_IsValid(led) == 0U)
    {
        return;
    }
    Led_CancelBlink(led);
    Led_Write(led, 0U);
}

void Led_Toggle(uint32_t led)
{
    if (Led_IsValid(led) == 0U)
    {
        return;
    }
    Led_CancelBlink(led);
    Led_Write(led, (s_is_on[led] != 0U) ? 0U : 1U);
}

uint8_t Led_IsOn(uint32_t led)
{
    return (Led_IsValid(led) == 0U) ? 0U : s_is_on[led];
}

void Led_AllOff(void)
{
    uint32_t i;

    for (i = 0U; i < LED_COUNT; i++)
    {
        Led_CancelBlink(i);
        Led_Write(i, 0U);
    }
}

/* ------------------------------------------------------------------------- */
/* 闪烁                                                                        */
/* ------------------------------------------------------------------------- */

void Led_BlinkCount(uint32_t led, uint32_t on_ms, uint32_t period_ms, uint32_t count)
{
    if (Led_IsValid(led) == 0U)
    {
        return;
    }

    /* 参数保护：周期必须大于 0，且亮的时间不能超过周期，否则会算出怪行为 */
    if (period_ms == 0U)
    {
        period_ms = LED_REFRESH_PERIOD_MS;
    }
    if (on_ms > period_ms)
    {
        on_ms = period_ms;
    }

    s_blink_on_ms[led]     = on_ms;
    s_blink_period_ms[led] = period_ms;
    s_blink_remain[led]    = count;
    s_blink_last_ms[led]   = HAL_GetTick();
    s_blinking[led]        = 1U;

    /* 立刻进入"亮"阶段，避免调用后要等一个周期才看到反应 */
    Led_Write(led, (on_ms > 0U) ? 1U : 0U);
}

void Led_BlinkForever(uint32_t led, uint32_t on_ms, uint32_t period_ms)
{
    Led_BlinkCount(led, on_ms, period_ms, 0U);
}

void Led_Refresh(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t i;

    for (i = 0U; i < LED_COUNT; i++)
    {
        uint32_t elapsed;

        if (s_blinking[i] == 0U)
        {
            continue;
        }

        elapsed = now - s_blink_last_ms[i];     /* 无符号相减，溢出自动正确 */

        if (elapsed >= s_blink_period_ms[i])
        {
            /* 一个完整周期结束 */
            s_blink_last_ms[i] = now;

            if (s_blink_remain[i] > 0U)
            {
                s_blink_remain[i]--;

                if (s_blink_remain[i] == 0U)
                {
                    /* 次数用完，停在"灭"上，并退出闪烁模式 */
                    s_blinking[i] = 0U;
                    Led_Write(i, 0U);
                    continue;
                }
            }

            elapsed = 0U;
        }

        /* 周期内：前 on_ms 亮，其余灭 */
        if (elapsed < s_blink_on_ms[i])
        {
            Led_Write(i, 1U);
        }
        else
        {
            Led_Write(i, 0U);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* 自检                                                                        */
/* ------------------------------------------------------------------------- */

void Led_TestSequence(void)
{
    Led_AllOff();

    /* 1. 单独点亮，逐个确认哪盏是哪盏 */
    Led_On(LED_1);
    HAL_Delay(1000U);
    Led_Off(LED_1);

    Led_On(LED_2);
    HAL_Delay(1000U);
    Led_Off(LED_2);

    /* 2. 交替闪 3 次：亮100ms 灭200ms */
    Led_BlinkCount(LED_1, 100U, 300U, 3U);
    Led_BlinkCount(LED_2, 100U, 300U, 3U);
    {
        uint32_t t;
        for (t = 0U; t < 50U; t++)      /* 50 x 20ms = 1 秒 */
        {
            HAL_Delay(LED_REFRESH_PERIOD_MS);
            Led_Refresh();
        }
    }

    /* 3. 两盏同步闪（用不同相位交叉，便于区分是否接反） */
    Led_BlinkForever(LED_1, 300U, 600U);
    {
        uint32_t t;
        for (t = 0U; t < 150U; t++)     /* 150 x 20ms = 3 秒 */
        {
            HAL_Delay(LED_REFRESH_PERIOD_MS);
            Led_Refresh();
        }
    }

    Led_AllOff();
}

//
// Created by JiangyaF on 2026/10/11.
//
