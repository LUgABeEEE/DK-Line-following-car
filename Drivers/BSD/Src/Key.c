//
// Created by JiangyaF on 2026/10/11.
//
// 板载按键驱动实现
//
// 每个按键维护四个状态：
//   raw_level    : 最近一次读到的原始电平（已按极性翻成"1=按下"）
//   stable_state : 消抖后的稳定状态（对外生效的就是它）
//   counter      : 连续读到"与稳定态相反"的次数，达到阈值才翻转
//   press_start  : 按下的时间戳，用于长按判定和按住时长查询
//
// 事件产生条件（全部基于 stable_state 的变化，而不是原始电平）：
//   stable 由 0 变 1  -> PRESS，并记录时间戳
//   stable 保持 1 且时长跨过阈值且还没报过 -> LONG_PRESS（只报一次）
//   stable 由 1 变 0  -> RELEASE
//

#include "Key.h"
#include <stdio.h>
#include <string.h>

/* 串口句柄，自检时用于打印 */
extern UART_HandleTypeDef huart2;

/* ------------------------------------------------------------------------- */
/* 硬件映射                                                                    */
/* ------------------------------------------------------------------------- */

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} Key_Hw_t;

static const Key_Hw_t s_key_hw[KEY_COUNT] =
{
    { KEY1_GPIO_Port, KEY1_Pin },      /* KEY_1 : PB0 */
    { KEY2_GPIO_Port, KEY2_Pin },      /* KEY_2 : PA4 */
    { KEY3_GPIO_Port, KEY3_Pin },      /* KEY_3 : PA5 */
};

/* ------------------------------------------------------------------------- */
/* 运行状态                                                                    */
/* ------------------------------------------------------------------------- */

static uint8_t  s_stable[KEY_COUNT];        /* 消抖后的状态：1 = 按下        */
static uint8_t  s_counter[KEY_COUNT];       /* 连续反向读数计数              */
static uint8_t  s_long_reported[KEY_COUNT]; /* 本次按下是否已报过长按        */
static uint32_t s_press_start[KEY_COUNT];   /* 按下时刻的时间戳              */

/* 事件队列：用位掩码记录"哪些事件还没被取走"。
   两个按键不会在同一轮产生同一个事件槽位冲突，因为槽位是 事件号 + 按键号*4。 */
static uint8_t  s_event_pending[KEY_COUNT];  /* 每个按键待处理的事件类型，0 = 无 */
static Key_Callback_t s_callback = NULL;

/* ------------------------------------------------------------------------- */
/* 内部辅助                                                                    */
/* ------------------------------------------------------------------------- */

static uint8_t Key_IsValid(uint32_t key)
{
    return (key < KEY_COUNT) ? 1U : 0U;
}

/* 读一次引脚，并按极性翻译成"1 = 按下" */
static uint8_t Key_ReadPressed(uint32_t key)
{
    GPIO_PinState level = HAL_GPIO_ReadPin(s_key_hw[key].port, s_key_hw[key].pin);

#if (KEY_ACTIVE_LOW != 0)
    return (level == GPIO_PIN_RESET) ? 1U : 0U;
#else
    return (level == GPIO_PIN_SET) ? 1U : 0U;
#endif
}

/* 投递一个事件：写进队列，如果注册了回调就再调一次 */
static void Key_PushEvent(uint32_t key, uint8_t event)
{
    if (s_event_pending[key] == KEY_EVENT_NONE)
    {
        /* 上一轮的事件已经被取走了，可以直接放新的 */
        s_event_pending[key] = event;
    }
    else
    {
        /*
         * 上一轮的事件还没被取走（说明调用方没有及时 Key_GetEvent）。
         * 这里保留旧事件，避免覆盖——旧事件通常更重要（比如 PRESS 被
         * LONG_PRESS 顶掉就丢了"按下"这个事实）。
         * 正常情况下每个扫描周期都会把队列取空，不会走到这里。
         */
    }

    if (s_callback != NULL)
    {
        s_callback(key, event);
    }
}

/* ------------------------------------------------------------------------- */
/* 初始化                                                                      */
/* ------------------------------------------------------------------------- */

void Key_Init(void)
{
    uint32_t i;

    for (i = 0U; i < KEY_COUNT; i++)
    {
        /* 上电时直接采信当前电平作为初始稳定态，
           避免"上电瞬间按键恰好被按住"被误判成一次按下事件 */
        s_stable[i]        = Key_ReadPressed(i);
        s_counter[i]       = 0U;
        s_long_reported[i] = 0U;
        s_press_start[i]   = HAL_GetTick();

        if (s_stable[i] != 0U)
        {
            s_long_reported[i] = 1U;    /* 上电就按着，不当作有效长按 */
        }

        s_event_pending[i] = KEY_EVENT_NONE;
    }
}

/* ------------------------------------------------------------------------- */
/* 扫描：消抖 + 事件产生                                                       */
/* ------------------------------------------------------------------------- */

void Key_Scan(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t i;

    for (i = 0U; i < KEY_COUNT; i++)
    {
        uint8_t raw = Key_ReadPressed(i);

        /* ---------- 1. 消抖 ---------- */
        if (raw != s_stable[i])
        {
            if (s_counter[i] < 0xFFU)
            {
                s_counter[i]++;
            }

            if (s_counter[i] >= KEY_DEBOUNCE_SAMPLES)
            {
                /* 连续多次读到相反值，采信新状态 */
                s_stable[i]  = raw;
                s_counter[i] = 0U;

                /* ---------- 2. 状态真的翻转了，产生事件 ---------- */
                if (raw != 0U)
                {
                    s_press_start[i]   = now;
                    s_long_reported[i] = 0U;
                    Key_PushEvent(i, KEY_EVENT_PRESS);
                }
                else
                {
                    s_long_reported[i] = 0U;
                    Key_PushEvent(i, KEY_EVENT_RELEASE);
                }
            }
        }
        else
        {
            /* 读回原值，说明之前是抖动，计数清零重来 */
            s_counter[i] = 0U;
        }

        /* ---------- 3. 长按判定（和消抖分开，每轮都要检查） ---------- */
        if ((s_stable[i] != 0U) && (s_long_reported[i] == 0U))
        {
            if ((now - s_press_start[i]) >= KEY_LONG_PRESS_MS)
            {
                s_long_reported[i] = 1U;    /* 只报一次 */
                Key_PushEvent(i, KEY_EVENT_LONG_PRESS);
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* 事件读取                                                                    */
/* ------------------------------------------------------------------------- */

uint8_t Key_GetEvent(uint32_t *key, uint8_t *event)
{
    uint32_t i;

    for (i = 0U; i < KEY_COUNT; i++)
    {
        if (s_event_pending[i] != KEY_EVENT_NONE)
        {
            if (key != NULL)
            {
                *key = i;
            }
            if (event != NULL)
            {
                *event = s_event_pending[i];
            }

            s_event_pending[i] = KEY_EVENT_NONE;
            return 1U;
        }
    }

    return 0U;
}

void Key_ClearEvents(void)
{
    uint32_t i;

    for (i = 0U; i < KEY_COUNT; i++)
    {
        s_event_pending[i] = KEY_EVENT_NONE;
    }
}

void Key_RegisterCallback(Key_Callback_t callback)
{
    s_callback = callback;
}

/* ------------------------------------------------------------------------- */
/* 状态查询                                                                    */
/* ------------------------------------------------------------------------- */

uint8_t Key_IsPressed(uint32_t key)
{
    return (Key_IsValid(key) == 0U) ? 0U : s_stable[key];
}

uint32_t Key_GetHoldTime(uint32_t key)
{
    if (Key_IsValid(key) == 0U)
    {
        return 0U;
    }

    if (s_stable[key] == 0U)
    {
        return 0U;
    }

    return HAL_GetTick() - s_press_start[key];
}

/* ------------------------------------------------------------------------- */
/* 自检                                                                        */
/* ------------------------------------------------------------------------- */

void Key_TestSequence(void)
{
    static const char *key_name[KEY_COUNT] = { "KEY1(PB0)", "KEY2(PA4)", "KEY3(PA5)" };
    static const char *event_name[] = { "NONE", "PRESS", "LONG", "RELEASE" };

    char     buf[160];
    uint32_t t;
    uint32_t i;

    Key_Init();

    {
        const char *msg = "\r\n[Key] self test, press each key (short and long)\r\n"
                          "  columns: raw level per key | hold time | last event\r\n"
                          "  ---------------------------------------------------------\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 100U);
    }

    for (t = 0U; t < 1000U; t++)     /* 1000 x 20ms = 20 秒 */
    {
        uint32_t key;
        uint8_t  event;

        HAL_Delay(KEY_SCAN_PERIOD_MS);
        Key_Scan();

        /* 把本轮产生的事件打印出来 */
        while (Key_GetEvent(&key, &event) != 0U)
        {
            int len = snprintf(buf, sizeof(buf),
                               "  >>> EVENT: %s = %s\r\n",
                               key_name[key],
                               (event <= KEY_EVENT_RELEASE) ? event_name[event] : "?");
            if (len > 0)
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
            }
        }

        /* 每 5 轮（100ms）打印一次实时状态 */
        if ((t % 5U) == 0U)
        {
            int len = snprintf(buf, sizeof(buf), "  ");

            for (i = 0U; i < KEY_COUNT; i++)
            {
                len += snprintf(buf + len, sizeof(buf) - (size_t)len, "%s=%u(%4ums) ",
                                key_name[i],
                                (unsigned int)Key_IsPressed(i),
                                (unsigned int)Key_GetHoldTime(i));
            }

            if (len > 0)
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
                HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, 100U);
            }
        }
    }

    {
        const char *done = "[Key] test done\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)done, (uint16_t)strlen(done), 100U);
    }
}

//
// Created by JiangyaF on 2026/10/11.
//
