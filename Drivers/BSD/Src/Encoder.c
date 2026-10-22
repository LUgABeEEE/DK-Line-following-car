//
// Created by JiangyaF on 2026/10/11.
//
// 增量式正交编码器读取实现
//
// 核心思路：
//   1. 编码器模式让硬件自动加减计数，软件只需读 CNT，不占用任何中断资源
//   2. CNT 是当前"位置"，不是"速度"，速度必须靠固定周期取差分得到
//   3. 16 位计数器会回绕，用 (int16_t)(now - last) 让回绕自动算对
//

#include "Encoder.h"
#include <stdio.h>
#include <string.h>

/* TIM3 / TIM4 句柄定义在 Core/Src/main.c 中 */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* 串口句柄，自检时用于打印。波特率与 MX_USART2_UART_Init() 保持一致 */
extern UART_HandleTypeDef huart2;

/* ------------------------------------------------------------------------- */
/* 硬件映射与运行状态                                                          */
/* ------------------------------------------------------------------------- */

typedef struct
{
    TIM_HandleTypeDef *htim;
} Encoder_Hw_t;

static const Encoder_Hw_t s_encoder[] =
{
    { &htim4 },     /* ENCODER_A : E1 = PB6/PB7 */
    { &htim3 },     /* ENCODER_B : E2 = PA6/PA7 */
};

#define ENCODER_COUNT   (sizeof(s_encoder) / sizeof(s_encoder[0]))

/* 每个编码器的运行状态 */
static uint16_t s_last_count[ENCODER_COUNT];    /* 上次采样时的 CNT            */
static uint32_t s_last_tick[ENCODER_COUNT];     /* 上次采样时的时间戳          */
static float    s_speed_cms[ENCODER_COUNT];     /* 最近一次算出的速度 cm/s     */
static float    s_distance_cm[ENCODER_COUNT];   /* 累计行驶距离 cm             */

/* ------------------------------------------------------------------------- */
/* 内部函数                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * 不用 RTOS 临界区，直接用开关全局中断：
 *   - 只要 2 条指令，无函数调用开销
 *   - 不依赖 FreeRTOS 是否已启动调度器，裸机阶段也能正常工作
 * 若以后改成多任务且担心影响实时性，可换成 taskENTER_CRITICAL() / taskEXIT_CRITICAL()。
 */
#define ENCODER_ENTER_CRITICAL()    __disable_irq()
#define ENCODER_EXIT_CRITICAL()     __enable_irq()

/*
 * 读取自上次调用以来的计数增量，带 16 位回绕处理。
 *
 * 回绕原理：两个 uint16_t 相减会整型提升成 int，再截断回 int16_t，
 * 跨零点时差值自动正确。例如 now=5、last=65530，结果就是 +11，
 * 完全不需要写 if 判断。
 */
static int16_t Encoder_ReadDelta(uint32_t encoder)
{
    int16_t delta;

    ENCODER_ENTER_CRITICAL();
    {
        uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(s_encoder[encoder].htim);
        delta = (int16_t)(now - s_last_count[encoder]);
        s_last_count[encoder] = now;
    }
    ENCODER_EXIT_CRITICAL();

    return delta;
}

/* 判断编号是否合法，避免越界访问静态数组。 */
static uint8_t Encoder_IsValid(uint32_t encoder)
{
    return (encoder < ENCODER_COUNT) ? 1U : 0U;
}

/* ------------------------------------------------------------------------- */
/* 对外接口                                                                    */
/* ------------------------------------------------------------------------- */

void Encoder_Init(void)
{
    uint32_t i;

    /* 两路都要启动，漏掉一路对应的轮子计数永远是 0 */
    for (i = 0U; i < ENCODER_COUNT; i++)
    {
        HAL_TIM_Encoder_Start(s_encoder[i].htim, TIM_CHANNEL_ALL);
    }

    Encoder_Reset();
}

void Encoder_Reset(void)
{
    uint32_t i;

    /* 先原子地把计数器清零并记录基准，保证 Reset 之后第一帧 delta 为 0 */
    ENCODER_ENTER_CRITICAL();
    for (i = 0U; i < ENCODER_COUNT; i++)
    {
        __HAL_TIM_SET_COUNTER(s_encoder[i].htim, 0U);
        s_last_count[i] = 0U;
    }
    ENCODER_EXIT_CRITICAL();

    /* 滤波器和里程状态不能在临界区里做浮点运算，放在外面 */
    for (i = 0U; i < ENCODER_COUNT; i++)
    {
        s_last_tick[i]   = HAL_GetTick();
        s_speed_cms[i]   = 0.0f;
        s_distance_cm[i] = 0.0f;
    }
}

uint16_t Encoder_GetCount(uint32_t encoder)
{
    if (Encoder_IsValid(encoder) == 0U)
    {
        return 0U;
    }
    return (uint16_t)__HAL_TIM_GET_COUNTER(s_encoder[encoder].htim);
}

int16_t Encoder_GetDelta(uint32_t encoder)
{
    if (Encoder_IsValid(encoder) == 0U)
    {
        return 0;
    }
    return Encoder_ReadDelta(encoder);
}

void Encoder_ResetFilter(uint32_t encoder)
{
    if (Encoder_IsValid(encoder) == 0U)
    {
        return;
    }
    s_speed_cms[encoder] = 0.0f;
}

void Encoder_UpdateSpeed(void)
{
    uint32_t i;
    uint32_t now_tick = HAL_GetTick();

    for (i = 0U; i < ENCODER_COUNT; i++)
    {
        int32_t  d;
        uint32_t dt_ms;
        float    raw_cms;

        /* ReadDelta 内部已关中断，这里不需要再保护 */
        d = (int32_t)Encoder_ReadDelta(i);

        /* 用实际经过的时间做分母，比假定固定周期更稳，也能自动适应调用抖动 */
        dt_ms = now_tick - s_last_tick[i];
        if (dt_ms == 0U)
        {
            dt_ms = ENCODER_PERIOD_MS;      /* 防止除零 */
        }
        s_last_tick[i] = now_tick;

        /* 距离：计数 -> mm -> cm */
        s_distance_cm[i] += (float)d * ENCODER_MM_PER_COUNT * 0.1f;

        /* 速度：单位 mm/ms = m/s，乘 100 得 cm/s */
        raw_cms = (float)d * ENCODER_MM_PER_COUNT / (float)dt_ms * 100.0f;

        /* 一阶低通滤波，抑制低速时的量化抖动 */
        s_speed_cms[i] += ENCODER_SPEED_FILTER * (raw_cms - s_speed_cms[i]);
    }
}

float Encoder_GetSpeed(uint32_t encoder)
{
    if (Encoder_IsValid(encoder) == 0U)
    {
        return 0.0f;
    }
    return s_speed_cms[encoder];
}

float Encoder_GetDistance(uint32_t encoder)
{
    if (Encoder_IsValid(encoder) == 0U)
    {
        return 0.0f;
    }
    return s_distance_cm[encoder];
}

void Encoder_TestSequence(void)
{
    uint16_t base[ENCODER_COUNT];       /* 各编码器的计数基准            */
    int32_t  total[ENCODER_COUNT];      /* 自检期间累计增量              */
    char     buf[160];                  /* 需容纳最长那行提示，留足余量  */
    uint32_t i;
    uint32_t t;

    /* 基准取 0，因为下面会把计数器真正清零，这样打印的 CNT 就是本段增量 */
    for (i = 0U; i < ENCODER_COUNT; i++)
    {
        __HAL_TIM_SET_COUNTER(s_encoder[i].htim, 0U);
        base[i]  = 0U;
        total[i] = 0;
    }

    {
        int len = snprintf(buf, sizeof(buf),
                           "\r\n[Encoder] self test start, rotate each wheel by hand\r\n"
                           "  1 wheel revolution should give %d counts\r\n",
                           (int)ENCODER_COUNTS_PER_REV);
        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
        }
    }

    for (t = 0U; t < 60U; t++)      /* 60 x 100ms = 6 秒 */
    {
        HAL_Delay(100U);

        for (i = 0U; i < ENCODER_COUNT; i++)
        {
            uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(s_encoder[i].htim);

            /* 用无符号相减再截断，保证回绕时结果依然正确 */
            total[i] += (int16_t)(now - base[i]);
            base[i]   = now;
        }

        {
            int len = snprintf(buf, sizeof(buf),
                               "E1(TIM4/A) CNT=%5u   E2(TIM3/B) CNT=%5u\r\n",
                               (unsigned int)Encoder_GetCount(ENCODER_A),
                               (unsigned int)Encoder_GetCount(ENCODER_B));
            if (len > 0)
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
            }
        }
    }

    {
        int len = snprintf(buf, sizeof(buf),
                           "[Encoder] total delta: E1(A)=%ld  E2(B)=%ld\r\n"
                           "  positive value = forward rotation, negative = reverse\r\n",
                           (long)total[ENCODER_A], (long)total[ENCODER_B]);
        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
        }
    }

    Encoder_Reset();
}

//
// Created by JiangyaF on 2026/10/11.
//
