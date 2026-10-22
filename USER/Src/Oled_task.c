//
// Created by JiangyaF on 2026/10/22.
//
// OLED 显示任务实现
//
// 核心：用计数信号量累计按键事件，防止连按丢包。
//
// 为什么需要两个信号量：
//   一个计数信号量只能表达"有几件事发生了"，不能表达"是加还是减"。
//   如果用单个信号量，任务取走 3 次计数时无法知道这 3 次里
//   是 KEY1 按了 2 次还是 KEY2 按了 2 次，方向就错了。
//   所以 KEY1、KEY2 各用一个计数信号量，分别累计各自的按键次数。
//
// 计数信号量 vs 二值信号量：
//   二值信号量最大计数是 1。100ms 内连按 3 次 KEY1，二值信号量
//   只会留下 1 个"有事件"的标志，另外 2 次丢失。
//   计数信号量的计数可以到 OLED_KEY_SEM_MAX_COUNT(16)，每次 Give
//   都会把计数 +1，任务一次 Take 循环把计数取空，就能一次累加多次。
//
// 取空信号量的写法：
//   while (xSemaphoreTake(sem, 0) == pdTRUE) { count++; }
//   这里用 0 超时（不阻塞），把当前所有待处理事件一次性取完。
//

#include "Oled_task.h"
#include "Oled.h"
#include "Key.h"
#include "Led.h"
/*
 * 信号量 API 在 semphr.h 里，FreeRTOS.h 只包含基础类型和配置。
 * 不包含它会导致 SemaphoreHandle_t 未定义、xSemaphoreCreateCounting
 * 和 xSemaphoreGive 隐式声明（C89 里会当作 int 返回值，链接时失败）。
 */
#include "semphr.h"
#include <stdio.h>
#include <string.h>

/* 串口句柄定义在 main.c 中，用于调试打印 */
extern UART_HandleTypeDef huart2;

/* ------------------------------------------------------------------------- */
/* 任务内部状态                                                                */
/* ------------------------------------------------------------------------- */

static TaskHandle_t s_task_handle = NULL;

/* KEY1 / KEY2 各一个计数信号量 */
static SemaphoreHandle_t s_sem_key1 = NULL;
static SemaphoreHandle_t s_sem_key2 = NULL;

/* 当前的计数值 */
static volatile int32_t s_counter = 0;

/* 累计接受过的按键次数（用于确认没有丢包） */
static volatile uint32_t s_key1_total = 0U;
static volatile uint32_t s_key2_total = 0U;

/* ------------------------------------------------------------------------- */
/* 内部辅助                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * 把信号量里的待处理事件全部取走，返回取到的次数。
 * 超时传 0，表示"有多少拿多少，不等待"。
 */
static uint32_t OledTask_DrainSemaphore(SemaphoreHandle_t sem)
{
    uint32_t count = 0U;

    if (sem == NULL)
    {
        return 0U;
    }

    while (xSemaphoreTake(sem, 0) == pdTRUE)
    {
        count++;

        /* 防御：正常情况下信号量计数上限是 16，不会无限循环 */
        if (count > OLED_KEY_SEM_MAX_COUNT)
        {
            break;
        }
    }

    return count;
}

/* 限制计数值在显示范围内 */
static int32_t OledTask_Clamp(int32_t value)
{
    if (value > OLED_COUNTER_MAX)
    {
        return OLED_COUNTER_MAX;
    }
    if (value < OLED_COUNTER_MIN)
    {
        return OLED_COUNTER_MIN;
    }
    return value;
}

/* ------------------------------------------------------------------------- */
/* 按键上报接口（由控制任务调用）                                              */
/* ------------------------------------------------------------------------- */

uint8_t OledTask_NotifyKeyPress(uint32_t key)
{
    BaseType_t ret = pdFAIL;

    if (key == KEY_1)
    {
        if (s_sem_key1 != NULL)
        {
            ret = xSemaphoreGive(s_sem_key1);
            if (ret == pdTRUE)
            {
                s_key1_total++;
            }
        }
    }
    else if (key == KEY_2)
    {
        if (s_sem_key2 != NULL)
        {
            ret = xSemaphoreGive(s_sem_key2);
            if (ret == pdTRUE)
            {
                s_key2_total++;
            }
        }
    }
    else
    {
        return 0U;      /* 不是我们关心的键 */
    }

    /*
     * 返回 0 表示信号量已满（计数到达 OLED_KEY_SEM_MAX_COUNT）。
     * 这不是故障——说明两次显示刷新之间按了超过 16 次，已经超出人类极限，
     * 但保留返回值便于调试时发现异常。
     */
    return (ret == pdTRUE) ? 1U : 0U;
}

int32_t OledTask_GetCounter(void)
{
    return s_counter;
}

/* ------------------------------------------------------------------------- */
/* OLED 绘制                                                                   */
/* ------------------------------------------------------------------------- */

static void OledTask_Draw(void)
{
    char line[32];

    OLED_Clear();

    /* 第一行：标题（字母） */
    OLED_ShowString(0U, 0U, "OLED TASK", OLED_FONT_8X16, OLED_WHITE);

    /* 第二行：大字计数值（数字） */
    (void)snprintf(line, sizeof(line), "%ld", (long)s_counter);
    OLED_ShowString(0U, 18U, line, OLED_FONT_8X16, OLED_WHITE);

    /* 第三行：按键累计次数（字母 + 数字） */
    (void)snprintf(line, sizeof(line), "K1:%lu K2:%lu",
                   (unsigned long)s_key1_total,
                   (unsigned long)s_key2_total);
    OLED_ShowString(0U, 38U, line, OLED_FONT_6X8, OLED_WHITE);

    /* 第四行：操作提示（字母） */
    OLED_ShowString(0U, 48U, "K1:+  K2:-", OLED_FONT_6X8, OLED_WHITE);

    /*
     * 第五行：I2C 异步传输诊断（ok/timeout/busy）。
     *
     * 这是判断 DMA 是否真正工作的直接证据：
     *   I2C 1/0/0   -> 1 次传输成功，无超时无忙 -> 中断模式正常
     *   I2C 1/1/0   -> 有 1 次超时 -> 回调/中断链有问题
     *   I2C 0/0/0   -> 计数器没动 -> 走的是轮询模式
     */
    {
        uint32_t dma_ok = 0U, dma_to = 0U, dma_busy = 0U;

        OLED_GetAsyncStats(&dma_ok, &dma_to, &dma_busy);
        (void)snprintf(line, sizeof(line), "I2C %lu/%lu/%lu",
                       (unsigned long)dma_ok,
                       (unsigned long)dma_to,
                       (unsigned long)dma_busy);
        OLED_ShowString(0U, 56U, line, OLED_FONT_6X8, OLED_WHITE);
    }

    (void)OLED_Refresh();
}

/* ------------------------------------------------------------------------- */
/* 任务主体                                                                    */
/* ------------------------------------------------------------------------- */

static void OledTask_Main(void *argument)
{
    uint32_t k1_count;
    uint32_t k2_count;
    int32_t  old_counter;

    (void)argument;

    /* 等 OLED 和按键初始化稳定 */
    vTaskDelay(pdMS_TO_TICKS(200U));

    /*
     * 现在调度器已经跑起来了，可以安全启用异步传输（I2C 中断 / DMA）。
     *
     * 之前 OLED_Init() 在调度器启动前调用，那时只能用轮询
     * （阻塞式信号量在调度器启动前会卡死整个系统）。
     * 从这里开始，OLED 的 I2C 传输走中断方式，发送期间任务阻塞、
     * CPU 让给控制任务和蓝牙任务。
     */
    OLED_EnableAsync();

    /* 首帧 */
    s_counter = 0;
    OledTask_Draw();

    for (;;)
    {
        old_counter = s_counter;

        /* ---- 1. 把两个信号量里的待处理按键全部取走 ---- */
        k1_count = OledTask_DrainSemaphore(s_sem_key1);
        k2_count = OledTask_DrainSemaphore(s_sem_key2);

        /* ---- 2. 更新计数值 ---- */
        if ((k1_count != 0U) || (k2_count != 0U))
        {
            s_counter += (int32_t)k1_count;
            s_counter -= (int32_t)k2_count;
            s_counter = OledTask_Clamp(s_counter);

            /* KEY1 按下的反馈：点一下 LED1（可选，方便确认按键被识别） */
            if (k1_count != 0U)
            {
                Led_Toggle(LED_1);
            }
        }

        /* ---- 3. 只在数值变化时重绘，减少 I2C 流量 ---- */
        if (s_counter != old_counter)
        {
            OledTask_Draw();
        }

        vTaskDelay(pdMS_TO_TICKS(OLED_TASK_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------------- */
/* 任务创建                                                                    */
/* ------------------------------------------------------------------------- */

uint8_t OledTask_Init(void)
{
    BaseType_t ret;

    if (s_task_handle != NULL)
    {
        return 1U;      /* 已创建 */
    }

    /*
     * 创建两个计数信号量。
     * 初始计数 0，最大计数 OLED_KEY_SEM_MAX_COUNT。
     * 必须在创建任务之前创建信号量，否则任务第一次运行时会拿到 NULL。
     */
    s_sem_key1 = xSemaphoreCreateCounting(OLED_KEY_SEM_MAX_COUNT, 0U);
    s_sem_key2 = xSemaphoreCreateCounting(OLED_KEY_SEM_MAX_COUNT, 0U);

    if ((s_sem_key1 == NULL) || (s_sem_key2 == NULL))
    {
        /* 堆不够，创建失败 */
        if (s_sem_key1 != NULL)
        {
            vSemaphoreDelete(s_sem_key1);
            s_sem_key1 = NULL;
        }
        if (s_sem_key2 != NULL)
        {
            vSemaphoreDelete(s_sem_key2);
            s_sem_key2 = NULL;
        }
        return 0U;
    }

    ret = xTaskCreate(OledTask_Main,
                      "OledTask",
                      OLED_TASK_STACK_WORDS,
                      NULL,
                      OLED_TASK_PRIORITY,
                      &s_task_handle);

    if (ret != pdPASS)
    {
        vSemaphoreDelete(s_sem_key1);
        vSemaphoreDelete(s_sem_key2);
        s_sem_key1 = NULL;
        s_sem_key2 = NULL;
        s_task_handle = NULL;
        return 0U;
    }

    return 1U;
}

//
// Created by JiangyaF on 2026/10/22.
//
