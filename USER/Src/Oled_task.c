//
// Created by JiangyaF on 2026/10/22.
//
// OLED 显示 + PID 调参任务实现
//
// 核心机制：三个按键各用一个【计数信号量】累计事件，防止连按丢包。
//
// 为什么一个键一个信号量：
//   计数信号量只记录"发生了几次"，不记录"是哪个键"。
//   如果三个键共用一个信号量，取走 3 次计数时无法区分
//   "KEY1 按了 2 次、KEY2 按了 1 次" 还是 "KEY3 按了 3 次"，
//   调整方向和参数选择就全乱了。
//
// 取空信号量的写法：
//   while (xSemaphoreTake(sem, 0) == pdTRUE) { ... }
//   超时传 0 表示"有多少拿多少，不阻塞"，把当前所有待处理事件一次性取完。
//

#include "Oled_task.h"
#include "Oled.h"
#include "Key.h"
#include "Led.h"
#include "SpeedPID.h"
#include "Control_task.h"
#include <stdio.h>
#include <string.h>

/*
 * 信号量 API 在 semphr.h 里，FreeRTOS.h 只包含基础类型和配置。
 * 不包含它会导致 SemaphoreHandle_t 未定义、xSemaphoreCreateCounting
 * 和 xSemaphoreGive 隐式声明。
 */
#include "semphr.h"

/* 串口句柄定义在 main.c 中，用于调试打印 */
extern UART_HandleTypeDef huart2;

/* ------------------------------------------------------------------------- */
/* 可调参数表                                                                  */
/* ------------------------------------------------------------------------- */

/*
 * 每个参数的步长和上下限。
 *
 * 步长不是随便定的——必须和这个参数的典型量级匹配：
 *   步长太大 -> 按一下就过头，永远调不到最优点
 *   步长太小 -> 从 0 调到目标值要按上百下
 *
 * 详细理由见 Oled_task.h 的说明。
 */
typedef struct
{
    const char *name;       /* 显示用名称          */
    float       step;       /* 每按一下的变化量    */
    float       min;        /* 下限                */
    float       max;        /* 上限                */
} TuneParam_t;

#define TUNE_IDX_KP         0U
#define TUNE_IDX_KI         1U
#define TUNE_IDX_KD         2U
#define TUNE_IDX_KFF        3U
#define TUNE_IDX_TARGET     4U
#define TUNE_PARAM_COUNT    5U

static const TuneParam_t s_params[TUNE_PARAM_COUNT] =
{
    /* 名称       步长     下限    上限   */
    { "KP",      0.10f,   0.00f,  5.00f },
    { "KI",      0.10f,   0.00f,  5.00f },
    { "KD",      0.01f,   0.00f,  1.00f },   /* KD 量级小，用 0.01 */
    { "KFF",     0.05f,   0.00f,  2.00f },
    { "TARGET",  1.00f,   0.00f, 40.00f },   /* 单位 cm/s */
};

/* ------------------------------------------------------------------------- */
/* 任务内部状态                                                                */
/* ------------------------------------------------------------------------- */

static TaskHandle_t s_task_handle = NULL;

/* 三个按键各一个计数信号量 */
static SemaphoreHandle_t s_sem_key1 = NULL;
static SemaphoreHandle_t s_sem_key2 = NULL;
static SemaphoreHandle_t s_sem_key3 = NULL;

/* 当前选中的参数编号 */
static volatile uint8_t s_param_index = TUNE_IDX_KP;

/* 累计接受过的按键次数（诊断用，确认没丢包） */
static volatile uint32_t s_key1_total = 0U;
static volatile uint32_t s_key2_total = 0U;
static volatile uint32_t s_key3_total = 0U;

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

/* 读当前选中参数的值 */
static float OledTask_ReadParam(uint8_t idx)
{
    if (idx >= TUNE_PARAM_COUNT)
    {
        return 0.0f;
    }

    if (idx == TUNE_IDX_TARGET)
    {
        return g_speed_test_target;
    }

    /* 左右轮参数始终同步，读 A 的就行 */
    return Speed_PID_GetGain(MOTOR_A, idx);
}

/* 写当前选中参数的值（自动限幅） */
static void OledTask_WriteParam(uint8_t idx, float value)
{
    if (idx >= TUNE_PARAM_COUNT)
    {
        return;
    }

    /* 限幅到该参数允许的范围 */
    if (value < s_params[idx].min)
    {
        value = s_params[idx].min;
    }
    else if (value > s_params[idx].max)
    {
        value = s_params[idx].max;
    }

    if (idx == TUNE_IDX_TARGET)
    {
        /*
         * 目标速度。注意：只改数值，不改模式。
         * 如果当前是 STOP 模式，改目标不会让车动——
         * 这是刻意设计，避免误碰按键导致车突然冲出去。
         */
        g_speed_test_target = value;
        return;
    }

    /*
     * 左右轮必须同步改，否则两轮增益不同，车会跑偏。
     * SetGain 内部会 Reset 积分和微分滤波状态。
     */
    Speed_PID_SetGain(MOTOR_A, idx, value);
    Speed_PID_SetGain(MOTOR_B, idx, value);
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
    else if (key == KEY_3)
    {
        if (s_sem_key3 != NULL)
        {
            ret = xSemaphoreGive(s_sem_key3);
            if (ret == pdTRUE)
            {
                s_key3_total++;
            }
        }
    }
    else
    {
        return 0U;      /* 不是我们关心的键 */
    }

    /*
     * 返回 0 表示信号量已满（计数到达 OLED_KEY_SEM_MAX_COUNT）。
     * 这不是故障——说明两次显示刷新之间按了超过 16 次，已超出人类极限，
     * 但保留返回值便于调试时发现异常。
     */
    return (ret == pdTRUE) ? 1U : 0U;
}

uint8_t OledTask_GetParamIndex(void)
{
    return s_param_index;
}

float OledTask_GetParamValue(void)
{
    return OledTask_ReadParam(s_param_index);
}

/* ------------------------------------------------------------------------- */
/* OLED 绘制                                                                   */
/* ------------------------------------------------------------------------- */

static void OledTask_Draw(void)
{
    char    line[32];
    float   value;

    value = OledTask_ReadParam(s_param_index);

    OLED_Clear();

    /* ---- 第 1 行：标题（8x16） ---- */
    OLED_ShowString(0U, 0U, "PID TUNE", OLED_FONT_8X16, OLED_WHITE);

    /*
     * ---- 第 2 行：当前参数名 + 值（8x16 大字，最醒目） ----
     *
     * KD 用两位小数显示，其余用一位——因为 KD 的步长是 0.01，
     * 只显示一位的话按一下看不出变化。
     */
    if (s_param_index == TUNE_IDX_KD)
    {
        (void)snprintf(line, sizeof(line), "%s %.2f",
                       s_params[s_param_index].name, (double)value);
    }
    else
    {
        (void)snprintf(line, sizeof(line), "%s %.1f",
                       s_params[s_param_index].name, (double)value);
    }
    OLED_ShowString(0U, 18U, line, OLED_FONT_8X16, OLED_WHITE);

    /*
     * ---- 第 3 行：目标速度 vs 实测速度 ----
     *
     * 这是调内环最关键的反馈：看实测能不能跟上目标，
     * 以及有没有超调、震荡。
     */
    (void)snprintf(line, sizeof(line), "T:%4.1f A:%4.1f",
                   (double)g_speed_test_target,
                   (double)Speed_PID_GetMeasure(MOTOR_A));
    OLED_ShowString(0U, 38U, line, OLED_FONT_6X8, OLED_WHITE);

    /* ---- 第 4 行：操作提示 ---- */
    OLED_ShowString(0U, 48U, "K1- K2+ K3:next", OLED_FONT_6X8, OLED_WHITE);

    /*
     * ---- 第 5 行：运行模式 + 按键累计次数 ----
     *
     * 按键计数用来验证计数信号量没丢包：
     * 如果显示的数字比你实际按的次数少，说明丢事件了。
     */
    {
        const char *mode_str;

        switch (g_ctrl_mode)
        {
        case CTRL_MODE_SPEED_TEST: mode_str = "SPD";  break;
        case CTRL_MODE_LINE_FOLLOW: mode_str = "LINE"; break;
        case CTRL_MODE_STOP:
        default:                    mode_str = "STOP"; break;
        }

        (void)snprintf(line, sizeof(line), "%s K1:%lu K2:%lu K3:%lu",
                       mode_str,
                       (unsigned long)s_key1_total,
                       (unsigned long)s_key2_total,
                       (unsigned long)s_key3_total);
        OLED_ShowString(0U, 56U, line, OLED_FONT_6X8, OLED_WHITE);
    }

    (void)OLED_Refresh();
}

/* ------------------------------------------------------------------------- */
/* 任务主体                                                                    */
/* ------------------------------------------------------------------------- */

static void OledTask_Main(void *argument)
{
    uint32_t k1n;
    uint32_t k2n;
    uint32_t k3n;
    uint8_t  old_index;
    float    old_value;

    (void)argument;

    /* 等 OLED 和按键初始化稳定 */
    vTaskDelay(pdMS_TO_TICKS(200U));

    /*
     * 现在调度器已经跑起来了，可以安全启用异步传输（I2C 中断 / DMA）。
     *
     * 之前 OLED_Init() 在调度器启动前调用，那时只能用轮询
     * （阻塞式信号量在调度器启动前会卡死整个系统）。
     */
    OLED_EnableAsync();

    /* 首帧 */
    OledTask_Draw();

    for (;;)
    {
        old_index = s_param_index;
        old_value = OledTask_ReadParam(s_param_index);

        /* ---- 1. 把三个信号量里的待处理按键全部取走 ---- */
        k1n = OledTask_DrainSemaphore(s_sem_key1);
        k2n = OledTask_DrainSemaphore(s_sem_key2);
        k3n = OledTask_DrainSemaphore(s_sem_key3);

        /* ---- 2. KEY3：切换参数（每按一下前进一档，循环） ---- */
        if (k3n != 0U)
        {
            s_param_index = (uint8_t)((s_param_index + k3n) % TUNE_PARAM_COUNT);

            /*
             * 换了参数必须重新取基准值！
             *
             * old_value 是在切换之前读的，属于【旧参数】的值。
             * 如果同一轮里又按了 KEY1/KEY2，就会拿旧参数的值
             * 去加减新参数的步长，写进新参数——数值会跳到离谱的地方。
             */
            old_value = OledTask_ReadParam(s_param_index);
        }

        /* ---- 3. KEY1/KEY2：按步长增减当前参数 ---- */
        if ((k1n != 0U) || (k2n != 0U))
        {
            float step = s_params[s_param_index].step;
            float delta = (float)k2n * step - (float)k1n * step;

            OledTask_WriteParam(s_param_index, old_value + delta);
        }

        /*
         * ---- 4. 只在参数或数值变化时重绘 ----
         *
         * 但 TARGET 和实测速度每拍都可能变（车在跑时实测速度一直在变），
         * 所以运行在 SPEED_TEST/LINE 模式下需要每拍刷新，保证能看到实时速度。
         * 停车状态下才做"变化才刷"的优化，减少 I2C 流量。
         */
        if ((old_index != s_param_index) ||
            (old_value != OledTask_ReadParam(s_param_index)) ||
            (g_ctrl_mode != CTRL_MODE_STOP))
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
     * 创建三个计数信号量。
     * 初始计数 0，最大计数 OLED_KEY_SEM_MAX_COUNT。
     * 必须在创建任务之前创建信号量，否则任务第一次运行时会拿到 NULL。
     */
    s_sem_key1 = xSemaphoreCreateCounting(OLED_KEY_SEM_MAX_COUNT, 0U);
    s_sem_key2 = xSemaphoreCreateCounting(OLED_KEY_SEM_MAX_COUNT, 0U);
    s_sem_key3 = xSemaphoreCreateCounting(OLED_KEY_SEM_MAX_COUNT, 0U);

    if ((s_sem_key1 == NULL) || (s_sem_key2 == NULL) || (s_sem_key3 == NULL))
    {
        /* 堆不够，创建失败，把已创建的回收掉，避免内存泄漏 */
        if (s_sem_key1 != NULL) { vSemaphoreDelete(s_sem_key1); s_sem_key1 = NULL; }
        if (s_sem_key2 != NULL) { vSemaphoreDelete(s_sem_key2); s_sem_key2 = NULL; }
        if (s_sem_key3 != NULL) { vSemaphoreDelete(s_sem_key3); s_sem_key3 = NULL; }
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
        vSemaphoreDelete(s_sem_key3);
        s_sem_key1 = NULL;
        s_sem_key2 = NULL;
        s_sem_key3 = NULL;
        s_task_handle = NULL;
        return 0U;
    }

    return 1U;
}

//
// Created by JiangyaF on 2026/10/22.
//
