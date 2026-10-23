//
// Created by JiangyaF on 2026/10/22.
//
// 实时控制任务实现
//
// 为什么用 vTaskDelayUntil 而不是 vTaskDelay：
//   vTaskDelay(n) 是"从此刻起睡 n 个 tick"，本身执行耗时会让周期越拖越长。
//   vTaskDelayUntil(&last, n) 是"睡到绝对时刻 last+n"，会补偿执行耗时，
//   长期平均周期精确。PID 的积分步长依赖固定周期，所以这里必须用前者。
//
// 为什么三个节拍放在同一个任务里而不是拆成三个任务：
//   它们共享大量状态（编码器速度、PID 状态、灰度数据），拆成三个任务就要加锁，
//   而且任务切换开销更大。用"一个任务 + 三个时间戳"最简单可靠。
//

#include "Control_task.h"
#include "PWM.h"
#include "Encoder.h"
#include "SpeedPID.h"
#include "LinePD.h"
#include "Gray.h"
#include "Led.h"
#include "Key.h"
#include "Ble.h"
#include "Oled.h"
#include "Oled_task.h"

extern Gray_Sensor_t g_gray;
extern Line_PD_t     g_line_pd;
extern I2C_HandleTypeDef hi2c1;

/* ------------------------------------------------------------------------- */
/* 任务内部状态                                                                */
/* ------------------------------------------------------------------------- */

static TaskHandle_t s_task_handle = NULL;

/*
 * 电机手动模式标志（详见 Control_task.h 的说明）。
 * 上电默认 0 = 正常 PID 控制。
 */
volatile uint8_t g_motor_manual_mode = 0U;

/* ------------------------------------------------------------------------- */
/* 任务主体                                                                    */
/* ------------------------------------------------------------------------- */

static void ControlTask_Main(void *argument)
{
    TickType_t last_wake;
    TickType_t last_outer;
    TickType_t last_ui;
    uint32_t   fps_counter = 0U;

    (void)argument;

    /*
     * 取当前 tick 作为基准。
     * 注意：三个节拍都用 vTaskDelayUntil 的绝对时刻推进，所以基准必须
     * 在任务首次运行时取，不能放在创建任务之前（中间可能有延时）。
     */
    last_wake  = xTaskGetTickCount();
    last_outer = last_wake;
    last_ui    = last_wake;

    for (;;)
    {
        TickType_t now;

        /* ================= 节拍 1：内环速度环，20ms ================= */
        Encoder_UpdateSpeed();

        if (CTRL_TASK_ENABLE_MOTION != 0)
        {
            /*
             * 外环会把左右轮目标速度写进内环（LinePD_Update 内部调用
             * Speed_PID_Update）。这里不再重复调用，避免一周期内跑两次 PID
             * 导致积分步长翻倍。
             */
        }
        else if (g_motor_manual_mode == 0U)
        {
            /* 安全模式：目标速度锁 0，电机保持停止 */
            Speed_PID_Update(MOTOR_A, 0.0f);
            Speed_PID_Update(MOTOR_B, 0.0f);
        }
        else
        {
            /*
             * 手动模式：完全不碰电机。
             * 这里不能调用 Speed_PID_Update，否则 PID 会把电机拉回 0，
             * 覆盖蓝牙任务设置的转速。
             */
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SPEED_PID_PERIOD_MS));

        /* ================= 节拍 2：外环循迹 PD，100ms ================= */
        now = xTaskGetTickCount();
        if ((now - last_outer) >= pdMS_TO_TICKS(LINE_PD_PERIOD_MS))
        {
            last_outer = now;

#if (CTRL_TASK_ENABLE_MOTION != 0)
            (void)LinePD_Update(&g_line_pd, &g_gray, &hi2c1);
#else
            /* 安全模式：仍然读灰度把数据刷新出来（便于 OLED/调试观察），
               但把结果丢弃，不驱动电机。 */
            (void)Gray_Update(&g_gray, &hi2c1);
#endif

            fps_counter++;

            /* OLED 刷新与外环同频。I2C2 是 100kHz 时整屏刷新约 19ms，
               占外环周期的 19%，所以默认关闭；改成 400kHz 后可打开。 */
            // OLED_ShowDashboard(&g_line_pd);
        }

        /* ================= 节拍 3：人机界面，20ms ================= */
        now = xTaskGetTickCount();
        if ((now - last_ui) >= pdMS_TO_TICKS(KEY_SCAN_PERIOD_MS))
        {
            last_ui = now;

            Led_Refresh();
            Key_Scan();
            BLE_UpdateLed();

            /* 按键事件分发。
               KEY1/KEY2 的短按交给 OLED 任务计数（通过计数信号量），
               长按保留作急停。 */
            {
                uint32_t key;
                uint8_t  event;

                while (Key_GetEvent(&key, &event) != 0U)
                {
                    if (event == KEY_EVENT_PRESS)
                    {
                        if ((key == KEY_1) || (key == KEY_2))
                        {
                            /*
                             * 关键：这里调用 xSemaphoreGive() 把按键"记账"，
                             * 而不是直接改数字。
                             *
                             * 为什么：本任务 20ms 循环一次，OLED 任务 100ms
                             * 才处理一次。如果用户在两帧之间连按 3 次，
                             * 这里每按一次 Give 一次，信号量计数变成 3；
                             * OLED 任务下次取走时一次加上 3，不会丢。
                             *
                             * 如果直接 OledTask_GetCounter() + 1 再写回，
                             * 反而会丢失（因为写回是覆盖式的）。
                             */
                            (void)OledTask_NotifyKeyPress(key);
                        }
                    }
                    else if ((key == KEY_2) && (event == KEY_EVENT_LONG_PRESS))
                    {
                        /* 长按 KEY2：急停（保留的安全功能） */
                        Speed_PID_StopAll();
                        LinePD_Stop(&g_line_pd);
                        Led_Off(LED_1);
                    }
                }
            }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* 任务创建                                                                    */
/* ------------------------------------------------------------------------- */

uint8_t ControlTask_Init(void)
{
    BaseType_t ret;

    if (s_task_handle != NULL)
    {
        return 1U;
    }

    ret = xTaskCreate(ControlTask_Main,
                      "CtrlTask",
                      CTRL_TASK_STACK_WORDS,
                      NULL,
                      CTRL_TASK_PRIORITY,
                      &s_task_handle);

    if (ret != pdPASS)
    {
        s_task_handle = NULL;
        return 0U;
    }

    return 1U;
}

//
// Created by JiangyaF on 2026/10/22.
//
