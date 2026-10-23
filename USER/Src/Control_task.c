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
 * 运行模式（详见 Control_task.h）。
 * 上电默认进入【速度环测试】——当前阶段的目标就是调内环。
 */
volatile uint8_t g_ctrl_mode       = CTRL_MODE_SPEED_TEST;
volatile float   g_speed_test_target = 20.0f;

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

        switch (g_ctrl_mode)
        {
        case CTRL_MODE_LINE_FOLLOW:
            /*
             * 双环模式：外环 LinePD_Update（在外环节拍里）会调用
             * Speed_PID_Update 把左右轮目标写进内环。
             * 这里不能重复调用，否则一周期跑两次 PID，积分步长翻倍。
             */
            break;

        case CTRL_MODE_SPEED_TEST:
            /*
             * 速度环测试：两轮追踪同一个目标速度。
             * 内环每 20ms 跑一次，编码器反馈闭环。
             */
            (void)Speed_PID_Update(MOTOR_A, g_speed_test_target);
            (void)Speed_PID_Update(MOTOR_B, g_speed_test_target);
            break;

        case CTRL_MODE_STOP:
        default:
            /* 停车：目标锁 0，内环把电机拉回静止 */
            (void)Speed_PID_Update(MOTOR_A, 0.0f);
            (void)Speed_PID_Update(MOTOR_B, 0.0f);
            break;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SPEED_PID_PERIOD_MS));

        /* ================= 节拍 2：外环循迹 PD，100ms ================= */
        now = xTaskGetTickCount();
        if ((now - last_outer) >= pdMS_TO_TICKS(LINE_PD_PERIOD_MS))
        {
            last_outer = now;

if (g_ctrl_mode == CTRL_MODE_LINE_FOLLOW)
            {
                /* 双环模式：外环算差速并写入内环目标 */
                (void)LinePD_Update(&g_line_pd, &g_gray, &hi2c1);
            }
            else
            {
                /*
                 * 非循迹模式：仍然读灰度刷新数据（便于 OLED/调试观察），
                 * 但丢弃结果，不驱动电机。
                 */
                (void)Gray_Update(&g_gray, &hi2c1);
            }

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
               KEY1/KEY2/KEY3 的短按全部交给 OLED 任务做 PID 调参，
               通过计数信号量传递，避免两次任务调度之间连按丢事件。 */
            {
                uint32_t key;
                uint8_t  event;

                while (Key_GetEvent(&key, &event) != 0U)
                {
                    if (event == KEY_EVENT_PRESS)
                    {
                        if ((key == KEY_1) || (key == KEY_2) || (key == KEY_3))
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
                        /* 长按 KEY2：急停。切换回停车模式，电机目标锁 0 */
                        g_ctrl_mode = CTRL_MODE_STOP;
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

/* ------------------------------------------------------------------------- */
/* 模式切换接口                                                                */
/* ------------------------------------------------------------------------- */

void ControlTask_RequestStop(void)
{
    /*
     * 只改模式，不直接调 Speed_PID_StopAll()。
     *
     * 原因：本函数可能被低优先级的蓝牙任务调用，而控制任务（优先级 3）
     * 每 20ms 会按 g_ctrl_mode 重新下发电机指令。
     * 如果这里直接 StopAll，控制任务下一拍就会用旧目标覆盖回去。
     * 改模式才是最可靠的做法——控制任务看到 STOP 会持续输出 0。
     */
    g_ctrl_mode = CTRL_MODE_STOP;
}

void ControlTask_RequestSpeedTest(float target_cms)
{
    g_speed_test_target = target_cms;
    g_ctrl_mode         = CTRL_MODE_SPEED_TEST;
}
