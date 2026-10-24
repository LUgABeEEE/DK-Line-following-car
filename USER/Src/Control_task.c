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
volatile float   g_speed_test_target = 0.0f;

/*
 * 斜坡化之后的目标速度（内部状态）。
 * 实际下发给内环的是这个值，而不是 g_speed_test_target 的阶跃值。
 */
static float s_ramped_target = 0.0f;

/*
 * 上一拍的模式，用于检测"刚切换到 STOP"这个瞬间。
 * 进入停车模式时需要清一次 PID 状态，之后就不必每拍都清。
 */
static uint8_t s_last_mode = CTRL_MODE_SPEED_TEST;

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

        /*
         * 模式切换的一次性处理。
         *
         * 只处理"刚切过来"这一拍，之后 s_last_mode 就更新了。
         */
        if (g_ctrl_mode != s_last_mode)
        {
            if (g_ctrl_mode == CTRL_MODE_LINE_FOLLOW)
            {
                /*
                 * 刚进入循迹：清外环历史。
                 *
                 * 为什么必须清：外环的微分项是 (error - last_error)/dt。
                 * 如果从 SPEED_TEST 直接切过来，last_error 还停在上次
                 * 循迹结束时的旧值，first_run 也已经是 0，
                 * 那么第一拍会算出一个巨大的微分 ——
                 * steer 瞬间打满差速，车会猛打一下方向。
                 *
                 * LinePD_Reset 会把 first_run 重新置 1，
                 * 下一拍走"首拍不做微分"的保护分支。
                 */
                LinePD_Reset(&g_line_pd);
            }
        }

        /* ================= 节拍 1：内环速度环，20ms ================= */
        Encoder_UpdateSpeed();

        switch (g_ctrl_mode)
        {
        case CTRL_MODE_LINE_FOLLOW:
            /*
             * 双环模式：外环只负责"算目标"，这里负责"跟目标"。
             *
             * 外环 LinePD_Update 每 100ms 跑一次，把左右轮目标速度
             * 写进 g_line_pd.left_target / right_target；
             * 内环在这里以 20ms 的节奏持续追踪这两个目标。
             *
             * 两个环各跑各的周期，互不牵制：
             *   内环 50Hz —— 保证轮速跟得上
             *   外环 10Hz —— 保证转向指令有足够带宽
             *
             * （以前外环里直接调 Speed_PID_Update，导致速度环也被
             *   拖到 10Hz，轮子跟不上目标，循迹只要有一点弯就跟不住。）
             */
            (void)Speed_PID_Update(MOTOR_A, g_line_pd.left_target);
            (void)Speed_PID_Update(MOTOR_B, g_line_pd.right_target);
            break;

        case CTRL_MODE_SPEED_TEST:
            /*
             * 速度环测试：两轮追踪同一个目标速度，带加减速斜坡。
             *
             * 每 20ms 最多变化 SPEED_RAMP_CMS_PER_SEC * 0.02 = 0.8 cm/s。
             * 目标 20 cm/s 大约 0.5 秒爬到，避免阶跃起步时两轮不同步。
             */
            {
                float step = SPEED_RAMP_CMS_PER_SEC
                           * ((float)SPEED_PID_PERIOD_MS / 1000.0f);

                if (s_ramped_target < g_speed_test_target)
                {
                    s_ramped_target += step;
                    if (s_ramped_target > g_speed_test_target)
                    {
                        s_ramped_target = g_speed_test_target;
                    }
                }
                else if (s_ramped_target > g_speed_test_target)
                {
                    s_ramped_target -= step;
                    if (s_ramped_target < g_speed_test_target)
                    {
                        s_ramped_target = g_speed_test_target;
                    }
                }
                else
                {
                    /* 已在目标值，无需调整 */
                }

                (void)Speed_PID_Update(MOTOR_A, s_ramped_target);
                (void)Speed_PID_Update(MOTOR_B, s_ramped_target);
            }
            break;

        case CTRL_MODE_STOP:
        default:
            /*
             * 停车：直接用 Motor_Brake() 短接电机两端制动，
             * 不走 PID。
             *
             * 为什么不用 Speed_PID_Update(0)：
             *   PID 的积分项是"历史载荷"的累积。以 15 cm/s 巡航时
             *   积分可能已经到 20 左右。停车时 Target=0，但
             *     out = KFF*0 + KP*(-measure) + 积分
             *         = 0 - 12 + 20 = +8
             *   仍然是正数，电机继续往前爬 —— 表现就是"发停车命令但车不停"。
             *
             *   而且积分分离会"帮倒忙"：停车瞬间误差很大（负数），
             *   积分分离会冻结积分，残留的正积分长时间不衰减。
             *
             * 硬制动不受这些影响：直接把 H 桥下桥臂全开，
             * 电机绕组短接，靠反电动势制动，立刻停住并保持。
             */
            if (s_last_mode != CTRL_MODE_STOP)
            {
                /*
                 * 只在刚进入停车模式时清一次 PID 状态。
                 * 清完积分和微分历史，下次启动才从干净状态开始，
                 * 不会带着巡航时的残留量猛冲。
                 */
                Speed_PID_Reset(MOTOR_A);
                Speed_PID_Reset(MOTOR_B);
            }

            s_ramped_target = 0.0f;
            Motor_Brake();
            break;
        }

        s_last_mode = g_ctrl_mode;

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
                        /*
                         * 长按 KEY2：急停（硬制动）。
                         *
                         * 和蓝牙 X 命令走同一个函数，保证两条路径行为一致：
                         *   切停车模式 + 清 PID 积分 + 立即制动 + 停循迹
                         */
                        ControlTask_RequestStop();
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
     * 急停的统一入口。蓝牙 X 命令和 KEY2 长按都走这里，
     * 保证两条路径的停车行为完全一致。
     *
     * 四个动作缺一不可：
     *
     *   1. 切模式 —— 这是最关键的一步。
     *      控制任务每 20ms 按 g_ctrl_mode 重新下发指令，
     *      只清 PID 而不改模式的话，下一拍就会被旧目标覆盖回去。
     *      改成 STOP 后，控制任务会持续调用 Motor_Brake() 保持制动。
     *
     *   2. 清 PID 积分 —— 积分是"历史载荷"的累积量。
     *      巡航时积分可能已到 20 左右，不清掉的话即使目标为 0，
     *      out = KP*(-measure) + 积分 仍可能是正数，电机继续往前爬。
     *
     *   3. 停循迹 —— 停掉外环 PD，避免它继续往下写目标速度。
     *
     *   4. 立即制动 —— 不等内环下一拍（最多 20ms），马上短接电机绕组。
     *
     * 说明：本函数可能被蓝牙任务（优先级 2）调用，与控制任务（优先级 3）
     * 存在极小的并发窗口。但制动是幂等操作（写同样的 GPIO 状态），
     * 最坏情况只是控制任务下一拍再写一次相同值，没有副作用。
     */
    g_ctrl_mode = CTRL_MODE_STOP;

    /*
     * 目标速度也清零。
     *
     * 这是"目标即油门"设计的关键一环：
     * 急停后目标归零，用户必须重新用 KEY2 把速度一档档加上来，
     * 不会因为残留的目标值在解除停车时突然窜出去。
     *
     * 同时也让屏幕上的 T 值立刻变成 0，视觉反馈明确。
     */
    g_speed_test_target = 0.0f;

    Speed_PID_Reset(MOTOR_A);
    Speed_PID_Reset(MOTOR_B);

    /*
     * 清掉编码器的速度滤波状态。
     *
     * Encoder_UpdateSpeed 里对速度做了一阶低通，s_speed_cms[] 会保留
     * 停车前的值。不清的话，重新启动后的头几拍测速会带着这个旧值，
     * PID 看到的"实测速度"是错的，起步会顿一下。
     *
     * 用 Encoder_ResetFilter 而不是 Encoder_Reset：
     *   后者会把计数和累计里程也清零，而我们只想清滤波状态。
     */
    Encoder_ResetFilter(ENCODER_A);
    Encoder_ResetFilter(ENCODER_B);

    LinePD_Stop(&g_line_pd);

    /* 斜坡值也归零，下次启动从 0 平滑爬升 */
    s_ramped_target = 0.0f;

    Motor_Brake();
}

void ControlTask_RequestSpeedTest(float target_cms)
{
    g_speed_test_target = target_cms;
    g_ctrl_mode         = CTRL_MODE_SPEED_TEST;
}
