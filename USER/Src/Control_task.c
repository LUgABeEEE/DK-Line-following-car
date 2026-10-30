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
#include "Ble_task.h"       /* BleTask_SendTag()：跑圈结果打标 */
#include "MPU6050.h"        /* 陀螺仪：yaw 阻尼/定角转弯用 */

extern Gray_Sensor_t g_gray;
extern Line_PD_t     g_line_pd;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/* ------------------------------------------------------------------------- */
/* 主循环节拍                                                                  */
/* ------------------------------------------------------------------------- */

/*
 * 控制任务的主循环周期（ms）。
 *
 * 原来等于 SPEED_PID_PERIOD_MS(20ms)，导致所有"子节拍"最快也只能 20ms
 * 跑一次 —— 灰度采样就算写成 10ms，实际还是 20ms，等于没改。
 * 现在循环跑 5ms，各功能按自己的 elapsed 判断：
 *   灰度采样   5ms   （启停线脉冲窗口只有 15~37ms，越快越不容易漏）
 *   速度环    20ms   （编码器分辨率 0.56cm/s @20ms，更快无意义）
 *   外环 PD   20ms
 *   按键/UI   20ms
 */
#define CTRL_LOOP_PERIOD_MS     5U

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

/* 比赛自动停车的一圈周长，cm；0 = 关闭（练习用） */
volatile float   g_race_lap_cm     = 10.0f;  /* 10 = 自适应（<50 都是自适应） */

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

/* 自动停车只打印一次 */
static uint8_t s_race_announced = 0U;

/* 自动停车阶段（用于只在"识别到/到位"那一刻各发一条队列消息） */
static uint8_t s_race_last_phase = 0U;

/*
 * 是否允许打跑圈标记。
 *
 * 循迹中要打，停车后（自动停车/长按 KEY2/蓝牙急停）也要能补打：
 *   STOP 模式下 MODE 页：短按 KEY1 = 失败，长按 KEY3 = 成功。
 * 仍然要求停在 MODE 页，避免在调参页误触。
 */
static uint8_t ControlTask_TagAllowed(void)
{
    return ((g_ctrl_mode == CTRL_MODE_LINE_FOLLOW) ||
            (g_ctrl_mode == CTRL_MODE_STOP)) ? 1U : 0U;
}

/*
 * 按键短按/长按判定。
 *
 *   PRESS       -> 清长按标志
 *   LONG_PRESS  -> 置位（KEY2 的长按仍然是急停）
 *   RELEASE     -> 没置位就是短按
 *
 * 循迹模式 + MODE 页下，短按 KEY2 = 成功标记、短按 KEY1 = 失败标记。
 * 放在松手时判定，是为了避免"长按 KEY2 停车"同时打出一个假的成功标记。
 */
static uint8_t s_key_was_long[3] = {0U, 0U, 0U};

/*
 * 按下瞬间是否"已经在循迹模式 + MODE 页"。
 *
 * 必须记在按下时刻：从 STOP/SPD 切到 LINE 的那一次短按 KEY2，
 * 松手时模式已经是 LINE 了，如果不看按下时的状态，
 * 就会在刚进入循迹模式时凭空打出一个 [T] OK。
 */
static uint8_t s_key_press_eligible[3] = {0U, 0U, 0U};

/* ------------------------------------------------------------------------- */
/* 任务主体                                                                    */
/* ------------------------------------------------------------------------- */

static void ControlTask_Main(void *argument)
{
    TickType_t last_wake;
    TickType_t last_outer;
    TickType_t last_gray;
    TickType_t last_speed;
    TickType_t last_mpu;
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
    last_gray  = last_wake;
    last_speed = last_wake;
    last_mpu   = last_wake;
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

                /* 启动比赛自动停车（g_race_lap_cm=0 时等于关闭） */
                /*
                 * 启动比赛自动停车。
                 *
                 * ⚠️ 这里【不能】直接打印：BleTask_Print 是阻塞 UART 发送，
                 *    放在控制环里会拖住 5ms 节拍。参数是否生效统一由
                 *    BLE 任务的 [P] 行显示（lap=全局值 arm=实际武装点）：
                 *    两者不一致就说明参数没进到 LinePD。
                 */
                LinePD_RaceStart(&g_line_pd, g_race_lap_cm);
                s_race_announced = 0U;
                s_race_last_phase = 0U;

                /* 每次开跑把陀螺航向清零，[L] 的 y 和定角转弯都从 0 起算 */
                MPU6050_ResetYaw(0.0f);
            }
            else
            {
                LinePD_RaceCancel(&g_line_pd);
            }
        }

        /* ================= 节拍 1：内环速度环，20ms ================= */
        /*
         * ⚠️ 编码器测速必须保持 20ms 窗口。
         *
         * 主循环改成 5ms 后这里曾被无条件每拍调用，等于把测速窗口缩到 5ms：
         *   20ms 下 1 计数 = 0.56cm/s，5ms 下 1 计数 = 2.26cm/s
         * 同样一个计数抖动，速度反馈噪声放大 4 倍，速度环微分项跟着噪，
         * 表现就是"提高采样率后车变抖"。测速窗口必须和速度环同频。
         */
        now = xTaskGetTickCount();
        if ((now - last_speed) >= pdMS_TO_TICKS(SPEED_PID_PERIOD_MS))
        {
            last_speed = now;

            Encoder_UpdateSpeed();
        }

        /*
         * 陀螺仪：读角速度 + 积分航向，门控在 20ms。
         *
         * ⚠️ 主循环改成 5ms 后这里曾经变成每 5ms 读一次：
         *    MPU6050_Update() 内部是【阻塞式 I2C2 读】（约 0.35ms），
         *    而 I2C2 上还挂着 OLED 的 DMA 整屏传输（一页 129 字节约 3ms）。
         *    两者抢同一条总线时，控制任务可能被拖住好几毫秒 ——
         *    这正是"识别启停线时控制反应不过来"的来源。
         *    门控回 20ms 后，阻塞读次数降到原来的 1/4，
         *    而且航向积分本来就是按 HAL_GetTick 的真实间隔算的，频率降低不影响精度。
         */
        now = xTaskGetTickCount();
        if ((now - last_mpu) >= pdMS_TO_TICKS(20U))
        {
            last_mpu = now;

            MPU6050_Update();
        }

        /*
         * MPU6050 掉线重试：上电时可能因为模块还没稳定/接插件接触不良
         * 导致初始化失败。这里每 3 秒重试一次；只有车停着（STOP）才标定，
         * 避免把行驶中的角速度当零偏。
         */
        /*
         * MPU6050 掉线重试：只在【停车】状态下做。
         *
         * MPU6050_Init 内部有 HAL_Delay(30) + HAL_Delay(60)，
         * MPU6050_Calibrate(100) 又是 100 次 HAL_Delay(2) ——
         * 合计 90~300ms 的阻塞。放在控制环里跑车时会卡住 5ms 节拍，
         * 正好可能错过启停线。停车时重试没有任何副作用。
         */
        if ((MPU6050_IsReady() == 0U) && (g_ctrl_mode == CTRL_MODE_STOP))
        {
            static uint32_t s_mpu_retry_tick = 0U;

            if ((HAL_GetTick() - s_mpu_retry_tick) >= 3000U)
            {
                s_mpu_retry_tick = HAL_GetTick();

                if (MPU6050_Init(&hi2c2) != 0U)
                {
                    (void)MPU6050_Calibrate(100U);
                }
            }
        }

        /*
         * ---- 节拍 1b：灰度采样，5ms ----
         *
         * 为什么是 5ms：
         *   启停线识别靠"中间连续黑 + 两边留白"的短脉冲，脉冲窗口在
         *   55cm/s 时只有 15.5mm ≈ 28ms，20ms 采样只能抓到 1~2 拍，
         *   偶尔整拍漏掉。5ms 能抓 5~7 拍，漏检率大幅下降。
         *
         * 为什么不影响控制律：
         *   LinePD 仍然在下面节拍 2 按 LINE_PD_PERIOD_MS(20ms) 跑，
         *   速度环门控在 20ms，这里只是把"采样"加快。
         *
         * 为什么 CPU 不受影响：
         *   采样走 DMA 异步双缓冲（Gray_StartSample 发起后立刻返回，
         *   完成时由中断翻缓冲、置就绪标志，中断里不调用任何 FreeRTOS API）。
         *
         * 所有模式都发起采样（SPD/STOP 模式下 OLED 调试页也要看灰度），
         * 但只有循迹模式才消费（Gray_Update）。
         */
        now = xTaskGetTickCount();
        if ((now - last_gray) >= pdMS_TO_TICKS(GRAY_SAMPLE_PERIOD_MS))
        {
            last_gray = now;

            /*
             * 先发起一次异步 DMA 采样（不等待），再消费"上一拍已完成"的那一帧。
             * 数据滞后一个采样周期（5ms），对循迹和启停线检测都没有影响，
             * 换来的是传输期间 CPU 完全不被占用。
             */
            Gray_StartSample(&hi2c1);

            /*
             * 所有模式都消费一帧。
             *
             * 改成 DMA 异步后 Gray_Update() 里已经没有 I2C 事务了
             * （只是解析一块缓冲），所以非循迹模式也调用它【不再有总线代价】。
             * 之前 cg 版把它从非循迹分支里删掉，导致 SPD/STOP 模式下
             * OLED 传感器页和 [G] 调试输出全是 0（e=0/0），
             * 没法单独验证传感器 —— 这里恢复。
             */
            (void)Gray_Update(&g_gray, &hi2c1);
        }

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
            /* 速度环与测速同频（上面那个门控里已经跑过测速） */
            (void)Speed_PID_Update(MOTOR_A, g_line_pd.left_target);
            (void)Speed_PID_Update(MOTOR_B, g_line_pd.right_target);

            /*
             * 自动停车：外环判定"测试点已到启停线"后，
             * 走和急停完全相同的路径（切 STOP + 清积分 + 硬制动），
             * 保证停车行为和长按 KEY2 / 蓝牙 X 一致。
             */
            /*
             * 自动停车阶段变化：只在"识别到启停线"那一拍入队一条 [RACE] det。
             * 串口打印放在蓝牙任务里，控制任务不阻塞。
             */
            {
                uint8_t ph = LinePD_RacePhase(&g_line_pd);

                if ((ph == 2U) && (s_race_last_phase != 2U))
                {
                    (void)BleTask_SendTag(2U);
                }
                s_race_last_phase = ph;
            }

            if (LinePD_RaceDone(&g_line_pd) != 0U)
            {
                ControlTask_RequestStop();
                Motor_Brake();

                if (s_race_announced == 0U)
                {
                    s_race_announced = 1U;
                    (void)BleTask_SendTag(3U);      /* 队列里发 [RACE] stop */
                }
            }
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

        /*
         * 主循环节拍：5ms。
         *
         * 原来这里是 SPEED_PID_PERIOD_MS(20ms)，导致所有"子节拍"最快也只能
         * 20ms 跑一次 —— 灰度采样写 10ms 实际上还是 20ms，等于没改。
         * 现在循环本身跑 5ms，各功能用各自的 elapsed 判断：
         *   灰度采样   5ms  （启停线脉冲窗口只有 15~37ms，越快越不容易漏）
         *   速度环    20ms
         *   外环 PD   20ms
         *   按键/UI   20ms
         */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CTRL_LOOP_PERIOD_MS));

        /* ================= 节拍 2：外环循迹 PD，100ms ================= */
        now = xTaskGetTickCount();
        if ((now - last_outer) >= pdMS_TO_TICKS(LINE_PD_PERIOD_MS))
        {
            last_outer = now;

if (g_ctrl_mode == CTRL_MODE_LINE_FOLLOW)
            {
                /* 双环模式：外环算差速并写入内环目标 */
                (void)LinePD_Update(&g_line_pd, &g_gray);
            }
            else
            {
                /*
                 * 非循迹模式：灰度采样已由上面的 10ms 节拍负责，
                 * 这里不再重复读总线（以前两个分支各读一次，白占带宽）。
                 */
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
                    if (key > KEY_3)
                    {
                        continue;
                    }

                    if (event == KEY_EVENT_PRESS)
                    {
                        s_key_was_long[key]      = 0U;
                        s_key_press_eligible[key] =
                            ((ControlTask_TagAllowed() != 0U) &&
                             (OledTask_IsModePage() != 0U)) ? 1U : 0U;

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
                    else if (event == KEY_EVENT_LONG_PRESS)
                    {
                        s_key_was_long[key] = 1U;

                        if (key == KEY_1)
                        {
                            /*
                             * 长按 KEY1：在 STOP + MODE 页时进入 SPD（速度测试）。
                             * KEY2 短按已经改成"直接进 LINE"，SPD 挪到这里，
                             * 避免循迹前误进速度测试。
                             */
                            if ((g_ctrl_mode == CTRL_MODE_STOP) &&
                                (OledTask_IsModePage() != 0U))
                            {
                                g_ctrl_mode = CTRL_MODE_SPEED_TEST;
                            }
                        }
                        else if (key == KEY_2)
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
                        else if (key == KEY_3)
                        {
                            /*
                             * 长按 KEY3：本次跑【成功】-> [T] OK
                             *
                             * 为什么从"短按 KEY2"改到"长按 KEY3"：
                             *   停车要长按 KEY2，按键那一刻会先被当成一次
                             *   短按，容易误打一个成功标记；把成功标挪到
                             *   长按 KEY3，和停车彻底分开，不会互相干扰。
                             *
                             * 长按 KEY3 在循迹模式下随时可用（不要求当前在
                             * MODE 页）：按下瞬间 KEY3 会先切走页面，
                             * 打标后这里把它退回 MODE 页，方便看实时数据。
                             */
                            if (ControlTask_TagAllowed() != 0U)
                            {
                                (void)BleTask_SendTag(1U);
                            }

                            /*
                             * 不管在哪个模式，长按 KEY3 都把页面切回 MODE 页：
                             * 按下瞬间会把页面翻走，这里负责翻回来。
                             */
                            OledTask_SetModePage();
                        }
                    }
                    else if (event == KEY_EVENT_RELEASE)
                    {
                        /*
                         * 短按 KEY1：本次跑【失败】-> [T] FAIL。
                         * 长按 KEY3 的成功标在 LONG_PRESS 里已经发过，
                         * 松手时不再重复。
                         */
                        if ((s_key_was_long[key] == 0U) &&
                            (s_key_press_eligible[key] != 0U) &&
                            (key == KEY_1))
                        {
                            (void)BleTask_SendTag(0U);
                        }
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

uint32_t ControlTask_GetStackFreeWords(void)
{
    if (s_task_handle == NULL)
    {
        return 0U;
    }
    return (uint32_t)uxTaskGetStackHighWaterMark(s_task_handle);
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
