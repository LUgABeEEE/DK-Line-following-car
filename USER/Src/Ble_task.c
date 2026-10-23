//
// Created by JiangyaF on 2026/10/22.
//
// 蓝牙任务（FreeRTOS）—— 骨架版
//
// 本文件只保留任务骨架，业务逻辑留空由你填写。
// 驱动层（Ble.c）的收发接口保持不变，直接用即可：
//
//   发送：BLE_Send(buf, len)  /  BLE_SendString("text")  /  BLE_SendByte(b)
//   接收：BLE_GetByte(&b)     /  BLE_GetLine(buf, len)
//   状态：BLE_IsConnected()
//
// 任务参数：
//   优先级 2，栈 320 字（1280 字节），周期 100ms
//
// ⚠️ 关于调试打印：本任务所有输出都走 USART2，也就是 HC-05 所接的串口。
//    所以 BleTask_Print() 的内容手机端也能看到；反过来手机发来的数据
//    也会进入本任务的接收缓冲。调试打印和数据通道是同一个串口。
//

#include "Ble_task.h"
#include "Ble.h"
#include "Led.h"
#include "PWM.h"            /* Motor_SetSpeed / Motor_Brake / Motor_Coast */
#include "Control_task.h"   /* g_motor_manual_mode */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 串口句柄定义在 main.c 中 */
extern UART_HandleTypeDef huart2;

/* ------------------------------------------------------------------------- */
/* 电机测试参数                                                                */
/* ------------------------------------------------------------------------- */

/*
 * 每收到一个方向命令，电机通电多久（毫秒）后自动停止。
 *
 * 【实测结论】10ms 完全看不到轮子转——齿轮减速电机的静摩擦和
 *   机械时间常数都需要几十毫秒才能克服，10ms 只会"咔"一声。
 *   所以默认改成 300ms（能清楚看到转动约 1/3 圈）。
 *
 *   调参建议：
 *     150ms  只转一点点，适合微调
 *     300ms  默认，能看清方向和幅度
 *     800ms+ 移动距离明显，注意车会跑远
 */
#define MOTOR_TEST_PULSE_MS     300U

/* 测试用的 PWM 占空比（-100 ~ +100）。给 70 兼顾力矩和安全性 */
#define MOTOR_TEST_SPEED        70

/*
 * 左右轮映射。
 *
 * 如果实车方向是反的（发 W 车倒着走），把这两个宏互换即可：
 *     #define MOTOR_TEST_LEFT   MOTOR_B
 *     #define MOTOR_TEST_RIGHT  MOTOR_A
 */
#define MOTOR_TEST_LEFT         MOTOR_A
#define MOTOR_TEST_RIGHT        MOTOR_B

/* ------------------------------------------------------------------------- */
/* 任务内部状态                                                                */
/* ------------------------------------------------------------------------- */

static TaskHandle_t s_task_handle = NULL;

/* 上一次的连接状态，用于检测变化。0xFF 表示尚未初始化 */
static uint8_t  s_last_connected = 0xFFU;
static uint32_t s_connect_count  = 0U;

/* ------------------------------------------------------------------------- */
/* 内部辅助                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * 统一的任务内打印入口。
 *
 * 走 USART2，所以同时出现在：
 *   - USB-TTL 串口助手（旁听 PA3）
 *   - 手机蓝牙串口 App
 *
 * 注意：这是阻塞发送，超时 100ms。在任务上下文里可以接受
 *      （优先级 2，低于控制任务 3，不会抢占实时控制）。
 */
static void BleTask_Print(const char *fmt, ...)
{
    char    buf[192];
    va_list args;
    int     len;

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0)
    {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
    }
}

/* ------------------------------------------------------------------------- */
/* 电机测试                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * 给左右轮一个短脉冲：通电 MOTOR_TEST_PULSE_MS 毫秒后制动停止。
 *
 *   left > 0 : 左轮正转（车前进方向）
 *   right> 0 : 右轮正转
 *
 * 为什么用 Motor_Brake() 而不是 Motor_SetSpeed(0,0)：
 *   SetSpeed(0,0) 让 H 桥断开，电机滑行，轮子还会惯性转一会儿；
 *   制动则把电机两端短接，停得干脆，方向判断更清晰。
 */
static void Motor_Pulse(int16_t left, int16_t right)
{
    /* 先确保上一次的制动状态被解除，再给新方向 */
    Motor_SetSpeed(left, right);

    vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_PULSE_MS));

    Motor_Brake();
}

/*
 * 处理一个方向命令字符。
 * 返回 1 = 已识别并执行；0 = 不认识的字符（忽略）。
 */
static uint8_t BleTask_HandleMotionCmd(char ch)
{
    switch (ch)
    {
    case 'W':
    case 'w':
        /* 前进：两轮同向正转 */
        Motor_Pulse(MOTOR_TEST_SPEED, MOTOR_TEST_SPEED);
        BleTask_Print("W -> forward %ums\r\n", (unsigned int)MOTOR_TEST_PULSE_MS);
        return 1U;

    case 'S':
    case 's':
        /* 后退：两轮同向反转 */
        Motor_Pulse(-MOTOR_TEST_SPEED, -MOTOR_TEST_SPEED);
        BleTask_Print("S -> backward %ums\r\n", (unsigned int)MOTOR_TEST_PULSE_MS);
        return 1U;

    case 'A':
    case 'a':
        /* 左转：左轮反转 + 右轮正转 = 原地逆时针旋转 */
        Motor_Pulse(-MOTOR_TEST_SPEED, MOTOR_TEST_SPEED);
        BleTask_Print("A -> turn left %ums\r\n", (unsigned int)MOTOR_TEST_PULSE_MS);
        return 1U;

    case 'D':
    case 'd':
        /* 右转：左轮正转 + 右轮反转 = 原地顺时针旋转 */
        Motor_Pulse(MOTOR_TEST_SPEED, -MOTOR_TEST_SPEED);
        BleTask_Print("D -> turn right %ums\r\n", (unsigned int)MOTOR_TEST_PULSE_MS);
        return 1U;

    case 'X':
    case 'x':
        /* 急停：立即制动，不脉冲 */
        Motor_Brake();
        BleTask_Print("X -> brake\r\n");
        return 1U;

    default:
        return 0U;
    }
}

/* ------------------------------------------------------------------------- */
/* 任务主体                                                                    */
/* ------------------------------------------------------------------------- */

static void BleTask_Main(void *argument)
{
    (void)argument;

    /* 让出一次 CPU，确保串口和 LED 初始化完成后再开始打印 */
    vTaskDelay(pdMS_TO_TICKS(100U));

    /*
     * 进入电机手动模式：让 ControlTask 停止写电机，
     * 由本任务独占控制。这样 PID 完全停摆，纯测驱动。
     */
    Motor_Brake();                  /* 先停稳，避免上电瞬间乱转 */
    g_motor_manual_mode = 1U;

    BleTask_Print("\r\n[BleTask] started, period=%ums\r\n",
                  (unsigned int)BLE_TASK_PERIOD_MS);
    BleTask_Print("[BleTask] MOTOR TEST MODE (PID disabled)\r\n");
    BleTask_Print("[BleTask] speed=%d%%  pulse=%ums\r\n",
                  (int)MOTOR_TEST_SPEED,
                  (unsigned int)MOTOR_TEST_PULSE_MS);
    BleTask_Print("[BleTask] W=forward S=backward A=left D=right X=brake\r\n");

    /* 同步一次连接状态，避免把"上电时已连接"误报成一次新连接 */
    s_last_connected = BLE_IsConnected();

    for (;;)
    {
        /*
         * ---- 电机测试命令 ----
         *
         * 按【字节】取，因为 W/S/A/D 都是单字符命令，不需要回车。
         * 手机 App 如果自动补 \r\n，这两个字符会被 HandleMotionCmd
         * 忽略（返回 0），不影响功能。
         *
         * 每个命令产生一个 MOTOR_TEST_PULSE_MS 的短脉冲后自动制动，
         * 所以车不会一直跑——适合台面测试方向。
         */
        {
            uint8_t b;

            while (BLE_GetByte(&b) != 0U)
            {
                (void)BleTask_HandleMotionCmd((char)b);
            }
        }

        /* ---- 连接状态变化：更新 LED 并提示 ---- */
        {
            uint8_t connected = BLE_IsConnected();

            if (connected != s_last_connected)
            {
                s_last_connected = connected;

                if (connected != 0U)
                {
                    s_connect_count++;
                    BleTask_Print("[BLE] connected (times=%lu)\r\n",
                                  (unsigned long)s_connect_count);
                    BleTask_Print("[BLE] send W/S/A/D to drive the car\r\n");
                }
                else
                {
                    BleTask_Print("[BLE] disconnected\r\n");
                }
            }

            /* 未连接慢闪 / 已连接常亮。内部只在状态变化时重设 LED */
            BLE_UpdateLed();
        }

        vTaskDelay(pdMS_TO_TICKS(BLE_TASK_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------------- */
/* 任务创建                                                                    */
/* ------------------------------------------------------------------------- */

uint8_t BleTask_Init(void)
{
    BaseType_t ret;

    if (s_task_handle != NULL)
    {
        return 1U;      /* 已经创建过，避免重复创建 */
    }

    ret = xTaskCreate(BleTask_Main,
                      "BleTask",
                      BLE_TASK_STACK_WORDS,
                      NULL,
                      BLE_TASK_PRIORITY,
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
