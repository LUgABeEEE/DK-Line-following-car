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
#include "Control_task.h"   /* ControlTask_RequestStop() */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 串口句柄定义在 main.c 中 */
extern UART_HandleTypeDef huart2;

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
/* 任务主体                                                                    */
/* ------------------------------------------------------------------------- */

static void BleTask_Main(void *argument)
{
    (void)argument;

    /* 让出一次 CPU，确保串口和 LED 初始化完成后再开始打印 */
    vTaskDelay(pdMS_TO_TICKS(100U));

    /*
     * 注意：本任务【不再】驱动电机。
     * 电机完全由 ControlTask 的内环/外环管理，
     * 这里只保留一个"停车"命令作为安全兜底。
     */
    BleTask_Print("\r\n[BleTask] started, period=%ums\r\n",
                  (unsigned int)BLE_TASK_PERIOD_MS);
    BleTask_Print("[BleTask] motor control: NONE (handled by ControlTask)\r\n");
    BleTask_Print("[BleTask] send X = EMERGENCY STOP\r\n");

    /* 同步一次连接状态，避免把"上电时已连接"误报成一次新连接 */
    s_last_connected = BLE_IsConnected();

    for (;;)
    {
        /*
         * ---- 蓝牙命令：只保留急停 ----
         *
         * 按【字节】取。识别 X / S / Q 三个字符任一即停车，
         * 覆盖不同 App 的习惯。
         *
         * 实现上只切模式（ControlTask_RequestStop），不直接调
         * Speed_PID_StopAll()——因为控制任务优先级更高，
         * 直接 Stop 会被它下一拍用旧目标覆盖回去。
         */
        {
            uint8_t b;

            while (BLE_GetByte(&b) != 0U)
            {
                switch ((char)b)
                {
                case 'X':
                case 'x':
                case 'S':
                case 's':
                case 'Q':
                case 'q':
                    ControlTask_RequestStop();
                    BleTask_Print("[BLE] EMERGENCY STOP\r\n");
                    break;

                default:
                    /* 其它字符忽略（包括 App 自动补的 \r \n） */
                    break;
                }
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
