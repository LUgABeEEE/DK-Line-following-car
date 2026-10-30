//
// Created by JiangyaF on 2026/10/22.
//
// 蓝牙任务（FreeRTOS）—— 骨架版
//
// 职责：
//   1. 持续监视蓝牙连接状态（STATE 引脚），连接/断开时打印提示
//   2. 更新 LED2 指示连接状态（未连接慢闪 / 已连接常亮）
//   3. 留出接收处理的位置，由你填写具体业务逻辑
//
// 硬件：HC-05 模块，接在 USART2 上（PA2/PA3），默认波特率 9600。
//
// ⚠️ 打印与数据共用同一个串口：
//    本任务的调试输出和手机收发的数据都走 USART2，两者会混在一起。
//    实车调试时如果觉得刷屏，可以把 BleTask_Print 的输出关掉。
//
// 任务参数：
//    优先级   : 2（低于控制任务 3，高于空闲）
//    栈       : 320 字 = 1280 字节
//    周期     : 100ms
//
// 可用的驱动接口（详见 Ble.h）：
//    发送：BLE_Send / BLE_SendString / BLE_SendByte
//    接收：BLE_GetByte / BLE_GetLine
//    状态：BLE_IsConnected
//

#ifndef LINEFOLLOWING_CAR_BLE_TASK_H
#define LINEFOLLOWING_CAR_BLE_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------------- */
/* 任务参数                                                                    */
/* ------------------------------------------------------------------------- */

/* 任务优先级。0 最低（空闲），configMAX_PRIORITIES-1 最高 */
/*
 * 周期遥测开关（比赛用）。
 *
 *   1 = 正常：每 200ms 一条 [L]、每 2s 一条 [G]、每 10s 一条 [M]
 *   0 = 静默：只保留事件类输出（[T] 打标、[RACE] det/stop、[PD] 丢线/找回）
 *
 * 为什么静默不是"关任务"：
 *   - 控制环对 BLE 的所有调用都是 xQueueSend(..., 0) 非阻塞，
 *     队列不在或满了都立刻返回 0，所以关不关任务都【不影响控制】
 *   - BLE 任务优先级 2 < 控制任务 3，永远不会抢占控制环
 *   - 但一旦关掉任务，比赛现场出问题就彻底没有日志可查
 * 所以默认只静默周期输出，事件类保留。
 */
#define BLE_PERIODIC_LOG_ENABLE     0

#define BLE_TASK_PRIORITY           2U

/*
 * 任务栈深度，单位【字】（Word，不是字节）。320 字 = 1280 字节。
 *
 * 依据实测（-fstack-usage 逐函数栈帧）：
 *   BleTask_Main(88) + BleTask_Print(208) + vsnprintf(100)
 *   + HAL_UART_Transmit(48) = 444 B，加上 FreeRTOS 上下文帧与中断余量
 *   => 峰值约 600 B。1280 B 预算约有 2 倍余量。
 *
 * ⚠️ 本工程 configCHECK_FOR_STACK_OVERFLOW = 0，栈溢出是静默的，
 *    所以宁可留余量，不要压到刚好够用。
 */
/*
 * 384 字 = 1536 B。
 * 原来 320 字（1280 B），实测 [M] 行给出的历史最小剩余只有 344 B ——
 * 这个任务要做 vsnprintf + 多条最长 72 字节的 [L] 行，余量偏薄；
 * 堆还有 5 KB 空闲，直接加一档，避免偶发栈越界。
 */
#define BLE_TASK_STACK_WORDS        384U

/* 任务主循环周期，单位 ms
 *
 * 【从 100ms 降到 25ms 的原因】
 *   PID 数据每 200ms 输出一次，但检查节拍是 100ms：
 *   一次 62ms 的 [L] 发送 + 100ms 的轮询粒度，实测 [L] 间隔变成
 *   中位 266ms（约 3.8Hz），比设计的 5Hz 少了 1/4，锐角弯只采到 1~2 拍。
 *   降到 25ms 后，[L] 间隔约 200~230ms，接近设计值；
 *   顺带把蓝牙急停命令的响应从"最坏 100ms+"降到 25ms 级。
 */
#define BLE_TASK_PERIOD_MS          25U

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/*
 * 创建蓝牙任务。
 * 请在调度器启动（vTaskStartScheduler）之前、且 BLE_Init() 之后调用。
 *
 * 返回值：
 *   1 = 创建成功
 *   0 = 创建失败（通常是堆不够，xTaskCreate 返回 errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY）
 */
uint8_t BleTask_Init(void);

/* 诊断：本任务栈的历史最小剩余量（单位：字）。任务未创建时返回 0。 */
uint32_t BleTask_GetStackFreeWords(void);

/*
 * 跑圈结果打标（由控制任务在"循迹模式 + MODE 页"下调用）。
 *
 *   ok = 1 -> 串口打印 [T] OK   #n t...   （长按 KEY3）
 *   ok = 0 -> 串口打印 [T] FAIL #n t...   （短按 KEY1）
 *   ok = 2 -> 串口打印 [RACE] det t...    （自动停车：识别到启停线）
 *   ok = 3 -> 串口打印 [RACE] stop t...   （自动停车：已到位）
 *
 * 为什么不在调用处直接发串口：
 *   调用方是控制任务（优先级 3），串口是蓝牙任务的。直接写会打断
 *   蓝牙任务正在发送的一行（半截 [L] 行就是这么来的），所以这里只
 *   往队列里放一个字节，由蓝牙任务统一打印。
 *
 * 返回 1 = 已排队；0 = 队列未创建或已满。
 */
uint8_t BleTask_SendTag(uint8_t ok);

/*
 * 事件码：控制任务把要打印的内容压成 (码, 值1, 值2) 入队，
 * 由蓝牙任务统一格式化输出，避免控制环直接抢 USART2。
 */
#define BLE_EVT_PD_LOST     1U   /* a=丢线里程(cm)                     */
#define BLE_EVT_PD_RECOVER  2U   /* a=搜索耗时(ms)  b=转角(度)         */
#define BLE_EVT_PD_CORNER   3U   /* a=角点类型      b=方向             */

/*
 * 把一条控制事件入队（非阻塞，队列满直接丢弃）。
 * 控制任务专用：不要在这里做任何格式化或串口操作。
 */
uint8_t BleTask_SendEvent(uint8_t code, int16_t a, int16_t b);

/* 已发送的标记个数（从上次上电算起） */
uint16_t BleTask_GetTagCount(void);

#endif //LINEFOLLOWING_CAR_BLE_TASK_H
