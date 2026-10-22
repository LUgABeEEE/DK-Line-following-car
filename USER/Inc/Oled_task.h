//
// Created by JiangyaF on 2026/10/22.
//
// OLED 显示任务（FreeRTOS）
//
// 功能：
//   1. 在 0.96 寸 SSD1306 OLED 上显示字母和数字
//   2. KEY1 每次按下 -> 计数值 +1
//   3. KEY2 每次按下 -> 计数值 -1
//   4. 用【计数信号量】记录按键事件，防止两次任务循环之间连按多次时丢事件
//
// 为什么用计数信号量而不是二值信号量：
//   二值信号量只能记 0 或 1。如果按键在 100ms 内被连按 3 次，
//   二值信号量只会记住"发生过"，剩下 2 次凭空丢失。
//   计数信号量的计数值 = 未处理的事件个数，任务每次循环取走当前计数
//   并一次性加上，就能把连按全部算进去。
//
// 数据流：
//   Key_Scan()（在控制任务里，20ms 一次）产生 PRESS 事件
//        ↓ 读到 KEY1/KEY2 的 PRESS
//   OledTask_NotifyKeyPress()  -> xSemaphoreGive()（计数 +1）
//        ↓
//   OledTask_Main()（本任务，100ms 一次）
//        ↓ xSemaphoreTake(..., 0) 取走一个
//        ↓ 把信号量的当前计数一次取空，累加到计数值
//   OLED_ShowNumber() -> OLED_Refresh()
//
// 任务参数：
//   优先级 1（低于控制任务 3 和蓝牙任务 2，显示不影响实时性）
//   栈     320 字 = 1280 字节
//   周期   100ms
//

#ifndef LINEFOLLOWING_CAR_OLED_TASK_H
#define LINEFOLLOWING_CAR_OLED_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------------- */
/* 任务参数                                                                    */
/* ------------------------------------------------------------------------- */

#define OLED_TASK_PRIORITY          1U
#define OLED_TASK_STACK_WORDS       320U
#define OLED_TASK_PERIOD_MS         100U

/*
 * 计数信号量的最大计数。
 *
 * 这个值决定了"两次任务循环之间最多能记住多少次按键"。
 * 任务 100ms 一次，人最快连按约 5~8 次/秒，所以 16 已经很宽裕。
 * 设得太大没有意义（人按不了那么快），设得太小会在极端连按时饱和。
 */
#define OLED_KEY_SEM_MAX_COUNT      16U

/* 计数值的上下限，防止长时间按住/连按导致溢出显示 */
#define OLED_COUNTER_MIN            (-9999)
#define OLED_COUNTER_MAX            (9999)

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/*
 * 创建 OLED 显示任务，并创建按键计数信号量。
 * 必须在 vTaskStartScheduler() 之前调用。
 *
 * 返回 1 = 成功，0 = 失败（堆不够）。
 */
uint8_t OledTask_Init(void);

/*
 * 上报一次按键按下事件（由按键扫描方调用）。
 *   key : KEY_1 或 KEY_2
 *
 * 内部调用 xSemaphoreGive()，是中断安全的写法吗？——**不是**。
 * 它只能在任务上下文调用（Key_Scan 在控制任务里跑，符合要求）。
 * 如果将来要在中断里调用，请改用 OledTask_NotifyKeyPressFromISR()。
 *
 * 返回值：1 = 已记入队列；0 = 不是这两个键，或信号量已满
 */
uint8_t OledTask_NotifyKeyPress(uint32_t key);

/* 查询当前计数值（调试用）。 */
int32_t OledTask_GetCounter(void);

#endif //LINEFOLLOWING_CAR_OLED_TASK_H
