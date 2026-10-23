//
// Created by JiangyaF on 2026/10/22.
//
// OLED 显示 + PID 按键调参任务（FreeRTOS）
//
// 功能：
//   1. 在 0.96 寸 SSD1306 OLED 上显示当前调参状态与实时速度
//   2. KEY3 切换要调节的参数（KP / KI / KD / KFF / TARGET）
//   3. KEY1 每按一下参数减小一个步长
//   4. KEY2 每按一下参数增大一个步长
//   5. 用【计数信号量】累计按键事件，防止两次任务循环之间连按丢包
//
// 为什么用计数信号量而不是直接读电平：
//   本任务 100ms 才跑一次，而控制任务的按键扫描是 20ms 一次。
//   如果用户在 100ms 内连按 3 下，二值信号量只会记住"有事件"，
//   只加一次，另外两次凭空消失。
//   计数信号量的计数值 = 未处理事件个数，本任务一次取空并逐次应用，
//   连按 3 下就是精确的 3 个步长。
//
// 参数步长为什么不一样：
//   KP/KI 典型量级在 0.5~3，步长 0.1 大约是 3%~20%，够用。
//   KD 典型量级在 0.01~0.1，用 0.1 的步长一下就从 0 跳到 0.1，
//   根本没法细调，所以单独用 0.01。
//   KFF 通常是 0.5~1.5，用 0.05 起步比较平滑。
//   TARGET 是 cm/s，用 1.0 一次一档最直观。
//
// 数据流：
//   Key_Scan()（控制任务，20ms）产生 PRESS 事件
//        ↓ OledTask_NotifyKeyPress()
//   xSemaphoreGive()  计数 +1
//        ↓
//   OledTask_Main()（本任务，100ms）
//        ↓ xSemaphoreTake(..., 0) 逐个取走
//   Speed_PID_SetGain() / g_speed_test_target 更新
//        ↓
//   OLED 刷新
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
/*
 * 任务栈深度（字）。320 字 = 1280 字节。
 *
 * 为什么从 320 加到 448：
 *   调参路径里连续用到 dbg[96] + dbg2[128] 两个临时缓冲，
 *   再加 snprintf 自身约 300 字节的内部用量，320 字已接近危险边缘。
 *   本项目 configCHECK_FOR_STACK_OVERFLOW = 0，溢出不报警只会静默
 *   破坏相邻内存（表现为变量莫名其妙不变），所以加一点余量。
 *
 * 448 字 = 1792 字节，堆里还够用。
 */
#define OLED_TASK_STACK_WORDS       448U
#define OLED_TASK_PERIOD_MS         100U

/*
 * 计数信号量的最大计数。
 *
 * 这个值决定了"两次任务循环之间最多能记住多少次按键"。
 * 任务 100ms 一次，人最快连按约 5~8 次/秒，所以 16 已经很宽裕。
 */
#define OLED_KEY_SEM_MAX_COUNT      16U

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/*
 * 创建 OLED 显示 + 调参任务，并创建三个按键计数信号量。
 * 必须在 vTaskStartScheduler() 之前调用。
 *
 * 返回 1 = 成功，0 = 失败（堆不够）。
 */
uint8_t OledTask_Init(void);

/*
 * 上报一次按键按下事件（由按键扫描方调用）。
 *   key : KEY_1 / KEY_2 / KEY_3
 *
 * 内部调用 xSemaphoreGive()，不是中断安全的写法。
 * 只能在任务上下文调用（Key_Scan 在控制任务里跑，符合要求）。
 *
 * 返回值：1 = 已记入队列；0 = 不认识的键，或信号量已满
 */
uint8_t OledTask_NotifyKeyPress(uint32_t key);

/* 查询当前选中的参数编号（0=KP,1=KI,2=KD,3=KFF,4=TARGET） */
uint8_t OledTask_GetParamIndex(void);

/* 查询当前选中参数的值 */
float OledTask_GetParamValue(void);

#endif //LINEFOLLOWING_CAR_OLED_TASK_H
