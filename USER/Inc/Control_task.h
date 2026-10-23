//
// Created by JiangyaF on 2026/10/22.
//
// 实时控制任务（FreeRTOS）
//
// 职责：把原来主循环里 while(1) 的三个节拍搬进一个任务：
//
//   优先级 3（高于蓝牙任务的 2）——控制是硬实时需求，
//   蓝牙监视和命令解析晚几十毫秒没人会在意，电机晚一拍就会抖。
//
//   节拍 1：内环速度环        20ms   Encoder_UpdateSpeed() + Speed_PID_Update()
//   节拍 2：外环循迹 PD      100ms   LinePD_Update()
//   节拍 3：人机界面          20ms   Led_Refresh() + Key_Scan() + BLE_UpdateLed()
//
// ⚠️ 当前状态：**骨架阶段，电机输出被显式停在 0**。
//    原因：本阶段的目标是验证蓝牙模块，不应该让车动起来。
//    要启用循迹，把 CTRL_TASK_ENABLE_MOTION 改成 1，然后先跑
//    LinePD_VerifySigns() 确认符号（见 LinePD.h）。
//

#ifndef LINEFOLLOWING_CAR_CONTROL_TASK_H
#define LINEFOLLOWING_CAR_CONTROL_TASK_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* ------------------------------------------------------------------------- */
/* 配置                                                                        */
/* ------------------------------------------------------------------------- */

/* 任务优先级。高于蓝牙任务（2），低于将来可能加的传感器任务 */
#define CTRL_TASK_PRIORITY          3U

/*
 * 任务栈，单位【字】。320 字 = 1280 字节。
 *
 * 依据实测（编译时开了 -fstack-usage，逐函数栈帧可查）：
 *
 *   循迹环   48(CtrlTask) + 72(LinePD_Update) + 72(Gray_Update)
 *            + 56(HAL_I2C_Mem_Read) = 248 B
 *   开 OLED  48 + 52(ShowDashboard) + 160(WritePageData) + 40(HAL) = 300 B
 *   再叠加 FreeRTOS 上下文帧(64) + Cortex-M 异常压栈(32) + 中断余量(60)
 *   => 实测峰值约 408 B（含 OLED 时约 460 B）
 *
 * 1280 B 的预算对应 408 B 峰值，余量 872 B（约 3 倍），
 * 既不像 2048 B 那样浪费，也留足了将来加功能的余地。
 */
#define CTRL_TASK_STACK_WORDS       320U

/*
 * 是否让电机真正输出。
 *   0 = 安全模式：每个控制周期都把目标速度锁在 0，车不会动
 *   1 = 正常模式：外环 PD 输出的目标速度会真正驱动电机
 *
 * 首次上电调试蓝牙请保持 0。
 */
#define CTRL_TASK_ENABLE_MOTION     0

/*
 * 电机"手动模式"标志。
 *
 * 背景：
 *   ControlTask 每 20ms 会调用 Speed_PID_Update(MOTOR_x, 0.0f) 把电机
 *   拉回停止。如果蓝牙任务同时用 Motor_SetSpeed() 直接驱动电机，
 *   两者会互相覆盖，表现为"电机抖动/一顿一顿"。
 *
 * 用法：
 *   g_motor_manual_mode = 1  -> ControlTask 完全不碰电机（PID 停摆），
 *                               由蓝牙任务独占控制
 *   g_motor_manual_mode = 0  -> 恢复正常的 PID/循迹控制
 *
 * 定义在 Control_task.c 中。
 */
extern volatile uint8_t g_motor_manual_mode;

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/* 创建实时控制任务。必须在 vTaskStartScheduler() 之前调用。返回 1 = 成功。 */
uint8_t ControlTask_Init(void);

#endif //LINEFOLLOWING_CAR_CONTROL_TASK_H
