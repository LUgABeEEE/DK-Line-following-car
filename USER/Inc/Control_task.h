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

/* ------------------------------------------------------------------------- */
/* 运行模式（运行时可变，供按键/蓝牙切换）                                      */
/* ------------------------------------------------------------------------- */

/*
 * 控制任务的工作模式。
 *
 * 之所以做成运行时变量而不是编译期宏：
 *   测试阶段需要在"停车 / 速度环测试 / 循迹"之间来回切换，
 *   每次改宏都要重新编译烧录，效率太低。
 */
#define CTRL_MODE_STOP          0U      /* 电机停：内环目标锁 0            */
#define CTRL_MODE_SPEED_TEST    1U      /* 只跑内环：两轮追踪固定目标速度   */
#define CTRL_MODE_LINE_FOLLOW   2U      /* 双环串级：外环循迹 + 内环速度    */

extern volatile uint8_t g_ctrl_mode;

/*
 * 速度环测试模式下的目标速度，单位 cm/s。
 *
 * 按键调参时 K5（TARGET）改的就是这个值。
 * 初始值给 20 cm/s 是个折中：太快车会冲出去，太慢编码器分辨率不够。
 */
extern volatile float g_speed_test_target;

/*
 * 目标速度的爬升速率，单位 cm/s²。
 *
 * 为什么需要斜坡：
 *   直接把目标从 0 阶跃到 20 cm/s，误差瞬间就是 20，比例项立刻把输出
 *   推到很大。而硬地面上两轮的静摩擦不同，可能一个先挣脱一个还卡着，
 *   卡住那一路的误差会持续很久——即便有积分分离，两轮启动时刻不同步
 *   依然会造成短暂的差速，表现就是"起步原地转圈"。
 *
 *   加减速斜坡让目标缓缓上升，两轮都在低占空比下慢慢克服静摩擦，
 *   起步过程同步得多。
 *
 * 取值 40 的依据：
 *   20 cm/s 目标约 0.5 秒爬到，既不会肉到影响操控，
 *   又足够让两轮同步启动。
 */
#define SPEED_RAMP_CMS_PER_SEC      40.0f

/* 便捷接口：请求停车 / 请求进入速度环测试 */
void ControlTask_RequestStop(void);

/*
 * 比赛自动停车用的一圈周长（cm）。0 = 关闭自动停车。
 * 由 OLED 的 LAP 参数读写；默认 514cm（2026 H 题椭圆地图实测）。
 */
extern volatile float g_race_lap_cm;
void ControlTask_RequestSpeedTest(float target_cms);

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/* 创建实时控制任务。必须在 vTaskStartScheduler() 之前调用。返回 1 = 成功。 */
uint8_t ControlTask_Init(void);

/*
 * 诊断：任务栈的历史最小剩余量（单位：字，1 字 = 4 字节）。
 * 数值越小越危险；接近 0 就说明栈快不够用了。任务未创建时返回 0。
 * 用来抓"长时间运行后开始抖动"这类内存类问题。
 */
uint32_t ControlTask_GetStackFreeWords(void);

#endif //LINEFOLLOWING_CAR_CONTROL_TASK_H
