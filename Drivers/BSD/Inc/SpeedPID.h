//
// Created by JiangyaF on 2026/10/11.
//
// 内环：左右轮速度 PID 控制器（串级控制的内层）
//
// 在系统中的位置：
//
//   [外环 循迹PD] --目标速度(cm/s)--> [本模块 速度PID] --占空比--> [TB6612] --> 电机
//          ^                                                                     |
//          |                                                                     v
//          +------------------ [灰度模块] <--实际位置-- 车体 <-- [编码器] <--实际速度
//
// 周期约定（非常重要）：
//   本模块的执行周期必须等于 ENCODER_PERIOD_MS，因为速度是"编码器采样窗口内的
//   位移 / 时间"，同时也是 PID 的积分步长。两者不一致会导致 PID 计算错误。
//   外环（循迹）周期应为本模块周期的 3~5 倍。
//
// 调参顺序（务必先把内环调好，再上外环）：
//   第1步 只用前馈 KFF     -> 让实测速度接近目标，说明前馈量级对了
//   第2步 加 Kp            -> 能快速纠正误差
//   第3步 加 Ki            -> 稳态速度精确等于目标值，消除静差
//   第4步 Kd 一般保持 0    -> 电机惯性和编码器滤波已提供足够阻尼
//

#ifndef LINEFLLOWING_CAR_SPEED_PID_H
#define LINEFLLOWING_CAR_SPEED_PID_H

#include "main.h"
#include "PWM.h"
#include "Encoder.h"

/* ------------------------------------------------------------------------- */
/* 控制器内部结构体                                                            */
/* ------------------------------------------------------------------------- */

/*
 * 结构体定义放在头文件是为了让模块内部数组能静态分配，
 * 但请你把它当成"不透明"的类型：不要直接访问它的字段，
 * 参数一律通过 Speed_PID_SetGains() 之类的接口修改。
 * 这样模块内部改实现时不会影响你的代码。
 */
typedef struct
{
    /* ---- 可调参数 ---- */
    float kp;               /* 比例增益，单位：占空比 / (cm/s)                */
    float ki;               /* 积分增益（量纲已内含 1/周期）                  */
    float kd;               /* 微分增益（量纲已内含周期）                     */
    float kff;              /* 前馈增益，单位：占空比 / (cm/s)                */

    /* ---- 限幅 ---- */
    float integral_min;     /* 积分项下限，抗饱和。与上限都设 0 即禁用积分     */
    float integral_max;     /* 积分项上限，抗饱和                             */
    float output_min;       /* 总输出下限（负值对应反转）                     */
    float output_max;       /* 总输出上限                                     */
    float target_limit;     /* 目标速度限幅，单位 cm/s                        */

    /* ---- 微分低通滤波 ---- */
    float d_filter;         /* (0,1]，越小越平滑但越滞后。1.0f = 关闭滤波     */

    /* ---- 运行状态：由 Init/Reset 维护，不要手动改 ---- */
    float   target;         /* 当前目标速度 cm/s                              */
    float   measure;        /* 最近一次实测速度 cm/s                          */
    float   integral;       /* 积分累加量                                     */
    float   d_filt_state;   /* 微分项滤波状态                                 */
    float   output;         /* 最近一次算出的输出（占空比）                   */
    uint8_t saturated;      /* 0=正常，1=正向饱和，2=负向饱和                 */
} Speed_PID_t;

/* ------------------------------------------------------------------------- */
/* 默认参数                                                                    */
/* ------------------------------------------------------------------------- */

/* 速度环执行周期，单位 ms。必须与 ENCODER_PERIOD_MS 保持一致 */
#define SPEED_PID_PERIOD_MS         ENCODER_PERIOD_MS

/* 目标速度限幅，单位 cm/s。防止外环异常时给出离谱目标值 */
#define SPEED_LIMIT_CMS             80.0f

/* PID 输出（占空比）限幅。TB6612 的占空比行程是 ±MOTOR_DUTY_MAX */
#define SPEED_OUTPUT_LIMIT          ((float)MOTOR_DUTY_MAX)

/* 编译期默认增益，改这里即可，也可以用运行时的 Speed_PID_SetGains() 覆盖 */
#define SPEED_PID_DFLT_KP           0.8f
#define SPEED_PID_DFLT_KI           1.5f
#define SPEED_PID_DFLT_KD           0.0f
#define SPEED_PID_DFLT_KFF          1.0f

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/*
 * 标定前馈增益 KFF —— 这是最值得先做的一步。
 *
 * 含义：占空比 ≈ KFF * 目标速度(cm/s)
 * 方法：把 Kp/Ki/Kd 全设 0，只留前馈，给一个固定目标速度跑一下，
 *       若实测偏小（例如目标 30 实测 24），把 KFF 乘以 30/24 ≈ 1.25。
 * 起点：1.0f 对应"30cm/s 给 30% 占空比"；电机偏弱或轮径偏小可加到 1.3~1.6。
 */
void Speed_PID_SetGains(uint32_t motor, float kp, float ki, float kd, float kff);

/*
 * 初始化：把两个轮子的控制器恢复到编译期默认参数并清零全部运行状态。
 * 请在 Encoder_Init() 之后调用一次。
 */
void Speed_PID_Init(void);

/*
 * 复位指定轮子的运行状态：清积分、置首拍标志、清输出。
 * 在"从停止状态重新起步"或"切换目标速度"前调用可避免输出突变。
 */
void Speed_PID_Reset(uint32_t motor);

/*
 * 执行一次 PID 计算并直接驱动电机。请按 SPEED_PID_PERIOD_MS 周期调用。
 *
 *   motor  : MOTOR_A / MOTOR_B
 *   target : 目标速度，cm/s，正值前进
 *
 * 内部完成：读编码器实测速度 -> PID 运算 -> 限幅 -> Motor_SetSpeed()。
 * 返回本次输出的占空比，便于调试打印。
 */
float Speed_PID_Update(uint32_t motor, float target);

/* 取最近一次的目标速度 / 实测速度 / 输出（占空比），供调试和 OLED 显示使用。 */
float Speed_PID_GetTarget(uint32_t motor);
float Speed_PID_GetMeasure(uint32_t motor);
float Speed_PID_GetOutput(uint32_t motor);

/* 立即停止该轮：清积分、清输出，并让该轮滑行。 */
void Speed_PID_Stop(uint32_t motor);

/* 两个轮子都停止，并清除各自的积分状态。 */
void Speed_PID_StopAll(void);

/*
 * 阶跃响应自检：依次给定 20 -> 30 -> 40 -> 0 的目标速度，每档保持约 1 秒，
 * 通过 huart2 打印"目标值 / 实测值 / 输出"三列，用于标定 KFF 和观察超调。
 * 阻塞约 5 秒，需要把轮子架空。串口波特率 115200。
 */
void Speed_PID_TestSequence(void);

#endif //LINEFLLOWING_CAR_SPEED_PID_H
