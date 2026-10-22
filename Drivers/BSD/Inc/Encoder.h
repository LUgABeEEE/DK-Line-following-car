//
// Created by JiangyaF on 2026/10/11.
//
// 增量式正交编码器读取模块
//
// 硬件映射（与 linefllowing_car.ioc 保持一致）：
//   编码器 E1: PB6 = TIM4_CH1, PB7 = TIM4_CH2
//   编码器 E2: PA6 = TIM3_CH1, PA7 = TIM3_CH2
//
// 定时器参数：TIM3 / TIM4 -> PSC = 49, ARR = 65535
//   APB1 分频为 2，定时器时钟自动 x2 = 72MHz
//   计数时钟 = 72MHz / (49+1) = 1.44MHz  ->  可可靠计数的边沿频率上限约 720kHz
//   ARR 取满量程 65535，对应 16 位计数器，溢出前可累计 65535 个计数
//
// 注意：本模块的编号 ENCODER_A / ENCODER_B 与 PWM.h 的 MOTOR_A / MOTOR_B 对应，
//       即同一颗电机的驱动和编码器用同一个编号，方便 PID 里直接配套使用。
//

#ifndef LINEFLLOWING_CAR_ENCODER_H
#define LINEFLLOWING_CAR_ENCODER_H

#include "main.h"

/* ------------------------------------------------------------------------- */
/* 机械参数 —— 必须按你的实际电机和轮子改，否则速度值不准                        */
/* ------------------------------------------------------------------------- */

/* 编码器线数（每转脉冲数，通常标在电机铭牌上，常见 11 / 13 / 20） */
#define ENCODER_PPR                 13.0f

/* 减速比。写在电机型号末尾，比如 "MG513P30" 就是 30 */
#define ENCODER_GEAR_RATIO          30.0f

/* 车轮直径，单位 mm。拿卡尺量，或用下面的标定方法反推 */
#define ENCODER_WHEEL_DIAMETER_MM   65.0f

/*
 * 标定技巧：如果速度值整体偏大或偏小，不要去猜直径，直接用比例修正。
 * 例如让车直行 2 秒、实测走了 40cm，而程序算出 44cm，就把下面的系数设成 40.0f/44.0f。
 */
#define ENCODER_CALIBRATION         1.0f

/* x4 模式（TIM1+TI2 双边沿）时为 4，x2 模式（仅 TI1）时为 2 */
#define ENCODER_EDGE_MULTIPLIER     4.0f

/* ------------------------------------------------------------------------- */
/* 换算常量 —— 由上面的机械参数自动推导，一般不用手改                            */
/* ------------------------------------------------------------------------- */

/* 输出轴（轮子）转一圈的计数：线数 x 倍频 x 减速比 */
#define ENCODER_COUNTS_PER_REV      (ENCODER_PPR * ENCODER_EDGE_MULTIPLIER * ENCODER_GEAR_RATIO)

/* 轮子周长，单位 mm */
#define ENCODER_WHEEL_CIRC_MM       (3.14159265f * ENCODER_WHEEL_DIAMETER_MM)

/* 一个计数对应的行驶距离，单位 mm */
#define ENCODER_MM_PER_COUNT        (ENCODER_WHEEL_CIRC_MM / ENCODER_COUNTS_PER_REV * ENCODER_CALIBRATION)

/* ------------------------------------------------------------------------- */
/* 采样参数                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * 速度计算的时间窗口，单位 ms。
 * 这是"分辨率"和"响应速度"的权衡：
 *   窗口太短 -> 低速时 delta 只有几个计数，量化误差大，PID 会抖
 *   窗口太长 -> 速度反馈滞后，PID 响应变慢
 * 10ms 时每个计数约代表 2cm/s；若觉得低速抖动明显，加大到 20~50ms。
 */
#define ENCODER_PERIOD_MS           20U

/* 速度值的一阶低通滤波系数，范围 (0, 1]：越小越平滑但越滞后，1 表示不滤波 */
#define ENCODER_SPEED_FILTER        0.5f

/* 编码器编号（与 PWM.h 的 MOTOR_A / MOTOR_B 一致） */
#define ENCODER_A                   0U      /* E1: PB6/PB7 -> TIM4 */
#define ENCODER_B                   1U      /* E2: PA6/PA7 -> TIM3 */

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/*
 * 使能两路编码器计数（内部调用 HAL_TIM_Encoder_Start）。
 * 请在 MX_TIM3_Init() / MX_TIM4_Init() 之后调用一次。
 * 注意：CubeMX 生成代码不会自动启动编码器，漏掉这句计数器会永远为 0。
 */
void Encoder_Init(void);

/*
 * 复位"上次采样值"基准，并把计数器和滤波状态清零。
 * 在开始一次新的速度测量前调用，避免把上电以来的累计计数当成速度。
 */
void Encoder_Reset(void);

/* 读取计数器原始值（0~65535，回绕）。调试和标定时用。 */
uint16_t Encoder_GetCount(uint32_t encoder);

/* 读取自上次调用以来的计数增量，自带 16 位回绕处理。 */
int16_t Encoder_GetDelta(uint32_t encoder);

/* 清空某个编码器的滤波状态（换向、急停后想立刻重新读速度时用）。 */
void Encoder_ResetFilter(uint32_t encoder);

/*
 * 按 ENCODER_PERIOD_MS 计算两轮速度，单位 cm/s，带符号（正转为正）。
 * 调用节奏很关键：必须固定按 ENCODER_PERIOD_MS 的周期调用，
 * 且两次调用之间的时间就是速度的积分窗口。放进一个 20ms 周期的任务里最合适。
 */
void Encoder_UpdateSpeed(void);

/* 取最近一次 Encoder_UpdateSpeed() 算出的速度，单位 cm/s。 */
float Encoder_GetSpeed(uint32_t encoder);

/* 取累计行驶距离，单位 cm，带符号。 */
float Encoder_GetDistance(uint32_t encoder);

/*
 * 自检：阻塞约 6 秒，每 100ms 打印一次两个编码器的原始计数，
 * 最后打印累计变化量。手工拨动轮子即可判断接线和计数方向是否正确。
 * 通过 huart2 输出，波特率需与 MX_USART2_UART_Init() 的配置一致。
 */
void Encoder_TestSequence(void);

#endif //LINEFLLOWING_CAR_ENCODER_H
