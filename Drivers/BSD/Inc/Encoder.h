//
// Created by JiangyaF on 2026/10/11.
//
// 增量式正交编码器读取模块
//
// 硬件映射（与 linefllowing_car.ioc 保持一致）：
//   编码器 E1: PB6 = TIM4_CH1, PB7 = TIM4_CH2
//   编码器 E2: PA6 = TIM3_CH1, PA7 = TIM3_CH2
//
// 定时器参数：TIM3 PSC = 49 / TIM4 PSC = 0，ARR = 65535
//   ⚠️ 两路预分频当前不一致（见 README「已知问题」）：编码器模式下 PSC 对边沿
//      直接分频，PSC=49 会让该路每 50 个边沿才计 1 个数，左右轮速度不可比。
//      正确做法是两路都设 PSC = 0，此时每轮计数 = 线数 × 4 × 减速比。
//   ARR 取满量程 65535，对应 16 位计数器，溢出前可累计 65535 个计数
//
// 注意：本模块的编号 ENCODER_A / ENCODER_B 与 PWM.h 的 MOTOR_A / MOTOR_B 对应，
//       即同一颗电机的驱动和编码器用同一个编号，方便 PID 里直接配套使用。
//

#ifndef LINEFLLOWING_CAR_ENCODER_H
#define LINEFLLOWING_CAR_ENCODER_H

#include "main.h"

/* ------------------------------------------------------------------------- */
/* 机械参数 —— 换电机 / 换轮子只需要改这一段                                    */
/* ------------------------------------------------------------------------- */

/*
 * 【方式 A · 推荐】直接给「轮子转一整圈的计数」
 *
 * 好处：不用管编码器线数、减速比、几倍频，换任何电机都只量一个数：
 *   1) 车架空，串口跑 Encoder_TestSequence()（或看 OLED 上的编码器计数）
 *   2) 在轮子上做个记号，手转【正好一圈】回到原点
 *   3) 把读到的计数增量填到下面
 *
 * 参考量级（7 线磁编码器 × 4 倍频，即 counts/rev = 28 × 减速比）：
 *   1:30 -> 840    1:50 -> 1400   1:100 -> 2800
 *   1:150 -> 4200  1:210 -> 5880  1:298 -> 8344
 *
 * ⚠️ 2800 是按「N20-6V + 7 线编码器 + 1:100 + 43mm 轮」填的占位值，
 *    实车必须手转一圈核对。填错不会飞车，但 cm/s 会整体按错误比例缩放，
 *    速度环前馈 KFF 也跟着错位。
 */
#define ENCODER_COUNTS_PER_WHEEL_REV    2800.0f

/*
 * 车轮直径，单位 mm。N20 常用 43 mm。
 * 更准的做法：让轮子在地上滚一整圈，量走过的直线距离再除以 π（含轮胎压缩量）。
 */
#define ENCODER_WHEEL_DIAMETER_MM       43.0f

/*
 * 标定技巧：如果速度值整体偏大或偏小，不要去猜直径，直接用比例修正。
 * 例如让车直行 2 秒、实测走了 40cm，而程序算出 44cm，就把下面的系数设成 40.0f/44.0f。
 */
#define ENCODER_CALIBRATION             1.0f

/* ------------------------------------------------------------------------- */
/* 换算常量 —— 由上面的机械参数自动推导，一般不用手改                            */
/* ------------------------------------------------------------------------- */

/*
 * 轮子转一圈的计数。
 *
 * 【方式 B】知道电机型号时也可以按公式写（与方式 A 二选一）：
 *     线数 × 倍频 × 减速比
 * 例：7 线 × 4 × 100 = 2800。驱动用的是 TIM 硬件编码器模式（TI12，x4），
 * 倍频固定为 4，所以只要线数和减速比对，两种写法结果完全一样。
 */
#define ENCODER_COUNTS_PER_REV      (ENCODER_COUNTS_PER_WHEEL_REV)

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
