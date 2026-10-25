//
// Created by JiangyaF on 2026/10/11.
//
// TB6612 双路直流电机驱动（PWM + 方向控制）
//
// 硬件映射（与 linefllowing_car.ioc 保持一致）：
//   电机 A : PWMA = PA9  -> TIM1_CH2    AIN1 = PB13    AIN2 = PB12
//   电机 B : PWMB = PA8  -> TIM1_CH1    BIN1 = PB14    BIN2 = PB15
//   公共   : STBY = PA10（高电平才工作）
//
// 定时器参数：TIM1 -> PSC = 71, ARR = 99  ==>  PWM 频率 = 72MHz/(72*100) = 10kHz
// CubeMX 里已按此配置，main.c 的 MX_TIM1_Init() 也是同样的值。
// 若以后在 CubeMX 里改了 ARR，请同步修改下面的 MOTOR_PWM_PERIOD。
//

#ifndef LINEFLLOWING_CAR_PWM_H
#define LINEFLLOWING_CAR_PWM_H

#include "main.h"
#include "FreeRTOS.h"

/* ------------------------------------------------------------------------- */
/* 可调参数                                                                    */
/* ------------------------------------------------------------------------- */

/* TIM1 的自动重装载值 ARR。必须与 CubeMX 里 Counter Period 填的数一致。 */
#define MOTOR_PWM_PERIOD        99U

/* 占空比行程：-MOTOR_DUTY_MAX ~ +MOTOR_DUTY_MAX，符号代表方向。 */
#define MOTOR_DUTY_MAX          100

/* 电机编号 */
#define MOTOR_A                 0U
#define MOTOR_B                 1U

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/* 启动两路 PWM 并把 TB6612 置为使能状态。请在 MX_TIM1_Init() 之后调用一次。 */
void Motor_Init(void);

/*
 * 设置两个电机的转速。
 *   duty > 0 : 正转，数值越大越快
 *   duty < 0 : 反转
 *   duty == 0: 滑行停止（H 桥断开，靠惯性溜车）
 * 传入值会自动限幅到 ±MOTOR_DUTY_MAX，越界不会撞坏寄存器。
 */
void Motor_SetSpeed(int16_t duty_a, int16_t duty_b);

/* 单电机版本，motor 取 MOTOR_A / MOTOR_B。 */
void Motor_SetOne(uint32_t motor, int16_t duty);

/* 刹车停止：电机两端被短接，电机立刻抱死。 */
void Motor_Brake(void);

/* 滑行停止：等价于 Motor_SetSpeed(0, 0)。 */
void Motor_Coast(void);

/* TB6612 总使能。enable != 0 -> 正常工作；enable == 0 -> 待机（电机无力）。 */
void Motor_Standby(uint8_t enable);

/*
 * 自检序列：正转缓升 -> 滑行 -> 反转缓升 -> 刹车 -> 停止 -> 待机。
 * 用于裸机阶段验证硬件接线，全程约 10 秒，会阻塞当前任务。
 * 在 main 的 USER CODE BEGIN 2 里调用一次即可。
 */
void Motor_TestSequence(void);

#endif //LINEFLLOWING_CAR_PWM_H
