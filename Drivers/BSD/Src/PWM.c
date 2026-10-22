//
// Created by JiangyaF on 2026/10/11.
//
// TB6612 双路直流电机驱动实现
//
// 控制逻辑（详见 TB6612 手册 表 3-1-2 真值表）：
//   IN1=1, IN2=0, PWM 有效 -> 正转
//   IN1=0, IN2=1, PWM 有效 -> 反转
//   IN1=IN2              -> 制动停止（刹车）
//   PWM=0                -> 滑行停止
//
// 方向由 IN1/IN2 决定，转速由 PWM 占空比决定，两者缺一不可。
//

#include "PWM.h"
#include "stm32f1xx_hal_tim.h"

/* TIM1 句柄定义在 Core/Src/main.c 中，这里引用它。 */
extern TIM_HandleTypeDef htim1;

/* ------------------------------------------------------------------------- */
/* 硬件映射表                                                                  */
/* ------------------------------------------------------------------------- */

typedef struct
{
    TIM_HandleTypeDef *htim;     /* 产生 PWM 的定时器                        */
    uint32_t           channel;  /* PWM 通道                                 */
    GPIO_TypeDef      *dir_port; /* 方向引脚所在端口                         */
    uint16_t           in1_pin;  /* 方向引脚 1                               */
    uint16_t           in2_pin;  /* 方向引脚 2                               */
} Motor_Hw_t;

static const Motor_Hw_t s_motor[] =
{
    /* MOTOR_A : PWMA = PA9 = TIM1_CH2, AIN1 = PB13, AIN2 = PB12 */
    { &htim1, TIM_CHANNEL_2, M_AIN1_GPIO_Port, M_AIN1_Pin, M_AIN2_Pin },

    /* MOTOR_B : PWMB = PA8 = TIM1_CH1, BIN1 = PB14, BIN2 = PB15 */
    { &htim1, TIM_CHANNEL_1, M_BIN1_GPIO_Port, M_BIN1_Pin, M_BIN2_Pin },
};

#define MOTOR_COUNT     (sizeof(s_motor) / sizeof(s_motor[0]))

/* ------------------------------------------------------------------------- */
/* 内部函数                                                                    */
/* ------------------------------------------------------------------------- */

/* 把占空比限幅到 [-MOTOR_DUTY_MAX, +MOTOR_DUTY_MAX]。 */
static int16_t Motor_Clamp(int16_t duty)
{
    if (duty > (int16_t)MOTOR_DUTY_MAX)
    {
        return (int16_t)MOTOR_DUTY_MAX;
    }
    if (duty < -(int16_t)MOTOR_DUTY_MAX)
    {
        return -(int16_t)MOTOR_DUTY_MAX;
    }
    return duty;
}

/* 按占空比符号配置方向引脚，并把幅值写进 CCR。 */
static void Motor_Drive(const Motor_Hw_t *m, int16_t duty)
{
    uint16_t magnitude = (uint16_t)((duty < 0) ? -duty : duty);

    if (duty > 0)
    {
        /* 正转：IN1 = 1, IN2 = 0 */
        HAL_GPIO_WritePin(m->dir_port, m->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->dir_port, m->in2_pin, GPIO_PIN_RESET);
    }
    else if (duty < 0)
    {
        /* 反转：IN1 = 0, IN2 = 1 */
        HAL_GPIO_WritePin(m->dir_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->dir_port, m->in2_pin, GPIO_PIN_SET);
    }
    else
    {
        /* 停止：IN1 = IN2 = 0，H 桥断开，电机滑行 */
        HAL_GPIO_WritePin(m->dir_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->dir_port, m->in2_pin, GPIO_PIN_RESET);
    }

    /* CCR 决定高电平持续多少个计数周期，PWM1 模式下 0 = 全低，PERIOD = 全高 */
    __HAL_TIM_SET_COMPARE(m->htim, m->channel, (uint32_t)magnitude);
}

/* ------------------------------------------------------------------------- */
/* 对外接口                                                                    */
/* ------------------------------------------------------------------------- */

void Motor_Init(void)
{
    uint32_t i;

    /* STBY 拉高，否则 TB6612 处于待机状态，PWM 和方向怎么给都不会转 */
    HAL_GPIO_WritePin(M_STBY_GPIO_Port, M_STBY_Pin, GPIO_PIN_SET);

    /* 两个通道都必须 Start，只启动一个的话另一个电机不会有输出 */
    for (i = 0; i < MOTOR_COUNT; i++)
    {
        HAL_TIM_PWM_Start(s_motor[i].htim, s_motor[i].channel);
    }

    /* 上电先停稳，避免复位瞬间电机乱冲 */
    Motor_Coast();
}

void Motor_SetSpeed(int16_t duty_a, int16_t duty_b)
{
    Motor_Drive(&s_motor[MOTOR_A], Motor_Clamp(duty_a));
    Motor_Drive(&s_motor[MOTOR_B], Motor_Clamp(duty_b));
}

void Motor_SetOne(uint32_t motor, int16_t duty)
{
    if (motor >= MOTOR_COUNT)
    {
        return;
    }
    Motor_Drive(&s_motor[motor], Motor_Clamp(duty));
}

void Motor_Brake(void)
{
    uint32_t i;

    /* IN1 = IN2 = 1 时为制动停止：电机两端被下桥臂短接，反电动势形成制动电流 */
    for (i = 0; i < MOTOR_COUNT; i++)
    {
        __HAL_TIM_SET_COMPARE(s_motor[i].htim, s_motor[i].channel, 0U);
        HAL_GPIO_WritePin(s_motor[i].dir_port, s_motor[i].in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(s_motor[i].dir_port, s_motor[i].in2_pin, GPIO_PIN_SET);
    }
}

void Motor_Coast(void)
{
    Motor_SetSpeed(0, 0);
}

void Motor_Standby(uint8_t enable)
{
    HAL_GPIO_WritePin(M_STBY_GPIO_Port, M_STBY_Pin,
                      (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (enable == 0U)
    {
        Motor_Coast();
    }
}

void Motor_TestSequence(void)
{
    int16_t duty;

    /* 1. 电机 A 正转缓慢加速：0 -> 100 */
    for (duty = 0; duty <= (int16_t)MOTOR_DUTY_MAX; duty += 5)
    {
        Motor_SetSpeed(duty, 0);
        HAL_Delay(60);
    }
    HAL_Delay(500);

    /* 2. 滑行停止，观察轮子靠惯性继续转一会儿 */
    Motor_Coast();
    HAL_Delay(1000);

    /* 3. 电机 A 反转缓慢加速 */
    for (duty = 0; duty >= -(int16_t)MOTOR_DUTY_MAX; duty -= 5)
    {
        Motor_SetSpeed(duty, 0);
        HAL_Delay(60);
    }
    HAL_Delay(500);

    /* 4. 刹车，观察轮子立刻抱死 */
    Motor_Brake();
    HAL_Delay(1000);

    /* 5. 电机 B 正转走一遍，确认第二路通道映射正确 */
    for (duty = 0; duty <= (int16_t)MOTOR_DUTY_MAX; duty += 5)
    {
        Motor_SetSpeed(0, duty);
        HAL_Delay(60);
    }
    HAL_Delay(500);

    /* 6. 全部停止并进入待机 */
    Motor_Coast();
    HAL_Delay(300);
    Motor_Standby(0);
}

//
// Created by JiangyaF on 2026/10/11.
//
