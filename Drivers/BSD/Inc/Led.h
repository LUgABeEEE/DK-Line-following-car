//
// Created by JiangyaF on 2026/10/11.
//
// 板载 LED 驱动（LED1 = PB5, LED2 = PB4）
//
// 极性：CubeMX 里两盏 LED 的初始电平都是 GPIO_PIN_SET（高），
//       也就是说上电默认是"灭"的，由此可知是【低电平点亮】（灌电流接法）。
//       如果你的板子实际相反，把 LED_ACTIVE_LOW 改成 0 即可，本文件其余部分不用动。
//
// GPIO 本身的初始化由 CubeMX 生成的 MX_GPIO_Init() 完成，
// 本模块不做硬件初始化，只负责逻辑。
//
// 用法：
//   Led_Init();
//   Led_On(LED1);
//   Led_BlinkForever(LED2, 500, 500);   // 500ms 亮 500ms 灭
//   while (1) {
//       Led_Refresh();                  // 必须周期性调用，闪烁才动得起来
//   }
//

#ifndef LINEFLLOWING_CAR_LED_H
#define LINEFLLOWING_CAR_LED_H

#include "main.h"

/* ------------------------------------------------------------------------- */
/* 配置                                                                        */
/* ------------------------------------------------------------------------- */

/* 1 = 低电平点亮（本板默认）；0 = 高电平点亮 */
#define LED_ACTIVE_LOW          1

/* 灯的数量 */
#define LED_COUNT               2U

/* 编号：与 main.h 的 LED1 / LED2 对应 */
#define LED_1                   0U      /* PB5 */
#define LED_2                   1U      /* PB4 */

/*
 * Led_Refresh() 的调用周期，单位 ms。
 * 只用于估算"从未调用过 Refresh"的情况，实际闪烁基于 HAL_GetTick() 时间戳，
 * 所以调用周期不必严格等于本值，但应小于最短的亮/灭时间。
 */
#define LED_REFRESH_PERIOD_MS   20U

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/* 初始化：把两盏灯都关掉，清空闪烁状态。请在 MX_GPIO_Init() 之后调用一次。 */
void Led_Init(void);

/* 点亮 / 熄灭指定 LED。会同时取消该灯的闪烁。 */
void Led_On(uint32_t led);
void Led_Off(uint32_t led);

/* 翻转指定 LED 的状态。会取消闪烁（否则会和闪烁逻辑抢控制权）。 */
void Led_Toggle(uint32_t led);

/* 查询当前是否点亮。 */
uint8_t Led_IsOn(uint32_t led);

/*
 * 闪烁指定次数后自动熄灭。
 *   on_ms      : 每个周期里"亮"的持续时间，单位 ms
 *   period_ms  : 一个完整周期（亮 + 灭）的时间，单位 ms
 *   count      : 闪烁次数；传 0 表示无限循环
 *
 * 例：Led_BlinkCount(LED1, 100, 200, 3) -> 亮100灭100，重复3次后停止
 */
void Led_BlinkCount(uint32_t led, uint32_t on_ms, uint32_t period_ms, uint32_t count);

/* 无限闪烁。等价于 count 传 0。 */
void Led_BlinkForever(uint32_t led, uint32_t on_ms, uint32_t period_ms);

/*
 * 闪烁刷新：必须周期性调用（建议 20ms 一次，放在主循环里）。
 * 用时间戳计算，不做延时，不会阻塞。
 */
void Led_Refresh(void);

/* 把所有灯都关掉并停止闪烁。 */
void Led_AllOff(void);

/*
 * 自检：阻塞约 6 秒，依次点亮 LED1、LED2、交替闪、同步闪。
 * 用于确认极性（如果"点亮"看起来是灭的，就把 LED_ACTIVE_LOW 改成 0）。
 */
void Led_TestSequence(void);

#endif //LINEFLLOWING_CAR_LED_H
