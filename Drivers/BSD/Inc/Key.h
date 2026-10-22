//
// Created by JiangyaF on 2026/10/11.
//
// 板载按键驱动（KEY1 = PB0, KEY2 = PA4, KEY3 = PA5）
//
// 硬件：三个按键都是【按下为低电平】，CubeMX 里已启用内部上拉（GPIO_PULLUP），
//       所以不需要外部上拉电阻，松开时读到高电平。
//
// 消抖策略：轮询 + 状态机，不使用外部中断。
//   为什么不用 EXTI 中断？机械按键抖动会产生几十次边沿中断，中断里还得做
//   延时消抖（延时又不能在中断里做）。轮询方案在主循环里顺序执行，时序确定、
//   不占中断资源，也不会因为抖动而爆栈。
//
// 消抖原理（每个按键独立跑一套）：
//   每次扫描读一次引脚，和"稳定态"比较；
//   连续 KEY_DEBOUNCE_SAMPLES 次读到相反值，才认为状态真的变了。
//   中途只要有一次读回原值，计数器就归零重来——这能滤掉短脉冲干扰。
//
// 用法（事件模式）：
//   Key_Init();
//   while (1) {
//       Key_Scan();
//       uint8_t key, event;
//       while (Key_GetEvent(&key, &event)) {     // 把本轮所有事件取完
//           if (key == KEY_1 && event == KEY_EVENT_PRESS) { ... }
//       }
//   }
//
// 用法（回调模式，适合做菜单）：
//   Key_RegisterCallback(Key_OnPress);
//

#ifndef LINEFLLOWING_CAR_KEY_H
#define LINEFLLOWING_CAR_KEY_H

#include "main.h"

/* ------------------------------------------------------------------------- */
/* 配置                                                                        */
/* ------------------------------------------------------------------------- */

/* 1 = 按下为低电平（本板默认，已启用内部上拉）；0 = 按下为高电平 */
#define KEY_ACTIVE_LOW          1

#define KEY_COUNT               3U

/* 编号：与 main.h 的 KEY1 / KEY2 / KEY3 对应 */
#define KEY_1                   0U      /* PB0 */
#define KEY_2                   1U      /* PA4 */
#define KEY_3                   2U      /* PA5 */

/* 消抖：连续读到几次相同值才认账。配合 20ms 扫描周期，约 40ms 消抖时间 */
#define KEY_DEBOUNCE_SAMPLES    2U

/* 长按判定：按下后持续多少毫秒算长按 */
#define KEY_LONG_PRESS_MS       1000U

/*
 * Key_Scan() 的调用周期，单位 ms。
 * 只用于参数合理性检查，实际计时基于 HAL_GetTick() 时间戳，
 * 所以周期略有抖动不影响判定，但应明显小于 KEY_LONG_PRESS_MS。
 */
#define KEY_SCAN_PERIOD_MS      20U

/* ------------------------------------------------------------------------- */
/* 按键事件                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * 一次完整的按下-松开过程会产生最多三个事件，按时间顺序：
 *
 *   按下瞬间      -> KEY_EVENT_PRESS       （每次按下必定产生一次）
 *   按住超过阈值  -> KEY_EVENT_LONG_PRESS  （只产生一次，不重复）
 *   松开瞬间      -> KEY_EVENT_RELEASE     （每次松开必定产生一次）
 *
 * 短按的典型处理：在 RELEASE 里判断"这次按下期间没有触发过长按"，
 * 或者更简单——直接用 PRESS 做单次触发（大多数场景够用）。
 */
#define KEY_EVENT_NONE          0U
#define KEY_EVENT_PRESS         1U
#define KEY_EVENT_LONG_PRESS    2U
#define KEY_EVENT_RELEASE       3U

/* 事件回调函数类型 */
typedef void (*Key_Callback_t)(uint32_t key, uint8_t event);

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/* 初始化：读取当前引脚电平作为初始稳定态，清空所有状态。
   请在 MX_GPIO_Init() 之后调用一次。 */
void Key_Init(void);

/*
 * 扫描一次所有按键，更新消抖状态并产生事件。
 * 请按 KEY_SCAN_PERIOD_MS 周期调用（建议放在主循环里，和 Led_Refresh 一起）。
 */
void Key_Scan(void);

/*
 * 取出一个待处理事件。
 *   key   : 输出，事件来自哪个按键（KEY_1 / KEY_2 / KEY_3）
 *   event : 输出，事件类型（KEY_EVENT_xxx）
 * 返回值：1 = 取到了事件；0 = 没有待处理事件了
 *
 * 典型用法是 while (Key_GetEvent(&k, &e)) { ... }，把本轮事件全部处理完。
 */
uint8_t Key_GetEvent(uint32_t *key, uint8_t *event);

/* 查询某个按键当前是否处于按下状态（已消抖）。 */
uint8_t Key_IsPressed(uint32_t key);

/* 查询某个按键已连续按住了多少毫秒；未按下时返回 0。 */
uint32_t Key_GetHoldTime(uint32_t key);

/*
 * 注册回调。注册后，事件除了进队列，还会额外调用一次回调。
 * 传 NULL 表示取消回调。队列机制不受影响，两种方式可以同时用。
 */
void Key_RegisterCallback(Key_Callback_t callback);

/* 丢弃所有待处理事件（例如状态切换后不想响应旧按键）。 */
void Key_ClearEvents(void);

/*
 * 自检：阻塞约 20 秒，实时打印三个按键的原始电平、消抖后状态、
 * 按住时长和产生的事件。用于确认接线、极性和消抖是否正常。
 * 串口波特率 115200。
 */
void Key_TestSequence(void);

#endif //LINEFLLOWING_CAR_KEY_H
