//
// Created by JiangyaF on 2026/10/25.
//
// FreeRTOS 钩子函数
//
// 之前这两个钩子是关掉的（configCHECK_FOR_STACK_OVERFLOW = 0 /
// configUSE_MALLOC_FAILED_HOOK = 0），含义是：
//   任务栈溢出、堆分配失败都是【静默】的 —— 内存被悄悄写坏，
//   表现为跑一段时间后开始抖、乱走、偶发死机，而且查不到原因。
//
// 现在都打开：
//   栈溢出/堆失败 -> 立刻刹停电机、点亮 LED 并停机，
//   把一个"玄学抖动"变成"上电就能复现并定位的硬故障"。
//

#include "FreeRTOS.h"
#include "task.h"

#include "PWM.h"
#include "Led.h"

/* ------------------------------------------------------------------------- */
/* 栈溢出                                                                      */
/* ------------------------------------------------------------------------- */
/*
 * 检测方式（configCHECK_FOR_STACK_OVERFLOW = 2）：
 *   任务创建时在栈底写入魔数，每次任务切换检查它是否被改写。
 * 触发时栈内容可能已经不可信，所以这里只做最基本、不依赖栈的操作：
 *   刹停电机、点亮 LED、关中断停机。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    Motor_Brake();          /* 电机短接制动，避免车子失控 */
    Led_On(LED_1);          /* 常亮，肉眼可辨 */

    taskDISABLE_INTERRUPTS();
    for (;;)
    {
        /* 停机等待人工复位/重新上电 */
    }
}

/* ------------------------------------------------------------------------- */
/* 堆分配失败                                                                  */
/* ------------------------------------------------------------------------- */
void vApplicationMallocFailedHook(void)
{
    Motor_Brake();
    Led_On(LED_2);

    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
