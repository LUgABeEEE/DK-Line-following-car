//
// Created by JiangyaF on 2026/10/11.
//
// 0.96 寸 SSD1306 OLED 驱动（128x64，4 针 I2C 版）
//
// 硬件（依据板子网表 循迹小车板V1.1）：
//   OLED_SCL -> H2.3 -> PB10 (I2C2_SCL)
//   OLED_SDA -> H2.4 -> PB11 (I2C2_SDA)
//   板载已有 4.7k 上拉（网络 +3.3V），所以直连即可，无需外加电阻。
//
// 模块参数：
//   控制芯片 : SSD1306
//   分辨率   : 128 x 64
//   I2C 地址 : 0x3C（7 位）。厂商例程里写的是 0x78，那是把 7 位地址
//              左移一位后的 8 位写地址，本质是同一个地址。HAL 需要 7 位地址。
//   控制字节 : 0x00 = 后续是命令，0x40 = 后续是数据
//
// 显存组织（本驱动采用页寻址模式）：
//   逻辑坐标以【像素】为单位：x 取 0~127，y 取 0~63。
//   内部按页（page）存放，每页 8 个像素高：
//     页 p 覆盖 y = p*8 ~ p*8+7
//     页内 bit0 是【上方】像素，bit7 是下方像素
//   整屏 8 页 x 128 列 = 1024 字节显存。
//
// 为什么用本地显存 + 整屏刷新：
//   直接往屏上画点需要每点一次 I2C 事务，画一屏要几千次，慢到不可用。
//   本驱动在 RAM 里维护一份 1KB 显存，所有绘制操作只改 RAM，
//   最后由 OLED_Refresh() 一次性推给屏：每页 1 次命令 + 1 次 128 字节数据，
//   共 8 + 8 = 16 次 I2C 事务，约 3~4ms（100kHz 下）。
//
// ⚠️ RAM 开销：1024 字节。占用情况见 OLED_ENABLE_FRAMEBUFFER 的说明。
//

#ifndef LINEFLLOWING_CAR_OLED_H
#define LINEFLLOWING_CAR_OLED_H

#include "main.h"
#include "LinePD.h"     /* 只为拿到 Line_PD_t 的完整定义（dashboard 要用） */

/* ------------------------------------------------------------------------- */
/* 配置                                                                        */
/* ------------------------------------------------------------------------- */

/* SSD1306 的 7 位 I2C 地址 */
#define OLED_I2C_ADDR_7BIT      0x3C

/* 屏幕分辨率 */
#define OLED_WIDTH              128U
#define OLED_HEIGHT             64U

/* 页数 = 高度 / 8 */
#define OLED_PAGE_COUNT         (OLED_HEIGHT / 8U)

/*
 * 单次 I2C 事务超时，单位 ms。
 * 一整页 128 字节在 100kHz 下约 11.5ms（含地址和 ACK），
 * 所以这里给 50ms 而不是像灰度传感器那样给 5ms。
 * 绝不用 HAL_MAX_DELAY——屏没插时会把主循环永久卡死。
 */
#define OLED_I2C_TIMEOUT_MS     50U

/* ------------------------------------------------------------------------- */
/* DMA 配置                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * 1 = 用 DMA 发送（CubeMX 里已给 I2C2 配了 DMA1_Channel4 TX）
 * 0 = 用阻塞式轮询发送（原来的方式）
 *
 * 用 DMA 的好处：
 *   - CPU 不用逐字节喂 I2C，可以去做别的事
 *   - 128 字节一页的传输完全由 DMA 搬运
 *
 * 用 DMA 必须处理两件事（本驱动已处理）：
 *   1. 发送缓冲区必须是【静态】的，不能放栈上——DMA 是异步的，
 *      函数返回后栈会被覆盖，DMA 就会读到垃圾数据
 *   2. 必须等上一笔 DMA 完成才能发下一笔，否则 HAL 返回 HAL_BUSY
 */
#define OLED_USE_DMA            1U

/*
 * 1 = 用 I2C 中断方式发送（HAL_I2C_Master_Transmit_IT）
 *
 * 与 DMA 的区别：
 *   - 数据由 CPU 在中断里逐字节搬运（DMA 由硬件搬运）
 *   - 不需要 DMA 通道，只用到 I2C2_EV 中断
 *   - 比轮询省 CPU（发送期间任务阻塞，CPU 去做别的事）
 *   - 比 DMA 多占用一点 CPU（中断服务程序逐字节处理）
 *
 * 三个模式互斥，优先级：DMA > IT > 轮询。
 * 全部关掉就退化为轮询。
 */
#define OLED_USE_IT             0U

/*
 * 是否使用"异步传输 + 信号量等待"模式（IT 或 DMA 任一开启即为真）
 *
 * 异步模式的两个前提（缺一不可）：
 *   1. I2C2_EV_IRQn 优先级数值 >= 5
 *      否则中断里调 xSemaphoreGiveFromISR 会触发 configASSERT 死机
 *   2. 只能在调度器启动后使用
 *      调度器启动前调用 xSemaphoreTake 会卡死整个系统，
 *      所以 OLED_Init() 阶段强制走轮询（见 Oled.c 里 s_async_ready）
 */
#define OLED_ASYNC_MODE         ((OLED_USE_DMA != 0U) || (OLED_USE_IT != 0U))

/*
 * 1 = 用 FreeRTOS 信号量等待传输完成（任务阻塞，CPU 可做别的事）
 * 0 = 轮询 hi2c2.State 等待完成（不调用任何 FreeRTOS API）
 *
 * 【诊断用】设为 0 可以把"中断传输"和"信号量等待"两件事分开：
 *   - IT=1, SEM=0 : I2C 中断负责搬数据，主循环轮询状态等完成
 *                   全程不调用 xSemaphoreGiveFromISR，不会触发优先级断言
 *   - IT=1, SEM=1 : 中断里给信号量，任务阻塞等待（需要优先级正确）
 *
 * 如果 SEM=0 能工作而 SEM=1 卡死，就说明问题在
 * xSemaphoreGiveFromISR 的优先级校验，而不是 I2C 中断本身。
 */
#define OLED_USE_RTOS_SEM       0U

/*
 * 等待一笔 DMA 传输完成的最长时间，单位 ms。
 * 一页 129 字节 @100kHz 约 12ms，给 50ms 留足余量。
 */
#define OLED_DMA_TIMEOUT_MS     50U

/* 是否启用本地显存。1 = 启用（推荐），0 = 每次绘制直接写屏（极慢，仅调试用） */
#define OLED_ENABLE_FRAMEBUFFER 1

/*
 * 是否把显存放到 CCM/SRAM 之外。F103C8T6 没有 CCM，本宏保留为将来换芯片用。
 * 默认 0：显存放普通 SRAM。
 */
#define OLED_FRAMEBUFFER_CCM    0

/* ------------------------------------------------------------------------- */
/* 颜色与字体                                                                  */
/* ------------------------------------------------------------------------- */

#define OLED_BLACK              0U      /* 熄灭像素 */
#define OLED_WHITE              1U      /* 点亮像素 */

/* 字体高度，传给 OLED_ShowString 用 */
#define OLED_FONT_6X8           8U      /* 宽 6，高 8，一屏可显示 8 行 21 列 */
#define OLED_FONT_8X16          16U     /* 宽 8，高 16，一屏可显示 4 行 16 列 */

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/*
 * 初始化 SSD1306。
 * 请在 MX_I2C2_Init() 之后调用一次。
 * 返回 1 = 成功，0 = 屏无应答（I2C 不通）。
 *
 * 内部流程：等屏上电稳定 -> 发 25 条初始化命令 -> 清屏 -> 开显示。
 * 整个过程约 20ms（含上电等待），是阻塞的，只在启动时调用一次。
 */
uint8_t OLED_Init(void);

/*
 * 通知 OLED 驱动：一笔 I2C DMA 传输已完成。
 *
 * 【必须由 HAL_I2C_MasterTxCpltCallback() 调用】——本驱动不自己定义
 * 那个回调，因为 USE_HAL_I2C_REGISTER_CALLBACKS=0 时 HAL 库自带 __weak
 * 版本，写在本文件里的强定义会被链接器丢弃。
 *
 * 调用方（main.c）的写法：
 *     void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
 *     {
 *         if (hi2c->Instance == I2C2) { OLED_NotifyTxComplete(); }
 *     }
 *
 * 如果在中断上下文调用（这是预期用法），本函数内部用
 * xSemaphoreGiveFromISR，是安全的。
 */
void OLED_NotifyTxComplete(void);

/*
 * 启用异步传输（I2C 中断 或 DMA）。
 *
 * 【必须在 FreeRTOS 调度器启动之后调用】——通常放在显示任务的开头。
 *
 * 为什么不能在 OLED_Init() 里自动启用：
 *   OLED_Init() 在 vTaskStartScheduler() 之前调用，此时调用
 *   xSemaphoreTake() 会让系统卡死（pxCurrentTCB 尚未建立）。
 *   所以初始化阶段强制走轮询，等调度器跑起来后再由任务调用本函数
 *   切到 DMA。
 *
 * 调用前：轮询发送（安全，但 CPU 占用高）
 * 调用后：中断/DMA 发送（任务阻塞等待完成，CPU 可做别的事）
 */
void OLED_EnableAsync(void);

/*
 * 读取异步传输诊断计数（轮询模式下始终为 0）。
 *   ok      : 成功的中断/DMA 传输次数
 *   timeout : 等待完成的超时次数
 *   busy    : HAL 返回 BUSY 的次数
 *
 * 判断标准：
 *   ok 持续增长、timeout=0、busy=0  -> DMA 工作正常
 *   timeout 增长                     -> 中断/回调链有问题
 *   busy 增长                        -> 上一笔没等干净
 */
void OLED_GetAsyncStats(uint32_t *ok, uint32_t *timeout, uint32_t *busy);

/* 清空显存（注意：还需要调用 OLED_Refresh 才会真正清屏）。 */
void OLED_Clear(void);

/* 把显存内容整屏推送到 OLED。这是唯一真正产生 I2C 流量的绘制函数。 */
uint8_t OLED_Refresh(void);

/* 开关显示。0 = 熄屏（芯片仍在工作，显存保留）。 */
uint8_t OLED_DisplayOn(void);
uint8_t OLED_DisplayOff(void);

/* 设置对比度（亮度），0~255。默认 0xCF。 */
uint8_t OLED_SetContrast(uint8_t contrast);

/* 整屏反色 / 正常显示。 */
uint8_t OLED_SetInvert(uint8_t invert);

/*
 * 屏幕旋转 180 度。
 * 模块机械安装方向反了的时候用它，比改接线方便。
 */
uint8_t OLED_SetFlip(uint8_t flip);

/* ---- 基础绘制（只改显存，不产生 I2C 流量） ---- */

/* 画一个点。坐标越界会被安全忽略。 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t color);

/* 画水平线 / 垂直线。 */
void OLED_DrawHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color);
void OLED_DrawVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color);

/* 画矩形：fill = 0 空心，1 实心。 */
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t fill, uint8_t color);

/* 画任意直线（Bresenham 算法）。 */
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);

/* ---- 文字 ---- */

/*
 * 显示一个 ASCII 字符。
 *   x, y   : 左上角坐标（像素）
 *   ch     : 字符，可打印范围 0x20~0x7E
 *   size   : OLED_FONT_6X8 或 OLED_FONT_8X16
 *   color  : OLED_WHITE 或 OLED_BLACK（黑字可用于在实心矩形上"抠字"）
 * 返回下一个字符的 x 坐标，便于连续输出。
 */
uint8_t OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size, uint8_t color);

/*
 * 显示字符串，自动换行（超出右边界回到 x=0 并下移一行）。
 * 返回显示结束后的 y 坐标。
 */
uint8_t OLED_ShowString(uint8_t x, uint8_t y, const char *text, uint8_t size, uint8_t color);

#endif //LINEFLLOWING_CAR_OLED_H
