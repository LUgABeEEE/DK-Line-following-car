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

/* 读一个点的当前颜色。 */
uint8_t OLED_GetPoint(uint8_t x, uint8_t y);

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

/*
 * 显示有符号整数。
 *   num    : 数值
 *   len    : 最小位宽，不足时左边补空格
 * 返回下一个字符的 x 坐标。
 */
uint8_t OLED_ShowNumber(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t size, uint8_t color);

/*
 * 显示定点小数，例如 OLED_ShowFloat(x, y, 30.567f, 2, 6, ...) 显示 "30.57"。
 *   decimals : 小数位数，建议不超过 3
 */
uint8_t OLED_ShowFloat(uint8_t x, uint8_t y, float value, uint8_t decimals, uint8_t len,
                       uint8_t size, uint8_t color);

/* ---- 本项目专用：把整车状态一次画出来 ---- */

/*
 * 显示一屏调试信息。请每 100ms 左右调用一次（和外环同频即可）。
 *
 * 内容布局（6x8 字体，8 行 x 21 列）：
 *   行0: 蓝牙连接状态
 *   行1: 灰度位置 pos / 丢线标志
 *   行2: 循迹误差 err / 差速量 steer
 *   行3: 左轮 目标 / 实测 (cm/s)
 *   行4: 右轮 目标 / 实测 (cm/s)
 *   行5: 外环 PD 参数
 *   行6: 内环 PID 参数
 *   行7: 状态提示
 *
 * pd 传主程序里的 Line_PD_t 实例地址。本函数只读各模块的查询接口，
 * 不改变任何控制状态，可以放心随时调用。
 */
void OLED_ShowDashboard(const Line_PD_t *pd);

/*
 * 自检：阻塞约 5 秒，依次显示
 *   1. 全屏点亮（检查坏点）
 *   2. 边框 + 对角线
 *   3. ASCII 字符表
 *   4. 两种字体的示例文字
 * 用于确认 I2C 通路、寻址模式、坐标方向是否正确。
 */
void OLED_TestSequence(void);

#endif //LINEFLLOWING_CAR_OLED_H
