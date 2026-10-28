//
// Created by JiangyaF on 2026/10/11.
// 重写于 2026/10/22：适配 HC-05（原版按 JDY-31 编写，两者协议完全不同）
//
// HC-05 蓝牙串口透传模块驱动
//
// 硬件连接（依据板子网表 循迹小车板V1.1，P1 排针）：
//   模块 TXD   -> P1.3 -> PA3 (USART2_RX)   ← STM32 接收
//   模块 RXD   -> P1.2 -> PA2 (USART2_TX)   ← STM32 发送
//   模块 STATE -> P1.1 -> PA1 (BLE_STATE)   ← 连接状态：未连接低，已连接高
//   模块 KEY   -> P1.6 -> PB3 (BLE_EN)      ← 高电平 = AT 配置模式
//   VCC / GND  -> P1.5(+5V) / P1.4(GND)
//
// 模块参数（HC-05 默认值）：
//   型号       : HC-05（经典蓝牙 SPP 从机）
//   广播名     : HC-05
//   配对码     : 1234（部分固件是 0000）
//   通信波特率 : 9600 8N1
//   AT 模式    : 上电时 KEY 为高电平（或按住模块上的按键再上电）
//
// ⚠️ HC-05 和 JDY-31 的 AT 指令集完全不同：
//      HC-05  : AT / AT+NAME / AT+PSWD / AT+UART / AT+ROLE / AT+VERSION?
//      JDY-31 : AT+VERSION / AT+BAUD / AT+ENLOG / AT+PIN ...
//
// ⚠️ 电压注意：模块 VCC 接 5V，其 TXD 输出为 5V 电平，
//      而 STM32 的 PA3 不耐 5V。建议在模块 TXD 与 PA3 之间加分压：
//          模块 TXD ──[1kΩ]──┬── PA3
//                             │
//                           [2kΩ]
//                             │
//                            GND
//
// 两种工作模式：
//   1. 透传模式（日常使用）：手机发什么串口就收到什么，无需任何 AT 指令
//   2. AT 模式（仅配置用）：KEY=高电平，用 AT 指令改名称/波特率/配对码
//

#ifndef LINEFOLLOWING_CAR_BLE_H
#define LINEFOLLOWING_CAR_BLE_H

#include "main.h"

/* ------------------------------------------------------------------------- */
/* 硬件与协议常量                                                              */
/* ------------------------------------------------------------------------- */

/* HC-05 默认通信波特率。改模块波特率后这里也要同步，否则通信全乱 */
#define BLE_DEFAULT_BAUD        9600U

/* UART 发送超时，单位 ms */
#define BLE_UART_TIMEOUT_MS     100U

/* 接收环形缓冲区大小 */
#define BLE_RX_BUFFER_SIZE      128U

/* 单行缓冲大小 */
#define BLE_LINE_BUFFER_SIZE    128U

/* 等待 AT 响应的字节间隔超时，单位 ms */
#define BLE_AT_GAP_TIMEOUT_MS   50U

/* KEY 引脚：高电平 = AT 配置模式 */
#define BLE_KEY_PORT            BLE_EN_GPIO_Port
#define BLE_KEY_PIN             BLE_EN_Pin

/* STATE 引脚：已连接时为高电平 */
#define BLE_STATE_PORT          BLE_STATE_GPIO_Port
#define BLE_STATE_PIN           BLE_STATE_Pin
#define BLE_STATE_CONNECTED     GPIO_PIN_SET

/* 运行模式（内部用） */
#define BLE_MODE_TRANSPARENT    0U
#define BLE_MODE_AT             1U

/* ------------------------------------------------------------------------- */
/* 初始化与状态                                                                */
/* ------------------------------------------------------------------------- */

/*
 * 初始化 HC-05 驱动。
 * 请在 MX_USART2_UART_Init() 之后调用。
 *
 * 注意：本函数**不会**把模块切到 AT 模式——它把 KEY 拉低，
 * 让模块处于正常透传状态。需要配置模块时先调用 BLE_EnterATMode()。
 */
void BLE_Init(void);

/* 读取 STATE 引脚：1 = 手机/电脑已连接，0 = 未连接 */
uint8_t BLE_IsConnected(void);

/* 查询软件当前是否认为处于 AT 模式 */
uint8_t BLE_IsATMode(void);

/* ------------------------------------------------------------------------- */
/* AT 配置模式                                                                 */
/* ------------------------------------------------------------------------- */

/*
 * 把 KEY 拉高（AT 模式）。
 *
 * HC-05 的 AT 模式由**上电瞬间 KEY 的电平**决定，所以只在运行中拉高
 * 不一定能进入 AT 模式。可靠做法：
 *   1. 调用本函数（KEY 变高）
 *   2. 给模块断电再上电
 *   3. 之后再发 AT 指令
 *
 * 返回 1 = 已设置（不代表模块已进入 AT 模式）
 */
uint8_t BLE_EnterATMode(void);

/* 把 KEY 拉低（退出 AT 模式，回到透传）。同样建议断电重启才可靠。 */
void BLE_ExitATMode(void);

/*
 * 发送一条 AT 指令并等待响应（仅 AT 模式下有效）。
 *
 *   cmd      : 指令内容，例如 "AT" / "AT+NAME?" / "AT+PSWD=1234"
 *              **不需要自己加 \r\n**，函数会自动补
 *   response : 输出缓冲，可为 NULL
 *   resp_size: 缓冲大小
 *
 * 返回 1 = 收到响应（内容在 response 里）
 *      0 = 超时无响应（最常见原因：模块不在 AT 模式，或波特率不对）
 *
 * 典型响应：
 *   "AT"            -> "OK"
 *   "AT+NAME?"      -> "+NAME:HC-05\r\nOK"
 *   "AT+VERSION?"   -> "+VERSION:2.0-20100601\r\nOK"
 *   "AT+UART?"      -> "+UART:9600,0,0\r\nOK"
 */
uint8_t BLE_SendAT(const char *cmd, char *response, uint16_t resp_size);

/* ---- 常用配置封装 ---- */

/* 查询版本。返回 1 = 成功。 */
uint8_t BLE_QueryVersion(char *response, uint16_t resp_size);

/*
 * 查询通信波特率。返回实际波特率（如 9600 / 115200），失败返回 0。
 * 响应形如：+UART:9600,0,0
 */
uint32_t BLE_QueryBaud(void);

/*
 * 设置波特率（仅 AT 模式）。
 * 发 "AT+UART=<baud>,0,0"。返回 1 = 模块回 OK。
 *
 * ⚠️ 改完必须把 STM32 的 USART2 改成同样的波特率，否则通信中断。
 *    模块重启后才用新波特率。
 */
uint8_t BLE_SetBaud(uint32_t baud);

/* 设置广播名（最长 32 字节）。 */
uint8_t BLE_SetName(const char *name);

/* 上电是否自动配置模块（从机 + 名字 LFCar + 配对码 1234）。1=开启 */
#define BLE_AUTOCONFIG_ON_BOOT      1

/*
 * 上电自动配置 HC-05：按 9600 / 38400 两种 AT 波特率握手，
 * 成功则设置 从机、广播名 LFCar、配对码 1234。
 * 返回 1 = 配置成功；0 = 模块不在 AT 模式（不影响透传，忽略即可）。
 * 说明：HC-05 上电、未配对时就是 AT 模式（9600），所以上电即可配置，
 *       不需要手动按 KEY。
 */
uint8_t BLE_AutoConfig(void);

/* 自动识别到的模块名："JDY-31" / "HC-05" / "unknown"（用于启动打印） */
const char *BLE_ModuleName(void);

/* 上次 BLE_AutoConfig 的结果：1 = 成功识别并配置，0 = 失败 */
uint8_t BLE_GetCfgResult(void);

/* 设置配对码（4 位数字字符串，如 "1234"）。 */
uint8_t BLE_SetPin(const char *pin);

/* 查询角色：0=从机，1=主机，0xFF=失败。 */
uint8_t BLE_QueryRole(void);

/* 软复位模块。复位后约 500ms 内不要发指令。 */
uint8_t BLE_Reset(void);

/* 恢复出厂设置。执行后波特率回到 9600，需要同步 STM32 侧。 */
uint8_t BLE_RestoreDefault(void);

/* 断开当前连接（HC-05 无专用指令，只能复位或由对端断开）。 */
uint8_t BLE_Disconnect(void);

/* ------------------------------------------------------------------------- */
/* 透传收发                                                                    */
/* ------------------------------------------------------------------------- */

/* 发送原始数据。成功返回发送字节数，失败返回 0。 */
uint16_t BLE_Send(const uint8_t *data, uint16_t len);

/* 发送字符串（不含结尾 '\0'）。 */
uint16_t BLE_SendString(const char *text);

/* 发送一个字节。 */
void BLE_SendByte(uint8_t byte);

/* 取一个接收字节。返回 1 = 取到，0 = 空。 */
uint8_t BLE_GetByte(uint8_t *byte);

/*
 * 按行取一条透传消息（以 \n 或 \r 结尾）。
 * 返回 1 = 取到完整一行（已去掉行尾符）；0 = 还没有完整的一行。
 */
uint8_t BLE_GetLine(char *line, uint16_t max_len);

/* 清空接收缓冲和行缓冲。 */
void BLE_FlushRx(void);

/* ------------------------------------------------------------------------- */
/* 人机提示与自检                                                              */
/* ------------------------------------------------------------------------- */

/* 用 LED2 指示连接状态：未连接慢闪，已连接常亮。 */
void BLE_UpdateLed(void);

/* 非阻塞状态监视：每 200ms 打印 STATE / 连接状态 / 累计接收字节数。 */
void BLE_TestSequence(uint32_t duration_sec);

/*
 * AT 通路验证：阻塞约 3 秒，依次发送
 *     AT / AT+VERSION? / AT+UART? / AT+NAME? / AT+ROLE?
 * 并打印结果。
 *
 * ⚠️ 前提：模块必须处于 AT 模式（上电时 KEY 为高）。
 *     如果全部无响应，先确认 KEY 引脚电平和上电时机。
 */
void BLE_Verify(void);

#endif //LINEFOLLOWING_CAR_BLE_H
