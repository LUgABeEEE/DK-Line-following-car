//
// Created by JiangyaF on 2026/10/11.
//
// JDY-31 蓝牙 SPP 串口透传模块驱动
//
// 硬件（依据板子网表 循迹小车板V1.1）：
//   模块 TXD  -> P1.2 -> PA2 (USART2_TX)      ← STM32 发送
//   模块 RXD  -> P1.3 -> PA3 (USART2_RX)      ← STM32 接收
//   模块 STAT -> P1.1 -> PA1 (BLE_STATE)      ← 连接状态：未连接低，已连接高
//   模块 EN   -> P1.6 -> PB3 (BLE_EN)         ← 功能未在手册中说明，见下
//   模块 VCC                                                  接 +3.3V
//
// 模块参数（手册）：
//   蓝牙版本   : Bluetooth 3.0 SPP（经典蓝牙，从机）
//   工作电压   : 1.8~3.6V，本板接 3.3V，因此 UART 电平与 STM32 直连安全
//   默认波特率 : 9600（AT+BAUD 可改为 4~9 共六档，见 BLE_BAUD_xxx）
//   默认广播名 : JDY-31-SPP
//   默认配对码 : 1234
//   支持平台   : Windows / Linux / Android（**不支持 iOS**，因为 iOS 只支持 BLE 4.0 GATT）
//
// AT 指令：**每条指令必须以 \r\n 结尾**，否则模块不响应。
//
// IDLE 状态是否有 \r\n 也能收：模块没有"命令模式"和"透传模式"的切换，
// AT 指令和透传数据共用一个串口，所以本驱动用环形缓冲区分"按行取"和"按字节取"。
//
// ⚠️ 重要：USART2 也是本工程所有 xxx_TestSequence() 打印调试信息的串口。
//    所以蓝牙模块和 USB-TTL 调试线不能同时占用 USART2——只能二选一，
//    或者把 USB-TTL 的 RX 直接并在 PA3 上"旁听"（TX 由 STM32 独占，
//    并联 RX 不会冲突，两边都能收到同样的调试输出）。
//

#ifndef LINEFLLOWING_CAR_BLE_H
#define LINEFOLLOWING_CAR_BLE_H

#include "main.h"

/* ------------------------------------------------------------------------- */
/* 配置                                                                        */
/* ------------------------------------------------------------------------- */

/*
 * BLE_EN (PB3) 的有效电平。
 *
 * 手册没有说明 EN 引脚的功能，这里按 JDY 系列惯例假定"高电平使能"。
 * 如果你实测发现模块行为相反，把下面改成 0。
 */
#define BLE_EN_ACTIVE_HIGH      1

/* 模块 STAT 引脚：已连接时输出高电平 */
#define BLE_STATE_CONNECTED     GPIO_PIN_SET

/* 接收环形缓冲区大小。透传场景建议不小于 64，避免高吞吐时丢数据 */
#define BLE_RX_BUFFER_SIZE      128U

/* 单行 AT 响应缓冲区大小。模块回显加响应最长约 40 字节，128 足够 */
#define BLE_LINE_BUFFER_SIZE    128U

/* 等待 AT 响应时，两个字节之间的最大间隔。超过就认为响应结束 */
#define BLE_AT_GAP_TIMEOUT_MS   50U

/* ------------------------------------------------------------------------- */
/* 波特率档位（对应 AT+BAUD<Param> 的 Param）                                   */
/* ------------------------------------------------------------------------- */

#define BLE_BAUD_9600           4U
#define BLE_BAUD_19200          5U
#define BLE_BAUD_38400          6U
#define BLE_BAUD_57600          7U
#define BLE_BAUD_115200         8U
#define BLE_BAUD_128000         9U

/* 模块出厂默认波特率 */
#define BLE_BAUD_DEFAULT        9600U

/* ------------------------------------------------------------------------- */
/* 运行模式                                                                    */
/* ------------------------------------------------------------------------- */

#define BLE_MODE_TRANSPARENT    0U      /* 透传模式：原始字节收发 */
#define BLE_MODE_AT             1U      /* AT 指令模式：按行收发   */

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/*
 * 初始化：拉高 BLE_EN 使能模块，启动 USART2 的字节中断接收。
 * 请在 MX_USART2_UART_Init() 之后调用一次。
 *
 * 注意：本函数**不会**修改模块的波特率，也不会发送任何 AT 指令。
 * 上电时模块默认 9600，而本工程 USART2 配的是 115200，两者不匹配，
 * 所以首次使用必须先把模块波特率改成 115200（见 BLE_ChangeBaud）。
 */
void BLE_Init(void);

/* ---- 连接状态 ---- */

/* 读取 STAT 引脚：1 = 已连接（手机/电脑已连上），0 = 未连接 */
uint8_t BLE_IsConnected(void);

/* ---- 透传收发 ---- */

/*
 * 发送原始数据（透传）。成功返回发送的字节数，失败返回 0。
 * 阻塞式发送，115200 下 32 字节约 2.8ms，可以放在主循环里用。
 */
uint16_t BLE_Send(const uint8_t *data, uint16_t len);

/* 发送字符串（自动计算长度，末尾的 '\0' 不发送）。 */
uint16_t BLE_SendString(const char *text);

/*
 * 发送一个字节（通常用于调试打印）。
 * 与 printf 二选一时优先用它——printf 会拖进大量库代码。
 */
void BLE_SendByte(uint8_t byte);

/* ---- 接收 ---- */

/*
 * 取一个接收到的字节。
 * 返回 1 = 取到（写入 *byte）；0 = 缓冲区空。
 */
uint8_t BLE_GetByte(uint8_t *byte);

/*
 * 按行取一条透传消息（以 \n 或 \r 结尾）。
 *   line     : 输出缓冲区
 *   max_len  : 缓冲区大小（含结尾 '\0'）
 * 返回 1 = 取到完整一行（已去掉行尾符）；0 = 还没有完整的一行。
 *
 * 典型用途：手机 App 发 "FWD\n" 这样的一条命令。
 */
uint8_t BLE_GetLine(char *line, uint16_t max_len);

/* 清空接收缓冲和行缓冲。 */
void BLE_FlushRx(void);

/* ---- AT 指令 ---- */

/*
 * 发送一条 AT 指令并等待响应。
 *   cmd      : 指令内容，例如 "AT+VERSION"。函数会自动补 "\r\n"
 *   response : 输出缓冲区，可为 NULL（表示不关心响应内容）
 *   resp_size: response 缓冲区大小
 * 返回 1 = 收到了响应；0 = 超时无响应。
 *
 * 典型返回内容（模块真实回显，不同固件略有差异）：
 *   "AT+VERSION"  -> "+VERSION=JDY-31-V1.2,Bluetooth V3.0"
 *   "AT+BAUD"     -> "+BAUD=4"
 *   "AT+RESET"    -> "+OK"
 */
uint8_t BLE_SendAT(const char *cmd, char *response, uint16_t resp_size);

/*
 * 查询模块版本号，写入 response。
 * 这是**验证 AT 通路是否正常最直接的办法**：能拿到 +VERSION=... 就说明
 * 波特率、接线、AT 支持全部正常。
 */
uint8_t BLE_QueryVersion(char *response, uint16_t resp_size);

/* 查询当前波特率档位（4~9）。失败返回 0。 */
uint8_t BLE_QueryBaud(void);

/*
 * 修改模块波特率，返回 1 = 成功。
 *
 * ⚠️ 流程很重要，容易搞错顺序：
 *   1. 先用【旧波特率】发 AT+BAUD<新档位>
 *   2. 模块回 +OK 后立刻切换
 *   3. 所以本函数发完指令后，会调用方需要自己把 USART2 也切到新波特率
 *
 * 本函数只负责发指令和验证响应，不修改 STM32 侧配置。
 * 改完请自行重配 huart2.Init.BaudRate 并重新 HAL_UART_Init()，
 * 或者干脆在 CubeMX 里把 USART2 设成 115200，然后只调用一次本函数把模块改过来。
 *
 * 因为波特率档位只有 6 档，这里用 BLE_BAUD_xxx 常量，不要传裸数字。
 */
uint8_t BLE_SetBaud(uint8_t baud_code);

/*
 * 设置蓝牙广播名（手机搜索时看到的名字），最长 18 字节。
 */
uint8_t BLE_SetName(const char *name);

/*
 * 设置配对密码（4 位数字字符串）。出厂默认 "1234"。
 */
uint8_t BLE_SetPin(const char *pin);

/*
 * 软复位模块。复位后需要等待约 500ms 才能重新通信。
 */
uint8_t BLE_Reset(void);

/*
 * 断开当前蓝牙连接。仅在已连接状态下有效。
 */
uint8_t BLE_Disconnect(void);

/*
 * 恢复出厂设置（波特率会回到 9600，广播名回到 JDY-31-SPP）。
 * ⚠️ 调用后必须把 USART2 也切回 9600，否则再也通不上。
 */
uint8_t BLE_RestoreDefault(void);

/*
 * 开关模块的串口状态输出（连接/断开时模块主动往串口发提示）。
 *   1 = 打开（出厂默认），0 = 关闭
 * 调试时打开有用，正式跑车时建议关掉，避免状态字符串混进透传数据。
 */
uint8_t BLE_SetLogEnable(uint8_t enable);

/* ---- 人机提示 ---- */

/*
 * 用 LED 指示蓝牙状态：未连接慢闪，已连接常亮。
 * 请在主循环里周期性调用（和 Led_Refresh 一起，20ms 一次）。
 * 它只在连接状态**发生变化**时重新设置 LED，不会每拍都覆盖你的自定义闪烁。
 */
void BLE_UpdateLed(void);

/*
 * 状态自检：不阻塞，每 200ms 打印一次 STAT 引脚、连接状态、USB 侧收到的字节数。
 * 建议配合 BLE_Verify 一起用。
 */
void BLE_TestSequence(uint32_t duration_sec);

/*
 * AT 通路验证：阻塞约 3 秒，依次执行
 *   AT+VERSION / AT+BAUD / AT+NAME / AT+LADDR
 * 并把结果打印到 USART2（也就是同一根线，所以你会看到自己发的指令回显）。
 *
 * 用途：确认模块波特率是否和 USART2 匹配。
 *   - 打印出 "+VERSION=..."     -> ✅ 通路正常
 *   - 完全没响应                -> ❌ 波特率不匹配，或 AT+ENLOG 被关了
 *   - 打印乱码                  -> ❌ 波特率不匹配（最典型的现象）
 */
void BLE_Verify(void);

#endif //LINEFLLOWING_CAR_BLE_H
