//
// Created by JiangyaF on 2026/10/11.
// 重写于 2026/10/22：适配 HC-05（原版按 JDY-31 编写，两者协议完全不同）
//
// HC-05 驱动实现
//
// 设计要点：
//
//   1. 接收用【环形缓冲区 + 字节中断】。不用阻塞式接收，否则会把
//      FreeRTOS 的 BleTask 卡住。
//
//   2. AT 指令和透传数据共用同一个串口。AT 模式由 KEY 引脚的状态决定，
//      不由 UART 决定，所以驱动里只用一个软件标志 s_mode 记录意图。
//
//   3. HC-05 的 AT 响应以 "OK" 或 "+XXX:...\r\nOK" 结束，
//      用"字节间隔超时"判断响应结束（响应长度不固定）。
//
//   4. 所有 KEY 引脚操作集中在 BLE_EnterATMode / BLE_ExitATMode 两个函数里，
//      避免散落各处。
//

#include "Ble.h"
#include "Led.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 串口句柄定义在 main.c 中 */
extern UART_HandleTypeDef huart2;

/* ------------------------------------------------------------------------- */
/* 模块内部状态                                                                */
/* ------------------------------------------------------------------------- */

/* 接收环形缓冲区 */
static volatile uint8_t  s_rx_buf[BLE_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;         /* 写入位置（中断里改）  */
static volatile uint16_t s_rx_tail;         /* 读取位置（主循环里改）*/
static volatile uint32_t s_rx_total;        /* 累计收到字节数        */
static volatile uint8_t  s_rx_overflow;     /* 是否发生过溢出        */

/* 单字节中断接收的落点 */
static uint8_t  s_rx_byte;

/* 行缓冲 */
static char     s_line_buf[BLE_LINE_BUFFER_SIZE];
static uint16_t s_line_len;
static volatile uint8_t s_line_ready;
static char     s_last_char;

/* 当前模式 */
static uint8_t  s_mode = BLE_MODE_TRANSPARENT;

/* LED 状态跟踪 */
static uint8_t  s_last_connected = 0xFFU;   /* 0xFF = 尚未同步过 */

/* ------------------------------------------------------------------------- */
/* 环形缓冲区                                                                  */
/* ------------------------------------------------------------------------- */

static void Ble_RxPush(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % BLE_RX_BUFFER_SIZE);

    s_rx_total++;

    if (next == s_rx_tail)
    {
        /* 缓冲区满：丢弃新字节（保留可能正在等待处理的旧命令） */
        s_rx_overflow = 1U;
        return;
    }

    s_rx_buf[s_rx_head] = byte;
    s_rx_head = next;
}

static uint8_t Ble_RxPop(uint8_t *byte)
{
    if (s_rx_tail == s_rx_head)
    {
        return 0U;      /* 空 */
    }

    *byte = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % BLE_RX_BUFFER_SIZE);
    return 1U;
}

/* ------------------------------------------------------------------------- */
/* 接收中断回调                                                                */
/* ------------------------------------------------------------------------- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
    {
        return;
    }

    /* 1. 入环形缓冲区 */
    Ble_RxPush(s_rx_byte);

    /* 2. 行组装：以 \n 或 \r 结尾算一行 */
    if ((s_rx_byte == '\n') || (s_rx_byte == '\r'))
    {
        if ((s_last_char != '\n') && (s_last_char != '\r'))
        {
            if (s_line_len < BLE_LINE_BUFFER_SIZE)
            {
                s_line_buf[s_line_len] = '\0';
            }
            s_line_ready = 1U;
        }
    }
    else if (s_line_ready == 0U)
    {
        if (s_line_len < (BLE_LINE_BUFFER_SIZE - 1U))
        {
            s_line_buf[s_line_len] = (char)s_rx_byte;
            s_line_len++;
        }
    }

    s_last_char = (char)s_rx_byte;

    /* 3. 立刻重新挂上接收 */
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

/* ------------------------------------------------------------------------- */
/* 初始化                                                                      */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* 模块类型（自动识别）                                                        */
/* ------------------------------------------------------------------------- */
#define BLE_MODULE_UNKNOWN      0U
#define BLE_MODULE_HC05         1U
#define BLE_MODULE_JDY31        2U

static uint8_t s_module_type = BLE_MODULE_UNKNOWN;
static uint8_t s_cfg_ok = 0U;      /* 上次 BLE_AutoConfig 是否成功 */

uint8_t BLE_GetCfgResult(void)
{
    return s_cfg_ok;
}

uint32_t BLE_GetBaudrate(void)
{
    return huart2.Init.BaudRate;
}

const char *BLE_ModuleName(void)
{
    switch (s_module_type)
    {
        case BLE_MODULE_JDY31: return "JDY-31";
        case BLE_MODULE_HC05:  return "HC-05";
        default:               return "unknown";
    }
}

void BLE_Init(void)
{
    /* 清状态 */
    s_rx_head        = 0U;
    s_rx_tail        = 0U;
    s_rx_total       = 0U;
    s_rx_overflow    = 0U;
    s_line_len       = 0U;
    s_line_ready     = 0U;
    s_last_char      = '\0';
    s_mode           = BLE_MODE_TRANSPARENT;
    s_last_connected = 0xFFU;

    /*
     * KEY 拉低 = 正常透传模式。
     * 注意：HC-05 只在**上电瞬间**采样 KEY，运行中拉低不会退出 AT 模式，
     * 但如果模块本来就在透传模式，这个操作是安全的空操作。
     */
    HAL_GPIO_WritePin(BLE_KEY_PORT, BLE_KEY_PIN, GPIO_PIN_RESET);

    /* 启动字节中断接收 */
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

/* ------------------------------------------------------------------------- */
/* 连接状态                                                                    */
/* ------------------------------------------------------------------------- */

uint8_t BLE_IsConnected(void)
{
    return (HAL_GPIO_ReadPin(BLE_STATE_PORT, BLE_STATE_PIN) == BLE_STATE_CONNECTED)
           ? 1U : 0U;
}

uint8_t BLE_IsATMode(void)
{
    return (s_mode == BLE_MODE_AT) ? 1U : 0U;
}

/* ------------------------------------------------------------------------- */
/* AT 模式控制                                                                 */
/* ------------------------------------------------------------------------- */

uint8_t BLE_EnterATMode(void)
{
    HAL_GPIO_WritePin(BLE_KEY_PORT, BLE_KEY_PIN, GPIO_PIN_SET);
    s_mode = BLE_MODE_AT;

    /*
     * 给模块 200ms 稳定时间。注意这不能保证真正进入 AT 模式——
     * HC-05 的 AT 模式需要在上电瞬间 KEY 为高。运行中切换只在
     * 部分固件上有效。可靠做法是断电重启。
     */
    HAL_Delay(200U);

    return 1U;
}

void BLE_ExitATMode(void)
{
    HAL_GPIO_WritePin(BLE_KEY_PORT, BLE_KEY_PIN, GPIO_PIN_RESET);
    s_mode = BLE_MODE_TRANSPARENT;
    BLE_FlushRx();
}

/*
 * 切换 USART2 的波特率（只用于 AT 握手时试 9600/38400）。
 * 重新 Init 之后必须重新挂上中断接收，否则收不到模块响应。
 */
static uint8_t BLE_SetBaudInternal(uint32_t baud)
{
    huart2.Init.BaudRate = baud;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        return 0U;
    }

    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
    return 1U;
}

/* ------------------------------------------------------------------------- */
/* 发送                                                                        */
/* ------------------------------------------------------------------------- */

uint16_t BLE_Send(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (len == 0U))
    {
        return 0U;
    }

    /* 给明确的超时，不用 HAL_MAX_DELAY：模块被拔掉时不会永久卡死 */
    status = HAL_UART_Transmit(&huart2, (uint8_t *)data, len, BLE_UART_TIMEOUT_MS);

    return (status == HAL_OK) ? len : 0U;
}

uint16_t BLE_SendString(const char *text)
{
    if (text == NULL)
    {
        return 0U;
    }
    return BLE_Send((const uint8_t *)text, (uint16_t)strlen(text));
}

void BLE_SendByte(uint8_t byte)
{
    (void)BLE_Send(&byte, 1U);
}

/* ------------------------------------------------------------------------- */
/* 接收                                                                        */
/* ------------------------------------------------------------------------- */

uint8_t BLE_GetByte(uint8_t *byte)
{
    if (byte == NULL)
    {
        return 0U;
    }
    return Ble_RxPop(byte);
}

uint8_t BLE_GetLine(char *line, uint16_t max_len)
{
    uint16_t copy_len;

    if ((line == NULL) || (max_len == 0U))
    {
        return 0U;
    }

    if (s_line_ready == 0U)
    {
        return 0U;
    }

    copy_len = s_line_len;
    if (copy_len >= max_len)
    {
        copy_len = (uint16_t)(max_len - 1U);
    }

    memcpy(line, s_line_buf, copy_len);
    line[copy_len] = '\0';

    s_line_len   = 0U;
    s_line_ready = 0U;

    return 1U;
}

void BLE_FlushRx(void)
{
    s_rx_tail    = s_rx_head;
    s_line_len   = 0U;
    s_line_ready = 0U;
    s_last_char  = '\0';
}

/* ------------------------------------------------------------------------- */
/* AT 指令                                                                     */
/* ------------------------------------------------------------------------- */

uint8_t BLE_SendAT(const char *cmd, char *response, uint16_t resp_size)
{
    char     cmd_buf[64];
    uint16_t cmd_len;
    uint16_t got = 0U;
    uint32_t last_rx_tick;

    if (cmd == NULL)
    {
        return 0U;
    }

    /* HC-05 的 AT 指令以 \r\n 结尾 */
    cmd_len = (uint16_t)snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);
    if ((cmd_len == 0U) || (cmd_len >= sizeof(cmd_buf)))
    {
        return 0U;      /* 指令太长，已被截断 */
    }

    /* 发之前清空接收，避免把上一条的残留响应当成本次结果 */
    BLE_FlushRx();

    if (BLE_Send((const uint8_t *)cmd_buf, cmd_len) == 0U)
    {
        return 0U;
    }

    /*
     * 收响应：只要还有字节持续到达就继续收，直到连续
     * BLE_AT_GAP_TIMEOUT_MS 没有新字节。这样既能收 "OK"（2 字节），
     * 也能收 "+VERSION:2.0-20100601\r\nOK" 这样的多行响应。
     */
    last_rx_tick = HAL_GetTick();

    for (;;)
    {
        uint8_t byte;

        if (Ble_RxPop(&byte) != 0U)
        {
            last_rx_tick = HAL_GetTick();

            if ((response != NULL) && (got < (uint16_t)(resp_size - 1U)))
            {
                response[got] = (char)byte;
                got++;
            }
            continue;
        }

        if ((HAL_GetTick() - last_rx_tick) >= BLE_AT_GAP_TIMEOUT_MS)
        {
            break;
        }
    }

    if (response != NULL)
    {
        response[got] = '\0';
    }

    return (got > 0U) ? 1U : 0U;
}

uint8_t BLE_QueryVersion(char *response, uint16_t resp_size)
{
    return BLE_SendAT("AT+VERSION?", response, resp_size);
}

uint32_t BLE_QueryBaud(void)
{
    char    resp[BLE_LINE_BUFFER_SIZE];
    char   *pos;
    long    value;

    if (BLE_SendAT("AT+UART?", resp, sizeof(resp)) == 0U)
    {
        return 0U;
    }

    /*
     * 期望形如：+UART:9600,0,0
     * 用 strstr 找 "UART:" 然后解析数字。
     */
    pos = strstr(resp, "UART:");
    if (pos == NULL)
    {
        return 0U;
    }

    pos += 5;       /* 跳过 "UART:" */

    value = strtol(pos, NULL, 10);
    if ((value <= 0) || (value > 4000000L))
    {
        return 0U;
    }

    return (uint32_t)value;
}

uint8_t BLE_SetBaud(uint32_t baud)
{
    char cmd[48];
    char resp[BLE_LINE_BUFFER_SIZE];

    /* 合理性检查：HC-05 支持的波特率范围大致是 1200~1382400 */
    if ((baud < 1200U) || (baud > 1382400U))
    {
        return 0U;
    }

    if (s_module_type == BLE_MODULE_JDY31)
    {
        /* JDY-31：AT+BAUD<4..9>（4=9600 5=19200 6=38400 7=57600 8=115200 9=128000） */
        const char *param = NULL;

        switch (baud)
        {
            case 9600U:   param = "4"; break;
            case 19200U:  param = "5"; break;
            case 38400U:  param = "6"; break;
            case 57600U:  param = "7"; break;
            case 115200U: param = "8"; break;
            case 128000U: param = "9"; break;
            default: return 0U;
        }

        (void)snprintf(cmd, sizeof(cmd), "AT+BAUD%s", param);
    }
    else
    {
        /* HC-05 的命令是 AT+UART=<baud>,<stop>,<parity> */
        (void)snprintf(cmd, sizeof(cmd), "AT+UART=%lu,0,0", (unsigned long)baud);
    }

    return BLE_SendAT(cmd, resp, sizeof(resp));
}

/*
 * 上电自动配置 HC-05。
 *
 * 依据 HC-05 规格书：
 *   - 模块上电、未配对时处于 AT 命令模式，波特率=模块自身波特率（出厂 9600）；
 *   - 若上电瞬间 KEY 为高，则固定 38400。
 * 所以这里先试 9600、再试 38400，握手成功后配置：
 *   AT+ROLE=0        从机
 *   AT+NAME=LFCar    广播名
 *   AT+PSWD=1234     配对码
 * 波特率保持 9600 不动（规格书：改波特率后要重新上电才生效，没必要冒险）。
 *
 * 返回 1 = 握手并配置成功；0 = 没找到模块/不在 AT 模式（不影响透传使用）。
 */
uint8_t BLE_AutoConfig(void)
{
#if (BLE_MODULE_FORCE_JDY31 != 0)
    /*
     * JDY-31 专用流程（固定 9600）：
     *   1) 在 9600/38400/115200 里找模块（发 AT+VERSION，能回就说明在 AT 模式）；
     *   2) 发 AT+BAUD4 把它设成 9600，等复位/生效后 MCU 也切 9600；
     *   3) 发 AT+NAMELFCar / AT+PIN1234 / AT+ENLOG0（JDY 格式，无 '='）。
     */
    static const uint32_t bauds[3] = { 9600U, 38400U, 115200U };
    char     resp[BLE_LINE_BUFFER_SIZE];
    uint8_t  found = 0U;
    uint32_t found_baud = 9600U;
    uint32_t i;

    s_module_type = BLE_MODULE_JDY31;

    for (i = 0U; i < 3U; i++)
    {
        if (BLE_SetBaudInternal(bauds[i]) == 0U)
        {
            continue;
        }

        BLE_FlushRx();
        HAL_Delay(20U);

        if ((BLE_SendAT("AT+VERSION", resp, sizeof(resp)) != 0U) &&
            (strstr(resp, "VERSION") != NULL))
        {
            found = 1U;
            found_baud = bauds[i];
            break;
        }
    }

    if (found == 0U)
    {
        s_cfg_ok = 0U;
        (void)BLE_SetBaudInternal(9600U);
        return 0U;
    }

    if (found_baud != 9600U)
    {
        char chk[BLE_LINE_BUFFER_SIZE];

        (void)BLE_SendAT("AT+BAUD4", chk, sizeof(chk));   /* 4 = 9600 */
        (void)BLE_SendAT("AT+RESET", chk, sizeof(chk));
        HAL_Delay(400U);
        (void)BLE_SetBaudInternal(9600U);
        BLE_FlushRx();
        HAL_Delay(30U);

        if ((BLE_SendAT("AT+VERSION", chk, sizeof(chk)) == 0U) ||
            (strstr(chk, "VERSION") == NULL))
        {
            /* 没切成：退回找到的波特率，至少链路可用 */
            (void)BLE_SetBaudInternal(found_baud);
        }
    }

    (void)BLE_SendAT("AT+BAUD4",    resp, sizeof(resp));  /* 幂等，确保 9600 */
    (void)BLE_SendAT("AT+NAMELFCar", resp, sizeof(resp));
    (void)BLE_SendAT("AT+PIN1234",   resp, sizeof(resp));
    (void)BLE_SendAT("AT+ENLOG0",    resp, sizeof(resp));

    BLE_FlushRx();
    HAL_Delay(20U);
    s_cfg_ok = 1U;
    return 1U;
#else
    static const uint32_t bauds[3] = { 9600U, 38400U, 115200U };
    char     resp[BLE_LINE_BUFFER_SIZE];
    uint8_t  detected = 0U;
    uint32_t used_baud = 9600U;
    uint32_t i;

    /*
     * 自动识别模块：
     *   JDY-31  : AT+VERSION 返回 "+VERSION=JDY-31-..."
     *   HC-05   : AT 返回 "OK"
     * JDY-31 未连接时就是 AT 模式，不需要 KEY 引脚；
     * HC-05 出厂 9600、上电未配对时也是 AT 模式。
     */
    s_module_type = BLE_MODULE_UNKNOWN;

    for (i = 0U; i < 3U; i++)
    {
        if (BLE_SetBaudInternal(bauds[i]) == 0U)
        {
            continue;
        }

        BLE_FlushRx();
        HAL_Delay(20U);

        if ((BLE_SendAT("AT+VERSION", resp, sizeof(resp)) != 0U) &&
            (strstr(resp, "JDY") != NULL))
        {
            s_module_type = BLE_MODULE_JDY31;
            detected = 1U;
            used_baud = bauds[i];
            break;
        }

        if ((BLE_SendAT("AT", resp, sizeof(resp)) != 0U) &&
            (strstr(resp, "OK") != NULL))
        {
            s_module_type = BLE_MODULE_HC05;
            detected = 1U;
            used_baud = bauds[i];
            break;
        }
    }

    if (detected == 0U)
    {
        s_cfg_ok = 0U;
        (void)BLE_SetBaudInternal(9600U);
        return 0U;
    }

    if (s_module_type == BLE_MODULE_HC05)
    {
        /* HC-05 的参数要先进 AT 模式才能改 */
        BLE_EnterATMode();
    }

    /*
     * 统一波特率到 9600。
     *
     * 背景：模块被电机干扰复位后会回到出厂 9600，而固件还停在识别到的
     * 38400/115200，双方失配 -> "蓝牙连得上但一个字节都没有"。
     * 这里识别成功后主动把模块改回 9600，之后双方永远一致。
     *
     * 改完立刻用 AT 验证；改不动（比如命令不被支持）就回退到原速率，
     * 保证不会把链路弄死。
     */
    if (used_baud != 9600U)
    {
        char chk[BLE_LINE_BUFFER_SIZE];

        if (s_module_type == BLE_MODULE_JDY31)
        {
            (void)BLE_SendAT("AT+BAUD4", chk, sizeof(chk));   /* 4 = 9600 */
            (void)BLE_SendAT("AT+RESET", chk, sizeof(chk));
        }
        else
        {
            (void)BLE_SendAT("AT+UART=9600,0,0", chk, sizeof(chk));
            (void)BLE_SendAT("AT+RESET", chk, sizeof(chk));
        }

        HAL_Delay(400U);
        (void)BLE_SetBaudInternal(9600U);
        BLE_FlushRx();
        HAL_Delay(30U);

        if ((BLE_SendAT("AT", chk, sizeof(chk)) != 0U) &&
            (strstr(chk, "OK") != NULL))
        {
            used_baud = 9600U;      /* 改成功 */
        }
        else
        {
            /* 改不动：回到识别到的速率，链路保持可用 */
            (void)BLE_SetBaudInternal(used_baud);
        }
    }

    if (s_module_type == BLE_MODULE_JDY31)
    {
        /*
         * JDY-31 固定 9600：AT+BAUD4 是幂等的（本来就是 9600 也无害）。
         * 这样即使模块被改过速率/复位，也保证双方一致。
         */
        (void)BLE_SendAT("AT+BAUD4", resp, sizeof(resp));

        /* JDY-31：AT+NAME<name> / AT+PIN<4位> / AT+ENLOG<0|1> */
        (void)BLE_SendAT("AT+NAMELFCar", resp, sizeof(resp));
        (void)BLE_SendAT("AT+PIN1234",  resp, sizeof(resp));
        (void)BLE_SendAT("AT+ENLOG0",   resp, sizeof(resp));  /* 别让状态行混进日志 */
    }
    else
    {
        /* HC-05：AT 模式已在上面进入，参数用 = 形式 */
        (void)BLE_SendAT("AT+ROLE=0",    resp, sizeof(resp));
        (void)BLE_SendAT("AT+NAME=LFCar", resp, sizeof(resp));
        (void)BLE_SendAT("AT+PSWD=1234",  resp, sizeof(resp));
    }

    /*
     * 注意：这里【不再强制回到 9600】。
     * 如果模块本来跑在 38400/115200（比如之前被改过），
     * 识别成功后把 MCU 拉回 9600 就会出现"连得上、没数据"。
     * 保持识别到的波特率，透传才和模块一致。
     */
    if (s_module_type == BLE_MODULE_HC05)
    {
        BLE_ExitATMode();
    }

    BLE_FlushRx();
    HAL_Delay(20U);
    s_cfg_ok = 1U;
    return 1U;
#endif /* BLE_MODULE_FORCE_JDY31 */
}

uint8_t BLE_SetName(const char *name)
{
    char cmd[64];
    char resp[BLE_LINE_BUFFER_SIZE];

    if (name == NULL)
    {
        return 0U;
    }

    /* HC-05 名字最长 32 字节 */
    if (strlen(name) > 32U)
    {
        return 0U;
    }

    if (s_module_type == BLE_MODULE_JDY31)
    {
        /* JDY-31：AT+NAME<name>，最长 18 字节 */
        (void)snprintf(cmd, sizeof(cmd), "AT+NAME%s", name);
    }
    else
    {
        (void)snprintf(cmd, sizeof(cmd), "AT+NAME=%s", name);
    }

    return BLE_SendAT(cmd, resp, sizeof(resp));
}

uint8_t BLE_SetPin(const char *pin)
{
    char cmd[32];
    char resp[BLE_LINE_BUFFER_SIZE];

    if ((pin == NULL) || (strlen(pin) != 4U))
    {
        return 0U;      /* HC-05 要求 4 位 */
    }

    if (s_module_type == BLE_MODULE_JDY31)
    {
        (void)snprintf(cmd, sizeof(cmd), "AT+PIN%s", pin);
    }
    else
    {
        (void)snprintf(cmd, sizeof(cmd), "AT+PSWD=%s", pin);
    }

    return BLE_SendAT(cmd, resp, sizeof(resp));
}

uint8_t BLE_QueryRole(void)
{
    char  resp[BLE_LINE_BUFFER_SIZE];
    char *pos;

    if (BLE_SendAT("AT+ROLE?", resp, sizeof(resp)) == 0U)
    {
        return 0xFFU;
    }

    /* 期望形如：+ROLE:0 */
    pos = strstr(resp, "ROLE:");
    if (pos == NULL)
    {
        return 0xFFU;
    }

    pos += 5;

    if ((*pos < '0') || (*pos > '1'))
    {
        return 0xFFU;
    }

    return (uint8_t)(*pos - '0');
}

uint8_t BLE_Reset(void)
{
    char    resp[BLE_LINE_BUFFER_SIZE];
    uint8_t ok = BLE_SendAT("AT+RESET", resp, sizeof(resp));

    /* 复位需要时间，期间不要发指令 */
    HAL_Delay(500U);

    /* 复位会打断接收中断链，重新挂上 */
    BLE_FlushRx();
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);

    return ok;
}

uint8_t BLE_RestoreDefault(void)
{
    char    resp[BLE_LINE_BUFFER_SIZE];
    uint8_t ok = BLE_SendAT("AT+ORGL", resp, sizeof(resp));

    /*
     * HC-05 的恢复出厂指令是 AT+ORGL。
     * 执行后波特率回到 9600，模块需要重新上电才完全生效。
     */
    HAL_Delay(500U);
    BLE_FlushRx();
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);

    return ok;
}

uint8_t BLE_Disconnect(void)
{
    /*
     * HC-05 没有"主动断开"的 AT 指令。
     * 这里用软复位实现断开：复位会让模块断开当前连接并重新广播。
     */
    return BLE_Reset();
}

/* ------------------------------------------------------------------------- */
/* LED 指示                                                                    */
/* ------------------------------------------------------------------------- */

void BLE_UpdateLed(void)
{
    uint8_t connected = BLE_IsConnected();

    /* 状态没变就不动 LED，否则会不断重置闪烁计时，表现为常亮 */
    if (connected == s_last_connected)
    {
        return;
    }

    s_last_connected = connected;

    if (connected != 0U)
    {
        Led_On(LED_2);                          /* 已连接：常亮 */
    }
    else
    {
        Led_BlinkForever(LED_2, 100U, 1000U);   /* 未连接：1 秒闪一次 */
    }
}

/* ------------------------------------------------------------------------- */
/* 自检                                                                        */
/* ------------------------------------------------------------------------- */

void BLE_TestSequence(uint32_t duration_sec)
{
    char     buf[160];
    uint32_t elapsed_ms = 0U;
    uint32_t total_ms   = duration_sec * 1000U;
    uint32_t next_print = 0U;

    {
        int len = snprintf(buf, sizeof(buf),
                           "\r\n[Ble] HC-05 status monitor for %lu s\r\n"
                           "  STATE=PA1  KEY=PB3  UART=USART2(PA2/PA3) @ %lu\r\n"
                           "  time   STATE  connected  rx_total  overflow\r\n",
                           (unsigned long)duration_sec,
                           (unsigned long)BLE_DEFAULT_BAUD);
        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
        }
    }

    while (elapsed_ms < total_ms)
    {
        HAL_Delay(10U);
        elapsed_ms += 10U;

        if (elapsed_ms >= next_print)
        {
            int len;

            next_print = elapsed_ms + 200U;

            len = snprintf(buf, sizeof(buf), "  %5lu   %u     %u        %6lu    %u\r\n",
                           (unsigned long)elapsed_ms,
                           (unsigned int)HAL_GPIO_ReadPin(BLE_STATE_PORT, BLE_STATE_PIN),
                           (unsigned int)BLE_IsConnected(),
                           (unsigned long)s_rx_total,
                           (unsigned int)s_rx_overflow);
            if (len > 0)
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
            }
        }
    }

    {
        const char *done = "[Ble] monitor done\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)done, (uint16_t)strlen(done), 100U);
    }
}

void BLE_Verify(void)
{
    static const char *test_cmds[] =
    {
        "AT",
        "AT+VERSION?",
        "AT+UART?",
        "AT+NAME?",
        "AT+ROLE?",
    };

    char     resp[BLE_LINE_BUFFER_SIZE];
    uint32_t i;

    {
        char msg[224];

        int len = snprintf(msg, sizeof(msg),
                           "\r\n[Ble] HC-05 AT verification\r\n"
                           "  prerequisite: module must be in AT mode (KEY high at power-on)\r\n"
                           "  expected baud: %lu\r\n"
                           "  if all commands time out: module is NOT in AT mode\r\n",
                           (unsigned long)BLE_DEFAULT_BAUD);
        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)len, 100U);
        }
    }

    for (i = 0U; i < (sizeof(test_cmds) / sizeof(test_cmds[0])); i++)
    {
        char line[256];
        int  len;

        len = snprintf(line, sizeof(line), "  --> %s\r\n", test_cmds[i]);
        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, 100U);
        }

        if (BLE_SendAT(test_cmds[i], resp, sizeof(resp)) != 0U)
        {
            len = snprintf(line, sizeof(line), "  <-- %s\r\n", resp);
        }
        else
        {
            len = snprintf(line, sizeof(line), "  <-- (no response)\r\n");
        }

        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, 100U);
        }

        HAL_Delay(200U);
    }

    {
        const char *done = "[Ble] verification done\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)done, (uint16_t)strlen(done), 100U);
    }
}

//
// Created by JiangyaF on 2026/10/11.
//
