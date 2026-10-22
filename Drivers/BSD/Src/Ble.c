//
// Created by JiangyaF on 2026/10/11.
//
// JDY-31 蓝牙 SPP 串口透传驱动实现
//
// 设计要点：
//
//   1. 接收用【环形缓冲区 + 字节中断】。
//      为什么不用 HAL_UART_Receive 阻塞式？一次收 N 个字节就把主循环卡住了，
//      20ms 的速度环会被拖垮。中断方式每收一个字节进一次回调，主循环完全不受影响。
//
//   2. AT 指令和透传数据共用同一个串口（模块没有模式切换）。
//      所以提供两套读取接口：BLE_GetByte 按字节取（透传），
//      BLE_GetLine 按行取（手机发 "FWD\n" 这类命令）。
//
//   3. 环形缓冲区的读写指针都用 uint16_t 无符号回绕。
//      和编码器计数器同样的技巧：用模运算代替 if 判断。
//
//   4. AT 响应用"字节间隔超时"判断结束，而不是等固定长度。
//      因为响应长度不固定（+VERSION 很长，+OK 只有 3 字节）。
//

#include "Ble.h"
#include "Led.h"
#include <stdio.h>
#include <string.h>

/* 串口句柄定义在 main.c 中 */
extern UART_HandleTypeDef huart2;

/* ------------------------------------------------------------------------- */
/* 硬件映射                                                                    */
/* ------------------------------------------------------------------------- */

/* STAT 引脚（PA1）：未连接低，已连接高 */
#define BLE_STATE_PORT      BLE_STATE_GPIO_Port
#define BLE_STATE_PIN       BLE_STATE_Pin

/* EN 引脚（PB3）：手册未说明，按高电平使能处理 */
#define BLE_EN_PORT         BLE_EN_GPIO_Port
#define BLE_EN_PIN          BLE_EN_Pin

/* ------------------------------------------------------------------------- */
/* 模块内部状态                                                                */
/* ------------------------------------------------------------------------- */

/* 接收环形缓冲区 */
static volatile uint8_t  s_rx_buf[BLE_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;         /* 写入位置（中断里改）  */
static volatile uint16_t s_rx_tail;         /* 读取位置（主循环里改）*/
static volatile uint32_t s_rx_total;        /* 累计收到字节数，用于诊断 */
static volatile uint8_t  s_rx_overflow;     /* 是否发生过溢出（只置位不清零）*/

/* 单字节中断接收的落点 */
static uint8_t  s_rx_byte;

/* 行缓冲：GetLine 用 */
static char     s_line_buf[BLE_LINE_BUFFER_SIZE];
static uint16_t s_line_len;
static volatile uint8_t s_line_ready;
static char     s_last_char;

/* 运行模式 */
static uint8_t  s_mode = BLE_MODE_TRANSPARENT;

/* LED 状态跟踪，避免每拍都重设闪烁（会打断闪烁计时） */
static uint8_t  s_last_connected = 0xFFU;   /* 0xFF = 尚未同步过 */

/* ------------------------------------------------------------------------- */
/* 环形缓冲区                                                                */
/* ------------------------------------------------------------------------- */

static void Ble_RxPush(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % BLE_RX_BUFFER_SIZE);

    s_rx_total++;

    if (next == s_rx_tail)
    {
        /* 缓冲区满：丢弃新字节，并标记溢出。
           丢新而不是丢旧——旧数据可能是一条正在等待处理的完整命令。 */
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

/*
 * 每收到一个字节进一次。这里只做两件事：入环形缓冲、推进行组装。
 * 绝不做耗时操作——中断里超时会丢掉后续字节。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
    {
        return;
    }

    /* 1. 入环形缓冲区 */
    Ble_RxPush(s_rx_byte);

    /* 2. 行组装：以 \n 或 \r 结尾算一行。
         连续的行尾符（\r\n）只认第一个，避免产生空行。 */
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
        /* 上一行还没被取走就不再往行缓冲里塞，避免覆盖未读数据 */
        if (s_line_len < (BLE_LINE_BUFFER_SIZE - 1U))
        {
            s_line_buf[s_line_len] = (char)s_rx_byte;
            s_line_len++;
        }
    }

    s_last_char = (char)s_rx_byte;

    /* 3. 立刻重新挂上接收，保证不丢字节 */
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

/* ------------------------------------------------------------------------- */
/* 初始化                                                                      */
/* ------------------------------------------------------------------------- */

void BLE_Init(void)
{
    GPIO_PinState en_level;

    /* 清状态 */
    s_rx_head     = 0U;
    s_rx_tail     = 0U;
    s_rx_total    = 0U;
    s_rx_overflow = 0U;
    s_line_len    = 0U;
    s_line_ready  = 0U;
    s_last_char   = '\0';
    s_mode        = BLE_MODE_TRANSPARENT;
    s_last_connected = 0xFFU;

    /* 拉 EN 使能模块 */
#if (BLE_EN_ACTIVE_HIGH != 0)
    en_level = GPIO_PIN_SET;
#else
    en_level = GPIO_PIN_RESET;
#endif
    HAL_GPIO_WritePin(BLE_EN_PORT, BLE_EN_PIN, en_level);

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

    /*
     * 给一个明确的超时，不用 HAL_MAX_DELAY：
     * 115200 下 128 字节约 11ms，给 100ms 已经非常宽松。
     * 用 HAL_MAX_DELAY 的话，模块被拔掉就会永久卡死在这里。
     */
    status = HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100U);

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
        copy_len = (uint16_t)(max_len - 1U);    /* 截断保护 */
    }

    memcpy(line, s_line_buf, copy_len);
    line[copy_len] = '\0';

    /* 复位行组装状态，准备接收下一行 */
    s_line_len   = 0U;
    s_line_ready = 0U;

    return 1U;
}

void BLE_FlushRx(void)
{
    s_rx_tail    = s_rx_head;       /* 丢弃全部未读字节 */
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

    /* AT 指令必须以 \r\n 结尾，手册明确要求 */
    cmd_len = (uint16_t)snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);
    if ((cmd_len == 0U) || (cmd_len >= sizeof(cmd_buf)))
    {
        return 0U;      /* 指令太长，截断了 */
    }

    /* 发指令前清空接收，否则会把上一条的残留响应当成本次结果 */
    BLE_FlushRx();
    s_mode = BLE_MODE_AT;

    if (BLE_Send((const uint8_t *)cmd_buf, cmd_len) == 0U)
    {
        s_mode = BLE_MODE_TRANSPARENT;
        return 0U;
    }

    /*
     * 收响应：不等到固定长度，而是"只要还有字节持续到达就继续收，
     * 直到连续 BLE_AT_GAP_TIMEOUT_MS 没有新字节"。
     * 这样既能收 3 字节的 "+OK"，也能收几十字节的 "+VERSION=..."。
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

        /* 没有新字节了，判断是否已经静默足够久 */
        if ((HAL_GetTick() - last_rx_tick) >= BLE_AT_GAP_TIMEOUT_MS)
        {
            break;
        }
    }

    if (response != NULL)
    {
        response[got] = '\0';
    }

    s_mode = BLE_MODE_TRANSPARENT;

    return (got > 0U) ? 1U : 0U;
}

uint8_t BLE_QueryVersion(char *response, uint16_t resp_size)
{
    return BLE_SendAT("AT+VERSION", response, resp_size);
}

uint8_t BLE_QueryBaud(void)
{
    char    resp[BLE_LINE_BUFFER_SIZE];
    char   *pos;

    if (BLE_SendAT("AT+BAUD", resp, sizeof(resp)) == 0U)
    {
        return 0U;
    }

    /* 期望形如 "+BAUD=4" */
    pos = strstr(resp, "BAUD=");
    if (pos == NULL)
    {
        return 0U;
    }

    pos += 5;       /* 跳过 "BAUD=" */

    if ((*pos < '4') || (*pos > '9'))
    {
        return 0U;
    }

    return (uint8_t)(*pos - '0');
}

uint8_t BLE_SetBaud(uint8_t baud_code)
{
    char cmd[16];
    char resp[BLE_LINE_BUFFER_SIZE];

    if ((baud_code < BLE_BAUD_9600) || (baud_code > BLE_BAUD_128000))
    {
        return 0U;
    }

    (void)snprintf(cmd, sizeof(cmd), "AT+BAUD%u", (unsigned int)baud_code);
    return BLE_SendAT(cmd, resp, sizeof(resp));
}

uint8_t BLE_SetName(const char *name)
{
    char cmd[64];
    char resp[BLE_LINE_BUFFER_SIZE];

    if (name == NULL)
    {
        return 0U;
    }

    /* 手册限制广播名最长 18 字节 */
    if (strlen(name) > 18U)
    {
        return 0U;
    }

    (void)snprintf(cmd, sizeof(cmd), "AT+NAME%s", name);
    return BLE_SendAT(cmd, resp, sizeof(resp));
}

uint8_t BLE_SetPin(const char *pin)
{
    char cmd[16];
    char resp[BLE_LINE_BUFFER_SIZE];

    if ((pin == NULL) || (strlen(pin) != 4U))
    {
        return 0U;      /* 手册要求 4 位 */
    }

    (void)snprintf(cmd, sizeof(cmd), "AT+PIN%s", pin);
    return BLE_SendAT(cmd, resp, sizeof(resp));
}

uint8_t BLE_Reset(void)
{
    char    resp[BLE_LINE_BUFFER_SIZE];
    uint8_t ok = BLE_SendAT("AT+RESET", resp, sizeof(resp));

    /* 复位需要时间，期间不要再发指令 */
    HAL_Delay(500U);

    /* 复位会打断接收中断链，重新挂上 */
    BLE_FlushRx();
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);

    return ok;
}

uint8_t BLE_Disconnect(void)
{
    char resp[BLE_LINE_BUFFER_SIZE];

    if (BLE_IsConnected() == 0U)
    {
        return 0U;      /* 手册：仅在连接状态下有效 */
    }

    return BLE_SendAT("AT+DISC", resp, sizeof(resp));
}

uint8_t BLE_RestoreDefault(void)
{
    char    resp[BLE_LINE_BUFFER_SIZE];
    uint8_t ok = BLE_SendAT("AT+DEFAULT", resp, sizeof(resp));

    HAL_Delay(500U);
    BLE_FlushRx();
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);

    return ok;
}

uint8_t BLE_SetLogEnable(uint8_t enable)
{
    char cmd[16];
    char resp[BLE_LINE_BUFFER_SIZE];

    (void)snprintf(cmd, sizeof(cmd), "AT+ENLOG%u", (enable != 0U) ? 1U : 0U);
    return BLE_SendAT(cmd, resp, sizeof(resp));
}

/* ------------------------------------------------------------------------- */
/* LED 指示                                                                    */
/* ------------------------------------------------------------------------- */

void BLE_UpdateLed(void)
{
    uint8_t connected = BLE_IsConnected();

    /* 状态没变就什么都不做。
       这一点很重要：如果每拍都调 Led_BlinkForever，闪烁的计时会被反复重置，
       LED 就变成常亮了，看不出"在闪"。 */
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
    char     msg[224];      /* 表头较长，单独用一个足够大的缓冲区 */
    uint32_t elapsed_ms = 0U;
    uint32_t total_ms   = duration_sec * 1000U;
    uint32_t next_print = 0U;

    {
        int len = snprintf(msg, sizeof(msg),
                           "\r\n[Ble] JDY-31 status monitor for %u s\r\n"
                           "  STAT=PA1  EN=PB3  UART=USART2(PA2/PA3)\r\n"
                           "  note: this text goes out on the SAME uart as the module\r\n"
                           "  time   STAT  connected  rx_total  overflow\r\n",
                           (unsigned int)duration_sec);
        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)len, 100U);
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

            len = snprintf(buf, sizeof(buf), "  %5u   %u     %u        %6u    %u\r\n",
                           (unsigned int)elapsed_ms,
                           (unsigned int)HAL_GPIO_ReadPin(BLE_STATE_PORT, BLE_STATE_PIN),
                           (unsigned int)BLE_IsConnected(),
                           (unsigned int)s_rx_total,
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
    char     resp[BLE_LINE_BUFFER_SIZE];
    uint32_t i;

    static const char *test_cmds[] =
    {
        "AT+VERSION",
        "AT+BAUD",
        "AT+NAME",
        "AT+LADDR",
    };

    {
        char msg[224];      /* 表头较长，不该复用只用来装 AT 响应的 resp */

        int len = snprintf(msg, sizeof(msg),
                           "\r\n[Ble] AT verification, %u commands\r\n"
                           "  if nothing comes back: baud rate mismatch (module default %u)\r\n"
                           "  if you see garbage    : baud rate mismatch\r\n",
                           (unsigned int)(sizeof(test_cmds) / sizeof(test_cmds[0])),
                           (unsigned int)BLE_BAUD_DEFAULT);
        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)len, 100U);
        }
    }

    for (i = 0U; i < (sizeof(test_cmds) / sizeof(test_cmds[0])); i++)
    {
        /* 缓冲区要能装下"模块回显 + 真正响应"，最长的是 AT+VERSION 的响应 */
        char line[256];
        int  len;

        /* 打印我们发出去的指令，便于和回显对照 */
        len = snprintf(line, sizeof(line), "  --> %s\r\n", test_cmds[i]);
        if (len > 0)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, 100U);
        }

        /* 模块通常会把指令原样回显一遍，再跟真正的响应 */
        if (BLE_SendAT(test_cmds[i], resp, sizeof(resp)) != 0U)
        {
            len = snprintf(line, sizeof(line), "  <-- %s\r\n", resp);
        }
        else
        {
            len = snprintf(line, sizeof(line), "  <-- (no response, timeout)\r\n");
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
