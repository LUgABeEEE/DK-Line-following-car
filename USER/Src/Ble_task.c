//
// Created by JiangyaF on 2026/10/22.
//
// 蓝牙任务（FreeRTOS）—— 骨架版
//
// 本文件只保留任务骨架，业务逻辑留空由你填写。
// 驱动层（Ble.c）的收发接口保持不变，直接用即可：
//
//   发送：BLE_Send(buf, len)  /  BLE_SendString("text")  /  BLE_SendByte(b)
//   接收：BLE_GetByte(&b)     /  BLE_GetLine(buf, len)
//   状态：BLE_IsConnected()
//
// 任务参数：
//   优先级 2，栈 320 字（1280 字节），周期 100ms
//
// ⚠️ 关于调试打印：本任务所有输出都走 USART2，也就是 HC-05 所接的串口。
//    所以 BleTask_Print() 的内容手机端也能看到；反过来手机发来的数据
//    也会进入本任务的接收缓冲。调试打印和数据通道是同一个串口。
//

#include "Ble_task.h"
#include "Ble.h"
#include "Led.h"
#include "Control_task.h"   /* ControlTask_RequestStop() */
#include "Gray.h"           /* 灰度传感器调试输出 */
#include "SpeedPID.h"       /* 内环 PID 运行数据 */
#include "PWM.h"            /* MOTOR_A / MOTOR_B */
#include "LinePD.h"         /* 外环运行数据 */
#include "Encoder.h"        /* 编码器原始计数与里程 */
#include "Oled_task.h"      /* OledTask_GetStackFreeWords() */
#include "MPU6050.h"        /* 陀螺仪状态/航向 */

/*
 * 构建标记。
 *
 * 直接放进 [L] 行的开头（而不是单独一行），原因：
 *   手机的串口 APP 常在 50 字符左右截断/换行，
 *   单独一行 [VER] 容易被用户错过，行尾的版本线索又会被截掉。
 *   放在 [L] 行最前面，只要能看到这一行就一定看得到版本。
 *
 * 每次行为有变化就改这里的字母。
 */
#define BLE_BUILD_TAG   "cb"

/*
 * UART 发送超时，单位 ms。
 *
 * 【为什么必须按行长算，不能拍脑袋给 50】
 *   USART2 是 9600 8N1，1 个字节约 1.04ms。
 *   [L] 行现在有 60 字节左右，完整发完要约 62ms。
 *   以前这里写 50U —— 结果是【每一行 [L] 都在 "sr" 处被截断】，
 *   后面紧跟的 [A] 行被拼到同一行里，日志看起来像
 *   "sc=NONE cd+0 sr[A] t0.0 ..."。
 *   实车日志（logs/hc05test.log）里 22 行 [L] 全部截断，就是这么来的。
 *
 *   给到 250ms 留足余量：即使被更高优先级任务抢占几十毫秒，
 *   也不会因为超时把一帧数据切掉半截。
 */
#define BLE_TX_TIMEOUT_MS   250U
#include "queue.h"          /* 跑圈结果标记队列 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* heap_4 的查询接口（FreeRTOS.h 不保证给出原型，这里自己声明） */
extern size_t xPortGetFreeHeapSize(void);

/* 运行期资源诊断周期：每 10s 打一行 [M]，看堆和任务栈的余量 */
#define BLE_DIAG_PERIOD_MS      10000U

/*
 * 跑圈结果标记队列。
 *
 * 控制任务只负责"按了哪个键"，把 1/0 塞进来；串口的实际打印
 * 永远发生在蓝牙任务里。长度 8 足够（人不会在 25ms 内连按 8 次）。
 */
#define BLE_TAG_QUEUE_LEN       8U
static QueueHandle_t s_tag_queue = NULL;
static uint16_t      s_tag_seq   = 0U;

/* 串口句柄定义在 main.c 中 */
extern UART_HandleTypeDef huart2;

/* 灰度传感器实例定义在 main.c 中 */
extern Gray_Sensor_t g_gray;

/* I2C1 句柄（灰度传感器所挂总线） */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/* 外环实例定义在 main.c 中 */
extern Line_PD_t g_line_pd;

/* ------------------------------------------------------------------------- */
/* 灰度传感器调试输出                                                          */
/* ------------------------------------------------------------------------- */

/*
 * 定期把灰度传感器的原始数据打给手机，用于：
 *   1. 判断自学习校准是否正确（模拟值 vs 阈值）
 *   2. 判断 8 路通道顺序是否对应
 *   3. 观察线位置和信号强度
 *
 * 9600 波特率下每 100ms 只能传约 96 字节，所以周期取 1 秒，
 * 单次输出约 120 字节（约 125ms），占空比 12%，不影响实时性。
 *
 * 可以用手机发 'D' 命令开关。
 */
#define GRAY_DUMP_PERIOD_MS     2000U

/*
 * 是否输出灰度数据。1 = 开，0 = 关。
 *
 * 【为什么做成编译期常量而不是运行时命令】
 *   之前用手机发 'D' 来开关，结果经常莫名其妙变成 OFF——
 *   原因是 HC-05 连接瞬间缓冲区里可能有杂散字节，
 *   或者手机 App 的按钮会误发 'D'，被当成了关闭命令。
 *
 *   灰度调试数据属于"开着也没坏处"的信息，不需要频繁开关，
 *   所以改成编译期开关：要关就改这里重新编译。
 */
#define GRAY_DUMP_ENABLE        1

static uint32_t s_gray_dump_tick = 0U;

/* ------------------------------------------------------------------------- */
/* PID 运行数据输出                                                            */
/* ------------------------------------------------------------------------- */

/*
 * 每 PID_DUMP_PERIOD_MS 输出三行，内环外环都覆盖。
 *
 * 输出示例（单位 cm/s，output 是 PWM 占空比 -100~100）：
 *   [L] e+1.2 s+9.6 L24.6 R35.4     <- 外环：误差/差速/左右目标
 *   [A] t30.0 m28.5 o72.0 i1.3      <- 内环A：目标/实测/输出/积分
 *   [B] t30.0 m29.1 o68.0 i1.1      <- 内环B
 *
 * 怎么用这些数据调参：
 *
 *   内环（看 [A] / [B] 两行）
 *     m 长期低于 t            -> KI 偏小或 KFF 偏小，轮子带不动
 *     m 在 t 附近振荡          -> KP 偏大
 *     o 已经打到 ±100 还是跟不上 -> 电压/负载问题，不是增益问题
 *     i 一直很大（>50）        -> 积分饱和，起步会冲，应减小 KI
 *
 *   外环（看 [L] 那一行）
 *     e 和 s 符号相同          -> 方向正确（线在左时两者都为正）
 *     L > R                    -> 车会右转
 *     L < R                    -> 车会左转
 *     线在左却 L > R           -> 差速方向反了
 */
#define PID_DUMP_PERIOD_MS      160U

static uint32_t s_pid_dump_tick = 0U;
/* 内环两行的分频计数：每两次才发一次 [A]/[B]（2.5Hz），省 9600 带宽 */
static uint8_t  s_speed_div = 0U;

/* ------------------------------------------------------------------------- */
/* 任务内部状态                                                                */
/* ------------------------------------------------------------------------- */

static TaskHandle_t s_task_handle = NULL;

/* 上一次的连接状态，用于检测变化。0xFF 表示尚未初始化 */
static uint8_t  s_last_connected = 0xFFU;
static uint32_t s_connect_count  = 0U;

/* ------------------------------------------------------------------------- */
/* 内部辅助                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * 统一的任务内打印入口。
 *
 * 走 USART2，所以同时出现在：
 *   - USB-TTL 串口助手（旁听 PA3）
 *   - 手机蓝牙串口 App
 *
 * 注意：这是阻塞发送，超时 100ms。在任务上下文里可以接受
 *      （优先级 2，低于控制任务 3，不会抢占实时控制）。
 */
static void BleTask_Print(const char *fmt, ...)
{
    char    buf[192];
    va_list args;
    int     len;

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0)
    {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, BLE_TX_TIMEOUT_MS);
    }
}

/* ------------------------------------------------------------------------- */
/* 灰度传感器调试输出                                                          */
/* ------------------------------------------------------------------------- */

/*
 * 输出一行 8 路数据。line_type 决定是模拟值 'A' 还是阈值 'T'。
 */
/* 前置声明：DumpGray 里的 [S] 行要用到 */
static const char *BleTask_Fmt1(char *buf, uint16_t size, float value);

static void BleTask_DumpGrayArray(char line_type, const uint16_t *arr)
{
    char     line[160];
    int      len;
    uint32_t i;

    len = snprintf(line, sizeof(line), "[G] %c", (int)line_type);

    for (i = 0U; (i < GRAY_CHANNEL_COUNT) && (len > 0) && (len < (int)sizeof(line) - 8); i++)
    {
        len += snprintf(&line[len], sizeof(line) - (size_t)len,
                        " %4u", (unsigned int)arr[i]);
    }

    if (len > 0)
    {
        /* 行尾补 \r\n */
        if (((size_t)len + 2U) < sizeof(line))
        {
            line[len]     = '\r';
            line[len + 1] = '\n';
            line[len + 2] = '\0';
            (void)HAL_UART_Transmit(&huart2, (uint8_t *)line,
                                    (uint16_t)(len + 2), BLE_TX_TIMEOUT_MS);
        }
    }
}

/*
 * 完整输出一次灰度传感器状态。
 *
 * 输出格式（3 行）：
 *   [G] S=18 P=-7.3mm str=1834 e=0/1234 L=0
 *   [G] A 1834 1756 1203  990  872  801  760  745
 *   [G] T 1200 1200 1200 1200 1200 1200 1200 1200
 *
 * 字段含义：
 *   S  数字状态字节（十六进制，bit0 = 通道1）
 *   P  线位置，单位 mm，左正右负
 *   str 归一化信号强度之和，接近 0 说明没看到线
 *   e  I2C 累计失败/总读取 —— 判断硬件健康度
 *   L  丢线标志（1 = 所有通道都没看到线）
 *   A  8 路模拟原始值（0~4095）
 *   T  8 路阈值
 *
 * 怎么用它判断校准是否正确：
 *   看 A 和 T 两行——看到线的通道，A 应该明显【小于】T
 *   （白底黑线时，黑线反射弱 -> 模拟值低）。
 *   如果所有 A 都远大于 T，或者所有 A 都远小于 T，
 *   说明阈值学得不对，需要重新做自学习校准。
 */
static void BleTask_DumpGray(void)
{
    char    line[128];
    int     len;
    int32_t pos_tenths;
    int32_t ip;
    int32_t fp;

    /* 位置换算成 0.1mm 精度的整数，避开 newlib-nano 不支持 %f 的问题 */
    pos_tenths = (int32_t)(g_gray.position_mm * 10.0f
                           + ((g_gray.position_mm >= 0.0f) ? 0.5f : -0.5f));
    ip = pos_tenths / 10;
    fp = pos_tenths % 10;
    if (fp < 0)
    {
        fp = -fp;
    }

    len = snprintf(line, sizeof(line),
                   "[G] S=%02X P=%ld.%ldmm str=%ld e=%lu/%lu L=%u\r\n",
                   (unsigned int)g_gray.state_byte,
                   (long)ip, (long)fp,
                   /* 强度放大 100 倍显示：0.76 -> 76，避免被整数截断成 0 */
                   (long)(g_gray.strength * 100.0f),
                   (unsigned long)g_gray.total_fails,
                   (unsigned long)g_gray.total_reads,
                   (unsigned int)g_gray.line_lost);

    if (len > 0)
    {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)line,
                                (uint16_t)strlen(line), BLE_TX_TIMEOUT_MS);
    }

    BleTask_DumpGrayArray('A', g_gray.analog);
    BleTask_DumpGrayArray('T', g_gray.threshold);

    /*
     * 场景判据的原始特征，单独一行（2 秒一次）。
     *
     * 放在这里而不是 [L] 行，是因为 [L] 每 200ms 一次、容易被 APP 截断；
     * 这些是标定阈值用的数据，不需要那么高的刷新率。
     *
     *   ln/rt : 左/右半边激活路数
     *   cl/sp : 簇个数 / 最大簇跨度
     *   lv    : 最后一次有效位置 cm
     */
    {
        char sv[14];

        (void)snprintf(line, sizeof(line), "[S] ln%u rt%u cl%u sp%u lv%s\r\n",
                       (unsigned int)g_gray.scene_left,
                       (unsigned int)g_gray.scene_right,
                       (unsigned int)g_gray.scene_clusters,
                       (unsigned int)g_gray.scene_span,
                       BleTask_Fmt1(sv, sizeof(sv),
                                    g_gray.last_valid_mm * 0.1f));
    }
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)line,
                            (uint16_t)strlen(line), BLE_TX_TIMEOUT_MS);
}

/* ------------------------------------------------------------------------- */
/* I2C 总线扫描（诊断用）                                                      */
/* ------------------------------------------------------------------------- */

/*
 * 扫描 I2C1 上所有 7 位地址，报告谁应答。
 *
 * 这是排查"I2C 完全读不到数据"最快的手段：
 *
 *   一个都没有
 *       -> 总线上根本没有器件应答。最可能是【缺上拉电阻】：
 *          I2C 是开漏总线，没有上拉时 SDA/SCL 都是浮空，
 *          永远拉不高，任何地址都不会 ACK。
 *          其次是 SDA/SCL 接反、或者模块没供电。
 *
 *   只应答 0x5D
 *       -> 正常，灰度传感器在 0x5D。
 *
 *   应答一串连续地址（比如 0x08~0x77 全应答）
 *       -> 总线被短路到地，或上拉完全失效（读到的都是"假 ACK"）。
 *
 *   应答了别的地址
 *       -> 传感器地址和资料不符，用扫出来的地址改 GRAY_I2C_ADDR_7BIT。
 *
 * 用手机发 'I' 触发。
 */
static void BleTask_ScanI2C(void)
{
    uint8_t  addr;
    uint8_t  found = 0U;
    uint32_t mapr;

    /*
     * 顺便扫一下 I2C2（OLED + MPU6050 共总线）。
     * MPU6050：AD0=GND -> 0x68，AD0=VCC -> 0x69；这里能直接看出模块在不在。
     */
    {
        uint8_t i2c2_found = 0U;

        BleTask_Print("[I2C2] scan:");
        for (addr = 0x08U; addr < 0x78U; addr++)
        {
            if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(addr << 1), 2U, 5U) == HAL_OK)
            {
                BleTask_Print(" 0x%02X", (unsigned int)addr);
                i2c2_found++;
            }
        }
        BleTask_Print(i2c2_found ? "\r\n" : " (none)\r\n");
    }

    /*
     * 先报告三个硬件状态，这是判断"为什么读不到"的关键：
     *
     *   MAPR  : AFIO_MAPR 寄存器的值。
     *           bit1 = I2C1_REMAP。必须为 1，否则 I2C1 还在默认的
     *           PB6/PB7 上（那两根线被 TIM4 编码器占着），
     *           传感器接的 PB8/PB9 根本不会有信号。
     *
     *   ST    : HAL 的 I2C 状态机。0=READY 正常；其它值说明外设卡住了。
     *
     *   ERR   : HAL 记录的 I2C 错误码。4=NACK，2=总线错误，1=超时。
     */
    /*
     * 先做一次总线恢复。
     * 如果之前总线被从机卡住（表现为 err=TIMEOUT），
     * 这一步很可能直接把它救回来，随后扫描就能找到器件。
     */
    BleTask_Print("[I2C] running bus recovery...\r\n");
    Gray_BusRecovery(&hi2c1);

    mapr = AFIO->MAPR;

    BleTask_Print("[I2C] MAPR=0x%08lX  I2C1_REMAP=%u\r\n",
                  (unsigned long)mapr,
                  (unsigned int)((mapr & AFIO_MAPR_I2C1_REMAP) ? 1U : 0U));
    BleTask_Print("[I2C] state=%u err=0x%08lX\r\n",
                  (unsigned int)hi2c1.State,
                  (unsigned long)hi2c1.ErrorCode);

    BleTask_Print("[I2C] scanning bus 1 (PB8=SCL, PB9=SDA)...\r\n");

    for (addr = 1U; addr < 128U; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 2U, 10U) == HAL_OK)
        {
            BleTask_Print("[I2C] ACK at 0x%02X\r\n", (unsigned int)addr);
            found++;
        }

        /* 扫描期间让出 CPU，避免长时间阻塞蓝牙任务 */
        if ((addr % 16U) == 0U)
        {
            vTaskDelay(pdMS_TO_TICKS(5U));
        }
    }

    if (found == 0U)
    {
        BleTask_Print("[I2C] NO DEVICE. Check: 1) pull-up resistors "
                      "2) SDA/SCL wiring 3) sensor power\r\n");
    }
    else
    {
        BleTask_Print("[I2C] scan done, %u device(s)\r\n", (unsigned int)found);
    }
}

/*
 * 把浮点数格式化成带 1 位小数的字符串。
 *
 * 不能用 snprintf 的 %f —— newlib-nano 没有浮点格式化支持，
 * 会输出空字符串。这里用整数缩放自己做。
 */
static const char *BleTask_Fmt1(char *buf, uint16_t size, float value)
{
    int32_t scaled;
    int32_t ip;
    int32_t fp;

    if ((buf == NULL) || (size == 0U))
    {
        return buf;
    }

    scaled = (int32_t)(value * 10.0f + ((value >= 0.0f) ? 0.5f : -0.5f));
    ip = scaled / 10;
    fp = scaled % 10;
    if (fp < 0)
    {
        fp = -fp;
    }

    (void)snprintf(buf, size, "%ld.%ld", (long)ip, (long)fp);
    return buf;
}

/* 前置声明（DumpGains 用） */
/*
 * 输出一次 PID 运行数据：外环一行 + 内环两行。
 */
static void BleTask_DumpPid(uint8_t dump_speed)
{
    char line[128];
    char b1[14];
    char b2[14];
    char b3[14];
    char b4[14];
    /*
     * 里程单独一个缓冲。
     * 之前和 b3（输出）共用了，而 snprintf 的参数求值顺序不确定，
     * 结果 o 打印出来的其实是里程值 —— 排查"输出和公式对不上"时
     * 白走了不少弯路。
     */
    char b5[14];

    /*
     * ---- 外环 ----
     *
     * 行首带上模式标记，避免"看着数据以为是循迹、其实在速度测试"这种误判。
     *   STOP / SPD / LINE
     *
     * 注意：只有 LINE 模式下 L/R 才会有非零值，
     * 其它模式外环不执行，这四个字段恒为 0。
     */
    {
        const char *mode_tag;

        switch (g_ctrl_mode)
        {
        case CTRL_MODE_LINE_FOLLOW: mode_tag = "LINE"; break;
        case CTRL_MODE_SPEED_TEST:  mode_tag = "SPD "; break;
        default:                    mode_tag = "STOP"; break;
        }

        /*
         * 末尾的 s±1 是【搜索方向锁存值】：
         *    0  = 当前不在搜索状态
         *   +1  = 正在向左搜索
         *   -1  = 正在向右搜索
         *
         * 加这个字段是为了让"丢线后往哪转"这个决策可见、可核对，
         * 而不是一个黑盒。如果看到方向和自己判断的不一致，
         * 就能立刻从这个字段确认是哪一拍锁存的。
         */
        /*
         * 行尾字段：
         *   sd±1 : 搜索方向锁存值
         *   sc=?? : 防抖之后的稳定场景
         *           0=NONE 1=LINE 2=ALL 3=SIDE_L 4=SIDE_R 5=FORK
         *   cd±1  : 角点事件锁存的方向（+1 左 / -1 右 / 0 未锁存）
         *   sr=?? : 本帧【未经防抖】的瞬时场景（编号同 sc）
         *   cs=0/1: 是否处于角点事件中
         *   y±deg : 编码器估算的偏航角（左转为正），用于"定角转弯"标定
         *           看进弯前和重新找到线时的差值，就知道这个弯要转多少度
         *
         * sr 和 sc 放在同一行，是用来验证防抖效果的：
         *   锐角弯里 sr 会快速在 3/4/5 之间跳，而 sc 应该稳定，
         *   cs 应该在进弯时变 1、出弯 3 拍后回 0。
         *   如果 sc 也跟着 sr 跳，说明 GRAY_SCENE_*_SAMPLES 还要调大。
         */
        {
            static const char *scene_name[] = {
                "NONE", "LINE", "ALL ", "SIDE_L", "SIDE_R", "FORK"
            };
            const char *sname = (g_line_pd.scene_last < 6U)
                                ? scene_name[g_line_pd.scene_last] : "?";

            (void)snprintf(line, sizeof(line),
                           "[L%s] %s e%s s%s L%s R%s sd%+d sc=%s cd%+d sr=%d cs=%d y%+d\r\n",
                           BLE_BUILD_TAG,
                           mode_tag,
                           BleTask_Fmt1(b1, sizeof(b1), g_line_pd.error),
                           BleTask_Fmt1(b2, sizeof(b2), g_line_pd.steer),
                           BleTask_Fmt1(b3, sizeof(b3), g_line_pd.left_target),
                           BleTask_Fmt1(b4, sizeof(b4), g_line_pd.right_target),
                           (int)g_line_pd.search_dir,
                           sname,
                           (int)g_line_pd.corner_dir,
                           (int)g_gray.scene_raw,
                           (int)g_line_pd.corner_active,
                           (int)g_line_pd.yaw_deg);
        }
    }
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)line,
                            (uint16_t)strlen(line), BLE_TX_TIMEOUT_MS);

    /*
     * 内环两行按 2.5Hz 分频：dump_speed=0 的这一拍照常只发外环。
     * 9600 带宽下三行同时 5Hz 会超过 60%，挤掉灰度行并拖慢急停响应。
     */
    if (dump_speed == 0U)
    {
        return;
    }

    /*
     * ---- 内环 A ----
     *
     * 末尾额外输出两个原始量：
     *   c  = 编码器计数器当前值（0~65535，会循环）
     *   d  = 累计里程 cm
     *
     * 这两个是用来判断"编码器到底有没有在计数"的硬指标：
     *   车明显在动，但 c 一直不变、d 一直是 0
     *     -> 编码器信号没进来（接线/供电/上拉问题），
     *        速度环实际上在开环运行，m 恒为 0。
     */
    (void)snprintf(line, sizeof(line), "[A] t%s m%s o%s i%s c%u d%s\r\n",
                   BleTask_Fmt1(b1, sizeof(b1), Speed_PID_GetTarget(MOTOR_A)),
                   BleTask_Fmt1(b2, sizeof(b2), Speed_PID_GetMeasure(MOTOR_A)),
                   BleTask_Fmt1(b3, sizeof(b3), Speed_PID_GetOutput(MOTOR_A)),
                   BleTask_Fmt1(b4, sizeof(b4), Speed_PID_GetIntegral(MOTOR_A)),
                   (unsigned int)Encoder_GetCount(ENCODER_A),
                   BleTask_Fmt1(b5, sizeof(b5), Encoder_GetDistance(ENCODER_A)));
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)line,
                            (uint16_t)strlen(line), BLE_TX_TIMEOUT_MS);

    /* ---- 内环 B ---- */
    (void)snprintf(line, sizeof(line), "[B] t%s m%s o%s i%s c%u d%s\r\n",
                   BleTask_Fmt1(b1, sizeof(b1), Speed_PID_GetTarget(MOTOR_B)),
                   BleTask_Fmt1(b2, sizeof(b2), Speed_PID_GetMeasure(MOTOR_B)),
                   BleTask_Fmt1(b3, sizeof(b3), Speed_PID_GetOutput(MOTOR_B)),
                   BleTask_Fmt1(b4, sizeof(b4), Speed_PID_GetIntegral(MOTOR_B)),
                   (unsigned int)Encoder_GetCount(ENCODER_B),
                   BleTask_Fmt1(b5, sizeof(b5), Encoder_GetDistance(ENCODER_B)));
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)line,
                            (uint16_t)strlen(line), BLE_TX_TIMEOUT_MS);
}

/*
 * 输出当前内环增益。每 2 秒一次，和灰度数据同一节拍。
 *
 * 只有四个增益，一行就够：
 *   [K] A kp0.80 ki1.50 kd0.00 kff1.00
 *
 * 用来核对"按键调参到底改没改到"，以及排查
 * "输出占空比和公式对不上"这类问题（往往是增益和记忆中的不一样）。
 */
static void BleTask_DumpGains(void)
{
    char line[128];
    char b1[14];
    char b2[14];
    char b3[14];
    char b4[14];

    (void)snprintf(line, sizeof(line), "[K] A kp%s ki%s kd%s kff%s\r\n",
                   BleTask_Fmt1(b1, sizeof(b1),
                                Speed_PID_GetGain(MOTOR_A, SPEED_PID_GAIN_KP)),
                   BleTask_Fmt1(b2, sizeof(b2),
                                Speed_PID_GetGain(MOTOR_A, SPEED_PID_GAIN_KI)),
                   BleTask_Fmt1(b3, sizeof(b3),
                                Speed_PID_GetGain(MOTOR_A, SPEED_PID_GAIN_KD)),
                   BleTask_Fmt1(b4, sizeof(b4),
                                Speed_PID_GetGain(MOTOR_A, SPEED_PID_GAIN_KFF)));
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)line,
                            (uint16_t)strlen(line), BLE_TX_TIMEOUT_MS);

    /*
     * 外环增益单独一行。
     * 之前只输出了内环，排查循迹问题时看不到 LKP/LKD，只能靠反推。
     */
    (void)snprintf(line, sizeof(line), "[P] lkp%s lkd%s lbas%s yff%s\r\n",
                   BleTask_Fmt1(b1, sizeof(b1), g_line_pd.kp),
                   BleTask_Fmt1(b2, sizeof(b2), g_line_pd.kd),
                   BleTask_Fmt1(b3, sizeof(b3), g_line_pd.base_speed),
                   BleTask_Fmt1(b4, sizeof(b4), g_line_pd.theta_rate_ff));
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)line,
                            (uint16_t)strlen(line), BLE_TX_TIMEOUT_MS);
}

/* ------------------------------------------------------------------------- */
/* 任务主体                                                                    */
/* ------------------------------------------------------------------------- */

static void BleTask_Main(void *argument)
{
    (void)argument;

    /* 让出一次 CPU，确保串口和 LED 初始化完成后再开始打印 */
    vTaskDelay(pdMS_TO_TICKS(100U));

    /*
     * 注意：本任务【不再】驱动电机。
     * 电机完全由 ControlTask 的内环/外环管理，
     * 这里只保留一个"停车"命令作为安全兜底。
     */
    BleTask_Print("\r\n[BleTask] started, period=%ums\r\n",
                  (unsigned int)BLE_TASK_PERIOD_MS);
    BleTask_Print("[BleTask] motor control: NONE (handled by ControlTask)\r\n");
    /*
     * 固件版本标记。
     *
     * 排查问题时最怕"以为烧了新固件其实没有"，或者在两种版本间来回搞混。
     * 每次有重要改动就更新这里的日期字母后缀，连上蓝牙就能确认。
     */
    BleTask_Print("[VER] line-follower " BLE_BUILD_TAG " P%d (built " __DATE__ " " __TIME__ ")\r\n",
                  (int)LINE_SIMPLE_PROFILE);
    BleTask_Print("[BleTask] X=stop I=scan I2C N=name\r\n");

    BleTask_Print("[BLE] module=%s\r\n", BLE_ModuleName());

    /* 陀螺仪自检结果 */
    if (MPU6050_IsReady() != 0U)
    {
        BleTask_Print("[IMU] MPU6050 ok id=0x%02X biasZ=%+d (0.1dps)\r\n",
                      (unsigned int)MPU6050_GetWhoAmI(),
                      (int)(MPU6050_GetBiasDps() * 10.0f));
    }
    else
    {
        BleTask_Print("[IMU] MPU6050 NOT FOUND\r\n");
    }

    /* 同步一次连接状态，避免把"上电时已连接"误报成一次新连接 */
    s_last_connected = BLE_IsConnected();

    for (;;)
    {
        /*
         * ---- 蓝牙命令：只保留急停 ----
         *
         * 按【字节】取。识别 X / S / Q 三个字符任一即停车，
         * 覆盖不同 App 的习惯。
         *
         * 实现上只切模式（ControlTask_RequestStop），不直接调
         * Speed_PID_StopAll()——因为控制任务优先级更高，
         * 直接 Stop 会被它下一拍用旧目标覆盖回去。
         */
        {
            uint8_t b;

            while (BLE_GetByte(&b) != 0U)
            {
                switch ((char)b)
                {
                case 'X':
                case 'x':
                case 'S':
                case 's':
                case 'Q':
                case 'q':
                    ControlTask_RequestStop();
                    BleTask_Print("[BLE] EMERGENCY STOP\r\n");
                    break;

                case 'I':
                case 'i':
                    /* 扫描 I2C 总线，诊断灰度传感器通信问题 */
                    BleTask_ScanI2C();
                    break;

                case 'Z':
                case 'z':
                    /*
                     * 静止重新标定零偏。
                     * 上电标定时车可能在被人拿着/移动，零偏会标错，
                     * 表现为停车时 [I] 的 y 一直漂。发 Z 之前让车完全静止。
                     */
                    if (g_ctrl_mode != CTRL_MODE_STOP)
                    {
                        BleTask_Print("[IMU] recal ignored (not STOP)\r\n");
                    }
                    else if (MPU6050_IsReady() == 0U)
                    {
                        BleTask_Print("[IMU] recal skipped (no IMU)\r\n");
                    }
                    else if (MPU6050_Calibrate(400U) != 0U)
                    {
                        MPU6050_ResetYaw(0.0f);
                        BleTask_Print("[IMU] recal ok biasZ=%+d (0.1dps)\r\n",
                                      (int)(MPU6050_GetBiasDps() * 10.0f));
                    }
                    else
                    {
                        BleTask_Print("[IMU] recal FAILED\r\n");
                    }
                    break;

                case 'N':
                case 'n':
                    /*
                     * 把 HC-05 的广播名改成 LFCar（只成功一次即可，模块自己存着）。
                     * 前提：HC-05 处于 AT 模式（KEY/EN 在上电时为高）。
                     */
                    if (BLE_SetName("LFCar") != 0U)
                    {
                        BleTask_Print("[BLE] name set to LFCar\r\n");
                    }
                    else
                    {
                        BleTask_Print("[BLE] name set FAILED (need AT mode)\r\n");
                    }
                    break;


                default:
                    /* 其它字符忽略（包括 App 自动补的 \r \n） */
                    break;
                }
            }
        }

        /*
         * ---- 跑圈结果标记 ----
         *
         * [T] OK #3 t123456 / [T] FAIL #4 t130000
         * 分析日志时按 [T] 行切段，就能知道哪一段是成功/失败，
         * 不用再靠"看起来像成功"猜。
         */
        {
            uint8_t tag;

            while (xQueueReceive(s_tag_queue, &tag, 0U) == pdTRUE)
            {
                if (tag == 2U)
                {
                    /* 自动停车：识别到启停线（控制任务只入队，不阻塞串口） */
                    BleTask_Print("[RACE] det d%u lrn%u t%lu\r\n",
                                  (unsigned int)(g_line_pd.race_detect_cm - g_line_pd.race_start_cm),
                                  (unsigned int)LinePD_RaceLearnedLapCm(),
                                  (unsigned long)HAL_GetTick());
                }
                else if (tag == 3U)
                {
                    /* 自动停车：已到位 */
                    BleTask_Print("[RACE] stop t%lu\r\n",
                                  (unsigned long)HAL_GetTick());
                }
                else
                {
                    s_tag_seq++;
                    BleTask_Print("[T] %s #%u t%lu\r\n",
                                  (tag != 0U) ? "OK" : "FAIL",
                                  (unsigned int)s_tag_seq,
                                  (unsigned long)HAL_GetTick());
                }
            }
        }

        /*
         * ---- 灰度传感器数据输出 ----
         *
         * 每 GRAY_DUMP_PERIOD_MS 输出一次，手机端能看到 8 路模拟值
         * 和阈值。可以用 'D' 命令开关。
         */
        if ((GRAY_DUMP_ENABLE != 0) &&
            ((HAL_GetTick() - s_gray_dump_tick) >= GRAY_DUMP_PERIOD_MS))
        {
            s_gray_dump_tick = HAL_GetTick();
            BleTask_DumpGray();
            BleTask_DumpGains();
        }

        /*
         * ---- PID 运行数据 ----
         *
         * 外环 [L] 约 160~200ms 一次（5Hz）：60 字节 ≈ 62ms，占 9600 约 35%
         * 内环 [A]/[B] 降到 2.5Hz（隔一次发一次）：
         *   三行同时 5Hz 时合计约 124 字节 = 129ms/200ms，
         *   已经占掉 65% 带宽，会挤掉灰度行、拖慢急停响应。
         *   控制分析主要看 [L]，内环看趋势 2.5Hz 足够。
         */
        if ((HAL_GetTick() - s_pid_dump_tick) >= PID_DUMP_PERIOD_MS)
        {
            s_pid_dump_tick = HAL_GetTick();
            s_speed_div ^= 1U;
            BleTask_DumpPid(s_speed_div);
        }

        /*
         * ---- 陀螺仪遥测 ----
         *
         * [I] y<航向角> g<角速度> b<零偏×10> f<累计I2C失败>
         * 2s 一条，用来确认模块在线、方向符号是否正确。
         */
        {
            static uint32_t s_imu_tick = 0U;

            if (MPU6050_IsReady() != 0U)
            {
                if ((HAL_GetTick() - s_imu_tick) >= 2000U)
                {
                    s_imu_tick = HAL_GetTick();

                    BleTask_Print("[I] id0x%02X sg%+d y%+d g%+d b%+d f%u\r\n",
                                  (unsigned int)MPU6050_GetWhoAmI(),
                                  (int)MPU6050_YAW_SIGN,
                                  (int)MPU6050_GetYawDeg(),
                                  (int)MPU6050_GetYawRateDps(),
                                  (int)(MPU6050_GetBiasDps() * 10.0f),
                                  (unsigned int)MPU6050_GetFaults());
                }
            }
        }

        /*
         * ---- 运行期资源诊断 ----
         *
         * 每 10 秒一行，专门回答"长时间运行后空间够不够"：
         *   heap     : 剩余堆字节（任务栈、信号量都从这里分配）
         *   stkC/O/B : 三个任务栈的【历史最小剩余字节数】
         *              （uxTaskGetStackHighWaterMark × 4，越接近 0 越危险）
         *   i2c      : 灰度传感器累计失败/总读取，用来看长期错误率
         *
         * 跑久之后 stk 余量一直变小 -> 栈在持续增长；
         * heap 一直变小 -> 有动态内存泄漏。
         */
        {
            static uint32_t s_diag_tick = 0U;

            if ((HAL_GetTick() - s_diag_tick) >= BLE_DIAG_PERIOD_MS)
            {
                s_diag_tick = HAL_GetTick();

                BleTask_Print("[M] heap%u stkC%u stkO%u stkB%u i2c%u/%u lrn%u mod=%s cfg%u baud%lu\r\n",
                              (unsigned int)xPortGetFreeHeapSize(),
                              (unsigned int)(ControlTask_GetStackFreeWords() * 4U),
                              (unsigned int)(OledTask_GetStackFreeWords() * 4U),
                              (unsigned int)(BleTask_GetStackFreeWords() * 4U),
                              (unsigned int)g_gray.total_fails,
                              (unsigned int)g_gray.total_reads,
                              (unsigned int)LinePD_RaceLearnedLapCm(),
                              BLE_ModuleName(),
                              (unsigned int)BLE_GetCfgResult(),
                              (unsigned long)BLE_GetBaudrate());
            }
        }

        /* ---- 连接状态变化：更新 LED 并提示 ---- */
        {
            uint8_t connected = BLE_IsConnected();

            if (connected != s_last_connected)
            {
                s_last_connected = connected;

                if (connected != 0U)
                {
                    s_connect_count++;
                    BleTask_Print("[BLE] connected (times=%lu)\r\n",
                                  (unsigned long)s_connect_count);

                    /*
                     * 【每次连上都打印版本】
                     *
                     * 原来只在任务启动时打印一次，而那时手机通常还没连上，
                     * 结果永远看不到 —— 导致好几次搞不清烧的是哪一版。
                     * 放在这里就能保证每次连接都能确认。
                     */
                    BleTask_Print("[VER] line-follower " BLE_BUILD_TAG " P%d (built "
                                  __DATE__ " " __TIME__ ")\r\n",
                                  (int)LINE_SIMPLE_PROFILE);
                    BleTask_Print("[BLE] gray data: 3 lines per second\r\n");
                }
                else
                {
                    BleTask_Print("[BLE] disconnected\r\n");
                }
            }

            /* 未连接慢闪 / 已连接常亮。内部只在状态变化时重设 LED */
            BLE_UpdateLed();
        }

        vTaskDelay(pdMS_TO_TICKS(BLE_TASK_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------------- */
/* 任务创建                                                                    */
/* ------------------------------------------------------------------------- */

uint8_t BleTask_Init(void)
{
    BaseType_t ret;

    if (s_task_handle != NULL)
    {
        return 1U;      /* 已经创建过，避免重复创建 */
    }

    /* 跑圈结果标记队列（必须在任务启动前建好） */
    if (s_tag_queue == NULL)
    {
        s_tag_queue = xQueueCreate(BLE_TAG_QUEUE_LEN, sizeof(uint8_t));
        if (s_tag_queue == NULL)
        {
            return 0U;
        }
    }

    ret = xTaskCreate(BleTask_Main,
                      "BleTask",
                      BLE_TASK_STACK_WORDS,
                      NULL,
                      BLE_TASK_PRIORITY,
                      &s_task_handle);

    if (ret != pdPASS)
    {
        s_task_handle = NULL;
        return 0U;
    }

    return 1U;
}


uint32_t BleTask_GetStackFreeWords(void)
{
    if (s_task_handle == NULL)
    {
        return 0U;
    }
    return (uint32_t)uxTaskGetStackHighWaterMark(s_task_handle);
}

uint8_t BleTask_SendTag(uint8_t ok)
{
    uint8_t item = (ok != 0U) ? 1U : 0U;

    if (s_tag_queue == NULL)
    {
        return 0U;
    }

    return (xQueueSend(s_tag_queue, &item, 0U) == pdTRUE) ? 1U : 0U;
}

uint16_t BleTask_GetTagCount(void)
{
    return s_tag_seq;
}

//
// Created by JiangyaF on 2026/10/22.
//
