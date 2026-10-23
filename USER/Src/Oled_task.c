//
// Created by JiangyaF on 2026/10/22.
//
// OLED 显示 + PID 调参任务实现
//
// 核心机制：三个按键各用一个【计数信号量】累计事件，防止连按丢包。
//
// 为什么一个键一个信号量：
//   计数信号量只记录"发生了几次"，不记录"是哪个键"。
//   如果三个键共用一个信号量，取走 3 次计数时无法区分
//   "KEY1 按了 2 次、KEY2 按了 1 次" 还是 "KEY3 按了 3 次"，
//   调整方向和参数选择就全乱了。
//
// 取空信号量的写法：
//   while (xSemaphoreTake(sem, 0) == pdTRUE) { ... }
//   超时传 0 表示"有多少拿多少，不阻塞"，把当前所有待处理事件一次性取完。
//

#include "Oled_task.h"
#include "Oled.h"
#include "Key.h"
#include "Led.h"
#include "SpeedPID.h"
#include "Encoder.h"
#include "Gray.h"
#include "Control_task.h"
#include <stdio.h>
#include <string.h>

/*
 * 信号量 API 在 semphr.h 里，FreeRTOS.h 只包含基础类型和配置。
 * 不包含它会导致 SemaphoreHandle_t 未定义、xSemaphoreCreateCounting
 * 和 xSemaphoreGive 隐式声明。
 */
#include "semphr.h"

/* 串口句柄定义在 main.c 中，用于调试打印 */
extern UART_HandleTypeDef huart2;

/* 灰度传感器实例定义在 main.c 中，传感器调试页只读它 */
extern Gray_Sensor_t g_gray;

/* ------------------------------------------------------------------------- */
/* 可调参数表                                                                  */
/* ------------------------------------------------------------------------- */

/*
 * 每个参数的步长和上下限。
 *
 * 步长不是随便定的——必须和这个参数的典型量级匹配：
 *   步长太大 -> 按一下就过头，永远调不到最优点
 *   步长太小 -> 从 0 调到目标值要按上百下
 *
 * 详细理由见 Oled_task.h 的说明。
 */
typedef struct
{
    const char *name;       /* 显示用名称          */
    float       step;       /* 每按一下的变化量    */
    float       min;        /* 下限                */
    float       max;        /* 上限                */
} TuneParam_t;

#define TUNE_IDX_KP         0U
#define TUNE_IDX_KI         1U
#define TUNE_IDX_KD         2U
#define TUNE_IDX_KFF        3U
#define TUNE_IDX_TARGET     4U
#define TUNE_IDX_ENCA       5U
#define TUNE_IDX_SENSOR     6U
#define TUNE_PARAM_COUNT    7U

static const TuneParam_t s_params[TUNE_PARAM_COUNT] =
{
    /* 名称       步长     下限    上限   */
    { "KP",      0.10f,   0.00f,  5.00f },
    { "KI",      0.10f,   0.00f,  5.00f },
    { "KD",      0.01f,   0.00f,  1.00f },   /* KD 量级小，用 0.01 */
    { "KFF",     0.05f,   0.00f,  2.00f },
    { "TARGET",  1.00f,   0.00f, 40.00f },   /* 单位 cm/s */
    /*
     * ENCA: A 轮编码器的分轮修正系数。
     *
     * 步长 0.005 = 0.5%，够细也够快。
     * 只有 A 可调，B 固定 1.0 作为参考基准——
     * 两个都调会让人搞不清是哪个在起作用。
     *
     * 调整方向：车往【右】偏 -> A 实际更快 -> ENCA 调【大】。
     */
    { "ENCA",    0.005f,  0.90f,  1.10f },
    /*
     * LINE: 不是可调参数，而是一个【传感器调试页】。
     * step 设 0，所以 KEY1/KEY2 在这一页不起作用（不会误改数值）。
     * 页面内容在 DrawSensorPage() 里单独绘制。
     */
    { "LINE",    0.000f,  0.00f,  0.00f },
};

/* ------------------------------------------------------------------------- */
/* 任务内部状态                                                                */
/* ------------------------------------------------------------------------- */

static TaskHandle_t s_task_handle = NULL;

/* 三个按键各一个计数信号量 */
static SemaphoreHandle_t s_sem_key1 = NULL;
static SemaphoreHandle_t s_sem_key2 = NULL;
static SemaphoreHandle_t s_sem_key3 = NULL;

/* 当前选中的参数编号 */
static volatile uint8_t s_param_index = TUNE_IDX_KP;

/* 累计接受过的按键次数（诊断用，确认没丢包） */
static volatile uint32_t s_key1_total = 0U;
static volatile uint32_t s_key2_total = 0U;
static volatile uint32_t s_key3_total = 0U;

/* ------------------------------------------------------------------------- */
/* 内部辅助                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * 把信号量里的待处理事件全部取走，返回取到的次数。
 * 超时传 0，表示"有多少拿多少，不等待"。
 */
static uint32_t OledTask_DrainSemaphore(SemaphoreHandle_t sem)
{
    uint32_t count = 0U;

    if (sem == NULL)
    {
        return 0U;
    }

    while (xSemaphoreTake(sem, 0) == pdTRUE)
    {
        count++;

        /* 防御：正常情况下信号量计数上限是 16，不会无限循环 */
        if (count > OLED_KEY_SEM_MAX_COUNT)
        {
            break;
        }
    }

    return count;
}

/* 读当前选中参数的值 */
static float OledTask_ReadParam(uint8_t idx)
{
    if (idx >= TUNE_PARAM_COUNT)
    {
        return 0.0f;
    }

    if (idx == TUNE_IDX_TARGET)
    {
        return g_speed_test_target;
    }

    if (idx == TUNE_IDX_ENCA)
    {
        return Encoder_GetFix(ENCODER_A);
    }

    /* 左右轮参数始终同步，读 A 的就行 */
    return Speed_PID_GetGain(MOTOR_A, idx);
}

/* 写当前选中参数的值（自动限幅） */
static void OledTask_WriteParam(uint8_t idx, float value)
{
    if (idx >= TUNE_PARAM_COUNT)
    {
        return;
    }

    /* 限幅到该参数允许的范围 */
    if (value < s_params[idx].min)
    {
        value = s_params[idx].min;
    }
    else if (value > s_params[idx].max)
    {
        value = s_params[idx].max;
    }

    if (idx == TUNE_IDX_TARGET)
    {
        /*
         * 目标速度兼作"油门 + 启停开关"。
         *
         *   value > 0  ->  切到 SPEED_TEST 模式，车按斜坡加速到该速度
         *   value == 0 ->  切到 STOP 模式，硬制动停车
         *
         * 这样设计的原因：
         *   急停之后如果只清模式不清目标，用户没有任何途径重新启动
         *   （除非断电重启）。把目标值本身当作开关，语义最直观：
         *   "油门为零就是停车，给油就跑"。
         *
         * 安全前提：急停函数会把目标清零，所以停车后目标一定是 0，
         *          不会出现"按一下 KEY2 车就窜出去"的情况——
         *          需要连续按才能把速度加上来。
         */
        g_speed_test_target = value;

        if (value > 0.01f)
        {
            g_ctrl_mode = CTRL_MODE_SPEED_TEST;
        }
        else
        {
            g_ctrl_mode = CTRL_MODE_STOP;
        }
        return;
    }

    if (idx == TUNE_IDX_ENCA)
    {
        /* 分轮标定：只改 A，B 保持 1.0 作为基准 */
        Encoder_SetFix(ENCODER_A, value);
        return;
    }

    /*
     * 左右轮必须同步改，否则两轮增益不同，车会跑偏。
     * SetGain 内部会 Reset 积分和微分滤波状态。
     */
    Speed_PID_SetGain(MOTOR_A, idx, value);
    Speed_PID_SetGain(MOTOR_B, idx, value);
}

/* ------------------------------------------------------------------------- */
/* 按键上报接口（由控制任务调用）                                              */
/* ------------------------------------------------------------------------- */

uint8_t OledTask_NotifyKeyPress(uint32_t key)
{
    BaseType_t ret = pdFAIL;

    if (key == KEY_1)
    {
        if (s_sem_key1 != NULL)
        {
            ret = xSemaphoreGive(s_sem_key1);
            if (ret == pdTRUE)
            {
                s_key1_total++;
            }
        }
    }
    else if (key == KEY_2)
    {
        if (s_sem_key2 != NULL)
        {
            ret = xSemaphoreGive(s_sem_key2);
            if (ret == pdTRUE)
            {
                s_key2_total++;
            }
        }
    }
    else if (key == KEY_3)
    {
        if (s_sem_key3 != NULL)
        {
            ret = xSemaphoreGive(s_sem_key3);
            if (ret == pdTRUE)
            {
                s_key3_total++;
            }
        }
    }
    else
    {
        return 0U;      /* 不是我们关心的键 */
    }

    /*
     * 返回 0 表示信号量已满（计数到达 OLED_KEY_SEM_MAX_COUNT）。
     * 这不是故障——说明两次显示刷新之间按了超过 16 次，已超出人类极限，
     * 但保留返回值便于调试时发现异常。
     */
    return (ret == pdTRUE) ? 1U : 0U;
}

uint8_t OledTask_GetParamIndex(void)
{
    return s_param_index;
}

float OledTask_GetParamValue(void)
{
    return OledTask_ReadParam(s_param_index);
}

/* ------------------------------------------------------------------------- */
/* OLED 绘制                                                                   */
/* ------------------------------------------------------------------------- */

/*
 * 把浮点数格式化成定点字符串，【不依赖 printf 的 %f】。
 *
 * 为什么必须自己格式化：
 *   本工程链接时用了 --specs=nano.specs（newlib-nano），
 *   它默认不包含浮点格式化代码。在没有 -u _printf_float 的情况下，
 *   snprintf("%.1f", 1.1f) 输出的是【空字符串】。
 *
 *   表现就是：屏幕上 "KP" 后面是空白，按键改了内部数值（串口能看到
 *   rb 在变）但屏幕毫无反应——非常难查。
 *
 *   用整数运算自己拼小数，既省 Flash，也不受链接配置影响。
 *   代价是调用方要多给一个临时缓冲。
 *
 * 返回 buf，方便直接当 snprintf 的 %s 参数用。
 */
static const char *OledTask_FmtFloat(char *buf, uint16_t size,
                                     float value, uint8_t decimals)
{
    int32_t scale;
    int32_t scaled;
    int32_t int_part;
    int32_t frac_part;

    if ((buf == NULL) || (size == 0U))
    {
        return buf;
    }

    /* 按小数位数放大后四舍五入 */
    scale  = (decimals == 3U) ? 1000 : (decimals == 2U) ? 100 : 10;
    scaled = (int32_t)(value * (float)scale
                       + ((value >= 0.0f) ? 0.5f : -0.5f));

    int_part  = scaled / scale;
    frac_part = scaled % scale;
    if (frac_part < 0)
    {
        frac_part = -frac_part;
    }

    if (decimals == 3U)
    {
        (void)snprintf(buf, size, "%ld.%03ld",
                       (long)int_part, (long)frac_part);
    }
    else if (decimals == 2U)
    {
        (void)snprintf(buf, size, "%ld.%02ld",
                       (long)int_part, (long)frac_part);
    }
    else
    {
        (void)snprintf(buf, size, "%ld.%ld",
                       (long)int_part, (long)frac_part);
    }

    return buf;
}

/*
 * 传感器调试页。
 *
 * 调外环之前必须先确认三件事，这一页就是为它们准备的：
 *
 *   1. 8 路状态对不对
 *        把一个物体（或黑线）依次放到每个探头下面，
 *        对应位置的那个字符应该从 '.' 变成 '#'。
 *        如果错位，说明通道顺序或位置映射有问题。
 *
 *   2. 位置的正负号对不对（pos 那一行）
 *        把黑线放到传感器【左】半边 -> pos 应该变正（左正右负）
 *        放到【右】半边 -> pos 应该变负
 *        反了就把 GRAY_DIR_SIGN 改成 -1.0f。
 *
 *   3. 通道间距（GRAY_CHANNEL_PITCH_MM）
 *        程序里的 pos 是按 10mm 间距算出来的。
 *        用卡尺量出相邻探头中心的实际间距，填回 Gray.h。
 *        这个值主要影响转向增益的标定，稍有偏差可以用 KP 补偿。
 */
static void OledTask_DrawSensorPage(void)
{
    char     line[28];
    uint8_t  i;
    char     bits[GRAY_CHANNEL_COUNT + 1U];

    OLED_Clear();

    /* ---- 第 1 行：标题 ---- */
    OLED_ShowString(0U, 0U, "LINE SENSOR", OLED_FONT_8X16, OLED_WHITE);

    /* ---- 第 2 行：通道编号参考 ---- */
    OLED_ShowString(0U, 18U, "12345678", OLED_FONT_6X8, OLED_WHITE);

    /* ---- 第 3 行：8 路数字状态，'#' = 看到线 ---- */
    for (i = 0U; i < GRAY_CHANNEL_COUNT; i++)
    {
        bits[i] = (g_gray.channel[i] != 0U) ? '#' : '.';
    }
    bits[GRAY_CHANNEL_COUNT] = '\0';
    OLED_ShowString(0U, 26U, bits, OLED_FONT_6X8, OLED_WHITE);

    /* ---- 第 4 行：位置（mm，左正右负）---- */
    {
        char vbuf[14];

        (void)snprintf(line, sizeof(line), "pos %s mm",
                       OledTask_FmtFloat(vbuf, sizeof(vbuf),
                                         Gray_GetPositionMm(&g_gray), 1U));
        OLED_ShowString(0U, 36U, line, OLED_FONT_6X8, OLED_WHITE);
    }

    /* ---- 第 5 行：信号强度 + 丢线标志 ---- */
    (void)snprintf(line, sizeof(line), "str %ld L:%u",
                   (long)g_gray.strength,
                   (unsigned int)g_gray.line_lost);
    OLED_ShowString(0U, 46U, line, OLED_FONT_6X8, OLED_WHITE);

    /* ---- 第 6 行：提示 ---- */
    OLED_ShowString(0U, 56U, "K3:next page", OLED_FONT_6X8, OLED_WHITE);

    (void)OLED_Refresh();
}

static void OledTask_Draw(void)
{
    char    line[40];
    float   value;

    /* 传感器调试页走单独的绘制路径 */
    if (s_param_index == TUNE_IDX_SENSOR)
    {
        OledTask_DrawSensorPage();
        return;
    }

    value = OledTask_ReadParam(s_param_index);

    OLED_Clear();

    /* ---- 第 1 行：标题（8x16） ---- */
    OLED_ShowString(0U, 0U, "PID TUNE", OLED_FONT_8X16, OLED_WHITE);

    /*
     * ---- 第 2 行：当前参数名 + 值（8x16 大字，最醒目） ----
     *
     * KD 用两位小数显示，其余用一位——因为 KD 的步长是 0.01，
     * 只显示一位的话按一下看不出变化。
     */
    {
        char vbuf[16];

        (void)snprintf(line, sizeof(line), "%s %s",
                       s_params[s_param_index].name,
                       OledTask_FmtFloat(vbuf, sizeof(vbuf), value,
                                         (s_param_index == TUNE_IDX_ENCA) ? 3U :
                                         (s_param_index == TUNE_IDX_KD)   ? 2U : 1U));
    }
    OLED_ShowString(0U, 18U, line, OLED_FONT_8X16, OLED_WHITE);

    /*
     * ---- 第 3 行：目标速度 + 两轮实测速度 ----
     *
     * 两轮分开显示是排查"一轮快一轮慢"的关键：
     *   - 快的那轮实测值远大于目标 -> 编码器反馈异常（见第 4 行输出）
     *   - 两轮实测相近但转速不同     -> 编码器换算系数或机械问题
     */
    {
        char vb1[10];
        char vb2[10];
        char vb3[10];

        (void)snprintf(line, sizeof(line), "T%.9s A%.9s B%.9s",
                       OledTask_FmtFloat(vb1, sizeof(vb1), g_speed_test_target, 1U),
                       OledTask_FmtFloat(vb2, sizeof(vb2), Speed_PID_GetMeasure(MOTOR_A), 1U),
                       OledTask_FmtFloat(vb3, sizeof(vb3), Speed_PID_GetMeasure(MOTOR_B), 1U));
    }
    OLED_ShowString(0U, 36U, line, OLED_FONT_6X8, OLED_WHITE);

    /*
     * ---- 第 4 行：PID 输出（占空比 -100~100）----
     *
     * 判断方法：
     *   某一路输出恒为 +100 或 -100（打满）：
     *       说明该路的闭环失效——PID 拼命加速但测速跟不上目标。
     *       最常见原因是编码器没计数 / 方向反了 / 接线松。
     *   两路输出都在合理范围（比如 20~40）：
     *       说明闭环正常，转速差异来自机械或编码器标定。
     */
    {
        const char *mode_str;

        switch (g_ctrl_mode)
        {
        case CTRL_MODE_SPEED_TEST:  mode_str = "SPD";  break;
        case CTRL_MODE_LINE_FOLLOW: mode_str = "LINE"; break;
        case CTRL_MODE_STOP:
        default:                    mode_str = "STOP"; break;
        }

        (void)snprintf(line, sizeof(line), "%s oA%3d oB%3d",
                       mode_str,
                       (int)Speed_PID_GetOutput(MOTOR_A),
                       (int)Speed_PID_GetOutput(MOTOR_B));
        OLED_ShowString(0U, 46U, line, OLED_FONT_6X8, OLED_WHITE);
    }

    /* ---- 第 5 行：操作提示 ---- */
    OLED_ShowString(0U, 55U, "K1- K2+ K3:next", OLED_FONT_6X8, OLED_WHITE);

    (void)OLED_Refresh();
}

/* ------------------------------------------------------------------------- */
/* 任务主体                                                                    */
/* ------------------------------------------------------------------------- */

static void OledTask_Main(void *argument)
{
    uint32_t k1n;
    uint32_t k2n;
    uint32_t k3n;
    uint8_t  old_index;
    float    old_value;

    (void)argument;

    /* 等 OLED 和按键初始化稳定 */
    vTaskDelay(pdMS_TO_TICKS(200U));

    /*
     * 现在调度器已经跑起来了，可以安全启用异步传输（I2C 中断 / DMA）。
     *
     * 之前 OLED_Init() 在调度器启动前调用，那时只能用轮询
     * （阻塞式信号量在调度器启动前会卡死整个系统）。
     */
    OLED_EnableAsync();

    /*
     * 启动横幅：确认调参任务真的在跑。
     * 如果连这一行都看不到，说明 OledTask 根本没启动或串口不通。
     */
    {
        const char *banner = "[tune] OledTask ready, 3 keys armed\r\n";
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)banner,
                                (uint16_t)strlen(banner), 50U);
    }

    /* 首帧 */
    OledTask_Draw();

    for (;;)
    {
        old_index = s_param_index;
        old_value = OledTask_ReadParam(s_param_index);

        /* ---- 1. 把三个信号量里的待处理按键全部取走 ---- */
        k1n = OledTask_DrainSemaphore(s_sem_key1);
        k2n = OledTask_DrainSemaphore(s_sem_key2);
        k3n = OledTask_DrainSemaphore(s_sem_key3);

        /* ---- 2. KEY3：切换参数（每按一下前进一档，循环） ---- */
        if (k3n != 0U)
        {
            s_param_index = (uint8_t)((s_param_index + k3n) % TUNE_PARAM_COUNT);

            /*
             * 换了参数必须重新取基准值！
             *
             * old_value 是在切换之前读的，属于【旧参数】的值。
             * 如果同一轮里又按了 KEY1/KEY2，就会拿旧参数的值
             * 去加减新参数的步长，写进新参数——数值会跳到离谱的地方。
             */
            old_value = OledTask_ReadParam(s_param_index);
        }

        /* ---- 2.5 【临时诊断】任何按键到达都打印 ---- */
        if ((k1n != 0U) || (k2n != 0U) || (k3n != 0U))
        {
            char dbg[96];

            (void)snprintf(dbg, sizeof(dbg),
                           "[tune] got k1=%lu k2=%lu k3=%lu idx=%u val=%ld\r\n",
                           (unsigned long)k1n,
                           (unsigned long)k2n,
                           (unsigned long)k3n,
                           (unsigned int)s_param_index,
                           (long)(OledTask_ReadParam(s_param_index) * 100.0f));
            (void)HAL_UART_Transmit(&huart2, (uint8_t *)dbg,
                                    (uint16_t)strlen(dbg), 50U);
        }

        /* ---- 3. KEY1/KEY2：按步长增减当前参数 ---- */
        if ((k1n != 0U) || (k2n != 0U))
        {
            float step = s_params[s_param_index].step;
            float delta = (float)k2n * step - (float)k1n * step;
            char  dbg2[112];

            OledTask_WriteParam(s_param_index, old_value + delta);

            /*
             * 写入后立刻读回，验证 SetGain 是否真的生效。
             */
            (void)snprintf(dbg2, sizeof(dbg2),
                           "[tune] w %s step=%ld d=%ld old=%ld new=%ld rb=%ld\r\n",
                           s_params[s_param_index].name,
                           (long)(step * 1000.0f),
                           (long)(delta * 1000.0f),
                           (long)(old_value * 100.0f),
                           (long)((old_value + delta) * 100.0f),
                           (long)(OledTask_ReadParam(s_param_index) * 100.0f));
            (void)HAL_UART_Transmit(&huart2, (uint8_t *)dbg2,
                                    (uint16_t)strlen(dbg2), 50U);
        }

        /*
         * ---- 4. 只在参数或数值变化时重绘 ----
         *
         * 但 TARGET 和实测速度每拍都可能变（车在跑时实测速度一直在变），
         * 所以运行在 SPEED_TEST/LINE 模式下需要每拍刷新，保证能看到实时速度。
         * 停车状态下才做"变化才刷"的优化，减少 I2C 流量。
         */
        /*
         * 传感器调试页必须【每拍都刷新】——它显示的是 8 路实时状态，
         * 如果沿用"数值变化才刷"的策略，停车状态下画面会冻住，
         * 根本看不到探头被遮挡时的变化。
         */
        if ((s_param_index == TUNE_IDX_SENSOR) ||
            (old_index != s_param_index) ||
            (old_value != OledTask_ReadParam(s_param_index)) ||
            (g_ctrl_mode != CTRL_MODE_STOP))
        {
            OledTask_Draw();
        }

        vTaskDelay(pdMS_TO_TICKS(OLED_TASK_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------------- */
/* 任务创建                                                                    */
/* ------------------------------------------------------------------------- */

uint8_t OledTask_Init(void)
{
    BaseType_t ret;

    if (s_task_handle != NULL)
    {
        return 1U;      /* 已创建 */
    }

    /*
     * 创建三个计数信号量。
     * 初始计数 0，最大计数 OLED_KEY_SEM_MAX_COUNT。
     * 必须在创建任务之前创建信号量，否则任务第一次运行时会拿到 NULL。
     */
    s_sem_key1 = xSemaphoreCreateCounting(OLED_KEY_SEM_MAX_COUNT, 0U);
    s_sem_key2 = xSemaphoreCreateCounting(OLED_KEY_SEM_MAX_COUNT, 0U);
    s_sem_key3 = xSemaphoreCreateCounting(OLED_KEY_SEM_MAX_COUNT, 0U);

    if ((s_sem_key1 == NULL) || (s_sem_key2 == NULL) || (s_sem_key3 == NULL))
    {
        /* 堆不够，创建失败，把已创建的回收掉，避免内存泄漏 */
        if (s_sem_key1 != NULL) { vSemaphoreDelete(s_sem_key1); s_sem_key1 = NULL; }
        if (s_sem_key2 != NULL) { vSemaphoreDelete(s_sem_key2); s_sem_key2 = NULL; }
        if (s_sem_key3 != NULL) { vSemaphoreDelete(s_sem_key3); s_sem_key3 = NULL; }
        return 0U;
    }

    ret = xTaskCreate(OledTask_Main,
                      "OledTask",
                      OLED_TASK_STACK_WORDS,
                      NULL,
                      OLED_TASK_PRIORITY,
                      &s_task_handle);

    if (ret != pdPASS)
    {
        vSemaphoreDelete(s_sem_key1);
        vSemaphoreDelete(s_sem_key2);
        vSemaphoreDelete(s_sem_key3);
        s_sem_key1 = NULL;
        s_sem_key2 = NULL;
        s_sem_key3 = NULL;
        s_task_handle = NULL;
        return 0U;
    }

    return 1U;
}

//
// Created by JiangyaF on 2026/10/22.
//
