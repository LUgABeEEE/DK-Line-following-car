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
#include "LinePD.h"
#include "Control_task.h"
#include "Ble_task.h"       /* BleTask_GetTagCount()：显示已打标次数 */
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

/* 外环实例定义在 main.c 中，MODE 页要显示它的基础速度 */
extern Line_PD_t g_line_pd;

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
#define TUNE_IDX_MODE       7U
#define TUNE_IDX_LKP        8U
#define TUNE_IDX_LKD        9U
#define TUNE_IDX_LBAS       10U
#define TUNE_IDX_LAP        11U     /* 比赛自动停车：一圈周长 cm */
#define TUNE_PARAM_COUNT    12U

static const TuneParam_t s_params[TUNE_PARAM_COUNT] =
{
    /* 名称       步长     下限    上限   */
    { "KP",      0.10f,   0.00f,  5.00f },
    { "KI",      0.10f,   0.00f,  5.00f },
    { "KD",      0.01f,   0.00f,  1.00f },   /* KD 量级小，用 0.01 */
    { "KFF",     0.05f,   0.00f,  2.00f },
    { "TARGET",  1.00f,   0.00f, 40.00f },   /* 单位 cm/s */
    /*
     * ENCA: A 轮编码器的分轮修正系数（改的是【测量值】，不是电机）。
     *
     * 原理：这个系数直接乘在测速结果上
     *       raw_cms = 计数 × 每计数距离 × ENCA / 时间
     *
     *       ENCA 调大 -> 测得速度变大 -> 速度环以为跑快了 -> 压低占空比
     *                -> A 轮【实际变慢】-> 车往【左】偏
     *       ENCA 调小 -> A 轮【实际变快】-> 车往【右】偏
     *
     * 所以调整方向和直觉相反：
     *       车往【左】偏  ->  A 太慢  ->  ENCA 【减小】（KEY1）
     *       车往【右】偏  ->  A 太快  ->  ENCA 【加大】（KEY2）
     *
     * 实测记录：曾把 ENCA 加大想纠正左偏，结果左偏更严重，
     * 印证了"ENCA 大 = A 轮慢"这个方向。
     *
     * 判断两轮是否平衡的客观依据（SPEED_TEST 直道）：
     *   直道上 d_A 和 d_B 的增量应该相等，差多少就说明两轮差多少。
     */
    { "ENCA",    0.005f,  0.70f,  1.30f },
    /*
     * LINE: 不是可调参数，而是一个【传感器调试页】。
     * step 设 0，所以 KEY1/KEY2 在这一页不起作用（不会误改数值）。
     * 页面内容在 DrawSensorPage() 里单独绘制。
     */
    { "LINE",    0.000f,  0.00f,  0.00f },
    /*
     * MODE: 运行模式切换页。不是数值参数，KEY1/KEY2 用来循环切换
     * STOP -> SPD -> LINE。页面在 DrawModePage() 里单独绘制。
     */
    { "MODE",    1.000f,  0.00f,  2.00f },

    /*
     * ---- 外环（循迹 PD）参数 ----
     *
     * LKP : 比例增益，单位 cm/s 差速量 每 cm 误差
     *       步长 0.5 —— 实测范围大约 6~15，0.5 已经是 3%~8%，够细
     *       范围上限 30 是安全值，再大一定画龙
     *
     * LKD : 微分增益，抑制过弯回摆
     *       步长 0.1 —— 典型值 0~6，0.1 足够
     *
     * LBAS: 循迹基础速度，单位 cm/s
     *       步长 1.0 —— 1 cm/s 一档最直观
     */
    { "LKP",     0.50f,   0.00f, 30.00f },
    { "LKD",     0.10f,   0.00f, 20.00f },
    { "LBAS",    1.00f,   0.00f, 50.00f },
    /*
     * LAP: 比赛自动停车用的一圈周长，单位 cm（步长 10cm）。
     *   0      = 关闭自动停车
     *   10~40  = 自适应模式：只靠启停线识别，不需要知道周长
     *            （从 0 按一下就是 10，正好进自适应）
     *   >=50   = 周长模式：识别为主，里程兜底
     * 之所以用 10cm 步长：周长是几百上千厘米，1cm 步长要按几百次。
     */
    { "LAP",    10.00f,   0.00f, 3000.00f },
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

/* 1 = 下一轮强制重绘（跨任务请求切页时用） */
static volatile uint8_t s_force_redraw = 0U;

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

    if (idx == TUNE_IDX_MODE)
    {
        return (float)g_ctrl_mode;
    }

    if (idx == TUNE_IDX_LAP)
    {
        return g_race_lap_cm;
    }

    /* ---- 外环参数直接读写 g_line_pd ---- */
    if (idx == TUNE_IDX_LKP)
    {
        return g_line_pd.kp;
    }
    if (idx == TUNE_IDX_LKD)
    {
        return g_line_pd.kd;
    }
    if (idx == TUNE_IDX_LBAS)
    {
        return g_line_pd.base_speed;
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

    /* ---- 外环（循迹 PD）参数：直接写 g_line_pd ---- */
    if (idx == TUNE_IDX_LKP)
    {
        g_line_pd.kp = value;
        return;
    }

    if (idx == TUNE_IDX_LKD)
    {
        g_line_pd.kd = value;
        return;
    }

    if (idx == TUNE_IDX_LBAS)
    {
        /*
         * 基础速度（只改循迹基础速度）。
         *
         * 以前这里同时写 g_speed_test_target，结果"调个 LBAS"就顺手把
         * 速度测试模式的目标也改了——如果当时正好在 SPD 模式，车会自己
         * 跑起来。现在两者彻底解耦：SPD 的速度只由 TARGET 参数控制。
         */
        g_line_pd.base_speed = value;
        return;
    }

    if (idx == TUNE_IDX_LAP)
    {
        /*
         * 自动停车的一圈周长（cm）。0 = 关闭。
         * 只在停车状态下改，运行中改不会生效（RaceStart 只在进入 LINE 时读一次）。
         */
        g_race_lap_cm = value;
        return;
    }

    if (idx == TUNE_IDX_MODE)
    {
        /*
         * 模式切换。value 已被限幅到 [0, 2]，正好对应
         * CTRL_MODE_STOP / SPEED_TEST / LINE_FOLLOW。
         *
         * 切到 STOP 时要把目标速度也清零，否则下次进 SPEED_TEST
         * 会因为残留目标突然窜出去。
         */
        uint8_t new_mode = (uint8_t)value;

        if (new_mode == CTRL_MODE_STOP)
        {
            ControlTask_RequestStop();
        }
        else
        {
            g_ctrl_mode = new_mode;
        }
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

uint8_t OledTask_IsModePage(void)
{
    return (s_param_index == TUNE_IDX_MODE) ? 1U : 0U;
}

void OledTask_SetModePage(void)
{
    s_param_index  = TUNE_IDX_MODE;
    s_force_redraw = 1U;
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
    /* 强度放大 100 倍：0.76 显示成 76，否则整数截断后永远是 0 */
    (void)snprintf(line, sizeof(line), "str %ld L:%u",
                   (long)(g_gray.strength * 100.0f),
                   (unsigned int)g_gray.line_lost);
    OLED_ShowString(0U, 46U, line, OLED_FONT_6X8, OLED_WHITE);

    /*
     * ---- 第 6 行：I2C 健康度 + 翻页提示 ----
     *
     * e<失败次数>/<总读取次数>
     *   这是判断"电源反接有没有留下后遗症"的关键指标：
     *   跑一段时间后如果 e 一直是 0，说明模块和走线都健康；
     *   如果 e 在涨，说明有接触不良或器件受损。
     */
    (void)snprintf(line, sizeof(line), "e%lu/%lu K3:next",
                   (unsigned long)g_gray.total_fails,
                   (unsigned long)g_gray.total_reads);
    OLED_ShowString(0U, 56U, line, OLED_FONT_6X8, OLED_WHITE);

    (void)OLED_Refresh();
}

/*
 * 运行模式切换页。
 *
 * KEY1 / KEY2 循环切换三种模式：
 *   STOP  停车（硬制动）
 *   SPD   只跑内环，两轮追踪 g_speed_test_target
 *   LINE  双环循迹（外环读灰度算差速 + 内环速度闭环）
 *
 * ⚠️ 切到 LINE 车会立刻按基础速度跑起来，确保车已经在赛道上方。
 */
static void OledTask_DrawModePage(void)
{
    char        line[28];
    char        hint_buf[28];
    const char *mode_str;
    const char *hint;

    switch (g_ctrl_mode)
    {
    case CTRL_MODE_SPEED_TEST:
        mode_str = "SPEED TEST";
        hint     = "inner loop only";
        break;

    case CTRL_MODE_LINE_FOLLOW:
        mode_str = "LINE FOLLOW";
        /*
         * 循迹运行中模式锁定（见 OledTask_Main 的 KEY1/KEY2 分支）：
         *   KEY1 短按 = 本次跑失败   -> 串口 [T] FAIL
         *   KEY3 长按 = 本次跑成功   -> 串口 [T] OK
         *   KEY2 长按 = 停车（Control_task 处理）
         */
        {
            uint16_t tag_n = BleTask_GetTagCount();

            if (tag_n > 0U)
            {
                (void)snprintf(hint_buf, sizeof(hint_buf),
                               "K1 FAIL K3H OK #%u", (unsigned int)tag_n);
                hint = hint_buf;
            }
            else
            {
                hint = "K1 FAIL  K3H OK/MODE";
            }
        }
        break;

    case CTRL_MODE_STOP:
    default:
        mode_str = "STOP";
        /*
         * 停车状态下也能补打标记（自动停车/急停之后）：
         * 短按 KEY1 = 失败，长按 KEY3 = 成功。
         */
        hint = "K2:LINE K1H:SPD";
        break;
    }

    OLED_Clear();

    OLED_ShowString(0U, 0U, "RUN MODE", OLED_FONT_8X16, OLED_WHITE);
    OLED_ShowString(0U, 18U, mode_str, OLED_FONT_8X16, OLED_WHITE);
    OLED_ShowString(0U, 36U, hint, OLED_FONT_6X8, OLED_WHITE);

    /*
     * 实时显示外环的中间量和输出。
     *
     * 这一页是排查"循迹方向/增益是否真的生效"的关键：
     *   L / R  : 外环算出的左右轮目标速度（cm/s）
     *            LKP=LKD=0 时这两个数必须【完全相等】，
     *            如果不等，说明增益没写进去或者还有别的转向来源。
     *   e      : 当前误差（cm），左正右负
     *   s      : 当前差速量 steer（cm/s）
     *
     * 通过 L 和 R 的大小关系，可以直接判断转向方向对不对，
     * 不用让车跑起来。
     */
    {
        char b1[10];
        char b2[10];

        (void)snprintf(line, sizeof(line), "L%s R%s",
                       OledTask_FmtFloat(b1, sizeof(b1), g_line_pd.left_target, 1U),
                       OledTask_FmtFloat(b2, sizeof(b2), g_line_pd.right_target, 1U));
        OLED_ShowString(0U, 46U, line, OLED_FONT_6X8, OLED_WHITE);

        (void)snprintf(line, sizeof(line), "e%s s%s",
                       OledTask_FmtFloat(b1, sizeof(b1), g_line_pd.error, 1U),
                       OledTask_FmtFloat(b2, sizeof(b2), g_line_pd.steer, 1U));
        OLED_ShowString(0U, 56U, line, OLED_FONT_6X8, OLED_WHITE);
    }

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

    /* 模式切换页同样单独绘制 */
    if (s_param_index == TUNE_IDX_MODE)
    {
        OledTask_DrawModePage();
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

            /*
             * 注：KEY3 长按兼作"本次跑成功"的打标键。
             * 这里不做锁页 —— 按下时按正常逻辑切页，
             * 等 LONG_PRESS 事件到了，控制任务会打标并把页面
             * 退回到 MODE 页（OledTask_SetModePage）。
             */
        }

        /* ---- 3. KEY1/KEY2：按步长增减当前参数 ---- */
        if ((k1n != 0U) || (k2n != 0U))
        {
            if ((s_param_index == TUNE_IDX_MODE) &&
                (g_ctrl_mode == CTRL_MODE_LINE_FOLLOW))
            {
                /*
                 * 循迹运行中：模式锁定。
                 *
                 * 以前在 MODE 页按 KEY1/KEY2 会直接把模式切走，
                 * 车还在跑的时候一不小心就切到 SPD/STOP，很危险。
                 * 现在这两个键改为"跑圈结果打标"：
                 *   KEY1 = 失败、KEY2 = 成功，由控制任务在【松手】时
                 *   判定为短按后调用 BleTask_SendTag()。
                 * 要停车请长按 KEY2（走 ControlTask_RequestStop）。
                 */
            }
            else
            {
                float step   = s_params[s_param_index].step;
                float delta  = (float)k2n * step - (float)k1n * step;
                float target = old_value + delta;

                /*
                 * 模式页：STOP 状态下按 KEY2 直接进 LINE（跳过 SPD）。
                 *
                 * 以前 STOP -> SPD -> LINE 要按两下，误按一下车就在
                 * SPEED_TEST 里直着冲出去（实测发生过三次"直角冲出道"）。
                 * SPD 改为长按 KEY1 进入（Control_task 处理）。
                 */
                if ((s_param_index == TUNE_IDX_MODE) &&
                    (g_ctrl_mode == CTRL_MODE_STOP) &&
                    (k2n > 0U))
                {
                    target = (float)CTRL_MODE_LINE_FOLLOW;
                }

                OledTask_WriteParam(s_param_index, target);
            }
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
        if ((s_force_redraw != 0U) ||
            (s_param_index == TUNE_IDX_SENSOR) ||
            (s_param_index == TUNE_IDX_MODE) ||
            (old_index != s_param_index) ||
            (old_value != OledTask_ReadParam(s_param_index)) ||
            (g_ctrl_mode != CTRL_MODE_STOP))
        {
            OledTask_Draw();
            s_force_redraw = 0U;
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


uint32_t OledTask_GetStackFreeWords(void)
{
    if (s_task_handle == NULL)
    {
        return 0U;
    }
    return (uint32_t)uxTaskGetStackHighWaterMark(s_task_handle);
}

//
// Created by JiangyaF on 2026/10/22.
//
