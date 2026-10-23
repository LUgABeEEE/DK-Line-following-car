//
// Created by JiangyaF on 2026/10/11.
//
// 外环：循迹 PD 控制器（串级控制的外层）
//
// 控制结构：
//
//   灰度模块 ──error(cm)──> [本模块 PD] ──差速量──> 左右轮目标速度(cm/s)
//                                                        │
//                                                        v
//                                              [内环 SpeedPID] ──占空比──> 电机
//
// 控制律：
//
//   目标速度 = base_speed * 弯道减速系数 - Kp*error - Kd*d(error)/dt
//   左轮 = 目标速度 + 差速量
//   右轮 = 目标速度 - 差速量
//
// 前馈项 base_speed 是设计关键：它提供"直道上该跑多快"，PD 只负责修正偏差。
// 若省掉它让 PD 直接从零凑输出，直道上输出必须靠积分维持，必然积分饱和、
// 一转弯就失控。本模块因此刻意不含积分项 —— 线本身就是绝对位置基准，
// 不存在需要积分去消除的稳态误差。
//
// 符号约定（已与实测核对）：
//   误差为正 = 线偏左
//   差速量为正 = 右轮加速 / 左轮减速 = 车向左转
//   目标速度为正 = 前进
//
// 周期约定：
//   本模块周期必须是内环 SPEED_PID_PERIOD_MS 的 3~5 倍，否则两层会互相打架。
//   LinePD_Update() 内部做了检查，周期不满足时会拒绝运行并返回错误码。
//

#ifndef LINEFLLOWING_CAR_LINEPD_H
#define LINEFLLOWING_CAR_LINEPD_H

#include "main.h"
#include "Gray.h"
#include "SpeedPID.h"

/* ------------------------------------------------------------------------- */
/* LinePD_Update() 的返回值                                                    */
/* ------------------------------------------------------------------------- */

#define LINE_PD_OK              0U      /* 正常                                */
#define LINE_PD_ERR_I2C         1U      /* 灰度通信失败，本次未更新电机指令    */
#define LINE_PD_ERR_LOST        2U      /* 丢线超时，已按配置处理              */
#define LINE_PD_ERR_PERIOD      3U      /* 内外环周期比不合理，拒绝运行        */

/* ------------------------------------------------------------------------- */
/* 周期与限幅                                                                  */
/* ------------------------------------------------------------------------- */

/* 外环执行周期，单位 ms。应取 max(灰度模块更新率, 内环周期 x 3) */
#define LINE_PD_PERIOD_MS           100U

/*
 * 内外环周期比的下限。小于这个值说明外环跑太快，和内环抢执行机会，
 * 表现为整车高频振荡，而且分不清是哪个环的参数不对。
 */
#define LINE_MIN_PERIOD_RATIO       3U

/* 差速量限幅，单位 cm/s。防"内侧轮反转"导致原地打转 */
#define LINE_DIFF_LIMIT             25.0f

/* 目标速度限幅，单位 cm/s。内环本身也有限幅，这里提前夹一次更安全 */
#define LINE_SPEED_LIMIT            60.0f

/*
 * 弯道减速：误差达到该值时，目标速度降到基础速度的这个比例。
 * 目的是让车在急弯前主动放慢，而不是硬冲出去。
 * 设为 0.0f 表示不减速（全程保持基础速度）。
 */
#define LINE_CURVE_SLOW_MIN         0.55f

/* 编译期默认增益 */
#define LINE_PD_DFLT_KP             8.0f    /* cm/s 差速量 每 cm 误差      */
#define LINE_PD_DFLT_KD             3.0f    /* 阻尼，抑制画龙              */
#define LINE_PD_DFLT_BASE_SPEED     30.0f   /* 基础速度 cm/s               */

/*
 * 微分低通滤波系数 (0,1]。灰度解算出的位置在探头交界处有固有台阶，
 * 不滤波的话 D 项会把台阶当成分辨率极高的真实变化而放大。
 * 但系数太小会让 D 项严重滞后、失去阻尼作用（外环 100ms 周期下尤其明显），
 * 所以取 0.7 而不是更小。
 */
#define LINE_PD_DFLT_D_FILTER       0.7f

/*
 * 丢线容忍的连续周期数。超过之后按着下面两个开关处理。
 * 1 个周期 = LINE_PD_PERIOD_MS。
 */
#define LINE_LOST_TOLERANCE         3U

/*
 * 丢线超时后是否停车。
 *
 *   1 = 停车（**初次调试务必用这个**）
 *       丢线就停，车不会冲出赛道，你也能立刻发现"传感器没看到线"。
 *
 *   0 = 继续转向搜索
 *       按灰度模块给出的"最后偏离方向"继续满舵前进，尝试找回线。
 *       适合调试完成后的实跑，但线丢得远时车会一直转圈。
 */
#define LINE_STOP_IF_LINE_LOST      1

/* ------------------------------------------------------------------------- */
/* 控制器结构体                                                                */
/* ------------------------------------------------------------------------- */

typedef struct
{
    /* ---- 可调参数 ---- */
    float kp;               /* 比例增益，单位 cm/s 差速量 每 cm 误差          */
    float kd;               /* 微分增益，单位 cm/s 差速量 每 (cm/s) 误差变化率 */
    float base_speed;       /* 基础速度，cm/s                                 */
    float diff_limit;       /* 差速量限幅，cm/s                               */
    float speed_limit;      /* 目标速度限幅，cm/s                             */
    float d_filter;         /* 微分低通系数 (0,1]                             */

    /* ---- 运行状态：由 Init/Reset 维护 ---- */
    float   error;          /* 最近一次误差，cm（左正右负）                   */
    float   last_error;     /* 上一次误差，用于求微分                          */
    float   d_filter_state; /* 微分项滤波状态                                 */
    float   steer;          /* 最近一次差速量，cm/s                           */
    float   speed;          /* 最近一次的目标基础速度，cm/s                   */
    uint8_t lost_count;     /* 连续丢线周期数                                 */
    uint8_t line_lost;      /* 1 = 本次判定为丢线                             */
    uint8_t first_run;      /* 首拍标志，避免微分冲击                         */
} Line_PD_t;

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/* 用编译期默认参数初始化，并复位全部运行状态。 */
void LinePD_Init(Line_PD_t *pd);

/* 复位运行状态（清微分历史、丢线计数、输出），参数保持不变。 */
void LinePD_Reset(Line_PD_t *pd);

/* 运行时修改增益。会自动复位，避免旧状态污染新参数。 */
void LinePD_SetGains(Line_PD_t *pd, float kp, float kd, float base_speed);

/*
 * 核心接口：读一次灰度传感器，做 PD 运算，并把左右轮目标速度写进内环。
 *
 *   pd    : 控制器实例
 *   gray  : 灰度传感器实例
 *   hi2c  : 灰度传感器所在的 I2C 句柄（你的板子是 &hi2c1）
 *
 * 返回值：
 *   LINE_PD_OK         正常
 *   LINE_PD_ERR_I2C    灰度通信失败（本次未更新电机指令）
 *   LINE_PD_ERR_LOST   丢线超时，已按 LINE_STOP_IF_LINE_LOST 处理
 *   LINE_PD_ERR_PERIOD 内外环周期比不合理，拒绝运行（配置错误）
 *
 * 注意：本函数只负责"算"，不负责"等"。请由调用方按 LINE_PD_PERIOD_MS
 *       周期调用它，例如放在主循环的时间戳判断里。
 */
uint8_t LinePD_Update(Line_PD_t *pd, Gray_Sensor_t *gray, I2C_HandleTypeDef *hi2c);

/* 停车并复位（内环也会被停止）。 */
void LinePD_Stop(Line_PD_t *pd);

/* 查询接口，供调试和 OLED 显示使用。 */
float   LinePD_GetError(const Line_PD_t *pd);    /* 最近一次误差 cm        */
float   LinePD_GetSteer(const Line_PD_t *pd);    /* 最近一次差速量 cm/s    */
float   LinePD_GetSpeed(const Line_PD_t *pd);    /* 最近一次基础速度 cm/s  */
uint8_t LinePD_IsLineLost(const Line_PD_t *pd);

/*
 * 符号与转向方向验证（**上车跑圈之前必须先做这一步**）。
 *
 * 串级控制能否稳定，完全取决于符号是否自洽：
 *   电机正转 = 前进、编码器前进读数为正、线偏左误差为正
 * 三个环节只要有一个反了，闭环就变成正反馈，车会直接冲出去。
 * 本函数通过两段可观察的测试把这些符号钉死：
 *
 *   interactive = 0  自动模式：只验证误差符号，电机全程不动，安全
 *   interactive = 1  交互模式：额外做转向验证，电机会转动
 *
 * 自动模式的判定方法：手动把线依次放在中心、左侧、右侧，
 * 观察串口打印的 error 符号是否符合"左正右负"。
 *
 * 交互模式的转向验证分两步：
 *   第1步 左轮 0%、右轮 25% —— 车应该**原地左转**（右轮往前推，左轮不动）
 *   第2步 左轮 25%、右轮 0% —— 车应该**原地右转**
 * 若方向相反，说明左右电机的映射或接线反了，必须先修正再跑循迹。
 *
 * 需要轮子架空或留出足够空间，串口波特率 115200。
 */
void LinePD_VerifySigns(Line_PD_t *pd, Gray_Sensor_t *gray,
                        I2C_HandleTypeDef *hi2c, uint8_t interactive);

/*
 * 循迹自检：给定几段目标速度，让车实际跟着线跑，每 100ms 打印
 *   error / steer / 基础速度 / 左右轮目标速度 / 实测速度
 * 用于观察跟随效果和调 Kp、Kd。
 * 函数会持续运行 duration_sec 秒，然后自动停车。
 * 需要把车放在赛道上，串口波特率 115200。
 */
void LinePD_TestSequence(Line_PD_t *pd, Gray_Sensor_t *gray,
                         I2C_HandleTypeDef *hi2c, uint32_t duration_sec);

#endif //LINEFLLOWING_CAR_LINEPD_H
