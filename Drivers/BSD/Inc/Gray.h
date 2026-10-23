//
// Created by JiangyaF on 2026/10/11.
//
// 幻尔(Hiwonder) 8 路 RGB 灰度巡线传感器驱动
//
// 硬件与协议（依据官方手册 + 原理图）：
//   供电      : DC 5V / 85mA（注意别从 STM32 的 3.3V 引脚取电）
//   I2C 地址  : 0x5D（7 位，固定）
//   检测线宽  : 0.6cm ~ 6.9cm
//   检测高度  : 10mm ~ 40mm（标定与运行时的高度必须一致！）
//
// 寄存器映射：
//   5      全通道数字状态    uint8_t  1 字节，bit0~bit7 = 通道1~8，检测到线 = 1
//   6~21   8 路灰度模拟值    uint16_t 每路 2 字节，低字节在前（6,8,10,...,20）
//   22~37  8 路灰度阈值      uint16_t 每路 2 字节，低字节在前（22,24,...,36）
//
// 重要：模拟量在寄存器 6~21 是物理连续的，一次 I2C 事务就能读完全部 16 字节。
//       阈值同理（22~37 连续）。不要一路一路读，那样既慢又白占总线。
//
// 板载 MCU 已经替我们完成了：一键自学习校准、自动选择最优检测光色、阈值判决。
// 所以本驱动不需要做黑白标定，只需要把阈值读回来用于归一化即可。
//

#ifndef LINEFLLOWING_CAR_GRAY_H
#define LINEFLLOWING_CAR_GRAY_H

#include "main.h"

/* ------------------------------------------------------------------------- */
/* 协议常量                                                                    */
/* ------------------------------------------------------------------------- */

#define GRAY_I2C_ADDR_7BIT      0x5D    /* 7 位地址，HAL 调用时要左移一位        */
#define GRAY_REG_STATE          5U      /* 全通道数字状态，1 字节               */
#define GRAY_REG_ANALOG_CH1     6U      /* 通道 1 模拟值，16 字节连续           */
#define GRAY_REG_THRESHOLD_CH1  22U     /* 通道 1 阈值，16 字节连续             */

#define GRAY_CHANNEL_COUNT      8U      /* 通道数                               */

/*
 * 单次 I2C 事务的超时，单位 ms。
 * 绝对不要用 HAL_MAX_DELAY：模块无响应时会把控制循环卡死。
 * 正常一次读只需不到 1ms，给 5ms 已经很宽松。
 */
#define GRAY_I2C_TIMEOUT_MS     5U

/* 连续读失败多少次就判定传感器掉线，需要由上层做停车保护 */
#define GRAY_FAIL_LIMIT         10U

/* ------------------------------------------------------------------------- */
/* 机械标定                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * 相邻两个探头的间距，单位 mm。
 * 用尺子量 PCB 上两个相邻探头中心孔的距离填进来。
 * 这个值只影响误差的"物理量纲"（厘米），不影响 PD 能不能工作，
 * 但填准了 Kp 才有明确含义（"偏 1cm 修多少 cm/s"）。
 */
#define GRAY_CHANNEL_PITCH_MM   10.0f

/*
 * 通道顺序方向：
 *   +1.0f = 模块的通道 1 在车的左侧（通道 8 在右侧）
 *   -1.0f = 反过来（模块旋转 180° 安装）
 *
 * 判断方法：装好车后，把黑线放在传感器最左边那个探头下面，
 * 看串口打印的 position 是正还是负。如果是"负"而你希望"左偏为正"，
 * 就把这个值改成相反数。
 */
#define GRAY_DIR_SIGN           1.0f

/*
 * 阵列半跨度：最外侧探头到中心的距离，单位 mm。
 * 这同时就是位置解算能达到的最大幅值（线性区之外的饱和值），
 * 外环用它作为误差限幅，两个模块之间的公式只写这一处。
 */
#define GRAY_HALF_SPAN_MM       (GRAY_CHANNEL_PITCH_MM * (float)(GRAY_CHANNEL_COUNT - 1U) * 0.5f)

/*
 * 线条相对背景的明暗关系
 *
 *   白底黑线（最常见） -> 线条比背景暗 -> GRAY_LINE_IS_DARK
 *   黑底白线           -> 线条比背景亮 -> GRAY_LINE_IS_BRIGHT
 *
 * 手册里并没有说明模块的归一化约定，所以这里做成可配置的。
 * 判断方法：把传感器放在白底黑线的赛道上，看串口打印的 sum 值
 *   sum 明显 > 0 → 当前设置正确
 *   sum 接近 0   → 改成另一个值
 */
enum
{
    GRAY_LINE_IS_DARK = 0,      /* 白底黑线：默认 */
    GRAY_LINE_IS_BRIGHT = 1     /* 黑底白线 */
};

/*
 * 丢线时的处理策略：
 *   线丢了以后，误差保持不变已经没有意义，这里用一个"记忆值"朝最后
 *   偏离的方向继续打满，让车尽可能转回线上。
 * 这个常量就是丢线时给出的误差幅值（= 最外侧探头的位置）。
 */
#define GRAY_LOST_ERROR_FACTOR  1.0f

/* ------------------------------------------------------------------------- */
/* 数据结构                                                                    */
/* ------------------------------------------------------------------------- */

typedef struct
{
    /* ---- 原始数据 ---- */
    uint8_t  state_byte;                        /* 寄存器 5 的原始字节          */
    uint8_t  state_mask;                        /* 同上，语义化别名              */
    uint16_t analog[GRAY_CHANNEL_COUNT];        /* 各通道灰度模拟值              */
    uint16_t threshold[GRAY_CHANNEL_COUNT];     /* 各通道阈值                    */

    /* ---- 解算结果 ---- */
    float    position_mm;       /* 线相对传感器中心的位置，单位 mm，左正右负    */
    float    position_cm;       /* 同上，单位 cm                                */
    float    strength;          /* 各通道归一化量之和，用于判断是否看到线        */
    uint8_t  channel[GRAY_CHANNEL_COUNT];       /* 各通道是否看到线（0/1）       */
    uint8_t  active_count;      /* 看到线的通道数                                */
    uint8_t  line_lost;         /* 1 = 丢线（所有通道都没看到线）                */
    uint8_t  line_valid;        /* 1 = 本次位置解算有效                          */
    uint8_t  threshold_loaded;  /* 1 = 阈值已经从模块读回                        */
    uint8_t  fail_count;        /* 连续 I2C 失败次数                             */
} Gray_Sensor_t;

/* ------------------------------------------------------------------------- */
/* 接口                                                                        */
/* ------------------------------------------------------------------------- */

/*
 * 初始化驱动状态。I2C 外设本身由 CubeMX 生成的 MX_I2C1_Init() 负责，
 * 本函数不做硬件初始化，只清零内部状态。
 * 请在 MX_I2C1_Init() 之后调用一次。
 */
void Gray_Init(Gray_Sensor_t *sensor);

/* 设置线条的明暗关系，默认 GRAY_LINE_IS_DARK。 */
void Gray_SetLinePolarity(uint8_t line_is_bright);

/*
 * 从模块强制重新读取一次阈值（16 字节）。重新做过一键校准后调用。
 * 正常情况下不需要手动调用：Gray_Update() 会在首次运行时自动加载。
 */
uint8_t Gray_RefreshThreshold(Gray_Sensor_t *sensor, I2C_HandleTypeDef *hi2c);

/*
 * 核心接口：读一次传感器并完成全部解算。建议放在外环（循迹环）里调用。
 *
 * 返回值：
 *   0 = 一切正常
 *   1 = I2C 通信失败（本次数据无效，结构体中的位置信息保持上一次的值）
 *   2 = 传感器连续失败次数已达 GRAY_FAIL_LIMIT，判定掉线，上层应停车
 *
 * 内部做了三件事：
 *   1. 一次事务读寄存器 5（数字状态），一次事务读寄存器 6~21（16 字节模拟量）
 *   2. 用阈值把模拟量归一化，在线条方向取正
 *   3. 加权质心求线位置（模拟量算法，误差是连续的，D 项才有意义）
 */
uint8_t Gray_Update(Gray_Sensor_t *sensor, I2C_HandleTypeDef *hi2c);

/* 取线位置，单位 cm，左正右负。丢线时返回带方向的记忆值。 */
float Gray_GetPositionCm(const Gray_Sensor_t *sensor);

/* 取线位置，单位 mm。 */
float Gray_GetPositionMm(const Gray_Sensor_t *sensor);

/* 是否丢线：所有通道都没看到线。 */
uint8_t Gray_IsLineLost(const Gray_Sensor_t *sensor);

/* 是否线在正中间（用于判断可以加速的直道）。 */
uint8_t Gray_IsCentered(const Gray_Sensor_t *sensor, float tolerance_cm);

/*
 * 自检：阻塞约 6 秒，每 200ms 通过 huart2 打印一行
 *   8 路模拟值 / 8 路阈值 / 数字状态位图 / 解算出的 position
 * 用于验证接线、判断通道顺序、确认线条明暗关系、观察丢线行为。
 * 串口波特率 115200。
 */
void Gray_TestSequence(Gray_Sensor_t *sensor, I2C_HandleTypeDef *hi2c);

#endif //LINEFLLOWING_CAR_GRAY_H
