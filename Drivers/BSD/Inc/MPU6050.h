/**
 * @file    MPU6050.h
 * @brief   MPU6050 六轴陀螺仪驱动（I2C2，与 OLED 共总线）
 *
 * 用途：给循迹外环提供"车身真实偏航角速度"和"航向角"，
 *       用于 yaw 阻尼、定角转弯、航向融合。
 *
 * 安装约定（实测）：
 *   - X 轴指向前方，Z 轴竖直向上（板子平放）；
 *   - 左转为正：绕 +Z 逆时针（从上往下看），与 corner_dir 的符号约定一致。
 *
 * 如果实际装法不同，只改下面两个宏即可，不用动逻辑：
 *   MPU6050_YAW_AXIS  : 0=X 1=Y 2=Z（yaw 用哪根轴）
 *   MPU6050_YAW_SIGN  : +1 / -1（装反了就翻符号）
 */

#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include <stdint.h>

/*
 * I2C 地址（7 位）：
 *   AD0 接 GND -> 0x68
 *   AD0 接 VCC -> 0x69   （本车实际接 VCC，见用户说明）
 * 驱动会在初始化时自动探测这两个地址，不用手动改；
 * WHO_AM_I（0x75）固定返回 0x68，与地址无关。
 */
#define MPU6050_ADDR_7BIT_LOW   0x68U
#define MPU6050_ADDR_7BIT_HIGH  0x69U
/*
 * WHO_AM_I 允许值。
 * 市面上大量"MPU6050"实际是 MPU6500/9250 克隆片，ID 是 0x70/0x71/0x73；
 * 寄存器映射对陀螺部分兼容，所以这些 ID 也接受。
 */
#define MPU6050_WHO_AM_I_VAL    0x68U
#define MPU6050_WHO_AM_I_6500   0x70U
#define MPU6050_WHO_AM_I_6500B  0x71U
#define MPU6050_WHO_AM_I_9250   0x73U

/* yaw 轴映射：X 朝前、Z 竖直 -> 用 Z 轴，左转为正 */
#define MPU6050_YAW_AXIS        2U
/*
 * yaw 符号：逆时针（左转）为正。
 * 2026-10-28 实测（ax，SIGN=-1）：手转逆时针时 [I] g 仍为负、y 减小，
 * 说明原始 gyro Z 对逆时针就是正，因此这里取 +1。
 * [I] 行会带 sg%+d，烧错符号一眼可见。
 */
#define MPU6050_YAW_SIGN        (+1.0f)

/*
 * 400kHz 探测失败时是否允许降到 100kHz 重试。
 * 2026-10-28 已在 I2C 总线上补了 1.7k 上拉，锁死 400kHz；
 * 如果以后换板/线材又出现认不到，把它改回 1 即可。
 */
#define MPU6050_ALLOW_SLOW_FALLBACK  0

/* 量程：±500°/s（循迹车最快约 200°/s，留余量） */
#define MPU6050_GYRO_FS         500.0f
#define MPU6050_GYRO_LSB_PER_DPS (65536.0f / (2.0f * MPU6050_GYRO_FS))

/**
 * 初始化（探测 WHO_AM_I、复位、配置 DLPF/量程/采样率）。
 * 返回 1 = 成功，0 = 没找到模块（不影响其它功能）。
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);

/**
 * 静止零偏标定：连续采样 samples 次取平均。
 * 必须在车静止时调用（上电后、启动前）。
 * 返回 1 = 成功。
 */
uint8_t MPU6050_Calibrate(uint16_t samples);

/**
 * 周期读取 + 航向积分。放在 20ms 控制节拍里调用。
 * 读取失败只累计 faults，不阻塞。
 */
void MPU6050_Update(void);

uint8_t  MPU6050_IsReady(void);
float    MPU6050_GetYawRateDps(void);   /* 当前偏航角速度，左正右负 */
float    MPU6050_GetYawDeg(void);       /* 累计航向角，度；左正右负 */
float    MPU6050_GetBiasDps(void);      /* 标定得到的 Z 轴零偏 */
uint16_t MPU6050_GetFaults(void);       /* 累计 I2C 失败次数 */
uint8_t  MPU6050_GetWhoAmI(void);       /* 实际读到的 WHO_AM_I（诊断用） */
void     MPU6050_ResetYaw(float deg);   /* 把航向角清零/置为指定值 */

#endif /* MPU6050_H */
