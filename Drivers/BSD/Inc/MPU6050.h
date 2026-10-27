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

/* 7 位地址 0x68（AD0 接地），HAL 用 8 位地址 */
#define MPU6050_I2C_ADDR        (0x68U << 1)
#define MPU6050_WHO_AM_I_VAL    0x68U

/* yaw 轴映射：X 朝前、Z 竖直 -> 用 Z 轴，左转为正 */
#define MPU6050_YAW_AXIS        2U
#define MPU6050_YAW_SIGN        (+1.0f)

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
void     MPU6050_ResetYaw(float deg);   /* 把航向角清零/置为指定值 */

#endif /* MPU6050_H */
