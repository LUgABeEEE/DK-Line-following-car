/**
 * @file    MPU6050.c
 * @brief   MPU6050 驱动实现（轮询 I2C，和 OLED 共用 I2C2）
 */

#include "MPU6050.h"
#include <string.h>

/* ------------------------------------------------------------------------- */
/* 寄存器                                                                      */
/* ------------------------------------------------------------------------- */
#define MPU_REG_SMPLRT_DIV      0x19U
#define MPU_REG_CONFIG          0x1AU
#define MPU_REG_GYRO_CONFIG     0x1BU
#define MPU_REG_ACCEL_CONFIG    0x1CU
#define MPU_REG_INT_ENABLE      0x38U
#define MPU_REG_ACCEL_XOUT_H    0x3BU
#define MPU_REG_GYRO_XOUT_H     0x43U
#define MPU_REG_USER_CTRL       0x6AU
#define MPU_REG_PWR_MGMT_1      0x6BU
#define MPU_REG_WHO_AM_I        0x75U

/* ------------------------------------------------------------------------- */
/* 内部状态                                                                    */
/* ------------------------------------------------------------------------- */
static I2C_HandleTypeDef *s_hi2c = NULL;
static uint16_t s_addr  = (uint16_t)(MPU6050_ADDR_7BIT_LOW << 1);  /* 8 位地址 */
static uint8_t  s_ready = 0U;
static uint16_t s_faults = 0U;

static float    s_bias_x = 0.0f;
static float    s_bias_y = 0.0f;
static float    s_bias_z = 0.0f;

static float    s_yaw_rate_dps = 0.0f;
static float    s_yaw_deg = 0.0f;
static uint32_t s_last_tick = 0U;

/* ------------------------------------------------------------------------- */
/* 底层读写                                                                    */
/* ------------------------------------------------------------------------- */
static uint8_t MPU_WriteReg(uint8_t reg, uint8_t value)
{
    if (s_hi2c == NULL)
    {
        return 0U;
    }

    return (HAL_I2C_Mem_Write(s_hi2c, s_addr, reg,
                              I2C_MEMADD_SIZE_8BIT, &value, 1U, 10U) == HAL_OK)
           ? 1U : 0U;
}

static uint8_t MPU_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if ((s_hi2c == NULL) || (buf == NULL) || (len == 0U))
    {
        return 0U;
    }

    return (HAL_I2C_Mem_Read(s_hi2c, s_addr, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, len, 10U) == HAL_OK)
           ? 1U : 0U;
}

static uint8_t MPU_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t b[6];

    if (MPU_ReadRegs(MPU_REG_GYRO_XOUT_H, b, sizeof(b)) == 0U)
    {
        return 0U;
    }

    *gx = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    *gy = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
    *gz = (int16_t)(((uint16_t)b[4] << 8) | b[5]);

    return 1U;
}

static float MPU_RawToDps(int16_t raw)
{
    return (float)raw / MPU6050_GYRO_LSB_PER_DPS;
}

/* 按宏选择 yaw 轴 */
static float MPU_PickYawAxis(int16_t gx, int16_t gy, int16_t gz)
{
#if (MPU6050_YAW_AXIS == 0)
    return MPU_RawToDps(gx);
#elif (MPU6050_YAW_AXIS == 1)
    return MPU_RawToDps(gy);
#else
    return MPU_RawToDps(gz);
#endif
}

/* ------------------------------------------------------------------------- */
/* 初始化和标定                                                                */
/* ------------------------------------------------------------------------- */
/*
 * 在当前 I2C 速率下探测地址：
 *   AD0 接 GND = 0x68，接 VCC = 0x69；WHO_AM_I 固定 0x68，与地址无关。
 * 返回 1 = 找到并已设置 s_addr。
 */
static uint8_t MPU_ProbeAddress(void)
{
    static const uint8_t cand[2] =
    {
        (uint8_t)(MPU6050_ADDR_7BIT_LOW  << 1),
        (uint8_t)(MPU6050_ADDR_7BIT_HIGH << 1)
    };
    uint8_t who = 0U;
    uint8_t k;

    for (k = 0U; k < 2U; k++)
    {
        s_addr = cand[k];

        if ((MPU_ReadRegs(MPU_REG_WHO_AM_I, &who, 1U) != 0U) &&
            (who == MPU6050_WHO_AM_I_VAL))
        {
            return 1U;
        }
    }

    return 0U;
}

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
        return 0U;
    }

    s_hi2c = hi2c;
    s_ready = 0U;
    s_faults = 0U;
    s_yaw_rate_dps = 0.0f;
    s_yaw_deg = 0.0f;

    /* 模块上电需要几十毫秒稳定 */
    HAL_Delay(30U);

    if (MPU_ProbeAddress() == 0U)
    {
        /*
         * 400kHz 探不到时降到 100kHz 再试一次。
         *
         * 杜邦线长、模块上拉电阻弱、与 OLED 共总线时，400kHz 很容易失败；
         * 这种"线材问题"在 100kHz 下往往就正常了。降速只影响刷新速度，
         * 不影响控制（IMU 本身 20ms 读一次）。
         */
        if (hi2c->Init.ClockSpeed > 100000U)
        {
            uint32_t old_speed = hi2c->Init.ClockSpeed;

            hi2c->Init.ClockSpeed = 100000U;
            (void)HAL_I2C_Init(hi2c);
            HAL_Delay(5U);

            if (MPU_ProbeAddress() != 0U)
            {
                /* 保持 100kHz：OLED 会稍慢，但 IMU 能用了 */
            }
            else
            {
                /* 还是找不到：恢复原速率，按"没接模块"处理 */
                hi2c->Init.ClockSpeed = old_speed;
                (void)HAL_I2C_Init(hi2c);
                return 0U;
            }
        }
        else
        {
            return 0U;
        }
    }

    (void)MPU_WriteReg(MPU_REG_PWR_MGMT_1, 0x80U);  /* 复位            */
    HAL_Delay(60U);
    (void)MPU_WriteReg(MPU_REG_PWR_MGMT_1, 0x01U);  /* PLL(X 陀螺) 时钟 */
    (void)MPU_WriteReg(MPU_REG_SMPLRT_DIV, 0x04U);  /* 1kHz/(1+4)=200Hz */
    (void)MPU_WriteReg(MPU_REG_CONFIG,     0x03U);  /* DLPF 44Hz       */
    (void)MPU_WriteReg(MPU_REG_GYRO_CONFIG, 0x08U); /* ±500°/s         */
    (void)MPU_WriteReg(MPU_REG_ACCEL_CONFIG, 0x00U);/* ±2g             */
    (void)MPU_WriteReg(MPU_REG_INT_ENABLE, 0x00U);  /* 轮询，不用中断  */
    (void)MPU_WriteReg(MPU_REG_USER_CTRL,  0x00U);  /* 关 FIFO         */

    s_last_tick = HAL_GetTick();
    s_ready = 1U;

    return 1U;
}

uint8_t MPU6050_Calibrate(uint16_t samples)
{
    int32_t  sx = 0;
    int32_t  sy = 0;
    int32_t  sz = 0;
    int16_t  gx, gy, gz;
    uint16_t i;

    if ((s_ready == 0U) || (samples == 0U))
    {
        return 0U;
    }

    for (i = 0U; i < samples; i++)
    {
        if (MPU_ReadGyroRaw(&gx, &gy, &gz) == 0U)
        {
            s_faults++;
            return 0U;
        }

        sx += gx;
        sy += gy;
        sz += gz;

        HAL_Delay(2U);
    }

    s_bias_x = (float)sx / (float)samples / MPU6050_GYRO_LSB_PER_DPS;
    s_bias_y = (float)sy / (float)samples / MPU6050_GYRO_LSB_PER_DPS;
    s_bias_z = (float)sz / (float)samples / MPU6050_GYRO_LSB_PER_DPS;

    s_yaw_deg = 0.0f;
    s_last_tick = HAL_GetTick();

    return 1U;
}

/* ------------------------------------------------------------------------- */
/* 周期更新                                                                    */
/* ------------------------------------------------------------------------- */
void MPU6050_Update(void)
{
    int16_t  gx, gy, gz;
    uint32_t now;
    uint32_t dt_ms;
    float    rate;

    if (s_ready == 0U)
    {
        return;
    }

    if (MPU_ReadGyroRaw(&gx, &gy, &gz) == 0U)
    {
        s_faults++;
        s_yaw_rate_dps = 0.0f;      /* 这一拍不积分，避免脏数据 */
        return;
    }

    (void)gx;
    (void)gy;

    rate = MPU_PickYawAxis(gx, gy, gz);

    /* 减零偏（只有 Z 轴有独立 bias；X/Y 留着以后用） */
#if (MPU6050_YAW_AXIS == 0)
    rate -= s_bias_x;
#elif (MPU6050_YAW_AXIS == 1)
    rate -= s_bias_y;
#else
    rate -= s_bias_z;
#endif

    s_yaw_rate_dps = MPU6050_YAW_SIGN * rate;

    now = HAL_GetTick();
    dt_ms = now - s_last_tick;
    s_last_tick = now;

    if (dt_ms > 200U)
    {
        dt_ms = 200U;               /* 任务被长时间阻塞时不硬积 */
    }

    s_yaw_deg += s_yaw_rate_dps * ((float)dt_ms / 1000.0f);
}

/* ------------------------------------------------------------------------- */
/* 查询接口                                                                    */
/* ------------------------------------------------------------------------- */
uint8_t MPU6050_IsReady(void)
{
    return s_ready;
}

float MPU6050_GetYawRateDps(void)
{
    return s_yaw_rate_dps;
}

float MPU6050_GetYawDeg(void)
{
    return s_yaw_deg;
}

float MPU6050_GetBiasDps(void)
{
    return s_bias_z;
}

uint16_t MPU6050_GetFaults(void)
{
    return s_faults;
}

void MPU6050_ResetYaw(float deg)
{
    s_yaw_deg = deg;
}
