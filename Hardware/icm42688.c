#include "icm42688.h"

float icm_accel_x, icm_accel_y, icm_accel_z;
float icm_gyro_x,  icm_gyro_y,  icm_gyro_z;

/**
 * @brief ICM42688 初始化（I2C0, 400kHz）
 *        需在 SYSCFG_DL_init() 后调用
 */
void ICM42688_Init(void)
{
    /* TODO: 实现 I2C 寄存器配置 */
    (void)0;
}

/**
 * @brief 读取加速度计原始值
 */
void ICM42688_Read_Accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    /* TODO: 通过 I2C 读取 ACCEL_DATA_X1 起始的6字节 */
    *ax = 0; *ay = 0; *az = 0;
}

/**
 * @brief 读取陀螺仪原始值
 */
void ICM42688_Read_Gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    /* TODO: 通过 I2C 读取 GYRO_DATA_X1 起始的6字节 */
    *gx = 0; *gy = 0; *gz = 0;
}
