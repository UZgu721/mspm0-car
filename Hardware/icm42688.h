#ifndef _ICM42688_H
#define _ICM42688_H
#include "ti_msp_dl_config.h"

/* ICM42688 I2C 地址 (AD0=0) */
#define ICM42688_I2C_ADDR      0x68

/* 寄存器地址 */
#define ICM42688_WHO_AM_I      0x75
#define ICM42688_BANK_SEL      0x76
#define ICM42688_PWR_MGMT0     0x4E
#define ICM42688_ACCEL_DATA_X1 0x1F
#define ICM42688_GYRO_DATA_X1  0x25

/* 初始化与数据读取 */
void ICM42688_Init(void);
void ICM42688_Read_Accel(int16_t *ax, int16_t *ay, int16_t *az);
void ICM42688_Read_Gyro(int16_t *gx, int16_t *gy, int16_t *gz);

/* 全局变量（供外部直接读取） */
extern float icm_accel_x, icm_accel_y, icm_accel_z;
extern float icm_gyro_x,  icm_gyro_y,  icm_gyro_z;

#endif
