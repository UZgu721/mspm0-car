#ifndef _ICM42688_H
#define _ICM42688_H
#include "ti_msp_dl_config.h"

/* I2C 地址 (AD0=0) */
#define ICM42688_I2C_ADDR       (0x68)

/* ── Bank 0 寄存器 ── */
#define ICM42688_DEVICE_CONFIG      0x11
#define ICM42688_DRIVE_CONFIG       0x13
#define ICM42688_INT_CONFIG         0x14
#define ICM42688_FIFO_CONFIG        0x16
#define ICM42688_TEMP_DATA1         0x1D
#define ICM42688_TEMP_DATA0         0x1E
#define ICM42688_ACCEL_DATA_X1      0x1F
#define ICM42688_ACCEL_DATA_X0      0x20
#define ICM42688_ACCEL_DATA_Y1      0x21
#define ICM42688_ACCEL_DATA_Y0      0x22
#define ICM42688_ACCEL_DATA_Z1      0x23
#define ICM42688_ACCEL_DATA_Z0      0x24
#define ICM42688_GYRO_DATA_X1       0x25
#define ICM42688_GYRO_DATA_X0       0x26
#define ICM42688_GYRO_DATA_Y1       0x27
#define ICM42688_GYRO_DATA_Y0       0x28
#define ICM42688_GYRO_DATA_Z1       0x29
#define ICM42688_GYRO_DATA_Z0       0x2A
#define ICM42688_INT_STATUS         0x2D
#define ICM42688_FIFO_COUNTH        0x2E
#define ICM42688_FIFO_COUNTL        0x2F
#define ICM42688_FIFO_DATA          0x30
#define ICM42688_SIGNAL_PATH_RESET  0x4B
#define ICM42688_INTF_CONFIG0       0x4C
#define ICM42688_INTF_CONFIG1       0x4D
#define ICM42688_PWR_MGMT0          0x4E
#define ICM42688_GYRO_CONFIG0       0x4F
#define ICM42688_ACCEL_CONFIG0      0x50
#define ICM42688_GYRO_CONFIG1       0x51
#define ICM42688_GYRO_ACCEL_CONFIG0 0x52
#define ICM42688_ACCEL_CONFIG1      0x53
#define ICM42688_FIFO_CONFIG1       0x5F
#define ICM42688_FIFO_CONFIG2       0x60
#define ICM42688_FIFO_CONFIG3       0x61
#define ICM42688_INT_CONFIG0        0x63
#define ICM42688_INT_CONFIG1        0x64
#define ICM42688_INT_SOURCE0        0x65
#define ICM42688_INT_SOURCE1        0x66
#define ICM42688_WHO_AM_I           0x75
#define ICM42688_REG_BANK_SEL       0x76

/* WHO_AM_I 期望值 */
#define ICM42688_WHO_AM_I_VAL       (0x47)

/* ── 量程/ODR 枚举 ── */
enum icm42688_afs {
    ICM42688_AFS_16G = 0,
    ICM42688_AFS_8G,
    ICM42688_AFS_4G,
    ICM42688_AFS_2G,
};
enum icm42688_gfs {
    ICM42688_GFS_2000DPS = 0,
    ICM42688_GFS_1000DPS,
    ICM42688_GFS_500DPS,
    ICM42688_GFS_250DPS,
    ICM42688_GFS_125DPS,
    ICM42688_GFS_62_5DPS,
    ICM42688_GFS_31_25DPS,
    ICM42688_GFS_15_625DPS,
};
enum icm42688_aodr {
    ICM42688_AODR_1000HZ = 5,
};
enum icm42688_godr {
    ICM42688_GODR_1000HZ = 5,
};

/* ── 全局变量（原始数据） ── */
extern float icm42688_acc_x, icm42688_acc_y, icm42688_acc_z;
extern float icm42688_gyro_x, icm42688_gyro_y, icm42688_gyro_z;
extern float gx, gy, gz;
extern float ax, ay, az;

typedef struct {
    float gx, gy, gz;
    float ax, ay, az;
} ICM42688_Sample_t;

/* ── API ── */
uint8_t ICM42688_Init(void);   /* 返回1=成功, 0=失败 */
void ICM42688_Read_Accel(void);
void ICM42688_Read_Gyro(void);
void ICM42688_Read_GyroZ(void);  /* 只读陀螺Z（快通道，2次I2C） */
void ICM42688_ReadSample(ICM42688_Sample_t *sample);
void ICM42688_Set_Range(enum icm42688_afs afs, enum icm42688_aodr aodr,
                        enum icm42688_gfs gfs, enum icm42688_godr godr);

#endif
