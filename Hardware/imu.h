#ifndef _IMU_H
#define _IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float gx, gy, gz;   /* Gyroscope, dps, sensor coordinates */
    float ax, ay, az;   /* Accelerometer, g, sensor coordinates */
    float dt;           /* Actual elapsed time, seconds */
} IMU_Data;

extern IMU_Data imu_data;

bool IMU_InitAndCalibrate(void);
bool IMU_Update(float dt, bool allow_bias_tracking);
void IMU_Read(IMU_Data *data);
float IMU_GetGyroZBias(void);
float IMU_GetGyroZ(void);
float IMU_GetYawRate(void);
uint16_t IMU_GetLastDtMs(void);
uint32_t IMU_GetLateUpdateCount(void);
bool IMU_IsCalibrated(void);

#endif
