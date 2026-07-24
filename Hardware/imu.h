#ifndef _IMU_H
#define _IMU_H

#include <stdbool.h>

typedef struct {
    float gx, gy, gz;
    float ax, ay, az;
    float dt;
} IMU_Data;

extern IMU_Data imu_data;

bool IMU_InitAndCalibrate(void);
bool IMU_Update(float dt, bool allow_bias_tracking);
void IMU_Read(IMU_Data *data);
float IMU_GetGyroZBias(void);

#endif
