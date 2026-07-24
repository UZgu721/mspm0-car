#include "imu.h"
#include "ahrs.h"
#include "board.h"
#include "icm42688.h"
#include <math.h>

#define IMU_CALIBRATION_DISCARD_COUNT  100U
#define IMU_CALIBRATION_SAMPLE_COUNT   300U
#define IMU_CALIBRATION_INTERVAL_MS    2U
#define IMU_BIAS_TRACK_GAIN             0.001f

IMU_Data imu_data;

static float gyro_bias_x;
static float gyro_bias_y;
static float gyro_bias_z;
static bool imu_ready;

bool IMU_InitAndCalibrate(void)
{
    ICM42688_Sample_t sample;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    uint32_t i;

    imu_ready = false;
    if (ICM42688_Init() == 0U) {
        return false;
    }

    for (i = 0U; i < IMU_CALIBRATION_DISCARD_COUNT; ++i) {
        ICM42688_ReadSample(&sample);
        delay_ms(IMU_CALIBRATION_INTERVAL_MS);
    }

    for (i = 0U; i < IMU_CALIBRATION_SAMPLE_COUNT; ++i) {
        ICM42688_ReadSample(&sample);
        sum_x += sample.gx;
        sum_y += sample.gy;
        sum_z += sample.gz;
        delay_ms(IMU_CALIBRATION_INTERVAL_MS);
    }

    gyro_bias_x = sum_x / (float)IMU_CALIBRATION_SAMPLE_COUNT;
    gyro_bias_y = sum_y / (float)IMU_CALIBRATION_SAMPLE_COUNT;
    gyro_bias_z = sum_z / (float)IMU_CALIBRATION_SAMPLE_COUNT;
    imu_data = (IMU_Data){0};
    imu_ready = true;
    return true;
}

bool IMU_Update(float dt, bool allow_bias_tracking)
{
    ICM42688_Sample_t sample;
    float accel_norm_sq;

    if (!imu_ready || dt <= 0.0f) {
        return false;
    }
    if (dt > 0.05f) {
        dt = 0.05f;
    }

    ICM42688_ReadSample(&sample);
    imu_data.gx = sample.gx - gyro_bias_x;
    imu_data.gy = sample.gy - gyro_bias_y;
    imu_data.gz = sample.gz - gyro_bias_z;
    imu_data.ax = sample.ax;
    imu_data.ay = sample.ay;
    imu_data.az = sample.az;
    imu_data.dt = dt;

    accel_norm_sq = sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az;
    if (allow_bias_tracking && accel_norm_sq > 0.90f && accel_norm_sq < 1.10f &&
        fabsf(imu_data.gx) < 0.8f && fabsf(imu_data.gy) < 0.8f && fabsf(imu_data.gz) < 0.8f) {
        gyro_bias_x += IMU_BIAS_TRACK_GAIN * (sample.gx - gyro_bias_x);
        gyro_bias_y += IMU_BIAS_TRACK_GAIN * (sample.gy - gyro_bias_y);
        gyro_bias_z += IMU_BIAS_TRACK_GAIN * (sample.gz - gyro_bias_z);
    }

    AHRS_Update(imu_data.gx, imu_data.gy, imu_data.gz,
                imu_data.ax, imu_data.ay, imu_data.az, imu_data.dt);
    return true;
}

void IMU_Read(IMU_Data *data)
{
    if (data != 0) {
        *data = imu_data;
    }
}

float IMU_GetGyroZBias(void)
{
    return gyro_bias_z;
}
