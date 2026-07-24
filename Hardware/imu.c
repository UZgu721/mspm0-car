#include "imu.h"
#include "ahrs.h"
#include "board.h"
#include "icm42688.h"
#include <math.h>

#define IMU_CALIBRATION_DISCARD_COUNT  100U
#define IMU_CALIBRATION_SAMPLE_COUNT   300U
#define IMU_CALIBRATION_INTERVAL_MS    2U
#define IMU_BIAS_TRACK_GAIN             0.001f
#define IMU_MG_TO_G                     0.001f

IMU_Data imu_data;

static float gyro_bias_x;
static float gyro_bias_y;
static float gyro_bias_z;
static float yaw_axis_x;
static float yaw_axis_y;
static float yaw_axis_z;
static float yaw_rate_dps;
static uint16_t last_dt_ms;
static uint32_t late_update_count;
static bool imu_ready;

bool IMU_InitAndCalibrate(void)
{
    ICM42688_Sample_t sample;
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    float gravity_norm;
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
        sum_gx += sample.gx;
        sum_gy += sample.gy;
        sum_gz += sample.gz;
        sum_ax += sample.ax;
        sum_ay += sample.ay;
        sum_az += sample.az;
        delay_ms(IMU_CALIBRATION_INTERVAL_MS);
    }

    gyro_bias_x = sum_gx / (float)IMU_CALIBRATION_SAMPLE_COUNT;
    gyro_bias_y = sum_gy / (float)IMU_CALIBRATION_SAMPLE_COUNT;
    gyro_bias_z = sum_gz / (float)IMU_CALIBRATION_SAMPLE_COUNT;

    gravity_norm = sqrtf(sum_ax * sum_ax + sum_ay * sum_ay + sum_az * sum_az);
    if (gravity_norm < 1.0f) {
        return false;
    }
    yaw_axis_x = sum_ax / gravity_norm;
    yaw_axis_y = sum_ay / gravity_norm;
    yaw_axis_z = sum_az / gravity_norm;

    /* Keep the yaw sign compatible with the former direct Z-axis integration. */
    if (yaw_axis_z < 0.0f) {
        yaw_axis_x = -yaw_axis_x;
        yaw_axis_y = -yaw_axis_y;
        yaw_axis_z = -yaw_axis_z;
    }

    imu_data = (IMU_Data){0};
    yaw_rate_dps = 0.0f;
    last_dt_ms = 0U;
    late_update_count = 0U;
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

    last_dt_ms = (uint16_t)(dt * 1000.0f + 0.5f);
    if (last_dt_ms > 20U) {
        ++late_update_count;
    }

    ICM42688_ReadSample(&sample);
    imu_data.gx = sample.gx - gyro_bias_x;
    imu_data.gy = sample.gy - gyro_bias_y;
    imu_data.gz = sample.gz - gyro_bias_z;
    imu_data.ax = sample.ax * IMU_MG_TO_G;
    imu_data.ay = sample.ay * IMU_MG_TO_G;
    imu_data.az = sample.az * IMU_MG_TO_G;
    imu_data.dt = dt;

    accel_norm_sq = imu_data.ax * imu_data.ax + imu_data.ay * imu_data.ay + imu_data.az * imu_data.az;
    if (allow_bias_tracking && accel_norm_sq > 0.90f && accel_norm_sq < 1.10f &&
        fabsf(imu_data.gx) < 0.8f && fabsf(imu_data.gy) < 0.8f && fabsf(imu_data.gz) < 0.8f) {
        gyro_bias_x += IMU_BIAS_TRACK_GAIN * (sample.gx - gyro_bias_x);
        gyro_bias_y += IMU_BIAS_TRACK_GAIN * (sample.gy - gyro_bias_y);
        gyro_bias_z += IMU_BIAS_TRACK_GAIN * (sample.gz - gyro_bias_z);
    }

    /* Project the three sensor gyro axes onto the calibrated vertical axis.
     * This remains valid when the ICM module is fixed but tilted on the car. */
    yaw_rate_dps = imu_data.gx * yaw_axis_x +
                   imu_data.gy * yaw_axis_y +
                   imu_data.gz * yaw_axis_z;

    AHRS_Update(imu_data.gx, imu_data.gy, yaw_rate_dps,
                imu_data.ax, imu_data.ay, imu_data.az, imu_data.dt);
    return true;
}

void IMU_Read(IMU_Data *data)
{
    if (data != 0) {
        *data = imu_data;
    }
}

float IMU_GetGyroZBias(void)       { return gyro_bias_z; }
float IMU_GetGyroZ(void)           { return imu_data.gz; }
float IMU_GetYawRate(void)          { return yaw_rate_dps; }
uint16_t IMU_GetLastDtMs(void)      { return last_dt_ms; }
uint32_t IMU_GetLateUpdateCount(void) { return late_update_count; }
bool IMU_IsCalibrated(void)         { return imu_ready; }
