#include "ahrs.h"
#include <math.h>

#define AHRS_RAD_TO_DEG          57.2957795f
#define AHRS_GYRO_DEADBAND_DPS   0.20f
#define AHRS_ACCEL_ALPHA         0.98f

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

static float previous_gz;
static int previous_gz_valid;

static float AHRS_WrapAngle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

void AHRS_Init(float Kp, float halfT)
{
    (void)Kp;
    (void)halfT;
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    roll = 0.0f;
    pitch = 0.0f;
    AHRS_ResetYaw();
}

void AHRS_ResetYaw(void)
{
    yaw = 0.0f;
    previous_gz = 0.0f;
    previous_gz_valid = 0;
}

void AHRS_Update(float gx, float gy, float gz,
                 float ax, float ay, float az, float dt)
{
    float accel_norm;
    float roll_gyro;
    float pitch_gyro;

    if (dt <= 0.0f) {
        return;
    }

    if (fabsf(gx) < AHRS_GYRO_DEADBAND_DPS) gx = 0.0f;
    if (fabsf(gy) < AHRS_GYRO_DEADBAND_DPS) gy = 0.0f;
    if (fabsf(gz) < AHRS_GYRO_DEADBAND_DPS) gz = 0.0f;

    roll_gyro = roll + gx * dt;
    pitch_gyro = pitch + gy * dt;
    accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (accel_norm > 0.95f && accel_norm < 1.05f) {
        float roll_acc = atan2f(ay, az) * AHRS_RAD_TO_DEG;
        float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * AHRS_RAD_TO_DEG;
        roll = AHRS_ACCEL_ALPHA * roll_gyro + (1.0f - AHRS_ACCEL_ALPHA) * roll_acc;
        pitch = AHRS_ACCEL_ALPHA * pitch_gyro + (1.0f - AHRS_ACCEL_ALPHA) * pitch_acc;
    } else {
        roll = roll_gyro;
        pitch = pitch_gyro;
    }

    if (!previous_gz_valid) {
        previous_gz = gz;
        previous_gz_valid = 1;
    }
    yaw = AHRS_WrapAngle(yaw + 0.5f * (previous_gz + gz) * dt);
    previous_gz = gz;
}

float AHRS_GetYaw(void)   { return yaw; }
float AHRS_GetRoll(void)  { return roll; }
float AHRS_GetPitch(void) { return pitch; }
