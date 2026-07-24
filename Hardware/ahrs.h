#ifndef _AHRS_H
#define _AHRS_H

extern float q0, q1, q2, q3;
extern float roll, pitch, yaw;

void AHRS_Init(float Kp, float halfT);
void AHRS_ResetYaw(void);
void AHRS_Update(float gx, float gy, float gz,
                 float ax, float ay, float az, float dt);
float AHRS_GetYaw(void);
float AHRS_GetRoll(void);
float AHRS_GetPitch(void);

#endif
