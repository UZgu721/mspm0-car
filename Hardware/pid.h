#ifndef _PID_H
#define _PID_H
#include "ti_msp_dl_config.h"

enum {
    POSITION_PID = 0,
    DELTA_PID,
};

typedef struct 
{
    float Target,Actual,Out;
    float Kp,Ki,Kd;
    float Pout,Iout,Dout;
    float error[3];
    float Outmax,Outmin,ErrInt,Alpha;
    uint8_t PID_MODE;
}PID_t;

void PID_Control(void);
void PID_Cal(PID_t *PID);


#endif 