#include"pid.h"
#include"motor.h"
extern int32_t vel_left,vel_right;
//四倍频编码器测速，40ms测一次，这里编码器测速速度范围约为 -200 ~ 200
PID_t motorA={
    .Kp = 5,
    .Ki = 2,
    .Kd = 0,
    .ErrInt=1500,
    .Outmax=2000,
    .Outmin=-2000,
    .PID_MODE = POSITION_PID
};
PID_t motorB={
    .Kp = 5,
    .Ki = 2,
    .Kd = 0,
    .ErrInt=1500,
    .Outmax=2000,
    .Outmin=-2000,
    .PID_MODE = POSITION_PID
};

//位置式PID实现，适用绝大多数情况
void PID_Control(void)
{
    motorA.Actual = vel_left;
    motorB.Actual = vel_right;

    PID_Cal(&motorA);
    PID_Cal(&motorB);

    Motor_SetSpeed(1,motorA.Out);
	Motor_SetSpeed(2,motorB.Out);
}

void PID_Cal(PID_t *PID)
{
    //保留上次和上上次的误差
    PID->error[2]=PID->error[1];
    PID->error[1]=PID->error[0];
    //计算本次误差
    PID->error[0]=PID->Target - PID->Actual;
    //计算输出
    if(PID->PID_MODE==POSITION_PID)
    {
        PID->Pout =  PID->Kp * PID->error[0];
        PID->Iout += PID->Ki * PID->error[0];
        if(PID->Iout>= PID->ErrInt) PID->Iout =  PID->ErrInt;
        if(PID->Iout<=-PID->ErrInt) PID->Iout = -PID->ErrInt;
        PID->Dout = (1.0f-(PID->Alpha))*PID->Kd*(PID->error[0]-PID->error[1])+(PID->Alpha)*PID->Dout;
        PID->Out  =  PID->Pout + PID->Iout + PID->Dout;
    }else if(PID->PID_MODE==DELTA_PID)
    {
        PID->Pout =   PID->Kp * (PID->error[0]-PID->error[1]);
        PID->Iout =   PID->Ki * PID->error[0];
        PID->Dout =   PID->Kd * (PID->error[0]-2*PID->error[1]+PID->error[2]);
        PID->Out  +=  PID->Pout + PID->Iout + PID->Dout;
    }
    //输出限幅
    if(PID->Out >= PID->Outmax) PID->Out = PID->Outmax;
    if(PID->Out <= PID->Outmin) PID->Out = PID->Outmin; 
}