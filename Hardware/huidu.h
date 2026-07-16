#ifndef _HUIDU_H
#define _HUIDU_H
#include "ti_msp_dl_config.h"
#include "graysensor.h"

extern No_MCU_Sensor huidu_sensor;

void    Huidu_Sensor_Init(void);
uint8_t Huidu_AutoCalibrate(void);
void    Huidu_Sensor_Task(void);
uint8_t Read_HuiDu(void);
int     Get_HuiDu_Error(void);
void    HuiDu_PID(void);

#endif
