#include "motor.h"
//speed取值为-2000~+2000,num=1为A号电机，=2为B号电机
void Motor_SetSpeed(uint8_t num,int32_t Speed)
{
	
	if(num==1)
	{
		if (Speed >= 0)							//如果设置正转的速度值
		{
			DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_13);	//PA4置高电平
			DL_GPIO_clearPins(GPIOA,DL_GPIO_PIN_14);	//PA5置低电平，设置方向为正转
			DL_Timer_setCaptureCompareValue(TIMG6,Speed,DL_TIMER_CC_0_INDEX);//PWM设置为速度值
		}
		else									//否则，即设置反转的速度值
		{
			DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_14);	//PA4置低电平
			DL_GPIO_clearPins(GPIOA,DL_GPIO_PIN_13);	//PA5置高电平，设置方向为反转
			DL_Timer_setCaptureCompareValue(TIMG6,-Speed,DL_TIMER_CC_0_INDEX);//PWM设置为负的速度值，因为此时速度值为负数，而PWM只能给正数
		}
	}
	if(num==2)
	{
		if (Speed >= 0)							//如果设置正转的速度值
		{
			DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_17);	//PA4置高电平
			DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_16);	//PA5置低电平，设置方向为正转
			DL_Timer_setCaptureCompareValue(TIMG6,Speed,DL_TIMER_CC_1_INDEX);//PWM设置为速度值
		}
		else									//否则，即设置反转的速度值
		{
			DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_16);	//PA4置低电平
			DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_17);	//PA5置高电平，设置方向为反转
			DL_Timer_setCaptureCompareValue(TIMG6,-Speed,DL_TIMER_CC_1_INDEX);//PWM设置为负的速度值，因为此时速度值为负数，而PWM只能给正数
		}
	}
}





