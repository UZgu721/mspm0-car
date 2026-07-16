#include "led.h"


void LED_ON(void)
{
	DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_9);
}

void LED_OFF(void)
{
	DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_9);
}

void LED_Toggle(void)
{
	DL_GPIO_togglePins(GPIOB,DL_GPIO_PIN_9);
}

void LED_Flash(uint16_t time)
{
	static uint16_t temp;
	if(time==0) LED_ON();
	else if(++temp==time) LED_Toggle(),temp=0;
}



