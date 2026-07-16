#include "KEY.h"
struct Bkeys bkeys[3]={0};//两个按键
uint8_t key_read()
{
	if(DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_8)==0) return 1;
	else return 0;
}
void key_serv_double()
{
	uint8_t key_sta=key_read();
	if(key_sta!=0)
	{
		bkeys[key_sta].age++;
		if(bkeys[key_sta].age==2) bkeys[key_sta].press=1;
	}else
	{
		for(int i=0;i<3;i++)
		{
			if(bkeys[i].age > 69)  // 按住超过700ms
			{
				bkeys[i].long_flag = 1;  // 松手后才标记长按
			}

			if(bkeys[i].press==1&&bkeys[i].double_ageEN==1)//判断双击，过程为：当一次短单击按下后，press置1，松开后进入else部分，ageEN=1，开始计数，同时会执行把press置0，判断是否是单击，这段时间内可能会进入多次这个函数，当相隔不久的下一次单击按下后，press又会置1，且没有判断为单击ageEN没有置0，松开后又一次进入else部分，判断为双击，双击标志位置1，其他一些清零         
			{
				bkeys[i].double_flag=1;
				bkeys[i].press=0;//同理，双击判断完成后清零ageEN标志位方便下一次判断单双击？
				bkeys[i].double_ageEN=0;
				bkeys[i].double_age=0;//原视频没有清0？
			}
			if(bkeys[i].press==1&&bkeys[i].long_flag==0) bkeys[i].double_ageEN=1;//按下一次press=1后，且不是长按，松开后开始使能计时		
			if(bkeys[i].double_ageEN==1) bkeys[i].double_age++;
			if(bkeys[i].double_ageEN==1&&bkeys[i].double_age>20)//按下一次按键后，等了一会并没有按下第二次，是一个单击的过程，即判断按键单击
			{
				bkeys[i].short_flag=1;
				bkeys[i].double_age=0;//每一个按键按下都是独立的过程需要double_age从0开始计数，这样每次启用double_ageEN，
				bkeys[i].double_ageEN=0;//完成按键判断后，都需要将这两个清0方便下一次的判断，这样判断之间是相互独立的不干扰
			}
			bkeys[i].age=0;
			bkeys[i].press=0;
			//bkeys[i].long_flag=0;
		}
	}
	//if(bkeys[key_sta].age>69) bkeys[key_sta].long_flag=1;//计时超过700ms，认定为有效的长按键
}
