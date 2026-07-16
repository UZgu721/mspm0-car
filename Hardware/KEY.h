#ifndef _KEY_H
#define _KEY_H
#include "ti_msp_dl_config.h"
uint8_t key_read();
void key_serv_double();
struct Bkeys
{
	uint8_t age;
	uint8_t short_flag;
	uint8_t long_flag;
	uint8_t press;
	uint8_t double_ageEN;
	uint8_t double_age;
	uint8_t double_flag;
};

#endif 
