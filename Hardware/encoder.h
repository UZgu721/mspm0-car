#ifndef _ENCODER_H
#define _ENCODER_H
#include "ti_msp_dl_config.h"

void Encoder_Init(void);
int32_t Get_encoder_left(void);
int32_t Get_encoder_right(void);

#endif
