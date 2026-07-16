#ifndef _ADC_DMA_H
#define _ADC_DMA_H
#include "ti_msp_dl_config.h"
#include <stdbool.h>

extern volatile bool gCheckADC;
unsigned int adc_getValue(void);
#endif
