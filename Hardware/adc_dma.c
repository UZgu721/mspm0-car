#include "adc_dma.h"
#include "board.h"   /* delay_us */

/**
 * @brief 4次ADC平均采集（降噪2×，单次野点不影响结果）
 */
unsigned int adc_getValue(void)
{
    unsigned int sum = 0;
    for (int i = 0; i < 4; i++) {
        delay_us(6);   /* 等ADC完成一轮转换（ADC时钟5MHz，~5µs/次） */
        sum += DL_ADC12_getMemResult(ADC12_0_INST, ADC12_0_ADCMEM_ADC_CH0);
    }
    return sum / 4;
}
