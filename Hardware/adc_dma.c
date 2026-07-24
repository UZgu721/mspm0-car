#include "adc_dma.h"
#include "board.h"

/**
 * @brief 4次ADC平均采集（ADC连续模式，delay保证读取不同采样）
 */
unsigned int adc_getValue(void)
{
    unsigned int sum = 0;
    for (int i = 0; i < 4; i++) {
        delay_us(8);  /* ADC 5MHz, 每~5µs完成一次新转换 */
        sum += DL_ADC12_getMemResult(ADC12_0_INST, ADC12_0_ADCMEM_ADC_CH0);
    }
    return sum / 4;
}
