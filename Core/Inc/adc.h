#include <stdint.h>
#define ADC_BUF_SIZE 1024U

void Adc_Init(void);

uint8_t Adc_Pop(uint16_t *sample);

uint32_t Adc_GetSampleCount(void);

uint32_t Adc_GetOverrunCount(void);

uint32_t Adc_GetDroppedCount(void);