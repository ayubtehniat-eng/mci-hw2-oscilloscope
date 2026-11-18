#include "stm32f3xx.h"
#include "adc.h"

#define ADC_BUF_MASK  (ADC_BUF_SIZE - 1U)

/* Ring buffer: the ISR only writes s_head, the main loop only writes s_tail */
static volatile uint16_t s_buf[ADC_BUF_SIZE];
static volatile uint32_t s_head = 0;
static volatile uint32_t s_tail = 0;

static volatile uint32_t s_count    = 0;   /* conversions read */
static volatile uint32_t s_overruns = 0;   /* ADC_ISR.OVR events */
static volatile uint32_t s_dropped  = 0;   /* ring buffer full */

static void Adc_GpioInit(void)
{
    /* RCC_AHBENR.IOPAEN = 1 : clock GPIOA */
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    /* GPIOA_MODER.MODER1 = 11 : PA1 in analog mode (digital buffer disconnected) */
    GPIOA->MODER |= GPIO_MODER_MODER1;
    /* GPIOA_PUPDR.PUPDR1 = 00 : no pull-up / pull-down on the analog pin */
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;
}

static void Adc_Calibrate(void)
{
    /* ADC1_CR.ADVREGEN = 00 : leave the "intermediate" state first */
    ADC1->CR &= ~ADC_CR_ADVREGEN;
    /* ADC1_CR.ADVREGEN = 01 : enable the ADC voltage regulator */
    ADC1->CR |= ADC_CR_ADVREGEN_0;
    /* Regulator start-up time is 10 us max: ~2000 loop iterations is well over that at 48 MHz */
    for (volatile uint32_t i = 0; i < 2000U; i++) { }

    /* ADC1_CR.ADCALDIF = 0 : calibrate for single-ended inputs */
    ADC1->CR &= ~ADC_CR_ADCALDIF;
    /* ADC1_CR.ADCAL = 1 : start calibration (ADEN must be 0 at this point) */
    ADC1->CR |= ADC_CR_ADCAL;
    /* ADC1_CR.ADCAL : hardware clears it when calibration is complete */
    while ((ADC1->CR & ADC_CR_ADCAL) != 0U) { }
}

void Adc_Init(void)
{
    Adc_GpioInit();

    /* RCC_AHBENR.ADC12EN = 1 : clock ADC1 and ADC2 */
    RCC->AHBENR |= RCC_AHBENR_ADC12EN;
    /* ADC12_CCR.CKMODE = 01 : ADC clock = HCLK/1 = 48 MHz (synchronous mode, AHB prescaler must be 1) */
    ADC12_COMMON->CCR = (ADC12_COMMON->CCR & ~ADC_CCR_CKMODE) | ADC_CCR_CKMODE_0;

    Adc_Calibrate();

    /* ADC1_CFGR.RES = 00 : 12-bit resolution */
    ADC1->CFGR &= ~ADC_CFGR_RES;
    /* ADC1_CFGR.ALIGN = 0 : right-aligned result (0..4095 in DR[11:0]) */
    ADC1->CFGR &= ~ADC_CFGR_ALIGN;
    /* ADC1_CFGR.CONT = 0 : single conversion per trigger (the timer paces the sampling) */
    ADC1->CFGR &= ~ADC_CFGR_CONT;
    /* ADC1_CFGR.OVRMOD = 0 : on overrun keep the old data; the OVR flag tells us a sample was missed */
    ADC1->CFGR &= ~ADC_CFGR_OVRMOD;
    /* ADC1_CFGR.EXTSEL = 1011 (EXT11) : trigger source is TIM2_TRGO (see RM0316, ADC1/2 regular triggers) */
    ADC1->CFGR = (ADC1->CFGR & ~ADC_CFGR_EXTSEL)
               | ADC_CFGR_EXTSEL_0 | ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_3;
    /* ADC1_CFGR.EXTEN = 01 : hardware trigger on the rising edge of the selected trigger */
    ADC1->CFGR = (ADC1->CFGR & ~ADC_CFGR_EXTEN) | ADC_CFGR_EXTEN_0;

    /* ADC1_SMPR1.SMP2 = 101 : channel 2 sampling time 61.5 ADC clock cycles
       (conversion total = 61.5 + 12.5 = 74 cycles = 1.54 us, far less than the 100 us period) */
    ADC1->SMPR1 = (ADC1->SMPR1 & ~ADC_SMPR1_SMP2) | ADC_SMPR1_SMP2_0 | ADC_SMPR1_SMP2_2;

    /* ADC1_SQR1.L = 0000 : regular sequence has 1 conversion */
    ADC1->SQR1 &= ~ADC_SQR1_L;
    /* ADC1_SQR1.SQ1 = 2 : first (only) conversion is channel 2 = PA1 */
    ADC1->SQR1 = (ADC1->SQR1 & ~ADC_SQR1_SQ1) | (2U << ADC_SQR1_SQ1_Pos);

    /* ADC1_ISR: clear any stale flags by writing 1 (EOC, OVR) */
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_OVR;
    /* ADC1_IER.EOCIE = 1 : interrupt at the end of every conversion */
    ADC1->IER |= ADC_IER_EOC;
    /* ADC1_IER.OVRIE = 1 : interrupt if a conversion result was not read in time */
    ADC1->IER |= ADC_IER_OVR;

    /* NVIC: ADC must be the highest priority so no sample is ever delayed or lost */
    NVIC_SetPriority(ADC1_2_IRQn, 0);
    NVIC_EnableIRQ(ADC1_2_IRQn);

    /* ADC1_ISR.ADRDY = 1 : write 1 to clear the ready flag before enabling */
    ADC1->ISR = ADC_ISR_ADRDY;
    /* ADC1_CR.ADEN = 1 : enable the ADC */
    ADC1->CR |= ADC_CR_ADEN;
    /* ADC1_ISR.ADRDY : wait until the ADC is ready to convert */
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U) { }

    /* ADC1_CR.ADSTART = 1 : with EXTEN != 00 this arms the ADC; each TIM2 TRGO edge now starts a conversion */
    ADC1->CR |= ADC_CR_ADSTART;
}

/* Name comes from the startup file's vector table (shared by ADC1 and ADC2) */
void ADC1_2_IRQHandler(void)
{
    uint32_t isr = ADC1->ISR;

    if ((isr & ADC_ISR_OVR) != 0U)
    {
        ADC1->ISR = ADC_ISR_OVR;           /* ADC1_ISR.OVR : write 1 to clear */
        s_overruns++;
    }

    if ((isr & ADC_ISR_EOC) != 0U)
    {
        uint16_t sample = (uint16_t)ADC1->DR;   /* reading ADC1_DR also clears ADC1_ISR.EOC */
        s_count++;

        uint32_t next = (s_head + 1U) & ADC_BUF_MASK;
        if (next != s_tail)                /* room in the ring buffer? */
        {
            s_buf[s_head] = sample;
            s_head = next;
        }
        else
        {
            s_dropped++;                   /* consumer too slow: count it, do not overwrite */
        }
    }
}

uint8_t Adc_Pop(uint16_t *sample)
{
    if (s_tail == s_head)
    {
        return 0U;                         /* empty */
    }
    *sample = s_buf[s_tail];
    s_tail = (s_tail + 1U) & ADC_BUF_MASK;
    return 1U;
}

uint32_t Adc_GetSampleCount(void)  { return s_count; }
uint32_t Adc_GetOverrunCount(void) { return s_overruns; }
uint32_t Adc_GetDroppedCount(void) { return s_dropped; }