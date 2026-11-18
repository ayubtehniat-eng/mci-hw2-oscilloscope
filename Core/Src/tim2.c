#include "stm32f3xx.h"
#include "tim2.h"

/* Timer clock = 48 MHz (APB1 = 24 MHz, prescaler /2 -> timer clock x2).
   f_update = f_tim / ((PSC+1) * (ARR+1)) = 48 MHz / (48 * 100) = 10 kHz */
#define TIM2_PSC_VALUE   47U
#define TIM2_ARR_VALUE   99U

void Tim2_Init10kHz(void)
{
    /* RCC_APB1ENR.TIM2EN = 1 : clock the TIM2 peripheral */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* TIM2_PSC : counter clock = 48 MHz / (47+1) = 1 MHz */
    TIM2->PSC = TIM2_PSC_VALUE;
    /* TIM2_ARR : update event every (99+1) counts = 100 us -> 10 kHz */
    TIM2->ARR = TIM2_ARR_VALUE;

    /* TIM2_CR1.ARPE = 1 : ARR is preloaded (changes take effect at the update event) */
    TIM2->CR1 |= TIM_CR1_ARPE;
    /* TIM2_CR1.DIR = 0 and CMS = 00 : up-counting, edge-aligned (reset values) */

    /* TIM2_CR2.MMS = 010 : "update" event is sent on TRGO (this triggers the ADC) */
    TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | TIM_CR2_MMS_1;

    /* TIM2_EGR.UG = 1 : generate an update now so PSC/ARR are loaded into the shadow
       registers. Done BEFORE the ADC is armed so this one TRGO pulse is not used. */
    TIM2->EGR = TIM_EGR_UG;
    /* TIM2_SR.UIF = 0 : clear the update flag set by the UG above */
    TIM2->SR = 0U;
}

void Tim2_Start(void)
{
    /* TIM2_CR1.CEN = 1 : start counting */
    TIM2->CR1 |= TIM_CR1_CEN;
}