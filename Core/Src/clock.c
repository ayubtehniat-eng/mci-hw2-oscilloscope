#include "stm32f3xx.h"
#include "clock.h"

void Clock_Init48MHz(void)
{
    /* FLASH_ACR.LATENCY = 001 : one wait state (required for 24 MHz < HCLK <= 48 MHz) */
    FLASH->ACR |= FLASH_ACR_LATENCY_0;
    /* FLASH_ACR.PRFTBE = 1 : enable the prefetch buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* RCC_CFGR.PLLSRC = 0 : PLL input is HSI/2 = 4 MHz (reset value, cleared to be explicit) */
    RCC->CFGR &= ~RCC_CFGR_PLLSRC;
    /* RCC_CFGR.PLLMUL = 1010 : PLL multiplier x12 -> 4 MHz x 12 = 48 MHz (PLL must be off) */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PLLMUL) | RCC_CFGR_PLLMUL12;
    /* RCC_CFGR.PPRE1 = 100 : APB1 prescaler /2 -> 24 MHz (APB1 max is 36 MHz) */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV2;
    /* RCC_CFGR.HPRE = 0000 and PPRE2 = 000 : AHB and APB2 not divided (reset values) */

    /* RCC_CR.PLLON = 1 : switch the PLL on */
    RCC->CR |= RCC_CR_PLLON;
    /* RCC_CR.PLLRDY : wait until the PLL has locked */
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U) { }

    /* RCC_CFGR.SW = 10 : select the PLL as system clock */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    /* RCC_CFGR.SWS = 10 : wait until hardware confirms the PLL is the system clock */
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }

    /* CMSIS: recompute the SystemCoreClock variable (now 48000000) */
    SystemCoreClockUpdate();
}