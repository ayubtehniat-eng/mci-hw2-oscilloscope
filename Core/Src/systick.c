#include "stm32f3xx.h"
#include "systick.h"

static volatile uint32_t s_ms = 0;          /* ms counter, incremented in the ISR */
static void (*s_cb)(void) = 0;              /* called once per second */

void Systick_Init1ms(void (*every_second_cb)(void))
{
    s_cb = every_second_cb;
    /* CMSIS SysTick_Config(): SYST_RVR = HCLK/1000 - 1 (reload for 1 ms),
       SYST_CVR = 0, SYST_CSR.CLKSOURCE = 1 (HCLK), TICKINT = 1, ENABLE = 1.
       It also sets the SysTick priority to the lowest level. */
    SysTick_Config(SystemCoreClock / 1000U);
}

uint32_t Systick_Ms(void)
{
    return s_ms;
}

/* Name comes from the startup file's vector table */
void SysTick_Handler(void)
{
    s_ms++;
    if ((s_ms % 1000U) == 0U && s_cb != 0)
    {
        s_cb();
    }
}