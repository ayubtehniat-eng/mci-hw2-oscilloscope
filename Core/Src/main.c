#include "stm32f3xx.h"
#include "clock.h"
#include "systick.h"
#include "tim2.h"
#include "adc.h"
#include "uart.h"

/* Watch these in the debugger (Live Expressions) */
volatile uint32_t samples_last_second = 0;  /* must read 10000 every second after the first */
volatile uint32_t overruns_total      = 0;  /* must stay 0: no missed sample */
volatile uint32_t dropped_total       = 0;  /* must stay 0: ADC ring buffer never full */
volatile uint32_t uart_dropped_total  = 0;  /* must stay 0: UART buffer never full */
volatile uint16_t last_sample         = 0;  /* raw 12-bit value, 0..4095 (1.65 V is about 2048) */

static uint32_t s_prev_count = 0;

/* Called by the SysTick interrupt exactly once every 1000 ms */
static void OnEverySecond(void)
{
    uint32_t now = Adc_GetSampleCount();
    samples_last_second = now - s_prev_count;
    s_prev_count = now;
    overruns_total     = Adc_GetOverrunCount();
    dropped_total      = Adc_GetDroppedCount();
    uart_dropped_total = Uart_GetDropped();
}

int main(void)
{
    Clock_Init48MHz();               /* 48 MHz system clock */
    Systick_Init1ms(OnEverySecond);  /* 1 ms tick + once-per-second callback */
    Uart_Init();                     /* USART1 ready to send */
    Tim2_Init10kHz();                /* 10 kHz TRGO, not yet running */
    Adc_Init();                      /* ADC armed, waiting for TIM2 TRGO */
    Tim2_Start();                    /* sampling begins here */

    while (1)
    {
        uint16_t s;
        while (Adc_Pop(&s))          /* drain the ADC ring buffer into the UART */
        {
            last_sample = s;
            Uart_SendSample(s);
        }
    }
}