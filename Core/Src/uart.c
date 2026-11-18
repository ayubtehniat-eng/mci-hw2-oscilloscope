#include "stm32f3xx.h"
#include "uart.h"

#define TX_BUF_MASK  (UART_TX_BUF_SIZE - 1U)

/* TX ring buffer: main loop writes s_head, the USART1 ISR writes s_tail */
static volatile uint8_t  s_buf[UART_TX_BUF_SIZE];
static volatile uint32_t s_head = 0;
static volatile uint32_t s_tail = 0;
static volatile uint32_t s_dropped = 0;   /* samples thrown away because the buffer was full */

void Uart_Init(void)
{
    /* RCC_AHBENR.IOPCEN = 1 : clock GPIOC (PC4 = USART1_TX, wired to the ST-LINK virtual COM port) */
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    /* GPIOC_MODER.MODER4 = 10 : PC4 in alternate-function mode */
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER4) | GPIO_MODER_MODER4_1;
    /* GPIOC_AFRL.AFRL4 = 0111 : alternate function 7 = USART1_TX */
    GPIOC->AFR[0] = (GPIOC->AFR[0] & ~(0xFU << (4U * 4U))) | (7U << (4U * 4U));
    /* GPIOC_OSPEEDR.OSPEEDR4 = 11 : high speed output */
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4;

    /* RCC_APB2ENR.USART1EN = 1 : clock USART1 (PCLK2 = 48 MHz is its default kernel clock) */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* USART1_CR1.UE = 0 : peripheral off while configuring (reset value, cleared to be explicit) */
    USART1->CR1 &= ~USART_CR1_UE;
    /* USART1_CR1.M = 0 (8 data bits), PCE = 0 (no parity), OVER8 = 0 (oversampling by 16) */
    USART1->CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_OVER8);
    /* USART1_CR2.STOP = 00 : 1 stop bit  ->  8N1 */
    USART1->CR2 &= ~USART_CR2_STOP;
    /* USART1_BRR = 48 MHz / 230400 = 208.33 -> 208 (baud error -0.16 %) */
    USART1->BRR = 48000000U / 230400U;
    /* USART1_CR1.TE = 1 : transmitter enable */
    USART1->CR1 |= USART_CR1_TE;

    /* NVIC: lower priority than the ADC (0) so the UART can never delay a sample */
    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);

    /* USART1_CR1.UE = 1 : peripheral on */
    USART1->CR1 |= USART_CR1_UE;
}

/* Queue one byte. Caller has already checked there is room. */
static void Push(uint8_t b)
{
    s_buf[s_head] = b;
    s_head = (s_head + 1U) & TX_BUF_MASK;
}

void Uart_SendSample(uint16_t sample)
{
    uint32_t used = (s_head - s_tail) & TX_BUF_MASK;
    if ((UART_TX_BUF_SIZE - 1U - used) < 2U)   /* need room for both bytes, or send none */
    {
        s_dropped++;
        return;
    }

    /* Packet: first byte  = 1 0 h h h h h h   (bit 7 = 1 marks the start, h = sample bits 11..6)
               second byte = 0 0 l l l l l l   (bit 7 = 0, l = sample bits 5..0)
       The PC can find the start of a packet at any moment, so it recovers after a dropped byte. */
    Push((uint8_t)(0x80U | ((sample >> 6) & 0x3FU)));
    Push((uint8_t)(sample & 0x3FU));

    /* USART1_CR1.TXEIE = 1 : interrupt when the transmit register is empty (starts sending) */
    USART1->CR1 |= USART_CR1_TXEIE;
}

uint32_t Uart_GetDropped(void)
{
    return s_dropped;
}

/* Name comes from the startup file's vector table */
void USART1_IRQHandler(void)
{
    /* USART1_ISR.TXE = 1 : transmit data register is empty, we may write the next byte */
    if (((USART1->ISR & USART_ISR_TXE) != 0U) && ((USART1->CR1 & USART_CR1_TXEIE) != 0U))
    {
        if (s_tail != s_head)
        {
            USART1->TDR = s_buf[s_tail];   /* writing USART1_TDR clears TXE */
            s_tail = (s_tail + 1U) & TX_BUF_MASK;
        }
        else
        {
            /* nothing left: USART1_CR1.TXEIE = 0 stops the interrupt */
            USART1->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}
