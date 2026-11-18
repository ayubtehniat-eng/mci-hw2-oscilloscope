#ifndef UART_H
#define UART_H

#include <stdint.h>

#define UART_TX_BUF_SIZE  2048U

void     Uart_Init(void);
void     Uart_SendSample(uint16_t sample);
uint32_t Uart_GetDropped(void);

#endif /* UART_H */