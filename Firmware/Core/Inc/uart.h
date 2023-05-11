#ifndef UART_H
#define UART_H

/* C Library Includes */
#include <stdint.h>

void uart_transmit(uint8_t* data, uint16_t numBytes);

void uart_init(void);

#endif // UART_H