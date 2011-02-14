#ifndef _UART_DEBUG_H
#define _UART_DEBUG_H

#include <inttypes.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>


#define UART_BUFFER_SIZE	128
#define UART_BUFFER_MASK	(UART_BUFFER_SIZE-1)

void uart_debug_init(void);
void uart_debug_prints(const uint8_t * pstr, ...);


#endif

