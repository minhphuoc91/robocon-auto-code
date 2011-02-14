#include "uart_debug.h"


static uint8_t tx_buf[UART_BUFFER_SIZE];
static uint8_t tx_head = 0;
static uint8_t tx_tail = 0;


ISR(USART0_UDRE_vect) {
	if(tx_head != tx_tail) {
		UDR0 = tx_buf[tx_tail];
		tx_tail = (tx_tail + 1) & UART_BUFFER_MASK;
	
	}else{
		UCSR0B &= ~_BV(UDRIE0);

	}

}

void uart_debug_init(void) {
    // 115200 8-N-1
    UBRR0H = 0;
    UBRR0L = 7;
    UCSR0B = _BV(TXEN0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
}


static void uart_debug_write_byte(const uint8_t data) {
	uint8_t tmp_head;
	tmp_head = (tx_head + 1) & UART_BUFFER_MASK;
	while(tmp_head==tx_tail);
	
	tx_buf[tmp_head] = data;
	tx_head = tmp_head;
	
	UCSR0B |= _BV(UDRIE0);

}

void uart_debug_prints(const uint8_t * pstr, ...)
{

	va_list arglist;
	uint8_t buf[40], *fp;
	
	va_start(arglist, pstr);
	vsprintf(buf, pstr, arglist);
	va_end(arglist);
	
	fp = buf;
	while (*fp)
		uart_debug_write_byte(*fp++);	

}

void uart_debug_printsf(PGM_P pstr, ...)
{
	va_list arglist;
	uint8_t buf[40], *fp;
	
	va_start(arglist, pstr);
	vsprintf_P(buf, pstr, arglist);
	va_end(arglist);
	
	fp = buf;
	while (*fp)
		uart_debug_write_byte(*fp++);	
}