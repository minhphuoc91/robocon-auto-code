#include "avr_protocol.h"



PROGMEM const struct avr_protocol protocol_map[AVR_PROTOCOL_LEN] = 
{
{NULL,0},				//debug_set
{ping_hello_core1,1},	//ping_hello_core1
{ping_hello_core2,1},	//ping_hello_core2
{NULL,0},				//lcd_clear
{NULL,0},				//lcd_printData
{NULL,0},				//lcd_gotoxy
{NULL,0},				//lcd_putchar
{psc_report,10},		//psc_monitor
{NULL,0},				//lcd_printDataxy
{NULL,0},				//servo_control
{NULL,0},				//pwm_control
{NULL,0},
{linesensor_report,1},
{NULL,0}
};


uint8_t protocol_length(const uint8_t cmd) 
{
	if(cmd<AVR_PROTOCOL_LEN) 
	{
		return pgm_read_byte(&protocol_map[cmd].len);
	}
	return 0;

}

void * protocol_consumer(const uint8_t cmd) 
{
	if(cmd<AVR_PROTOCOL_LEN) 
	{
		return (void * ) pgm_read_word(&protocol_map[cmd].funcPtr);
	}
	return NULL;
}