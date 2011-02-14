#ifndef	_AVR_PROTOCOL_H
#define _AVR_PROTOCOL_H

#include <inttypes.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include "main.h"


#define CMD_debug_set			0 
#define CMD_ping_hello_core1	1
#define CMD_ping_hello_core2	2
#define CMD_lcd_clear			3
#define CMD_lcd_printData		4
#define CMD_lcd_gotoxy			5
#define CMD_lcd_putchar			6
#define CMD_psc_report			7
#define CMD_lcd_printDataxy		8
#define CMD_servo_control		9
#define CMD_pwm_control			10
#define CMD_lm629_chip_report	11
#define CMD_linesensor_report	12
#define CMD_linesensor_set		13

#define AVR_PROTOCOL_LEN	14





struct avr_protocol 
{
	void (*funcPtr)(const uint8_t * );
	uint8_t len;
};

uint8_t protocol_length(const uint8_t cmd);
void * protocol_consumer(const uint8_t cmd);

PROGMEM extern const struct avr_protocol protocol_map[AVR_PROTOCOL_LEN];



#endif
