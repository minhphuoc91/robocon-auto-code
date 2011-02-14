#ifndef _UART_COMM_H
#define _UART_COMM_H

#include <inttypes.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "core2_util.h"

#define NORMAL 0
#define LEFT 1
#define RIGHT 2

//For send
#define UART_POS_SEND 		0x01

//For receive
#define UART_POS_ENABLE		0x02
#define UART_POS_DISABLE	0x03
#define UART_POS_SET		0x04
#define UART_POS_CAL		0x05
#define LINE_SENS_ENABLE	0X06
#define LINE_SENS_DISABLE	0x07


void uart_comm_init(void);
void uart_send_u8(uint8_t data);
void uart_send_16(int16_t data);

void position_status_set(uint8_t status,int16_t angle, int16_t x, int16_t y);
void position_set(int16_t angle, int16_t x, int16_t y);
void position_cal(void);
uint8_t position_get_status(void);

int16_t position_get_X(void);
int16_t position_get_Y(void);
int16_t position_get_angle(void);
uint8_t position_get_flag(void);
#endif
