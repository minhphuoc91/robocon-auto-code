/******************************************************************************
Hong Kong Univerisity Of Science and Technology, Copyright (c) 2005
Robocon 2005 Team 1

$Id: uart.h 1 2005-02-06 10:40:06Z sam $

******************************************************************************/

#ifndef _UART_H_
#define _UART_H_


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <string.h>
#include "delay.h"
#include "avr_protocol.h"




/******************************************************************************
  Variable Declaration
******************************************************************************/
#define	COMM_BUFFER_SIZE 128 // In 2^n number, otherwise the mask would not work
#define COMM_BUFFER_MASK ( COMM_BUFFER_SIZE - 1 )

#define TWI_OTHER		2
#define	TWI_ID			1
#define COMM_CHANNEL 	1
#define COMM_TWI		0
#define COMM_UART0		1		//deprecated
#define COMM_UART1		2		//deprecated

extern uint8_t online[COMM_CHANNEL];

/******************************************************************************
  Procedure Declaration
******************************************************************************/
// UART Initialization
void comm_init (void);

// Transfer one command packet from RX buffer to landing buffer
// return the actual bytes transfered, or 0 if no avaiable packet
//   (a packet have at least 1 byte long)
//
// Pre:  Assume buf is long enough to hold the longest packet
uint8_t comm_fetch (uint8_t * buf);


// Communication Channel
// 0: UART0
// 1: UART1
// 2: I2C
//
// To send to UART channel, simply use comm_write(_char) function to write the
//   data wish to be sent
// To send to I2C channel, application must write the Address (u8) and Count (u8)
//   using comm_write(_char) first. The behaviour of not obeying this is undef.
// For I2C channel, Each call must exactly contain one packet.
//   One packet: Address, Size, [Data 0]...[Data Size]

// Send data to corresponding communication channel
// The operation might block if the buffer is full
//
// Pre:  Assume data is as least as long as count
//
// comm_write_real is guarantee to return non-zero only when data is successfully buffered
uint8_t comm_write_real (const uint8_t * data, const uint8_t count);
inline void comm_write_byte(const uint8_t data);






#endif
