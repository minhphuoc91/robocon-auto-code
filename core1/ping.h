#ifndef _PING_H
#define _PING_H

#include <inttypes.h>
#include "main.h"
#include "avr_protocol.h"
#include "comm.h"


void ping_hello_core1(const uint8_t * data);
void ping_hello_core2(const uint8_t * data);


#endif