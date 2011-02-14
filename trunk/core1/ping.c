#include "ping.h"


void ping_hello_core1(const uint8_t * data) 
{
	online[COMM_TWI] = 100;

}


void ping_hello_core2(const uint8_t * data) 
{	
	uint8_t _data[2] = {CMD_ping_hello_core2,online[COMM_TWI]};
	comm_write_real(_data,(uint8_t) pgm_read_byte(&protocol_map[CMD_ping_hello_core2].len) + 1);
}
