#ifndef _MAIN_H
#define _MAIN_H

#define F_CPU 14745600

#define CORE1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <string.h>
#include <stdlib.h>


#include "comm.h"
#include "uart_debug.h"
#include "delay.h"
#include "avr_protocol.h"
#include "ping.h"
#include "core2_util.h"
#include "lm629.h"
#include "adc.h"
#include "uart_comm.h"
#include "PID.h"
#include "tri_val.h"

#ifndef TRUE
#define TRUE	1
#endif

#ifndef FALSE
#define FALSE	0
#endif

void menu_linesensor_test(void);
void menu_manualControl(void);

#endif