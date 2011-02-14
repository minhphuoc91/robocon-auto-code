#ifndef _CORE2_UTIL
#define _CORE2_UTIL


#include <avr/io.h>
#include <inttypes.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "comm.h"
#include "avr_protocol.h"


//debug
#define DEBUG_BUZZER 0
#define DEBUG_RED 1
#define DEBUG_GREEN 2
#define DEBUG_BLUE 3
void debug_set(const uint8_t channel,const uint8_t duty,const uint8_t count);


//lcd
void lcd_clear(void);
void lcd_prints(const uint8_t * pstr, ...);
void lcd_printsf(PGM_P pstr, ...);
void lcd_puts(const uint8_t * data);
void lcd_putsf(PGM_P pstr);
void lcd_printxys(const uint8_t x, const uint8_t y, const uint8_t * pstr,...);
void lcd_printxysf(const uint8_t x, const uint8_t y, PGM_P pstr, ...);
void lcd_putxys(const uint8_t x ,const uint8_t y,const uint8_t * data);
void lcd_putxysf(const uint8_t x ,const uint8_t y,PGM_P pstr);
void lcd_gotoxy(const uint8_t x,const uint8_t y); 
void lcd_putchar(const uint8_t c);


//psc
// First Byte: psc_data/down/up[0]
#define PSC_BUT1	0
#define PSC_SELECT	0x01
#define PSC_START	0x08
#define PSC_UP		0x10
#define PSC_DOWN	0x40
#define PSC_LEFT	0x80
#define PSC_RIGHT	0x20
#define PSC_JOYLEFT		0x02
#define PSC_JOYRIGHT	0x04
// Second Byte: psc_data/down/up[1]
#define PSC_BUT2	1
#define PSC_L1	0x04
#define PSC_L2	0x01
#define PSC_R1	0x08
#define PSC_R2	0x02
#define PSC_A	0x10
#define PSC_O	0x20
#define PSC_X	0x40
#define PSC_T	0x80
struct _psc
{
	uint16_t data[2];
	uint16_t up[2];
	uint16_t down[2];
	uint8_t joyleftx;
	uint8_t joylefty;
	uint8_t joyrightx;
	uint8_t joyrighty;
	uint8_t mode;
	uint8_t old_mode;
};
extern struct _psc psc;
extern uint8_t psc_flag;
void psc_report(const uint8_t * data);


//servo
void servo_control(const uint8_t id,const uint8_t angle);


//pwm
void pwm_control(const uint8_t motor,const uint8_t speed,const uint8_t direction);


//line sensor
#define SENS_DISABLE	0
#define SENS_RAW		1
#define SENS_MEAN		2

void linesensor_report(const uint8_t* data);
void linesensor_set(const uint8_t _flag);
uint8_t linesensor_report_status(void);
uint8_t linesensor_report_data(void);
int8_t linesensor_report_mean(void);

	
/*
//battery
#define BATTERY_RAW		0
#define BATTERY_LOGIC	1
#define BATTERY_RAIL1	2
#define BATTERY_RAIL2	3
extern uint8_t battery_refresh;
extern const PROGMEM uint8_t battery_limit[4][4];
extern uint8_t battery_voltage[4];
void battery_report(const uint8_t * data);
*/

#endif