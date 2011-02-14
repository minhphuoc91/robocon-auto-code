#include "core2_util.h"

//debug
void debug_set(const uint8_t channel,const uint8_t duty,const uint8_t count) {
	uint8_t data[4] = { CMD_debug_set,channel,duty,count};
	comm_write_real(data,4);

}


//lcd
void lcd_gotoxy(const uint8_t x,const uint8_t y) 
{
	uint8_t data[3] = {CMD_lcd_gotoxy,x,y};
	comm_write_real(data,3);

}

void lcd_prints(const uint8_t * pstr, ...) 
{
	va_list	arglist;
	uint8_t buf[22];

	va_start(arglist,pstr);
	vsnprintf(&buf[2],20,pstr,arglist);
	va_end(arglist);
	buf[0] = CMD_lcd_printData;
	buf[1] = strlen(&buf[2]);
	comm_write_real(buf,buf[1] + 2);


}

void lcd_printxys(const uint8_t x, const uint8_t y, const uint8_t * pstr,...) 
{
	va_list	arglist;
	uint8_t buf[24];

	va_start(arglist,pstr);
	vsnprintf(&buf[4],20,pstr,arglist);
	va_end(arglist);
	buf[0] = CMD_lcd_printDataxy;
	buf[1] = strlen(&buf[4]);
	buf[2] = x;
	buf[3] = y;
	comm_write_real(buf,buf[1] + 4);
}

void lcd_putxys(const uint8_t x, const uint8_t y, const uint8_t * data) 
{
	uint8_t buf[24];

	buf[0] = CMD_lcd_printDataxy;
	buf[1] = strlen(data);
	buf[2] = x;
	buf[3] = y;
	memcpy(&buf[4],data,(buf[1] > 20) ? 20 : buf[1]);
	comm_write_real(buf,buf[1] + 4);
}

void lcd_putxysf(const uint8_t x, const uint8_t y,PGM_P pstr) 
{
	uint8_t buf[24];

	buf[0] = CMD_lcd_printDataxy;
	buf[1] = strlen_P(pstr);
	buf[2] = x;
	buf[3] = y;
	memcpy_P(&buf[4],pstr,(buf[1] > 20) ? 20 : buf[1]);
	comm_write_real(buf,buf[1] + 4);
}
	

void lcd_printsf(PGM_P pstr, ...) 
{
	va_list	arglist;
	uint8_t buf[22];

	va_start(arglist,pstr);
	vsnprintf_P(&buf[2],20,pstr,arglist);
	va_end(arglist);
	buf[0] = CMD_lcd_printData;
	buf[1] = strlen(&buf[2]);
	comm_write_real(buf,buf[1] + 2);
}

void lcd_printxysf(const uint8_t x,const uint8_t y, PGM_P pstr, ...)
{
	va_list	arglist;
	uint8_t buf[24];

	va_start(arglist,pstr);
	vsnprintf_P(&buf[4],20,pstr,arglist);
	va_end(arglist);
	buf[0] = CMD_lcd_printDataxy;
	buf[1] = strlen_P(&buf[4]);
	buf[2] = x;
	buf[3] = y;
	comm_write_real(buf,buf[1] + 4);
}

void lcd_puts(const uint8_t * data) 
{
	uint8_t buf[22];
	buf[0] = CMD_lcd_printData;
	buf[1] = strlen(data);
	memcpy(&buf[2],data,(buf[1] > 20) ? 20 : buf[1]);
	comm_write_real(buf,buf[1] + 2);

}

void lcd_putsf(PGM_P data) 
{
	uint8_t buf[22];
	buf[0] = CMD_lcd_printData;
	buf[1] = strlen_P(data);
	memcpy_P(&buf[2],data,(buf[1] > 20) ? 20 : buf[1]);
	comm_write_real(buf,buf[1] + 2);


}

void lcd_clear(void) 
{
	uint8_t data[2] = {CMD_lcd_clear,123};
	comm_write_real(data,2);

}

void lcd_putchar(const uint8_t c) 
{
	uint8_t data[2] = {CMD_lcd_putchar,c};
	comm_write_real(data,2);
}


// psc
struct _psc psc={{0,0},{0,0},{0,0},0x80,0x80,0x80,0x80,0,0};
uint8_t psc_flag = FALSE;

//Reflash ps controller data
void psc_report(const uint8_t * data) 
{
	psc.data[PSC_BUT1] = (uint8_t)~data[1];
	psc.data[PSC_BUT2] = (uint8_t)~data[2];
	
	psc.up[PSC_BUT1] = data[1] & (data[1] ^ data[3]);
	psc.up[PSC_BUT2] = data[2] & (data[2] ^ data[4]);
	
	psc.down[PSC_BUT1] = data[3] & (data[1] ^ data[3]);
	psc.down[PSC_BUT2] = data[4] & (data[2] ^ data[4]);

	
	psc.joyrightx = data[5];
	psc.joyrighty = data[6];
	
	psc.joyleftx = data[7];
	psc.joylefty = data[8];
	
	psc.mode = data[9];
	psc.old_mode = data[10];
	
	psc_flag = TRUE;

}

//servo
void servo_control(const uint8_t id,const uint8_t angle) 
{
	uint8_t data[3] = {CMD_servo_control,id,angle};
	comm_write_real(data,3);
}

//pwm
void pwm_control(const uint8_t motor,const uint8_t speed,const uint8_t direction) 
{
	uint8_t data[4] = {CMD_pwm_control,motor,speed,direction};
	comm_write_real(data,4);
}


//line sensor
uint8_t sensor_data = 0;
int8_t sensor_mean = 0;
uint8_t sensor_flag = 1;

void linesensor_report(const uint8_t* data)
{
	if (sensor_flag == SENS_RAW) sensor_data = data[1];
	else 
	{
		sensor_mean = 0;
		sensor_mean |= data[1];
	}
}

void linesensor_set(const uint8_t _flag)
{
	sensor_flag = _flag;
	uint8_t data[2] = {CMD_linesensor_set,sensor_flag};
	comm_write_real(data,2);
}

uint8_t linesensor_report_status(void)
{
	return sensor_flag;
}

uint8_t linesensor_report_data(void)
{
	if (sensor_flag ==SENS_RAW) return sensor_data;
	else return 255;
}

int8_t linesensor_report_mean(void)
{
	if (sensor_flag ==SENS_MEAN) return sensor_mean;
	else return 127;	
}