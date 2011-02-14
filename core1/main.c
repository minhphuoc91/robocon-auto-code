#include "main.h"



//---------------------------------------------------
//#define SERVO ANGLE 
//---------------------------------------------------
#define SERVO_OPEN 105
#define SERVO_CLOSE 180

uint8_t online[COMM_CHANNEL];
volatile uint8_t ticks=0;

static void tick_init(void)
{
	TCCR0 = _BV(CS02) | _BV(CS01);
	TIMSK |= _BV(TOIE0);
}

ISR(TIMER0_OVF_vect)
 {
	static uint8_t landing_buf[64];

	TCCR0 = 0;
	TCNT0 = 0xff - F_CPU/ 256 /256;
	
	ticks++;
	if(comm_fetch(landing_buf)!=255) 
	{
			void (*addr)(const uint8_t *)  = protocol_consumer(landing_buf[0]);
			if(addr!=NULL) 
			{
				addr(landing_buf);
			}	
	}
	if((ticks % 128)==0) 
	{
		ping_hello_core2(NULL);
		
		if(online[COMM_TWI] != 0) 
		{
			online[COMM_TWI]--;

			
			if(online[COMM_TWI] == 0) 
			{
				uart_debug_prints("core 2 dead");
			}
		
		}
	}
	
	if (ADCSRA & _BV(ADIF))
	{
		adc_dispatch();
	}
	

	TCCR0 = _BV(CS02) | _BV(CS01);
}


int main(void) 
{
	_delay_ms(20);
	online[COMM_TWI] = 100;
	
	sei();
	adc_init();
	tick_init();
	uart_debug_init();
	uart_comm_init();
	comm_init();
	lm629_init();
	
	lm629_set_filter(0,85,0,1024,2048);
	lm629_set_filter(1,85,0,1024,2048);
	lm629_set_filter(2,85,0,1024,2048);
	lm629_set_filter(3,85,0,1024,2048);

	position_status_set(UART_POS_ENABLE,0,0,0);

	lm629_acceleration(0,1000);
	lm629_acceleration(1,1000);
	lm629_acceleration(2,1000);
	lm629_acceleration(3,1000);
	lm629_define_home (0);
	lm629_define_home (1);
	lm629_define_home (2);
	lm629_define_home (3);
	lm629_stop_abruplty(0);
	lm629_stop_abruplty(1);
	lm629_stop_abruplty(2);
	lm629_stop_abruplty(3);
	
	for(int i = 0;i<600; i++) _delay_ms(10);

	int32_t v[2];
	v[0] = 1500000;
	v[1] = 1500000;

	
	
	PID_set_target_XYAngle(0,2000,0);
	while(!PID_position_done()){
		PID_calculation_u();
		PID_velocity_twin_start(1500000,1500000);
		_delay_ms(10);
	}	
	
	PID_path_arc(3000,2000,0,2000,0);
	PID_arc_speed(v);
	while(!PID_arc_done()){
		PID_calculation_u();
		PID_velocity_twin_start(v[0],v[1]);
		_delay_ms(10);
	}

	
	PID_set_target_XYAngle(3000,0,180);
	while(!PID_position_done()){
		PID_calculation_u();
		PID_velocity_twin_start(1500000,1500000);
		_delay_ms(10);
	}
	
	/*
	PID_set_target_XYAngle_back(0,-3000,0,0,0);
	while(!PID_position_done()){
		PID_calculation_u();
		PID_velocity_twin_start(-1000000,-1000000);
		_delay_ms(10);
	}
	*/
	lm629_stop_abruplty(0);
	lm629_stop_abruplty(1);	

	while(1);

	return 0;


}


