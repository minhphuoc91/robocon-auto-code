#include "uart_comm.h"

#define X_OFF 0
#define Y_OFF 0

volatile int16_t tmp_data[3];
volatile uint8_t tmp_cmd = 0;

volatile int16_t pos_X = X_OFF;
volatile int16_t pos_Y = Y_OFF;
volatile int16_t real_angle = 362;
volatile uint8_t slide_flag = 0;

volatile uint8_t state = 0;
volatile uint8_t rx_data = 0;

void uart_comm_init(void)
{
    UBRR1H = 0;
    UBRR1L = 15;
    UCSR1B = _BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1);
    UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);
}

void uart_send_u8(uint8_t data)
{
    while (!(UCSR1A & _BV(UDRE1)));
    UDR1 = data;
    while (!(UCSR1A & _BV(TXC1)));
    UCSR1A |= _BV(TXC1);
}

void uart_send_16(int16_t data)
{
    uint8_t tmp = 0;
    //uint16_t _data = 0;
    //_data |= data;
    while (!(UCSR1A & _BV(UDRE1)));

    tmp |= data >> 8;
    UDR1 = tmp;//data >> 8;				//Send MSB;
    while (!(UCSR1A & _BV(UDRE1)));
    UCSR1A |= _BV(TXC1);

    tmp = 0;
    tmp |= data;
    UDR1 = tmp;//data;				//Send LSB;
    while (!(UCSR1A & _BV(UDRE1)));
    UCSR1A |= _BV(TXC1);
}

void position_set(int16_t angle, int16_t x, int16_t y)
{
    uart_send_u8(UART_POS_SET);
    my_delay_ms(20);
    uart_send_16(angle);
    uart_send_16(x);
    uart_send_16(y);
    uart_send_u8(0);
    real_angle = 361;
    for (uint8_t i = 0; i< 5; i++)
        my_delay_ms(10);
    while (position_get_status() != 1);
}

void position_status_set(uint8_t status,int16_t angle, int16_t x, int16_t y)
{
    uart_send_u8(status);
    my_delay_ms(20);
    uart_send_16(360-angle);
    uart_send_16(x);
    uart_send_16(y);
    uart_send_u8(0);
    real_angle = 361;
}

void position_cal(void)
{
    uart_send_u8(UART_POS_CAL);
    my_delay_ms(20);
    uart_send_16(361);
    uart_send_16(9999);
    uart_send_16(9999);
    uart_send_u8(0);
    for (uint8_t i = 0; i< 100; i++)
        my_delay_ms(10);
    real_angle = 361;
    while (position_get_status() != 1);
}

int16_t position_get_X(void)
{
    return -pos_X;
}

int16_t position_get_Y(void)
{
    return -pos_Y;
}

int16_t position_get_angle(void)
{
	/*
	if (real_angle <= 360)
		return (360-((real_angle)%361))%360;
		
	else*/
	return real_angle;
}

uint8_t position_get_flag(void)
{
    return slide_flag;
}

uint8_t position_get_status(void)
{
    if (real_angle <= 360) return 1;
    else if (real_angle == 361) return 2;
    else return 0;
}

ISR (USART1_RX_vect)
{
    rx_data = UDR1;

    switch (state)
    {
    case 0:
        tmp_cmd = rx_data;
        if (tmp_cmd == UART_POS_SEND)
        {
            tmp_data[0]=tmp_data[1]=tmp_data[2]=0;
            state++;
        }
        break;

    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
        if (state % 2)
        {
            tmp_data[(state-1)/2] |= (uint16_t)rx_data;
            tmp_data[(state-1)/2] <<= 8;
        }
        else
        {
            tmp_data[(state-1)/2] |= (uint16_t)rx_data;
        }

        state++;
        break;

    case 7:
        state = 0;
        //Assign Variables///////////////////////////
        //if (tmp_data[0] < 500 && tmp_data[0] >= 0)
        real_angle = tmp_data[0];

        //if (abs(tmp_data[1] - pos_X) < 85)
        pos_X = tmp_data[1];
        //if (abs(tmp_data[2] - pos_Y) < 85)
        pos_Y = tmp_data[2];

        /*if (tmp_data[1] < 0 || tmp_data[1] > 800 || tmp_data[2] < 0 || tmp_data[2] > 950)
        {
        	lm629_zero_drive(0);
        	lm629_zero_drive(1);
        	lm629_zero_drive(2);
        	lm629_zero_drive(3);
        	glcd192x64Draw(FILLWHITE,0,26,100,30);
        	glcd192x64Printsxy(0,26,"Angle: %d",real_angle);
        	glcd192x64Printsxy(0,36,"X:%d,Y:%d ERR",pos_X,pos_Y);
        	while(1);
        }*/
        slide_flag = rx_data;
        break;

    }
}
