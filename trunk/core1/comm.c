#include "comm.h"

/******************************************************************************
  Declaration
******************************************************************************/

// Channel 0: I2C
// Channel 1: UART 0
// Channel 2: UART 1

// buffer to store data
static uint8_t tx_buf[COMM_CHANNEL][COMM_BUFFER_SIZE];
static uint8_t rx_buf[COMM_CHANNEL][COMM_BUFFER_SIZE];

// pointer to head of ringbuffer, points to the last entry in buffer
static volatile uint8_t tx_head[COMM_CHANNEL];
// pointer to head of ringbuffer, points to the first empty buffer space
static volatile uint8_t rx_head[COMM_CHANNEL];
// pointer to the tail of ringbuffer, points to last byte sent
static volatile uint8_t tx_tail[COMM_CHANNEL];
// pointer to the tail of ringbuffer, points to first unread byte
static volatile uint8_t rx_tail[COMM_CHANNEL];

static volatile uint8_t rx_len[COMM_CHANNEL];

// static uint8_t ch2_prestart_buffer = 0;
static uint8_t twi_remaining = 0;
static volatile uint8_t twi_free = 0;


/******************************************************************************
  Procedures
******************************************************************************/


void comm_init (void) {
	uint8_t i;
	for(i=0;i<COMM_CHANNEL;i++) {
		tx_head[i] = tx_tail[i] = rx_head[i] = rx_tail[i] = rx_len[i] = 0;
		memset(tx_buf[i],0,COMM_BUFFER_SIZE);
		memset(rx_buf[i],0,COMM_BUFFER_SIZE);
	}
	

	
	#if defined(TWPS0)
		/* Disable the prescalar */
		TWSR = 0;
	#endif	
	TWBR = (F_CPU / 150000UL - 16) / 2;
	TWAR = TWI_ID << 1;
	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
	twi_free = 1;
	
	// Make SCL SDA as input pins and no pullup
	DDRD &= ~(_BV(0) | _BV(1));
	PORTD &= ~(_BV(0) | _BV(1));
	
	
}


// Transfer one command packet from RX buffer to landing buffer
// return the actual bytes transfered, or 0xFF if no avaiable packet
// 
// Pre:  Assume buf is long enough to hold the longest packet
// Post: Return no packet avaiable or a complete packet packet is copied to landing buffer
uint8_t comm_fetch (uint8_t * buf) {
	uint8_t i;	
	// arm_protocol at least return 0, so if rx_len[*]==0 it never true
	memset(buf,0,64);
	for (i = 0; i < COMM_CHANNEL; i++)
	{
		//if (rx_tail[i] == rx_head[i]) continue;
		// ^^^ len means the length of received chars.
		uint8_t _rx_len = rx_len[i];
		uint8_t len = protocol_length(rx_buf[i][rx_tail[i]]);
		if (_rx_len > len)
		{
			// lcd only
/*			if(rx_buf[i][rx_tail[i]] == CMD_lcd_printData) {
				if(_rx_len >= len + rx_buf[i][rx_tail[i] + 1]) 
				{
					len += rx_buf[i][rx_tail[i]+1];
				
				}else
				{
					return 255;
				
				}
			}*/
			
			

		    for (uint8_t j = 0; j <= len; j++) {
				*buf++ = rx_buf[i][rx_tail[i]];
				rx_tail[i] = ( rx_tail[i] + 1 ) & COMM_BUFFER_MASK;
		    }
			
			uint8_t t = SREG;				// ^^^ SREG : status register
			cli();							// ^^^ clear all interrupt
			rx_len[i] -= len + 1;			// ^^^ restore the length
			SREG = t;
			
			return i;
		}
	}
	return 255;
}


// Buffer the data to the txN buffer
//
// Pre:  comm_init() is called
// Post: returns # of bytes of data actually buffered.
//       either take all or take none, in case of buffer overflow.
uint8_t comm_write_real (const uint8_t * data,const uint8_t count)
{	
	// If we got nothing to write, do nothing
	if (count == 0) return 0;
	
	uint8_t i;
	cli();
	uint8_t tmphead = tx_head[COMM_TWI];
	
	tmphead = ( tmphead + 1 ) & COMM_BUFFER_MASK;
	if ( tmphead == tx_tail[COMM_TWI] ) return 0;
	tx_buf[COMM_TWI][tmphead] = TWI_OTHER;
	tmphead = ( tmphead + 1 ) & COMM_BUFFER_MASK;
	if ( tmphead == tx_tail[COMM_TWI] ) return 0;
	tx_buf[COMM_TWI][tmphead] = count;
			
	for (i = 0; i < count; i++) {
		/* calculate buffer index */
		tmphead = ( tmphead + 1 ) & COMM_BUFFER_MASK;
		if ( tmphead == tx_tail[COMM_TWI] ) {
			// Out of buffer space
			// Send nothings
			return 0;
		}
		tx_buf[COMM_TWI][tmphead] = data[i]; /* store data in buffer */
	}
	tx_head[COMM_TWI] = tmphead; /* store new index */
	sei();
		
	// Trigger sending action
	if (twi_free != 0)
	{
		twi_free = 0;
		TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);
	}

	return i;

}

inline void comm_write_byte(const uint8_t data) {
	while(comm_write_real(&data,1)==0);

};



ISR(TWI_vect) 
{
	uint8_t status = TW_STATUS;
	static uint8_t tmptail = 0;
	static uint8_t retry = 3;
	
	if (status == TW_START || status == TW_REP_START) // Start Condition Sent
	{
		if (tx_head[COMM_TWI] != tx_tail[COMM_TWI])
		{
			tmptail = ( tx_tail[COMM_TWI] + 1 ) & COMM_BUFFER_MASK;
			TWDR = tx_buf[COMM_TWI][tmptail] << 1; // SLA+W			
			tmptail = ( tmptail + 1 ) & COMM_BUFFER_MASK;
			twi_remaining = tx_buf[COMM_TWI][tmptail];
			TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
		} else
		{
			// Shouldn't reach here. Try to release the control of the bus
			// because have no data to send
			TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
			twi_free = 1;
		}
	} else if (status == TW_MT_SLA_ACK || status == TW_MT_DATA_ACK) // SLA+W/Data sent, ACK
	{		
		if (twi_remaining > 0)
		{
			tmptail = ( tmptail + 1 ) & COMM_BUFFER_MASK;
			twi_remaining--;
			TWDR = tx_buf[COMM_TWI][tmptail]; // Data		
			TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
		} else
		{
			retry = 3;
			tx_tail[COMM_TWI] = tmptail;
			// No more data in this packet
			if (tx_head[COMM_TWI] != tx_tail[COMM_TWI])
			{
				// All end? No, so restart.
				TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
			} else
			{				
				// All end? Yes, stop.
				TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
				twi_free = 1;
			}
		}
	} else if (status == TW_MT_DATA_NACK || status == TW_MT_SLA_NACK) // SLA+W/Data sent, NO ACK
	{
		if (--retry)
		{
			TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
		}
		else
		{
			retry = 3;
			tx_tail[COMM_TWI] = ( tmptail  + twi_remaining ) & COMM_BUFFER_MASK;
			// No more data in this packet
			if (tx_head[COMM_TWI] != tx_tail[COMM_TWI])
			{
				// All end? No, so restart.
				TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
			} else
			{				
				// All end? Yes, stop.
				TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
				twi_free = 1;
			}
		}
	} else if (status == TW_SR_SLA_ACK || status == TW_SR_ARB_LOST_SLA_ACK || 
					status == TW_SR_GCALL_ACK || status == TW_SR_ARB_LOST_GCALL_ACK )
	{
		twi_free = 0;
		// Being addressed by SLA+W, Due to normal or arbitration lost
		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
	} else if (status == TW_SR_DATA_ACK || status == TW_SR_DATA_NACK  || 
				status == TW_SR_GCALL_DATA_ACK  || status == TW_SR_GCALL_DATA_NACK) // Data received
	{
		twi_free = 0;
	    uint8_t data;
	    uint8_t tmphead;
	
	    data = TWDR; /* read the received data */
		
	    /* calculate buffer index */
	    tmphead = rx_head[COMM_TWI];
	    
	    /* store received data in buffer */
	    rx_buf[COMM_TWI][tmphead] = data;
	    rx_head[COMM_TWI] = (tmphead + 1) & COMM_BUFFER_MASK; /* store new index */
    	
    	rx_len[COMM_TWI]++;
    	
		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
	} else if (status == 	TW_SR_STOP || status == TW_BUS_ERROR) // Stop / Repeated Start, Illegal Status
	{
		if (tx_head[COMM_TWI] != tx_tail[COMM_TWI])
		{
			// All end? No, so start.
			TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
			twi_free = 0;
		} else
		{
			// All end? Yes, stop.
			TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
			twi_free = 1;
		}
	} else if (status == TW_MT_ARB_LOST ) // Arbitration lost in SLA+W
	{
		TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
		twi_free = 0;
	} else
	{
		// twi_free = 1;
		// Shouldn't reach here
		PORTB &= ~_BV(0 + 4);
		while(1);
	}
}
