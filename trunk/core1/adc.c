#include "adc.h"

/******************************************************************************
  Declaration
******************************************************************************/
#define ADC_MAX_JOB		8		// In 2^n number, otherwise the mask would not work
#define ADC_QUEUE_MASK ( ADC_MAX_JOB - 1 )

// buffer to store job
static uint8_t job_mux[ADC_MAX_JOB];	// ADMUX
static void* job_consumer[ADC_MAX_JOB];	// Consumer function pointer
static uint8_t job_tag[ADC_MAX_JOB];	// Tag (For recording purposing)

// pointer to head of ringbuffer, points to the last entry in buffer
static uint8_t adc_head = 0, adc_tail = 0;
static uint8_t converting = 0;

uint16_t adc_value = 0;

void adc_execute (void);

/******************************************************************************
  Procedures
******************************************************************************/

// ADC Initialization
void adc_init (void)
{
	// Enable ADC, Clear interrupt flag, Prescalar = 128
	ADCSRA = _BV(ADEN) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
}

// Executing the jobs[adc_tail]
void adc_execute (void)
{
	ADMUX = job_mux[adc_tail];
	ADCSRA |= _BV(ADSC);
}

// ADC is finished converting
// now execute the next one and call the consumer
void adc_dispatch (void)
{
	if (adc_head == adc_tail) return;
	
	void (*consumer)(uint8_t) = job_consumer[adc_tail];
	uint8_t tag = job_tag[adc_tail];
	adc_tail = (adc_tail + 1) & ADC_QUEUE_MASK;
	// ^^^ 10 bits 
	adc_value = ADC;
	ADCSRA |= _BV(ADIF);
	if (adc_head != adc_tail) adc_execute(); else converting = 0;
	
	consumer(tag);
}

// Queue an ADC Job
void adc_add (uint8_t mux, void(* consumer)(uint8_t), uint8_t tag)
{
	while(((adc_head + 1) & ADC_QUEUE_MASK)==adc_tail) {
			if (ADCSRA & _BV(ADIF)) 
			{
				adc_dispatch();
			}
	};
	
	job_mux[adc_head] = mux;
	job_consumer[adc_head] = consumer;
	job_tag[adc_head] = tag;
	
	adc_head = (adc_head + 1) & ADC_QUEUE_MASK;
	
	if (!converting)
	{
		converting = 1;
		adc_execute();
	}
}
