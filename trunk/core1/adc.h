#ifndef _ADC_H
#define _ADC_H

#include <avr/io.h>
#include <inttypes.h>

/******************************************************************************
  Variable Declaration
******************************************************************************/
// Result would be stored here
extern uint16_t adc_value;

/******************************************************************************
  Procedure Declaration
******************************************************************************/
// ADC Initialization
void adc_init (void);

// When ADIF is on, this is called
void adc_dispatch (void);

// Queue an ADC Job
void adc_add (uint8_t mux, void(* consumer)(uint8_t), uint8_t tag);

#endif