/******************************************************************************
Hong Kong Univerisity Of Science and Technology, Copyright (c) 2005
Robocon 2005 Team 1

Fills the gap of AVR Libc 1.0.4 and 1.2.*

$Id: delay.h 1 2005-02-06 10:40:06Z sam $

******************************************************************************/

#ifndef _DELAY_H_
#define _DELAY_H_

#define F_CPU 14745600
#include <avr/delay.h>

// The functions are borrowed from avr-libc 1.2.*

/******************************************************************************
Portions of avr-libc are Copyright (c) 1999-2004
Keith Gudger,
Steinar Haugen,
Peter Jansen,
Reinhard Jessich,
Magnus Johansson,
Artur Lipowski,
Marek Michalkiewicz,
Colin O'Flynn,
Bob Paddock,
Reiner Patommel,
Michael Rickman,
Theodore A. Roth,
Juergen Schilling,
Philip Soeberg,
Nils Kristian Strom,
Michael Stumpf,
Stefan Swanepoel,
Eric B. Weddington,
Joerg Wunsch,
The Regents of the University of California. 
All rights reserved.

Portions of avr-libc documentation Copyright (c) 1990, 1991, 1993, 1994
The Regents of the University of California.
All rights reserved.


   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE. 

******************************************************************************/

/**
   \ingroup avr_delay

   Perform a delay of \c __us microseconds, using _delay_loop_1().

   The macro F_CPU is supposed to be defined to a
   constant defining the CPU clock frequency (in Hertz).

   The maximal possible delay is 768 us / F_CPU in MHz.
   
   i.e. around 52us for 14.7456MHz
 */
 
#ifndef _delay_loop_1
 // ^^^ The loop(_delay_loop_2) executes three CPU cycles per iteration
static __inline__ void my_delay_us(double __us)
{
	uint8_t __ticks;
	double __tmp = ((F_CPU) / 3e6) * __us;
	// ^^^ 3e6 : 3 -> 3 ins , 6 -> us
	if (__tmp < 1.0)
		__ticks = 1;
	else if (__tmp > 255)
		__ticks = 0;	/* i.e. 256 */
	else
		__ticks = (uint8_t)__tmp;
	_delay_loop_1(__ticks);
}

/**
   \ingroup avr_delay

   Perform a delay of \c __ms milliseconds, using _delay_loop_2().

   The macro F_CPU is supposed to be defined to a
   constant defining the CPU clock frequency (in Hertz).

   The maximal possible delay is 262.14 ms / F_CPU in MHz.
   
   i.e. around 17.77ms for 14.7456Mhz
 */
 // ^^^ The loop(_delay_loop_2) executes four CPU cycles per iteration
static __inline__ void my_delay_ms(double __ms)
{
	uint16_t __ticks;
	double __tmp = ((F_CPU) / 4e3) * __ms;
	// ^^^ 3e6 : 4 -> 4 ins , 3 -> ms
	if (__tmp < 1.0)
		__ticks = 1;
	else if (__tmp > 65535)
		__ticks = 0;	/* i.e. 65536 */
	else
		__ticks = (uint16_t)__tmp;
	_delay_loop_2(__ticks);
}
#endif

#endif