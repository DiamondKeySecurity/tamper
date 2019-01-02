/* Copyright (c) 2016, NORDUnet A/S. */
/*

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

- Neither the name of the NORDUnet nor the names of its contributors may
be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (c) 2018, Diamond Key Security, NFP.


Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

- Neither the name of the Diamond Key Security nor the names of its
contributors maybe used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"
#include "tamper.h"
#include "ssp.h"

void initTimer1(uint8_t timer)
{
	TCCR0A = (1 << WGM01);             //CTC mode
	TCCR0B = (1 << CS00);              //div1
	OCR0A = timer;						// 50us compare value
	TIMSK0 |= (1<<OCIE0A);              //if you want interrupt
}



//watchdog timer
ISR (TIMER0_COMPA_vect)
{
	/*disable interrupts, should we or allow NMI of tamper switch detect? */
	cli();
	int i;
	/* first timer for wdog is 140ms so we hit 3/4 through (105ms) thereafter, every 140ms
	to hit the middle of the watchdog open window*/
	/*if (wd_init) {
		initTimer1(TWD_RESET);
		wd_init = 0x00;
	}*/
	/*TBD !! back to back writes should create the minimum 10us low pulse*/
	
	/*if(wd_init == 0x01) {
		//ssp_write(ssp_out & (~WDOG_RS));
		PORTC |= (1<<PORTC6);
		wd_init = 0x00;	
	}
	else {
		//ssp_write(ssp_out | WDOG_RS);
		wd_init = 0x01;
		PORTC &= (0<<PORTC6);
	}*/
	if (start_bit == 1){
		if (!PORTB3) {	//looking for and found valid start bit
			start_bit = 0;
		}
		else {
			OCR0A = FULL_BIT;    // 104uS bit time
		}
	}
	else {
		if (bit_count > 0) {     //looking for data
			rcv_char |= ((bit_count-1)<<PORTB3);
			bit_count--;
		}
		else if (bit_count == 0) {  //looking for stop bit
			if (!PORTB3) {
				rcv_valid = 1;
			}
			else {
				rcv_error_stop = 1;
			}
			TIMSK0 &= ~(1<<OCIE0A);		//stop bit timer
			PCMSK1 |= _BV(PCINT11);		//enable start bit detect
			
		}
	}
	sei();
}