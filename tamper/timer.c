/*
 * timer.c
 * ---------------
 * timer init and functions (uart, watchdog).
 *
 * Copyright © 2018, 2019 Diamond Key Security, NFP
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License only.
 * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, If not, see <https://www.gnu.org/licenses/>.*/

#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"
#include "tamper.h"
#include "ssp.h"

void initTimer1(uint8_t timer)
{
	TCCR0A = (1<<COM0A1) | (1 << WGM01);             //CTC mode
	TCCR0B = (1 << CS00);              //div1
	OCR0A = timer;						// 50us compare value
	//TIFR1 = 
	TIMSK0 |= (1<<OCIE0A);              //if you want interrupt
	
}
void initTimer2(uint8_t timer)
{
	TCCR1A = (1<<COM0A1) | (1 << WGM01);             //CTC mode
	TCCR1B = (1 << CS00);              //div1
	OCR0A = timer;						// 50us compare value
	//TIFR1 =
	TIMSK1 |= (1<<OCIE1A);              //if you want interrupt
	
}


//soft-uart timer
ISR (TIMER1_COMPA_vect)
{
	/*disable interrupts, should we or allow NMI of tamper switch detect? */
	TCNT1 = 0x00;
	
	int i;
	//PORTC ^= _BV(PORTC6);		//toggle gp6 to check on progress
	if (receiving) {
		if (start_bit)	{
			start_bit--;
			//skip first bit
		}
		else {	
			if (rcv_bit_count-1 > 0) {     //looking for data) 
				rcv_char |= ((PINB & (1<<3))>>3)<<(9-rcv_bit_count);  //from (1<<2) to (1<<3)
				//rcv[9-rcv_bit_count] = (PINB & (1<<3))>>3;
			}
			else if (rcv_bit_count == 0) {  //looking for stop bit
				if (PINB & (1<<PINB3)) {    //PINB2 to PINB3
					rcv_valid = 1;
					AVR_LED_PORT ^= _BV(AVR_LED_RED_BIT);
				}
				else {
					rcv_error_stop = 1;
					spi_disable = 0;
				}
				TIMSK1 &= ~(1<<OCIE1A);		//stop the bit timer
				PCMSK1 |= _BV(PCINT11);		//enable start bit detect INT10 to INT11
				receiving = 0;
			}
			rcv_bit_count--;
		}
	}
	if (sending){
		if (start_bit == 1)	{
			start_bit = 0;
			PORTB &= ~_BV(PORTB2);  //PORTB3 to PORTB2
		}
		else {
			if (tx_bit_count<8){
				if (tx_char & (1<<tx_bit_count)){  //check the bit sending LSB first
					PORTB |= _BV(PORTB2);
				}
				else{
					PORTB &= ~_BV(PORTB2);
				}
			}
			else if (tx_bit_count == 8){
				PORTB |= _BV(PORTB2);        //send stop bit
				TIMSK1 &= ~(1<<OCIE1A);		//stop the bit timer
				sending = 0;
				spi_disable = 0;
			}
			tx_bit_count++;
		}
		
	}
	//
	//sei();
	
}

//ISR (TIMER1_COMPA_vect)
ISR (TIMER0_COMPA_vect)
{
	/*disable interrupts, should we or allow NMI of tamper switch detect? */
	//cli();
	//sei();
	
	TCNT0 = 0x00;
	int i;
	//PORTC ^= _BV(PORTC6);		//toggle gp6 to check on progress
	TIMSK0 &= ~(1<<OCIE0A);		//stop the  timer
	TIFR0 |= (1<<OCF0A);
	if (spi_to_flag) {
		spi_to--;
		if (spi_to == 0){
			spi_to_flag = 0;
		}
	}
	if (fifo_delay_flag) {
		fifo_delay--;
		if (fifo_delay == 0){
			fifo_delay_flag = 0;
		}
		else {
			TIMSK0 |= (1<<OCIE0A);
		}
	}
	usart_to = 0;
	
}