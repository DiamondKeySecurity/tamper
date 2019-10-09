/*
 * n25.c
 * ---------------
 * flash eeprom driver code.
 *
 * Copyright © 2018, 2019 Diamond Key Security, NFP
 # Copyright (c) 2019  Diamond Key Security, NFP
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
# - Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
#
# - Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#
# - Neither the name of the Diamond Key Security nor the names of its contributors may
#   be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
# IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * n25.c
 *
 * Created: 1/22/2019 11:45:24 AM
 *  Author: michael
 */ 
#include <inttypes.h>
#include <avr/io.h> /* -D__AVR_ATtiny828__ will include <avr/iotn828.h> */
#include "n25.h"
#include "tamper.h"
#include "ssp.h"


void n25_chip_select(int select_flag)
{
	if (select_flag) {
		//AVR_GPIO_PORT &= ~_BV(PORTC6); /* CS low. */
	}
	else {
		//AVR_GPIO_PORT |= _BV(PORTC6); /* CS high. */
	}
}

void n25_read(uint8_t add1, uint8_t add2, uint8_t add3, uint8_t cnt)
{
	/* ADX chip manufacturer ID */
	uint8_t temp;
	n25_chip_select(1);
	USART_Receive(N25_RD, 0x20);
	USART_Receive(add1, 0x20);  //device id address MSB
	USART_Receive(add2, 0x20);  //device id address MID
	USART_Receive(add3, 0x20);  //device id address LSB
	for(int i = 0; i< cnt; i++){
		temp = USART_Receive(0x00, 0x20);//dummy byte to read in register
	}
	n25_chip_select(0);
}
