/*
 * n25.c
 * ---------------
 * flash eeprom driver code.
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
