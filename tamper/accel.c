/* 2018, 2019  Diamond Key Security, NFP   All rights reserved. */
/*
 * accel.c
 * ---------------
 * accelerometer chip driver code and function.
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

#include <inttypes.h>
#include <avr/io.h> /* -D__AVR_ATtiny828__ will include <avr/iotn828.h> */
#include "accel.h"
#include "tamper.h"
#include "ssp.h"


void adx_chip_select(int select_flag)
{
	if (select_flag) {
	AVR_GPIO_PORT &= ~_BV(DB_ADX_SS); /* CS low. */
	}
	else {
	AVR_GPIO_PORT |= _BV(DB_ADX_SS); /* CS high. */
	}
}

void adx_read_id()
{
	/* ADX chip manufacturer ID */
	uint8_t temp;
	adx_chip_select(1);
	USART_Receive(ADX_RD, VIBE);
	USART_Receive(ADX_DEVID, VIBE);  //device id address 0x00
	temp = USART_Receive(0x00, VIBE);//dummy byte to read in register
	adx_chip_select(0);
}

void adx_wr_reg(uint8_t reg, uint8_t value)
{
	/* for register setting*/
	uint8_t temp;
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(reg, VIBE);		//register
	USART_Receive(value, VIBE);  // value 
	temp = USART_Receive(0x00, VIBE);//dummy byte to read in register
	adx_chip_select(0);
}

void adx_setup(){
	adx_wr_reg(ADX_THRESH_ACT_L, vibe_lo_thresh);
	adx_wr_reg(ADX_THRESH_ACT_H, vibe_hi_thresh);
	adx_wr_reg(ADX_TIME_ACT, 5);
	adx_wr_reg(ADX_FILTER_CTL, 0x11);  //divide time_act by 1
	adx_wr_reg(ADX_ACT_INACT_CTL, 0x03); //Activity enabled in referenced mode
	adx_wr_reg(ADX_INTMAP2, 0x90); //int2 interrupt low on active
	adx_wr_reg(ADX_POWER_CTL, 0x0A); //ADX turned on in Wake mode measurement on
}

uint8_t adx_rd_reg(uint8_t reg){
	/* ADX status */
	uint8_t temp;
	adx_chip_select(1);
	USART_Receive(ADX_RD, VIBE);
	USART_Receive(reg, VIBE);  //device id address 0x00
	temp = USART_Receive(0x00, VIBE);
	adx_chip_select(0);
	return temp;
}

uint8_t adx_read_status(){
	/* ADX status */
	uint8_t temp;
	adx_chip_select(1);
	USART_Receive(ADX_RD, VIBE);
	USART_Receive(ADX_STATUS, VIBE);  //device id address 0x00
	temp = USART_Receive(0x00, VIBE);
    adx_chip_select(0);
	
	return temp;
}
