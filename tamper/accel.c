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

void adx_soft_r(){
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(0x1f, VIBE);
	USART_Receive(0x52, VIBE);		//register
	adx_chip_select(0);
}

void adx_setup(){
	//adx_wr_reg(0x1f, 0x52);
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(0x1f, VIBE);		// soft reset
	USART_Receive(0x52, VIBE);		//register
	adx_chip_select(0);
	//adx_wr_reg(ADX_THRESH_ACT_L, vibe_lo_thresh);
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(ADX_THRESH_ACT_L, VIBE);		//0x20
	USART_Receive(vibe_lo_thresh, VIBE);		//register
	adx_chip_select(0);
	//adx_wr_reg(ADX_THRESH_ACT_H, vibe_hi_thresh);
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(ADX_THRESH_ACT_H, VIBE);		//0x21
	USART_Receive(vibe_hi_thresh, VIBE);		//register
	adx_chip_select(0);
	//adx_wr_reg(ADX_TIME_ACT, 5);
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(ADX_TIME_ACT, VIBE);			//0x22
	USART_Receive(0, VIBE);		//register
	adx_chip_select(0);
	
	//adx_wr_reg(ADX_ACT_INACT_CTL, 0x03); //Activity enabled in referenced mode
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(ADX_ACT_INACT_CTL, VIBE);  //register 0x27
	USART_Receive(0x00, VIBE);		//absolute mode, activity only
	adx_chip_select(0);
	//set the FIFO
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(ADX_FIFO_CONTROL, VIBE);    //register 0x28
	USART_Receive(0x00, VIBE);		// set AH bit (allow for up to 511 samples) and triggered mode
	adx_chip_select(0);
	//set the FIFO samples
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(ADX_FIFO_SAMPLES, VIBE);    //register 0x29
	USART_Receive(200, VIBE);		// allow for 200 samples (most of available RAM)
	adx_chip_select(0);
	//adx_wr_reg(ADX_INTMAP2, 0x90); //int2 interrupt low on active
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(ADX_INTMAP1, VIBE);       //register 0x2A
	USART_Receive(0x10, VIBE);		//register
	adx_chip_select(0);
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(ADX_INTMAP2, VIBE);       //register 0x2B
	USART_Receive(0x10, VIBE);		//register
	adx_chip_select(0);
	//adx_wr_reg(ADX_FILTER_CTL, 0x11);  //divide time_act by 1
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(ADX_FILTER_CTL, VIBE);      //register 0x2c
	USART_Receive(0x03, VIBE);		//register
	adx_chip_select(0);
	//adx_wr_reg(ADX_POWER_CTL, 0x0A); //ADX turned on in Wake mode measurement on
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(ADX_POWER_CTL, VIBE);
	USART_Receive(0x0A, VIBE);		//register
	adx_chip_select(0);
}
void adx_temp(){
	uint8_t temp1, temp2;
	adx_chip_select(1);
	USART_Receive(ADX_RD, VIBE);
	USART_Receive(0x14, VIBE);
	temp1 = USART_Receive(0x00, VIBE);//dummy byte to read in register
	temp2 = USART_Receive(0x00, VIBE);//dummy byte to read in register
	
	adx_chip_select(0);
	temperature = (int32_t)(temp1 | temp2<<8) * 0.065;	
}

void adx_wr_reg(uint8_t reg, uint8_t value)
{
	/* for register setting*/
	uint8_t temp;
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(reg, VIBE);		//register
	USART_Receive(value, VIBE);  // value 
	//temp = USART_Receive(0x00, VIBE);//dummy byte to read in register
	adx_chip_select(0);
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
	//temp = USART_Receive(0x00, VIBE);
    adx_chip_select(0);
	adx_chip_select(1);
	USART_Receive(ADX_RD, VIBE);
	USART_Receive(0x0E, VIBE);  //register 0x27
	xlo = USART_Receive(0x00, VIBE);		// x-axis LSB
	xhi = USART_Receive(0x00, VIBE);	// x-axis MSB
	ylo = USART_Receive(0x00, VIBE);		// y-axis LSB
	yhi = USART_Receive(0x00, VIBE);		// y-axis MSB
	zlo = USART_Receive(0x00, VIBE);			// z-axis LSB
	zhi = USART_Receive(0x00, VIBE);	// z-axis MSB
	adx_chip_select(0);
	x = (uint16_t)xhi<<8 | (uint16_t)xlo;
	y = (uint16_t)yhi<<8 | (uint16_t)ylo;
	z = (uint16_t)zhi<<8 | (uint16_t)zlo;
	//x and y vlaues are static at rest , while z has a constant 1G, at least on earth
	if (abs(x)>vibe_thresh | abs(y) > vibe_thresh | abs(z) > vibe_thresh + 1000)
	{
		temp |= 0x10;
	}
	


	return temp;
}

void adx_set_threshold(){
	//adx_wr_reg(ADX_THRESH_ACT_L, vibe_lo_thresh);
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(ADX_THRESH_ACT_L, VIBE);		//0x20
	USART_Receive(vibe_lo_thresh, VIBE);		//register
	adx_chip_select(0);
	//adx_wr_reg(ADX_THRESH_ACT_H, vibe_hi_thresh);
	adx_chip_select(1);
	USART_Receive(ADX_WR, VIBE);
	USART_Receive(ADX_THRESH_ACT_H, VIBE);		//0x21
	USART_Receive(vibe_hi_thresh, VIBE);		//register
	adx_chip_select(0);
}

uint16_t adx_read_fifo_count(){
	/* get number of samples in fifo */
	uint8_t temp;
	uint16_t samples, sample;
	adx_chip_select(1);
	USART_Receive(ADX_RD, VIBE);
	USART_Receive(ADX_FIFO_ENTRIES, VIBE);  //device id address 0x00
	temp = USART_Receive(0x00, VIBE);
	samples = (uint16_t)  USART_Receive(0x00, VIBE)<<8 |(uint16_t) temp;
    adx_chip_select(0);
	return samples;
}

uint8_t adx_read_fifo_samples(uint8_t samples){
	uint8_t temp;
	uint16_t sample;
	for (int i = 0; i<samples; i++) {
		adx_chip_select(1);
		USART_Receive(ADX_RD, VIBE);
		USART_Receive(ADX_FF, VIBE);  //device id address 0x00
		temp = USART_Receive(0x00, VIBE);
		sample = (uint16_t)USART_Receive(0x00, VIBE)<<8 |(uint16_t) temp;
		adx_chip_select(0);
		fifo[i] = sample;
		//to do insert timer here
		/*fifo_delay_flag = 1;
		OCR0A = 10;
		fifo_delay = 1;
		TCNT0 = 0x00;
		TIMSK0 |= (1<<OCIE0A);
		while (fifo_delay_flag){};*/
		//TIMSK0 &= ~(1<<OCIE0A);		//stop the  timer
		//TIFR0 |= (1<<OCF0A);
	}
	//OCR0A = 185;
	return temp;
}