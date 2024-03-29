/*
 * optic.c
 * ---------------
 * optic chip driver code and functions.
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


#include <inttypes.h>
#include <avr/io.h> /* -D__AVR_ATtiny828__ will include <avr/iotn828.h> */
#include "optic.h"
#include "tamper.h"
#include "ssp.h"



void mlx_chip_select(int select_flag)
{
	if (select_flag)
	AVR_GPIO_PORT &= ~_BV(DB_MLX_SS); /* CS low. */
	else
	AVR_GPIO_PORT |= _BV(DB_MLX_SS); /* CS high. */
}

void mlx_start_meas()
{
	/* MLX chip start temperature measurement */
	mlx_chip_select(1);
	USART_Receive(MLX_SM, LIGHT);
	USART_Receive(MLX_TEMP_AMB, LIGHT);
	mlx_chip_select(0);
}

void mlx_get_meas()
{
    uint16_t temp_t;
	volatile uint8_t temp_1, temp_2; 
	/* MLX chip get temp and ambient chan values */
	mlx_chip_select(1);
	USART_Receive(MLX_RO, LIGHT);
	USART_Receive(0, LIGHT);
	//read in values
	//temp_1 = USART_Receive(0, LIGHT)<<8;
	temp_1 = USART_Receive(0, LIGHT);
	temp_2 = USART_Receive(0, LIGHT);
	temp_t = (uint16_t) temp_1*256  +(uint16_t) temp_2;
	light =  USART_Receive(0, LIGHT)<<8;
	light |= USART_Receive(0, LIGHT); 
	mlx_chip_select(0);
}

void mlx_setup()
{
	/* MLX chip setup enable temp and Ambient Ch. C */
	mlx_chip_select(1);
	USART_Receive(MLX_WR, LIGHT);
	USART_Receive(MLX_CH_EN, LIGHT);
	USART_Receive(MLX_CH_EN_ADDP, LIGHT);
	mlx_chip_select(0);
}
void mlx_write_reg(uint8_t reg, uint8_t value )
{
	/* MLX chip setup enable temp and Ambient Ch. C */
	int Control1 = 0x87;
	int Control2 = value;
	int Control3;
	int i = 1;
	int ParityCheckSum = 1;
	int Counter = 0;
	while (i <= 12)
	{
		if (ParityCheckSum & ((value*16) + reg))
		{
			Counter++;
		}
		ParityCheckSum = ParityCheckSum << 1;
		i++;
	}
	if (Counter % 2 == 0)
	{
		Control3 = ((reg<<4) & 0xF0) + 8;
	}
	else
	{
		Control3 = ((reg<<4) & 0xF0) + 4;
	}
	mlx_chip_select(1);
	USART_Receive((uint8_t) Control1, LIGHT);
	USART_Receive((uint8_t) Control2, LIGHT);
	USART_Receive((uint8_t) Control3, LIGHT);
	mlx_chip_select(0);
}

void mlx_get_calib()
{
	/* MLX chip get temp and ambient chan values */
	mlx_chip_select(1);
	USART_Receive(MLX_RR, LIGHT);
	USART_Receive(MLX_CALIB1, LIGHT);
	//read in value
	calib1 = USART_Receive(0, LIGHT);
	mlx_chip_select(0);
	mlx_chip_select(1);
	USART_Receive(MLX_RR, LIGHT);
	USART_Receive(MLX_CALIB2, LIGHT);
	//read in value
	calib2 = USART_Receive(0, LIGHT);
	mlx_chip_select(0);
	
}
void mlx_nop(){
	mlx_chip_select(1);
	USART_Receive(0, LIGHT);
	USART_Receive(0, LIGHT);
	mlx_chip_select(0);
}
void mlx_reset(){
	mlx_chip_select(1);
	USART_Receive(MLX_RS, LIGHT);
	USART_Receive(0, LIGHT);
	mlx_chip_select(0);
}

void mlx_request_sleep(){
	mlx_chip_select(1);
	USART_Receive(MLX_RSLP, LIGHT);
	USART_Receive(0, LIGHT);
	mlx_chip_select(0);
}

void mlx_confirm_sleep(){
	mlx_chip_select(1);
	USART_Receive(MLX_CSLP, LIGHT);
	USART_Receive(0, LIGHT);
	mlx_chip_select(0);
}

uint8_t mlx_sleep()
{
	uint8_t confirm;
	mlx_chip_select(1);
	USART_Receive(MLX_RS, LIGHT);
	USART_Receive(0, LIGHT);
	mlx_chip_select(0);
	/* put MLX in to lowest power, disables WDOG and measurements*/
	/*first make request*/
	mlx_chip_select(1);
	USART_Receive(MLX_RSLP, LIGHT);
	USART_Receive(0, LIGHT);
	mlx_chip_select(0);
	
	/*then confirm*/
	mlx_chip_select(1);
	confirm = USART_Receive(MLX_CSLP, LIGHT);
	USART_Receive(0, LIGHT);
	mlx_chip_select(0);
	
	/*then confirm*/
	mlx_chip_select(1);
	confirm = USART_Receive(MLX_CSLP, LIGHT);
	USART_Receive(0, LIGHT);
	mlx_chip_select(0);
	for (int i = 0; i < 16000; i++);
	return confirm;	
}
