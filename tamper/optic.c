/*
 * optic.c
 * ---------------
 * optic chip driver code and functions.
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
	int Control1 = 0x87;	int Control2 = value;	int Control3;	int i = 1;	int ParityCheckSum = 1;	int Counter = 0;	while (i <= 12)	{		if (ParityCheckSum & ((value*16) + reg))		{			Counter++;		}		ParityCheckSum = ParityCheckSum << 1;		i++;	}	if (Counter % 2 == 0)	{		Control3 = ((reg<<4) & 0xF0) + 8;	}	else	{		Control3 = ((reg<<4) & 0xF0) + 4;	}
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