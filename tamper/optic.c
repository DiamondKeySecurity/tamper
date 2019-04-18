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
	USART_Receive(MLX_SM);
	USART_Receive(MLX_TEMP_AMB);
	mlx_chip_select(0);
}

void mlx_get_meas()
{
	/* MLX chip get temp and ambient chan values */
	mlx_chip_select(1);
	USART_Receive(MLX_RO);
	USART_Receive(0);
	//read in values
	temperature = USART_Receive(0)<<8;
	temperature |= USART_Receive(0);
	light =  USART_Receive(0)<<8;
	light |= USART_Receive(0); 
	mlx_chip_select(0);
	
}

void mlx_setup()
{
	/* MLX chip setup enable temp and Ambient Ch. C */
	mlx_chip_select(1);
	USART_Receive(MLX_WR);
	USART_Receive(MLX_CH_EN);
	USART_Receive(MLX_CH_EN_ADDP);
	mlx_chip_select(0);
}

void mlx_nop(){
	mlx_chip_select(1);
	USART_Receive(0);
	USART_Receive(0);
	mlx_chip_select(0);
}
void mlx_reset(){
	mlx_chip_select(1);
	USART_Receive(MLX_RS);
	USART_Receive(0);
	mlx_chip_select(0);
}

void mlx_request_sleep(){
	mlx_chip_select(1);
	USART_Receive(MLX_RSLP);
	USART_Receive(0);
	mlx_chip_select(0);
}

void mlx_confirm_sleep(){
	mlx_chip_select(1);
	USART_Receive(MLX_CSLP);
	USART_Receive(0);
	mlx_chip_select(0);
}

uint8_t mlx_sleep()
{
	uint8_t confirm;
	mlx_chip_select(1);
	USART_Receive(MLX_RS);
	USART_Receive(0);
	mlx_chip_select(0);
	/* put MLX in to lowest power, disables WDOG and measurements*/
	/*first make request*/
	mlx_chip_select(1);
	USART_Receive(MLX_RSLP);
	USART_Receive(0);
	mlx_chip_select(0);
	
	/*then confirm*/
	mlx_chip_select(1);
	confirm = USART_Receive(MLX_CSLP);
	USART_Receive(0);
	mlx_chip_select(0);
	
	/*then confirm*/
	mlx_chip_select(1);
	confirm = USART_Receive(MLX_CSLP);
	USART_Receive(0);
	mlx_chip_select(0);
	for (int i = 0; i < 16000; i++);
	return confirm;	
}