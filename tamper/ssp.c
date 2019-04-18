/*
 * ssp.c
 * ---------------
 * SPI expansion chip driver code.
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
#include "ssp.h"
#include "tamper.h"
#include "accel.h"
#include "optic.h"
#include "n25.h"



void
ssp_chip_select(int select_flag)
{
	if (select_flag)
	AVR_GPIO_PORT &= ~_BV(DB_SSP_SS); /* CS low. */
	else
	AVR_GPIO_PORT |= _BV(DB_SSP_SS); /* CS high. */
}

void
spi_usart_setup(int on_flag){
	if (on_flag)
	{
		UBRR = 0;
		/* set xck as output */
		DDRC = (1<<DDRC0) | (1<<DDRC4) | (1<<DDRC5) | (1<<DDRC6) | (1<<DDRC7);
		
		/*set USART to Master, SPI mode 0*/
		UCSRC = (1<<UMSEL1)|(1<<UMSEL0)|(1<<UCSZ0)|(1<UCPOL);
		/*Enable receiver and transmitter */
		UCSRB = (1<<RXEN)|(1<<TXEN);
		/*set baud rate */
		UBRR = 0x0008;
		ssp_chip_select(0);
		mlx_chip_select(0);
		adx_chip_select(0);
		n25_chip_select(0);
	}
}

void
ssp_boot()
{
	/* on power up ssp chip needs it's chip select toggled once */
	ssp_chip_select(1);
	for (int k = 0; k < 1500;k++);
	ssp_chip_select(0);
	
}

uint8_t
USART_Receive (uint8_t data)
{
	/* wait for empty transmit buffer */
	while (!(UCSRA & (1<<UDRE)));
	/*put data into buffer, sends data */
	UDR = data;
	/*wait for the data to be received */
	while (!(UCSRA & (1<<RXC)));
	return UDR;
}

void
ssp_setup()
{
	/* set for write and set I/O dir. register per board design */
	ssp_chip_select(1);
	USART_Receive(SSP_WRITE);
	USART_Receive(IODIRA);
	USART_Receive(SSP_IO);
	ssp_chip_select(0);
	
}

void
ssp_int_config()
{
	
	/* set tamper gpio for interrupt on change */
	ssp_chip_select(1);
	USART_Receive(SSP_WRITE);
	USART_Receive(INTCONA);
	USART_Receive(TAMP_MON);
	ssp_chip_select(0);
	/* set tamper gpio for default value '0' */
	ssp_chip_select(1);
	USART_Receive(SSP_WRITE);
	USART_Receive(DEFVALA);
	USART_Receive(0x00);
	ssp_chip_select(0);
	//set IOCON INTPOL to 1?
	ssp_chip_select(1);
	USART_Receive(SSP_WRITE);
	USART_Receive(IOCON);
	USART_Receive(0x00);
	ssp_chip_select(0);
	/* read A to clear */
	ssp_chip_select(1);
	USART_Receive(SSP_READ);
	USART_Receive(GPIOA);
	USART_Receive(0x00);
	ssp_chip_select(0);
	/* read incapA to clear */
	ssp_chip_select(1);
	USART_Receive(SSP_READ);
	USART_Receive(INTCAPA);
	USART_Receive(0x00);
	ssp_chip_select(0);
	/* setup tamper gpio interrupt pin */
	ssp_chip_select(1);
	USART_Receive(SSP_WRITE);
	USART_Receive(GPINTENA);
	USART_Receive(TAMP_MON);
	ssp_chip_select(0);
}

void ssp_int_reset()
{
	/* read incapA to clear */
	ssp_chip_select(1);
	USART_Receive(SSP_READ);
	USART_Receive(INTCAPA);
	USART_Receive(0x00);
	ssp_chip_select(0);
}
	
                   
uint8_t
ssp_read_byte()
{
	/* read port A */
	uint8_t temp;
	ssp_chip_select(1);
	USART_Receive(SSP_READ);
	USART_Receive(INTFA);
	temp = USART_Receive(0x00);
	ssp_chip_select(0);
	return temp;
}

void
ssp_write(uint8_t value)
{
	/* set for write and set I/O dir. register per board design */
	//ssp_out |= value;
	ssp_chip_select(1);
	USART_Receive(SSP_WRITE);
	USART_Receive(GPIOA);
	USART_Receive(value);
	ssp_chip_select(0);
	ssp_chip_select(1);
	USART_Receive(SSP_WRITE);
	USART_Receive(OLATA);
	USART_Receive(value);
	ssp_chip_select(0);
}


void
ssp_clear_tamper()
{
	/* set for write and set I/O dir. register per board design */
	ssp_chip_select(1);
	USART_Receive(SSP_WRITE);
	USART_Receive(GPIOA);
	USART_Receive(0x08);
	ssp_chip_select(0);
	ssp_chip_select(1);
	USART_Receive(SSP_WRITE);
	USART_Receive(0x14);
	USART_Receive(0x08);
	ssp_chip_select(0);
}