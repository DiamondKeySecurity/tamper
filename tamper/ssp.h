/*
 * ssp.h
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


#ifndef SSP_H_
#define SSP_H_

/* SPI commands for SSP (MCP23S17) */
#define SSP_WRITE	0x40
#define SSP_READ	0x41
#define SSP_IO		0x61  /* three inputs GPA6, GPA5 and GPA0 */
#define IODIRA		0x00
#define GPIOA		0x12

#define GPINTENA	0x04
#define INTCONA		0x08
#define INTCAPA		0x10
#define IOCON		0x0A
#define DEFVALA		0x06
#define OLATA		0x14
#define INTFA		0x0E
#define GPA6		0x40
#define LV_IN		0x01
#define LV_IND		0x02
#define TAMP_CLR	0x04
#define TAMP_ON		0x08
#define WDOG_RS		0x10
#define BATT_ON		0x20
#define TAMP_MON	0x40

#define DB_SSP_SS PORTC7

uint8_t USART_Receive (uint8_t data, uint8_t source);
void spi_usart_setup(int on_flag);
void ssp_chip_select(int select_flag);
void ssp_boot();
void ssp_setup();
void ssp_int_config();
uint8_t ssp_read_byte();
void ssp_write(uint8_t value);
void ssp_clear_tamper();
void ssp_int_reset();


#endif /* SSP_H_ */