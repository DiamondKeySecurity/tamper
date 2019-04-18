/*
 * accel.h
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


#ifndef ACCEL_H_
#define ACCEL_H_
/* SPI commands for Accelerometer (ADXL362) */
#define ADX_WR				0x0A
#define ADX_RD				0x0B
#define ADX_FF				0x0D
#define ADX_ACT				0x10

/*registers*/
#define ADX_DEVID			0x00
#define ADX_STATUS			0x0D
#define ADX_THRESH_ACT_L	0x20
#define ADX_THRESH_ACT_H	0x21
#define	ADX_TIME_ACT		0x22
#define ADX_ACT_INACT_CTL	0x27
#define ADX_INTMAP2			0x2B
#define ADX_FILTER_CTL		0x2C
#define ADX_POWER_CTL		0x2D

#define DB_ADX_SS PORTC4
void adx_wr_reg(uint8_t reg, uint8_t value);
void adx_setup();
uint8_t adx_read_status();
void adx_chip_select(int select_flag);
uint8_t adx_rd_reg(uint8_t reg);



#endif /* ACCEL_H_ */