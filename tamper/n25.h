/*
 * n25.h
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
 * n25.h
 *
 * Created: 1/22/2019 11:55:04 AM
 *  Author: michael
 */ 


#ifndef N25_H_
#define N25_H_
/* SPI commands for Accelerometer (N25L362) */
#define N25_WR				0x0A
#define N25_RD				0x03
#define N25_FF				0x0D
#define N25_ACT				0x10

/*registers*/
#define N25_DEVID			0x00
#define N25_STATUS			0x0D
#define N25_THRESH_ACT_L	0x20
#define N25_THRESH_ACT_H	0x21
#define	N25_TIME_ACT		0x22
#define N25_ACT_INACT_CTL	0x27
#define N25_INTMAP2			0x2B
#define N25_FILTER_CTL		0x2C
#define N25_POWER_CTL		0x2D

#define DB_N25_SS PORTC4
void n25_read(uint8_t add1, uint8_t add2, uint8_t add3, uint8_t cnt);

#endif /* N25_H_ */