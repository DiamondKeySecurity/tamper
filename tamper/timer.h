/* 2018, 2019  Diamond Key Security, NFP   All rights reserved. /*
 * timer.c
 * ---------------
 * timer init and functions (uart, watchdog).
 *
 * Copyright � 2018, 2019 Diamond Key Security, NFP
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


#ifndef TIMER_H_
#define TIMER_H_

#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
void initTimer1(uint8_t timer);
#define TWD_INIT	1
#define TWD_RESET	137





#endif /* TIMER_H_ */