/*
 * tamper.h
 * ---------------
 * tamper initialization and execution loop.
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

/* Copyright (c) 2016, NORDUnet A/S. */
/*

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

- Neither the name of the NORDUnet nor the names of its contributors may
be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


*/


#ifndef TAMPER_H_
#define TAMPER_H_
#define AVR_GPIO_PORT PORTC	/* Port to communicate to sensor board */
#define HALF_BIT F_CPU/1 * 0.000105 - 1
#define FULL_BIT F_CPU/1 * 0.000105 - 1
#define AVR_LED_PORT PORTA
#define AVR_LED_BLUE_BIT PORTA0
#define AVR_LED_RED_BIT PORTA3
#define AVR_LED_YELLOW_BIT PORTA2
#define AVR_LED_GREEN_BIT PORTA1

#define SET_LIGHT	0x41
#define SET_TEMP_HI 0x42
#define SET_TEMP_LO	0x43
#define SET_VIBE	0x44


//uint8_t USART_Receive (uint8_t data);
void mkm_wipe();
volatile uint8_t ssp_out;
volatile char wd_init;
volatile uint16_t temperature;
volatile uint16_t light;
volatile uint8_t start_bit;
volatile uint8_t sending;
volatile uint8_t receiving;
volatile uint8_t rcv_bit_count;
volatile uint8_t rcv_char;
volatile uint8_t tx_char;
volatile uint8_t tx_bit_count;
volatile uint8_t rcv_valid;
volatile uint8_t rcv_error_stop;
volatile uint8_t light_status;
volatile uint8_t temp_status;
volatile uint8_t vibe_status;

volatile uint8_t rcv[9];

#endif /* TAMPER_H_ */