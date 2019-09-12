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
#define ENA_TAMP	0x45
#define DIS_TAMP	0x46
#define CHK_LIGHT	0x47
#define CHK_TEMP	0x48
#define CHK_VIBE	0x49
#define SET_CONFIG	0x4A
#define CHK_TAMP	0x4B
#define CHK_FAULT	0x4C
#define CHK_VIBE_S	0x4D
#define GET_VIBE_S	0x4E	
#define BATT_EN		0x4F
#define CHK_CONF	0x50
#define CHK_FAULT_LONG 0x51
#define CHK_CONF_EXT 0x52


#define LIGHT		0x01
#define TEMP		0x02
#define VIBE		0x04

#define CASE		0x08
#define SSP			0x10
#define LL			0x20
#define USART		0x40
#define UNK			0x80
#define TAMP_FLAGS	0x20
#define LIGHT_PRE	0x22
#define TEMP_PRE_HI 0x24
#define TEMP_PRE_LO 0x26
#define VIBE_PRE_HI	0x28
#define VIBE_PRE_LO 0x30

#define SENTINEL	8

//uint8_t USART_eceive (uint8_t data);
void mkm_wipe();
void process_message();
void send(uint8_t tx);
volatile uint8_t ssp_out;
volatile char wd_init;
volatile int32_t temperature;
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
volatile uint16_t light_thresh;
volatile int16_t temp_hi_thresh;
volatile int16_t temp_lo_thresh;
volatile uint8_t vibe_lo_thresh;
volatile uint8_t vibe_hi_thresh;
volatile uint16_t vibe_thresh;
volatile uint8_t light_status;
volatile uint8_t temp_status;
volatile uint8_t vibe_status;
volatile uint8_t light_enable;
volatile uint8_t light_get;
volatile uint8_t temp_enable;
volatile uint8_t vibe_enable;
volatile uint8_t case_enable;
volatile uint8_t ssp_enable;
volatile uint8_t ll_enable;
volatile uint8_t tamper_detected;
volatile uint8_t light_retrieve;
volatile int8_t calib1;
volatile int8_t calib2;
static uint8_t flags;
static uint8_t configured;
volatile uint8_t usart_to;
static volatile uint16_t ssp_fault;
static volatile uint16_t ssp_fault_max;
static volatile uint8_t vibe_fault;
static volatile uint8_t vibe_fault_reset;
static volatile uint8_t light_fault;
static volatile uint8_t n25_fault;
static volatile uint8_t unk_fault;
static volatile uint8_t fault_index;
static volatile uint8_t fault_code[6];
static volatile uint8_t fault_value1[6];
static volatile uint8_t fault_value2[6];
static volatile uint8_t fault_value3[6];
static volatile uint8_t fault_value4[6];
static volatile uint8_t fault_value5[6];
static volatile uint8_t fault_value6[6];

static volatile uint8_t case_flt_set;
static volatile uint8_t vibe_flt_set;
static volatile uint8_t temp_flt_set;
static volatile uint8_t light_flt_set;
static volatile uint8_t ssp_fault_set;
static volatile uint8_t ll_fault_set;
volatile uint8_t sent;
volatile uint8_t spi_to;
volatile uint8_t spi_to_flag;
volatile uint8_t fifo_delay;
volatile uint8_t fifo_delay_flag;
volatile uint32_t tamper_delay;
volatile uint8_t tamper_delay_flag;
volatile uint8_t spi_disable;
volatile uint8_t spi_disabled;
static volatile uint16_t samples;
volatile int16_t x, y, z;
volatile uint8_t xlo, xhi, ylo, yhi, zlo, zhi;
static volatile uint8_t batt_on;
static volatile uint8_t batt_on_cnt;
static volatile uint8_t temp_flt_cnt;
static volatile uint8_t light_flt_cnt;
static volatile uint8_t vibe_flt_cnt;
static volatile uint8_t ll_flt_cnt;
static volatile uint8_t case_flt_cnt;

volatile uint8_t rcv[9];
volatile static uint16_t fifo[30];

#endif /* TAMPER_H_ */