/*
 * accel.h
 * ---------------
 * accelerometer chip driver code and function.
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


#ifndef ACCEL_H_
#define ACCEL_H_
/* SPI commands for Accelerometer (ADXL362) */
#define ADX_WR				0x0A
#define ADX_RD				0x0B
#define ADX_FF				0x0D
#define ADX_ACT				0x10
#define ADX_ERR				0x80

/*registers*/
#define ADX_DEVID			0x00
#define ADX_STATUS			0x0B
#define ADX_FIFO_ENTRIES	0x0C
#define ADX_THRESH_ACT_L	0x20
#define ADX_THRESH_ACT_H	0x21
#define	ADX_TIME_ACT		0x22
#define ADX_ACT_INACT_CTL	0x27
#define ADX_FIFO_CONTROL	0x28
#define ADX_FIFO_SAMPLES	0x29
#define ADX_INTMAP1			0x2A
#define ADX_INTMAP2			0x2B
#define ADX_FILTER_CTL		0x2C
#define ADX_POWER_CTL		0x2D

#define DB_ADX_SS PORTC4
void adx_wr_reg(uint8_t reg, uint8_t value);
void adx_setup();
uint8_t adx_read_status();
void adx_chip_select(int select_flag);
uint8_t adx_rd_reg(uint8_t reg);
void adx_soft_r();
void adx_temp();
uint16_t adx_read_fifo_count();
uint8_t adx_read_fifo_samples(uint8_t samples);
void adx_set_threshold();

#endif /* ACCEL_H_ */
