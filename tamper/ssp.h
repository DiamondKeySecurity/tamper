/*
 * ssp.h
 * ---------------
 * SPI expansion chip driver code.
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


#ifndef SSP_H_
#define SSP_H_

/* SPI commands for SSP (MCP23S17) */
#define SSP_WRITE	0x40
#define SSP_READ	0x41
#define SSP_IO		0x61  /* three inputs GPA6, GPA5 and GPA0 */
#define IODIRA		0x00
#define GPIOA		0x12
#define GPPUA		0x0C
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
