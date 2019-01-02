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

Copyright (c) 2018, Diamond Key Security, NFP.


Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

- Neither the name of the Diamond Key Security nor the names of its
contributors maybe used to endorse or promote products derived from
this software without specific prior written permission.

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
#include <inttypes.h>
#include <avr/io.h> /* -D__AVR_ATtiny828__ will include <avr/iotn828.h> */
#include "accel.h"
#include "tamper.h"
#include "ssp.h"


void adx_chip_select(int select_flag)
{
	if (select_flag) {
	AVR_GPIO_PORT &= ~_BV(DB_ADX_SS); /* CS low. */
	}
	else {
	AVR_GPIO_PORT |= _BV(DB_ADX_SS); /* CS high. */
	}
}

void adx_read_id()
{
	/* ADX chip manufacturer ID */
	uint8_t temp;
	adx_chip_select(1);
	USART_Receive(ADX_RD);
	USART_Receive(ADX_DEVID);  //device id address 0x00
	temp = USART_Receive(0x00);//dummy byte to read in register
	adx_chip_select(0);
}

void adx_wr_reg(uint8_t reg, uint8_t value)
{
	/* for register setting*/
	uint8_t temp;
	adx_chip_select(1);
	USART_Receive(ADX_WR);
	USART_Receive(reg);		//register
	USART_Receive(value);  // value 
	temp = USART_Receive(0x00);//dummy byte to read in register
	adx_chip_select(0);
}

void adx_setup(){
	adx_wr_reg(ADX_THRESH_ACT_L, 250);
	adx_wr_reg(ADX_THRESH_ACT_H, 0);
	adx_wr_reg(ADX_TIME_ACT, 5);
	adx_wr_reg(ADX_FILTER_CTL, 0x11);  //divide time_act by 1
	adx_wr_reg(ADX_ACT_INACT_CTL, 0x03); //Activity enabled in referenced mode
	adx_wr_reg(ADX_INTMAP2, 0x90); //int2 interrupt low on active
	adx_wr_reg(ADX_POWER_CTL, 0x0A); //ADX turned on in Wake mode measurement on
}

uint8_t adx_rd_reg(uint8_t reg){
	/* ADX status */
	uint8_t temp;
	adx_chip_select(1);
	USART_Receive(ADX_RD);
	USART_Receive(reg);  //device id address 0x00
	temp = USART_Receive(0x00);
	adx_chip_select(0);
	return temp;
}

uint8_t adx_read_status(){
	/* ADX status */
	uint8_t temp;
	adx_chip_select(1);
	USART_Receive(ADX_RD);
	USART_Receive(ADX_STATUS);  //device id address 0x00
	temp = USART_Receive(0x00);
    adx_chip_select(0);
	
	return temp;
}
