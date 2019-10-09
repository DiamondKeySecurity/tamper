/*
 * n25.h
 * ---------------
 * flash eeprom driver code.
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
