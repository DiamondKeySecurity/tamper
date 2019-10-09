/*
 * optic.h
 * ---------------
 * optic chip driver code and functions.
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

#ifndef OPTIC_H_
#define OPTIC_H_
/* SPI commands for Melexis chip */

#define MLX_NOP			0x00
#define MLX_CALIB1		0xB0
#define MLX_CALIB2		0xC0
#define MLX_SM			0xD0
#define MLX_RS			0xF0
#define MLX_RO			0xC3
#define MLX_TEMP_AMB	0xC0
#define MLX_WR			0x87
#define MLX_RR			0x8E
#define MLX_RSLP		0xE1
#define MLX_CSLP		0xA3
#define MLX_CH_EN		0x84  //temp and Ambient ChC
#define MLX_CH_EN_ADDP	0xD4  //EnChan address 7 odd parity with enable bits
#define DB_MLX_SS PORTC5
void mlx_start_meas();
void mlx_chip_select(int select_flag);
void mlx_setup();
void mlx_get_meas();
void mlx_get_calib();
uint8_t mlx_sleep();
void mlx_reset();
void mlx_request_sleep();
void mlx_confirm_sleep();
void mlx_nop();

#endif /* OPTIC_H_ */
