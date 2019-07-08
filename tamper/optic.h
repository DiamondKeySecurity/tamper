/*
 * optic.h
 * ---------------
 * optic chip driver code and functions.
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