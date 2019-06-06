/*
 * tamper.c
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
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/


#include <inttypes.h>
#include <avr/io.h> /* -D__AVR_ATtiny828__ will include <avr/iotn828.h> */
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include "tamper.h"
#include "optic.h"
#include "accel.h"
#include "ssp.h"
#include "timer.h"
#include "n25.h"

/* Mapping of pins to names. */
#define MKM_AVR_CS_N PORTA5
#define MKM_CONTROL_AVR_ENA PORTA6
#define MKM_CONTROL_FPGA_DIS PORTA7

#define MKM_AVR_MOSI PORTD0
#define MKM_AVR_MISO PORTD1
#define MKM_AVR_SCK PORTD3

/* Input pins. */
#define AVR_PANIC_PIN PINA      /* Panic button. */
#define AVR_PANIC_BIT PINA4     /* 0 = panic. */

/* Output ports. */
#define AVR_LED_PORT PORTA
#define AVR_LED_BLUE_BIT PORTA0
#define AVR_LED_RED_BIT PORTA3
#define AVR_LED_YELLOW_BIT PORTA2
#define AVR_LED_GREEN_BIT PORTA1

#define MKM_CS_PORT PORTA       /* MKM chip select. */
#define MKM_CS_BIT MKM_AVR_CS_N /* 0 => selected. */

#define MKM_CONTROL_AVR_PORT PORTA              /* AVR in control. */
#define MKM_CONTROL_AVR_BIT MKM_CONTROL_AVR_ENA /* 0 => in control. */

#define MKM_CONTROL_FPGA_PORT PORTA /* FPGA in control. */
#define MKM_CONTROL_FPGA_BIT MKM_CONTROL_FPGA_DIS /* 0 => in control. */

/* SPI. */
#define MKM_AVR_MOSI_PORT PORTD
#define MKM_AVR_MOSI_BIT MKM_AVR_MOSI /* PD0, Master Out Slave In. */
#define MKM_AVR_MISO_PORT PORTD
#define MKM_AVR_MISO_BIT PORTD1 /* PD1, Master In Slave Out. */
#define MKM_AVR_SCK_PORT PORTD
#define MKM_AVR_SCK_BIT PORTD3  /* PD3, SPI clock. */


/* AVR_GPIO */


/*******/
uint8_t id[20];


/* MKM */
static inline void
mkm_chip_select(int select_flag)
{
  if (select_flag)
    MKM_CS_PORT &= ~_BV(MKM_CS_BIT); /* CS low. */
  else
    MKM_CS_PORT |= _BV(MKM_CS_BIT); /* CS high. */
}

static inline void
mkm_grab()
{
  MKM_CONTROL_FPGA_PORT |= _BV(MKM_CONTROL_FPGA_BIT);
  MKM_CONTROL_AVR_PORT &= ~_BV(MKM_CONTROL_AVR_BIT);
}

static inline void
mkm_release()
{
  MKM_CONTROL_AVR_PORT |= _BV(MKM_CONTROL_AVR_BIT);
  MKM_CONTROL_FPGA_PORT &= ~_BV(MKM_CONTROL_FPGA_BIT);
}

/*******/
/* SPI */
#define SPI_SS DDRC0            /* SPI slave select. */

static inline void
spi_setup(int on_flag)
{
  if (on_flag)
    {
      /* Disable SPI power reduction. */
      PRR &= ~_BV(PRSPI);

      /* Configure MOSI and SCK pins as output. */
      DDRD = _BV(MKM_AVR_MOSI_BIT) | _BV(MKM_AVR_SCK_BIT);

      /* Make sure SPI slave select (SS) is configured as output
         before enabling SPI master mode! */
      DDRC |= _BV(SPI_SS);  //MAD WTF? this is not the GPIO port......

      /* Enable SPI in master mode, clock rate f/16. */
      SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0);
	  
	  MKM_CS_PORT |= _BV(MKM_CS_BIT); /* CS high. */
    }
  else
    {
      SPCR &= ~_BV(SPE);        /* Disable SPI. */
      PRR |= _BV(PRSPI);        /* Enable SPI power reduction. */
    }
}



/* SPI commands for MKM (23K640). */
#define SPI_READ 0x03
#define SPI_WRITE 0x02
#define SPI_RDSR 0x05           /* Read status register. */
#define SPI_WRSR 0x01           /* Write status register. */
#define SPI_RDID 0x9E			/* Read Id information */



static inline void
spi_write(uint8_t val)
{
  /* Move the value to be sent to the SPI slave into the SPI
     register. This starts the SPI clock. */
  SPDR = val;

  /* Wait for the byte to be shifted into the slave. */
  loop_until_bit_is_set(SPSR, SPIF);
  //while (!_BV(SPIF));
}

static inline uint8_t
spi_read()
{
  /* Start clocking the SPI slave by moving a dummy byte (0) into the
     SPI register and wait for the byte from the slave to be shifted
     in. */
  spi_write(0);

  /* Read the SPI register and return the value. */
  return SPDR;
}

static inline uint8_t
spi_read_status()
{
  spi_write(SPI_RDSR);
  return SPDR;
}

#define SPI_OPERATION_BYTE 0x00
#define SPI_OPERATION_SEQUENCE 0x40
#define SPI_OPERATION_PAGE 0x80

static inline void
spi_set_operation(uint8_t mode)
{
  spi_write(SPI_WRSR);
  spi_write(mode);
}

#if 1
static void
spi_write_byte(uint16_t addr, uint8_t data)
{
  mkm_chip_select(1);
  spi_write(SPI_WRITE);
  spi_write(addr & 0xff00);
  spi_write(addr & 0x00ff);
  spi_write(data);
  mkm_chip_select(0);
}

static uint8_t
spi_read_byte(uint16_t addr)
{
  spi_setup(1);
  mkm_grab();	
  mkm_chip_select(1);
  spi_write(SPI_READ);
  spi_write(addr & 0xff00);
  spi_write(addr & 0x00ff);
  uint8_t data = spi_read();
  data = spi_read();
  data = spi_read();
  data = spi_read();
  data = spi_read();
  data = spi_read();
  data = spi_read();
  data = spi_read();
  data = spi_read();
  data = spi_read();
  data = spi_read();
 
  mkm_chip_select(0);
  mkm_release();
  spi_setup(0);
  
  return data;
}
#endif

/**********************/
/* Interrupts         */
/* Tamper protection. */
#if 1
/* Interrupt handler for panic switch. */
ISR (PCINT0_vect)
{
volatile int i =0;
}
#endif

#if 1
/* Interrupt handler for case tamper switches. */
ISR (INT0_vect)
{
	/*disable interrupts until memory is cleared */
	if(case_enable && configured == 0x55){
		cli();
		mkm_wipe();
		/*TAMP_ON turns on LED and provides falling edge signal to Pi to indicate tamper event */
		ssp_write(TAMP_ON);
		ssp_int_reset();   /*read INTFA reg to reset flag*/
		mkm_grab();		   /*prevents FPGA from accessing MKM */
		//To do: now disable case tamper (INT0_vect) interrupt until system is reset
		EIMSK = 0x00;
		sei();				/* re-enabling interrupts allows additional tamper trigger or tamper reset */
		
	}
	//sleep_disable();
}
#endif

/* Tamper reset. */
#if 0
/* Interrupt handler for tamper reset */
ISR (PCINT2_vect)
{
	/* tamper reset is triggered by a falling edge signal,
	this was due to the last minute changes that required this input
	and it was cleaner to do a pull-up on the AVR rather than a pull-up (or pull-down)
	in the external harness*/
	mkm_release();
	ssp_write(~TAMP_ON);
}
#endif

/* soft-UART RX */
#if 1
/* Detection of start bit */
ISR (PCINT1_vect)
{
	/* we need to do several things
	1. stop reacting to PINB2 changes until either;
		a.) 10 bit character is received or
		b.) we have a 10 bit timeout
	2. reload bit counter
	3. set timer A0 for a half-bit timer*/
	//PCMSK1 &= ~_BV(PCINT10); //was
	TCNT0 = 0x00;
	PCMSK1 &= ~_BV(PCINT11);
	//PCICR &= ~_BV(PCIE1);
	//PCIFR |= _BV(PCIF1);
	rcv_error_stop = 0;
	start_bit = 1;
	rcv_bit_count = 9;   // TBD 7 or 8? look for the stop bit?
	rcv_char = 0x00; //flush receive buffer
	receiving = 1;
	//OCR0A = 208;						// 208us compare value for 4800 baud
	
	TIMSK0 |= (1<<OCIE0A);              //if you want interrupt
	//PORTC ^= _BV(PORTC6);
}
#endif

static inline int
panic_p()
{
  return !bit_is_set(AVR_PANIC_PIN, AVR_PANIC_BIT);
}

static inline void
init_ports()
{
  /* Configure all PORTA pins except the tamper detection pin as
     outputs. */
  DDRA = 0xff & ~_BV(AVR_PANIC_BIT);
  //DDRB = 0xff & ~_BV(PINB3);
  //DDRB = 0xff & ~_BV(PINB2);
	DDRB = 0xf7; //was fb
}

static void
init_interrupts()
{
  //PCICR |= _BV(PCIE0|PCIE2);    /* Enable pin change interrupt on PCMSK0 & PCMSK2*/
  PCICR = (1<<PCIE0)|(1<<PCIE1)|(1<<PCIE2);
  PCMSK0 |= _BV(PCINT4);        /* Set mask bit for PCINT4 (panic) */
  /*setup PC6 as tamper disable */
  //PUEC = (1<<PUEC6);			/*enable internal pull-up to avoid false triggers */ 
  PORTC |= (1<<PORTC6);
  //PCMSK2 |= _BV(PCINT22);
  /*setup PB2 as soft-UART RX*/
  //PUEB = (1<<PUEB3) | (1<<PUEB2);			/*enable internal pull-up to detect start bit */
  PUEB = (1<<PUEB3);
  PCMSK1 |= _BV(PCINT11);
 // PCMSK1 |= _BV(PCINT10);
  sei();
}

static void 
init_tamper_values(uint8_t flags_set, uint8_t source)
{
	
	//if (source == 1) {
		//flags = eeprom_read_byte ((uint8_t *)TAMP_FLAGS);
	//}
	//else {
		flags = flags_set;
	//}
	if (flags & LIGHT) {
		light_enable = 1;
	}
	else {
		light_enable = 0;
	}
	if (flags & TEMP) {
		temp_enable = 1;
	}
	else {
		temp_enable = 0;
	}
	if (flags & VIBE) {
		vibe_enable = 1;
	}
	else{
		vibe_enable = 0;
	}
	if (flags & CASE) {
		case_enable = 1;
		init_int0();		/*enable INT0 */
	}
	else{
		case_enable = 0;
		EIMSK = 0x00;		/*dis-able INT0 */
	}
	/*light_thresh = eeprom_read_word((uint16_t *)LIGHT_PRE);
	temp_hi_thresh = eeprom_read_word((uint16_t *)TEMP_PRE_HI);
	temp_lo_thresh = eeprom_read_word((uint16_t *)TEMP_PRE_LO);
	vibe_hi_thresh = eeprom_read_byte ((uint8_t *) VIBE_PRE_HI);
	vibe_lo_thresh = eeprom_read_byte((uint8_t *) VIBE_PRE_HI);*/
	
}
static void
init_int0()
{
	//this has to be done after the SPI expander has been set up for the proper levels
	//
	/*lets use the INT0 pin as triggered form the SPI expander low level */
	EICRA = 0x00;   /* insure that we are interrupting on low-level */
	EIMSK = 0x01;		/*enable INT0 */
	//sei();
}



static void
init_power_reduction()
{
  /* TBD: Disable everything that we don't need? Note that the effect
     of this should marginal since it's only saving energy when we're
     awaken and actualy wiping memory. */
  //MAD not sure what the original developer meant by this.
}

/*static inline*/ void
mkm_wipe()
{
  AVR_LED_PORT |= _BV(AVR_LED_RED_BIT);

  spi_setup(1);
  mkm_grab();

  mkm_chip_select(1);
  spi_set_operation(SPI_OPERATION_SEQUENCE);
  mkm_chip_select(0);

  mkm_chip_select(1);
  spi_write(SPI_WRITE);
  spi_write(0);                    /* Address, high byte. */
  spi_write(0);                    /* Address, low byte. */
  for (int i = 0; i < 0x1fff; i++) /* 8192 bytes (64Kbit). */
    spi_write(0);
  mkm_chip_select(0);

  mkm_release();
  spi_setup(0);

  AVR_LED_PORT &= ~_BV(AVR_LED_RED_BIT);

  /* Flash blue LED three times to indicate wipe is done */
  for (int x = 0; x < 6; x++) {
    AVR_LED_PORT ^= _BV(AVR_LED_BLUE_BIT);
    for (int i = 0; i < 3200; i++);
  }
}



static inline void
sleep()
{
  SMCR &= ~0x2;               /* Sets and enables sleep mode to "power down"... */
  SMCR |= 0x05;                /* ... and enable sleep? seems redundant*/
  //CCP = 0xD8;
  asm("sei");                 /* Enable interrupts. */
  asm("sleep");               /* Go to sleep. */
  //sleep_cpu();			  /*MAD also redundant? ok, enough redundnacy already*/
 //SMCR &= ~0x1;               /* Disable sleep. Nah, let's not and say we did. */
}
int
main()
{
  mkm_grab();
  init_ports();
  init_power_reduction();
  init_interrupts();
  //mlx_get_calib();
  //init_tamper_values(0, 1);
	
  flags = 0xFF;
  configured = 0;
  tamper_detected = 0;
  uint8_t ssp_status = 0;
  sending = 0;
  receiving = 0;
  tx_char = 0;
  tx_bit_count = 0;
  light_fault = 0;
  vibe_fault = 0;
  ssp_fault = 0;
  n25_fault = 0;
  unk_fault = 0;
  PORTB |= _BV(PORTB3);   //set TX idle high
  wd_init = 0x01;
  ssp_out = WDOG_RS;
  PORTC |= _BV(PORTC6);
  TCNT0 = 0x00;
  TCCR0A = (1<<COM0A1) | (1 << WGM01);             //CTC mode
  TCCR0B = (1 << CS00);              //div1
  OCR0A = 153;						// 208us compare value for 4800 baud was 180
  
   TCNT1 = 0x00;
   TCCR1A = (1<<COM1A1) | (1 << WGM01);             //CTC mode
   TCCR1B = (1 << CS00);              //div1
   //OCR1A = 2000;						// 208us compare value for 4800 baud
   OCR1A = 2000;						// 208us compare value for 4800 baud
  spi_usart_setup(1);
 
  //put mlx to sleep as soon as possible to avoid wdog reset
 /* mlx_reset();
  mlx_reset();
 mlx_reset();
  mlx_request_sleep();
  mlx_confirm_sleep();
  mlx_nop();
  mlx_sleep();
  //ensure that adx is also at low power mode

  adx_wr_reg(ADX_POWER_CTL, 0x00);
  adx_rd_reg(ADX_STATUS);*/
 
  ssp_boot();
  ssp_read_byte();
  //setup directions for ssp pins
  ssp_setup();
  //cycle the TAMP_CLR to reset the relays and turn of LEDS
  //to save power. Keep WDOG signal in high state
  ssp_write(ssp_out|0x02);
  ssp_write(ssp_out);
  //setup the tamper monitoring input
  //ssp_int_config();
 uint8_t check =  ssp_read_byte();
  //init_int0();
  //mlx_sleep();
  // do not need to configure these for tamper switch only 
  adx_setup();
  mlx_reset();
 // mlx_setup();
  mlx_setup();
// mlx_write_reg(13, 0x94);
  mlx_start_meas();
  mlx_start_meas();
  mlx_start_meas();
  
  //initTimer1(TWD_INIT);
  
  mkm_release();
  /* Flash LED's at startup. */
  AVR_LED_PORT |= 0x0f;
  for (int i = 0; i < 16000; i++);
  AVR_LED_PORT &= ~0x0f;
  // go to sleep, interrupt will wake us
  //sleep();
  temperature = 80;
  light = 0xaa55;
  //init_int0();
  rcv_valid = 0;
  sei();
  while (1)
    {
		//n25_read(0x00, 0x00, 0x00, 8);
		//read in character and echo back
		if (rcv_valid == 1){
			rcv_valid = 0;
			start_bit = 1;
			process_message();
			send(0x15);
		}
		
		if (configured == 0x55) {
			check_usart_faults();
			/*read the ssp lines for events */
			//ssp_status = ssp_read_byte();
			//if ((ssp_status && LOW_VOLT)|(ssp_status && BATT_ON)) {
				//mkm_wipe();
			//}
			//read accelerometer 
			if ((adx_read_status() & ADX_ACT) && (vibe_enable)){
				fault_code = VIBE;
				mkm_wipe();
			}
			//}
			//read light and temp
			mlx_start_meas();
			mlx_get_meas();
			//if temp is out side normal storage or sensor is exposed to bright light
			//actual values 1TBD 
			if ((((temperature > temp_hi_thresh) | (temperature < temp_lo_thresh)) && temp_enable) ){
				fault_code = TEMP;
				fault_value1 = ((uint8_t) temperature>>8);
				fault_value2 = ((uint8_t) temperature&0xFF);
				mkm_wipe();
			}
			if((light > light_thresh) && light_enable ) {
				fault_code = LIGHT;
				fault_value1 = ((uint8_t) light>8);
				fault_value2 = ((uint8_t) light&0xFF);
				mkm_wipe();
			}
			//if (panic_p())
			//mkm_wipe();
			//sleep_enable();

			/* Sleep is not working right at the moment (the AVR seems to be reset
			when brought out of sleep mode by a press on the panic button).
			MAD 11/29/18 -Because the sleep power mode did not catch PCINTx type interrupts
			if (!panic_p())
			sleep();
			*/
		}//end if configured
    }//end while

  return 0;
}

void process_message(){
	if (rcv_char == SET_LIGHT) {
		int light_temp = 0;
		rcv_valid = 0;
		light_status = 1;
		while (!rcv_valid & !rcv_error_stop){ }
			
		light_temp = (uint16_t)rcv_char <<8;
		rcv_valid = 0;
		if (rcv_error_stop ==1) {
			rcv_error_stop = 0;
			light_status = 0;
			send(0x15);
		}
		else {
			light_temp = (uint16_t)rcv_char <<8;
		}
		
		while (!rcv_valid & !rcv_error_stop){ }
		rcv_valid = 0;
		if (rcv_error_stop) {
			light_status = 0;
			send(0x15);
		} 
		else{
			light_temp |= (uint16_t)rcv_char;
		}
		if (light_status){
			light_thresh = light_temp;
			//eeprom_write_word((uint16_t *)LIGHT_PRE, light_thresh);
			light_status = 0;
			send(0x14);
		}
	}
	if (rcv_char == SET_TEMP_HI) {
		rcv_valid = 0;
		//fix for 16 bit word
		while (!rcv_valid & !rcv_error_stop){ }
		rcv_valid = 0;
		if (rcv_error_stop ==1) {
			rcv_error_stop = 0;
			light_status = 0;
			send(0x15);
		}
		else {
			send(0x14);
			temp_hi_thresh = rcv_char;
			//eeprom_write_word((uint16_t *)TEMP_PRE_HI, temp_hi_thresh);
		}
	}
	if (rcv_char == SET_TEMP_LO) {
		rcv_valid = 0;
		//fix for 16 bit word
		while (!rcv_valid & !rcv_error_stop){ }
		rcv_valid = 0;
		if (rcv_error_stop ==1) {
			rcv_error_stop = 0;
			light_status = 0;
			send(0x15);
		}
		else {
			send(0x14);
			temp_hi_thresh = rcv_char;
			//eeprom_write_word((uint16_t *)TEMP_PRE_LO, temp_lo_thresh);
		}
	}
	if (rcv_char == SET_VIBE) {
		int vibe_temp_lo = 0;
		int vibe_temp_hi = 0;
		rcv_valid = 0;
		vibe_status = 1;
		while (!rcv_valid & !rcv_error_stop){ }
		
		vibe_temp_lo = rcv_char;
		rcv_valid = 0;
		if (rcv_error_stop ==1) {
			rcv_error_stop = 0;
			vibe_status = 0;
			send(0x15);
		}
		else {
			vibe_temp_lo = (uint16_t)rcv_char <<8;
		}
		
		while (!rcv_valid & !rcv_error_stop){ }
		rcv_valid = 0;
		if (rcv_error_stop) {
			vibe_status = 0;
			send(0x15);
		}
		else{
			vibe_temp_hi = rcv_char;
		}
		if (vibe_status){
			vibe_lo_thresh = vibe_temp_lo;
			vibe_hi_thresh = vibe_temp_hi;
			//eeprom_write_byte((uint8_t *)VIBE_PRE_HI, vibe_hi_thresh);
			//eeprom_write_byte((uint8_t *)VIBE_PRE_LO, vibe_lo_thresh);
			vibe_status = 0;
			adx_setup();
			send(0x14);
		}
	}
	if (rcv_char == ENA_TAMP) {
		rcv_valid = 0;
		//fix for 16 bit word
		while (!rcv_valid & !rcv_error_stop){ }
		rcv_valid = 0;
		if (rcv_error_stop ==1) {
			rcv_error_stop = 0;
			light_status = 0;
			send(0x15);
		}
		else {
			//uint8_t flags = eeprom_read_byte((uint8_t *)TAMP_FLAGS);
			flags |= rcv_char;
			init_tamper_values(flags, 0);
			send(0x14);
		}
	}
	if (rcv_char == DIS_TAMP) {
		rcv_valid = 0;
		//fix for 16 bit word
		while (!rcv_valid & !rcv_error_stop){ }
		rcv_valid = 0;
		if (rcv_error_stop ==1) {
			rcv_error_stop = 0;
			light_status = 0;
			send(0x15);
		}
		else {
			//uint8_t flags = eeprom_read_byte((uint8_t *)TAMP_FLAGS);
			flags &= ~rcv_char;
			init_tamper_values(flags, 0);
			send(0x14);
		}
	}
	if (rcv_char == CHK_LIGHT) {
		rcv_valid = 0;
		//fix for 16 bit word
		send (light>>8);
		send ((uint8_t) light&0xFF);
		send(0x14);
	}
	if (rcv_char == CHK_TEMP) {
		rcv_valid = 0;
		//fix for 16 bit word
		send (temperature>>8);
		send ((uint8_t)temperature&0xFF);
		send(0x14);
		
	}
	if (rcv_char == SET_CONFIG) {
		rcv_valid = 0;
		mkm_release();
		configured = 0x55;
		send(0x14);
	}
	
	if (rcv_char == CHK_FAULT) {
		rcv_valid = 0;
		send(fault_code);
		send(fault_value1);
		send(fault_value2);
		send(0x14);
	}
	
	if (rcv_char == CHK_TAMP) {
		rcv_valid = 0;
		if (tamper_detected == 1) {
			send(0x15);
		}
		else {
			send(0x14);
		}
	}
}

void send(uint8_t tx){
	tx_char = tx;
	tx_bit_count = 0;
	start_bit = 1;
	sending = 1;
	TCNT0 = 0x00;						//reset counter to avoid glitch on next rcv char.
	TIMSK0 |= (1<<OCIE0A);
	while(sending);
}

void check_usart_faults(){
	if ((light_fault > 20)&& (light_enable||temp_enable)){  //light and temp same chip
		fault_code = USART;
		fault_value1 = LIGHT;
		fault_value2 = 0x00;
		tamper_detected = 1;
	}
	if ((vibe_fault > 20) && vibe_enable){
		fault_code = USART;
		fault_value1 = VIBE;
		fault_value2 = 0x00;
		tamper_detected = 1;
	}
	if ((ssp_fault >20) && case_enable) {
		fault_code = USART;
		fault_value1 = SSP;
		fault_value2 = 0x00;
		tamper_detected = 1;
	}
	if (n25_fault >20) {
		fault_code = USART;
		fault_value1 = N25;
		fault_value2 = 0x00;
		tamper_detected = 1;
	}
	if (unk_fault >20) {
		fault_code = USART;
		fault_value1 = UNK;
		fault_value2 = 0x00;
		tamper_detected = 1;
	}
	
}