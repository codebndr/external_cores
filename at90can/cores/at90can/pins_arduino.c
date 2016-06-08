/*
  pins_arduino.c - pin definitions for the Arduino board
  Part of Arduino / Wiring Lite

  Copyright (c) 2005 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: pins_arduino.c 565 2009-03-25 10:50:00Z dmellis $
*/

#include <avr/io.h>
#include "wiring_private.h"
#include "pins_arduino.h"

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.


#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7


#define REPEAT8(x) x, x, x, x, x, x, x, x
#define BV0TO7 _BV(0), _BV(1), _BV(2), _BV(3), _BV(4), _BV(5), _BV(6), _BV(7)
#define BV7TO0 _BV(7), _BV(6), _BV(5), _BV(4), _BV(3), _BV(2), _BV(1), _BV(0)

const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	&DDRA,
	&DDRB,
	&DDRC,
	&DDRD,
	&DDRE,
	&DDRF,
	&DDRG,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	&PORTA,
	&PORTB,
	&PORTC,
	&PORTD,
	&PORTE,
	&PORTF,
	&PORTG,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PIN,
	&PINA,
	&PINB,
	&PINC,
	&PIND,
	&PINE,
	&PINF,
	&PING,
};

//Port for OSEMO Pelican
const uint8_t PROGMEM digital_pin_to_port_PGM[] = {	
	PD, // Serial1 RX  0
	PD, // Serial1 TX  1
	PE,
	PE,
	PE,
	PE,
	PE,
	PE,
	PD, /* D8*/
	PD,
	PB, 
	PB, // MOSI  11
	PB, // MISO  12
	PB, // SCK   13
	PG,
	PG,	
	PB, /* D16*/
	PD, // RXCAN  17
	PD, // TXCAN  18
	PG, // STATUS_LED  19
	PD, //SDA 20
	PD, //SCL 21
	PF,
	PF,
	PA, /* D24*/
	PA,
	PA,
	PA,
	PE, // Serial0 RX  28
	PE, // Serial0 TX  29
	PF, 
	PF,
	PF, /* D32*/
	PF,
	PF,
	PF, //35
	PB, // BoardX SS 36
	PB, // ? PB7
	PB, // ? PB6
	PG, // ? PG3
	PA, /* D40*/
	PA,
	PC,
	PC,
	PC,
	PC, /* D45*/
	PG, //? PG4
	PG, //? PG5
	PG, //? PG6
	PG, //? PG7
	PA, /* D50*/
	PA,
	PC,
	PC,
	PC,
	PC,
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {	
	_BV(2), // D
	_BV(3), // D
	_BV(2), // E
	_BV(3), // E
	_BV(4), // E
	_BV(5), // E
	_BV(6), // E
	_BV(7), // E	
	_BV(4), // D /* 8 */
	_BV(7), // D
	_BV(4), // B
	_BV(2), // B
	_BV(3), // B
	_BV(1), // B
	_BV(0), // G
	_BV(1),	// G
	_BV(5), // B /* 16  */
	_BV(6), // D
	_BV(5), // D
	_BV(2), // G
	_BV(1), // D
	_BV(0), // D
	_BV(7), // F
	_BV(6),	// F
	_BV(3), // A /* 24  */
	_BV(2), // A
	_BV(1), // A
	_BV(0), // A
	_BV(0), // E
	_BV(1), // E
	_BV(0), // F
	_BV(1), // F	
	_BV(2), // F /* 32  */
	_BV(3), // F
	_BV(4), // F
	_BV(5), // F
	_BV(0), // B
	_BV(7), // B
	_BV(6), // B
	_BV(3),	// G
	_BV(5), // A /* 40  */
	_BV(7), // A
	_BV(6), // C
	_BV(4), // C
	_BV(2), // C
	_BV(0), // C /* 45 */
	_BV(4),	// G
	_BV(5),	// G
	_BV(6),	// G	
	_BV(7),	// G
	_BV(4), // A /* 50 */
	_BV(6), // A
	_BV(7),	// C
	_BV(5), // C	
	_BV(3), // C
	_BV(1), // C	
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER, /* 0 PORT A */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER3A,
	TIMER3B,
	TIMER3C,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 8 PORT B */
	NOT_ON_TIMER,
	TIMER2A,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER1A,      /* 16 PORT C */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 24 PORT D */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 32 PORT E */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER1C,
	TIMER1B,
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 40 PORT G */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 45 PORT F */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER, /* 50 */
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	NOT_ON_TIMER,
};
