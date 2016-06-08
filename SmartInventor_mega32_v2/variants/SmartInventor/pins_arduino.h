/* ------------------------------------------------------*
*                                                        *
* Copyright (c) 2013, Jarek Zok <jarek.zok@fwioo.pl>     *
* All rights reserved.                                   *
*                                                        *
* This program is free software; you can redistribute it *
* and/or modify it under the terms of the GNU General    *
* Public License as published by the Free Software       *
* Foundation; either version 2 of the License, or        *
* (at your option) any later version.                    *
*                                                        *
* This program is distributed in the hope that it will   *
* be useful, but WITHOUT ANY WARRANTY; without even the  *
* implied warranty of MERCHANTABILITY or FITNESS FOR A   *
* PARTICULAR PURPOSE.  See the GNU General Public        *
* License for more details.                              *
*                                                        *
* You should have received a copy of the GNU General     *
* Public License along with this program; if not, write  *
* to the Free Software Foundation, Inc.,                 *
* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA *
*                                                        *
* Licence can be viewed at                               *
* http://www.fsf.org/licenses/gpl.txt                    *
*                                                        *
*                                                        *
**********************************************************/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS		32
#define NUM_ANALOG_INPUTS		8
#define analogInputToDigitalPin(p)	((p < 8) ? (p) + 19 : -1)
#define digitalPinHasPWM(p)	((p) == 29 || (p) == 30 || (p) == 5 || (p) == 7)
#define TIMER0		8 // available on ATMega32/644
//extern const int  analog_pin_to_pin[];
#define analogPinToChannel(p)	((p) - 19)

const static uint8_t SS   = 6;
const static uint8_t MOSI = 8;
const static uint8_t MISO = 9;
const static uint8_t SCK  = 10;

#define SDA 11
#define SCL 12

const static uint8_t A0 = 19;     
const static uint8_t A1 = 20;
const static uint8_t A2 = 21;
const static uint8_t A3 = 22;
const static uint8_t A4 = 23;
const static uint8_t A5 = 24;
const static uint8_t A6 = 25;
const static uint8_t A7 = 26;

#ifdef ARDUINO_MAIN

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	
	//uart
	PD  , // Digital 0 ** PD 0 ** DIP 14 ** USART0_RX	
	PD  , // Digital 1 ** PD 1 ** DIP 15 ** USART0_TX	
	
	//dc motor - A , B
	PB  , // Digital 2 ** PB 0 ** DIP 1  ** XCK            USABLE 
	PB  , // Digital 3 ** PB 1 ** DIP 2  ** T1             USABLE 
	PB  , // Digital 4 ** PB 2 ** DIP 3  ** INT2/AIN0  
	PB  , // Digital 5 ** PB 3 ** DIP 4  ** OC0/AIN1  
	
	//ir receiver
	PB  , // Digital 6 ** PB 4 ** DIP 5  ** SS
	
	// buzzer
	PD  , // Digital 7 ** PD 7 ** DIP 21 ** OC2            USABLE 
	
	//dip s/w
	PB  , // Digital 8 ** PB 5 ** DIP 6  ** MOSI
	PB  , // Digital 9 ** PB 6 ** DIP 7  ** MISO
	PB  , // Digital10 ** PB 7 ** DIP 8  ** SCK
	
	//Top display LED, Bottom Sensor
	PC  , // Digital11 ** PC 0 ** DIP 22 **                USABLE 
	PC  , // Digital12 ** PC 1 ** DIP 23 **                USABLE 
	PC  , // Digital13 ** PC 2 ** DIP 24 **                USABLE 
	PC  , // Digital14 ** PC 3 ** DIP 25 **                USABLE 
	PC  , // Digital15 ** PC 4 ** DIP 26 **                USABLE 
	PC  , // Digital16 ** PC 5 ** DIP 27 **                USABLE 
	PC  , // Digital17 ** PC 6 ** DIP 28 **                USABLE 
	PC  , // Digital18 ** PC 7 ** DIP 29 **                USABLE 
				
	//ADC , S1,S2,S3 + A3 ~ A7
	PA  , // Digital19 ** PA 0 ** DIP 40 **                USABLE 
	PA  , // Digital20 ** PA 1 ** DIP 39 **                USABLE 
	PA  , // Digital21 ** PA 2 ** DIP 38 **                USABLE 
	PA  , // Digital22 ** PA 3 ** DIP 37 **                USABLE 
	PA  , // Digital23 ** PA 4 ** DIP 36 **                USABLE 
	PA  , // Digital24 ** PA 5 ** DIP 35 **                USABLE 
	PA  , // Digital25 ** PA 6 ** DIP 34 **                USABLE 
	PA  , // Digital26 ** PA 7 ** DIP 33 **                USABLE 
	
	//SERVO
	PD  , // Digital27 ** PD 2 ** DIP 16 ** INT0 
	PD  , // Digital28 ** PD 3 ** DIP 17 ** INT1
	PD  , // Digital29 ** PD 4 ** DIP 18 ** OC1B           USABLE 
	PD  , // Digital30 ** PD 5 ** DIP 19 ** OC1A           USABLE 
	PD  , // Digital31 ** PD 6 ** DIP 20 ** ICP1           USABLE 

};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	//uart
	_BV(0), // Digital 0 ** PD 0 ** DIP 14 ** USART0_RX	
	_BV(1),	// Digital 1 ** PD 1 ** DIP 15 ** USART0_TX	
	
	//dc motor - A , B
	_BV(0), // Digital 2 ** PB 0 ** DIP 1  ** XCK            USABLE 
	_BV(1), // Digital 3 ** PB 1 ** DIP 2  ** T1             USABLE 
	_BV(2), // Digital 4 ** PB 2 ** DIP 3  ** INT2/AIN0  
	_BV(3), // Digital 5 ** PB 3 ** DIP 4  ** OC0/AIN1  
	
	//ir receiver
	_BV(4), // Digital 6 ** PB 4 ** DIP 5  ** SS
	
	// buzzer
	_BV(7), // Digital 7 ** PD 7 ** DIP 21 ** OC2            USABLE 
	
	//dip s/w
	_BV(5), // Digital 8 ** PB 5 ** DIP 6  ** MOSI
	_BV(6), // Digital 9 ** PB 6 ** DIP 7  ** MISO
	_BV(7), // Digital10 ** PB 7 ** DIP 8  ** SCK
	
	//Top display LED, Bottom Sensor
	_BV(0), // Digital11 ** PC 0 ** DIP 22 **                USABLE 
	_BV(1), // Digital12 ** PC 1 ** DIP 23 **                USABLE 
	_BV(2), // Digital13 ** PC 2 ** DIP 24 **                USABLE 
	_BV(3), // Digital14 ** PC 3 ** DIP 25 **                USABLE 
	_BV(4), // Digital15 ** PC 4 ** DIP 26 **                USABLE 
	_BV(5), // Digital16 ** PC 5 ** DIP 27 **                USABLE 
	_BV(6), // Digital17 ** PC 6 ** DIP 28 **                USABLE 
	_BV(7), // Digital18 ** PC 7 ** DIP 29 **                USABLE 
	
	//ADC , S1,S2,S3 + A3 ~ A7
	_BV(0), // Digital19 ** PA 0 ** DIP 40 **                USABLE 
	_BV(1), // Digital20 ** PA 1 ** DIP 39 **                USABLE 
	_BV(2), // Digital21 ** PA 2 ** DIP 38 **                USABLE 
	_BV(3), // Digital22 ** PA 3 ** DIP 37 **                USABLE 
	_BV(4), // Digital23 ** PA 4 ** DIP 36 **                USABLE 
	_BV(5), // Digital24 ** PA 5 ** DIP 35 **                USABLE 
	_BV(6), // Digital25 ** PA 6 ** DIP 34 **                USABLE 
	_BV(7), // Digital26 ** PA 7 ** DIP 33 **                USABLE 
	
	//SERVO
	_BV(2), // Digital27 ** PD 2 ** DIP 16 ** INT0 
	_BV(3), // Digital28 ** PD 3 ** DIP 17 ** INT1
	_BV(4), // Digital29 ** PD 4 ** DIP 18 ** OC1B           USABLE 
	_BV(5), // Digital30 ** PD 5 ** DIP 19 ** OC1A           USABLE 
	_BV(6), // Digital31 ** PD 6 ** DIP 20 ** ICP1           USABLE 
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER,    /* 0 - PD0 */
	NOT_ON_TIMER,	 /* 1 - PD1 */
	NOT_ON_TIMER,	 /* 2 - PD2 */
	NOT_ON_TIMER,    /* 3 - PD3 */
	NOT_ON_TIMER,         /* 4 - PD4 */
	TIMER0,         /* 5 - PD5 */
	NOT_ON_TIMER,    /* 6 - PD6 */
	TIMER2,          /* 7 - PD7 */	
	NOT_ON_TIMER,    /* 8 - PB0 */
	NOT_ON_TIMER,    /* 9 - PB1 */
   	 NOT_ON_TIMER,    /* 10 - PB2 */
	NOT_ON_TIMER,       	 /* 11 - PB3 --- TIMER OC0 */
	NOT_ON_TIMER, /* 12 - PB4 */
	NOT_ON_TIMER, /* 13 - PB5 */
	NOT_ON_TIMER, /* 14 - PB6 */
	NOT_ON_TIMER, /* 15 - PB7 */
	NOT_ON_TIMER, /* 16 - port C */
	NOT_ON_TIMER, /* 17 - PC1 */
	NOT_ON_TIMER, /* 18 - PC2 */
	NOT_ON_TIMER, /* 19 - PC3 */
	NOT_ON_TIMER, /* 20 - PC4 */
	NOT_ON_TIMER, /* 21 - PC5 */
	NOT_ON_TIMER, /* 22 - PC6 */
	NOT_ON_TIMER, /* 23 - PC7 */
	NOT_ON_TIMER, /* 24, port A */
	NOT_ON_TIMER, /* 25 - PA1 */
	NOT_ON_TIMER, /* 26 - PA2 */
	NOT_ON_TIMER, /* 27 - PA3 */
	NOT_ON_TIMER, /* 28 - PA4 */
	TIMER1B, /* 29 - PA5 */
	TIMER1A, /* 30 - PA6 */
	NOT_ON_TIMER, /* 31 - PA7 */
};

const int analog_pin_to_pin[] = {
	0,		// pin 0
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,		// pin 10
	0,		// pin 11
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,		// 8, 16, 24
	1,		// 9, 17, 25
	2,		// 10, 18
	3,		// 11, 19, 27
	4,		// 12, 20, 28
	5,		// 13, 21
	6,		// 14, 22
	7,		// 15, 23
	0,
	0,
	0,
	0		// pin 30
};

#endif
#endif
