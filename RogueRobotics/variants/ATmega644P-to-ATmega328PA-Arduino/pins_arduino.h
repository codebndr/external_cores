/*
|| @author         Brett Hagman <bhagman@wiring.org.co>
|| @url            http://wiring.org.co/
|| @url            http://roguerobotics.com/
||
|| @description
|| | ATmega644P-to-ATmega328PA-Arduino
|| | Board Specific Definitions for:
|| |   Rogue Robotics LEDHead (ATmega644P)
|| |     http://www.roguerobotics.com/products/electronics/ledhead
|| |   Rogue Robotics rEDI (ATmega644P)
|| |     http://www.roguerobotics.com/products/electronics/redi
|| |   (Atmel AVR 8 bit microcontroller core)
|| #
||
|| @notes
|| | For legacy reasons, the pin ordering for these boards have been
|| | chosen to match, as closely as possible, with the original
|| | Arduino designed Atmel ATmega328P based boards (e.g. Duemilanove, UNO, etc.)
|| |
|| | Luckily, only two ports on the ATmegaxx4 series have to be mangled
|| | to match; ports A and B.  However, I2C compatibility is broken.
|| | Nothing could be done, since the '328P has I2C multiplexed
|| | on the ADC port. :(
|| #
||
|| @license See accompanying License.txt, or see http://wiring.org.co/
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>


#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           8

#define analogInputToDigitalPin(p)  ((p < 8) ? (p) + 14 : -1)
#define digitalPinHasPWM(p)         ((((p) >= 4) && ((p) <= 7)) || ((p) == 9) || ((p) == 10))

#define TOTAL_PINS              32
#define TOTAL_ANALOG_PINS       8
#define FIRST_ANALOG_PIN        14

#define WLED                    31

// SPI port
static const uint8_t SS   = 10;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

// TWI port
static const uint8_t SCL  = 24;
static const uint8_t SDA  = 25;

static const uint8_t LED_BUILTIN = 31;

// Analog pins
static const uint8_t A0 = 14;
static const uint8_t A1 = 15;
static const uint8_t A2 = 16;
static const uint8_t A3 = 17;
static const uint8_t A4 = 18;
static const uint8_t A5 = 19;
static const uint8_t A6 = 20;
static const uint8_t A7 = 21;

// External Interrupts
static const uint8_t EI0 = 2;
static const uint8_t EI1 = 3;
static const uint8_t EI2 = 8;

// Hardware Serial port pins
static const uint8_t RX0 = 0;
static const uint8_t TX0 = 1;
static const uint8_t RX1 = 2;
static const uint8_t TX1 = 3;


#ifndef NOT_AN_INTERRUPT
#define NOT_AN_INTERRUPT (-1)
#endif
#define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : ((p) == 8 ? 2 : NOT_AN_INTERRUPT)))

#define digitalPinToPCICR(p)    (((p) >= 0 && (p) < NUM_DIGITAL_PINS) ? (&PCICR) : ((uint8_t *)0))

#define digitalPinToPCICRbit(p) ( ((p) <= 7) ? 3 : \
                                ( ((p) <= 13) ? 1 : \
                                ( ((p) <= 21) ? 0 : \
                                ( ((p) <= 23) ? 1 : \
                                ( ((p) <= 31) ? 2 : \
                                0 ) ) ) ) )

#define digitalPinToPCMSK(p)    ( ((p) <= 7) ? &PCMSK3 : \
                                ( ((p) <= 13) ? &PCMSK1 : \
                                ( ((p) <= 21) ? &PCMSK0 : \
                                ( ((p) <= 23) ? &PCMSK1 : \
                                ( ((p) <= 31) ? &PCMSK2 : \
                                ((uint8_t *)0) ) ) ) ) )

#define digitalPinToPCMSKbit(p) ( ((p) <= 7) ? (p) : \
                                ( ((p) <= 13) ? (p - 6) : \
                                ( ((p) <= 21) ? (p - 14) : \
                                ( ((p) <= 23) ? (p - 22) : \
                                ( ((p) <= 31) ? (p - 24) : \
                                (0) ) ) ) ) )


#ifdef ARDUINO_MAIN
#define PA 1
#define PB 2
#define PC 3
#define PD 4


const uint16_t PROGMEM port_to_mode_PGM[] =
{
  NOT_A_PORT,
  (uint16_t) &DDRA,
  (uint16_t) &DDRB,
  (uint16_t) &DDRC,
  (uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] =
{
  NOT_A_PORT,
  (uint16_t) &PORTA,
  (uint16_t) &PORTB,
  (uint16_t) &PORTC,
  (uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] =
{
  NOT_A_PORT,
  (uint16_t) &PINA,
  (uint16_t) &PINB,
  (uint16_t) &PINC,
  (uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] =
{
  PD,  // 0
  PD,
  PD,
  PD,
  PD,
  PD,
  PD,
  PD,

  PB,  // 8
  PB,
  PB,
  PB,
  PB,
  PB,
  PA,
  PA,

  PA,  // 16
  PA,
  PA,
  PA,
  PA,
  PA,
  PB,
  PB,

  PC,  // 24
  PC,
  PC,
  PC,
  PC,
  PC,
  PC,
  PC
};


const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] =
{
  _BV(0), // PD - 0
  _BV(1), // PD
  _BV(2), // PD
  _BV(3), // PD
  _BV(4), // PD
  _BV(5), // PD
  _BV(6), // PD
  _BV(7), // PD

  _BV(2), // PB - 8
  _BV(3), // PB
  _BV(4), // PB
  _BV(5), // PB
  _BV(6), // PB
  _BV(7), // PB
  _BV(0), // PA
  _BV(1), // PA

  _BV(2), // PA - 16
  _BV(3), // PA
  _BV(4), // PA
  _BV(5), // PA
  _BV(6), // PA
  _BV(7), // PA
  _BV(0), // PB
  _BV(1), // PB

  _BV(0), // PC - 24
  _BV(1), // PC
  _BV(2), // PC
  _BV(3), // PC
  _BV(4), // PC
  _BV(5), // PC
  _BV(6), // PC
  _BV(7), // PC
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] =
{
  NOT_ON_TIMER, // PD0 - 0
  NOT_ON_TIMER, // PD1
  NOT_ON_TIMER, // PD2
  NOT_ON_TIMER, // PD3
  TIMER1B,      // PD4
  TIMER1A,      // PD5
  TIMER2B,      // PD6
  TIMER2A,      // PD7

  NOT_ON_TIMER, // PB2 - 8
  TIMER0A,      // PB3
  TIMER0B,      // PB4
  NOT_ON_TIMER, // PB5
  NOT_ON_TIMER, // PB6
  NOT_ON_TIMER, // PB7
  NOT_ON_TIMER, // PA0
  NOT_ON_TIMER, // PA1

  NOT_ON_TIMER, // PA2 - 16
  NOT_ON_TIMER, // PA3
  NOT_ON_TIMER, // PA4
  NOT_ON_TIMER, // PA5
  NOT_ON_TIMER, // PA6
  NOT_ON_TIMER, // PA7
  NOT_ON_TIMER, // PB0
  NOT_ON_TIMER, // PB1
  
  NOT_ON_TIMER, // PA0 - 24
  NOT_ON_TIMER, // PA1
  NOT_ON_TIMER, // PA2
  NOT_ON_TIMER, // PA3
  NOT_ON_TIMER, // PA4
  NOT_ON_TIMER, // PA5
  NOT_ON_TIMER, // PA6
  NOT_ON_TIMER, // PA7
};

#endif // ARDUINO_MAIN

#endif // Pins_Arduino_h
