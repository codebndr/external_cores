/*
|| @author         Brett Hagman <bhagman@wiring.org.co>
|| @url            http://wiring.org.co/
||
|| @description
|| | Board Specific Definitions for:
|| |   Wiring S Play Shield (ATmega644P)
|| |   (Atmel AVR 8 bit microcontroller core)
|| |
|| | Note: The labelling on the PCB is incorrect for a few pins:
|| | SW1 should be connected to pin 28 (not 16)
|| | SW2 should be connected to pin 29 (not 17)
|| | D1 should be connected to pin 24 (not 18)
|| | D2 should be connected to pin 25 (not 19)
|| #
||
|| @license Please see http://wiring.org.co/
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           8
#define analogInputToDigitalPin(p)  ((p < 6) ? (p) + 14 : ((p < 8) ? (p) + 24 : -1))

#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 7 || (p) == 9 || (p) == 10)

#define TOTAL_PINS              32
#define TOTAL_ANALOG_PINS       8
#define FIRST_ANALOG_PIN        14

#define WLED                    23

// SPI port
static const uint8_t SS   = 10;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

// TWI port
static const uint8_t SCL  = 26;
static const uint8_t SDA  = 27;

static const uint8_t LED_BUILTIN = 23;

// Analog pins
static const uint8_t A0 = 14;
static const uint8_t A1 = 15;
static const uint8_t A2 = 16;
static const uint8_t A3 = 17;
static const uint8_t A4 = 18;
static const uint8_t A5 = 19;
static const uint8_t A6 = 30;
static const uint8_t A7 = 31;

// External Interrupts
static const uint8_t EI0 = 2;
static const uint8_t EI1 = 4;
static const uint8_t EI2 = 8;

// Hardware Serial port pins
static const uint8_t RX0 = 0;
static const uint8_t TX0 = 1;
static const uint8_t RX1 = 2;
static const uint8_t TX1 = 4;

#define digitalPinToPCICR(p)    ( ((p) >= 0 && (p) <= 31) ? (&PCICR) : ((uint8_t *)0) )

#define digitalPinToPCICRbit(p) ( ((p) <= 7) ? 3 : \
                                ( ((p) <= 13) ? 1 : \
                                ( ((p) <= 19) ? 0 : \
                                ( ((p) <= 27) ? 2 : \
                                ( ((p) <= 29) ? 1 : \
                                0 ) ) ) ) )

#define digitalPinToPCMSK(p)    ( ((p) <= 7) ? &PCMSK3 : \
                                ( ((p) <= 13) ? &PCMSK1 : \
                                ( ((p) <= 19) ? &PCMSK0 : \
                                ( ((p) <= 27) ? &PCMSK2 : \
                                ( ((p) <= 29) ? &PCMSK1 : \
                                ((uint8_t *)0) ) ) ) ) )

#define digitalPinToPCMSKbit(p) ( ((p) <= 7) ? (p) : \
                                ( ((p) <= 13) ? (p - 6) : \
                                ( ((p) <= 19) ? (p - 14) : \
                                ( ((p) <= 25) ? (p - 22) : \
                                ( ((p) <= 27) ? (p - 26) : \
                                ( ((p) <= 29) ? (p - 28) : \
                                ((p) - 24) ) ) ) ) ) )


#ifndef NOT_AN_INTERRUPT
#define NOT_AN_INTERRUPT (-1)
#endif
#define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 4 ? 1 : ((p) == 8 ? 2 : NOT_AN_INTERRUPT)))

#ifdef ARDUINO_MAIN

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
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
  // PORTLIST
  // -------------------------------------------
  PD  , // PD 0 ** 0 ** USART0_RX
  PD  , // PD 1 ** 1 ** USART0_TX
  PD  , // PD 2 ** 2 ** USART1_RX
  PD  , // PD 4 ** 3 ** PWM
  PD  , // PD 3 ** 4 ** USART1_TX
  PD  , // PD 5 ** 5 ** PWM
  PD  , // PD 6 ** 6 ** PWM
  PD  , // PD 7 ** 7 ** PWM
  PB  , // PB 2 ** 8 ** D8
  PB  , // PB 3 ** 9 ** PWM
  PB  , // PB 4 ** 10 ** SPI_SS
  PB  , // PB 5 ** 11 ** SPI_MOSI
  PB  , // PB 6 ** 12 ** SPI_MISO
  PB  , // PB 7 ** 13 ** SPI_SCK
  PA  , // PA 0 ** 14 ** ADC0
  PA  , // PA 1 ** 15 ** ADC1
  PA  , // PA 2 ** 16 ** ADC2
  PA  , // PA 3 ** 17 ** ADC3
  PA  , // PA 4 ** 18 ** ADC4
  PA  , // PA 5 ** 19 ** ADC5
  PC  , // PC 2 ** 20 ** D20
  PC  , // PC 3 ** 21 ** D21
  PC  , // PC 4 ** 22 ** D22
  PC  , // PC 5 ** 23 ** D23
  PC  , // PC 6 ** 24 ** D24
  PC  , // PC 7 ** 25 ** D25
  PC  , // PC 0 ** 26 ** I2C_SCL
  PC  , // PC 1 ** 27 ** I2C_SDA
  PB  , // PB 0 ** 28 ** D28
  PB  , // PB 1 ** 29 ** D29
  PA  , // PA 6 ** 30 ** D30/ADC6
  PA    // PA 7 ** 31 ** D31/ADC7
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] =
{
  _BV(0), // PD 0 ** 0 ** USART0_RX
  _BV(1), // PD 1 ** 1 ** USART0_TX
  _BV(2), // PD 2 ** 2 ** USART1_RX
  _BV(4), // PD 4 ** 3 ** PWM
  _BV(3), // PD 3 ** 4 ** USART1_TX
  _BV(5), // PD 5 ** 5 ** PWM
  _BV(6), // PD 6 ** 6 ** PWM
  _BV(7), // PD 7 ** 7 ** PWM
  _BV(2), // PB 2 ** 8 ** D8
  _BV(3), // PB 3 ** 9 ** PWM
  _BV(4), // PB 4 ** 10 ** SPI_SS
  _BV(5), // PB 5 ** 11 ** SPI_MOSI
  _BV(6), // PB 6 ** 12 ** SPI_MISO
  _BV(7), // PB 7 ** 13 ** SPI_SCK
  _BV(0), // PA 0 ** 14 ** ADC0
  _BV(1), // PA 1 ** 15 ** ADC1
  _BV(2), // PA 2 ** 16 ** ADC2
  _BV(3), // PA 3 ** 17 ** ADC3
  _BV(4), // PA 4 ** 18 ** ADC4
  _BV(5), // PA 5 ** 19 ** ADC5
  _BV(4), // PC 4 ** 20 ** D20
  _BV(5), // PC 5 ** 21 ** D21
  _BV(6), // PC 6 ** 22 ** D22
  _BV(7), // PC 7 ** 23 ** D23
  _BV(2), // PC 2 ** 24 ** D24
  _BV(3), // PC 3 ** 25 ** D25
  _BV(0), // PC 0 ** 26 ** I2C_SCL
  _BV(1), // PC 1 ** 27 ** I2C_SDA
  _BV(0), // PB 0 ** 28 ** D28
  _BV(1), // PB 1 ** 29 ** D29
  _BV(6), // PA 6 ** 30 ** D30/ADC6
  _BV(7)  // PA 7 ** 31 ** D31/ADC7
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] =
{
  NOT_ON_TIMER,   // PD 0 ** 0 ** USART0_RX
  NOT_ON_TIMER,   // PD 1 ** 1 ** USART0_TX
  NOT_ON_TIMER,   // PD 2 ** 2 ** USART1_RX
  TIMER1B,        // PD 4 ** 3 ** PWM
  NOT_ON_TIMER,   // PD 3 ** 4 ** USART1_TX
  TIMER1A,        // PD 5 ** 5 ** PWM
  TIMER2B,        // PD 6 ** 6 ** PWM
  TIMER2A,        // PD 7 ** 7 ** PWM
  NOT_ON_TIMER,   // PB 2 ** 8 ** D8
  TIMER0A,        // PB 3 ** 9 ** PWM
  TIMER0B,        // PB 4 ** 10 ** SPI_SS
  NOT_ON_TIMER,   // PB 5 ** 11 ** SPI_MOSI
  NOT_ON_TIMER,   // PB 6 ** 12 ** SPI_MISO
  NOT_ON_TIMER,   // PB 7 ** 13 ** SPI_SCK
  NOT_ON_TIMER,   // PA 0 ** 14 ** ADC0
  NOT_ON_TIMER,   // PA 1 ** 15 ** ADC1
  NOT_ON_TIMER,   // PA 2 ** 16 ** ADC2
  NOT_ON_TIMER,   // PA 3 ** 17 ** ADC3
  NOT_ON_TIMER,   // PA 4 ** 18 ** ADC4
  NOT_ON_TIMER,   // PA 5 ** 19 ** ADC5
  NOT_ON_TIMER,   // PC 2 ** 20 ** D20
  NOT_ON_TIMER,   // PC 3 ** 21 ** D21
  NOT_ON_TIMER,   // PC 4 ** 22 ** D22
  NOT_ON_TIMER,   // PC 5 ** 23 ** D23
  NOT_ON_TIMER,   // PC 6 ** 24 ** D24
  NOT_ON_TIMER,   // PC 7 ** 25 ** D25
  NOT_ON_TIMER,   // PC 0 ** 26 ** I2C_SCL
  NOT_ON_TIMER,   // PC 1 ** 27 ** I2C_SDA
  NOT_ON_TIMER,   // PB 0 ** 28 ** D28
  NOT_ON_TIMER,   // PB 1 ** 29 ** D29
  NOT_ON_TIMER,   // PA 6 ** 30 ** D30/ADC6
  NOT_ON_TIMER    // PA 7 ** 31 ** D31/ADC7
};

#endif // ARDUINO_MAIN

#endif // Pins_Arduino_h
