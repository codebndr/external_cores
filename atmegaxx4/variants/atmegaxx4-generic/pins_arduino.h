/*
|| @author         Brett Hagman <bhagman@wiring.org.co>
|| @url            http://wiring.org.co/
||
|| @description
|| | Pin definition for Atmel ATmegaxx4 series.
|| | (ATmega164PA, ATmega324PA, ATmega644PA, ATmega1284P)
|| #
||
|| @license See accompanying License.txt, or see http://wiring.org.co/
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>


/*
Pin Information for Wiring S (Generic ATmega644PA layout)


Pin Port F1        F2        F3        F4        Mask Notes
  0   0  RX0       PCINT24                       0x01 PORTD0
  1   0  TX0       PCINT25                       0x02 PORTD1
  2   0  RX1       PCINT26   EI0                 0x04 PORTD2,External Interrupt 0
  3   0  TX1       PCINT27   EI1                 0x08 PORTD3,External Interrupt 1
  4   0  PWM       PCINT28   XCK1                0x10 PORTD4,OC1B,SPI1 SCK (Master only)
  5   0  PWM       PCINT29                       0x20 PORTD5,OC1A
  6   0  PWM       PCINT30   ICP                 0x40 PORTD6,OC2B
  7   0  PWM       PCINT31                       0x80 PORTD7,OC1A
  8   1  SCL       PCINT16                       0x01 PORTC0
  9   1  SDA       PCINT17                       0x02 PORTC1
 10   1            PCINT18   TCK                 0x04 PORTC2,JTAG
 11   1            PCINT19   TMS                 0x08 PORTC3,JTAG
 12   1            PCINT20   TDO                 0x10 PORTC4,JTAG
 13   1            PCINT21   TDI                 0x20 PORTC5,JTAG
 14   1            PCINT22   TOSC1               0x40 PORTC6,External Timer Oscillator
 15   1  WLED      PCINT23   TOSC2               0x80 PORTC7,External Timer Oscillator,On-board LED
 16   2            PCINT8    XCK0      T0        0x01 PORTB0,SPI0 SCK (Master only),Timer 0 input
 17   2            PCINT9    CLKO      T1        0x02 PORTB1,Divided System Clock Output,Timer 1 input
 18   2            PCINT10   EI2       AIN0      0x04 PORTB2,External Interrupt 2,Analog Comparator Positive Input
 19   2  PWM       PCINT11             AIN1      0x08 PORTB3,OC0A,Analog Comparator Negative Input
 20   2  PWM       PCINT12   ~SS                 0x10 PORTB4,OC0B,SPI Slave Select Input
 21   2            PCINT13   MOSI                0x20 PORTB5,SPI Master Output/Slave Input
 22   2  *         PCINT14   MISO                0x40 PORTB6,SPI Master Input/Slave Output
 23   2  *         PCINT15   SCK                 0x80 PORTB7,SPI Master Clock
 24   3            PCINT0              ADC0      0x01 PORTA0,Analog Input 0
 25   3            PCINT1              ADC1      0x02 PORTA1,Analog Input 1
 26   3            PCINT2              ADC2      0x04 PORTA2,Analog Input 2
 27   3            PCINT3              ADC3      0x08 PORTA3,Analog Input 3
 28   3            PCINT4              ADC4      0x10 PORTA4,Analog Input 4
 29   3            PCINT5              ADC5      0x20 PORTA5,Analog Input 5
 30   3            PCINT6              ADC6      0x40 PORTA6,Analog Input 6
 31   3            PCINT7              ADC7      0x80 PORTA7,Analog Input 7

 * These pins have the ability to be PWM outputs on the ATmega1284P only.

*/

#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           8

#define analogInputToDigitalPin(p)  ((p < 8) ? (p) + 24 : -1)
#define digitalPinHasPWM(p)         ((((p) > 3) && ((p) < 8)) || ((p) == 19) || ((p) == 20))

#define TOTAL_PINS              32
#define TOTAL_ANALOG_PINS       8
#define FIRST_ANALOG_PIN        24

#define WLED                    15

static const uint8_t SS   = 20;
static const uint8_t MOSI = 21;
static const uint8_t MISO = 22;
static const uint8_t SCK  = 23;

static const uint8_t SDA = 9;
static const uint8_t SCL = 8;

static const uint8_t LED_BUILTIN = 15;

static const uint8_t A0 = 24;
static const uint8_t A1 = 25;
static const uint8_t A2 = 26;
static const uint8_t A3 = 27;
static const uint8_t A4 = 28;
static const uint8_t A5 = 29;
static const uint8_t A6 = 30;
static const uint8_t A7 = 31;

#define digitalPinToPCICR(p)    (((p) >= 0 && (p) < NUM_DIGITAL_PINS) ? (&PCICR) : ((uint8_t *)0))

#define digitalPinToPCICRbit(p) (3 - ((p) / 8))

#define digitalPinToPCMSK(p)    ((p) < 8 ? (&PCMSK3) : ((p) < 16 ? (&PCMSK2) : ((p) < 24 ? (&PCMSK1) : ((p) < 32 ? (&PCMSK0) : (uint8_t *)0))))
#define digitalPinToPCMSKbit(p) (((p) >= 0 && (p) < NUM_DIGITAL_PINS) ? ((p) % 8) : 0)

#ifndef NOT_AN_INTERRUPT
#define NOT_AN_INTERRUPT (-1)
#endif
#define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : ((p) == 18 ? 2 : NOT_AN_INTERRUPT)))

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
  PD,
  PD,
  PD,
  PD,
  PD,
  PD,
  PD,
  PD,

  PC,
  PC,
  PC,
  PC,
  PC,
  PC,
  PC,
  PC,

  PB,
  PB,
  PB,
  PB,
  PB,
  PB,
  PB,
  PB,

  PA,
  PA,
  PA,
  PA,
  PA,
  PA,
  PA,
  PA
};


const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] =
{
  _BV(0), // PD
  _BV(1), // PD
  _BV(2), // PD
  _BV(3), // PD
  _BV(4), // PD
  _BV(5), // PD
  _BV(6), // PD
  _BV(7), // PD

  _BV(0), // PC
  _BV(1), // PC
  _BV(2), // PC
  _BV(3), // PC
  _BV(4), // PC
  _BV(5), // PC
  _BV(6), // PC
  _BV(7), // PC

  _BV(0), // PB
  _BV(1), // PB
  _BV(2), // PB
  _BV(3), // PB
  _BV(4), // PB
  _BV(5), // PB
  _BV(6), // PB
  _BV(7), // PB

  _BV(0), // PA
  _BV(1), // PA
  _BV(2), // PA
  _BV(3), // PA
  _BV(4), // PA
  _BV(5), // PA
  _BV(6), // PA
  _BV(7), // PA
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] =
{
  NOT_ON_TIMER, // PD0
  NOT_ON_TIMER, // PD1
  NOT_ON_TIMER, // PD2
  NOT_ON_TIMER, // PD3
  TIMER1B,      // PD4
  TIMER1A,      // PD5
  TIMER2B,      // PD6
  TIMER2A,      // PD7

  NOT_ON_TIMER, // PC0
  NOT_ON_TIMER, // PC1
  NOT_ON_TIMER, // PC2
  NOT_ON_TIMER, // PC3
  NOT_ON_TIMER, // PC4
  NOT_ON_TIMER, // PC5
  NOT_ON_TIMER, // PC6
  NOT_ON_TIMER, // PC7

  NOT_ON_TIMER, // PB0
  NOT_ON_TIMER, // PB1
  NOT_ON_TIMER, // PB2
  TIMER0A,      // PB3
  TIMER0B,      // PB4
  NOT_ON_TIMER, // PB5
#ifdef TIMER3A
  TIMER3A,      // PB6 - ATmega1284P has a 4th timer
  TIMER3B,      // PB7
#else
  NOT_ON_TIMER, // PB6
  NOT_ON_TIMER, // PB7
#endif
  
  NOT_ON_TIMER, // PA0
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
