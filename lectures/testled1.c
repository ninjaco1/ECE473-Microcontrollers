// testled1.c
// R. Traylor
// 10.3.05
// Tests wiring of LED board to mega128.
// Select a digit with PORTB upper nibble then with
// port D push buttons illuminate a single segment.

// Port mapping:
// Port A:  bit0 brown  segment A
//          bit1 red    segment B
//          bit2 orange segment C
//          bit3 yellow segment D
//          bit4 green  segment E
//          bit5 blue   segment F
//          bit6 purple segment G
//          bit7 grey   decimal point
//               black  Vdd
//               white  Vss

// Port B:  bit4 green  seg0
//          bit5 blue   seg1
//          bit6 purple seg2
//          bit7 grey   pwm
//               black  Vdd
//               white  Vss

#include <avr/io.h>
#include <util/delay.h>
#define ZERO  0b00111111   // A, B, C, D, E, F
#define ONE   0b00000110   // B, C
#define TWO   0b01011011   // A, B, D, E, G
#define THREE 0b01001111   // A, B, C, D, G
#define FOUR  0b01100110   // B, C, F, G
#define FIVE  0b01101101   // A, C, D, F, G
#define SIX   0b01111101   // A, C, D, E, F, G
#define SEVEN 0b00000111   // A, B, C
#define EIGHT 0b01111111   // A, B, C, D, E, F, G
#define NINE  0b01100111   // A, B, C, F, G
#define BLANK 0x00
#define COLON 0b00000011   // A, B

int main()
{
  DDRA = 0xFF;  //set port A to all outputs
  DDRB = 0xF0;  //set port bits 4-7 B as outputs
  DDRD = 0x00;  //set port D all inputs
  PORTD = 0xFF; //set port D all pullups
  PORTA = 0xFF; //set port A to all ones  (off, active low)
  // num = 1023;

  while (1)
  {
    PORTB = 0x00; //digit zero  on
    PORTA = ~(THREE);
    _delay_ms(2);
    PORTB = 0x10; //digit one   on
    PORTA = ~(TWO);
    _delay_ms(2);
    PORTB = 0x20; //colon, indicator leds  on
    PORTA = ~(BLANK);
    _delay_ms(2);
    PORTB = 0x30; //digit two   on
    PORTA = ~(ZERO);
    _delay_ms(2);
    PORTB = 0x40; //digit three on
    PORTA = ~(ONE);

    _delay_ms(2);
    // PORTA = PIND; //push button determines which segment is on
  }               //while
} //main
