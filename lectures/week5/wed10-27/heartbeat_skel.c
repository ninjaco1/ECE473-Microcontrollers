// heartbeat.c
// Anthony Nguyen
// setup TCNT1 in PWM mode 
// setup TCNT3 in normal mode 
// set OC1A (PB5) as pwm output 
// pwm frequency:  (16,000,000)/(1 * (61440 + 1)) = 260hz
//

#include <avr/io.h>
#include <avr/interrupt.h>

uint16_t brightness[20] = {0x0000, 0x0ccc, 0x1998, 0x2664, 0x3330, 0x3ffc, 0x4cc8, 0x5994, 0x6660, 0x732c, 0x7ff8, 0x8cc4, 0x9990, 0x865c, 0xb328, 0xbff4, 0xccc0, 0xd98c, 0xe658, 0xf324, 0xf000};

ISR(TIMER3_COMPA_vect) {
  static uint8_t index=0;  //steps through the array 
  //PORTB |= 1 << PORTB5;
  if (index == 20){ 
    //PORTB &= 0 << PORTB5;
    index = 0;
  }  //set bounds on index
  OCR1A = brightness[index];  //sset OCR1A to new value
  index++;

}

int main() {
  // setup TCNT1 in PWM mode 
  //set PORTB bit 5 as the PWM output
  DDRB   = 0x20;          
  //fast PWM, set on match, clear at top, ICR1 holds value of TOP 
  TCCR1A |=  (1 << COM1A1) | (1 << COM1A0) | (1 << WGM11) | (0 << WGM10);
  //use ICR1 as source for TOP, use clk/1
  TCCR1B |=  (1 << ICES1) | (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10);
  //no forced compare 
  //TCCR1C = (0 << FOC1A);
  TCCR1C = 0x00;
  //clear at 0xF000                               
  ICR1  = 0xF000;
  
// setup TCNT3 in normal mode to control the update rate 
// heartbeat update frequency = (16,000,000)/(8 * 2^16) = 30 cycles/sec
  //normal mode
  TCCR3A = 0x00;
  //use clk/8  (30hz)  
  //TCCR3B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  TCCR3B = 0b00000010;
  //no forced compare 
  //TCCR3C |= (0 << FOC3A);
  TCCR3C = 0x00;
  //enable timer 3 interrupt on TOV
  //ETIMSK |= (1 << TOIE3);
  ETIMSK = 0b00010000;

  sei();  //set GIE
  while(1) {}; //loop forever waiting for interrupt
 
}  // main
