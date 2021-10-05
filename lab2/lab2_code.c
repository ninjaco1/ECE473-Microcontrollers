// lab2_skel.c
// Antony Nguyen
// 10.4.2021

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz
#define TRUE 1
#define FALSE 0
// segs to turn on for LED
#define ONE 0b0000 0110 // B, C
#define TWO 0b0101 1011 // A, B, D, E, G
#define THREE 0b0100 1111 // A, B, C, D, G
#define FOUR 0b0110 0110 // B, C, F, G
#define FIVE 0b0110 1101 // A, C, D, F, G
#define SIX 0b0111 1101 // A, C, D, E, F, G
#define SEVEN 0b0000 0111 // A, B, C
#define EIGHT 0b0111 1111 // A, B, C, D, E, F, G
#define NINE 0b0110 0111 // A, B, C, F, G


#include <avr/io.h>
#include <util/delay.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]

//******************************************************************************
//                            chk_buttons
//Checks the state of the button number passed to it. It shifts in ones till
//the button is pushed. Function returns a 1 only once per debounced button
//push so a debounce and toggle function can be implemented at the same time.
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"
//Expects active low pushbuttons on PINA port.  Debounce time is determined by
//external loop delay times 12.
//
uint8_t chk_buttons(uint8_t button) {

}
//******************************************************************************


//*******************************************************************************
//                            debounce_switch
// Adapted from Ganssel's "Guide to Debouncing"
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed.
// Function returns a 1 only once per debounced button push so a debounce and toggle
// function can be implemented at the same time.  Expects active low pushbutton on
// Port D bit zero.  Debounce time is determined by external loop delay times 12.
//
// Passed in the button we want to check
//
//*******************************************************************************
int8_t debounce_switch(int button) {
  static uint16_t state[MAX_BYTE_DEBOUNCE]; //holds present state

  // bit_is_clear: test whether but but in IO register sfr is clear. This will return non zero
  // if the but is clear, and 0 if the bit is set
  // handling multiple inputs
  // https://www.avrfreaks.net/sites/default/files/debouncing.pdf'
  
  state[button] = (state[button] << 1) | (!bit_is_clear(PINA, button)) | 0xE000; // when the second button is pressed

  // debuging to see how state debouncing works
  // PORTB = state[button];
  // _delay_ms(1000);

  if (state[button] == 0xF000)
    return 1;

  return 0;
}



//***********************************************************************************
//                                   segment_sum
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit
//BCD segment code in the array segment_data for display.
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum)
{
  //determine how many digits there are
  //break up decimal sum into 4 digit-segments
  //blank out leading zero digits
  //now move data to right place for misplaced colon position
} //segment_sum
//***********************************************************************************

//***********************************************************************************
uint8_t main() {
  DDRB = 0xE0; //set port B bits 4-7 B as outputs
  PORTB = 0xE0; // set port 
  int current_num = 0; // the number that will be on the display


  while (1)
  {
    //insert loop delay for debounce

    //make PORTA an input port with pullups
    DDRA = 0x00; // set port A as inputs
    PORTA = 0xFF; // set port A as pull ups

    //enable tristate buffer for pushbutton switches

    //now check each button and increment the count as needed

    //disable tristate buffer for pushbutton switches

    //bound the count to 0 - 1023

    //break up the disp_value to 4, BCD digits in the array: call (segsum)

    //bound a counter (0-4) to keep track of digit to display

    //make PORTA an output, 7 segment

    DDRA = 0xFF;
    


    //send 7 segment code to LED segment

    //send PORTB the digit to display

    //update digit to display

  } //while
} //main
