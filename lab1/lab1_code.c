// lab1_code.c
// Anthony Nguyen
// 9.22.21

//This program increments a binary display of the number of button pushes on switch
//S0 on the mega128 board.

#include <avr/io.h>
#include <util/delay.h>
#define MAX_BYTE_DEBOUNCE 8 // numbers of bytes for the debounce

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
int8_t debounce_switch(int button)
{
  static uint16_t state[MAX_BYTE_DEBOUNCE]; //holds present state

  // bit_is_clear: test whether but but in IO register sfr is clear. This will return non zero
  // if the but is clear, and 0 if the bit is set
  // handling multiple inputs
  // https://www.avrfreaks.net/sites/default/files/debouncing.pdf
  state[button] = (state[button] << 1) | (!bit_is_clear(PIND, button)) | 0xE000; // when the second button is pressed

  // debuging to see how state debouncing works
  // PORTB = state[button];
  // _delay_ms(1000);

  // once the certain switch reaches the value it will return 1
  if (state[button] == 0xF000)
    return 1;

  return 0;
}

//*******************************************************************************
// Check switch S0.  When found low for 12 passes of "debounce_switch(), increment
// PORTB.  This will make an incrementing count on the port B LEDS.
//*******************************************************************************
int main(){
  DDRB = 0xFF; //set port B to all outputs
  while (1){ //do forever
    // int result = debounce_switch();
    // if (result == 1) {
    //   PORTB++;
    // }             //if switch true for 12 passes, increment port B

    // check if S1 was pressed
    if (debounce_switch(0))
      PORTB++; // if it was pressed then increase value
    

    // Now...modify the code so that the S1 push button increments the count by one and
    // S2 decrements the count by one. The count will "roll over after 0xFF to 0x00 and
    // will roll under to 0xFF from 0x00. When S1 and S0 are pushed simultaneously, the
    // count will not change.

    // check if S2 was pressed
    if (debounce_switch(1))
      PORTB--; // if it was pressed then decrease value
    

    _delay_ms(2); //keep in loop to debounce 24ms
  }               //while

} //main
