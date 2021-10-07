// lab2_skel.c
// Antony Nguyen
// 10.4.2021

#include <avr/io.h>
#include <util/delay.h>

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz
#define TRUE 1
#define FALSE 0
#define MAX_BIT_DEBOUNCE 8 // numbers of bytes for the debounce

// segs to turn on for LED, negate everything
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
// Port B decoder, remember to not all the digits
#define DIGIT1 ((1 << PB6) | (0 << PB5) | (0 << PB4))
#define DIGIT2 ((0 << PB6) | (1 << PB5) | (0 << PB4))
#define DIGIT3 ((0 << PB6) | (0 << PB5) | (1 << PB4))
#define DIGIT4 ((0 << PB6) | (0 << PB5) | (0 << PB4))
#define DIS_COLON ((0 << PB6) | (1 << PB5) | (0 << PB4))
#define TRI_BUFFER ((1 << PB6) | (1 << PB5) | (1 << PB4))


//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5]

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]
dec_to_7seg[0] = ~ZERO;
dec_to_7seg[1] = ~ONE;
dec_to_7seg[2] = ~TWO;
dec_to_7seg[3] = ~THREE;
dec_to_7seg[4] = ~FOUR;
dec_to_7seg[5] = ~FIVE;
dec_to_7seg[6] = ~SIX;
dec_to_7seg[7] = ~SEVEN;
dec_to_7seg[8] = ~EIGHT;
dec_to_7seg[9] = ~NINE;
dec_to_7seg[10] = ~COLON;
dec_to_7seg[11] = ~BLANK;

// Decoder 3 to 8
uint8_t decoder[8]
decoder[0] = DIGIT4;
decoder[1] = DIGIT3;
decoder[2] = DIS_COLON;
decoder[3] = DIGIT2;
decoder[4] = DIGIT1;
decoder[7] = TRI_BUFFER; 

uint8_t chk_buttons(uint8_t button); // check what button is being pressed
void segsum(uint16_t sum);
void setDigit();
void clearDecoder();

uint8_t main()
{
    DDRB = 0xF0;         //set port B bits 4-7 B as outputs
   
    int current_num = 0; // the number that will be on the display

    // initially display all 0s on the display
    // Steps in what happens
    // person press a button, port A, phase 6
    // set that increment digit to a value, increment variable
    // which what is current digits on the display, variable
    // determine what digit needs to go where
    // use the decoder to send the number to the right digit, PORT B
    // switch PORTA(pull down) to an output, turn on the right segments 


    while (1) {

        int i,j, inc;
        //insert loop delay for debounce

        

        // for loop for each phase of the digit
        for (j = 0; j < 12; j++){ // for the debounce
            for (i = 0; i < 8; i++){
                // make PORTA an input port with pullups
                DDRA = 0x00;  // set port A as inputs
                PORTA = 0xFF; // set port A as pull ups
                // checking what button is being pressed
                inc = 0; // increment initalize to 0 first

                //enable tristate buffer for pushbutton switches
                clearDecoder();
                PORTB |= TRI_BUFFER;
                if(chk_button(i)){
                    //disable tristate buffer for pushbutton switches
                    clearDecoder();
                    inc = 1 << i; 
                    current_num = current + inc;
                    segsum(current_num); // set each digit
                    DDRA = 0xFF;
                    PORTA = 0x00;
                    setDigit();


                }
                
            }

        }
        

        

        //now check each button and increment the count as needed

        

        //bound the count to 0 - 1023

        //break up the disp_value to 4, BCD digits in the array: call (segsum)

        //bound a counter (0-4) to keep track of digit to display

        //make PORTA an output, 7 segment

        

        //send 7 segment code to LED segment

        //send PORTB the digit to display

        //update digit to display

    } //while
} //main


//******************************************************************************




//******************************************************************************
//                            chk_buttons
//Checks the state of the button number passed to it. It shifts in ones till
//the button is pushed. Function returns a 1 only once per debounced button
//push so a debounce and toggle function can be implemented at the same time.
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"
//Expects active low pushbuttons on PINA port.  Debounce time is determined by
//external loop delay times 12.
//
uint8_t chk_buttons(uint8_t button){

    static uint16_t state[MAX_BIT_DEBOUNCE]; //holds present state

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
    // sum is the total count, place each digit into segment_data[5]
    // determine how many digits there are
    //break up decimal sum into 4 digit-segments
    //blank out leading zero digits
    //now move data to right place for misplaced colon position

    segment_data[2] = 0; // doesn't turn on the colon
    int i, remainder, mod, digit;
    mod = 1000;
    for (i = 4; i >= 0; i--){
        if(i == 2)
            continue;
        digit = sum / mod;
        remainder = sum % mod;
        segment_data[4-i] = digit;
        sum = remainder;
        mod /= 10; // divide the mod to 10
    }

} //segment_sum




/*************************************************************************************/
//                                  clearDecoder
// Clears the bits for the decoder. Since the decoder usings bits 4-6, they will be 
// zero afterwards.
/*************************************************************************************/

void clearDecoder(){
    PORTB &= (0 << PB6) | (0 << PB5) | (0 << PB4);
}

/***************************************************************/
//                      setDigit function
// it will choose its given digit and set that number for it
//
/***************************************************************/

void setDigit(){
    int i;
    for(i = 0; i < 5; i++) {
        switch (segment_data[i])
        {

            case 0:
                

                break;
            case 1:

                    break;
            case 2:

                break;
            case 3:

                break;
            case 4:

                break;
            case 5:
                break;
            case 6:
                break;
            case 7:
                break;
            case 8:
                break;
            case 9:
                break;
            default:
                break;
        }

    }
    
}
