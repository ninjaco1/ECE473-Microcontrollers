// lab3_code.c
// Anthony Nguyen
// 10.4.2021

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

// Bargraph board           Mega128 board 
// --------------      ----------------------    
//     reglck            PORTD bit 2 (ss_n)                      
//     srclk             PORTB bit 1 (sclk)
//     sdin              PORTB bit 2 (mosi)
//     oe_n              PORTB bit 7
//     gnd2                   ground
//     vdd2                     vcc
//     sd_out               no connect

// Encoder board            Mega128 board
// --------------      ----------------------  
//   shift_ld_n             PORTE bit 6
//   clk_inh                PORTD bit 3 (ss)
//   sck                    PORTB bit 1 (sclk)
//   ser_in                 no connect
//   ser_out                PORTB bit 3 (miso)
//   vdd1                   vcc
//   gnd1                   ground

// #define F_CPU 16000000 // cpu speed in hertz
#define TRUE 1
#define FALSE 0
#define MAX_BIT_DEBOUNCE 8 // numbers of bytes for the debounce

// segs to turn on for LED, negate everything
#define ZERO 0b00111111  // A, B, C, D, E, F
#define ONE 0b00000110   // B, C
#define TWO 0b01011011   // A, B, D, E, G
#define THREE 0b01001111 // A, B, C, D, G
#define FOUR 0b01100110  // B, C, F, G
#define FIVE 0b01101101  // A, C, D, F, G
#define SIX 0b01111101   // A, C, D, E, F, G
#define SEVEN 0b00000111 // A, B, C
#define EIGHT 0b01111111 // A, B, C, D, E, F, G
#define NINE 0b01100111  // A, B, C, F, G
#define BLANK 0x00
#define COLON 0b00000011 // A, B
// Port B decoder, remember to not all the digits
#define DIGIT1 0x40     //((1 << PB6) | (0 << PB5) | (0 << PB4))
#define DIGIT2 0x30     //((0 << PB6) | (1 << PB5) | (0 << PB4))
#define DIGIT3 0x10     //((0 << PB6) | (0 << PB5) | (1 << PB4))
#define DIGIT4 0x00     //((0 << PB6) | (0 << PB5) | (0 << PB4))
#define DIS_COLON 0x20  //((0 << PB6) | (1 << PB5) | (0 << PB4))
#define TRI_BUFFER 0x70 //((1 << PB6) | (1 << PB5) | (1 << PB4))

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12];

// Decoder 3 to 8
uint8_t decoder[8];

// current number on the display
uint16_t current_num = 0;

// what value to display
static uint8_t barGraphDisplay = 0;

// flags
// determine if we are increment or decrement mode
static uint8_t incDec2 = 0;
static uint8_t incDec4 = 0;
static uint8_t data = 0;
static uint8_t colonDisplay = 0;

// clock
static uint8_t minutes = 0;
static uint8_t hours = 0;

// lab 2 functions
int8_t chk_buttons(int button); // check what button is being pressed
void segsum(uint16_t sum);
void setDigit();
void clearDecoder();
void set_dec_to_7seg();
void set_decoder();

// lab 3 functions
void barGraph();
uint8_t encoderRead(uint8_t data, uint8_t knob);
void spi_init(void);
void tcnt0_init(void);
ISR(TIMER0_OVF_vect);

// lab4 functions
void segclock();

int main()
{
    DDRB = 0xF0; //set port B bits 4-7 B as outputs
    DDRE |= 0b01000000; // set E6 to output
    DDRD |= 0b00001100; // slave select pins

    PORTB &= ~(1 << PORTB7); 

    tcnt0_init();  //initalize counter timer zero
    spi_init();    //initalize SPI port
    sei();         //enable interrupts before entering loop

    set_dec_to_7seg(); // set values for dec_to_7seg array
    set_decoder(); // set values for the decoder array

    

    while (1)
    {

        // spi 
        PORTD |= 1 << PORTD3; // clock_inh = 1
        PORTE &= 0 << PORTE6; // load sh/ld

        PORTE |= 1 << PORTE6; // sh/ld
        PORTD &= ~(1 << PORTD3); // clock_inh

        SPDR = 0; // writing a random value
       
        while (bit_is_clear(SPSR,SPIF)){}
        data = SPDR; // read data

        // end of another block

        barGraph();

        // if (current_num > 1023)
        //     current_num -= 1024;

        // segsum(current_num); // set each digit
        segclock(); // set each digit for the clock
        setDigit();         // setting the digit on display
    } //while
    return 0;
} //main

/******************************************************************************/
//                              set_dec_to_7seg
// setting the dec_to_7seg array for which segment to turn off in order to see
// the digit on the LED display. 
/******************************************************************************/
void set_dec_to_7seg()
{
    dec_to_7seg[0] = ~(ZERO);
    dec_to_7seg[1] = ~(ONE);
    dec_to_7seg[2] = ~(TWO);
    dec_to_7seg[3] = ~(THREE);
    dec_to_7seg[4] = ~(FOUR);
    dec_to_7seg[5] = ~(FIVE);
    dec_to_7seg[6] = ~(SIX);
    dec_to_7seg[7] = ~(SEVEN);
    dec_to_7seg[8] = ~(EIGHT);
    dec_to_7seg[9] = ~(NINE);
    dec_to_7seg[10] = ~(COLON);
    dec_to_7seg[11] = ~(BLANK);
}

/******************************************************************************/
//                              set_decoder
// This function sets the right value for decoder so that it display the right 
// digit. The index value of the decoder represents the Yx output of the decoder.
/******************************************************************************/
void set_decoder()
{
    decoder[0] = DIGIT4;
    decoder[1] = DIGIT3;
    decoder[2] = DIS_COLON;
    decoder[3] = DIGIT2;
    decoder[4] = DIGIT1;
    decoder[7] = TRI_BUFFER;
}

//******************************************************************************
//                            chk_buttons
//Checks the state of the button number passed to it. It shifts in ones till
//the button is pushed. Function returns a 1 only once per debounced button
//push so a debounce and toggle function can be implemented at the same time.
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"
//Expects active low pushbuttons on PINA port.  Debounce time is determined by
//external loop delay times 12.
//
int8_t chk_buttons(int button)
{

    static uint16_t state[MAX_BIT_DEBOUNCE]; //holds present state

    // bit_is_clear: test whether but but in IO register sfr is clear. This will return non zero
    // if the but is clear, and 0 if the bit is set
    // handling multiple inputs
    // https://www.avrfreaks.net/sites/default/files/debouncing.pdf'

    state[button] = (state[button] << 1) | (!bit_is_clear(PINA, button)) | 0xE000; // when the second button is pressed

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
    int i;//, leading_zero;

    segment_data[0] = sum % 10;
    segment_data[1] = (sum % 100) / 10;
    // segment_data[2] = 11; // doesn't turn on the colon, blank
    segment_data[2] = (colonDisplay == 1) ? 10 : 11;
    segment_data[3] = (sum % 1000) / 100;
    segment_data[4] = sum / 1000;

    // remove the leading zeros
    // leading_zero = 1;
    for(i = 4; i >0; i--){
        if(i == 2)
            continue;
        if(segment_data[i] == 0)
            segment_data[i] = 11; // replace it with a blank
        
        else
            break;
        
    }

} //segment_sum

//***********************************************************************************
//                                segclock
//takes two 8-bit binary values(hours and minutes) and places the appropriate 
//equivalent 4 digit.
//BCD segment code in the array segment_data for display.
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segclock(){
    segment_data[0] = minutes % 10;
    segment_data[1] = minutes / 10;
    segment_data[2] = (colonDisplay == 1) ? 10 : 11;
    segment_data[3] = hours % 10;
    segment_data[4] = hours / 10;
}


/***************************************************************/
//                      setDigit function
// it will choose its given digit and set that number for it.
// The cases set the value on PORTA to the right segments and PORTB
// to decoder.
/***************************************************************/

void setDigit()
{
    DDRA = 0xFF; // setting PORT A as an output
    int i;
    for (i = 0; i < 5; i++)
    { // looping through the segment data and assigning the port the right values.
        PORTB = decoder[i]; // enable the right digit to turn on
        PORTA = dec_to_7seg[segment_data[i]]; // turn on the right segments
        _delay_ms(0.5);
    }
}


/***********************************************************************/
//                            spi_init                               
//Initalizes the SPI port on the mega128. Does not do any further   
//external device specific initalizations.  Sets up SPI to be:                        
//master mode, clock=clk/2, cycle half phase, low polarity, MSB first
//interrupts disabled, poll SPIF bit in SPSR to check xmit completion
/***********************************************************************/
void spi_init(void){
    DDRB  |=   0x07; //Turn on SS, MOSI, SCLK
    SPCR  |=   (1 << SPE) | (1 << MSTR); //enable SPI, master mode 
    SPSR  |=   (1 << SPI2X); // double speed operation
}//spi_init

/***********************************************************************/
//                              tcnt0_init                             
//Initalizes timer/counter0 (TCNT0). TCNT0 is running in async mode
//with external 32khz crystal.  Runs in normal mode with no prescaling.
//Interrupt occurs at overflow 0xFF.
//
void tcnt0_init(void){
    ASSR   |=  (1 << AS0); //ext osc TOSC
    TIMSK  |=  (1 << TOIE0); //enable TCNT0 overflow interrupt
    TCCR0  |=  (1 << CS00); //normal mode, no prescale
}

/******************************************************************************/
//                                    ISR
// Then fucntion will will called when there is an interrupt within the system
// and when the overflow flag for timer counter 0 it set.
// This fucntions checks the push buttons to see which buttons were pressed
// then set it in its correct mode.
// Afterwards checks the encoder to see where it is.
/******************************************************************************/
ISR(TIMER0_OVF_vect){
    uint16_t i;
    static uint8_t count = 0, seconds;
    //insert loop demake lay for debounce

    // checking the push buttons 
    // for loop for each phase of the digit
    PORTB |= TRI_BUFFER;

    for (i = 0; i < 12; i++)
    { // for the debounce
        
        //make PORTA an input port with pullups
        DDRA = 0x00;  // set port A as inputs
        PORTA = 0xFF; // set port A as pull ups
        

        // checking what button is being pressed
        if (chk_buttons(1)) // set the increment mode on 
        {
            incDec2 ^= 1; // flip the bits for the flag
            barGraphDisplay ^= 1 << 0; // show up on the first led

        }
        if (chk_buttons(2)) // set the decrement mode on
        {
            incDec4 ^= 1; // flip the bits for the flag
            barGraphDisplay ^= 1 << 1; // show up on the 2nd led
        }
        
    }
    PORTB &= ~(TRI_BUFFER); // turn off the tri state buffer 

    // reading each knob
    uint8_t enc1 = encoderRead(data, 0);
    uint8_t enc2 = encoderRead(data, 1);

    // each case of what the knob or buttons will be
    if (incDec2 == 1 && incDec4 == 1){
        current_num = current_num;

    }
    else if (incDec2 == 0 && incDec4 == 0){
        if(enc1 == 0 || enc2 == 0)
            current_num -= 1;
        if (enc1 == 1 || enc2 == 1)
            current_num += 1;
    }
    else if (incDec2 == 1){
        if(enc1 == 0 || enc2 == 0)
            current_num -= 2;
        if (enc1 == 1 || enc2 == 1)
            current_num += 2;
    }
    else if (incDec4 == 1){
        if (enc1 == 0 || enc2 == 0)
            current_num -= 4;
        if (enc1 == 1 || enc2 == 1)
            current_num += 4;
    }

    // add a counter to determine one second 
    count++;
    if ((count % 128) == 0){
        // 1 second has past
        colonDisplay ^= 0x1; // blinking
        seconds++;
        if ((seconds % 60) == 0){
            minutes++;
            seconds = 0;
            if ((minutes % 60) == 0){
                hours++;
                minutes = 0;
                if (hours % 24 == 0)
                    hours = 0;
            }
        }
    }
    
}


/******************************************************************************/
//                                 encoderRead
// This function checks the state of the encoder so see what its behavior is.
// It will return -1 if there is no change within. It will return 1 if the system is 
// CW. It will return 0 if the system is CCW. 
/******************************************************************************/
uint8_t encoderRead(uint8_t data, uint8_t knob){

    // check for encoder
    static uint8_t old_state[4] = {0xff,0xff,0xff,0xff};
    uint8_t new_A = -1;
    uint8_t new_B = -1;
    static uint8_t count = 0; 
    uint8_t return_val, a, b, a_index, b_index; 

    a = (knob == 0) ? 1 : 4; // where the position of a is
    b = (knob == 0) ? 2 : 8; // where the position of b is

    a_index = (knob == 0) ? 0 : 2;
    b_index = (knob == 0) ? 1 : 3;
    
    new_A = (data & a) ? 1 : 0; // most LSB
    new_B = (data & b) ? 1 : 0; // 2nd LSB


    return_val = -1; // default return value, no change

    if ((new_A != old_state[a_index]) || (new_B != old_state[b_index])){ // if change occured
        if((new_A == 0) && (new_B == 0)){
            if (old_state[a_index] == 1){
                count++;
            }
            else{
                count--;
            }
        }
        else if ((new_A == 0) && (new_B == 1)){
            if (old_state[a_index] == 0){
                count++;
            }
            else{
                count--;     
            }
        }
        else if ((new_A == 1) && (new_B == 1)){ // detent position
            if (old_state[a_index] == 0){ // one direction 
                if (count == 3){
                    return_val = 0;
                }
            }
            else{ // or the other direction
                if (count == -3){
                    return_val = 1;
   
                }
            }
            count = 0; // count is always reset in detent position
        }
        else if ((new_A == 1) && (new_B == 0)){
            if (old_state[a_index] == 1){
                count++;
            }
            else{ 
                count--;
            }
        }

        old_state[a_index] = new_A; // save what are now old values
        old_state[b_index] = new_B;

    } // if changed occured
    // if return value is still -1 then nothing happen
    return (return_val); // return coder state
}


/******************************************************************************/
//                                  barGraph
// Set the mode on the bar graph. 
/******************************************************************************/
void barGraph(){
    
    SPDR = barGraphDisplay;
    while (bit_is_clear(SPSR,SPIF)){}               //wait till data sent out (while loop)
    PORTD |= (1 << PORTD2);          //HC595 output reg - rising edge...
    PORTD &= (0 << PORTD2);          //and falling edge

}
