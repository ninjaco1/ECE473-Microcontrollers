// lab2_skel.c
// Antony Nguyen
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


// prototypes
int8_t chk_buttons(int button); // check what button is being pressed
void segsum(uint16_t sum);
void setDigit();
void clearDecoder();
void set_dec_to_7seg();
void set_decoder();
// timers fucntions
void spi_init(void);
void tcnt0_init(void);
ISR(TIMER0_OVF_vect);
uint8_t read_write_spi(void);
// encoder
int8_t encoder_chk(uint8_t encoder_var);
void barGraph();

int main()
{
    DDRB = 0xF0; // set port B bits 4-7 B as outputs, decoder
    DDRE = 0x40; // set E6 to output
    DDRD = 0xB0; // slave select pins

    
    

    // current_num = 0; // the number that will be on the display
    tcnt0_init();  //initalize counter timer zero
    spi_init();    //initalize SPI port
    sei();         //enable interrupts before entering loop
    set_dec_to_7seg(); // set values for dec_to_7seg array
    set_decoder(); // set values for the decoder array


    while (1)
    {

        if (current_num > 1023)
            current_num -= 1024;

        segsum(current_num); // set each digit
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
    int i;//, leading_zero;

    segment_data[0] = sum % 10;
    segment_data[1] = (sum % 100) / 10;
    segment_data[2] = 11; // doesn't turn on the colon, blank
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
    uint8_t temp_portb; // holds old value of port b
    for (i = 0; i < 5; i++)
    { // looping through the segment data and assigning the port the right values.
        temp_portb = PORTB & 0x0f; // taket he 4 LSB bits 
        PORTB = decoder[i] | temp_portb; // enable the right digit to turn on
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

/*************************************************************************/
//                           timer/counter0 ISR                          
//When the TCNT0 overflow interrupt occurs, the count_7ms variable is    
//incremented. Every 7680 interrupts the minutes counter is incremented.
//TCNT0 interrupts come at 7.8125ms internals.
// 1/32768         = 30.517578uS
//(1/32768)*256    = 7.8125ms
//(1/32768)*256*64 = 500mS
/*************************************************************************/
ISR(TIMER0_OVF_vect){
    // static uint8_t count_7ms = 0;        //holds 7ms tick count in binary
    // static uint8_t display_count = 0x01; //holds count for display 

    // count_7ms++;                            //increment count every 7.8125 ms 
    // if ((count_7ms % 64)==0){               //?? interrupts equals one half second 
    //     SPDR = display_count;               //send to display 
    //     while (!(TIFR & (1 << TOV0))){}               //wait till data sent out (while loop)
    //     PORTB |= (1 << PORTB0);          //HC595 output reg - rising edge...
    //     PORTB &= (0 << PORTB0);          //and falling edge
    //     display_count = display_count << 1; //shift display bit for next time 
    // }
    // if (display_count == 0x80){
    //     display_count= 1;
    // } //back to 1st positon



    // check for encoder
    static uint8_t new_A = 0, old_A = 0, new_B = 0, old_B = 0, i = 0; 
    static uint8_t serial_out = 0, incrementFlag = 1, oldincflag, count=4;
    uint8_t return_val, i=0; 
    uint8_t left[4] = {0x03, 0x01, 0x00, 0x02}; // state machine for going cw
    uint8_t right[4] = {0x03, 0x02, 0x00, 0x01}; // state machine for going ccw

    
    // getting the data from serial out
    PORTD &= 0 << PORTD3; // turn on slave select the encoder
    PORTD &= 0 << PORTD2; // turn off slave select for bar graph
    PORTE |= 0 << PORTE6; // SH/LD held low so it doesn't read from serial in but from encoder

    // clk pulse
    PORTB |= 1 << PORTB1;
    PORTB &= 0 << PORTB1;
    serial_out = read_write_spi(); // read 
    

    // encoder 1
    new_A = serial_out & 0x03; // buttom 2 bits
    
    // encoder 2
    new_B = (serial_out & 0x0C) >> 2; // the next two then right shift the bits so they match new_A


    oldincflag = incrementFlag;
    return_val = -1; // default return value, no change
    // if ((new_A != old_A) || (new_B != old_B)){ // if change occured
    //     if((new_A == 0) && (new_B == 0)){
    //         if (old_A == 0){
    //             // count++;
    //             incrementFlag = 1;
    //         }
    //         else{
    //             // count--;
    //             incrementFlag = 0;
    //         }
    //     }
    //     else if ((new_A == 0) && (new_B == 1)){
    //         if (old_A == 0){
    //             // count++;
    //             incrementFlag = 1;
    //         }
    //         else{
    //             // count--;
    //             incrementFlag = 0;

    //         }
    //     }
    //     else if ((new_A == 1) && (new_B == 1)){ // detent position
    //         if (old_A == 0){ // one direction 
    //             // if (count == 3){
    //             //     return_val = 0;
    //             // }
    //             if (incrementFlag == 1)
    //                 return_val = 0;
    //         }
    //         else{ // or the other direction
    //             // if (count == -3){
    //             //     return_val = 1;
    //             // }
    //             if (incrementFlag == 0)
    //                 return_val = 1;
    //         }
    //         // count = 0; // count is always reset in detent position
    //     }
    //     else if ((new_A == 1) && (new_B == 0)){
    //         if (old_A == 1){
    //             // count++;
    //             incrementFlag = 1;
    //         }
    //         else{ 
    //             // count--;
    //             incrementFlag = 0;
    //         }
    //     }

    //     old_A = new_A; // save what are now old values
    //     old_B = new_B;

    // } // if changed occured
    // if return value is still -1 then nothing happen
    if (new_A != old_A){
        while (left[i%4] != new_A)
            i++;
        if (old_A == left[(i-1)%4])
            count--;
        while (right[i%4] != new_A)
            i++;
        if (old_A == right[(i-1)%4])
            count++;
    }
    if (new_B != old_B){
        while (left[i%4] != new_B)
            i++;
        if (old_B == left[(i-1)%4])
            count--;
        while (right[i%4] != new_B)
            i++;
        if (old_B == right[(i-1)%4])
            count++;
    }

        old_A = new_A; // save what are now old values
        old_B = new_B;
        
    // when the encoder hits for 4 then 
    if (count == 7){
        // increment
        incrementFlag = 1;
        count = 4; // to set the value
    }  
    if (count == 1){
        // decrement
        incrementFlag = 0;
        count = 4; // to reset the value
    } 
    

    // check for push buttons
    uint16_t i, j, inc;
    
    //enable tristate buffer for pushbutton switches
    PORTB |= TRI_BUFFER;
    //insert loop demake lay for debounce
    for (j = 0; j < 12; j++)
    { // for the debounce
        // for (i = 0; i < 8; i++)
        // {
            //make PORTA an input port with pullups
            DDRA = 0x00;  // set port A as inputs
            PORTA = 0xFF; // set port A as pull ups
            
            inc = 0; // increment initalize to 0 first


            // checking what button is being pressed
            // if (chk_buttons(i))
            // {
            //     inc = 1 << i;
            //     current_num = current_num + inc;
            // }
            
            if (chk_buttons(1)){ // S1, + or - 2
                inc = 1 << 1;
                // check if its increment or decrement
                // current_num += inc;
                if (return_val == -1){
                    
                }
                if (incrementFlag == 1)
                    current_num += inc;
                else if(incrementFlag == 0)
                    current_num -= inc;
            }
            if (chk_buttons(2)){ // S2, + or - 4
                inc = 1 << 2;
                // current_num += inc;
                if (incrementFlag == 1)
                    current_num += inc;
                else if (incrementFlag == 0)
                    current_num -= inc;
            }
        // }
    }




}

/*************************************************************************/
//                              encoder_chk
// This function checks which direction the encoder is spinning.
// If the encoder is rotated clockwise then it will return a 1.
// If the encoder is rotated counter clockwise then it will return a 0.
/*************************************************************************/
int8_t encoder_chk(uint8_t encoder_var){
    // A and B are in bits 0 and 1
    static uint16_t state = {0}; // hold bits from encoder
    uint8_t a_pin, b_pin;        // encoder pin states

    // a_pin and b_pin are asserted TRUE when low
    a_pin = ((encoder_var & 0x01) == 0) ? 0 : 1;
    b_pin = ((encoder_var & 0x02) == 0) ? 0 : 1;

    // update shift using only the A pin
    state = (state << 1) | a_pin | 0xe0;

    // check for falling edge on A pin
    // if it did, then B pin state indicates direction
    // of rotation. Return 1 for CW, 0 fro CCW
    if(state == 0xf0)
        return (b_pin) ? 1 : 0;
    else
        return -1;
}


/*************************************************************************/
//                            read_write_spi
//
/*************************************************************************/
uint8_t read_write_spi(uint8 dataout){
    // load data into spi buffer
    SPDR = 0;
    while (bit_is_clear(SPSR,SPIF)){}
    return (SPDR);
}