// lab3_code.c
// Antony Nguyen
// 10.27.2021

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

static uint8_t barGraphOutput = 0;
static uint8_t incrementFlag = -1;


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

// encoder
void barGraph(uint8_t);
uint8_t encoderRead(uint8_t data, uint8_t knob);

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

    uint8_t serial_out;
    static uint8_t bargraph = 0x00;
        static uint8_t light = 0x00;

    // getting the data from serial out

    PORTD |= 1 << PORTD3; // turning on clk_inh
    PORTE &= 0 << PORTE6; // turning SH/LD low

    _delay_ms(1);
    PORTD &= 0 << PORTD3; // turning off clk_inh
    PORTE |= 1 << PORTE6;// turing SH/LD high
  
    SPDR = light;
    while (bit_is_clear(SPSR,SPIF)){}               //wait till data sent out (while loop)
    

    bargraph = SPDR;
    if (bargraph != 0x00) {
        light = 0xff;
    }

    
    
    uint8_t temp = encoderRead(serial_out,0);
    // uint8_t temp1 = encoderRead(serial_out, 1);
    if (temp != -1)
        incrementFlag = temp;
    // uint8_t rotation[2] = {incrementFlag, temp1};

    barGraph(incrementFlag);
    // barGraph(rotation[0]);
    // barGraph(-1);


    // check for push buttons
    uint16_t j, inc;
    
    //enable tristate buffer for pushbutton switches
    PORTB |= TRI_BUFFER;
    //insert loop demake lay for debounce
    for (j = 0; j < 12; j++)
    { // for the debounce
      
        //make PORTA an input port with pullups
        DDRA = 0x00;  // set port A as inputs
        PORTA = 0xFF; // set port A as pull ups
        
        inc = 0; // increment initalize to 0 first
        
        if (chk_buttons(1)){ // S1, + or - 2
            inc = 1 << 1;
            // check if its increment or decrement
            current_num += inc;
            // if (return_val == -1){
            //     continue;
            // }
            // if (incrementFlag == 1)
            //     current_num += inc;
            // else if(incrementFlag == 0)
            //     current_num -= inc;
            
        }
        if (chk_buttons(2)){ // S2, + or - 4
            inc = 1 << 2;
            current_num += inc;
            // if (incrementFlag == 1)
            //     current_num += inc;
            // else if (incrementFlag == 0)
            //     current_num -= inc;
            
        }
    
    }
    // current_num += 1;
}


/*************************************************************************/
//                                  encoder
// processing the encoder 
/*************************************************************************/
uint8_t encoderRead(uint8_t data, uint8_t knob){
    
        // check for encoder
    static uint8_t new_A = -1, old_A = -1, new_B = -1, old_B = -1, count = 0; 
    uint8_t return_val, a, b; 

    a = (knob == 0) ? 1 : 4; // where the position of a is
    b = (knob == 0) ? 2 : 8; // where the position of b is

    new_A = data & a; // most LSB
    new_B = data & b; // 2nd LSB


    return_val = -1; // default return value, no change

    if ((new_A != old_A) || (new_B != old_B)){ // if change occured
        if((new_A == 0) && (new_B == 0)){
            if (old_A == 1){
                count++;
            }
            else{
                count--;
            }
        }
        else if ((new_A == 0) && (new_B == 1)){
            if (old_A == 0){
                count++;
            
            }
            else{
                count--;  
            }
        }
        else if ((new_A == 1) && (new_B == 1)){ // detent position
            if (old_A == 0){ // one direction 
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
            if (old_A == 1){
                count++;
           
            }
            else{ 
                count--;
              
            }
        }

        old_A = new_A; // save what are now old values
        old_B = new_B;

    } // if changed occured
    // if return value is still -1 then nothing happen
    return (return_val); // return coder state


}



/******************************************************************************/
//                             barGraph
//
//
//
/******************************************************************************/
void barGraph(uint8_t increment){
    uint8_t display_mode; //holds count for display 

    // top half on for increment
    if (increment == 1)
        display_mode = 0xF0;
    else if (increment == 0) // bot half on for decrement
        display_mode = 0x0F;
    else
        display_mode = 0b10101010; // otherwise display every other
    

    SPDR = display_mode;
    // barGraphOutput = display_mode;               //send to display 
    while (bit_is_clear(SPSR,SPIF)){}               //wait till data sent out (while loop)
    PORTD |= (1 << PORTD2);          //HC595 output reg - rising edge...
    PORTD &= (0 << PORTD2);          //and falling edge
 
}