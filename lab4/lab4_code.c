// lab3_code.c
// Anthony Nguyen
// 10.4.2021

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "hd44780.h"

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

// Audio Amp                Mega128 board
// --------------      ----------------------
//  vol                     PORTE bit 3

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

// lab 4 define
#define SNOOZE_TIMER 10

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

// determine if we are increment or decrement mode
static uint8_t data = 0;

// holding the ADC value
uint16_t adc_result;     //holds adc result 

// flags
static uint8_t colonDisplay = 0; // blinking for colons
static uint8_t timerFlag = 0; // if timer is on, 10 seconds
static uint8_t alarmFlag = 0; // indication on LED display
static uint8_t changeMinuteFlag = 0; // change the clock minutes
static uint8_t changeHourFlag = 0; // change the clock hours
static uint8_t setAlarm = 0; // setting the alarm to desire time
static uint8_t alarmInit = 0; // alarm desire declared, know many times the button has been pressed

// clock
static uint8_t minutes = 0;
static uint8_t hours = 0;
static uint16_t timer = 0; // in seconds
static uint8_t alarmMinute = 0;
static uint8_t alarmHour = 0;

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
void alarmDisplay();
void buttonPress(uint8_t);
void tcnt1_init(void); // frequency of notes
void tcnt2_init(void);
void tcnt3_init(void);
ISR(TIMER1_COMPA_vect); // ctc, notes
void setVolumeController();
void adc_init(void);
void adc_read(void);

int main()
{
    DDRB = 0xF0;        //set port B bits 4-7 B as outputs
    DDRE |= 0b01001000; // set E6, E3 to output
    DDRD |= 0b00001100; // slave select pins
    DDRC |= 0xFF;       // set prot C to all outputs

    PORTB &= ~(1 << PORTB7);
    PORTC |= (1 << PORTC6); // turn it on

    tcnt0_init(); //initalize counter timer zero
    tcnt1_init();  //initalize counter timer one
    tcnt2_init();  //initalize counter timer two
    tcnt3_init();  //initalize counter timer three
    adc_init();    // initalize the ADC
    spi_init(); //initalize SPI port
    sei();      //enable interrupts before entering loop

    set_dec_to_7seg(); // set values for dec_to_7seg array
    set_decoder();     // set values for the decoder array

    lcd_init(); // initalize the lcd display
    timer = SNOOZE_TIMER;

    while (1)
    {

        // spi
        PORTD |= 1 << PORTD3; // clock_inh = 1
        PORTE &= ~(1 << PORTE6); // load sh/ld

        PORTE |= 1 << PORTE6;    // sh/ld
        PORTD &= ~(1 << PORTD3); // clock_inh

        SPDR = 0; // writing a random value

        while (bit_is_clear(SPSR, SPIF)){}
        data = SPDR; // read data
        barGraph();
        // end of spi

        segclock();     // set each digit for the clock
        setDigit();     // setting the digit on display
        
       
        // check if the alarm matches the actually clock
        if ((alarmInit > 1) && (alarmMinute == minutes) && (alarmHour == hours)){
            timerFlag = 1; // make the timer go off
            
        
        }
        alarmDisplay(); // display "ALARM" on the LCD display
    }                   //while
    return 0;
} //main


/***********************************************************************/
//                            spi_init
//Initalizes the SPI port on the mega128. Does not do any further
//external device specific initalizations.  Sets up SPI to be:
//master mode, clock=clk/2, cycle half phase, low polarity, MSB first
//interrupts disabled, poll SPIF bit in SPSR to check xmit completion
/***********************************************************************/
void spi_init(void)
{
    DDRB |= 0x07;                     //Turn on SS, MOSI, SCLK
    SPCR |= (1 << SPE) | (1 << MSTR); //enable SPI, master mode
    SPSR |= (1 << SPI2X);             // double speed operation
} //spi_init

/***********************************************************************/
//                              tcnt0_init
//Initalizes timer/counter0 (TCNT0). TCNT0 is running in async mode
//with external 32khz crystal.  Runs in normal mode with no prescaling.
//Interrupt occurs at overflow 0xFF.
//
void tcnt0_init(void)
{
    ASSR |= (1 << AS0);    //ext osc TOSC
    TIMSK |= (1 << TOIE0); //enable TCNT0 overflow interrupt
    TCCR0 |= (1 << CS00);  //normal mode, no prescale
}

/***********************************************************************/
//                              tcnt1_init
//Initalizes timer/counter1 (TCNT1). TCNT1 is running in async mode
//with interal 16Mhz crystal.  Runs in normal mode with no prescaling.
//Interrupt occurs at OCR1A value.
//
void tcnt1_init(void)
{
    TIMSK |= (1 << OCIE1A);               //enable TCNT1 ctc interrupt
    TCCR1A |= 0;                          // CTC on OC1A
    TCCR1B |= (1 << CS10) | (1 << WGM12); // no prescalar and ctc
    TCCR1C |= 0x0;
    OCR1A = 18181; // 440Hz, A4, change later for beaver fight song
}

/***********************************************************************/
//                              tnct2_init
// Initalizes timer/counter2 (TCNT2). TNCT2 is running a fast PWM mode.
// This will be on PORTB7. This timer to be used to set the brightness of
// the LED display
/***********************************************************************/
void tcnt2_init(void)
{
  //fast PWM, set on match, 64 prescaler
  TCCR2 |= (1 << WGM21) | (1 << WGM20) | (1 << COM21) | (1 << COM20) | (1 << CS22);
  OCR2 = 0xF0; //clear at 0xF0 CLEAR AT BRIGHTNESS
}

/***********************************************************************/
//                              tnct3_init
// Initalizes timer/counter3 (TCNT3). TNCT3 is running a fast PWM mode,
// Uses OC3A which is on PE3. Clear at the bottom, inverting mode
// This sets the volume control for the speaker. 
/***********************************************************************/
void tcnt3_init(void)
{
    //Fast PWM, set on compare match
    TCCR3A |= (1 << WGM31) | (0 << WGM30) | (1 << COM3A1) | (1 << COM3A0); // inverting mode
    TCCR3B |= (1 << ICES3) | (1 << WGM33) | (1 << WGM32) | (1 << CS30); //No prescale
    TCCR3C  = 0x00; //no force compare
    OCR3A = 0xFFFF; // volume
    ICR3  = 0xFFFF; // top value


}

/***********************************************************************/
//                            adc_init
/***********************************************************************/
void adc_init(void){

    //Initalize ADC and its ports
    DDRF  &= ~(_BV(DDF7)); //make port F bit 7 the ADC input  
    PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off 

    ADMUX |= (0 << REFS1) | (1 << REFS0) | (0 << MUX4) | (0 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0);    //single-ended input, PORTF bit 7, right adjusted, 10 bits
                                            //reference is AVCC

    ADCSRA |=  (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADPS0); //ADC enabled, don't start yet, single shot mode 
                                            //division factor is 128 (125khz)
}

/***********************************************************************/
//                            adc_read()
/***********************************************************************/
void adc_read(){
    ADCSRA |= (1 << ADSC);                          //poke the ADSC bit and start conversion

    while(bit_is_clear(ADCSRA, ADIF)){}                     //spin while interrupt flag not set

    ADCSRA |= 1 << ADIF;                              //its done, clear flag by writing a one 

    adc_result = ADC;                      //read the ADC output as 16 bits


}

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
    int i; //, leading_zero;

    segment_data[0] = sum % 10;
    segment_data[1] = (sum % 100) / 10;
    // segment_data[2] = 11; // doesn't turn on the colon, blank
    segment_data[2] = (colonDisplay == 1) ? 10 : 11;
    segment_data[3] = (sum % 1000) / 100;
    segment_data[4] = sum / 1000;

    // remove the leading zeros
    // leading_zero = 1;
    for (i = 4; i > 0; i--)
    {
        if (i == 2)
            continue;
        if (segment_data[i] == 0)
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
void segclock()
{
    if(setAlarm == 0){
        segment_data[0] = minutes % 10;
        segment_data[1] = minutes / 10;
        segment_data[2] = (colonDisplay == 1) ? 10 : 11;
        segment_data[3] = hours % 10;
        segment_data[4] = hours / 10;

    }
    if (setAlarm == 0x1){
        segment_data[0] = alarmMinute % 10;
        segment_data[1] = alarmMinute / 10;
        segment_data[2] = (colonDisplay == 1) ? 10 : 11;
        segment_data[3] = alarmHour % 10;
        segment_data[4] = alarmHour / 10;
    }
    
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
    uint8_t dis;
    for (i = 0; i < 5; i++)
    {                       // looping through the segment data and assigning the port the right values.
        PORTB = decoder[i]; // enable the right digit to turn on
        dis = dec_to_7seg[segment_data[i]];
        if ((i == 4) && (alarmFlag == 1))
        {

            dis &= ~(1 << 7);
        }
        PORTA = dis; // turn on the right segments
        _delay_ms(0.5);
    }
}


/******************************************************************************/
//                                    ISR
// Then fucntion will will called when there is an interrupt within the system
// and when the overflow flag for timer counter 0 it set.
// This fucntions checks the push buttons to see which buttons were pressed
// then set it in its correct mode.
// Afterwards checks the encoder to see where it is.
/******************************************************************************/
ISR(TIMER0_OVF_vect)
{
    uint16_t i, j;
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
        for (j = 0; j < 8; j++)
        {
            if (chk_buttons(j))
                buttonPress(j);
        }
    }
    PORTB &= ~(TRI_BUFFER); // turn off the tri state buffer

    // reading each knob
    uint8_t enc1 = encoderRead(data, 0);
    uint8_t enc2 = encoderRead(data, 1);

    // each case of what the knob or buttons will be
    if (setAlarm == 0)
    {
        if (enc1 == 0 || enc2 == 0)
        {
            // ccw
            //current_num -= 1;
            if (changeMinuteFlag == 1 && changeHourFlag == 0)
            {
                // change minutes
                minutes--;
                if (minutes == 255) // since its unsign 255 = -1
                    minutes = 59;
            }
            if (changeHourFlag == 1 && changeMinuteFlag == 0)
            {
                hours--;
                if (hours == 255)
                    hours = 23;
            }
        }

        if (enc1 == 1 || enc2 == 1)
        {
            // cw
            // current_num += 1;
            if (changeMinuteFlag == 1 && changeHourFlag == 0)
            {
                // change minutes
                minutes++;
                if (minutes % 60 == 0)
                    minutes = 0;
            }
            else if (changeHourFlag == 1 && changeMinuteFlag == 0)
            {
                hours++;
                if (hours % 24 == 0)
                    hours = 0;
            }
        }
    }

    // when the alarm flag is on set the the alarm desire time
    if (setAlarm == 0x1){
        // have encoder 2 change the hours
        if (enc2 == 0){
            alarmHour--;
            if (alarmHour == 255)
                alarmHour = 23;
        }

        // have encoder 1 change the minutes
        if (enc1 == 0){
            // change minutes
                alarmMinute--;
                if (alarmMinute == 255) // since its unsign 255 = -1
                    alarmMinute = 59;
        }
    }

    // add a counter to determine one second
    count++;
    if ((count % 128) == 0)
    {
        // 1 second has past

        // timer, snooze 
        if (timerFlag == 0x1)
        {
            // count down from snooze
            // display alarm
            // 
            alarmFlag = 1; // display alarm
            // timer on
            timer--;
            if (timer == 0)
            {
                // timer goes off display alarm
                timerFlag = 0; // turn off timer
                alarmFlag = 0;
            }
        }

        // clock
        colonDisplay ^= 0x1; // blinking
        seconds++;

        if ((seconds % 60) == 0)
        {
            minutes++;
            seconds = 0;
            if ((minutes % 60) == 0)
            {
                hours++;
                minutes = 0;
                if (hours % 24 == 0)
                    hours = 0;
            }
        }
    }
    adc_read();     // read the ADC value
    if (adc_result < 100){OCR2 = 222;}
        else {OCR2 = adc_result * 0.4 + 80;}
       
}

/******************************************************************************/
//                                 encoderRead
// This function checks the state of the encoder so see what its behavior is.
// It will return -1 if there is no change within. It will return 1 if the system is
// CW. It will return 0 if the system is CCW.
/******************************************************************************/
uint8_t encoderRead(uint8_t data, uint8_t knob)
{

    // check for encoder
    static uint8_t old_state[4] = {0xff, 0xff, 0xff, 0xff};
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

    if ((new_A != old_state[a_index]) || (new_B != old_state[b_index]))
    { // if change occured
        if ((new_A == 0) && (new_B == 0))
        {
            if (old_state[a_index] == 1)
            {
                count++;
            }
            else
            {
                count--;
            }
        }
        else if ((new_A == 0) && (new_B == 1))
        {
            if (old_state[a_index] == 0)
            {
                count++;
            }
            else
            {
                count--;
            }
        }
        else if ((new_A == 1) && (new_B == 1))
        { // detent position
            if (old_state[a_index] == 0)
            { // one direction
                if (count == 3)
                {
                    return_val = 0;
                }
            }
            else
            { // or the other direction
                if (count == -3)
                {
                    return_val = 1;
                }
            }
            count = 0; // count is always reset in detent position
        }
        else if ((new_A == 1) && (new_B == 0))
        {
            if (old_state[a_index] == 1)
            {
                count++;
            }
            else
            {
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
void barGraph()
{

    SPDR = barGraphDisplay;
    while (bit_is_clear(SPSR, SPIF))
    {
    }                       //wait till data sent out (while loop)
    PORTD |= (1 << PORTD2); //HC595 output reg - rising edge...
    PORTD &= (0 << PORTD2); //and falling edge
}

/******************************************************************************/
//                                 alarmDisplay
// Display "ALARM" on the display if the alarm flag is on.
// Otherwise clear the screen.
/******************************************************************************/

void alarmDisplay()
{
    char lcd_string_array[16] = "     ALARM      ";
    if (alarmFlag == 0x1)
    {
        DDRE |= 1 << PORTE3; // turn off the port of the speaker 
        OCR3A = 0xA000; // turn on the volume
        clear_display(); // clear the display
        string2lcd(lcd_string_array);
    }
}

/******************************************************************************/
//                                 buttonPress
// Different cases for each button pressed
/******************************************************************************/
void buttonPress(uint8_t button)
{

    switch (button)
    {
    case 0:
    {
        // snooze, turn off LCD display
        timer = SNOOZE_TIMER; // reset the timer to 10 seconds
        OCR3A = 0xFFFF; // turn off volume
        timerFlag = 0; // turn off the timer
        alarmFlag = 0; // turn off the alarm
        barGraphDisplay &= ~(1 << 1); // turn off the timer modes
        barGraphDisplay &= ~(1 << 2); // turn off the timer modes
        PORTC &= ~(1 << PORTC0);
        PORTC &= ~(1 << PORTC1);
        alarmMinute = 0;
        alarmHour = 0;
        clear_display();
        clear_display();

        // turn off indication on LED display
        return;
    }
    case 1:
    {
        // alarmFlag ^= 0x1; // show on the LED Display that you want to change the alarm time
        setAlarm ^= 0x1; // toggle the alarm flag
        barGraphDisplay ^= 1 << 1;
        alarmInit++; // many times this button has been pressed
        return;
    }
    case 6:
    {
        // change minutes
        changeMinuteFlag ^= 1;
        barGraphDisplay ^= 1 << 6;
        return;
    }
    case 7:
    {
        // change hours
        changeHourFlag ^= 1;
        barGraphDisplay ^= 1 << 7;
        return;
    }
    default:
        break;
    }
}
/******************************************************************************/
/******************************************************************************/
ISR(TIMER1_COMPA_vect)
{
        PORTC ^= 1 << PORTC0; // turn on right speaker
        PORTC ^= 1 << PORTC1; // turn on left speaker
}


/******************************************************************************/
//                              setVolumeController
/******************************************************************************/
void setVolumeController(){}