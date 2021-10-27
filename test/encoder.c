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

static uint8_t barGraphOutput = 0;
static uint8_t incrementFlag = -1;


ISR(TIMER0_OVF_vect);

void spi_init(void);
void tcnt0_init(void);
void barGraph(uint8_t);
uint8_t encoderRead(uint8_t data, uint8_t knob);


int main() {

    DDRB = 0xF0; // set port B bits 4-7 B as outputs, decoder
    DDRE |= 0b01000000; // set E6 to output
    DDRD |= 0b00001100; // slave select pins

    tcnt0_init();  //initalize counter timer zero
    spi_init();    //initalize SPI port
    sei();         //enable interrupts before entering loop
 

    while(1){



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
    // TCCR0  |= (1 << CS02) | (1 << CS01) | (0 << CS00);
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
    // getting the data from serial out

  


    // before you read from SPI for the encoder
    // make sure to turn the clk_inh on
    // and turn shift/load low
    // delay for a bit
    // then turn clk_inh off, and shift/load high
    // then read from spi


   
  
    SPDR = barGraphOutput;
    PORTD |= 1 << PORTD3; // turning on clk_inh
    PORTE &= 0 << PORTE6; // turning SH/LD low

    _delay_ms(20);
    PORTD &= 0 << PORTD3; // turning off clk_inh
    PORTE |= 1 << PORTE6;// turing SH/LD high

    serial_out = SPDR;
    while (bit_is_clear(SPSR,SPIF)){}               //wait till data sent out (while loop)
    
    uint8_t temp = encoderRead(serial_out,0);
    uint8_t temp1 = encoderRead(serial_out, 1);
    if (temp != -1)
        incrementFlag = temp;
    uint8_t rotation[2] = {temp, temp1};


    barGraph(rotation[0]);
    


}


void barGraph(uint8_t increment){
    uint8_t display_mode; //holds count for display 

    // top half on for increment
    if (increment == 1)
        display_mode = 0xF0;
    else if (increment == 0) // bot half on for decrement
        display_mode = 0x0F;
    else
        display_mode = 0b10101010; // otherwise display every other
    

   
    barGraphOutput = display_mode;               //send to display 
    // while (bit_is_clear(SPSR,SPIF)){}               //wait till data sent out (while loop)
    PORTD |= (1 << PORTD2);          //HC595 output reg - rising edge...
    PORTD &= (0 << PORTD2);          //and falling edge
 

}

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