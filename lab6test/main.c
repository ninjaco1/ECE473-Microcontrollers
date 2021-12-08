#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <util/twi.h>
#include <avr/eeprom.h>
// #include <util/delay.h>
#include "uart_functions.h"

#include "twi_master.h" //my defines for TWCR_START, STOP, RACK, RNACK, SEND
#include "si4734.h"

volatile uint16_t current_fm_freq = 10110; //0x2706, arg2, arg3; 99.9Mhz, 200khz steps
volatile uint8_t current_volume;
extern uint8_t si4734_wr_buf[9];
extern uint8_t si4734_rd_buf[15];
extern uint8_t si4734_tune_status_buf[8];
extern volatile uint8_t STC_interrupt; //indicates tune or seek is done

ISR(TIMER1_COMPA_vect)
{

}
//You will also need this:
//******************************************************************************
//                          External Interrupt 7 ISR
// Handles the interrupts from the radio that tells us when a command is done.
// The interrupt can come from either a "clear to send" (CTS) following most
// commands or a "seek tune complete" interrupt (STC) when a scan or tune command
// like fm_tune_freq is issued. The GPIO2/INT pin on the Si4734 emits a low
// pulse to indicate the interrupt. I have measured but the datasheet does not
// confirm a width of 3uS for CTS and 1.5uS for STC interrupts.
//
// I am presently using the Si4734 so that its only interrupting when the
// scan_tune_complete is pulsing. Seems to work fine. (12.2014)
//
// External interrupt 7 is on Port E bit 7. The interrupt is triggered on the
// rising edge of Port E bit 7.  The i/o clock must be running to detect the
// edge (not asynchronouslly triggered)
//******************************************************************************
ISR(INT7_vect) { STC_interrupt = TRUE; }
//******************************************************************************
int main()
{
    init_twi();

    DDRE = 0xFF;
    DDRC = 0x08;
    sei();
    PORTE |= 0x04; //radio reset is on at powerup (active high)

    // ****************************************************************************
    //Fast PWM, set on compare match
    TCCR3A |= (1 << WGM31) | (1 << COM3A1) | (1 << COM3A0);                               // inverting mode
    TCCR3B |= /*(1 << ICES3) |*/ (1 << WGM33) | (1 << WGM32) | (1 << CS30) | (1 << CS31); //No prescale
    TCCR3C = 0x00;                                                                        //no force compare

    // OCR3A = 0xFFFF; // initally no volume
    OCR3A = 0x1000;
    ICR3 = 0xF000; // top value
    // **************************************************************************************************

    //TCNT1 (Alarm PWM)
    //	TCCR1A |= 0x00;
    TCCR1B |= (1 << WGM12) | (1 << CS10);
    //	TCCR1C |= 0x00;
    TIMSK |= (1 << OCIE1A);
    //set frequency
    OCR1A = 0x3210;
    // hi = 0;
    EICRB |= (1 << ISC71) | (1 < ISC70);
    EIMSK |= (1 << INT7);
    //hardware reset of Si4734
    PORTE &= ~(1 << PE7); //int2 initially low to sense TWI mode
    DDRE |= 0x80;         //turn on Port E bit 7 to drive it low
    PORTE |= (1 << PE2);  //hardware reset Si4734
    _delay_us(200);       //hold for 200us, 100us by spec
    PORTE &= ~(1 << PE2); //release reset
    _delay_us(30);        //5us required because of my slow I2C translators I suspect
                          //Si code in "low" has 30us delay...no explaination given
    DDRE &= ~(0x80);      //now Port E bit 7 becomes input from the radio interrupt

    fm_pwr_up(); //power up radio
    while (twi_busy())
    {
    } //spin while TWI is busy
    current_fm_freq = 9990;
    fm_tune_freq(); //tune to frequency

    while (1)
    {
    } //while
} //main