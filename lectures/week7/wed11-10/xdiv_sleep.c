//Clock frequency stepping via XDIV and sleep
//Anthony Nguyen 11.10.21

/*
*This code demonstrates the effect on power consumption of 
*not having pullups turned on. Also, the code steps through
*a reduction of operating frequency by using the XDIV register
*to show the power savings of reducing the clock frequency.
*
*After stepping down to 0.5Mhz, the processor goes to sleep.
*(AVR power down mode)  The processor sleeps until awakened
*by a falling edge interrupt on INT0.  Then the processor blinks
*PORTB.0 LED to indicate its awake again.
*/

#define F_CPU 16000000UL     //16Mhz clock
//note: effective F_CPU changes as the code executes

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

//****************************  INT0_vect ISR  ********************************
ISR(INT0_vect){
  XDIV  &= ~(1<<XDIVEN); //reset XDIVEN bit to run at 16Mhz again
  MCUCR &= ~(1<<SE);     //disable sleep so it can't happen utill sleep_cpu() 
                         //is executed.
}//ISR
//*****************************************************************************


int main(){
uint8_t i;

//initalize for external falling edge interrupt on pushbutton zero (PORTD.0)
  EICRA = (1<<ISC01);          //falling edge detection 
  EIMSK = (1<<INT0);           //enable interrupts on INT0 
  MCUCR |= (1<<SM1) | (1<<SE); //power down mode and sleep enabled 

/*
*this block sets all ports to inputs with pullups on except PORTB
*which is used as an indicator of what is going on. Comment and
*uncomment to see the difference in power consumption.
*/
  DDRA = 0x00;  PORTA = 0xFF; //all inputs with pullups
  DDRB = 0xFF;  PORTB = 0x00; //blink lights with PORTB 
  DDRC = 0x00;  PORTC = 0xFF; //all inputs with pullups
  DDRD = 0x00;  PORTD = 0xFF; //all inputs with pullups
  DDRE = 0x00;  PORTE = 0xFF; //all inputs with pullups
  DDRF = 0x00;  PORTF = 0xFF; //all inputs with pullups
  DDRG = 0x00;  PORTG = 0xFF; //all inputs with pullups

  sei();  //enable interrupts

  //16mhz clock
  PORTB = 0x01;
  for(i=0; i<=20; i++) _delay_ms(100); //delay 2 sec

  XDIV = 127; //setting for 8 Mhz clock 
  XDIV |= (1<<XDIVEN); //change clock freq
  PORTB = 0x02;
  for(i=0; i<=10; i++) _delay_ms(100); //delay 2 sec
  
  XDIV &= ~(1<<XDIVEN); //reset bit to change setting
  XDIV = 125; //setting for 4 Mhz clock 
  XDIV |= (1<<XDIVEN); //change clock freq
  PORTB = 0x04;
  for(i=0; i<=10; i++) _delay_ms(50); //delay 2 sec
  
  XDIV &= ~(1<<XDIVEN); //reset bit to change setting
  XDIV = 121; //setting for 2 Mhz clock 
  XDIV |= (1<<XDIVEN); //change clock freq
  PORTB = 0x08;
  for(i=0; i<=10; i++) _delay_ms(25); //delay 2 sec

  XDIV &= ~(1<<XDIVEN); //reset bit to change setting
  XDIV = 113; //setting for 1 Mhz clock 
  XDIV |= (1<<XDIVEN); //change clock freq
  PORTB = 0x10;
  for(i=0; i<=5; i++) _delay_ms(25); //delay 2 sec
  
  XDIV &= ~(1<<XDIVEN); //reset bit to change setting
  XDIV = 97; //setting for 0.5 Mhz clock 
  XDIV |= (1<<XDIVEN); //change clock freq
  PORTB = 0x20;
  for(i=0; i<=5; i++) _delay_ms(13); //delay 2 sec

  PORTB = 0x80;  //indicate we are entering sleep mode
  sleep_cpu();   //enter sleep mode, wait for interrupt.....

  //after being awakened by the interrupt, blink PORTB.0 LED

  PORTB =0x00; //reset PORTB LEDs

  while(1){
    for(i=0; i<=5; i++){_delay_ms(100);} //delay 0.5 sec
    PORTB ^= 0x01;                        //blink PORTB bit 2 forever
  }//while(1)

} //main
