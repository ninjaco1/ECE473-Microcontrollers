// thermo3_skel.c
// Anthony Nguyen
// 11.15.2011 (revised 11.3.2021)

//Demonstrates basic functionality of the LM73 temperature sensor
//Uses the mega128 board and interrupt driven TWI.
//Display is the raw binary output from the LM73.
//PD0 is SCL, PD1 is SDA.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
// #include <utils/twi.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"
#include "lm73_functions.h"
#include "twi_master.h"
#include "uart_functions.h"

// #define TEMP_MASTER


// uart functions 
volatile uint8_t rcv_rdy;
char rx_char;
uint8_t send_seq = 0; // transmit sequence number
char lcd_string[3]; // hold value of sequence number


// lm73 fucntions
char lcd_string_array[16]; //holds a string to refresh the LCD
char lcd_string_array_F[16]; //holds a string for F
char lcd_string_array_C[16]; //holds a string for C
uint8_t i;                 //general purpose index

extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];
ISR(USART0_RX_vect);
//********************************************************************
//                            spi_init
//Initalizes the SPI port on the mega128. Does not do any further
// external device specific initalizations.
//********************************************************************
void spi_init(void)
{
    DDRB |= 0x07; //Turn on SS, MOSI, SCLK
    //mstr mode, sck=clk/2, cycle 1/2 phase, low polarity, MSB 1st,
    //no interrupts, enable SPI, clk low initially, rising edge sample
    SPCR = (1 << SPE) | (1 << MSTR);
    SPSR = (1 << SPI2X); //SPI at 2x speed (8 MHz)
} //spi_init

/***********************************************************************/
/*                                main                                 */
/***********************************************************************/
#ifdef TEMP_MASTER
int main()// master mode
{
    uint16_t lm73_temp; //a place to assemble the temperature from the lm74
    // uint16_t lm73_temp_C, lm73_temp_F;
    DDRF |= 0x08; // lcd strobe bit
    spi_init(); //initalize SPI
    lcd_init(); //initalize LCD (lcd_functions.h)
    init_twi(); //initalize TWI (twi_master.h)
    uart_init(); // initalize UART (uart_functions.h)
    sei(); //enable interrupts before entering loop
    // DDRE = ~((1 << PORTE0) | (1 << PORTE1)); // making the ports inputs
    //set LM73 mode for reading temperature by loading pointer register
    // lm73_wr_buf[0] = LM73_PTR_TEMP;             //load lm73_wr_buf[0] with temperature pointer address
    // twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 2); //start the TWI write process
    _delay_ms(2);                               //wait for the xfer to finish

    clear_display(); //clean up the display
    // cursor_home();

    while (1)
    {                                               //main while loop
        _delay_ms(100);                             //tenth second wait
        // clear_display();                            //wipe the display
        // ************** start rcv portion *********************
        if(rcv_rdy == 1){
            clear_display();
            string2lcd(lcd_string_array); // write out string if its ready
            rcv_rdy = 0;
            cursor_home();
        }
        // *************** end rcv portion ***********************
    }                                               //while
} //main

ISR(USART0_RX_vect)
{
  static uint8_t i;
  rx_char = UDR0;               //get character
  lcd_string_array[i++] = rx_char; //store in array
                                //if entire string has arrived, set flag, reset index
  if (rx_char == '\0')
  {
    rcv_rdy = 1;
    // lcd_string_array[--i] = (' '); //clear the count field
    // lcd_string_array[i + 1] = (' ');
    // lcd_string_array[i + 2] = (' ');
    i = 0;
  }
}

#else
int main()
{ // slave
    int16_t lm73_temp; //a place to assemble the temperature from the lm74
    float lm73_temp_C, lm73_temp_F;
    spi_init(); //initalize SPI
    init_twi(); //initalize TWI (twi_master.h)
    uart_init(); // initalize UART (uart_functions.h)
    lcd_init(); // initalize LCD
    sei(); //enable interrupts before entering loop
    // DDRE = (1 << PORTE0) | (1 << PORTE1); // making the ports outputs
    //set LM73 mode for reading temperature by loading pointer register
    lm73_wr_buf[0] = LM73_PTR_TEMP;             //load lm73_wr_buf[0] with temperature pointer address
    twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 2); //start the TWI write process
    _delay_ms(2);                               //wait for the xfer to finish

    clear_display(); //clean up the display
    // cursor_home();

    while (1)
    {                                               //main while loop
        _delay_ms(1000);                             //tenth second wait
        clear_display();                            //wipe the display
        twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); //read temperature data from LM73 (2 bytes)
        _delay_ms(2);                               //wait for it to finish
        lm73_temp = lm73_rd_buf[0];                 //save high temperature byte into lm73_temp
        lm73_temp = lm73_temp << 8;                 //shift it into upper byte
        lm73_temp |= lm73_rd_buf[1];                //"OR" in the low temp byte to lm73_temp
        lm73_temp_C = lm73_temp / (float)256;               // how to find the temp in C
        lm73_temp_F = (lm73_temp_C * 9 / 5) + 32;    // convert C to F
        dtostrf(lm73_temp_C, 0, 1, lcd_string_array_C); // converting float to string
        dtostrf(lm73_temp_F, 0, 1, lcd_string_array_F); // converting float to string
        // itoa(lm73_temp_C, lcd_string_array, 10);      //convert to string in array with itoa() from avr-libc
        strcpy(lcd_string_array, lcd_string_array_C); // add C degrees
        strcat(lcd_string_array, "C ");
        strcat(lcd_string_array, lcd_string_array_F);
        strcat(lcd_string_array, "F");
        string2lcd(lcd_string_array);               //send the string to LCD (lcd_functions)

        // *************** start tx portion **********************
        uart_puts(lcd_string_array); // put what you want display, put string
        uart_putc('\0'); // null character
        // **************** end tx portion **********************

    }                                               //while
}

ISR(USART0_RX_vect){
    rx_char = UDR0;
    if (rx_char == '\0')
        rcv_rdy = 1;
}




#endif
