#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

int8_t encoder_chk(uint8_t encoder_var);

int main(){



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