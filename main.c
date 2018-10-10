/* 
    Copyright 2018 Alistair Buxton <a.j.buxton@gmail.com>
    This file is part of stepper.
    stepper is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    avr-teletext is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with stepper.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "avr_mcu_section.h"


// trace struct for simavr

const struct avr_mmcu_vcd_trace_t _mytrace[]  _MMCU_ = {

    { AVR_MCU_VCD_SYMBOL("B0"), .mask = (1 << 0), .what = (void*)&PORTB, },
    { AVR_MCU_VCD_SYMBOL("B1"), .mask = (1 << 1), .what = (void*)&PORTB, },
    { AVR_MCU_VCD_SYMBOL("B2"), .mask = (1 << 2), .what = (void*)&PORTB, },
    { AVR_MCU_VCD_SYMBOL("B3"), .mask = (1 << 3), .what = (void*)&PORTB, },
    { AVR_MCU_VCD_SYMBOL("B4"), .mask = (1 << 4), .what = (void*)&PORTB, },
    { AVR_MCU_VCD_SYMBOL("B5"), .mask = (1 << 5), .what = (void*)&PORTB, },
    { AVR_MCU_VCD_SYMBOL("B6"), .mask = (1 << 6), .what = (void*)&PORTB, },
    { AVR_MCU_VCD_SYMBOL("B7"), .mask = (1 << 7), .what = (void*)&PORTB, },

    { AVR_MCU_VCD_SYMBOL("C0"), .mask = (1 << 0), .what = (void*)&PORTC, },
    { AVR_MCU_VCD_SYMBOL("C1"), .mask = (1 << 1), .what = (void*)&PORTC, },
    { AVR_MCU_VCD_SYMBOL("C2"), .mask = (1 << 2), .what = (void*)&PORTC, },
    { AVR_MCU_VCD_SYMBOL("C3"), .mask = (1 << 3), .what = (void*)&PORTC, },
    { AVR_MCU_VCD_SYMBOL("C4"), .mask = (1 << 4), .what = (void*)&PORTC, },
    { AVR_MCU_VCD_SYMBOL("C5"), .mask = (1 << 5), .what = (void*)&PORTC, },
    { AVR_MCU_VCD_SYMBOL("C6"), .mask = (1 << 6), .what = (void*)&PORTC, },
    { AVR_MCU_VCD_SYMBOL("C7"), .mask = (1 << 7), .what = (void*)&PORTC, },

    { AVR_MCU_VCD_SYMBOL("D0"), .mask = (1 << 0), .what = (void*)&PORTD, },
    { AVR_MCU_VCD_SYMBOL("D1"), .mask = (1 << 1), .what = (void*)&PORTD, },
    { AVR_MCU_VCD_SYMBOL("D2"), .mask = (1 << 2), .what = (void*)&PORTD, },
    { AVR_MCU_VCD_SYMBOL("D3"), .mask = (1 << 3), .what = (void*)&PORTD, },
    { AVR_MCU_VCD_SYMBOL("D4"), .mask = (1 << 4), .what = (void*)&PORTD, },
    { AVR_MCU_VCD_SYMBOL("D5"), .mask = (1 << 5), .what = (void*)&PORTD, },
    { AVR_MCU_VCD_SYMBOL("D6"), .mask = (1 << 6), .what = (void*)&PORTD, },
    { AVR_MCU_VCD_SYMBOL("D7"), .mask = (1 << 7), .what = (void*)&PORTD, },

};


// macro to do nothing for 4 cycles
#define nop() __asm__ __volatile__("nop\nnop\nnop\nnop");

// this assumes 16MHz clock and 8x prescaler
#define USECS(n) (n*2)

void hw_setup(void)
{
    // configure GPIOs
    // TODO

    // configure UART
    // TODO

    // configure timer interrupt
    TCCR0A = 0x02;          // CTC mode
    TCCR0B = 0x02;          // 8x prescaler
    OCR0A = USECS(50);      // TOP
    TIMSK0 |= _BV(OCIE0A);  // unmask interrupt
}


volatile int direction = 0;  // bit field containing direction of each stepper. 1 for press, 0 for release.
int previous_direction = 0;  // bit field containing the previous directions of each stepper.
int onoff = 0;               // bit field containing the state of the step pulse lines. 1 for on, 0 for off.

// if you need values bigger that 255, change these to unsigned int.
// try to avoid that if possible though as 16 bit operations are slower.

unsigned char position[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};             // current position in half steps
unsigned char max[11] = {55, 50, 45, 40, 35, 30, 25, 20, 20, 20, 20};       // maximum position in half steps (the minimum is zero)
unsigned char ontime[11] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};       // amount of time units to keep the step pulse on during motion
unsigned char offtime[11]  = {20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30};  // amount of time units to keep the step pulse off during motion
unsigned char waits[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};                // current number of time units that the step pulse has been on or off during motion
unsigned char ramp[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};                 // number of half steps that the stepper has moved during the current motion


ISR(TIMER0_COMPA_vect)
{
    PORTD |= 0x80; // raise the high bit of port D so we can measure speed

    // work out which steppers changed direction
    int direction_changed = direction ^ previous_direction;
    previous_direction = direction;

    // in this for loop i is the stepper number and m is (1<<i)
    // apparently gcc is not smart enough to optimize all the (1<<i)
    // so the m variable gives it a hint to keep it around

    for (int i=0, m=1; i<11; i++, m=m<<1) {             //           <-------------------------------------------+
        if(waits[i] == 0) {                             //                                                       |
            if (direction&m) {                          // if the stepper is moving down (to press the button)   |
                if (position[i] == max[i]) {            //     if the stepper is all the way down already        |
                    onoff &= ~m;                        //         turn off the step pulse line bit              |
                    continue;                           //         continue to the next motor   >----------------+
                } else {                                //     else                                              |
                    position[i]++;                      //         increment the position                        |
                }                                       //                                                       |
            } else {                                    // else                                                  |
                if (position[i] == 0) {                 //     if the stepper is all the way up already          |
                    onoff &= ~m;                        //         turn off the step pulse line bit              |
                    continue;                           //         continue to the next motor   >----------------+
                } else {                                //     else
                    position[i]--;                      //         decrement the position
                }
            }
            onoff ^= m;                                 // toggle the step pulse line bit
            if (onoff&m) {                              // if the step pulse line is now on
                waits[i] = ontime[i];                   //     set waits to the on time for this motor
            } else {                                    // else
                waits[i] = offtime[i] - (ramp[i]>>4);   //     set waits to the off time for this motor minus the number of half steps already completed divided by 16.
            }                                           //     speed will therefore increase during the course of motion, up to 15 (255>>4) time units less than the base off time.
        }
        waits[i]--;                                     // decrement waits
        if (direction_changed&m) {                      // if direction changed
            ramp[i] = 0;                                //     reset ramp step count
        } else if (ramp[i] < 255) {                     // else if ramp less than maximum ramp
            ramp[i]++;                                  //     increment ramp step count
        }
    }

    PORTB = direction&0xff;                             // now send the control signals to the GPIOs.
    PORTC = onoff&0xff;                                 // NOTE: you will have to consider which pin each signal is connected to
    PORTD = ((direction>>8)&0x7)|((onoff>>5)&0x38);     //       in order to make this optimal.
}


int main(void)
{
    hw_setup();
    sei();

    for(;;)
    {
        for (int button=0; button<11; button++) {
            direction ^= (1<<button);
            for (int i=0; i<120; i++) {
                nop(); nop(); nop(); nop();
                nop(); nop(); nop(); nop();
                nop(); nop(); nop(); nop();
                nop(); nop(); nop(); nop();
            }
        }
    }

    // Never reached.
    return(0);
}
