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

#include <stdlib.h>

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


// Macro to do nothing for 4 cycles
#define nop() __asm__ __volatile__("nop\nnop\nnop\nnop");

// This assumes 8x prescaler.
#define USECS(n) ((int)(n * (F_CPU/8000000.0)))

void hw_setup(void)
{
    // configure GPIOs
    // TODO

    // configure UART - no interrupts
    //UBRRL = (F_CPU / (16UL * 115200)) - 1;
    //UCSRB = _BV(TXEN) | _BV(RXEN);

    // Configure TIMER0 for pulse generator
    TCCR0A = 0x02;          // CTC mode
    TCCR0B = 0x02;          // 8x prescaler
    OCR0A = USECS(50);      // counts between interrupts
    TIMSK0 |= _BV(OCIE0A);  // unmask interrupt

    // Configure TIMER1 for test data generator
    TCCR1A = 0x00;          // CTC mode
    TCCR1B = 0x0a;          // CTC mode, 8x prescaler
    OCR1A = USECS(16666);   // counts between interrupts
    TIMSK1 |= _BV(OCIE1A);  // unmask interrupt
}


volatile uint16_t set_direction = 0;  // bit field containing direction of each stepper. 1 for press, 0 for release.

ISR(TIMER0_COMPA_vect)
{
    PORTD |= 0x80; // Raise the high bit of port D for timing measurement.

    // This interrupt runs every 50us and updates the control signals to the
    // stepper drivers.

    // The ontime of the step pulse does not matter according to the datasheet,
    // as long as it is longer than 1.8us. So make it a single time unit and
    // then there is no need to count both periods.

    // Then the total step time is: (offtime + 1) * interrupt period
    // So (19 + 1) * 50us = 1ms = 5 rotations per second.

    // static variables remember their value between calls:

    static uint16_t direction = 0;  // direction stepper should move (1=press, 0=release)

    // If you need values bigger that 255, change these to uint16_t.
    // Try to avoid that if possible though as 16 bit operations are slower.

    static uint8_t position[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};             // current position in steps
    static uint8_t max[11] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10};       // maximum position in steps (the minimum is zero)
    static uint8_t offtime[11]  = {19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19};  // amount of time units to keep the step pulse off during motion
    static uint8_t waits[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};                // current number of time units that the step pulse has been off

    // onoff is not static, and will be recalculated each time:

    uint16_t onoff = 0; // by default clear all pulses

    // Calculate steppers which have changed direction:

    uint16_t dir_delta = (set_direction ^ direction);

    // In this for loop i is the stepper number and m is (1<<i).
    // Apparently gcc is not smart enough to optimize all the (1<<i)
    // so the m variable gives it a hint to keep it around.

    for (int i=0, m=1; i<11; i++, m=m<<1) {     //           <---------------------------------------------+
        if(waits[i] == 0) {                     // if stepper finished a step                              |
            direction ^= (dir_delta&m);         //     update direction for this stepper (see note below)  |
            if (direction&m) {                  //     if the stepper is moving down (to press the button) |
                if (position[i] == max[i]) {    //         if the stepper is all the way down already      |
                    continue;                   //             continue to the next motor   >--------------+
                } else {                        //         else                                            |
                    position[i]++;              //             increment the position                      |
                }                               //                                                         |
            } else {                            //     else                                                |
                if (position[i] == 0) {         //         if the stepper is all the way up already        |
                    continue;                   //             continue to the next motor   >--------------+
                } else {                        //         else
                    position[i]--;              //             decrement the position
                }
            }
            onoff |= m;                         //     send a pulse to this stepper
            waits[i] = offtime[i];              //     set the delay counter for this pulse
        } else {                                // else step is in progress
            waits[i]--;                         //     decrement waits
        }
    }

    // Note: by updating the direction only when waits == 0 the stepper will
    // never change direction mid step. This avoids a bug where the position
    // variable moves opposite to the direction the stepper actually stepped
    // and ensures that the minimum HOLD time is respected.

    // Send the calculated signals to GPIO pins:

    // Write the directions first:
    PORTB = direction&0xff;
    PORTD = (PORTD&(~(0x7)))|((direction>>8)&0x7);

    nop(); nop(); nop(); // Wait 12 cycles = 750ns to respect SETUP time.

    // Now write the pulses:
    PORTC = onoff&0xff;
    PORTD = (PORTD&(~(0x38)))|((onoff>>5)&0x38);

    PORTD &= 0x7f; // Lower high bit of port for timing measurement.
}


volatile uint16_t randomtmp;

ISR(TIMER1_COMPA_vect)
{
    // This interrupt runs approximately every 1/60th of a second (ie every 
    // frame) and changes the direction of the steppers randomly. It has two
    // test modes and it changes between modes every 128 frames.

    // If TIMER1 interrupt takes too long we can miss TIMER0 interrupts
    // so use a temporary variable filled from main() instead of calling
    // rand() in the interrupt. Making random numbers is slow.

    static uint8_t mode = 0;
    if ((mode++)&0x80) {
        // Completey randomize the directions of every stepper:
        set_direction = randomtmp;
    } else {
        // Reverse the direction of (at most) one stepper:
        set_direction ^= 1<<(randomtmp&0xf);
    }
}


void main(void)
{
    // Configure hardware:
    hw_setup();

    // Enable interrupts:
    sei();

    // Generate random numbers forever:
    for(;;) {
        randomtmp = rand();
    }
}
