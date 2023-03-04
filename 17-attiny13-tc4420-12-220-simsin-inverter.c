#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#define F_CPU 600000L
#include <util/delay.h>

/*
   • ATTINY13
   • Clock 600Khz (osc. 4.8Mhz)
   • 2 TC4420 on PB0, PB1
   • Circuit diagram:
   • PCB:
 */

static void wait_period(uint8_t output)
{
    while((TIFR0 & (1 << OCF0A)) == 0);
    PORTB   = output;
    TCNT0   = 0x00; // reset counter
    TIFR0   |= (1 << OCF0A);
}

int main()
{
    CLKPR   = (1 << CLKPCE);
    CLKPR   = (1 << CLKPS1) | (1 << CLKPS0); //fCPU / 8 = 600 Khz

    DDRB    = (1 << PB1) | (1 << PB0); //ouptut PB1, PB0 - TC4420s
    PORTB   = (1 << PB5) | (1 << PB4); //pull-up PB5 (reset), PB4 (button)

    TCCR0B  = (1 << WGM02) | (1 << CS01) | (1 << CS00); //CTC mode, CLK_IO / 64
    OCR0A   = 40; // for about 50HZ output
    TCNT0   = 0; // reset counter

    while(1)
    {
        wait_period(0x00);
        wait_period(1 << PB0);
        wait_period(0x00);
        wait_period(1 << PB1);
    }
    return 0;
}

