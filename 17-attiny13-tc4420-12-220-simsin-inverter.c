#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#define F_CPU 600000L
#include <util/delay.h>

/*
   • ATTINY13
   • Clock 600Khz (osc. 4.8Mhz)
   • 2 Soft PWM
   • 2 TC4420 on PWM outputs
   • Circuit diagram:
   • PCB:
 */

ISR(PCINT0_vect)
{
}

static void cpu_clock_div_8()
{
    CLKPR = 0b10000000;
    CLKPR = 0b00000011;
}

static void gpio_init()
{
    DDRB    = 0b000011; //ouptut PB0, PB1 - LEDs
    PORTB   = 0b110000; //pull-up PB5 (reset), PB4 (button)
}

static void interrupt_init()
{
    GIMSK |= 1 << PCIE; // enable PCINT[0:5] pin change interrupt
    PCMSK |= 1 << PCINT4; // configure interrupt at PB4
}

static void sys_init()
{
    cli();
    cpu_clock_div_8();
    gpio_init();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    interrupt_init();
    sei();
}

static uint8_t btn_state()
{
    return 0b010000 & PINB; //press - low, reelase - high
}

static void run()
{
    PORTB = 0b00000001;
    __builtin_avr_delay_cycles(2780);
    PORTB = 0b00000000;
    __builtin_avr_delay_cycles(2780);
    PORTB = 0b00000010;
    __builtin_avr_delay_cycles(2780);
    PORTB = 0b00000000;
    __builtin_avr_delay_cycles(2780);
}

int main()
{
    sys_init();

    while(1)
    {
        PORTB = 0b00000001;
        __builtin_avr_delay_cycles(2780);
        PORTB = 0b00000000;
        __builtin_avr_delay_cycles(2780);
        PORTB = 0b00000010;
        __builtin_avr_delay_cycles(2780);
        PORTB = 0b00000000;
        __builtin_avr_delay_cycles(2780);
    }

    return 0;
}

