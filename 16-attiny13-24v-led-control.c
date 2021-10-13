#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#define F_CPU 600000L
#include <util/delay.h>

/*
   • ATTINY13
   • Clock 600Khz (osc. 4.8Mhz)
   • 2 PWM, phase correct mode
   • 24v LED control
   • 2 paralell AO3400 NMOS
   • Circuit diagram:
   • PCB:
 */

static void cpu_clock_div_8()
{
    CLKPR = 0b10000000;
    CLKPR = 0b00000011;
}

static void gpio_init()
{
    DDRB    = 0b001001; //ouptut PB0 - gates, PB2 - sensor, PB3 - LED, PB4 - button
    PORTB   = 0b110100; //pull-up PB2 (sensor), PB4 (button), PB5 (reset)
}

static void pwm_init()
{
    TCCR0A = 0b10000001; //phase correct PWM on PB0
    TCCR0B = 0b00000001; //no prescaler: clkI/O / 1
}

static void sys_init()
{
    cli();
    cpu_clock_div_8();
    gpio_init();
    pwm_init();
    sei();
}

static uint8_t btn_state()
{
    return 0b010000 & PINB; //press - low, reelase - high
}

static void step()
{
    static int16_t s[] = {0, 0, 0, 0, 0, 0, 0, 0};
    static int16_t c[] = {120, 120, 120, 120, 120, 120, 120, 120};

    uint16_t v1 = 0;
    uint16_t v2 = 0;
    for(uint8_t i = 0; i < sizeof(s) / sizeof(*s); i ++) {
        c[i] -= s[i] / (i + 15);
        s[i] += c[i] / (i + 15);
        v1 += s[i];
        v2 += c[i];
    }
    OCR0A = v1 / (sizeof(s) / sizeof(*s)) + 120;
    OCR0B = v2 / (sizeof(c) / sizeof(*c)) + 120;
}

int main()
{
    sys_init();

    while(1)
    {
        while(btn_state())
            step();


       _delay_ms(50);
        while(!btn_state())
            step();
        OCR0A = 0;
        OCR0B = 0;
       _delay_ms(50);
        while(btn_state());
       _delay_ms(50);
        while(!btn_state());
       _delay_ms(50);
    }

    return 0;
}

