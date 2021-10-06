#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#define F_CPU 600000L
#include <util/delay.h>

/*
   ATTINY13 2 PWM
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

static void pwm_init()
{
    TCCR0A = 0b10100011; //fast PWM
    TCCR0B = 0b00000001; //no prescaler: clkI/O / 1
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
    pwm_init();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    interrupt_init();
    sei();
}

static uint8_t btn_state()
{
    return 0b010000 & PINB; //press - low, reelase - high
}

int main()
{
    sys_init();

    int16_t s[] = {0, 0, 0, 0, 0, 0, 0, 0};
    int16_t c[] = {120, 120, 120, 120, 120, 120, 120, 120};

    while(1)
    {
        while(btn_state())
        {
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

        OCR0A = 0;
        OCR0B = 0;

        _delay_ms(100);
        while(!btn_state());
        _delay_ms(100);
        sleep_cpu();
        _delay_ms(100);
        while(!btn_state());
        _delay_ms(100);
    }

    return 0;
}

