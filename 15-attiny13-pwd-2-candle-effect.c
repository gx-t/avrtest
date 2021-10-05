#include <avr/io.h>
#define F_CPU 600000L
#include <util/delay.h>

/*
    ATTINY13 2 PWM
 */

static void cpu_clock_div_1()
{
    CLKPR = 0b10000000;
    CLKPR = 0b00000000;
}

static void cpu_clock_div_8()
{
    CLKPR = 0b10000000;
    CLKPR = 0b00000011;
}


int main()
{
    cpu_clock_div_8();

    DDRB = 1 << DDB1 | 1 << DDB0;
    TCCR0A = 2 << COM0A0 | 2 << COM0B0 | 3 << WGM00;
    TCCR0B = 0 << WGM02 | 1 << CS00;

    int16_t s[] = {0, 0, 0, 0, 0, 0, 0, 0};
    int16_t c[] = {120, 120, 120, 120, 120, 120, 120, 120};

    while(1)
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

    return 0;
}



