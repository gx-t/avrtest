#include <avr/io.h>
#define F_CPU 1200000UL
#include <util/delay.h>

/*
    ATTINY13 2 PWM
 */

static void cpu_clock_set_1200khz()
{
    CLKPR = 0b10000000;
    CLKPR = 0b00000011;
}

int main()
{
    cpu_clock_set_1200khz();

    DDRB = 1 << DDB1 | 1 << DDB0;
    TCCR0A = 2 << COM0A0 | 2 << COM0B0 | 3 << WGM00;
    TCCR0B = 0 << WGM02 | 1 << CS00;

    int8_t s[] = {0, 0, 0, 0, 0, 0, 0, 0};
    int8_t c[] = {120, 120, 120, 120, 120, 120, 120, 120};
    int8_t f[] = {27, 28, 29, 30, 31, 32, 33, 34};

    for (;;)
    {
        c[0] -= s[0] / f[0];
        s[0] += c[0] / f[0];

        c[1] -= s[1] / f[1];
        s[1] += c[1] / f[1];

        c[2] -= s[2] / f[2];
        s[2] += c[2] / f[2];

        c[3] -= s[3] / f[3];
        s[3] += c[3] / f[3];

        c[4] -= s[4] / f[4];
        s[4] += c[4] / f[4];

        c[5] -= s[5] / f[5];
        s[5] += c[5] / f[5];

        c[6] -= s[6] / f[6];
        s[6] += c[6] / f[6];

        c[7] -= s[7] / f[7];
        s[7] += c[7] / f[7];

        OCR0A = s[0] / 4 + s[1] / 4 + s[2] / 4 + s[3] / 4 + 120;
        OCR0B = s[4] / 4 + s[5] / 4 + s[6] / 4 + s[7] / 4 + 120;
    }

    return 0;
}



