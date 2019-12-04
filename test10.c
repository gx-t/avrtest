#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdlib.h>

#define F_CPU 8000000UL
#include <util/delay.h>

static void effect_0()
{
    static float s[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    static float c[8] = {30, 30, 30, 30, 30, 30, 30, 30};
    const float f[8] = {0.01, 0.0099, 0.0097, 0.0094, 0.0099, 0.0098, 0.0096, 0.0093};

    while(1) {
        for(uint8_t i = 0; i < sizeof(s) / sizeof(s[0]); i ++) {

            c[i] -= s[i] * f[i];
            s[i] += c[i] * f[i];
        }

        OCR0A = s[0] + s[1] + s[2] + s[3] + 120;
        OCR0B = c[0] + c[1] + c[2] + c[3] + 120;
        OCR2A = s[4] + s[5] + s[6] + s[7] + 120;
        OCR2B = c[4] + c[5] + c[6] + c[7] + 120;
    }
}

static void effect_1()
{
    static float s[4] = {0, 0, 0, 0};
    static float c[4] = {127, 127, 127, 127};
    while(1) {

        int r = random();
        for(uint8_t i = 0; i < sizeof(s) / sizeof(s[0]); i ++, r >>= 1) {
            if(r & 1) {
                s[i] = 0;
                c[i] = 127;
            }
        }

        for(uint16_t i = 0; i < 5000; i ++) {
            for(uint8_t j = 0; j < sizeof(s) / sizeof(s[0]); j ++) {
                c[j] -= s[j] * 0.016;
                s[j] += c[j] * 0.016;
                s[j] += (0 - s[j]) * 0.0005;
            }

            OCR0A = (uint8_t)(s[0] + 127);
            OCR0B = (uint8_t)(s[1] + 127);
            OCR2A = (uint8_t)(s[2] + 127);
            OCR2B = (uint8_t)(s[3] + 127);
        }
    }
}

int main(void) {

    DDRD |= _BV(PD6) | _BV(PD5) | _BV(PD3);
    DDRB |= _BV(PB3);

    TCCR0A |= _BV(COM2A1) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
    TCCR0B |= _BV(CS00);

    TCCR2A |= _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21);
    TCCR2B |= _BV(CS00);

    effect_0();
    effect_1();

    return 0;
}

