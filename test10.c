#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdlib.h>

#define F_CPU 8000000UL
#include <util/delay.h>

static void effect_0()
{
    static int16_t s[2][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}};
    static int16_t c[2][4] = {{30 << 8, 30 << 8, 30 << 8, 30 << 8}, {30 << 8, 30 << 8, 30 << 8, 30 << 8}};
    const int16_t p[2][4] = {{100, 101, 103, 107}, {101, 102, 104, 108}};

    while(1) {

        c[0][0] -= s[0][0] / p[0][0];
        s[0][0] += c[0][0] / p[0][0];

        c[0][1] -= s[0][1] / p[0][1];
        s[0][1] += c[0][1] / p[0][1];

        c[0][2] -= s[0][2] / p[0][2];
        s[0][2] += c[0][2] / p[0][2];

        c[0][3] -= s[0][3] / p[0][3];
        s[0][3] += c[0][3] / p[0][3];

        c[1][0] -= s[1][0] / p[1][0];
        s[1][0] += c[1][0] / p[1][0];

        c[1][1] -= s[1][1] / p[1][1];
        s[1][1] += c[1][1] / p[1][1];

        c[1][2] -= s[1][2] / p[1][2];
        s[1][2] += c[1][2] / p[1][2];

        c[1][3] -= s[1][3] / p[1][3];
        s[1][3] += c[1][3] / p[1][3];

        uint8_t ss_0 = (s[0][0] + s[0][1] + s[0][2] + s[0][3] + (120 << 8)) / 256;
        uint8_t cc_0 = (c[0][0] + c[0][1] + c[0][2] + c[0][3] + (120 << 8)) / 256;
        uint8_t ss_1 = (s[1][0] + s[1][1] + s[1][2] + s[1][3] + (120 << 8)) / 256;
        uint8_t cc_1 = (c[1][0] + c[1][1] + c[1][2] + c[1][3] + (120 << 8)) / 256;

        OCR0A = ss_0;
        OCR0B = ss_1;
        OCR2A = cc_0;
        OCR2B = cc_1;
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
                c[j] -= s[j] / 64;
                s[j] += c[j] / 64;
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

