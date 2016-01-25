#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define CANDLE_ON_TIME	0x1000;

static uint16_t tm[] = {0x100, 0x120, 0x140, 0x160, 0x180, 0x1A0, 0x1B0}; //candle remaining times

static void init() {
	DDRB	= 0b00000100; //PB2 - output
	PORTB	= 0b00000001; //pull-up on PB0
	DDRD	= 0b01111111; //PORTD 0-6 output
	PORTD	= 0b00000000; //all LEDs on
	TCCR0A	= (1 << COM0A1) | (1 << WGM00);  // phase correct PWM mode
	//	TCCR1A = (1 << COM0A1) | (1 << WGM00);  // phase correct PWM mode
	OCR0A	= 0x00;                          // initial PWM pulse width
	//	OCR1A  = 0xFF;                          // initial PWM pulse width
	//    TCCR0B = (1 << CS01);   // clock source = CLK/8, start PWM
	TCCR0B	= (1 << CS00);   // clock source = CLK/1, start PWM
	//	TCCR1B = (1 << CS00);   // clock source = CLK/1, start PWM
}


static void check_coin()
{
	static uint8_t debounce = 0x00;
	if(debounce) {
		debounce --;
		return;
	}
	if(PINB & 0b00000001) {
		return;
	}
	debounce = 0xFF;
	uint8_t i = sizeof(tm) / sizeof(tm[0]);
	while(i--) {
		if(!tm[i]) {
			tm[i] = CANDLE_ON_TIME;
			PORTD &= ~(1 << i);
			break;
		}
	}
}

static uint8_t oscilate() {
	static float s[] = {0.0, 0.0, 0.0, 0.0, 0.0}; //sines
	static float c[] = {1.0, 1.0, 1.0, 1.0, 1.0}; //cosines
	static float f[] = {0.05, 0.06, 0.075, 0.087, 0.09}; //frequencies
	uint8_t i = sizeof(s) / sizeof(s[0]);
	float res = 0;
	//harmonic synthesis ;)
	while(i--) {
		c[i] -= s[i] * f[i];
		s[i] += c[i] * f[i];
		res += s[i];
	}
	res /= sizeof(s) / sizeof(s[0]);
	return 126 * (res + 1);
}

static void check_candle_times() {
	uint8_t i = sizeof(tm) / sizeof(tm[0]);
	while(i--) {
		if(tm[i]) {
			tm[i]--;
		} else {
			PORTD |= (1 << i);
		}
	}
}

int main(void)
{
	init();
	//program main loop
	while(1) {
		OCR0A = oscilate(); //PWM duty
		check_candle_times();
		check_coin();
//		OCR1A = val;
	}
	return 0;
}

