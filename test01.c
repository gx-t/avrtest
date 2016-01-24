#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>


int main(void)
{
	DDRB   = 0b00000100; //PB2 - output
	DDRD   = 0b01111111; //PORTD 0-6 output
	PORTD  = 0b00000000; //all LEDs on
	TCCR0A = (1 << COM0A1) | (1 << WGM00);  // phase correct PWM mode
//	TCCR1A = (1 << COM0A1) | (1 << WGM00);  // phase correct PWM mode
	OCR0A  = 0x00;                          // initial PWM pulse width
//	OCR1A  = 0xFF;                          // initial PWM pulse width
	//    TCCR0B = (1 << CS01);   // clock source = CLK/8, start PWM
	TCCR0B = (1 << CS00);   // clock source = CLK/1, start PWM
//	TCCR1B = (1 << CS00);   // clock source = CLK/1, start PWM
	static float s[] = {0.0, 0.0, 0.0, 0.0, 0.0}; //sines
	static float c[] = {1.0, 1.0, 1.0, 1.0, 1.0}; //cosines
	static float f[] = {0.05, 0.06, 0.075, 0.087, 0.09}; //frequencies
	static uint16_t tm[] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF}; //candel remaining times
	//program main loop
	while(1) {
		uint8_t i = sizeof(s) / sizeof(s[0]);
		float res = 0;
		//harmonic synthesis ;)
		while(i--) {
			c[i] -= s[i] * f[i];
			s[i] += c[i] * f[i];
			res += s[i];
		}
		res /= sizeof(s) / sizeof(s[0]);
		uint8_t val = 126 * (res + 1);
		OCR0A = val; //PWM duty
		i = sizeof(tm) / sizeof(tm[0]);
		//check candles on times
		#if 0
		while(i--) {
			if(tm[i]) {
				tm[i]--;
			} else {
				PORTD |= (1 << i);
			}
		}
		#endif
		//debounce counter
		static uint8_t debounce = 0;
		if(debounce) {
			debounce --;
			continue;
		}
		//probably new coin dropped
		if(PORTB & 0b00000001) {
			debounce = 0xFF;
			i = sizeof(tm) / sizeof(tm[0]);
			//find first free candel
			while(i--) {
				if(!tm[i]) {
					
					//set candel on duration
					tm[i] = 10000;
					break;
				}
			}
		}
//		OCR1A = val;
	}
}

