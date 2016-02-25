#define F_CPU 4000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

static uint8_t b[7];
ISR(TIMER0_COMPA_vect)
{
	static uint8_t i = 0;
	PORTD = 0b1111111;
	PORTB = b[i];
	PORTD = ~(0b1 << i);
	i ++;
	if(i == 7) {
		i = 0;
	}
}

int main (void)
{
	uint8_t i, j, rep, rep1;
	PORTB = 0x0;
    DDRB = 0xff;
    DDRD = 0b1111111;
	PORTD = 0b1111111;
	OCR0A  = 0xff;
	TCCR0A = 0x02;
	TIFR |= 0x01;
	TIMSK = 0x01;
	TCCR0B = 0x01;
	sei();
    while(1) 
	{
		rep1 = 2;
		while(rep1 --) {
			for(rep = 0; rep < 4; rep ++) {
				for(j = 0; j < 7; j ++) {
					for(i = 0; i < 8; i ++) {
						b[j] = 1 << i;
						__builtin_avr_delay_cycles(20000);
						b[j] = 0;
					}
				}
			}
			while(rep --) {
				j = 7;
				while(j --) {
					i = 8;
					while(i --) {
						b[j] = 1 << i;
						__builtin_avr_delay_cycles(20000);
						b[j] = 0;
					}
				}
			}
		}

		rep1 = 2;
		while(rep1 --) {
			rep = 8;
			while(rep --) {
				for(i = 0; i < 8; i ++) {
					b[0] = b[1] = b[2] = b[3] = b[4] = b[5] = b[6] = 1 << i;
					__builtin_avr_delay_cycles(100000);
					b[0] = b[1] = b[2] = b[3] = b[4] = b[5] = b[6] = 0;
				}
			}
			rep = 8;
			while(rep --) {
				i = 8;
				while(i --) {
					b[0] = b[1] = b[2] = b[3] = b[4] = b[5] = b[6] = 1 << i;
					__builtin_avr_delay_cycles(100000);
					b[0] = b[1] = b[2] = b[3] = b[4] = b[5] = b[6] = 0;
				}
			}
		}

		rep = 4;
		while(rep --) {
			for(j = 0; j < 7; j ++) {
				for(i = 0; i < 8; i ++) {
					b[j] |= 1 << i;
					__builtin_avr_delay_cycles(20000);
				}
			}
			while(j --) {
				i = 8;
				while(i --) {
					b[j] &= ~(1 << i);
					__builtin_avr_delay_cycles(20000);
				}
			}
		}

		rep = 4;
		while(rep --) {
			j = 7;
			while(j --) {
				i = 8;
				while(i --)  {
					b[j] |= 1 << i;
					__builtin_avr_delay_cycles(20000);
				}
			}
			for(j = 0; j < 7; j ++)  {
				for(i = 0; i < 8; i ++) {
					b[j] &= ~(1 << i);
					__builtin_avr_delay_cycles(20000);
				}
			}
		}

		rep1 = 2;
		while(rep1 --) {
			rep = 4;
			while(rep --) {
				for(i = 0; i < 8; i ++) {
					b[6] |= 1 << i;
					b[0] = b[1] = b[2] = b[3] = b[4] = b[5] = b[6];
					__builtin_avr_delay_cycles(100000);
				}
				i = 8;
				while(i --) {
					b[6] &= ~(1 << i);
					b[0] = b[1] = b[2] = b[3] = b[4] = b[5] = b[6];
					__builtin_avr_delay_cycles(100000);
				}
			}

			rep = 4;
			while(rep --) {
				i = 8;
				while(i --) {
					b[6] |= 1 << i;
					b[0] = b[1] = b[2] = b[3] = b[4] = b[5] = b[6];
					__builtin_avr_delay_cycles(100000);
				}
				for(i = 0; i < 8; i ++) {
					b[6] &= ~(1 << i);
					b[0] = b[1] = b[2] = b[3] = b[4] = b[5] = b[6];
					__builtin_avr_delay_cycles(100000);
				}
			}
		}
	}
}
