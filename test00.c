#define F_CPU 4000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/*
	7 x 8 LED matrix
	PORTD0-6 LED -
	PORTB0-7 LED +
	Battery LiIon (3.6V)
*/

static uint8_t b[7];
ISR(TIMER0_COMPA_vect) {
	static uint8_t i = 0;
	PORTD = 0b1111111;
	PORTB = b[i];
	PORTD = ~(0b1 << i);
	i ++;
	if(i == 7) {
		i = 0;
	}
}

static void sys_init() {
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
}

static void ff_s_l() {
	uint8_t rep = 8;
	while(rep --) {
		uint8_t i;
		for(i = 0; i < 8; i ++) {
			b[0] = b[1] = b[2] = b[3] = b[4] = b[5] = b[6] = 1 << i;
			__builtin_avr_delay_cycles(100000);
			b[0] = b[1] = b[2] = b[3] = b[4] = b[5] = b[6] = 0;
		}
	}
}

static void ff_s_r() {
	uint8_t rep = 8;
	while(rep --) {
		uint8_t i = 8;
		while(i --) {
			b[0] = b[1] = b[2] = b[3] = b[4] = b[5] = b[6] = 1 << i;
			__builtin_avr_delay_cycles(100000);
			b[0] = b[1] = b[2] = b[3] = b[4] = b[5] = b[6] = 0;
		}
	}
}

static void fast_bi() {
	uint8_t rep1, rep, j, i;
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
}

static void fast_s_bi() {
	uint8_t rep, j, i;
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
}

static void fill_bi() {
	uint8_t rep, j, i;
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
}

static void fill_s_bi() {
	uint8_t rep, i;
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
}

int main (void) {
	uint8_t i;
	sys_init();
    while(1) {
		fast_bi();
		i = 2;
		while(i --) {
			ff_s_l();
			ff_s_r();
		}
		fast_s_bi();
		ff_s_l();
		fill_bi();
		ff_s_r();
		i = 2;
		while(i --) {
			fill_s_bi();
			ff_s_l();
			fill_s_bi();
			ff_s_r();
		}
	}
}
