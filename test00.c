#define F_CPU 128000UL
//http://www.engbedded.com/fusecalc/
//128Khz
//avrdude -c USBASP -p t2313 -U lfuse:w:0xe6:m

//default (8Mhz/8):
//avrdude -c USBASP -p t2313 -U lfuse:w:0x64:m

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

static void flash(uint8_t time) {
	uint8_t i;
	while(time--) {
		for(i = 0; i < 7; i ++) {
			PORTB = b[i];
			PORTD = ~(0b1 << i);
			_delay_ms(1);
			PORTD = 0b1111111;
		}
	}
}

int main (void)
{
	uint8_t i, j;
	PORTB = 0x0;
    DDRB = 0xff;
    DDRD = 0b1111111;
	PORTD = 0b1111111;
	OCR0A  = 0x01;
	TCCR0A = 0x02;
	TIFR |= 0x01;
	TIMSK = 0x01;
	TCCR0B = 0x01;
	sei();
    while(1) 
	{
		for(j = 0; j < 7; j ++) {
			for(i = 0; i < 8; i ++) {
				b[j] = 1 << i;
				__builtin_avr_delay_cycles(50);
				b[j] = 0;
			}
		}
	}
}
