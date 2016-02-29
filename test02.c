#define F_CPU 128000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

static void sys_init() {
	DDRB	= 0b00001100; //PB2, PB3 - output
	PORTB	= 0b00000001; //pull-up on PB0
	DDRD	= 0b01100000; //PORTD 0-6 output
	PORTD	= 0b00000000; //PD5-6 low (LEDs on), pull-ups off
	TCCR0A	= (1 << COM0A1) | (1 << WGM00);  // phase correct PWM mode
	TCCR1A	= (1 << COM0A1) | (1 << WGM00);  // phase correct PWM mode
	OCR0A	= 0x00;                          // initial PWM pulse width
	OCR1A  	= 0x00;                          // initial PWM pulse width
	TCCR0B	= (1 << CS00);   // clock source = CLK/1, start PWM
	TCCR1B	= (1 << CS00);   // clock source = CLK/1, start PWM
	GIMSK	|= (1 << PCIE);  //pin change interrupt enable
	PCMSK	|= (1 << PCINT0);  //pin change interrupt 0 (PB0)
	MCUCR	|= (1 << SE) | (1 << SM0);  //sleep enable, power down mode
	asm("sei");  //enable interrupts
}

static uint8_t run = 0;

ISR(PCINT_vect) {
	if(PINB & 0b00000001) {
		run = !run;
	}
}

int main()
{
	static int8_t s1 = 0, s2 = 0, s3 = 0, s4 = 0;
	static int8_t c1 = 100, c2 = 100, c3 = 100, c4 = 100;
	sys_init();
	while(1) {
		s1 += c1 / 10;
		c1 -= s1 / 10;
		s2 += c2 / 12;
		c2 -= s2 / 12;
		s3 += c3 / 20;
		c3 -= s3 / 20;
		s4 += c4 / 21;
		c4 -= s4 / 21;
		OCR0A = ((s1 + s2 + s2 + s4) >> 2) + 127;
		OCR1A = ((c1 + c2 + c2 + c4) >> 2) + 127;
		if(!run) {
			asm("sleep");
		}
	}
	return 0;
}

