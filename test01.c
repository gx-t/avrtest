#define F_CPU 128000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

/*
	"Digital Candels" with coin payment
	
	PD0 -LED0+ PB2
	PD1 -LED1+ PB2
	PD2 -LED2+ PB2
	PD3 -LED3+ PB2
	PD4 -LED4+ PB2
	PD5 -LED5+ PB2
	PD6 -LED6+ PB2

	PB0 <coin> GND
	VCC 3V GND
*/

#define CANDLE_ON_TIME	0xFFFF;

//Candles on remaining times. On POR short, demonstration on.
static uint16_t tm[] = {0x100, 0x120, 0x140, 0x160, 0x180, 0x1A0, 0x1B0};
static uint8_t debounce = 0;

static void sys_init() {
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
	GIMSK |= (1 << PCIE);  //pin change interrupt enable
	PCMSK |= (1 << PCINT0);  //pin change interrupt 0 (PB0)
	MCUCR |= (1 << SE) | (1 << SM0);  //sleep enable, power down mode
	asm("sei");  //enable interrupts
}

ISR(PCINT_vect) {
	if(debounce) {
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
			break;
		}
	}
}

static uint8_t oscilate() {
	static int8_t s1 = 0, s2 = 0, s3 = 0, s4 = 0;
	static int8_t c1 = 100, c2 = 100, c3 = 100, c4 = 100;
	s1 += c1 / 10;
	c1 -= s1 / 10;
	s2 += c2 / 12;
	c2 -= s2 / 12;
	s3 += c3 / 20;
	c3 -= s3 / 20;
	s4 += c4 / 21;
	c4 -= s4 / 21;
	return ((s1 + s2 + s2 + s4) >> 2) + 127;
}

static void check_candle_times() {
	uint8_t i = sizeof(tm) / sizeof(tm[0]);
	uint8_t cnt = 0;
	while(i--) {
		if(tm[i]) {
			tm[i]--;
			cnt ++;
			PORTD &= ~(1 << i);
		} else {
			PORTD |= (1 << i);
		}
	}
	if(!cnt) {
		asm("sleep");
	}
}

int main()
{
	sys_init();
	//program main loop
	while(1) {
		OCR0A = oscilate(); //PWM duty
		check_candle_times();
		if(debounce) {
			debounce --;
		}
//		OCR1A = val;
	}
	return 0;
}

