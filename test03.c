#define F_CPU 128000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>

/*
Battery powered emergency light
Short flashes in dark
Permannent 2 min light on hand gesture in front of flashing LEDs
Press button until short flashes begin to switch on/off
Long press will switch on/off continuously

PD4 +LED- GND
PD5 +LED- GND
PD0 2M VCC
PD0 Rf GND
PB0 _/_ GND
VCC 0.68uF GND

LEDs - white leds with concentrator
Rf - photoresistor
VCC - 4Vx4Ah lead-acid
Brown-out detector 2.7V
*/

static uint8_t run = 1;

static void sys_init() {
	DDRB	= 0b00000000;
	DDRD	= 0b00110000;
	//pull-up on PB0
	PORTB	= 0b00000001;
	PORTD	= 0b00000000;
	asm("cli");
	MCUCR |= (1 << SE) | (1 << SM0);  //sleep enable, power down mode
	asm("wdr");
	WDTCSR = (1 << WDIE) | (1 << WDCE) | (1 << WDP2) | (1 << WDP1); 
	asm("sei");
}

//Watchdog timeout ISR
ISR(WDT_OVERFLOW_vect)
{
}

static void flash_fast_16() {
	uint8_t i = 16;
	while(i --) {
		PORTD = 0b00110000;
		_delay_ms(50);
		PORTD = 0b00000000;
		_delay_ms(50);
	}
}

static void test_button() {
	if(PINB & 0b00000001) {
		return;
	}
	//if btn pressed
	flash_fast_16();
	run = !run;
}

static void long_wait() {
	uint8_t i = 120;
	while(i-- && run) {
		test_button();
		asm("sleep");
	}
}

int main()
{
	sys_init();
	flash_fast_16();
	while(1) {
		test_button();
		asm("sleep");
		if(!(PIND & 1) || !run) {
			// ambient light
			continue;
		}
		PORTD = 0b00110000;
		_delay_ms(50);
		if(!(PIND & 1)) {
			// reflected light
			long_wait();
		}
		PORTD = 0b00000000;
	}
	return 0;
}

