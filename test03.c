#define F_CPU 128000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>

/*
Battery powered emergency light
Fast 16 short flashes when battery is just connected and when switching on/off
Short single flashes when dark
Permannent 2 min light on hand gesture in front of flashing LEDs
Press button until short flashes begin and release to switch on/off (2/3 short pulses)

PD0 +LED- GND
PD1 +LED- GND
PD6 5M VCC
PD6 Rf GND
PB5 _/_ GND
VCC 0.68uF GND

LEDs - white leds with concentrators
Rf - photoresistor (2MOhm at switch point)
VCC - 4Vx4AH lead-acid (low-high temperature)
Brown-out detector 2.7V
Consumes < 10uA when off or under ambient light (not flashing)
*/

static uint8_t run = 1;

static void sys_init() {
	DDRD	= 0b00000011;
	PORTD	= 0b00100000;
	asm("cli");
	//sleep enable, power down mode
	MCUCR |= (1 << SE) | (1 << SM0);
	wdt_reset();
	//enable WDT interrupt mode, 1 sec period
	WDTCSR = (1 << WDIE) | (1 << WDCE) | (1 << WDP2) | (1 << WDP1); 
	asm("sei");
}

//Watchdog timeout ISR
ISR(WDT_OVERFLOW_vect)
{
}

static void flash_fast(uint8_t count) {
	while(count --) {
		PORTD = 0b00100011;
		_delay_ms(50);
		PORTD = 0b00100000;
		_delay_ms(50);
	}
}

static void process_button_down() {
	if(PIND & 0b00100000) {
		return;
	}
	while(!(PIND & 0b00100000)) {
		PORTD = 0b00100011;
		_delay_ms(50);
		PORTD = 0b00100000;
		_delay_ms(50);
	}
	run = !run;
	_delay_ms(1000);
	flash_fast(run ? 2:5);
}

static void long_wait() {
	uint8_t i = 120;
	while(i-- && run) {
		process_button_down();
		sleep_cpu();
	}
}

int main()
{
	sys_init();
	flash_fast(16);
	while(1) {
		process_button_down();
		sleep_cpu();
		wdt_reset();
		if(!(PIND & 0b01000000) || !run) {
			// ambient light
			continue;
		}
		PORTD = 0b00100011;
		_delay_ms(50);
		if(!(PIND & 0b01000000)) {
			// reflected light
			long_wait();
		}
		PORTD = 0b00100000;
	}
	return 0;
}

