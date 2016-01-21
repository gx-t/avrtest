#define F_CPU 128000UL
//http://www.engbedded.com/fusecalc/
//128Khz
//avrdude -c USBASP -p t2313 -U lfuse:w:0xe6:m

//default (8Mhz/8):
//avrdude -c USBASP -p t2313 -U lfuse:w:0x64:m

#include <avr/io.h>
#include <util/delay.h>

static void flash(uint8_t leds1, uint8_t leds2) {
	uint8_t cnt = 0xff;
	while(cnt--) {
		PORTD = 0b01;
		PORTB = leds2;
		_delay_us(100);
		PORTB = 0;
		PORTD = 0b10;
		PORTB = leds1;
		_delay_us(100);
		PORTB = 0;
		PORTD = 0b11;
	}
	PORTB = 0x0;
}

static void flash1(uint8_t leds1, uint8_t leds2) {
	uint8_t cnt = 0x88;
	while(cnt--) {
		PORTD = 0b01;
		PORTB = leds2;
		_delay_us(100);
		PORTB = 0;
		PORTD = 0b10;
		PORTB = leds1;
		_delay_us(100);
		PORTB = 0;
		PORTD = 0b11;
	}
	PORTB = 0x0;
}

static void move_left_1() {
	flash(0b00000000, 0b00000001);
	flash(0b00000000, 0b00000010);
	flash(0b00000000, 0b00000100);
	flash(0b00000000, 0b00001000);
	flash(0b00000000, 0b00010000);
	flash(0b00000000, 0b00100000);
	flash(0b00000000, 0b01000000);
	flash(0b00000000, 0b10000000);
	
	flash(0b00000001, 0b00000000);
	flash(0b00000010, 0b00000000);
	flash(0b00000100, 0b00000000);
	flash(0b00001000, 0b00000000);
	flash(0b00010000, 0b00000000);
	flash(0b00100000, 0b00000000);
	flash(0b01000000, 0b00000000);
	flash(0b10000000, 0b00000000);
}

static void move_right_1() {
	flash(0b10000000, 0b00000000);
	flash(0b01000000, 0b00000000);
	flash(0b00100000, 0b00000000);
	flash(0b00010000, 0b00000000);
	flash(0b00001000, 0b00000000);
	flash(0b00000100, 0b00000000);
	flash(0b00000010, 0b00000000);
	flash(0b00000001, 0b00000000);
	
	flash(0b00000000, 0b10000000);
	flash(0b00000000, 0b01000000);
	flash(0b00000000, 0b00100000);
	flash(0b00000000, 0b00010000);
	flash(0b00000000, 0b00001000);
	flash(0b00000000, 0b00000100);
	flash(0b00000000, 0b00000010);
	flash(0b00000000, 0b00000001);
}
static void narrow_extend_2() {
	flash(0b10000000, 0b00000001);
	flash(0b01000000, 0b00000010);
	flash(0b00100000, 0b00000100);
	flash(0b00010000, 0b00001000);
	flash(0b00001000, 0b00010000);
	flash(0b00000100, 0b00100000);
	flash(0b00000010, 0b01000000);
	flash(0b00000001, 0b10000000);

	flash(0b00000001, 0b10000000);
	flash(0b00000010, 0b01000000);
	flash(0b00000100, 0b00100000);
	flash(0b00001000, 0b00010000);
	flash(0b00010000, 0b00001000);
	flash(0b00100000, 0b00000100);
	flash(0b01000000, 0b00000010);
	flash(0b10000000, 0b00000001);
}

static void fill_left_16() {
	flash(0b00000000, 0b00000001);
	flash(0b00000000, 0b00000011);
	flash(0b00000000, 0b00000111);
	flash(0b00000000, 0b00001111);
	flash(0b00000000, 0b00011111);
	flash(0b00000000, 0b00111111);
	flash(0b00000000, 0b01111111);
	flash(0b00000001, 0b11111111);
	flash(0b00000011, 0b11111111);
	flash(0b00000111, 0b11111111);
	flash(0b00001111, 0b11111111);
	flash(0b00011111, 0b11111111);
	flash(0b00111111, 0b11111111);
	flash(0b01111111, 0b11111111);
	flash(0b11111111, 0b11111111);
	flash(0b01111111, 0b11111111);
	flash(0b00111111, 0b11111111);
	flash(0b00011111, 0b11111111);
	flash(0b00001111, 0b11111111);
	flash(0b00000111, 0b11111111);
	flash(0b00000011, 0b11111111);
	flash(0b00000001, 0b11111111);
	flash(0b00000000, 0b01111111);
	flash(0b00000000, 0b00111111);
	flash(0b00000000, 0b00011111);
	flash(0b00000000, 0b00001111);
	flash(0b00000000, 0b00000111);
	flash(0b00000000, 0b00000011);
	flash(0b00000000, 0b00000001);
}

static void fill_right_16() {
	flash(0b10000000, 0b00000000);
	flash(0b11000000, 0b00000000);
	flash(0b11100000, 0b00000000);
	flash(0b11110000, 0b00000000);
	flash(0b11111000, 0b00000000);
	flash(0b11111100, 0b00000000);
	flash(0b11111110, 0b00000000);
	flash(0b11111111, 0b00000000);
	flash(0b11111111, 0b10000000);
	flash(0b11111111, 0b11000000);
	flash(0b11111111, 0b11100000);
	flash(0b11111111, 0b11110000);
	flash(0b11111111, 0b11111000);
	flash(0b11111111, 0b11111100);
	flash(0b11111111, 0b11111110);
	flash(0b11111111, 0b11111111);
	flash(0b11111111, 0b11111110);
	flash(0b11111111, 0b11111100);
	flash(0b11111111, 0b11111000);
	flash(0b11111111, 0b11110000);
	flash(0b11111111, 0b11100000);
	flash(0b11111111, 0b11000000);
	flash(0b11111111, 0b10000000);
	flash(0b11111111, 0b00000000);
	flash(0b11111110, 0b00000000);
	flash(0b11111100, 0b00000000);
	flash(0b11111000, 0b00000000);
	flash(0b11110000, 0b00000000);
	flash(0b11100000, 0b00000000);
	flash(0b11000000, 0b00000000);
	flash(0b10000000, 0b00000000);
}

static void move_left_3() {
	flash1(0b00000000, 0b00000001);
	flash1(0b00000000, 0b00000011);
	flash1(0b00000000, 0b00000111);
	flash1(0b00000000, 0b00001110);
	flash1(0b00000000, 0b00011100);
	flash1(0b00000000, 0b00111000);
	flash1(0b00000000, 0b01110000);
	flash1(0b00000000, 0b11100000);
	flash1(0b00000001, 0b11000000);
	flash1(0b00000011, 0b10000000);
	flash1(0b00000111, 0b00000000);
	flash1(0b00001110, 0b00000000);
	flash1(0b00011100, 0b00000000);
	flash1(0b00111000, 0b00000000);
	flash1(0b01110000, 0b00000000);
	flash1(0b11100000, 0b00000000);
	flash1(0b11000000, 0b00000000);
	flash1(0b10000000, 0b00000000);
}

static void move_right_3() {
	flash1(0b10000000, 0b00000000);
	flash1(0b11000000, 0b00000000);
	flash1(0b11100000, 0b00000000);
	flash1(0b01110000, 0b00000000);
	flash1(0b00111000, 0b00000000);
	flash1(0b00011100, 0b00000000);
	flash1(0b00001110, 0b00000000);
	flash1(0b00000111, 0b00000000);
	flash1(0b00000011, 0b10000000);
	flash1(0b00000001, 0b11000000);
	flash1(0b00000000, 0b11100000);
	flash1(0b00000000, 0b01110000);
	flash1(0b00000000, 0b00111000);
	flash1(0b00000000, 0b00011100);
	flash1(0b00000000, 0b00001110);
	flash1(0b00000000, 0b00000111);
	flash1(0b00000000, 0b00000011);
	flash1(0b00000000, 0b00000001);
}

int main (void)
{
	uint8_t cnt;
//    DDRB |= _BV(DDB0); 
	PORTB = 0x0;
    DDRB = 0xff;
    DDRD = 0b11;
	PORTD = 0b11;
    while(1) 
    {
//        PORTB ^= _BV(PB0);
//		delay();
//        PORTB = _BV(PB0);
		cnt = 3;
		while(cnt--) {
			move_left_1();
		}

		cnt = 3;
		while(cnt--) {
			move_right_1();
		}
		cnt = 3;
		while(cnt--) {
			narrow_extend_2();
		}

		cnt = 3;
		while(cnt--) {
			fill_left_16();
		}

		cnt = 3;
		while(cnt--) {
			fill_right_16();
		}

		cnt = 5;
		while(cnt--) {
			move_left_3();
		}

		cnt = 5;
		while(cnt--) {
			move_right_3();
		}
		//        PORTB ^= _BV(PB0);
		_delay_ms(1000);
	}
}
