#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>


//PD0 RX
//PD1 TX

#define DAT_IN			0b0100
#define DAT_OUT			0b1100

#define DAT_0			0
#define DAT_1			0b1000
#define CLK_0			0
#define CLK_1			0b0100

static void sys_init() {
	DDRD = 0b1100;

	//Baud rate 57600 for 1 MHZ, 460800 for 8MHZ
	UBRRL = 0;
	UBRRH = 0;

	//8 data bits, 1 stop bit
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0);
	
	cli();
	//sleep enable, power down mode
	MCUCR |= (1 << SE) | (1 << SM0);
	wdt_reset();
	//enable WDT interrupt mode, 1 sec period
	WDTCSR = (1 << WDIE) | (1 << WDCE) | (1 << WDP2) | (1 << WDP1); 
	sei();
}

//Watchdog timeout ISR
ISR(WDT_OVERFLOW_vect)
{
}

///////////////////////////////////////////////////////////////////////////////
static void uart_tx(uint8_t data)
{
	while (!(UCSRA & (1 << UDRE)));
	UDR = data;
}

///////////////////////////////////////////////////////////////////////////////
static void p_uint8(uint8_t val) {
	static const xx[] = {
		'0', '1', '2', '3', '4', '5', '6', '7',
		'8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
	};

	uart_tx(xx[(val >> 4) & 0b1111]);
	uart_tx(xx[(val >> 0) & 0b1111]);
	uart_tx(' ');
}

static void p_uint16(uint16_t val) {
	static const xx[] = {
		'0', '1', '2', '3', '4', '5', '6', '7',
		'8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
	};

	uart_tx(xx[(val >> 12) & 0b1111]);
	uart_tx(xx[(val >> 8) & 0b1111]);
	uart_tx(xx[(val >> 4) & 0b1111]);
	uart_tx(xx[(val >> 0) & 0b1111]);
	uart_tx(' ');
}

static void p_uint32(uint32_t val) {
	static const xx[] = {
		'0', '1', '2', '3', '4', '5', '6', '7',
		'8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
	};
	uart_tx('0');
	uart_tx('x');

	uart_tx(xx[(val >> 24) & 0b1111]);
	uart_tx(xx[(val >> 24) & 0b1111]);
	uart_tx(xx[(val >> 20) & 0b1111]);
	uart_tx(xx[(val >> 16) & 0b1111]);
	uart_tx(xx[(val >> 12) & 0b1111]);
	uart_tx(xx[(val >> 8) & 0b1111]);
	uart_tx(xx[(val >> 4) & 0b1111]);
	uart_tx(xx[(val >> 0) & 0b1111]);
	uart_tx(' ');
}

///////////////////////////////////////////////////////////////////////////////
//sht1x
static void sht1x_write_byte(uint8_t dat) {
	uint8_t i = 8;
	while(i --) {
		uint8_t bit = ((dat >> i) & 1) << 3;
		PORTD = bit | CLK_0;
		PORTD = bit | CLK_1; 
	}
	PORTD = DAT_0 | CLK_0;
}

static uint8_t sht1x_read_byte() {
	uint8_t data = 0;
	uint8_t i = 8;
	DDRD = DAT_IN;
	while(i --) {
		data <<= 1;
		PORTD = DAT_1 | CLK_1;
		data |= !!(PIND & 0b1000);
		PORTD = DAT_1 | CLK_0;
	}
	return data;
}

static void sht1x_ack() {
	DDRD = DAT_OUT;
	PORTD = DAT_0 | CLK_1;
	PORTD = DAT_0 | CLK_0;
}

static void sht1x_nack() {
	DDRD = DAT_OUT;
	PORTD = DAT_1 | CLK_1;
	PORTD = DAT_1 | CLK_0;
}

static void sht1x_start(uint8_t addr) {
	DDRD = DAT_OUT;

	//init sensor
	PORTD = CLK_0 | DAT_1;
	PORTD = CLK_1 | DAT_1;
	PORTD = CLK_1 | DAT_0;
	PORTD = CLK_0 | DAT_0;
	PORTD = CLK_1 | DAT_0;
	PORTD = CLK_1 | DAT_1;
	PORTD = CLK_0 | DAT_1;
	
	sht1x_write_byte(addr);

	DDRD = DAT_IN;
	PORTD = DAT_1 | CLK_1;
//	p_str((PIND & DAT_1) ? "nack1 " : "ack1 ");
	PORTD = DAT_1 | CLK_0;
}

static uint16_t sht1x_read() {
	uint16_t data;
	data = sht1x_read_byte(1);
	sht1x_ack();
	data <<= 8;
	data |= sht1x_read_byte(0);
	sht1x_nack();
	return data;
}

static uint16_t sht1x_ut = 0, sht1x_uh = 0;

//func, inst, cnt, val

static void sht1x_print() {
	static uint16_t sht1x_tt = 0;

	//function
	p_uint8(0);
	//instance
	p_uint8(0);
	//counter
	p_uint16(sht1x_tt);
	//temperature
	p_uint16(sht1x_ut);
	//humidity
	p_uint16(sht1x_uh);
	uart_tx('\n');
	sht1x_tt ++;
}

int main()
{
	sys_init();

	uint8_t tt = 0;

	while(1) {
		switch(tt++) {
			case 0:
				sht1x_start(0b00000011);
				break;
			case 1:
				sht1x_ut = sht1x_read();
				sht1x_start(0b00000101);
				break;
			case 3:
				sht1x_uh = sht1x_read();
				UCSRB |= (1 << TXEN);
				sht1x_print();
				_delay_us(30);
				UCSRB &= ~(1 << TXEN);
				tt = 0;
				break;
		}
		sleep_cpu();
	}
	return 0;
}

