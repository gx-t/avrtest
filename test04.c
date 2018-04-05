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
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	//Enable UART and interrupt on receive
	UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
	sei();
}

///////////////////////////////////////////////////////////////////////////////
static void uart_tx(uint8_t data)
{
	while (!(UCSRA & (1 << UDRE)));
	UDR = data;
}

//static uint8_t uart_rx()
//{
//	while(!(UCSRA & (1 << RXC)));
//
//	return UDR;
//}

ISR(USART_RX_vect)
{
}

///////////////////////////////////////////////////////////////////////////////
static void p_nibble(uint8_t val) {
	static const uint8_t xx[] = {
		'0', '1', '2', '3', '4', '5', '6', '7',
		'8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
	};
	val &= 0b1111;
	uart_tx(xx[val]);
}

static void p_uint8(uint8_t val) {
	p_nibble(val >> 4);
	p_nibble(val >> 0);
}

static void p_uint16(uint16_t val) {
	p_uint8(val >> 8);
	p_uint8(val >> 0);
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
		data |= !!(PIND & DAT_1);
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

static void sht1x_wait_result() {
	while(PIND & DAT_1);
}

static uint16_t sht1x_ut = 0, sht1x_uh = 0;

//func, inst, cnt, val

static void sht1x_print() {
	static uint16_t sht1x_tt = 0;
	//start
	uart_tx('$');
	uart_tx(' ');
	//function
	p_uint8(0);
	uart_tx(' ');
	//instance
	p_uint8(0);
	uart_tx(' ');
	//counter
	p_uint16(sht1x_tt);
	uart_tx(' ');
	//temperature
	p_uint16(sht1x_ut);
	uart_tx(' ');
	//humidity
	p_uint16(sht1x_uh);
	uart_tx('\r');
	uart_tx('\n');
	sht1x_tt ++;
}

int main()
{
	sys_init();

	while(1) {
		sleep_cpu();
		if(UCSRA & (1 << RXC)) {
			uint8_t ch = UDR;
			if(ch == 'r') {
				sht1x_start(0b00000011);
				sht1x_wait_result();
				sht1x_ut = sht1x_read();
				sht1x_start(0b00000101);
				sht1x_wait_result();
				sht1x_uh = sht1x_read();
				sht1x_print();
			}
		}
	}
	return 0;
}

