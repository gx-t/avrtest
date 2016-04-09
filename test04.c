#define F_CPU 1000000UL
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
	//disable interrupts
	cli();

	PORTD	= DAT_0 | CLK_0;
	DDRD	= DAT_OUT;

	//Baud rate 57600
	UBRRL = 0;
	UBRRH = 0;

	//Enable UART
	UCSRB = (1 << RXEN) | (1 << TXEN);

	//8 data bits, 1 stop bit
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0);
	
	//timer count to
	OCR0A  = 0xff;
	//timer CTC mode
	TCCR0A = 0x02;
	//clear interrupt flag
	TIFR |= 0x01;
	//TC0 compare match A interrupt enable
	TIMSK = 0x01;
	//clock source CLK/1024
	TCCR0B = 0x05;

	//enable interrupts
	sei();
}

///////////////////////////////////////////////////////////////////////////////
static uint8_t uart_rx()
{
	while(!(UCSRA & (1 << RXC)));

	return UDR;
}

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
	uart_tx('0');
	uart_tx('x');

	uart_tx(xx[(val >> 4) & 0b1111]);
	uart_tx(xx[(val >> 0) & 0b1111]);
	uart_tx(' ');
}

static void p_uint16(uint16_t val) {
	static const xx[] = {
		'0', '1', '2', '3', '4', '5', '6', '7',
		'8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
	};
	uart_tx('0');
	uart_tx('x');

	uart_tx(xx[(val >> 12) & 0b1111]);
	uart_tx(xx[(val >> 8) & 0b1111]);
	uart_tx(xx[(val >> 4) & 0b1111]);
	uart_tx(xx[(val >> 0) & 0b1111]);
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

static void p_str(const char* pp) {
	while(*pp) {
		uart_tx(*pp++);
	}
}

static void p_line(const char* pp) {
	while(*pp) {
		uart_tx(*pp++);
	}
	uart_tx('\r');
	uart_tx('\n');
}

static char* read_line() {
	static char buff[32];
	char* pp = buff;
	char* end = pp + 31;
	uint8_t sz = sizeof(buff) - 1;
	for(; '\r' != (*pp = uart_rx()) && pp != end; pp ++) {
		uart_tx(*pp);
	}
	p_str("\r\n");
	*pp = 0;
	return buff;
}

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

static char* fixed_02_to_ascii_2(int x) {
	uint8_t sign = x < 0;
    static char buff[8];
    char* pp = &buff[7];
	*pp = 0;
	if(sign) {
		x = -x;
	}
	*--pp = '0' + x % 10;
	x /= 10;
	*--pp = '0' + x % 10;
	*--pp = '.';
	x /= 10;
	*--pp = '0' + x % 10;
	x /= 10;
	*--pp = '0' + x % 10;
	if(sign) {
		*--pp = '-';
	}
    return pp;
}

static char* fixed_02_to_ascii_1(int x) {
    static char buff[16];
    char* pp = &buff[15];
	*pp-- = 0;
	*pp-- = '0' + x % 10;
	x /= 10;
	*pp-- = '0' + x % 10;
	x /= 10;
	*pp -- = '.';
	*pp = '0' + x % 10;
	x /= 10;
    while(x) {
		*--pp = '0' + x % 10;
		x /= 10;
    }
    return pp;
}

static char* fixed_02_to_ascii(int x) {
    static char buff[16];
    char* pp = &buff[15];
    pp[0]  = 0;
    pp[-3] = '.';
    while(x) {
        pp --;
        if(pp != &buff[12]) {
            *pp = '0' + x % 10;
            x /= 10;
        }
    }
    while(pp > &buff[11]) {
        --pp;
        if(pp != &buff[12]) {
            *pp = '0';
        }
    }
    return pp;
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

static uint16_t ut = 0, uh = 0;
static uint8_t tt = 0;

ISR(TIMER0_COMPA_vect) {
	switch(tt++) {
		case 0:
			sht1x_start(0b00000011);
			break;
		case 4:
			ut = sht1x_read();
			sht1x_start(0b00000101);
			break;
		case 6:
			uh = sht1x_read();
			break;
		case 32:
			tt = 0;
			break;
	}
}

static void sht1x_print() {
    const float c1=-2.0468;
    const float c2=+0.0367;
    const float c3=-0.0000015955;
    const float t1=+0.01;
    const float t2=+0.00008;
	float ft = ut;
	float fh = uh;

	ft = ft * 0.01 - 40.1;
	p_str("Temp.\t= ");
	p_line(fixed_02_to_ascii_2(ft * 100));
	fh = (ft - 25) * (t1 + t2 * fh) + c3 * fh * fh + c2 * fh + c1;
	fh *= 100;
	p_str("Hum.\t= ");
	p_line(fixed_02_to_ascii_2(fh));
	p_str("\r\n");
}

int main()
{
	sys_init();
	
	while(1) {
		p_line("Cmd:\ns - sht1x read temp./hum.\r\n");
		uart_tx('>');

		switch(uart_rx()) {
			case 's':
				p_line("sht1x temp./hum.:");
				sht1x_print();
				break;
			default:
				p_line("Invalid command.");
				break;
		}
	}
	return 0;
}

