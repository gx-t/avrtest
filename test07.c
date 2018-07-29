#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define F_CPU 8000000UL
#include <util/delay.h>

/*
ATMEGA 328P
    RTC: 9-10 32768 Hz QZ 
    9 -- 32768HZ -- 10
    Serial in/out
    LoRa:
        PB0 -- RST
        PB1 -- DIO0 (RX interrupt)
        PB2 -- NSS
        PB3 -- MOSI
        PB4 -- MISO
        PB5 -- SCK

        !! PB2 -- 100K -- VCC (disable loRa during ASP programming)
*/

ISR(TIMER2_OVF_vect)
{
}

static void rtc_init(void)
{  
    TCCR2A = 0x00;  //overflow
    TCCR2B = 0x05;  //5 gives 1 sec. prescale 
    TIMSK2 = 0x01;  //enable timer2A overflow interrupt
    ASSR  = 0x20;   //enable asynchronous mode
}

///////////////////////////////////////////////////////////////////////////////
static void spi_init()
{
    // CLK, MISO, MOSI, NSS, RX-INT, RST
    DDRB = 0b101101;
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPI2X); //enable SPI-master, clock/2 speed
    PORTB |= (1 << PB2);
}

//static void spi_write_byte(uint8_t data)
//{
//    SPDR = data;
//    while(!(SPSR & (1 << SPIF))); //wait for write end
//}

static void lora_reset()
{
    _delay_us(100);
    PORTB |= (1 << PB0);
    _delay_ms(5);
}

static uint8_t lora_read_reg(uint8_t reg)
{
    PORTB &= ~(1 << PB2);
    SPDR = reg;
    while(!(SPSR & (1 << SPIF)));
    SPDR = 0;
    while(!(SPSR & (1 << SPIF)));
    PORTB |= (1 << PB2);
    return SPDR;
}

///////////////////////////////////////////////////////////////////////////////
#define USART_BAUD 38400UL
#define USART_UBBR_VALUE ((F_CPU / (USART_BAUD << 4)) - 1)

ISR(USART_RX_vect)
{
}

static void uart_init()
{
	UBRR0H = (uint8_t) (USART_UBBR_VALUE >> 8);
	UBRR0L = (uint8_t) USART_UBBR_VALUE;

    UCSR0A = 0;


    //Enable UART
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

    //8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_tx(uint8_t data)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

//static void p_line(const char* pp)
//{
//	while(*pp) {
//		uart_tx(*pp++);
//	}
//	uart_tx('\r');
//	uart_tx('\n');
//}

static void p_hex_value(const char* name, uint8_t val)
{
    static const uint8_t hex_chars[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
	while(*name) {
		uart_tx(*name++);
	}
	uart_tx('=');
    uart_tx(hex_chars[(val & 0xF0) >> 4]);
    uart_tx(hex_chars[(val & 0x0F)]);
	uart_tx('\r');
	uart_tx('\n');
}


static void sys_init()
{
	cli();
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
	uart_init();
	rtc_init();
    spi_init();
    lora_reset();
	sei();
}

int main(void)
{
	sys_init();
    lora_reset();
    p_hex_value("REG 0x42", lora_read_reg(0x42));
	while(1) {
		sleep_cpu();
		if(UCSR0A & (1 << RXC0)) {
			uint8_t ch = UDR0;
			if(ch == '\r') {
                uart_tx('\n');
            }
            else {
                uart_tx(ch);
            }
		}
		else {
            p_hex_value("REG 0x42", lora_read_reg(0x42));
        //    p_line("RTC");
		}
	} 
	return 1;
}

