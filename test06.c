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
*/

static void rtc_init(void)
{  
	TCCR2A = 0x00;  //overflow
	TCCR2B = 0x03;  //5 gives 1 sec. prescale 
	TIMSK2 = 0x01;  //enable timer2A overflow interrupt
	ASSR  = 0x20;   //enable asynchronous mode
}

///////////////////////////////////////////////////////////////////////////////
#define USART_BAUD 38400UL
#define USART_UBBR_VALUE ((F_CPU / (USART_BAUD << 4)) - 1)

static void uart_init() {
	UBRR0H = (uint8_t) (USART_UBBR_VALUE >> 8);
	UBRR0L = (uint8_t) USART_UBBR_VALUE;

	UCSR0A = 0;
	

	//Enable UART
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

	//8 data bits, 1 stop bit
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static uint8_t uart_rx()
{
	while(!(UCSR0A & (1 << RXC0)));

	return UDR0;
}

static void uart_tx(uint8_t data)
{
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

static void p_line(const char* pp) {
	while(*pp) {
		uart_tx(*pp++);
	}
	uart_tx('\r');
	uart_tx('\n');
}

///////////////////////////////////////////////////////////////////////////////
//Interrupts
ISR(USART_RX_vect)
{
}

static char val[] = {'0', '0', '0', '0', '0', '0', '0', 0};

static void reset_val()
{
    char *p = val;
    while(*p) {
        *p ++ = '0';
    }
    TCNT2 = 0; //zero RTC counter
}

static void update_val()
{
    char* p = val + sizeof(val) - 2;
    while(p >= val) {
        if('9' >= ++(*p))
            break;
        *p -- = '0';
    }
}

ISR(TIMER2_OVF_vect)
{
}

static void p_val() {
    p_line(val);
}

static void sys_init() {
	cli();
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
	uart_init();
	rtc_init();
    reset_val();
	sei();
}

int main(void) {
	sys_init();
	while(1) {
		sleep_cpu();
		if(UCSR0A & (1 << RXC0)) {
			uint8_t ch = UDR0;
			if(ch == '\r') {
                p_line("Value reset");
                reset_val();
			}
            else {
                p_line("Press <ENTER> to reset the value");
            }
		}
		else {
            update_val();
			p_val();
		}
	} 

	return 1;
}

