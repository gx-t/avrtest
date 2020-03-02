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
    TCCR2B = 0x05;  //5 gives 1 sec. prescale 
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

//static uint8_t uart_rx()
//{
//  while(!(UCSR0A & (1 << RXC0)));
//
//  return UDR0;
//}

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

struct {
    uint8_t s0 : 4;
    uint8_t s1 : 4;
    uint8_t m0 : 4;
    uint8_t m1 : 4;
    uint8_t h0 : 4;
    uint8_t h1 : 4;
    uint8_t d0 : 4;
    uint8_t d1 : 4;
} static tm = {0};

static void reset_time()
{
    tm.s0 = 0;
    tm.s1 = 0;
    tm.m0 = 0;
    tm.m1 = 0;
    tm.h0 = 0;
    tm.h1 = 0;
    tm.d0 = 0;
    tm.d1 = 0;
    TCNT2 = 0; //zero RTC counter
}

static void update_time()
{
    if(++tm.s0 == 10) {
        tm.s0 = 0;
        if(++tm.s1 == 6) {
            tm.s1 = 0;
            if(++tm.m0 == 10) {
                tm.m0 = 0;
                if(++tm.m1 == 6) {
                    tm.m1 = 0;
                    if(++tm.h0 == 10) {
                        tm.h0 = 0;
                        ++tm.h1;
                    }
                    else if(tm.h0 == 4 && tm.h1 == 2) {
                        tm.h0 = 0;
                        tm.h1 = 0;
                        if(++tm.d0 == 10) {
                            tm.d0 = 0;
                            if(++tm.d1 == 10) {
                                tm.d1 = 0;
                            }
                        }
                    }
                }
            }
        }
    }
}

ISR(TIMER2_OVF_vect)
{
}

static void p_time() {
    static const uint8_t num_arr[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
    };
    uart_tx(num_arr[tm.d1]);
    uart_tx(num_arr[tm.d0]);
    uart_tx(':');
    uart_tx(num_arr[tm.h1]);
    uart_tx(num_arr[tm.h0]);
    uart_tx(':');
    uart_tx(num_arr[tm.m1]);
    uart_tx(num_arr[tm.m0]);
    uart_tx(':');
    uart_tx(num_arr[tm.s1]);
    uart_tx(num_arr[tm.s0]);
    uart_tx('\r');
    uart_tx('\n');
}

static void sys_init() {
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    uart_init();
    rtc_init();
    sei();
}

int main(void) {
    sys_init();
    while(1) {
        sleep_cpu();
        if(UCSR0A & (1 << RXC0)) {
            uint8_t ch = UDR0;
            if(ch == '\r') {
                p_line("RTC reset");
                reset_time();
            }
            else {
                p_line("Press <ENTER> to reset RTC");
            }
        }
        else {
            update_time();
            p_time();
        }
    } 
    return 1;
}

