#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h>

#define F_CPU 8000000UL
#include <util/delay.h>

/*
    Solar cell + ultracapacitor powered LED beacon

    ATMEGA328P
    7, 8 4F
    7 Schottky diode Solar +
    8 Solar -
    PB3, PD3, PD5, PD6   LED(n)+
    2.2uF 0805 7, 8
*/

#define USART_BAUD 38400UL
#define USART_UBBR_VALUE ((F_CPU / (USART_BAUD << 4)) - 1)

static void uart_init() {
    UBRR0H = (uint8_t) (USART_UBBR_VALUE >> 8);
    UBRR0L = (uint8_t) USART_UBBR_VALUE;
    UCSR0A = 0;
    //Enable UART
    UCSR0B = (1 << TXEN0);
    //8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static int uart_print_char(char ch, FILE* pf)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = ch;
    return 0;
}

static FILE uart_str = {
    .put = uart_print_char,
    .flags = _FDEV_SETUP_RW
};

//static void led_on()
//{
//    PORTD |= (_BV(PD6) | _BV(PD5) | _BV(PD3));
//    PORTB |= (_BV(PB3));
//    fprintf(&uart_str, "%s\r\n", __func__);
//}
//
//static void led_off()
//{
//    PORTD &= ~(_BV(PD6) | _BV(PD5) | _BV(PD3));
//    PORTB &= ~(_BV(PB3));
//    fprintf(&uart_str, "%s\r\n", __func__);
//}

static void reset_enable_pullup()
{
    PORTC |= _BV(PC6);
}

static void led_enable_pin()
{
    DDRD |= _BV(PD6) | _BV(PD5) | _BV(PD3);
    DDRB |= _BV(PB3);
}

static void rtc_init()
{  
    TCCR2A = 0x00;  //overflow
    TCCR2B = 0x05;  //5 gives 1 sec. prescale 
    TIMSK2 = 0x01;  //enable timer2A overflow interrupt
    ASSR  = 0x20;   //enable asynchronous mode
}

ISR(TIMER2_OVF_vect)
{
}

static void sys_init()
{
    cli();
    reset_enable_pullup();
    led_enable_pin();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    rtc_init();
    uart_init();
    sei();
}

int main(void) {
    printf("%s\r\n", __func__);
    sys_init();
    while(1) {
        PORTD |= _BV(PD6);
        fprintf(&uart_str, "LED-0 on\r\n");
        _delay_ms(5);
        PORTD &= ~_BV(PD6);
        fprintf(&uart_str, "LED-0 off\r\n");
        _delay_ms(50);
        PORTD |= _BV(PD5);
        fprintf(&uart_str, "LED-1 on\r\n");
        _delay_ms(5);
        PORTD &= ~_BV(PD5);
        fprintf(&uart_str, "LED-1 off\r\n");
        _delay_ms(50);
        PORTD |= _BV(PD3);
        fprintf(&uart_str, "LED-2 on\r\n");
        _delay_ms(5);
        PORTD &= ~_BV(PD3);
        fprintf(&uart_str, "LED-2 off\r\n");
        _delay_ms(50);
        PORTB |= _BV(PB3);
        fprintf(&uart_str, "LED-3 on\r\n");
        _delay_ms(5);
        PORTB &= ~_BV(PB3);
        fprintf(&uart_str, "LED-3 off\r\n");
        //        led_on();
        //        TCNT2 = 16;
        //        sleep_cpu();
        //        sleep_cpu();
//        _delay_ms(1000);
        sleep_cpu();
    }
    return 0;
}

