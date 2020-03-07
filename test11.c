#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
//#include <stdio.h>

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

//#define USART_BAUD 38400UL
//#define USART_UBBR_VALUE ((F_CPU / (USART_BAUD << 4)) - 1)
//
//static void uart_init() {
//    UBRR0H = (uint8_t) (USART_UBBR_VALUE >> 8);
//    UBRR0L = (uint8_t) USART_UBBR_VALUE;
//    UCSR0A = 0;
//    //Enable UART
//    UCSR0B = (1 << TXEN0);
//    //8 data bits, 1 stop bit
//    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
//}
//
//static int uart_print_char(char ch, FILE* pf)
//{
//    while (!(UCSR0A & (1 << UDRE0)));
//    UDR0 = ch;
//    return 0;
//}
//
//static FILE uart_str = {
//    .put = uart_print_char,
//    .flags = _FDEV_SETUP_RW
//};
//
static void reset_enable_pullup()
{
    PORTC |= _BV(PC6);
}

static void led_enable_pin()
{
    DDRD |= _BV(PD6) | _BV(PD5) | _BV(PD3);
    DDRB |= _BV(PB3);
}

//static void rtc_init()
//{  
//    TCCR2B = 0;  //stop Timer 2
//    TIMSK2 = 0; // disable Timer 2 interrupts
//    ASSR = (1<<AS2); // select asynchronous operation of Timer2
//    TCNT2 = 0; // clear Timer 2 counter
//    TCCR2A = 0; //normal count up mode, no port output
//    TCCR2B = (1<<CS22) | (1<<CS20); // select prescaler 128 => 1 sec between each overflow
//
//    while (ASSR & ((1<<TCN2UB)|(1<<TCR2BUB))); // wait for TCN2UB and TCR2BUB to clear
//
//    TIFR2 = 0xFF; // clear all interrupt flags
//    TIMSK2 = (1<<TOIE2); // enable Timer2 overflow interrupt
//}

static void wdt_init()
{
    wdt_reset();
    WDTCSR = 0b00011000;
    WDTCSR = 0b01001111;
}

ISR(WDT_vect)
{
//    fprintf(&uart_str, "interrupt\r\n");
}

static void wdt_sleep_2s()
{
    WDTCSR = 0b00011000;
    WDTCSR = 0b01001111;
    sleep_cpu();
}

static void wdt_sleep_64ms()
{
    WDTCSR = 0b00011000;
    WDTCSR = 0b01000011;
    sleep_cpu();
}

static void sys_init()
{
    cli();
    reset_enable_pullup();
    led_enable_pin();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    wdt_init();
//    uart_init();
    sei();
}

static void led_0_on()  { PORTD |= _BV(PD6); }
static void led_0_off() { PORTD &= ~_BV(PD6); }

static void led_1_on()  { PORTB |= _BV(PB3); }
static void led_1_off() { PORTB &= ~_BV(PB3); }

static void led_2_on()  { PORTD |= _BV(PD5); }
static void led_2_off() { PORTD &= ~_BV(PD5); }

static void led_3_on()  { PORTD |= _BV(PD3); }
static void led_3_off() { PORTD &= ~_BV(PD3); }

int main(void) {
    sys_init();
//    fprintf(&uart_str, "main\r\n");
    while(1) {
        led_0_on();
        _delay_ms(2);
        led_0_off();
        wdt_sleep_64ms();
        led_1_on();
        _delay_ms(2);
        led_1_off();
        wdt_sleep_64ms();
        led_2_on();
        _delay_ms(2);
        led_2_off();
        wdt_sleep_64ms();
        led_3_on();
        _delay_ms(2);
        led_3_off();
        wdt_sleep_64ms();
        led_0_on();
        _delay_ms(2);
        led_0_off();
        wdt_sleep_2s();
//        fprintf(&uart_str, "wake\r\n");
    }
    return 0;
}

