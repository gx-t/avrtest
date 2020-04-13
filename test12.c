#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h>

#define F_CPU 8000000UL
#include <util/delay.h>

/*
    ATMEGA328P
    GND - 8, 22
    VCC - 7, 20
    RX - PD0
    TX - PD1
    5 (PD3) - 100K 23(ADC0)
    23 (ADC0) - 2.2uF GND
*/

#define USART_BAUD 38400UL
#define USART_UBBR_VALUE ((F_CPU / (USART_BAUD << 4)) - 1)

static void init_uart() {
    UBRR0 = USART_UBBR_VALUE;
    UCSR0A = 0;
    //Enable UART
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    //8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

ISR(USART_RX_vect)
{
}

static int uart_print_char(char ch, FILE* pf)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = ch;
    return 0;
}

static FILE uart_str = {
    .put = uart_print_char,
    .flags = _FDEV_SETUP_WRITE
};

static void enable_reset_pullup()
{
    PORTC = 0b01000000;
}

static void enable_output_pin()
{
    DDRD = 0b00001000;
}

static void init_system()
{
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    enable_reset_pullup();
    enable_output_pin();
    init_uart();
    sei();
}

static void show_usage()
{
    fprintf(&uart_str, "\r\n");
    fprintf(&uart_str, "Usage:\r\n");
    fprintf(&uart_str, "a: GPIO set\r\n");
    fprintf(&uart_str, "b: GPIO reset\r\n");
    fprintf(&uart_str, "c: ADC read\r\n");
    fprintf(&uart_str, "\r\n");
}

static void f0_set_gpio()
{
    fprintf(&uart_str, "%s\r\n", __func__);
}

static void f0_reset_gpio()
{
    fprintf(&uart_str, "%s\r\n", __func__);
}

static void f0_read_adc()
{
    fprintf(&uart_str, "%s\r\n", __func__);
}

static void f0_input()
{
    void (*proc_tbl[])() = {
        f0_set_gpio,
        f0_reset_gpio,
        f0_read_adc
    };

    uint8_t ch = UDR0;
    if(ch < 'a' || ch > 'c')
        return show_usage();

    ch -= 'a';
    proc_tbl[ch]();
}

int main(void)
{
    init_system();
    while(1) {
        sleep_cpu();
        if(!(UCSR0A & (1 << RXC0)))
            continue;
        f0_input();
    }
    return 0;
}

