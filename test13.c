#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/twi.h>
#include <stdio.h>

#define F_CPU 8000000UL
#include <util/delay.h>

#define USART_BAUD 38400UL
#define USART_UBBR_VALUE ((F_CPU / (USART_BAUD << 4)) - 1)

static void uart_init() {
    UBRR0 = USART_UBBR_VALUE;
    UCSR0A = 0;
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

static void gpio_init()
{
    DDRB = 0b101101;
}

static void spi_init()
{
    SPSR = SPI2X; //clk / 2
    SPCR = (1 << SPE) | (1 << MSTR);
}

static void spi_chip_enable()
{
    PORTB &= ~0b00000100;
}

static void spi_chip_disable()
{
    PORTB |= 0b00000100;
}

static void spi_wait_write()
{
    while(!(SPSR & (1 << SPIF)));
}

static uint8_t spi_read_reg(uint8_t reg)
{
    spi_chip_enable();
    SPDR = reg;
    spi_wait_write();
    SPDR = 0;
    spi_wait_write();
    spi_chip_disable();
    return SPDR;
}

static void spi_print_reg(uint8_t reg)
{
    uint8_t val = spi_read_reg(reg);
    fprintf(&uart_str, "REG %02X=%02X\r\n", reg, val);
}

static void si4432_enable()
{
    PORTB &= ~0b00000001;
}

static void si4432_disable()
{
    PORTB |= 0b00000001;
}

static uint8_t si4432_init()
{
    si4432_disable();
    _delay_ms(20);
    si4432_enable();
    _delay_ms(100); //20 ms is not enough!

    if(0x08 != spi_read_reg(0x00)) {
        fprintf(&uart_str, "Unsupported device type\r\n");
        return 1;
    }
        
    if(0x06 != spi_read_reg(0x01)) {
        fprintf(&uart_str, "Unsupported device version\r\n");
        return 1;
    }
    fprintf(&uart_str, "%s, OK\r\n", __func__);
    return 0;
}

static void sys_init()
{
    cli();
    uart_init();
    gpio_init();
    spi_init();
    sei();
}

int main()
{
    sys_init();
    fprintf(&uart_str, "%s\r\n", __func__);
    si4432_init();

    return 0;
}
