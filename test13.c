//https://www.silabs.com/documents/public/application-notes/AN415.pdf
//https://www.silabs.com/documents/public/application-notes/AN440.pdf
//https://www.silabs.com/documents/public/data-sheets/Si4430-31-32.pdf
//https://www.silabs.com/documents/public/example-code/AN415_EZRadioPRO_Sample_Code.zip

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/twi.h>
#include <stdio.h>

#define F_CPU 1000000UL
#include <util/delay.h>

#define USART_BAUD 38400UL
#define USART_UBBR_VALUE ((F_CPU / (USART_BAUD << 4)) - 1)


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

static void led_flash_1()
{
    PORTD |= 0b10000000;
    _delay_ms(32);
    PORTD &= ~0b10000000;
}

static void led_flash_2()
{
    led_flash_1();
    _delay_ms(64);
    led_flash_1();
}

static uint8_t spi_read_reg(uint8_t reg)
{
    PORTB &= ~0b00000100;
    SPDR = reg;
    while(!(SPSR & (1 << SPIF)));
    SPDR = 0;
    while(!(SPSR & (1 << SPIF)));
    PORTB |= 0b00000100;
    return SPDR;
}

static void spi_write_reg(uint8_t reg, uint8_t val)
{
    PORTB &= ~0b00000100;
    SPDR = reg | 0x80;
    while(!(SPSR & (1 << SPIF)));
    SPDR = val;
    while(!(SPSR & (1 << SPIF)));
    PORTB |= 0b00000100;
}

static void spi_print_reg(uint8_t reg)
{
    uint8_t val = spi_read_reg(reg);
    fprintf(&uart_str, "REG %02X=%02X\r\n", reg, val);
}

static void si4432_cleanup_interrupt_status_regs()
{
    spi_read_reg(0x03);
    spi_read_reg(0x04);
}

static void si4432_disable_all_interrupts()
{
    spi_write_reg(0x05, 0x00);
    spi_write_reg(0x06, 0x00);
}

static void si4432_enable_all_interrupts()
{
    spi_write_reg(0x05, 0x00);
    spi_write_reg(0x06, 0x00);
}

static void si4432_enable_packet_sent_interrupt_only()
{
    spi_write_reg(0x05, 0x04);
    spi_write_reg(0x06, 0x00);
}

static void si4432_set_freq_433_92Mhz()
{
    spi_write_reg(0x75, 0x53);
    spi_write_reg(0x76, 0x62);
    spi_write_reg(0x77, 0x00);
    //fine tuning
    spi_write_reg(0x73, 0xA0);
}

static void si4432_set_tx_deviation_15khz()
{
    spi_write_reg(0x72, 0x18);
}

static void si4432_set_rx_deviation_25khz()
{
    spi_write_reg(0x72, 0x28);
}

static void si4432_set_tx_data_rate_9_6kbs()
{
    spi_write_reg(0x6E, 0x4E);
    spi_write_reg(0x6F, 0xA5);
    spi_write_reg(0x70, 0x2C);
    spi_write_reg(0x58, 0x80);
}

static void si4432_set_tx_data_rate_1_24kbs()
{
    spi_write_reg(0x6E, 0x0A);
    spi_write_reg(0x6F, 0x28);
    spi_write_reg(0x70, 0x2C);
    spi_write_reg(0x58, 0x80);
}

static void si4432_set_gpio0_tx_gpio1_rx()
{
    spi_write_reg(0x0B, 0x12);
    spi_write_reg(0x0C, 0x15);
}

static void si4432_sw_reset()
{
    spi_write_reg(0x07, 0x80);
}

static void si4432_set_modulation_none()
{
	spi_write_reg(0x71, 0x00);
}

static void si4432_set_modulation_gfsk_fifo_mode()
{
	spi_write_reg(0x71, 0x23);
}

static void si4432_set_preamble_len_5b()
{
    spi_write_reg(0x34, 0x0A);
}

static void si4432_set_synch_word_CCCC_no_header()
{
    spi_write_reg(0x33, 0x02);
    spi_write_reg(0x36, 0xCC);
	spi_write_reg(0x37, 0xCC);
}

static void si4432_enable_tx_handler_and_crc16()
{
	spi_write_reg(0x30, 0x0D);
}

static void si4432_fill_fifo(const uint8_t* data, uint8_t len)
{
    spi_write_reg(0x3E, len);

    while(len --)
        spi_write_reg(0x7F, *data ++);
}

static void si4432_set_tx_power_20dbm()
{
    spi_write_reg(0x6D, 0x0F);
}

static void si4432_vco_pll()
{
    //AN415.pdf, p.25
    spi_write_reg(0x5A, 0x7F);
    spi_write_reg(0x59, 0x40);
}

static void si4432_tx_enable()
{
    spi_write_reg(0x07, 0x09);
}

static void si4432_rx_enable()
{
    spi_write_reg(0x07, 0x05);
}

static void si4432_rx_tx_disable()
{
    spi_write_reg(0x07, 0x01);
}

static void si4432_tx_433_92mhz_20dbm_unmodulated_500ms()
{
    PORTB &= ~0b00000001;
    _delay_ms(32);
    si4432_cleanup_interrupt_status_regs();

    si4432_sw_reset();

    //wait for POR interrupt
    while(PINB & 0b10);
    si4432_cleanup_interrupt_status_regs();

    //wait for chip ready interrupt
    while(PINB & 0b10);
    si4432_cleanup_interrupt_status_regs();

    si4432_disable_all_interrupts();
    si4432_set_freq_433_92Mhz();
    si4432_set_modulation_none();
    si4432_set_gpio0_tx_gpio1_rx();
    si4432_set_tx_power_20dbm();
    si4432_vco_pll();
    si4432_tx_enable();

    led_flash_1();

    _delay_ms(500);

    //shutdown si4431
    PORTB |= 0b00000001;
    led_flash_2();
}

static void si4432_rx_433_92mhz__unmodulated()
{
    PORTB &= ~0b00000001;
    _delay_ms(32);
    si4432_cleanup_interrupt_status_regs();

    si4432_sw_reset();

    //wait for POR interrupt
    while(PINB & 0b10);
    si4432_cleanup_interrupt_status_regs();

    //wait for chip ready interrupt
    while(PINB & 0b10);
    si4432_cleanup_interrupt_status_regs();

    si4432_disable_all_interrupts();
    si4432_set_freq_433_92Mhz();
    si4432_set_modulation_none();

    //modem setup
    spi_write_reg(0x1C, 0x2C);
    spi_write_reg(0x20, 0x68);
    spi_write_reg(0x21, 0x01);
    spi_write_reg(0x22, 0x3A);
    spi_write_reg(0x23, 0x93);
    spi_write_reg(0x24, 0x01);
    spi_write_reg(0x25, 0x95);
    spi_write_reg(0x1D, 0x44);
    spi_write_reg(0x1E, 0x0A);
    spi_write_reg(0x2A, 0x1E);
    spi_write_reg(0x1F, 0x03);
    spi_write_reg(0x69, 0x60);

    spi_write_reg(0x35, 0x00);
    spi_write_reg(0x34, 0x00);

    si4432_set_gpio0_tx_gpio1_rx();
    si4432_vco_pll();

    si4432_rx_enable();
    si4432_enable_all_interrupts();
    si4432_cleanup_interrupt_status_regs();

    while(1) {
        while(PINB & 0b10);
        si4432_cleanup_interrupt_status_regs();
        led_flash_1();
//        spi_print_reg(0x04);
//        spi_print_reg(0x05);

//        si4432_rx_tx_disable();
    }

    //shutdown si4431
    PORTB |= 0b00000001;
}

static void si4432_tx_433_92mhz_gfsk_15khz_9_6kbs_20dbm()
{
    const uint8_t data[] = {
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
    };

    PORTB &= ~0b00000001;
    _delay_ms(32);
    si4432_cleanup_interrupt_status_regs();

    si4432_sw_reset();

    //wait for POR interrupt
    while(PINB & 0b10);
    si4432_cleanup_interrupt_status_regs();

    //wait for chip ready interrupt
    while(PINB & 0b10);
    si4432_cleanup_interrupt_status_regs();

    si4432_disable_all_interrupts();
    si4432_set_freq_433_92Mhz();
    si4432_set_modulation_gfsk_fifo_mode();
    //si4432_set_tx_deviation_15khz();
    si4432_set_rx_deviation_25khz();
    //si4432_set_tx_data_rate_9_6kbs();
    si4432_set_tx_data_rate_1_24kbs();
    si4432_set_preamble_len_5b();
    si4432_set_synch_word_CCCC_no_header();
    si4432_enable_tx_handler_and_crc16();
    si4432_set_gpio0_tx_gpio1_rx();
    si4432_set_tx_power_20dbm();
    si4432_vco_pll();
    si4432_enable_packet_sent_interrupt_only();
    si4432_cleanup_interrupt_status_regs();

    while(1) {
        _delay_ms(2000);
        si4432_fill_fifo(data, sizeof(data));
        led_flash_1();
        si4432_tx_enable();
        //wait for packet sent interrupt
        while(PINB & 0b10);
        si4432_cleanup_interrupt_status_regs();
        si4432_rx_tx_disable();
        led_flash_1();
    }

    //shutdown si4431
    PORTB |= 0b00000001;
}

int main()
{
    cli();

    //clock 1 Mhz
    CLKPR = 0x80;
    CLKPR = 0x03;

    UBRR0 = USART_UBBR_VALUE;
    UCSR0A = 0;
    UCSR0B = (1 << TXEN0);
    //8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    DDRB = 0b101101;
    PORTB |= 0b00000001; //shutdown si4432
    DDRD |= 0b10000000;

    SPSR = SPI2X; //clk / 2
    SPCR = (1 << SPE) | (1 << MSTR);

    sei();

    si4432_tx_433_92mhz_gfsk_15khz_9_6kbs_20dbm();
    //si4432_rx_433_92_unmodulated();
//    while(1) {
//        si4432_tx_433_92_20dbm_unmodulated_500ms();
//        _delay_ms(2000);
//    }

    return 0;
}
