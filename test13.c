//https://www.silabs.com/documents/public/application-notes/AN415.pdf
//https://www.silabs.com/documents/public/application-notes/AN440.pdf
//https://www.silabs.com/documents/public/data-sheets/Si4430-31-32.pdf
//https://www.silabs.com/documents/public/example-code/AN415_EZRadioPRO_Sample_Code.zip

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


static void cpu_set_freq_8M()
{
    cli();
    CLKPR = 0x80;
    CLKPR = 0x00;
    sei();
}

static void cpu_set_freq_1M()
{
    cli();
    CLKPR = 0x80;
    CLKPR = 0x03;
    sei();
}

static void wdt_init()
{
    wdt_reset();
    WDTCSR = 0b00011000;
    WDTCSR = 0b01000111;
}

ISR(WDT_vect)
{
}

static void wdt_sleep_2s()
{
    WDTCSR = 0b00011000;
    WDTCSR = 0b01000111;
    sleep_cpu();
}

static void wdt_sleep_1s()
{
    WDTCSR = 0b00011000;
    WDTCSR = 0b01000110;
    sleep_cpu();
}

static void wdt_sleep_64ms()
{
    WDTCSR = 0b00011000;
    WDTCSR = 0b01000010;
    sleep_cpu();
}

static void wdt_sleep_32ms()
{
    WDTCSR = 0b00011000;
    WDTCSR = 0b01000001;
    sleep_cpu();
}

static void uart_init() {
    UBRR0 = USART_UBBR_VALUE;
    UCSR0A = 0;
    UCSR0B = (1 << TXEN0);
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
    DDRD |= 0b10000000;
}

static void led_on()
{
    PORTD |= 0b10000000;
}

static void led_off()
{
    PORTD &= ~0b10000000;
}

static void led_flash_1()
{
    led_on();
    wdt_sleep_32ms();
    led_off();
}

static void led_flash_2()
{
    led_flash_1();
    wdt_sleep_64ms();
    led_flash_1();
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

static void spi_write_reg(uint8_t reg, uint8_t val)
{
    spi_chip_enable();
    SPDR = reg | 0x80;
    spi_wait_write();
    SPDR = val;
    spi_wait_write();
    spi_chip_disable();
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

static void si4432_reset()
{
    spi_write_reg(0x07, 0x80);
}

static void si4432_clear_interrupt_flags()
{
    spi_read_reg(0x03);
    spi_read_reg(0x04);
}

static void si4432_set_freq_433_92_low_side()
{
    spi_write_reg(0x75, 0x53);
    spi_write_reg(0x76, 0x62);
    spi_write_reg(0x77, 0x00);
}

static void si4432_set_freq_915_low_side()
{
    spi_write_reg(0x75, 0x75);        
    spi_write_reg(0x76, 0xBB);
    spi_write_reg(0x77, 0x80);
}

static void si4432_set_tx_data_rate_9_6kbs()
{
    spi_write_reg(0x6E, 0x4E);
    spi_write_reg(0x6F, 0xA5);
    spi_write_reg(0x70, 0x20);
}

static void si4432_set_tx_data_rate_1250bs()
{
    spi_write_reg(0x6E, 0x0A);
    spi_write_reg(0x6F, 0x3E);
    spi_write_reg(0x70, 0x20);
}

static void si4432_set_tx_data_rate_123bs()
{
    spi_write_reg(0x6E, 0x01);
    spi_write_reg(0x6F, 0x02);
    spi_write_reg(0x70, 0x20);
}

static void si4432_set_tx_deviation_45k()
{
    spi_write_reg(0x72, 0x48);
}

static void si4432_set_tx_deviation_625b()
{
    spi_write_reg(0x72, 0x01);
}

static void si4432_set_rf_switch_gpio1_tx_gpio2_rx()
{
    spi_write_reg(0x0C, 0x12);
    spi_write_reg(0x0D, 0x15);
}

static void si4432_set_rf_switch_gpio0_tx_gpio1_rx()
{
    spi_write_reg(0x0B, 0x12);
    spi_write_reg(0x0C, 0x15);
}

static void si4432_set_tx_power_1dbm()
{
    spi_write_reg(0x6D, 0x00);
}

static void si4432_set_tx_power_20dbm()
{
    spi_write_reg(0x6D, 0x0F);
}

static void si4432_enable_fifo_and_gfsk()
{
	spi_write_reg(0x71, 0x23);
}

static void si4432_enable_unmodulated()
{
	spi_write_reg(0x71, 0x00);
}

static void si4432_disable_all_interrupts()
{
    spi_write_reg(0x05, 0x00);
    spi_write_reg(0x06, 0x00);
}

static void si4432_set_preample_5_bytes()
{
    spi_write_reg(0x34, 0x0A);
}

static void si4432_set_synch_word_CCCC()
{
    spi_write_reg(0x33, 0x01);
    spi_write_reg(0x36, 0xCC);
    spi_write_reg(0x37, 0xCC);
}

static void si4432_fill_tx_fifo(const uint8_t* data, uint8_t cnt)
{
    spi_write_reg(0x3E, cnt);
    while(cnt--) {
        spi_write_reg(0x7F, *data ++);
    }
}

static void si4432_tx()
{
    led_flash_1();
    const uint8_t data[] = "\x55\x55\x55\x55";
    si4432_fill_tx_fifo(data, 4);

    //Disable all other interrupts and enable the packet sent interrupt only.
    //This will be used for indicating the successfull packet transmission for the MCU
//    spi_write_reg(0x05, 0x04);
//    spi_write_reg(0x06, 0x00);
//
//    si4432_clear_interrupt_flags();

    /*enable transmitter*/
    //The radio forms the packet and send it automatically.
    spi_write_reg(0x07, 0x09);

    //enable the packet sent interupt only
//    spi_write_reg(0x05, 0x04);


    /*wait for the packet sent interrupt*/
    //The MCU just needs to wait for the 'ipksent' interrupt.
//    while(PINB & 0x10);
//    si4432_clear_interrupt_flags();
    while(8 & spi_read_reg(0x07))
        wdt_sleep_32ms();

    led_flash_2();
}

static uint8_t si4432_init()
{
    si4432_enable();
    wdt_sleep_32ms();

    si4432_clear_interrupt_flags();
    si4432_reset();
    si4432_disable_all_interrupts();
    si4432_clear_interrupt_flags();

    while(PINB & 0b10);

    si4432_set_tx_power_20dbm(); //6D
    si4432_set_tx_data_rate_1250bs(); //6E, 6F, 70
    si4432_enable_fifo_and_gfsk(); //71
    si4432_set_tx_deviation_625b(); //72
    si4432_set_freq_433_92_low_side(); //75, 76, 77

    si4432_set_preample_5_bytes();

    si4432_set_synch_word_CCCC();

	//enable the TX packet handler and CRC-16 (IBM) check
	spi_write_reg(0x30, 0x0D);

    si4432_set_rf_switch_gpio0_tx_gpio1_rx();

	//set VCO and PLL
	spi_write_reg(0x5A, 0x7F);
	spi_write_reg(0x59, 0x40);

	//set Crystal Oscillator Load Capacitance register
	spi_write_reg(0x09, 0xD7);

    return 0;
}

static void sys_init()
{
    cli();
    CLKPR = 0x80;
    CLKPR = 0x00;
    wdt_reset();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    uart_init();
    gpio_init();
    spi_init();
    sei();
}

int main()
{
    sys_init();
    si4432_disable();
    wdt_sleep_32ms();
    while(1) {
        si4432_init();
        si4432_tx();
        spi_print_reg(0x07);
        si4432_disable();
        wdt_sleep_2s();
        wdt_sleep_2s();
    }

    return 0;
}
