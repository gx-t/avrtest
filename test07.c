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

static void spi_chip_enable()
{
    PORTB &= ~(1 << PB2);
}

static void spi_chip_disable()
{
    PORTB |= (1 << PB2);
}

static void spi_wait_write()
{
    while(!(SPSR & (1 << SPIF)));
}

//static void spi_write_byte(uint8_t data)
//{
//    spi_chip_enable();
//    SPDR = data;
//    while(!(SPSR & (1 << SPIF))); //wait for write end
//    spi_chip_disable();
//    return SPDR;
//}

///////////////////////////////////////////////////////////////////////////////
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
    UDR0 = data;
    while (!(UCSR0A & (1 << UDRE0)));
}

static void p_line(const char* pp)
{
    while(*pp) {
        uart_tx(*pp++);
    }
    uart_tx('\r');
    uart_tx('\n');
}

static void p_str(const char* str)
{
    while(*str) {
        uart_tx(*str++);
    }
}

///////////////////////////////////////////////////////////////////////////////
static void lora_reset()
{
    _delay_us(100);
    PORTB |= (1 << PB0);
    _delay_ms(5);
}

static uint8_t lora_read_reg(uint8_t reg)
{
    spi_chip_enable();
    SPDR = reg;
    spi_wait_write();
    SPDR = 0;
    spi_wait_write();
    spi_chip_disable();
    return SPDR;
}

static void lora_write_reg(uint8_t reg, uint8_t val)
{
    spi_chip_enable();
    SPDR = reg | 0x80;
    spi_wait_write();
    SPDR = val;
    spi_wait_write();
    spi_chip_disable();
}

static void lora_print_reg(uint8_t reg)
{
    static const uint8_t hex_chars[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    uint8_t val = lora_read_reg(reg);
    p_str("REG 0x");
    uart_tx(hex_chars[(reg & 0xF0) >> 4]);
    uart_tx(hex_chars[(reg & 0x0F)]);
    p_str("=0x");
    uart_tx(hex_chars[(val & 0xF0) >> 4]);
    uart_tx(hex_chars[(val & 0x0F)]);
    uart_tx('\r');
    uart_tx('\n');
}

static void lora_update_reg(uint8_t reg, uint8_t mask, uint8_t val)
{
    lora_write_reg(reg, val | (mask & lora_read_reg(reg)));
}

static void lora_set_sleep_mode()
{
    lora_update_reg(0x01, 0xF8, 0x00);
}

static void lora_set_lora_mode()
{
    lora_update_reg(0x01, 0x7F, 0x80);
}

static void lora_set_explicit_header()
{
    lora_update_reg(0x1D, 0xFE, 0x00);
}

static void lora_set_error_crc_cr8()
{
    lora_update_reg(0x1D, 0xF1, 4 << 1);
}

static void lora_set_bandwidth_62_5()
{
    lora_update_reg(0x1D, 0x0F, 6 << 4);
}

static void lora_set_sf_12()
{
    lora_update_reg(0x1E, 0x0F, 12 << 4);
}

static void lora_set_crc_off()
{
    lora_update_reg(0x1E, 0xFB, 0x00);
}

static void lora_set_overcurrent_prot_off()
{
    lora_update_reg(0x0B, 0b11011111, 0x00);
}

static void lora_set_max_tx_power_20dbm()
{
    lora_write_reg(0x4D, 0x87);
}

static void lora_set_pa_boost_20dbm()
{
    lora_write_reg(0x09, 0xF0 | (20 - 2));
}

static void lora_set_syncword_0x12()
{
    lora_write_reg(0x39, 0x12);
}

static void lora_set_preample_len_6()
{
    lora_write_reg(0x20, 0x00); //MSB
    lora_write_reg(0x21, 0x06); //LSB
}

static void lora_set_agc_on()
{
    lora_write_reg(0x26, 0b100);
}

static void lora_set_lna_gain_highest()
{
    lora_write_reg(0x0C, 0b100000);
}

static void lora_reset_tx_base_address()
{
    lora_write_reg(0x0E, 0x00);
}

static void lora_reset_rx_base_address()
{
    lora_write_reg(0x0F, 0x00);
}

static void lora_set_detection_optimize_for_sf7_to12()
{
    lora_write_reg(0x31, 0xC3);
}

static void lora_set_detection_threshold_for_sf7_to_sf12()
{
    lora_write_reg(0x37, 0x0A);
}

static void lora_set_freq_434800000()
{

    //Frf = Fosc * reg_value / 2 ^ 19
    //p. 109

    lora_write_reg(0x06, 0x6C);
    lora_write_reg(0x07, 0xB3);
    lora_write_reg(0x08, 0x34);
}

static void lora_set_low_data_optimize_on()
{
    lora_update_reg(0x26, 0xF7, 0x01 << 3);
}

static void lora_set_standby_mode()
{
    lora_update_reg(0x01, 0b11111000, 0b001);
}

static void lora_map_rx_to_dio0()
{
    lora_write_reg(0x40, 0 << 6);
}

static void lora_set_rx_cont_mode()
{
    lora_update_reg(0x01, 0b11111000, 0b101);
}

static void lora_init()
{
    lora_reset();
    lora_print_reg(0x42);
    lora_set_sleep_mode();
    lora_set_lora_mode();
    lora_set_explicit_header();
    lora_set_error_crc_cr8();
    lora_set_bandwidth_62_5();
    lora_set_sf_12();
    lora_set_crc_off();
    lora_set_overcurrent_prot_off();
    lora_set_max_tx_power_20dbm();
    lora_set_pa_boost_20dbm();
    lora_set_syncword_0x12();
    lora_set_preample_len_6();
    lora_set_agc_on();
    lora_set_lna_gain_highest();
    lora_reset_tx_base_address();
    lora_reset_rx_base_address();
    lora_set_detection_optimize_for_sf7_to12();
    lora_set_detection_threshold_for_sf7_to_sf12();
    lora_set_freq_434800000();
    lora_set_low_data_optimize_on();
    lora_set_standby_mode();
    lora_map_rx_to_dio0();
    lora_set_rx_cont_mode();
}

static void lora_reset_irq_flags()
{
    lora_write_reg(0x12, 0xff);
}

static void lora_read_rx_data()
{
    uint8_t nbytes = 0;
    lora_write_reg(0x0D, lora_read_reg(0x10));
    nbytes = lora_read_reg(0x13);
    spi_chip_enable();
    SPDR = 0x00;
    spi_wait_write();
    while(nbytes --) {
        SPDR = 0;
        spi_wait_write();
        uart_tx(SPDR);
    }
    spi_chip_disable();
    lora_reset_irq_flags();
    uart_tx('\r');
    uart_tx('\n');
}

static void lora_check_rx_complete_and_read()
{
    if(!(PINB & 0b10))
        return;
    p_line("RECEIVED!");
    if(!(0b1000000 & lora_read_reg(0x12)))
        return;
    lora_read_rx_data();
}

static void sys_init()
{
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    uart_init();
    rtc_init();
    spi_init();
    sei();
}

int main(void)
{
    sys_init();
    lora_init();
    while(1) {
        sleep_cpu();
        if(UCSR0A & (1 << RXC0)) {
            uint8_t ch = UDR0;
            if(ch == '\r') {
                uart_tx('\r');
                uart_tx('\n');
            }
            else {
                uart_tx(ch);
            }
        }
        else {
            lora_check_rx_complete_and_read();
        }
    } 
    return 1;
}

