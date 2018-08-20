#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define F_CPU 8000000UL
#include <util/delay.h>

#define LORA_RST        (1 << PB0)
#define LORA_RX_TX_DONE (1 << PB1)
#define LORA_TX_DONE    (1 << PB1)
#define LORA_NSS        (1 << PB2)
#define SPI_MOSI        (1 << PB3)
#define SPI_MISO        (1 << PB4)
#define SPI_SCK         (1 << PB5)

#define LED_PIN         (1 << PC0)

/*TODO:
  Try to write FIFO once and reset pos. for send ... done, write data to any offset, send reads from 0x00. Fifo stores data on extremely low power!
  Modify functions to remove lora_update_reg as much as possible create (register centered functions)
  Reset fifo to read
  Add voltage and temperature monitoring

  Command send-receive mode
*/

/*
JS calculations:
function calc_t_sym(sf, bw)
{
    return 2 ** sf / bw;
}

calc_t_sym(8, 7800);
0.03282051282051282

function calc_preample_t(n, tsym)
{
    return (n + 4.25) * tsym;
}

calc_preample_t(6, 0.03282051282051282);
0.3364102564102564

function calc_payload_symb_nb(pl, sf, h, de, cr)
{
    return 8 + Math.max(Math.ceil((8*pl - 4*sf +28 +16 - 20*h) / 4 / (sf - 2 * de)) * (cr + 4), 0);
}

calc_payload_symb_nb(1, 8, 0, 0, 1);
13

*/

/*
ATMEGA328 + LoRa RA01 receive mode
ATMEGA 328P:
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

Interrupts: RTC (timer 2 overflow), UART RX, pin change PB1
!! PB2 -- 100K -- VCC (disable loRa during ASP programming)

SX1276/77/78/79 datasheet:
https://www.semtech.com/uploads/documents/DS_SX1276-7-8-9_W_APP_V5.pdf

Designer’s Guide. AN1200.13
https://www.semtech.com/uploads/documents/LoraDesignGuide_STD.pdf

Decoding LoRa - modulation, SF, CR, BW:
https://revspace.nl/DecodingLora

RX = "receive"
TX = "transmit"
SF = "spreading factor"
BW = "bandwidth"
ECR = "error coding rate"
AGC = "automatic gain control"
OCP = "overcurrent protection"
PA = "power amplifier"
PCINT = "pin change interrupt"
*/

///////////////////////////////////////////////////////////////////////////////

static void led_init()
{
    DDRC |= LED_PIN;
}

static void led_on()
{
    PORTC |= LED_PIN;
}

static void led_off()
{
    PORTC &= ~LED_PIN;
}

///////////////////////////////////////////////////////////////////////////////

static void sys_enable_pcint1()
{
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT1);
}

static void rtc_init()
{  
    TCCR2A = 0x00;  //overflow
    TCCR2B = 0x03;  //0.25 s
	TIMSK2 = 0x01;  //enable timer2A overflow interrupt
    ASSR  = 0x20;   //enable asynchronous mode
}

ISR(TIMER2_OVF_vect)
{
}

///////////////////////////////////////////////////////////////////////////////

static void spi_init()
{
    DDRB = LORA_RST | LORA_NSS | SPI_MOSI | SPI_SCK;
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPI2X); //enable SPI-master, clock/2 speed
    PORTB |= LORA_NSS;
}

static void spi_chip_enable()
{
    PORTB &= ~LORA_NSS;
}

static void spi_chip_disable()
{
    PORTB |= LORA_NSS;
}

static void spi_wait_write()
{
    while(!(SPSR & (1 << SPIF)));
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define USART_BAUD 38400UL
#define USART_UBBR_VALUE ((F_CPU / (USART_BAUD << 4)) - 1)

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

static uint8_t uart_rx()
{
    return UDR0;
}

static void p_str(const char* str)
{
    if(!str)
        return;
    while(*str) {
        uart_tx(*str++);
    }
}

static void p_line(const char* pp)
{
    p_str(pp);
    uart_tx('\r');
    uart_tx('\n');
}

static void p_name_value(const char* name, const char* val, const char* units)
{
    p_str(name);
    p_str(" = ");
    p_str(val);
    p_line(units);
}

///////////////////////////////////////////////////////////////////////////////

static void sys_error()
{
    while(1) {
        led_on();
        sleep_cpu();
        led_off();
        sleep_cpu();
    }
}

///////////////////////////////////////////////////////////////////////////////

static void lora_reset()
{
    PORTB &= ~LORA_RST;
    _delay_us(100);
    PORTB |= LORA_RST;
    _delay_ms(100);
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

static void p_hex_digit(uint8_t val)
{
    static const uint8_t hex_chars[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    p_str("0x");
    uart_tx(hex_chars[(val & 0xF0) >> 4]);
    uart_tx(hex_chars[(val & 0x0F)]);
}

static void p_binary(uint8_t val)
{
    uint8_t i = 0b10000000;
    while(i > 0b00001000) {
        uart_tx(val & i ? '1' : '0');
        i >>= 1;
    }
    uart_tx('.');
    while(i) {
        uart_tx(val & i ? '1' : '0');
        i >>= 1;
    }
}

static void lora_print_reg(uint8_t reg)
{
    uint8_t val = lora_read_reg(reg);
    p_str("REG ");
    p_hex_digit(reg);
    p_str(" = ");
    p_hex_digit(val);
    p_str(" = ");
    p_binary(val);
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

static void lora_set_payload_length_1()
{
    lora_write_reg(0x22, 0x01);
}

static void lora_set_payload_length_5()
{
    lora_write_reg(0x22, 0x05);
}

static void lora_set_explicit_header()
{
    lora_update_reg(0x1D, 0xFE, 0x00);
}

static void lora_set_implicit_header()
{
    lora_update_reg(0x1D, 0xFE, 0x01);
}

static void lora_set_error_crc_cr_4_8()
{
    lora_update_reg(0x1D, 0xF1, 4 << 1);
}

static void lora_set_bandwidth_62_5()
{
    lora_update_reg(0x1D, 0x0F, 6 << 4);
}

static void lora_set_bandwidth_7_8()
{
    lora_update_reg(0x1D, 0x0F, 0 << 4);
}

static void lora_set_sf_12()
{
    lora_update_reg(0x1E, 0x0F, 12 << 4);
}

static void lora_set_sf_8()
{
    lora_update_reg(0x1E, 0x0F, 8 << 4);
}

static void lora_set_crc_off()
{
    lora_update_reg(0x1E, 0xFB, 0x00);
}

static void lora_set_ocp_off()
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

static void lora_set_detection_optimize_for_sf_7to12()
{
    lora_write_reg(0x31, 0xC3);
}

static void lora_set_detection_threshold_for_sf_7to12()
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

static void lora_map_tx_to_dio0()
{
    lora_write_reg(0x40, 1 << 6);
}

static void lora_set_rx_cont_mode()
{
    lora_update_reg(0x01, 0b11111000, 0b101);
}

static void lora_set_rx_mode()
{
    lora_update_reg(0x01, 0b11111000, 0b110);
}

static void lora_set_tx_mode()
{
    lora_update_reg(0x01, 0b11111000, 0b011);
}

static void lora_reset_irq()
{
    lora_write_reg(0x12, 0xff);
}

static uint8_t lora_get_rx_data_len()
{
    return lora_read_reg(0x13);
}

static uint8_t lora_get_rx_data_address()
{
    return lora_read_reg(0x10);
}

static void lora_set_fifo_buffer_address(uint8_t address)
{
    lora_write_reg(0x0D, address);
}

static void lora_init_common()
{
    lora_reset();
    if(0x12 != lora_read_reg(0x42))
        return sys_error();

    lora_set_sleep_mode();
    lora_set_lora_mode();
    lora_set_implicit_header();
    lora_set_payload_length_1(); //needed for implicit header mode
    lora_set_error_crc_cr_4_8();
    lora_set_bandwidth_7_8();
    lora_set_sf_8();
    lora_set_crc_off();
    lora_set_ocp_off();
    lora_set_max_tx_power_20dbm();
    lora_set_pa_boost_20dbm();
    lora_set_syncword_0x12();
    lora_set_preample_len_6();
    lora_set_agc_on();
    lora_set_lna_gain_highest();
    lora_reset_tx_base_address();
    lora_reset_rx_base_address();
    lora_set_detection_optimize_for_sf_7to12();
    lora_set_detection_threshold_for_sf_7to12();
    lora_set_freq_434800000();
    lora_set_low_data_optimize_on();
    lora_set_standby_mode();
}

static void lora_switch_bw()
{
    uint8_t val = lora_read_reg(0x1D);
    val >>= 4;
    val ++;
    if(9 < val)
        val = 0;
    lora_update_reg(0x1D, 0x0F, val << 4);
    lora_print_reg(0x1D);
}

static void lora_switch_sf()
{
    uint8_t val = lora_read_reg(0x1E);
    val >>= 4;
    val ++;
    if(12 < val)
        val = 6;
    lora_update_reg(0x1E, 0x0F, val << 4);
    lora_print_reg(0x1E);
}

static void lora_print_settings()
{
    lora_print_reg(0x01);
    lora_print_reg(0x06);
    lora_print_reg(0x07);
    lora_print_reg(0x08);
    lora_print_reg(0x1D);
    lora_print_reg(0x1E);
    uart_tx('\r');
    uart_tx('\n');
}

static void show_usage()
{
    p_line("Usage:");
    p_line("i - Current settings");
    p_line("r - RX mode (default)");
    p_line("t - TX mode");
    p_line("w - Switch BW");
    p_line("s - Switch SF");
    p_line("q - Restart");
}

static void lora_read_rx_data()
{
    lora_set_fifo_buffer_address(lora_get_rx_data_address());
    spi_chip_enable();
    SPDR = 0x00;
    spi_wait_write();
    uint8_t bt = SPDR;
    spi_chip_disable();
    'L' == bt ? led_on() : led_off();
}

static void lora_send_tx_data()
{
    lora_set_tx_mode();
    led_on();
}

static void lora_init_rx()
{
    lora_map_rx_to_dio0();
    lora_set_fifo_buffer_address(0x00);
    lora_set_rx_cont_mode();
    p_line("RX mode.");
}

static void lora_init_tx()
{
    lora_map_tx_to_dio0();
    p_line("TX mode.");
    lora_set_fifo_buffer_address(0x00);
    lora_write_reg(0x00, 'L');
    lora_send_tx_data();
}

static void lora_restart()
{
    lora_init_common();
    lora_init_rx();
    lora_print_settings();
}

ISR(USART_RX_vect)
{
    if(!(UCSR0A & (1 << RXC0)))
        return;
    uint8_t ch = uart_rx();
    if('i' == ch)
        return lora_print_settings();
    if('r' == ch)
        return lora_init_rx();
    if('t' == ch)
        return lora_init_tx();
    if('w' == ch)
        return lora_switch_bw();
    if('s' == ch)
        return lora_switch_sf();
    if('q' == ch)
        return lora_restart();
    show_usage();
}

static uint8_t lora_check_rx_done()
{
    return !!(0b1000000 & lora_read_reg(0x12));
}

static uint8_t lora_check_tx_done()
{
    return !!(0b0001000 & lora_read_reg(0x12));
}

ISR(PCINT0_vect)
{
    if(!(PINB & LORA_RX_TX_DONE))
        return;
    uint8_t status = lora_read_reg(0x12);

    do {
        if(0b1000000 & status) {
            lora_read_rx_data();
            break;
        }
        else if(0b0001000 & status) {
            lora_send_tx_data();
            break;
        }
    }while(1);
    lora_reset_irq();
}

static void sys_init()
{
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    uart_init();
    spi_init();
    sys_enable_pcint1();
    led_init();
    sei();
}

int main(void)
{
    sys_init();
    lora_init_common();
    lora_init_rx();
    lora_print_settings();
    while(1) {
        sleep_cpu();
        _delay_ms(10);
        led_off();
    } 
    return 0;
}

