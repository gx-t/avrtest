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

/*

TODO:
1. Decrease power consumption using PRR register
2. Use TX Done interrupt

Atmega328p Vcc ADC:
https://arduino.stackexchange.com/questions/23526/measure-different-vcc-using-1-1v-bandgap

BMP180:
https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
https://github.com/ControlEverythingCommunity/BMP180/blob/master/C/BMP180.c

ATMEGA328 + LoRa RA01 transmit mode

mcu = ATMEGA328:
lora = RA01
mcu.1 - 3300pf - gnd
mcu.7 - vcc
mcu.7 - 0.1uf - gnd
mcu.8 - gnd
mcu.9 - qz 32768hz - 10
mcu.14 - lora.reset
mcu.15 - lora.dio0
mcu.16 - lora.nss
mcu.16 - 100K - vcc
mcu.17 - lora.mosi
mcu.18 - lora.miso
mcu.19 - lora.sck
mcu.20 - vcc
mcu.21 - 0.1uF - gnd
mcu.22 - gnd



SX1276/77/78/79 datasheet:
https://www.semtech.com/uploads/documents/DS_SX1276-7-8-9_W_APP_V5.pdf

Decoding LoRa - modulation
https://revspace.nl/DecodingLora

 */

///////////////////////////////////////////////////////////////////////////////

static void rtc_init()
{  
    TCCR2A = 0x00;  //overflow
    TCCR2B = 0x05;  //1 s
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
    UCSR0B = (1 << TXEN0);

    //8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_tx(uint8_t data)
{
    UDR0 = data;
    while (!(UCSR0A & (1 << UDRE0)));
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
    p_str("\r\n");
}

static void p_name_value(const char* name, const char* val, const char* units)
{
    p_str(name);
    p_str(" = ");
    p_str(val);
    p_line(units);
}

///////////////////////////////////////////////////////////////////////////////

static void lora_reset_pin()
{
    PORTB &= ~LORA_RST;
    _delay_us(100);
    PORTB |= LORA_RST;
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
    uart_tx(' ');
    while(i) {
        uart_tx(val & i ? '1' : '0');
        i >>= 1;
    }
}

static void lora_print_reg(uint8_t reg)
{
    uint8_t val = lora_read_reg(reg);
    p_hex_digit(reg);
    p_str(": ");
    p_hex_digit(val);
    p_str(" = ");
    p_binary(val);
    p_str("\r\n");
}

//RegOpMode (0x01)
static void lora_set_sleep_mode()
{
    lora_write_reg(0x01, 0b10001000);
}

static void lora_init_tx()
{
    lora_reset_pin();
    static const uint8_t lora_tx_init_blob[] = {
        0x01, 0b10001000 //Sleep Mode
            , 0x06, 0x6C //MSB 433920000 Hz
            , 0x07, 0x7A //Mid
            , 0x08, 0xE1 //LSB
            , 0x0B, 0b00001011 //OCP off
            , 0x0E, 0x00 //TX base address
            , 0x00, 'L' //Data
            , 0x1D, 0b00100011 //BW = 15.2 Khz, CR=4/5, implicit header
            , 0x1E, 0b11000100 //SF = 12, CRC
            , 0x20, 0x00 //Preamble len MSB
            , 0x21, 0x06 //Preamble len LSB
            , 0x22, 0x01 //Payload length = 1
            , 0x26, 0b00001100 //Low Data Rate Optimize on, AGC on
            , 0x31, 0xC3 //Data Detection Optimize for SF = 7..12
            , 0x37, 0x0A //Detection Threshold for SF = 7..12
            , 0x39, 0x12 //Synch Word = 0x12
            , 0x40, 0b01000000 //Map TX Done to DIO0
            , 0x4D, 0b10000111 //PA BOOST on
            , 0x09, 0b11111111 //Max output power
            , 0x01, 0b10001011 //TX mode
            , 0xFF, 0xFF //end
    };
    const uint8_t* pp = lora_tx_init_blob;
    while(0xFF != *pp) {
        lora_write_reg(pp[0], pp[1]);
        pp+=2;
    }
}

static void lora_print_settings()
{
    lora_print_reg(0x01);
    lora_print_reg(0x06);
    lora_print_reg(0x07);
    lora_print_reg(0x08);
    lora_print_reg(0x19);
    lora_print_reg(0x1A);
    lora_print_reg(0x1B);
    lora_print_reg(0x1D);
    lora_print_reg(0x1E);
}

static uint8_t lora_check_tx_done()
{
    return !!(0b0001000 & lora_read_reg(0x12));
}

static void adc_read_vcc()
{
    ADMUX = 0b01001110; 
    ADCSRA = 0b11000111;
    while(ADCSRA & 0b01000000);
    ADCSRA = 0b11000111;
    while(ADCSRA & 0b01000000);

    //VCC = 1.1 * 1023 / read
    p_hex_digit(ADCL);
    p_hex_digit(ADCH);
    ADMUX = 0b00001111;
    ADCSRA = 0b00000111;
    p_line("");
}

static void f_tx()
{
    lora_init_tx();
    p_line("TX");
    adc_read_vcc();
//    lora_print_settings();
    while(!(PINB & LORA_RX_TX_DONE) || !lora_check_tx_done()) {
        p_line("TX Check");
        sleep_cpu();
    }
    p_line("TX Done");
    lora_set_sleep_mode();
    adc_read_vcc();
}

static void f_pause()
{
    uint16_t count = 1;
    while(count --) {
        sleep_cpu();
    }
}

static void sys_init()
{
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    uart_init();
    spi_init();
    rtc_init();
    sei();
}

int main(void)
{
    sys_init();
    while(1) {
        f_tx();
        f_pause();
    } 
    return 0;
}

