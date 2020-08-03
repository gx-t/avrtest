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

Decoding LoRa - modulation, SF, CR, BW:
https://revspace.nl/DecodingLora
 */

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
    //TODO: Wrong initialzation! Use this for fast SPI:
    //SPSR = SPI2X; //clk / 2
    //SPCR = (1 << SPE) | (1 << MSTR);
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

static void lora_init_rx()
{
    lora_reset_pin();
    static const uint8_t lora_init_blob[] = {
        0x01, 0x88 //Sleep Mode
            , 0x06, 0x6c //MSB 433920000 Hz
            , 0x07, 0x7a //Mid.
            , 0x08, 0xe1 //LSB
            , 0x0B, 0b00001011 //OCP off
            , 0x0C, 0b00100000 //LNA highest gain
            , 0x0E, 0x00 //TX base address
            , 0x0F, 0x00 //RX base address
            , 0x1D, 0x23 //BW, CR, header mode
            , 0x1E, 0xc4 //SF, CRC
            , 0x20, 0x0 //Preamble len MSB
            , 0x21, 0x6 //Preamble len LSB
            , 0x22, 0x1 //Payload length = 1
            , 0x26, 0xc //Low Data Rate Optimize, AGC
            , 0x31, 0xC3 //Data Detection Optimize for 7..12
            , 0x37, 0x0a //Data Detection Threshold for 7..12
            , 0x39, 0x12 //Synch Word
            , 0x40, 0x00 //Map RX Done to DIO0
            , 0x01, 0x8d //Receive continuous (RXCONTINUOUS)

            , 0xFF, 0xFF //END
    };
    const uint8_t* pp = lora_init_blob;
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

static uint8_t lora_check_rx_done()
{
    return !!(0b1000000 & lora_read_reg(0x12));
}

static void lora_set_fifo_buffer_address(uint8_t address)
{
    lora_write_reg(0x0D, address);
}

static void lora_read_rx_data()
{
    uint8_t data = lora_read_reg(0x00);
    
    'L' == data ? p_line("L") : p_line(":(");
}

//RegIrqFlags
static void lora_reset_irq()
{
    lora_write_reg(0x12, 0xff);
}

static void f_rx()
{
    lora_init_rx();
    lora_print_settings();
    while(1) {
        while(!(PINB & LORA_RX_TX_DONE) || !lora_check_rx_done()) {
            p_line("RX Check");
            sleep_cpu();
        }
        p_line("RX Done");
        lora_reset_irq();
        lora_read_rx_data();
    }
}

static void sys_init()
{
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    uart_init();
    spi_init();
    led_init();
    rtc_init();
    sei();
}

int main(void)
{
    sys_init();
    f_rx();
    return 0;
}

