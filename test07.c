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

struct {
    uint8_t led_status : 1;
    uint8_t tx_delay : 3;
    uint8_t padding : 4;
} static g_flags = {1, 6, 0};

/*

static uint8_t lora_init_blob[] = {
    0x00, 0b10001000 //Sleep Mode
    , 0x06, 0x6C //MSB 434800000 Hz
    , 0x07, 0xB3 //Mid
    , 0x08, 0x34 //LSB
    , 0x0B, 0b00001011 //OCP off
    , 0x0C, 0b100000 //gain - highest
    , 0x0E, 0x00 //TX base address
    , 0x0F, 0x00 //RX base address
    , 0x1D, 0b00001001 //BW = 7.8 Khz, CR=4/8, implicit header
    , 0x1E, 0b10000000 //SF = 8, No CRC
    , 0x20, 0x00 //Preamble len MSB
    , 0x21, 0x06 //Preamble len LSB
    , 0x22, 0x20 //Payload length = 32
    , 0x26, 0b00001100 //Low Data Rate Optimize on, AGC on
    , 0x31, 0xC3 //Data Detection Optimize for SF = 7..12
    , 0x37, 0x0A //Detection Threshold for SF = 7..12
    , 0x39, 0x12 //Synch Word = 0x12
    , 0x4D, 0b10000111 //PA BOOST on
    , 0x09, 0b11111111 //Max output power
    , 0x01, 0b10001001 //Standby Mode
};*/

/*TODO:
  Add register monitoring
  Add voltage monitoring
    https://arduino.stackexchange.com/questions/23526/measure-different-vcc-using-1-1v-bandgap
    https://arduino.stackexchange.com/questions/16352/measure-vcc-using-1-1v-bandgap
  Add bmp180 monitoring

  #include <avr/eeprom.h>
  double EEMEM EEVar;
  double a;

  void WriteDoubleToEeprom(double x){
    eeprom_write_block((const void*)&x, (void*)&EEVar, sizeof(double));
  }

  double ReadDoubleFromEeprom(void){
    double temp;
    eeprom_read_block((void*)&temp, (const void*)&EEVar, sizeof(double));
    return(temp);
  }

  Display RSSI and SNR ... added lora_print_register call
  Update 0x31 and 0x37 on SF change ... OK

  Command send-receive mode
 */

/*
Frequency JS calculation
F = Fr * (2 << 18) / 32e6;

Fr = 32e6 * F / (2 << 18);
*/

/*
   JS calculations:
Designer’s Guide. AN1200.13
https://www.semtech.com/uploads/documents/LoraDesignGuide_STD.pdf

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
    if(!g_flags.led_status)
        return;

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
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

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

static void lora_reset_pin()
{
    PORTB &= ~LORA_RST;
    _delay_us(100);
    PORTB |= LORA_RST;
    _delay_ms(10);
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

static void lora_update_reg(uint8_t reg, uint8_t mask, uint8_t val)
{
    lora_write_reg(reg, val | (mask & lora_read_reg(reg)));
}

//RegOpMode (0x01)
static void lora_set_sleep_mode()
{
    lora_write_reg(0x01, 0b10001000);
}

static void lora_set_standby_mode()
{
    lora_write_reg(0x01, 0b10001001);
}

static void lora_set_tx_mode()
{
    lora_write_reg(0x01, 0b10001011);
}

static void lora_set_rx_cont_mode()
{
    lora_write_reg(0x01, 0b10001101);
}

static void lora_set_rx_mode()
{
    lora_write_reg(0x01, 0b10001110);
}

static void lora_set_freq_434800000()
{

    //Frf = Fosc * reg_value / 2 ^ 19
    //p. 109

    lora_write_reg(0x06, 0x6C);
    lora_write_reg(0x07, 0xB3);
    lora_write_reg(0x08, 0x34);
}

static void lora_set_freq_433920000()
{
    lora_write_reg(0x06, 0x6C);
    lora_write_reg(0x07, 0x7A);
    lora_write_reg(0x08, 0xE1);
}

//RegOcp (0x0B);
static void lora_set_ocp_off()
{
    lora_write_reg(0x0B, 0b00001011);
}

//RegLna
static void lora_set_lna_gain_highest()
{
    lora_write_reg(0x0C, 0b100000);
}

//RegFifoTxBaseAddr
static void lora_reset_tx_base_address()
{
    lora_write_reg(0x0E, 0x00);
}

//RegFifoAddrPtr
static void lora_set_fifo_buffer_address(uint8_t address)
{
    lora_write_reg(0x0D, address);
}

//RegFifoRxBaseAddr
static void lora_reset_rx_base_address()
{
    lora_write_reg(0x0F, 0x00);
}

//RegFifoRxCurrentAddr
static uint8_t lora_get_rx_data_address()
{
    return lora_read_reg(0x10);
}

//RegIrqFlags
static void lora_reset_irq()
{
    lora_write_reg(0x12, 0xff);
}

//RegRxNbBytes
static uint8_t lora_get_rx_data_len()
{
    return lora_read_reg(0x13);
}

//RegModemConfig1 (0x1D)
static void lora_set_bw78_cr48_implicit()
{
    lora_write_reg(0x1D, 0b00001001);
}

static void lora_set_bw78_cr45_implicit()
{
    lora_write_reg(0x1D, 0b00000011);
}

//RegModemConfig2 (0x1E)
static void lora_set_sf8_nocrc()
{
    lora_write_reg(0x1E, 0b10000000);
}

static void lora_set_sf8_crc()
{
    lora_write_reg(0x1E, 0b10000100);
}

//RegPreambleMsb, RegPreambleLsb
static void lora_set_preample_len_6()
{
    lora_write_reg(0x20, 0x00); //MSB
    lora_write_reg(0x21, 0x06); //LSB
}

//RegPayloadLength (0x22)
static void lora_set_payload_length_1()
{
    lora_write_reg(0x22, 0x01);
}

static void lora_set_payload_length_5()
{
    lora_write_reg(0x22, 0x05);
}

static void lora_set_payload_length_32()
{
    lora_write_reg(0x22, 0x20);
}

//RegModemConfig3
static void lora_set_ldoon_agcon()
{
    lora_write_reg(0x26, 0b00001100);
}

//RegDetectOptimize
static void lora_set_detection_optimize_for_sf_7to12()
{
    lora_write_reg(0x31, 0xC3);
}

static void lora_set_detection_optimize_for_sf_6()
{
    lora_write_reg(0x31, 0xC5);
}

//RegDetectionThreshold
static void lora_set_detection_threshold_for_sf_7to12()
{
    lora_write_reg(0x37, 0x0A);
}

static void lora_set_detection_threshold_for_sf_6()
{
    lora_write_reg(0x37, 0x0C);
}

//RegSyncWord
static void lora_set_syncword_0x12()
{
    lora_write_reg(0x39, 0x12);
}

//RegDioMapping1
static void lora_map_rx_to_dio0()
{
    lora_write_reg(0x40, 0 << 6);
}

static void lora_map_tx_to_dio0()
{
    lora_write_reg(0x40, 1 << 6);
}

//RegPaDac
static void lora_set_max_tx_power_20dbm()
{
    lora_write_reg(0x4D, 0b10000111);
    lora_write_reg(0x09, 0b11111111);
}

static void lora_init_common()
{
    lora_reset_pin();
    if(0x12 != lora_read_reg(0x42))
        return sys_error();

    lora_set_sleep_mode();
    lora_set_freq_433920000();
    lora_set_ocp_off();
    lora_set_lna_gain_highest();
    lora_reset_tx_base_address();
    lora_reset_rx_base_address();
    lora_set_bw78_cr45_implicit();
    lora_set_sf8_crc();
    lora_set_preample_len_6();
    lora_set_payload_length_1();
    lora_set_ldoon_agcon();
    lora_set_detection_optimize_for_sf_7to12();
    lora_set_detection_threshold_for_sf_7to12();
    lora_set_syncword_0x12();
    lora_set_max_tx_power_20dbm();
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
    if(12 < val) {
        val = 6;
        lora_set_detection_optimize_for_sf_6();
        lora_set_detection_threshold_for_sf_6();
    } else {
        lora_set_detection_optimize_for_sf_7to12();
        lora_set_detection_threshold_for_sf_7to12();
    }
    lora_update_reg(0x1E, 0x0F, val << 4);
    lora_print_reg(0x1E);
}

static void print_led_status()
{
    p_str("LED status: ");
    p_line(g_flags.led_status ? "enabled" : "disabled");
}

static void print_tx_delay()
{
    p_str("TX delay: ");
    p_hex_digit(g_flags.tx_delay);
    p_line("");
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
    print_led_status();
    print_tx_delay();
}

static void show_usage()
{
    p_line("Usage:");
    p_line("i - Display registers");
    p_line("m - Next mode (RX,TX,SLEEP)");
    p_line("w - Next BW");
    p_line("s - Next SF");
    p_line("l - LED enable/disable");
    p_line("t - TX delay 0 - 7 log. units");
}

static void lora_init_rx()
{
    p_line("RX");
    lora_map_rx_to_dio0();
    lora_set_fifo_buffer_address(0x00);
    lora_set_rx_cont_mode();
}

static void lora_read_fifo_32_bytes(uint8_t buff[32])
{
    uint8_t* pp = buff;
    uint8_t count = 32;
    lora_set_fifo_buffer_address(lora_get_rx_data_address());
    spi_chip_enable();
    SPDR = 0;
    spi_wait_write();
    while(count--) {
        SPDR = 0;
        spi_wait_write();
        *pp++ = SPDR;
    }
    spi_chip_disable();
}

static void lora_read_rx_data()
{
    lora_set_fifo_buffer_address(lora_get_rx_data_address());
    'L' == lora_read_reg(0x00) ? led_on() : led_off();
}

static void lora_write_fifo_32_bytes(const uint8_t data[32])
{
    const uint8_t* pp = data;
    uint8_t count = 32;
    spi_chip_enable();
    SPDR = 0x80;
    spi_wait_write();
    while(count--) {
        SPDR = *pp++;
        spi_wait_write();
    }
    spi_chip_disable();
}

static void f_prepare_send_data()
{
    lora_write_reg(0x00, 'L');
}

static void lora_send_tx_data()
{
    f_prepare_send_data();
    lora_set_tx_mode();
    led_on();
}

static void lora_init_sleep()
{
    lora_set_sleep_mode();
    p_line("SLEEP");
}

static void lora_init_tx()
{
    p_line("TX");
    lora_map_tx_to_dio0();
    lora_send_tx_data();
}

static void led_toggle_status()
{
    g_flags.led_status = !g_flags.led_status;
    print_led_status();
}

static void switch_tx_delay()
{
    g_flags.tx_delay ++;
    print_tx_delay();
}

static void f_uart(uint8_t* state)
{
    if(!(UCSR0A & (1 << RXC0)))
        return;
    uint8_t ch = uart_rx();
    if('i' == ch)
        return lora_print_settings();
    if('m' == ch) {
        ++(*state);
        return;
    }
    if('w' == ch)
        return lora_switch_bw();
    if('s' == ch)
        return lora_switch_sf();
    if('l' == ch)
        return led_toggle_status();
    if('t' == ch)
        return switch_tx_delay();
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

static void f_sleep_loop()
{
    uint8_t run = 0;
    lora_init_sleep();
    led_off();
    while(!run) {
        sleep_cpu();
        f_uart(&run);
    }
}

static void f_tx_lora(uint8_t* count)
{
    if(!(PINB & LORA_RX_TX_DONE))
        return;
    if(!lora_check_tx_done())
        return;
    lora_reset_irq();
    led_off();
    *count = (1 << g_flags.tx_delay);
}

static void f_tx_rtc(uint8_t* count)
{
    if(!*count)
        return;
    if(!--(*count))
        lora_send_tx_data();
}

static void f_tx_loop()
{
    uint8_t run = 0;
    uint8_t count = 1;
    lora_init_tx();
    while(!run) {
        sleep_cpu();
        f_tx_rtc(&count);
        f_tx_lora(&count);
        f_uart(&run);
    }
}

static void f_rx_rtc()
{
    led_off();
}

static void f_rx_lora()
{
    if(!(PINB & LORA_RX_TX_DONE))
        return;
    if(!lora_check_rx_done())
        return;
    lora_reset_irq();
    lora_read_rx_data();
}

static void f_rx_loop()
{
    uint8_t run = 0;
    lora_init_rx();
    while(!run) {
        sleep_cpu();
        f_rx_rtc();
        f_rx_lora();
        f_uart(&run);
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
    lora_init_common();
    while(1) {
        f_rx_loop();
        f_tx_loop();
        f_sleep_loop();
    } 
    return 0;
}

