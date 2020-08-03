#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/twi.h>
#include <stdio.h>

#define F_CPU 8000000UL
#include <util/delay.h>

/*
Non-arduino and arduino code example for NRF24L01_PA_LNA
http://www.hotmcu.com/wiki/NRF24L01_PA_LNA_Wireless_Module

NRF24L01_PA_LNA in-depth explanation
https://lastminuteengineers.com/nrf24l01-arduino-wireless-communication/

https://embedds.com/programming-avr-i2c-interface/
https://github.com/knightshrub/I2C-master-lib
https://embedds.com/programming-avr-i2c-interface/
*/

/*
    ATMEGA328P
    GND - 8, 22
    VCC - 7, 20
    RX - PD0
    TX - PD1
    5 (PD3) - 100K 23(ADC0)
    23 (ADC0) - 2.2uF GND

    Loop 5V: 8Mhz-16.3ma, 1Mhz-10.9ma, 128Khz-9.9ma, 31Khz-9.9ma
    Loop 3.3V: 8Mhz-3.74ma, 1Mhz-0.74ma, 128Kh-0.21ma, 31Khz-0.15ma
*/

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

static void spi_chip_enable()
{
    PORTB &= ~0b100;
}

static void spi_chip_disable()
{
    PORTB |= 0b100;
}

static void spi_init()
{
    DDRB = 0b101100;
    spi_chip_disable();
    SPSR = SPI2X; //clk / 2
    SPCR = (1 << SPE) | (1 << MSTR);
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

static void gpio_enable_reset_pullup()
{
    PORTC = 0b01000000;
}

static void wdt_set_2s()
{
    WDTCSR = 0b00011000;
    WDTCSR = 0b01001111;
}

ISR(WDT_vect)
{
    wdt_set_2s();
//    fprintf(&uart_str, "WDT event\r\n");
}

static void cpu_clock_div_set(uint8_t num)
{
    cli();
    CLKPR = 0b10000000;
    CLKPR = num & 0b00001111;
    sei();
}

static void sys_init()
{
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    gpio_enable_reset_pullup();
    uart_init();
    spi_init();
    wdt_reset();
    wdt_set_2s();
    sei();
    fprintf(&uart_str, "main\r\n");
}

static void adc_set_src_1_1v__ref_avcc_with_cap_at_aref_pin()   { ADMUX = 0b01001110; }
static void adc_set_src_adc0__ref_vcc_with_cap_at_aref_pin()    { ADMUX = 0b01000000; }
static void adc_set_src_temp__ref_1_1v_with_cap_at_aref_pin()   { ADMUX = 0b11001000; }
static void adc_enable_start_conversion__div_128()              { ADCSRA = 0b11000111; }
static void adc_enable_start_conversion__div_2()                { ADCSRA = 0b11000001; }
static void adc_wait_convertion()                               { while(ADCSRA & 0b01000000); }
static uint16_t adc_read_result_16()                            { return ADC; }
static void adc_src_gnd__ref_off()                              { ADMUX = 0b00001111; }
static void adc_disable__div_128()                              { ADCSRA = 0b00000111; }

static void adc_release()
{
    adc_src_gnd__ref_off();
    adc_disable__div_128();
}

static void f0_vcc_read(const char* descr)
{
    adc_set_src_1_1v__ref_avcc_with_cap_at_aref_pin();
    adc_enable_start_conversion__div_2();
    adc_wait_convertion();
    float vcc = adc_read_result_16();
    uint8_t cnt = 0xFF;
    while(cnt --) {
        adc_enable_start_conversion__div_2();
        adc_wait_convertion();
        vcc += adc_read_result_16();
        vcc /= 2.0;
    }
    adc_release();
    fprintf(&uart_str, "%s: %u mV\r\n", descr, (uint16_t)(1100.0 * 1023.0 / vcc));
}

static void f0_gpio_set(const char* descr)
{
    DDRC = 0b00000001;
    PORTC = 0b00000001;
    fprintf(&uart_str, "%s\r\n", descr);
}

static void f0_gpio_unset(const char* descr)
{
    DDRC = 0b00000001;
    PORTC = 0b00000000;
    fprintf(&uart_str, "%s\r\n", descr);
}

static uint16_t adc_warmup_wait_read()
{
    adc_enable_start_conversion__div_2();
    adc_wait_convertion();
    adc_enable_start_conversion__div_2();
    adc_wait_convertion();
    return adc_read_result_16();
}

static uint16_t adc_wait_read_128()
{
    adc_enable_start_conversion__div_2();
    adc_wait_convertion();
    uint16_t val = adc_read_result_16();
    uint8_t cnt = 128;
    while(cnt--) {
        adc_enable_start_conversion__div_2();
        adc_wait_convertion();
        val += adc_read_result_16();
        val /= 2;
    }
    return val;
}

static void f0_adc_read(const char* descr)
{
    DDRC = 0b00000000;
    adc_set_src_adc0__ref_vcc_with_cap_at_aref_pin();
    uint16_t val = adc_warmup_wait_read();
    adc_release();
    fprintf(&uart_str, "%s: %u\r\n", descr, val);
}

static void f0_temp_read(const char* descr)
{
    adc_set_src_temp__ref_1_1v_with_cap_at_aref_pin();
    uint16_t val = adc_warmup_wait_read();
    adc_release();
    fprintf(&uart_str, "%s: %u\r\n", descr, val);
}

static void f0_gpio_time(const char* descr)
{
    DDRD = 0b00010000;
    PORTD = 0b00000000;
    DDRD = 0b00000000;
//    PORTD = 0b00010000;
    PORTD = 0b00000000;
    uint8_t i = 255;
    while(i-- && !(PIND & 0b00010000));

    fprintf(&uart_str, "%s %d\r\n", descr, i);
//    fprintf(&uart_str, "%s %d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d\r\n", descr
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000)
//            , !!(PIND & 0b00010000));
}

static void f0_cpu_clock_test(const char* descr)
{
    fprintf(&uart_str, "%s\r\n", descr);
    fprintf(&uart_str, "CPU 31250 Hz 5s\r\n"); //3.3v0.15ma,5v9.9ma
    _delay_ms(5);
    cpu_clock_div_set(0b00001000);
    _delay_ms(5000 / 256);
    cpu_clock_div_set(0b00000000);
    fprintf(&uart_str, "CPU 128 KHz 5s\r\n"); //3.3v0.21ma,5v9.9ma
    _delay_ms(5);
    cpu_clock_div_set(0b00000110);
    _delay_ms(5000 / 64);
    cpu_clock_div_set(0b00000000);
    fprintf(&uart_str, "CPU 1 Mhz 5s\r\n"); //3.3v0.74ma,5v10.9ma
    _delay_ms(5);
    cpu_clock_div_set(0b00000011);
    _delay_ms(5000 / 8);
    cpu_clock_div_set(0b00000000);
    fprintf(&uart_str, "CPU 8 Mhz 5s\r\n"); //3.3v3.74ma,5v16.3ma
    _delay_ms(5000);
    fprintf(&uart_str, "CPU done\r\n\r\n");
}

static void charge_loop()
{
    uint16_t cnt = 0;
    while(1) {
        DDRC = 0b00000001;
        PORTC = 0b00000001;
        sleep_cpu();
        if(UCSR0A & (1 << RXC0)) {
            break;
        }
        PORTC = 0b00000000;
        DDRC = 0b00000000;
        _delay_ms(100);
        adc_set_src_adc0__ref_vcc_with_cap_at_aref_pin();
        uint16_t val = adc_wait_read_128();
        adc_release();
        if(val > 980) {
            fprintf(&uart_str, "%s %d: %u\r\n", __func__, cnt, val);
            break;
        }
        cnt ++;
    }
}

static void discharge_loop()
{
    uint16_t cnt = 0;
    while(1) {
        DDRC = 0b00000001;
        PORTC = 0b00000000;
        sleep_cpu();
        if(UCSR0A & (1 << RXC0))
            break;
        PORTC = 0b00000000;
        DDRC = 0b00000000;
        _delay_ms(100);
        adc_set_src_adc0__ref_vcc_with_cap_at_aref_pin();
        uint16_t val = adc_wait_read_128();
        adc_release();
        if(val < 10) {
            fprintf(&uart_str, "%s %d: %u\r\n", __func__, cnt, val);
            break;
        }
        cnt ++;
    }
}

static void f0_cap_train(const char* descr)
{
    int cnt = 0;
    fprintf(&uart_str, "Start %s\r\n", descr);
    while(1) {
        discharge_loop();
        if(UCSR0A & (1 << RXC0))
            break;
        charge_loop();
        if(UCSR0A & (1 << RXC0))
            break;
        fprintf(&uart_str, "%s %d\r\n", descr, cnt);
        cnt ++;
    }
    adc_release();
    PORTC = 0b00000000;
    DDRC = 0b00000000;
    fprintf(&uart_str, "End %s %d\r\n", descr, cnt);
}

static void twi_init_400khz()
{
    TWSR = 0x00;
    TWBR = 0x0C;
    TWCR = (1 << TWEN);
}

static void twi_wait_complete()
{
    while(!(TWCR & (1 << TWINT)));
}

static void twi_send_start()
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    twi_wait_complete();
}

static void twi_send_stop()
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

static void twi_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    twi_wait_complete();
}

static uint8_t twi_read_ack()
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    twi_wait_complete();
    return TWDR;
}

static uint8_t twi_read_nack()
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    twi_wait_complete();
    return TWDR;
}

static uint8_t twi_get_status()
{
    return TWSR & 0xF8;
}

static uint8_t i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
    uint8_t res = 1;
    twi_send_start();

    if(TW_START != twi_get_status())
        return res;

    twi_write(dev_addr);
    if(TW_MT_SLA_ACK != twi_get_status())
        goto stop;

    twi_write(reg_addr);
    if(TW_MT_DATA_ACK != twi_get_status())
        goto stop;

    twi_send_start();
    if(TW_REP_START != twi_get_status())
        goto stop;

    twi_write(dev_addr | 1);
    if(TW_MR_SLA_ACK != twi_get_status())
        goto stop;

    while(-- len) {
        *data ++ = twi_read_ack();
        if(TW_MR_DATA_ACK != twi_get_status()) {
            goto stop;
        }
    }

    *data = twi_read_nack();
    if(TW_MR_DATA_NACK != twi_get_status())
        goto stop;

    res = 0;

stop:
    twi_send_stop();
    return res;
}

static uint8_t i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, uint8_t len)
{
    uint8_t res = 1;
    twi_send_start();

    if(TW_START != twi_get_status())
        return res;

    twi_write(dev_addr);
    if(TW_MT_SLA_ACK != twi_get_status())
        goto stop;

    twi_write(reg_addr);
    if(TW_MT_DATA_ACK != twi_get_status())
        goto stop;

    while(len --) {
        twi_write(*data ++);
        if(TW_MT_DATA_ACK != twi_get_status()) {
            goto stop;
        }
    }

    res = 0;

stop:
    twi_send_stop();
    return res;
}

#define LM75_ADDR       0x90

static void f0_lm75_read(const char* descr)
{
    uint8_t buff[2];
    uint8_t cfg = 0b00000000; //wake up
    if(i2c_write_reg(LM75_ADDR, 1, &cfg, sizeof(cfg))) {
        fprintf(&uart_str, "Error wakeup\r\n");
        return;
    }
//    _delay_ms(500);
    if(i2c_read_reg(LM75_ADDR, 0, buff, sizeof(buff))) {
        fprintf(&uart_str, "Error reading temperature\r\n");
        return;
    }
//    cfg = 0b00000001; //shut down
//    if(i2c_write_reg(LM75_ADDR, 1, &cfg, sizeof(cfg))) {
//        fprintf(&uart_str, "Error shutdown\r\n");
//        return;
//    }

    int16_t temp = buff[0] << 8 | buff[1];
    fprintf(&uart_str, "%s: %d\r\n", descr, temp / 256);
}

static void f1_si4432_transmit(const char* descr)
{
    spi_print_reg(0x00);
    spi_print_reg(0x01);
    spi_print_reg(0x02);
}

struct MENU_ITEM {
    char* descr;
    void (*proc)();
};

static uint8_t sys_wait_key_press()
{
    do {
        sleep_cpu();
    }while(!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

static void menu(const char* title, struct MENU_ITEM item_arr[], uint8_t count)
{
    fprintf(&uart_str, "\r\n%s.\r\n", title);
    while(1) {
        uint8_t ch = sys_wait_key_press();
        if('q' == ch)
            break;

        if(ch < 'a' || ch >= 'a' + count) {
            fprintf(&uart_str, "\r\n%s:\r\n", title);
            fprintf(&uart_str, "q: leave %s\r\n", title);

            for(uint8_t i = 0; i < count; i ++) {
                fprintf(&uart_str, "%c: %s\r\n", 'a' + i, item_arr[i].descr);
            }
            fprintf(&uart_str, "\r\n");
            continue;
        }

        ch -= 'a';
        item_arr[ch].proc(item_arr[ch].descr);
    }
    fprintf(&uart_str, "\r\nLeft %s.\r\n", title);
}

static void f0_level_1(const char* descr)
{
    struct MENU_ITEM f1_menu[] = {
        {"Si4432 transmit", f1_si4432_transmit},
    };
    menu(descr, f1_menu, sizeof(f1_menu) / sizeof(f1_menu[0]));
}

int main(void)
{
    struct MENU_ITEM f0_menu[] = {
        {"VCC read",    f0_vcc_read},
        {"GPIO set",    f0_gpio_set},
        {"GPIO unset",  f0_gpio_unset},
        {"ADC read",    f0_adc_read},
        {"Temp. read",  f0_temp_read},
        {"GPIO dU/dT",  f0_gpio_time},
        {"Clock test",  f0_cpu_clock_test},
        {"Cap train",   f0_cap_train},
        {"LM75 read",   f0_lm75_read},
        {"Level 1",     f0_level_1},
    };

    sys_init();

    while(1)
        menu("Level 0", f0_menu, sizeof(f0_menu) / sizeof(f0_menu[0]));

    return 0;
}

