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

static void gpio_enable_reset_pullup()
{
    PORTC = 0b01000000;
}

static void uart_init()
{
    UBRR0 = USART_UBBR_VALUE;
    UCSR0A = 0;
    UCSR0B = (1 << TXEN0);
    //8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
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

#define BMP180_ADDRESS		0xEE

static void bmp180_read(/*int i2c_fd, float* tf, float* pf, float* hg, float* hf*/)
{
    uint8_t buff[32] = {0};

    //read calibration data

    if(i2c_read_reg(BMP180_ADDRESS, 0xAA, buff, 22)) {
        fprintf(&uart_str, "BMP180: error reading calibration data\r\n");
        return;
    }
//    fprintf(&uart_str, "-->> %d, %d ... %d\r\n", buff[0], buff[1], buff[14]);

    short ac1, ac2, ac3, b1, b2, mc, md;

    unsigned short ac4, ac5, ac6;
    ac1 = buff[0]  << 8 | buff[1];
    ac2 = buff[2]  << 8 | buff[3];
    ac3 = buff[4]  << 8 | buff[5];
    ac4 = buff[6]  << 8 | buff[7];
    ac5 = buff[8]  << 8 | buff[9];
    ac6 = buff[10] << 8 | buff[11];
    b1  = buff[12] << 8 | buff[13];
    b2  = buff[14] << 8 | buff[15];
    mc  = buff[18] << 8 | buff[19];
    md  = buff[20] << 8 | buff[21];

    //init temp. measurement
    buff[0] = 0x2E;
    if(i2c_write_reg(BMP180_ADDRESS, 0xF4, buff, 1)) {
        fprintf(&uart_str, "BMP180: cannot initiate temperature measurement\r\n");
        return;
    }
    //wait for ADC to complete measurement
    _delay_ms(5);
    if(i2c_read_reg(BMP180_ADDRESS, 0xF6, buff, 2)) {
        fprintf(&uart_str, "BMP180: cannot read temperature\r\n");
        return;
    }
    long ut = buff[0] << 8 | buff[1];

    fprintf(&uart_str, "BMP180: uncompensated temperature: %ld\r\n", ut);

    //init pressure. measurement
    buff[0] = 0xF4;
    if(i2c_write_reg(BMP180_ADDRESS, 0xF4, buff, 1)) {
        fprintf(&uart_str, "BMP180: cannot initiate pressure measurement\r\n");
        return;
    }
    //wait for ADC to complete measurement
    _delay_ms(26);
    if(i2c_read_reg(BMP180_ADDRESS, 0xF6, buff, 3)) {
        fprintf(&uart_str, "BMP180: cannot read pressure\r\n");
        return;
    }
    long up = ((long)buff[0] << 16 | (long)buff[1] << 8 | (long)buff[2]) >> 5;
    fprintf(&uart_str, "BMP180: uncompensated pressure: %ld\r\n", up);

    //calculate true temperature

    long x1, x2, x3, b3, b5, b6;
    unsigned long b4, b7;
    x1 = (ut - ac6) * ac5 >> 15;
    x2 = (mc << 11) / (x1 + md);
    b5 = x1 + x2;
    float t = (b5 + 8) >> 4;
    t /= 10;

    //calculate true pressure

    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;

    b3 = (((ac1 * 4 + x3) << 3) + 2) >> 2;
    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = (x1 + x2 + 2) >> 2;
    b4 = (ac4 * (x3 + 32768)) >> 15;
    b7 = (up - b3) * (50000 >> 3);

    long p;
    if(b7 < 0x80000000)
        p = (b7 << 1) / b4;
    else
        p = (b7 / b4) << 1;

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += ((x1 + x2 + 3791) >> 4);


    fprintf(&uart_str, "BMP180: t = %ld, p = %ld\r\n", (long)t, (long)p);
}

static void sys_init()
{
    cli();
    gpio_enable_reset_pullup();
    uart_init();
    //TODO: check this:
    twi_init_400khz();
    sei();
}

int main()
{
    fprintf(&uart_str, "-->> start\r\n");
    sys_init();
    while(1) {
        _delay_ms(1000);
        bmp180_read();
    }
    return 0;
}
