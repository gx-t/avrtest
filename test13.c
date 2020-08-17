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

static uint8_t si4432_init()
{
    si4432_disable();
    _delay_ms(20);
    si4432_enable();
    _delay_ms(100); //20 ms is not enough!

    //read interrupt status registers to clear the interrupt flags and release NIRQ pin
    spi_read_reg(0x03); //read the Interrupt Status1 register
    spi_read_reg(0x04); //read the Interrupt Status2 register

    //SW reset   
    spi_write_reg(0x07, 0b10000000);

    while(PINB & 0b10);

    //read interrupt status registers to clear the interrupt flags and release NIRQ pin
    spi_read_reg(0x03); //read the Interrupt Status1 register
    spi_read_reg(0x04); //read the Interrupt Status2 register

    //set the center frequency to 915 MHz
    spi_write_reg(0x75, 0x75);        
    spi_write_reg(0x76, 0xBB);
    spi_write_reg(0x77, 0x80);

    //set the desired TX data rate (9.6kbps)
    spi_write_reg(0x6E, 0x4E);
    spi_write_reg(0x6F, 0xA5);
    spi_write_reg(0x70, 0x2C);

    //set the desired TX deviatioin (+-45 kHz)
    spi_write_reg(0x72, 0x48);

    //set the preamble length to 5bytes  
    spi_write_reg(0x34, 0x0A);

    //Disable header bytes; set variable packet length (the length of the payload is defined by the
    //received packet length field of the packet); set the synch word to two bytes long
    spi_write_reg(0x33, 0x02 );

    //Set the sync word pattern to 0x2DD4
    spi_write_reg(0x36, 0x2D);
    spi_write_reg(0x37, 0xD4);

	//enable the TX packet handler and CRC-16 (IBM) check
	spi_write_reg(0x30, 0x0D);
	//enable FIFO mode and GFSK modulation
	spi_write_reg(0x71, 0x63);

    /*Set the GPIO's to control the RF switch*/
    spi_write_reg(0x0C, 0x12);
    spi_write_reg(0x0D, 0x15);

							/*set the non-default Si4432 registers*/
	//set VCO and PLL
	spi_write_reg(0x5A, 0x7F);
	spi_write_reg(0x59, 0x40);

	//set Crystal Oscillator Load Capacitance register
	spi_write_reg(0x09, 0xD7);

    fprintf(&uart_str, "%s\r\n", __func__);
    return 0;
}

static void si4432_tx()
{
    fprintf(&uart_str, "TX start\r\n");
    led_on();
    /*SET THE CONTENT OF THE PACKET*/
    //set the length of the payload to 8bytes	
    spi_write_reg(0x3E, 8);
    //fill the payload into the transmit FIFO
    spi_write_reg(0x7F, 0x42);
    spi_write_reg(0x7F, 0x55);
    spi_write_reg(0x7F, 0x54);
    spi_write_reg(0x7F, 0x54);
    spi_write_reg(0x7F, 0x4F);
    spi_write_reg(0x7F, 0x4E);
    spi_write_reg(0x7F, 0x31);
    spi_write_reg(0x7F, 0x0D);

    //Disable all other interrupts and enable the packet sent interrupt only.
    //This will be used for indicating the successfull packet transmission for the MCU
    spi_write_reg(0x05, 0x04);
    spi_write_reg(0x06, 0x00);
    //Read interrupt status regsiters. It clear all pending interrupts and the nIRQ pin goes back to high.
    spi_read_reg(0x03);
    spi_read_reg(0x04);

    /*enable transmitter*/
    //The radio forms the packet and send it automatically.
    spi_write_reg(0x07, 0x09);

    //enable the packet sent interupt only
    spi_write_reg(0x05, 0x04);
    //read interrupt status to clear the interrupt flags
    spi_read_reg(0x03);
    spi_read_reg(0x04);

    /*wait for the packet sent interrupt*/
    //The MCU just needs to wait for the 'ipksent' interrupt.
    while(PINB & 0x10);
    //read interrupt status registers to release the interrupt flags
    spi_read_reg(0x03);
    spi_read_reg(0x04);

    fprintf(&uart_str, "TX end\r\n");
    led_off();
}

static void si4432_set_freq_433_92_low_side()
{
    //fb[4:0] = 19 (430–439.9 MHz)
    //433.92 = fo = 8, fb[4:0] = 19, fc = 0x09CC, hbsel = 0;
    //sbsel = 1 low-side injection

    spi_write_reg(0x73, 0x08);
    spi_write_reg(0x75, 0b01010011);
    spi_write_reg(0x76, 0x09);
    spi_write_reg(0x77, 0xCC);
    spi_print_reg(0x07);
    spi_print_reg(0x73);
    spi_print_reg(0x75);
    spi_print_reg(0x76);
    spi_print_reg(0x77);
}

static void si4432_tx_on()
{
    spi_write_reg(0x07, 0b1000);
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
    si4432_init();
    while(1) {
        si4432_tx();
        _delay_ms(1000);
    }

    return 0;
}
