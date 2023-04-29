#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#define F_CPU 600000L
#include <util/delay.h>

#define UP_DOWN_PERIOD_MS   300

ISR(PCINT0_vect)
{
}

//ISR(WDT_vect)
//{
//}

static void wdt_init()
{
    wdt_reset();
    WDTCR   = 0b00011000;
    WDTCR   = 0b01001111;
}


static void adc_init()
{
    ADMUX   = 0b00100001; //Vcc as ref., left adjust, ADC1 (PB2)
    ADCSRA  = 0b11000011; //enable, start, clk/8
}

/*
static uint8_t adc_read()
{
    ADCSRA |= 0b01000000; //start conversion
    led_pulse();
    return ADCH; //led_pulse delay is enough for ADC to finish operation
}
*/
static void sys_init()
{
    cli();

    //enable clock prescaler change
    CLKPR   = (1<<CLKPCE);
    //8 times clock division
    CLKPR   = (1<<CLKPS1) | (1<<CLKPS0);

//    //PB0, PB1 - LEDs, PB4 - button
    DDRB    = (1<<DDB1) | (1<<DDB0);
    //pull-up PB4 (button), PB5 (reset)
    PORTB   = (1<<PB5) | (1<<PB4);

    //phase correct PWM on PB0, PB1
    TCCR0A  = (1<<COM0A1) | (1<<COM0B1) | (1<<WGM00);
    //no prescaler: clkI/O / 1
    TCCR0B  = (1<<CS00);

//    adc_init();

    //enable PCINT[0:5] pin change interrupt
    GIMSK   |= (1<<PCIE);
    //configure interrupt at PB4
    PCMSK   |= (1<<PCINT4);

//    wdt_init();

    sei();
}

//static uint8_t mode = 0;
//static uint8_t level = 0xFF;
//
//static uint8_t* eeprom_addr_mode = (uint8_t*)0;
//static uint8_t* eeprom_addr_level = (uint8_t*)1;

static uint8_t level = 1;
static int level_up_down()
{
    static uint8_t up = 1;
    while(!(PINB & (1<<PB4)))
    {
        if(up)
        {
            if(0xFF != level)
            {
                level <<= 1;
                level |= 1;
            }
        }
        else
        {
            if(1 != level)
            {
                level >>= 1;
            }
        }
        OCR0A = level;
        OCR0B = level;
        _delay_ms(500);
    }
    up = !up;
    return 1;
}

static void btn_wait_release_forever()
{
    while(!(PINB & (1<<PB4)))
        _delay_ms(100);
}

static uint8_t btn_wait_release_or_up_down()
{
    uint8_t cnt = 16;
    while(!(PINB & (1<<PB4)) && --cnt)
        _delay_ms(100);

    return cnt ? 0 : level_up_down();
}

static void light_off()
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    sleep_cpu();

    while(!(PINB & (1<<PB4)))
        _delay_ms(100);

    OCR0A = 0;
    OCR0B = 0;
}

static void light_on()
{
    OCR0A = level;
    OCR0B = level;

    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();

    do {
        sleep_cpu();
    } while(btn_wait_release_or_up_down());
}

int main()
{
    sys_init();
    OCR0A = 1; //pwm value 1 (for testing)
    OCR0B = 1; //pwm value 1 (for testing)

    while(1)
    {
        light_off();
        light_on();
    }

    return 0;
}

