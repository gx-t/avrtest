#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define F_CPU 8000000UL
#include <util/delay.h>

/*
    Solar cell + ultracapacitor powered LED beacon

    ATTINY85
*/

static void gpio_init()
{
    DDRB = 0b00011011; //4 LEDs
    PORTB = 0b00100000; //pull-up of "reset"
}

static void wdt_init()
{
    wdt_reset();
    WDTCR = 0b00011000;
    WDTCR = 0b01001111;
}

ISR(WDT_vect)
{
}

static void wdt_sleep_2s()
{
    WDTCR = 0b00011000;
    WDTCR = 0b01001111;
    sleep_cpu();
}

static void wdt_sleep_64ms()
{
    WDTCR = 0b00011000;
    WDTCR = 0b01000011;
    sleep_cpu();
}

static void sys_init()
{
    cli();
    gpio_init();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    wdt_init();
    sei();
}

static void led_pulse(uint8_t mask)
{
    PORTB = mask;
    _delay_ms(2);
    PORTB = 0;
    wdt_sleep_64ms();
}

static void led_spark_if_dark()
{
//    if(PINB & 0b00000100)
//        return; //dark

    led_pulse(0b00000001);
    led_pulse(0b00000010);
    led_pulse(0b00001000);
    led_pulse(0b00010000);
    led_pulse(0b00000001);
}

int main(void) {
    sys_init();

    while(1) {
        led_spark_if_dark();
        wdt_sleep_2s();
    }
    return 0;
}

