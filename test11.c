#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#define F_CPU 1200000UL
#include <util/delay.h>

/*
    Solar cell + ultracapacitor powered LED beacon

    ATTINY85
    PB0 - LED0
    PB1 - LED1
    PB3 - LED2
    PB4 - LED3

    VCC, GND 2.2uF 0805
    VCC - 1N5817 - 4.7V zener - GND

    Solar - GND, 4.7V zener

    Sleep, WDT on 5V 9uA,3.3V 4.4uA

    Loop 5V: 8Mhz-10ma, 1Mhz-3.8ma, 128Khz-2,7ma, 31Khz-2.6ma
    Loop 3.3V: 8Mhz-4.7ma, 1Mhz-1.0ma, 128Kh-0.43ma, 31Khz-0.38ma
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

static void cpu_clock_set_1mhz()
{
    CLKPR = 0b10000000;
    CLKPR = 0b00000011;
}

static void sys_init()
{
    cli();
    cpu_clock_set_1mhz();
    gpio_init();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    wdt_init();
    sei();
}

static void led_pulse(uint8_t mask)
{
    PORTB = mask;
    _delay_ms(4);
    PORTB = 0;
    wdt_sleep_64ms();
}

static void led_spark_left()
{
    led_pulse(0b00000001);
    led_pulse(0b00000010);
    led_pulse(0b00001000);
    led_pulse(0b00010000);
    led_pulse(0b00000001);
}

static void led_spark_right()
{
    led_pulse(0b00010000);
    led_pulse(0b00001000);
    led_pulse(0b00000010);
    led_pulse(0b00000001);
    led_pulse(0b00010000);
}

int main(void) {
    sys_init();

    while(1) {
        led_spark_left();
        wdt_sleep_2s();
        led_spark_right();
        wdt_sleep_2s();
    }
    return 0;
}

