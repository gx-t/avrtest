#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#define F_CPU 600000L
#include <util/delay.h>

/*
   • ATTINY13
   • Clock 600Khz (osc. 4.8Mhz)
   • 2 PWM, phase correct mode
   • 24v LED control
   • 2 paralell AO3400 NMOS
   • Circuit diagram:
   • PCB:
 */

static void cpu_clock_div_8()
{
    CLKPR = 0b10000000;
    CLKPR = 0b00000011;
}

static void gpio_init()
{
    DDRB    = 0b001001; //ouptut PB0 - gates, PB2 - sensor, PB3 - LED, PB4 - button
    PORTB   = 0b110100; //pull-up PB2 (sensor), PB4 (button), PB5 (reset)
}

static void pwm_init()
{
    TCCR0A = 0b10000001; //phase correct PWM on PB0
    TCCR0B = 0b00000001; //no prescaler: clkI/O / 1
}

static void sys_init()
{
    cli();
    cpu_clock_div_8();
    gpio_init();
    pwm_init();
    sei();
}

static uint8_t btn_state()
{
    return 0b010000 & PINB; //press - low, release - high
}

static void led_pulse()
{
    PORTB |= 0b001000;
    _delay_ms(10);
    PORTB &= ~0b001000;
    _delay_ms(100);
}

static uint8_t mode = 0;
struct
{
    uint8_t up      : 1;
    uint8_t dummy   : 5;
    uint8_t val     : 3;
} static level = {0, 0};

static void light_common(uint8_t m0, uint8_t m1, void (*set_proc)(), void (*timeout_proc)())
{
    led_pulse();
    if(m0 != mode)
        return;
    eeprom_write_byte((uint8_t*)0, mode);
    set_proc();

    while(1)
    {
        uint8_t cnt = 1;
        while(btn_state())
        {
            _delay_ms(5);
            if(0 == cnt && timeout_proc)
                timeout_proc();
            cnt ++;
        }

        cnt = 1;
        while(!btn_state())
        {
            _delay_ms(5);
            if(0 == cnt)
                break;
            cnt ++;
        }

        if(0 != cnt)
            break;

        while(!btn_state())
        {
            _delay_ms(200);
            level.up ? (0 == level.val ? led_pulse() : level.val --) : (7 == level.val ? led_pulse() : level.val ++);
            set_proc();
        }
        level.up = !level.up;
        eeprom_write_byte((uint8_t*)1, *(uint8_t*)&level);
    }
    _delay_ms(100);
    mode = m1;
}

static void light_off()
{
    OCR0A = 0;
}

static void light_on()
{
    OCR0A = 255;
}

static void light_auto()
{
    OCR0A = PINB & 0b000100 ? (0xFF >> level.val) : 0;
}

int main()
{
    sys_init();
    mode = eeprom_read_byte((uint8_t*)0);
    *((uint8_t*)&level) = eeprom_read_byte((uint8_t*)1);

    while(1)
    {
        light_common(0, 1, light_off, 0);
        light_common(1, 2, light_on, 0);
        light_common(2, 0, light_auto, led_pulse);
    }

    return 0;
}

