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

static uint8_t mode = 0;
static uint8_t level = 0xFF;
static uint8_t up = 0;

static const uint8_t* eeprom_addr_mode = (uint8_t*)0;
static const uint8_t* eeprom_addr_level = (uint8_t*)1;

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

static void light_off()
{
    OCR0A = 0;
}

static void light_set()
{
    OCR0A = level;
}

static void light_auto()
{
    OCR0A = PINB & 0b000100 ? level : 0;
}

#define TIMEOUT_STEP_MS     5
#define UP_DOWN_PERIOD_MS   200

static void light_control(uint8_t m0, uint8_t m1, void (*setup)(), void (*timeout)())
{
    led_pulse();
    if(m0 != mode)
        return;
    eeprom_write_byte(eeprom_addr_mode, mode);
    setup();

    while(1)
    {
        uint8_t cnt = 1;
        while(btn_state())
        {
            _delay_ms(TIMEOUT_STEP_MS);
            if(0 == cnt && timeout)
                timeout();
            cnt ++;
        }

        cnt = 1;
        while(!btn_state())
        {
            _delay_ms(TIMEOUT_STEP_MS);
            if(0 == cnt)
                break;
            cnt ++;
        }

        if(0 != cnt)
            break;

        while(!btn_state())
        {
            _delay_ms(UP_DOWN_PERIOD_MS);
            if(up)
            {
                if(0xFF == level)
                    led_pulse();
                else
                {
                    level <<= 1;
                    level |= 1;
                }
            }
            else
            {
                if(level == 1)
                    led_pulse();
                else
                    level >>= 1;
            }
            light_set();
        }
        setup();
        up = !up;
        eeprom_write_byte(eeprom_addr_level, level);
    }
    mode = m1;
}

int main()
{
    sys_init();
    mode = eeprom_read_byte(eeprom_addr_mode);
    level = eeprom_read_byte(eeprom_addr_level);

    while(1)
    {
        light_control(0, 1, light_off, 0);
        light_control(1, 2, light_set, 0);
        light_control(2, 0, light_auto, led_pulse);
    }

    return 0;
}

