#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#define F_CPU 600000L
#include <util/delay.h>

/*
   • ATTINY13
   • Clock 600Khz (osc. 4.8Mhz)
   • 1 PWM, phase correct mode
   • 24v LED control
   • 2 paralell AO3400 NMOS
   • Circuit diagram, PCB: https://oshwlab.com/shah32768/led-24v-light-control
 */

static void cpu_clock_div_8()
{
    CLKPR = 0b10000000;
    CLKPR = 0b00000011;
}

static void gpio_init()
{
    DDRB    = 0b001001; //PB0 - gates, PB2 - sensor, PB3 - LED, PB4 - button
    PORTB   = 0b110100; //pull-up PB2 (sensor), PB4 (button), PB5 (reset)
}

static void pwm_init()
{
    TCCR0A = 0b10000001; //phase correct PWM on PB0
    TCCR0B = 0b00000001; //no prescaler: clkI/O / 1
}

static void led_pulse()
{
    PORTB |= 0b001000;
    _delay_ms(10);
    PORTB &= ~0b001000;
    _delay_ms(100);
}

static void adc_init()
{
    ADMUX = 0b00100001; //Vcc as ref., left adjust, ADC1 (PB2)
    ADCSRA = 0b11000011; //enable, start, clk/8
    led_pulse(); //no need to wait for ADC, this delay is enough
}

static uint8_t adc_read()
{
    ADCSRA |= 0b01000000; //start conversion
    led_pulse();
    return ADCH; //no need to wait for finish - delay if led_pulse is enough
}

static void sys_init()
{
    cli();
    cpu_clock_div_8();
    gpio_init();
    pwm_init();
    adc_init();
    sei();
}

static uint8_t mode = 0;
static uint8_t level = 0xFF;

static uint8_t* eeprom_addr_mode = (uint8_t*)0;
static uint8_t* eeprom_addr_level = (uint8_t*)1;

static uint8_t btn_state()
{
    return 0b010000 & PINB; //press - low, release - high
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
    uint8_t threshold = OCR0A ? 64 : 128; //off at 22/5 lx
    OCR0A = adc_read() > threshold ? level : 0;
}

#define TIMEOUT_STEP_MS     5
#define UP_DOWN_PERIOD_MS   200

static void light_6min()
{
    uint8_t cnt1 = 0;
    OCR0A = level;
    while(!btn_state());
    _delay_ms(100);

    while(btn_state() && ++cnt1)
    {
        uint8_t cnt2 = 0;
        led_pulse();
        led_pulse();
        while(btn_state() && ++cnt2)
            _delay_ms(TIMEOUT_STEP_MS);
    }
    OCR0A = 0;
    _delay_ms(100);
    while(!btn_state());
    _delay_ms(100);
}

static void level_up_down()
{
    static uint8_t up = 0;
    while(!btn_state())
    {
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
        _delay_ms(UP_DOWN_PERIOD_MS);
    }
    up = !up;
    eeprom_write_byte(eeprom_addr_level, level);
}

static void light_control(uint8_t m0, uint8_t m1, void (*op)(), void (*lp_op)())
{
    if(m0 != mode)
        return;
    eeprom_write_byte(eeprom_addr_mode, mode);
    while(1)
    {
        uint8_t cnt = 0;
        while(btn_state())
        {
            if(!cnt)
                op();
            cnt++;
            _delay_ms(TIMEOUT_STEP_MS);
        }
        cnt = 0;
        while(!btn_state() && ++cnt)
            _delay_ms(TIMEOUT_STEP_MS);
        if(cnt)
            break;
        lp_op();
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
        light_control(0, 1, light_off, light_6min);
        light_control(1, 2, light_set, level_up_down);
        light_control(2, 0, light_auto, level_up_down);
    }

    return 0;
}

