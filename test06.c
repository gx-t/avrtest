#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define F_CPU 8000000UL
#include <util/delay.h>

/*
   ATMEGA 328P
   Vcc -- 0.068 uF ceramic -- Gnd
   RTC 9 -- 32768 Hz QZ -- 10
   9 -- 32768HZ -- 10
   23 -- button -- Gnd
 */

///////////////////////////////////////////////////////////////////////////////
//LCD

// RST, RST, CE, DC, DIN, CLK, BTN

static void lcd_write_cmd(uint8_t cmd)
{
    uint8_t i = 8;
    while(i --) {
        if(cmd & (1 << i)) {
            PORTC = 0b1100101;
            PORTC = 0b1100111;
        }
        else {
            PORTC = 0b1100001;
            PORTC = 0b1100011;
        }
        PORTC = 0b1100001;
    }
}

static void lcd_write_data(uint8_t data)
{
    uint8_t i = 8;
    while(i --) {
        if(data & (1 << i)) {
            PORTC = 0b1101101;
            PORTC = 0b1101111;
        }
        else {
            PORTC = 0b1101001;
            PORTC = 0b1101011;
        }
        PORTC = 0b1101001;
    }
}

static void lcd_and_btn_init()
{
    // RST, RST, CE, DC, DIN, CLK, BTN
    uint8_t i, init_seq[] = {
        0x21, 0x13, 0x06, 0xC2, 0x20, 0x09, 0x80, 0x40, 0x08, 0x0C
    };
    DDRC = 0b111110;
    PORTC = 0b1000001; //reset
    _delay_ms(10);
    PORTC = 0b1100001;
    for(i = 0; i < sizeof(init_seq); i ++) {
        lcd_write_cmd(init_seq[i]);
    }
}

///////////////////////////////////////////////////////////////////////////////
//RTC

static void rtc_init(void)
{  
    TCCR2A = 0x00;  //overflow
    TCCR2B = 0x02;  //5 gives 1 sec. prescale 
    TIMSK2 = 0x01;  //enable timer2A overflow interrupt
    ASSR  = 0x20;   //enable asynchronous mode
}

///////////////////////////////////////////////////////////////////////////////
//Interrupts
ISR(TIMER2_OVF_vect)
{
}

///////////////////////////////////////////////////////////////////////////////
static char val[] = {'0', '0', '0', '0', '0', '0', '0', 0};

static void reset_val()
{
    char *p = val;
    while(*p) {
        *p ++ = '0';
    }
    TCNT2 = 0; //zero RTC counter
}

static void update_val()
{
    char* p = val + sizeof(val) - 2;
    while(p >= val) {
        if('9' >= ++(*p))
            break;
        *p -- = '0';
    }
}

static void p_val()
{
    p_line(val);
}

static void sys_init()
{
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    lcd_and_btn_init();
    rtc_init();
    reset_val();
    sei();
}

struct COLUMN {
    uint8_t x, hole_y;
};

struct BIRD {
    uint8_t y;
};

static void draw_cols(uint8_t x, uint8_t y, uint8_t* pix, struct COLUMN* col)
{
    if(col[0].hole_y == y) {
        if(col[0].x - 5 < x && col[0].x + 5 > x) {
            *pix = 0b10000001;
        }
    }
    else {
        if(col[0].x - 4 < x && col[0].x + 4 > x) {
            *pix = 0b11111111;
        }
    }

    if(col[1].hole_y == y) {
        if(col[1].x - 5 < x && col[1].x + 5 > x) {
            *pix = 0b10000001;
        }
    }
    else {
        if(col[1].x - 4 < x && col[1].x + 4 > x) {
            *pix = 0b11111111;
        }
    }
}

static void draw_bird(uint8_t x, uint8_t y, uint8_t* pix, struct BIRD* bird)
{
    union {
        uint16_t u16;
        uint8_t u8[2];
    } mask = {.u16 = 0b0000000000111100};

    if(y == bird->y >> 3) {
        if(6 < x && 14 > x) {
            mask.u16 <<= (bird->y % 8);
            *pix |= mask.u8[0];
        }
    }
    if(y - 1 == bird->y >> 3) {
        if(6 < x && 14 > x) {
            mask.u16 <<= (bird->y % 8);
            *pix |= mask.u8[1];
        }
    }
}

static void draw(struct COLUMN* col, struct BIRD* bird)
{
    uint8_t x, y, pix;
    for(y = 0; y < 6; y ++) {
        for(x = 0; x < 84; x ++) {
            pix = 0;
            draw_cols(x, y, &pix, col);
            draw_bird(x, y, &pix, bird);
            lcd_write_data(pix);
        }
    }
}

static uint8_t rand_6()
{
    static const uint8_t tbl[] = {0, 5, 6, 7, 1, 3, 2, 4};
    static uint8_t val = 0xFF;
    static uint8_t cnt = 0;
    val = tbl[(val ^ cnt ++) & 0b111];
    return 1 + (val >> 1);
}

static void update_scene(struct COLUMN* col, struct BIRD* bird)
{
    col[0].x--;
    if(col[0].x > 83) {
        col[0].x = 83;
        col[0].hole_y = rand_6();
    }

    col[1].x--;
    if(col[1].x > 83) {
        col[1].x = 83;
        col[1].hole_y = rand_6();
    }


    bird->y += PINC & 1 ? 1 : -2;
    if(bird->y > 41 + 4) {
        bird->y = 0;
    }
}

int main(void)
{
    struct COLUMN col[] = {[0].x = 83, [0].hole_y = 2, [1].x = 41, [1].hole_y = 2};
    struct BIRD bird = {.y = 16};
    sys_init();
    while(1) {
        draw(col, &bird);
        update_scene(col, &bird);
        sleep_cpu();
    } 
    return 0;
}

