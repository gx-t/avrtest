#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define F_CPU 8000000UL
#include <util/delay.h>

/*
   ATMEGA 328P + Nokia5110 LCD
   PB0 -- LCD-RST
   PB1 -- LCD-DC
   PB2 -- LCD-CE
   PB3 -- LCD-DIN
   PB4 --
   PB5 -- LCD-CLK
   PB6, PB7 -- 32Khz QZ

   PC0, GND -- BTN, internal pull-up
   PC6(Reset), GND -- 0.01uF C internal pull-up

   VCC, GND -- 0.1uF C
 */

static void lcd_init()
{
    uint8_t i, init_seq[] = {
        0x21, 0x13, 0x06, 0xC2, 0x20, 0x09, 0x80, 0x40, 0x08, 0x0C
    };

    PORTB = 0b000000; //LCD-RST
    _delay_ms(10);
    PORTB = 0b000001; //LCD-DC low

    for(i = 0; i < sizeof(init_seq); i ++) {
        SPDR = init_seq[i]; //send to SPI
        while(!(SPSR & (1 << SPIF))); //wait for write end
    }
}

static void rtc_init(void)
{  
    TCCR2A = 0x00;  //overflow
    TCCR2B = 0x02;  //5 gives 1 sec. prescale 
    TIMSK2 = 0x01;  //enable timer2A overflow interrupt
    ASSR  = 0x20;   //enable asynchronous mode
}

ISR(TIMER2_OVF_vect)
{
}

static void sys_init()
{
    DDRC    = 0b00000000;
    PORTC   = 0b01000001; //reset and button pull-up

    DDRB    = 0b00101111; //output CLK, MOSI, SS, LCD-DC, LCD-RST
    SPCR    = (1 << SPE) | (1 << MSTR) | (1 << SPI2X); //enable SPI-master, clock/2 speed

    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    lcd_init();
    rtc_init();
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
    PORTB = 0b000011; //LCD-CE on, LCD-DC high
    uint8_t x, y, pix;
    for(y = 0; y < 6; y ++) {
        for(x = 0; x < 84; x ++) {
            pix = 0;
            draw_cols(x, y, &pix, col);
            draw_bird(x, y, &pix, bird);

            while(!(SPSR & (1 << SPIF))); //wait for prev. write to end
            SPDR = pix; //send to SPI
        }
    }
    while(!(SPSR & (1 << SPIF))); //wait for last write to end
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


    bird->y += PINC & 1 ? 1 : -2; //check button press
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
//        PORTB = 0b000111; //LCD-CE off, LCD-DC high - same power consuption - 1.30ma x 3v
        update_scene(col, &bird);
        sleep_cpu();
    } 
    return 0;
}

