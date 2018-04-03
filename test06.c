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

static uint8_t lib_rand_6()
{
    static const uint8_t tbl[] = {0, 5, 6, 7, 1, 3, 2, 4};
    static uint8_t val = 0xFF;
    static uint8_t cnt = 0;
    val = tbl[(val ^ cnt ++) & 0b111];
    return 1 + (val >> 1);
}

static void spi_init()
{
    DDRB = 0b00101111; //output CLK, MOSI, SS, LCD-DC, LCD-RST
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPI2X); //enable SPI-master, clock/2 speed
}

static void spi_wait_write()
{
    while(!(SPSR & (1 << SPIF))); //wait for write end
}

static void spi_write_byte(uint8_t data)
{
    SPDR = data;
}

static void btn_init()
{
    DDRC = 0b00000000;
    PORTC = 0b01000001; //reset and button pull-up
}

static btn_0_get_state()
{
    return PINC & 1;
}

static void lcd_init()
{
    uint8_t i, init_seq[] = {
        0x21, 0x13, 0x06, 0xC2, 0x20, 0x09, 0x80, 0x40, 0x08, 0x0C
    };

    PORTB = 0b000000; //LCD-RST
    _delay_ms(10);
    PORTB = 0b000001; //LCD-DC low

    for(i = 0; i < sizeof(init_seq); i ++) {
        spi_write_byte(init_seq[i]);
        spi_wait_write();
    }
}

static void lcd_set_data_mode()
{
    PORTB = 0b000011; //LCD-CE on, LCD-DC high
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
    btn_init();
    spi_init();
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    lcd_init();
    lcd_set_data_mode();
    rtc_init();
    sei();
}


//-----------------------------------------------------------------------------
//F_0 demo

struct F_0_OBJ {
    struct {
        uint8_t x, hole_y;
    }col[2];

    struct {
        uint8_t y;
    }bird;
};

static void f_0_draw_cols(const struct F_0_OBJ* obj, uint8_t* pix, uint8_t x, uint8_t y)
{
    if(obj->col[0].hole_y == y) {
        if(obj->col[0].x - 5 < x && obj->col[0].x + 5 > x) {
            *pix = 0b10000001;
        }
    }
    else {
        if(obj->col[0].x - 4 < x && obj->col[0].x + 4 > x) {
            *pix = 0b11111111;
        }
    }

    if(obj->col[1].hole_y == y) {
        if(obj->col[1].x - 5 < x && obj->col[1].x + 5 > x) {
            *pix = 0b10000001;
        }
    }
    else {
        if(obj->col[1].x - 4 < x && obj->col[1].x + 4 > x) {
            *pix = 0b11111111;
        }
    }
}

static void f_0_draw_bird(const struct F_0_OBJ* obj, uint8_t* pix, uint8_t x, uint8_t y)
{
    union {
        uint16_t u16;
        uint8_t u8[2];
    } mask = {.u16 = 0b0000000000111100};

    if(y == obj->bird.y >> 3) {
        if(6 < x && 14 > x) {
            mask.u16 <<= (obj->bird.y % 8);
            *pix |= mask.u8[0];
        }
    }
    if(y - 1 == obj->bird.y >> 3) {
        if(6 < x && 14 > x) {
            mask.u16 <<= (obj->bird.y % 8);
            *pix |= mask.u8[1];
        }
    }
}

static void f_0_draw(const struct F_0_OBJ* obj)
{
    uint8_t y;
    for(y = 0; y < 6; y ++) {
        uint8_t x;
        for(x = 0; x < 84; x ++) {
            uint8_t pix = 0;
            f_0_draw_cols(obj, &pix, x, y);
            f_0_draw_bird(obj, &pix, x, y);

            spi_wait_write();
            spi_write_byte(pix);
        }
    }
    spi_wait_write();
}

static void f_0_update(struct F_0_OBJ* obj)
{
    obj->col[0].x--;
    if(obj->col[0].x > 83) {
        obj->col[0].x = 83;
        obj->col[0].hole_y = lib_rand_6();
    }

    obj->col[1].x--;
    if(obj->col[1].x > 83) {
        obj->col[1].x = 83;
        obj->col[1].hole_y = lib_rand_6();
    }

    obj->bird.y += btn_0_get_state() ? 1 : -2;
    if(obj->bird.y > 41 + 4) {
        obj->bird.y = 0;
    }
}

static void f_0_main()
{
    struct F_0_OBJ obj = {
        .col[0].x = 83, .col[0].hole_y = 2, .col[1].x = 41, .col[1].hole_y = 2,
        .bird.y = 16
    };
    while(1) {
        f_0_draw(&obj);
//        PORTB = 0b000111; //LCD-CE off, LCD-DC high - same power consuption - 1.30ma x 3v
        f_0_update(&obj);
        sleep_cpu();
    } 
}

int main(void)
{
    sys_init();
    while(1) {
        f_0_main();
    }
    return 0;
}

