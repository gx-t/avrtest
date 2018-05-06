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
    PORTC = 0b01000011; //reset and button pull-up
}

static uint8_t btn_get_state(uint8_t mask)
{
    return PINC & mask;
}

static void btn_wait_release(uint8_t mask)
{
    while(!btn_get_state(mask))
        sleep_cpu();
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

struct FRAGMENT {
    uint8_t x, y, val;
};

static void f_main_loop(void* obj, void (*update)(), void (*draw)())
{
    while(btn_get_state(2)) {
        struct FRAGMENT frag;
        for(frag.y = 0; frag.y < 6; frag.y ++) {
            for(frag.x = 0; frag.x < 84; frag.x ++) {
                frag.val = 0;
                draw(obj, &frag);
                spi_wait_write();
                spi_write_byte(frag.val);
            }
        }
        spi_wait_write();
        //        PORTB = 0b000111; //LCD-CE off, LCD-DC high - same power consuption - 1.30ma x 3v
        update(obj);
        sleep_cpu();
    } 
    btn_wait_release(2);
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

static void f_0_draw_1_col(const struct F_0_OBJ* obj, uint8_t index, struct FRAGMENT* frag)
{
    if(obj->col[index].hole_y == frag->y) {
        if(obj->col[index].x - 5 < frag->x && obj->col[index].x + 5 > frag->x) {
            frag->val = 0b10000001;
        }
    }
    else {
        if(obj->col[index].x - 4 < frag->x && obj->col[index].x + 4 > frag->x) {
            frag->val = 0b11111111;
        }
    }
}

static void f_0_draw_bird(const struct F_0_OBJ* obj, struct FRAGMENT* frag)
{
    union {
        uint16_t u16;
        uint8_t u8[2];
    } mask = {.u16 = 0b0000000000111100};

    if(frag->y == obj->bird.y >> 3) {
        if(6 < frag->x && 14 > frag->x) {
            mask.u16 <<= (obj->bird.y % 8);
            frag->val |= mask.u8[0];
        }
    }
    if(frag->y - 1 == obj->bird.y >> 3) {
        if(6 < frag->x && 14 > frag->x) {
            mask.u16 <<= (obj->bird.y % 8);
            frag->val |= mask.u8[1];
        }
    }
}

static void f_0_draw(const struct F_0_OBJ* obj, struct FRAGMENT* frag)
{
    f_0_draw_1_col(obj, 0, frag);
    f_0_draw_1_col(obj, 1, frag);
    f_0_draw_bird(obj, frag);
}

static void f_0_update_1_col(struct F_0_OBJ* obj, uint8_t index)
{
    obj->col[index].x--;
    if(obj->col[index].x > 83) {
        obj->col[index].x = 83;
        obj->col[index].hole_y = lib_rand_6();
    }
}

static void f_0_update_bird(struct F_0_OBJ* obj)
{
    obj->bird.y += btn_get_state(1) ? 1 : -2;
    if(obj->bird.y > 41 + 4) {
        obj->bird.y = 0;
    }
}

static void f_0_update(struct F_0_OBJ* obj)
{
    f_0_update_1_col(obj, 0);
    f_0_update_1_col(obj, 1);
    f_0_update_bird(obj);
}

static void f_0_main()
{
    struct F_0_OBJ obj = {
        .col[0].x = 83, .col[0].hole_y = 2, .col[1].x = 41, .col[1].hole_y = 2,
        .bird.y = 16
    };
    f_main_loop(&obj, f_0_update, f_0_draw);
}

int main(void)
{
    sys_init();
    while(1) {
        f_0_main();
    }
    return 0;
}

