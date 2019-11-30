#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define F_CPU 8000000UL
#include <util/delay.h>


int main (void) {

    /**
     * We will be using OCR1A as our PWM output which is the
     * same pin as PD6.
     */
    DDRD |= _BV(PD6) | _BV(PD5) | _BV(PB1) | _BV(PB2);

    /**
     * There are quite a number of PWM modes available but for the
     * sake of simplicity we'll just use the 8-bit Fast PWM mode.
     * This is done by setting the WGM00 and WGM01 bits.  The 
     * Setting COM0A1 tells the microcontroller to set the 
     * output of the OCR0A pin low when the timer's counter reaches
     * a compare value (which will be explained below).  CS00 being
     * set simply turns the timer on without a prescaler (so at full
     * speed).  The timer is used to determine when the PWM pin should be
     * on and when it should be off.
     */
    TCCR0A |= _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
    TCCR0B |= _BV(CS00);

    /**
     *  This loop is used to change the value in the OCR0A register.
     *  What that means is we're telling the timer waveform generator
     *  the point when it should change the state of the PWM pin.
     *  The way we configured it (with _BV(COM0A1) above) tells the
     *  generator to have the pin be on when the timer is at zero and then
     *  to turn it off once it reaches the value in the OCR0A register.
     *
     *  Given that we are using an 8-bit mode the timer will reset to zero
     *  after it reaches 0xff, so we have 255 ticks of the timer until it
     *  resets.  The value stored in OCR0A is the point within those 255
     *  ticks of the timer when the output pin should be turned off
     *  (remember, it starts on).
     *
     *  Effectively this means that the ratio of pwm / 255 is the percentage
     *  of time that the pin will be high.  Given this it isn't too hard
     *  to see what when the pwm value is at 0x00 the LED will be off
     *  and when it is 0xff the LED will be at its brightest.
     */
	int8_t s1 = 0, s2 = 0, s3 = 0, s4 = 0;
	int8_t c1 = 120, c2 = 120, c3 = 120, c4 = 120;
	while(1) {
		s1 += c1 / 10;
		c1 -= s1 / 10;
		s2 += c2 / 12;
		c2 -= s2 / 12;
		s3 += c3 / 20;
		c3 -= s3 / 20;
		s4 += c4 / 21;
		c4 -= s4 / 21;
		OCR0A = ((s1 + s2 + s2 + s4) >> 2) + 127;
		OCR0B = ((c1 + c2 + c2 + c4) >> 2) + 127;
        _delay_ms(5);
    }

    return 0;
}

