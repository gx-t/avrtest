all:
	avr-gcc -mmcu=attiny2313 -Wall -Os -s test.c -o test.elf
	avr-objcopy -j .text -j .data -O ihex test.elf test.hex
	rm test.elf
	avrdude -c USBASP -p t2313 -U flash:w:test.hex -U lfuse:w:0xe6:m

tags:
	ctags -R . /usr/lib/avr/include/

