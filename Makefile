all:
	@echo "make testxx"
test00:
	avr-gcc -mmcu=attiny2313 -Wall -Os -s test00.c -o test00.elf
	avr-objcopy -j .text -j .data -O ihex test00.elf test00.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test00.hex -U lfuse:w:0xe6:m

tags:
	ctags -R . /usr/lib/avr/include/

clean:
	rm *.o *.elf *.hex tags

