all:
	@echo "make testxx"
test00:
	avr-gcc -mmcu=attiny2313 -Werror -Os -s test00.c -o test00.elf
	avr-objcopy -j .text -j .data -O ihex test00.elf test00.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test00.hex -U lfuse:w:0xe2:m -U hfuse:w:0xdb:m

test01:
	avr-gcc -mmcu=attiny2313 -Werror -Os -s test01.c -o test01.elf
	avr-objcopy -j .text -j .data -O ihex test01.elf test01.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test01.hex -U lfuse:w:0xe6:m

test02:
	avr-gcc -mmcu=attiny2313 -Werror -Os -s test02.c -o test02.elf
	avr-objcopy -j .text -j .data -O ihex test02.elf test02.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test02.hex -U lfuse:w:0xe6:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

tags:
	ctags -R . /usr/lib/avr/include/

clean:
	rm -f *.o *.elf *.hex tags

