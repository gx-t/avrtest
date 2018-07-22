all:
	@echo "make testxx"
00:
	avr-gcc -mmcu=attiny2313 -Wall -Werror -Os -s test00.c -o test00.elf
	avr-objcopy -j .text -j .data -O ihex test00.elf test00.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test00.hex -U lfuse:w:0xe2:m -U hfuse:w:0xdb:m

01:
	avr-gcc -mmcu=attiny2313 -Wall -Werror -Os -s test01.c -o test01.elf
	avr-objcopy -j .text -j .data -O ihex test01.elf test01.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test01.hex -U lfuse:w:0xe6:m

02:
	avr-gcc -mmcu=attiny2313 -Wall -Werror -Os -s test02.c -o test02.elf
	avr-objcopy -j .text -j .data -O ihex test02.elf test02.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test02.hex -U lfuse:w:0xe6:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

03:
	avr-gcc -mmcu=attiny2313 -Wall -Werror -Os -s test03.c -o test03.elf
	avr-objcopy -j .text -j .data -O ihex test03.elf test03.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test03.hex -U lfuse:w:0xe6:m -U hfuse:w:0xdB:m -U efuse:w:0xff:m

04:
	avr-gcc -mmcu=attiny2313 -Wall -Werror -Os -s test04.c -o test04.elf
	avr-objcopy -j .text -j .data -O ihex test04.elf test04.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test04.hex -U lfuse:w:0xc4:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

05:
	avr-gcc -mmcu=atmega328p -Wall -Werror -Os -s test05.c -o test05.elf
	avr-objcopy -j .text -j .data -O ihex test05.elf test05.hex
	rm *.elf
	avrdude -c USBASP -p m328p -U flash:w:test05.hex -U lfuse:w:0xc2:m 

06:
	avr-gcc -mmcu=atmega328p -Wall -Werror -O2 -s test06.c -o test06.elf
	avr-objcopy -j .text -j .data -O ihex test06.elf test06.hex
	rm *.elf
	avrdude -c USBASP -p m328p -U flash:w:test06.hex -U lfuse:w:0xc2:m 

07:
	avr-gcc -mmcu=atmega328p -Wall -Werror -Os -s test07.c -o test07.elf
	avr-objcopy -j .text -j .data -O ihex test07.elf test07.hex
	rm *.elf
	avrdude -c USBASP -p m328p -U flash:w:test07.hex -U lfuse:w:0xc2:m 


client_rel:
	gcc -O2 -Werror -s client.c -o client

client_deb:
	gcc -g -Werror client.c -o client

tags: *.c
	ctags -R . /usr/lib/avr/include/

clean:
	rm -f *.o *.elf *.hex tags

