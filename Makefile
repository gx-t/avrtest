none:
	@echo "run make XX"
00:
	avr-gcc -mmcu=attiny2313 -Wall -Werror -Os -s test00.c -o test00.elf
	avr-objcopy -j .text -j .data -O ihex test00.elf test00.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test00.hex -U lfuse:w:0xe2:m -U hfuse:w:0xdb:m
	rm 00.hex

01:
	avr-gcc -mmcu=attiny2313 -Wall -Werror -Os -s test01.c -o test01.elf
	avr-objcopy -j .text -j .data -O ihex test01.elf test01.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test01.hex -U lfuse:w:0xe6:m
	rm 01.hex

02:
	avr-gcc -mmcu=attiny2313 -Wall -Werror -Os -s test02.c -o test02.elf
	avr-objcopy -j .text -j .data -O ihex test02.elf test02.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test02.hex -U lfuse:w:0xe6:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
	rm 02.hex

03:
	avr-gcc -mmcu=attiny2313 -Wall -Werror -Os -s test03.c -o test03.elf
	avr-objcopy -j .text -j .data -O ihex test03.elf test03.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test03.hex -U lfuse:w:0xe6:m -U hfuse:w:0xdB:m -U efuse:w:0xff:m
	rm 03.hex

04:
	avr-gcc -mmcu=attiny2313 -Wall -Werror -Os -s test04.c -o test04.elf
	avr-objcopy -j .text -j .data -O ihex test04.elf test04.hex
	rm *.elf
	avrdude -c USBASP -p t2313 -U flash:w:test04.hex -U lfuse:w:0xc4:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m
	rm 04.hex

05:
	avr-gcc -mmcu=atmega328p -Wall -Werror -Os -s test05.c -o test05.elf
	avr-objcopy -j .text -j .data -O ihex test05.elf test05.hex
	rm *.elf
	avrdude -c USBASP -p m328p -U flash:w:test05.hex  -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m 
	rm 05.hex

06:
	avr-gcc -mmcu=atmega328p -Wall -Werror -O2 -s test06.c -o test06.elf
	avr-objcopy -j .text -j .data -O ihex test06.elf test06.hex
	rm *.elf
	avrdude -c USBASP -p m328p -U flash:w:test06.hex -U lfuse:w:0xe2:m 
	rm 06.hex

07:
	avr-gcc -mmcu=atmega328p -Wno-unused-function -Wall -Werror -Os -s test07.c -o test07.elf
	avr-objcopy -j .text -j .data -O ihex test07.elf test07.hex
	rm *.elf
	avrdude -c USBASP -p m328p -U flash:w:test07.hex -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m 
	rm 07.hex

08:
	avr-gcc -mmcu=atmega328p -Wno-unused-function -Wall -Werror -Os -s test08.c -o test08.elf
	avr-objcopy -j .text -j .data -O ihex test08.elf test08.hex
	rm *.elf
	avrdude -c USBASP -p m328p -U flash:w:test08.hex -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m 
	rm 08.hex

09:
	avr-gcc -mmcu=atmega328p -Wno-unused-function -Wall -Werror -Os -s test09.c -o test09.elf
	avr-objcopy -j .text -j .data -O ihex test09.elf test09.hex
	rm *.elf
	avrdude -c USBASP -p m328p -U flash:w:test09.hex -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m 
	rm 09.hex

10:
	avr-gcc -mmcu=atmega328p -Wno-unused-function -Wall -Werror -Os -s test10.c -o test10.elf
	avr-objcopy -j .text -j .data -O ihex test10.elf test10.hex
	rm *.elf
	avrdude -c USBASP -p m328p -U flash:w:test10.hex -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
	rm 10.hex

11:
	avr-gcc -mmcu=attiny13 -Wall -Os -s 11-attiny13-led-solar-beacon.c -o test11.elf
	avr-objcopy -j .text -j .data -O ihex test11.elf test11.hex
	rm *.elf
	avrdude -c USBASP -p t13 -U flash:w:test11.hex -U lfuse:w:0x79:m -U hfuse:w:0xff:m
	rm 11.hex

12:
	avr-gcc -std=c99 -mmcu=atmega328p -Wno-unused-function -Wall -Werror -Os -s test12.c -o test12.elf
	avr-objcopy -j .text -j .data -O ihex test12.elf test12.hex
	rm *.elf
	avrdude -c USBASP -p m328p -U flash:w:test12.hex -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
	rm 12.hex

13:
	avr-gcc -std=c99 -mmcu=atmega328p -Wno-unused-function -Wall -Werror -Os -s test13.c -o test13.elf
	avr-objcopy -j .text -j .data -O ihex test13.elf test13.hex
	rm *.elf
	avrdude -c USBASP -p m328p -U flash:w:test13.hex -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
	rm 13.hex

14:
	avr-gcc -std=c99 -mmcu=atmega328p -Wno-unused-function -Wall -Werror -Os -s test14.c -o test14.elf
	avr-objcopy -j .text -j .data -O ihex test14.elf test14.hex
	rm *.elf
	avrdude -c USBASP -p m328p -U flash:w:test14.hex -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
	rm 14.hex

15:
	avr-gcc -mmcu=attiny13 -Wall -Os -s 15-attiny13-pwd-2-candle-effect.c -o 15.elf
	avr-objcopy -j .text -j .data -O ihex 15.elf 15.hex
	rm *.elf
	avrdude -c USBASP -p t13 -U flash:w:15.hex -U lfuse:w:0x79:m -U hfuse:w:0xff:m
	rm 15.hex

16:
	avr-gcc -mmcu=attiny13 -Wall -Wno-unused-function -Os -s 16-attiny13-24v-led-control.c -DLIGHT_AUTO_SUPPORT -o 16.elf
	avr-objcopy -j .text -j .data -O ihex 16.elf 16.hex
	rm *.elf
	avrdude -c USBASP -p t13 -U flash:w:16.hex -U lfuse:w:0x79:m -U hfuse:w:0xff:m -U eeprom:w:0x00,0xff:m
	rm 16.hex

16-no-light-auto:
	avr-gcc -mmcu=attiny13 -Wall -Wno-unused-function -Os -s 16-attiny13-24v-led-control.c -o 16.elf
	avr-objcopy -j .text -j .data -O ihex 16.elf 16.hex
	rm *.elf
	avrdude -c USBASP -p t13 -U flash:w:16.hex -U lfuse:w:0x79:m -U hfuse:w:0xff:m -U eeprom:w:0x00,0xff:m
	rm 16.hex

16-light-pulse:
	avr-gcc -mmcu=attiny13 -Wall -Wno-unused-function -Os -s 16-attiny13-24v-led-control.c -DLIGHT_PULSE_SUPPORT -o 16.elf
	avr-objcopy -j .text -j .data -O ihex 16.elf 16.hex
	rm *.elf
	avrdude -c USBASP -p t13 -U flash:w:16.hex -U lfuse:w:0x79:m -U hfuse:w:0xff:m -U eeprom:w:0x00,0xff:m
	rm 16.hex

16-light-auto-pulse:
	avr-gcc -mmcu=attiny13 -Wall -Wno-unused-function -Os -s 16-attiny13-24v-led-control.c -DLIGHT_AUTO_SUPPORT -DLIGHT_PULSE_SUPPORT -o 16.elf
	avr-objcopy -j .text -j .data -O ihex 16.elf 16.hex
	rm *.elf
	avrdude -c USBASP -p t13 -U flash:w:16.hex -U lfuse:w:0x79:m -U hfuse:w:0xff:m -U eeprom:w:0x00,0xff:m
	rm 16.hex

17:
	avr-gcc -mmcu=attiny13 -Wall -Os -s 17-attiny13-tc4420-12-220-simsin-inverter.c -o 17.elf
	avr-objcopy -j .text -j .data -O ihex 17.elf 17.hex
	rm *.elf
	avrdude -c USBASP -p t13 -U flash:w:17.hex -U lfuse:w:0x79:m -U hfuse:w:0xff:m
	rm 17.hex

client_rel:
	gcc -O2 -Werror -s client.c -o client

client_deb:
	gcc -g -Werror client.c -o client

tags: *.c
	ctags -R . /usr/lib/avr/include/

clean:
	rm -f *.o *.elf *.hex tags

