MAIN = main

FLASH.bin : $(MAIN).elf
	avr-objcopy -O binary $(MAIN).elf FLASH.bin

$(MAIN).elf : $(MAIN).o
	avr-gcc -mmcu=atmega32u4 $(MAIN).o -o $(MAIN).elf

$(MAIN).o : $(MAIN).c
	avr-gcc -O1 -mmcu=atmega32u4 -c $(MAIN).c -o $(MAIN).o -std=c99
