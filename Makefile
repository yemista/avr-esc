all:
	avr-gcc  -std=c99 -Wall -Os -v -mmcu=attiny13 -o attiny-esc.out attiny13-esc.c
	avr-objcopy -O ihex attiny-esc.out attiny-esc.hex

test:
	avr-gcc  -std=c99 -Wall -Os -v -mmcu=attiny13 -o test.out test.c
	avr-objcopy -O ihex test.out test.hex

clean:
	rm *.out
	rm *.hex
