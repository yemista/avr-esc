#ifndef F_CPU
#define F_CPU 960000UL 
#endif
 
#include <avr/io.h>
#include <util/delay.h> 

int main() {
	DDRB = (1 << PB1);

	while (1) {
		PORTB = (1 << PB1);
		_delay_ms(1000);
		PORTB = 0;
		_delay_ms(1000);
	}
}