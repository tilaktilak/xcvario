#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "softuart.h"
#include <avr/io.h>
#include <stdio.h>

#ifndef F_CPU
#define F_CPU 800000000UL
#endif

#ifndef BAUD
#define BAUD 9600
#endif
#include <util/setbaud.h>

/* http://www.cs.mun.ca/~rod/Winter2007/4723/notes/serial/serial.html */
void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */ 
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */    
}

void uart_putchar(char c, FILE *stream) {
    if (c == '\n') {
        uart_putchar('\r', stream);
    }
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
}

char uart_getchar(FILE *stream) {
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}
FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);



int main(void)
{
	char c;
	static const char pstring[] PROGMEM = 
		"adapted for Atmel AVR and this demo by Martin Thomas\r\n";

    uart_init();
	softuart_init();
	softuart_turn_rx_on(); /* redundant - on by default */
	
	sei();

	
    stdout = &uart_output;
	for (;;) {
	
		if ( softuart_kbhit() ) {
            c = softuart_getchar();
            putchar(c);
        }
	}
	
	return 0; /* never reached */
}