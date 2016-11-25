#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "softuart.h"
#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>

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

void timerRtc_init(void){
#define TIMER_FREQ_HZ   1 

    TCCR1B = _BV(CS10) | _BV(CS11)  | _BV(WGM12); // prescaler=64, clear timer/counter on compareA match 
    OCR1A = ((F_CPU/2/64/TIMER_FREQ_HZ) - 1 );   
    // enable Output Compare 1 overflow interrupt
    TIMSK1  = _BV(OCIE1A);   
}

uint64_t time_s = 0;
ISR(TIMER1_COMPA_vect)    // handler for Output Compare 1 overflow interrupt
{
    time_s = time_s ++;
}

uint64_t seconds(){
    uint64_t result;
    cli();
    result = time_s;
    sei();
    return result;
}


float alt2 = 0.f;
int i = 0;
extern float alt;
int main(void)
{
    int pres = 0;
	char c;
	static const char pstring[] PROGMEM = 
		"adapted for Atmel AVR and this demo by Martin Thomas\r\n";

    uart_init();
	//softuart_init();
	//softuart_turn_rx_on(); /* redundant - on by default */  
    i2c_init();
    //timerRtc_init();
	
	sei();

	
    stdout = &uart_output;
    printf("BMP : 0x%x\n",InitBMP280());
    while(1){
        for(i = 0; i<32000; i ++){
        }
        alt2 = AltitudeBMP280();
        //printf("%ld %ld\n",(int32_t)seconds(),(int32_t)(alt*100.));
        //printf("%ld\n",(int32_t)(alt));
        //printf("pres %d\n",pres);
    }

	for (;;) {
	
		if ( softuart_kbhit() ) {
            c = softuart_getchar();
            putchar(c);
        }
	}
	
	return 0; /* never reached */
}
