#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifndef F_CPU
#define F_CPU 800000000UL
#endif

#ifndef BAUD
#define BAUD 9600
#endif
#include <util/setbaud.h>

#include "i2c_master.h"
#include "bmp280.h"
#include "softuart.h"
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

int uart_putchar(char c, FILE *stream) {
    if (c == '\n') {
        uart_putchar('\r', stream);
    }
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

char uart_getchar(FILE *stream) {
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}
FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

#define TIMER_FREQ_HZ   1
/*
void timerRtc_init(void){

    TCCR1B = _BV(CS10) | _BV(CS11)  | _BV(WGM12); // prescaler=64, clear timer/counter on compareA match 
    OCR1A = ((F_CPU/2/64/TIMER_FREQ_HZ) - 1 );   
    // enable Output Compare 1 overflow interrupt
    TIMSK1  = _BV(OCIE1A);   
}

//uint8_t flag_period = 0;
uint64_t count_time = 0;
ISR(TIMER1_COMPA_vect)    // handler for Output Compare 1 overflow interrupt
{
    count_time ++;

    //flag_period = 1;
}*/

/*uint8_t get_flag_period(void){
    uint8_t result = 0;
    cli();
    result = flag_period;
    sei();
    return result;
}

void set_flag_period(uint8_t f){
    cli();
    flag_period = f;
    sei();
}*/


int main(void)
{
    //OCR1A = ((F_CPU/2/64/TIMER_FREQ_HZ) - 1 );   
    // enable Output Compare 1 overflow interrupt
    //TIMSK1  = _BV(OCIE1A);   
	//char c;
	//static const char pstring[] PROGMEM = 
	//	"adapted for Atmel AVR and this demo by Martin Thomas\r\n";

    uart_init();
	softuart_init();
	softuart_turn_rx_on(); /* redundant - on by default */  
    i2c_init();
    //timerRtc_init();
	
	sei();

	
    stdout = &uart_output;
    InitBMP280();
    float alt = 0.f;
    const float dt = powf(2,16)*(8.f/8E6);
    float smooth_alt = 0.f;
    float old_smooth_alt =0.f;
    float der_alt = 0.f;
    float smooth_der_alt = 0.f;
    float old_smooth_der_alt = 0.f;
    const float tau_alt = 0.7f;
    const float tau_der = 2.f;
    
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1C = 0;
    TCCR1B |=(1<<CS11);// | (1<<CS10);// (1<<CS10) | (1<<CS11);// prescaler=64 
    smooth_alt = AltitudeBMP280();// Init
    old_smooth_alt = smooth_alt;
    alt = smooth_alt;

	for (;;) {
        while((TIFR1 & (1<<TOV1))!=(1<<TOV1));// Wait until flag set
        TIFR1 |= (1 << TOV1);
        alt = AltitudeBMP280();
        smooth_alt = tau_alt*old_smooth_alt + (1.f - tau_alt)*alt;
        der_alt=(smooth_alt-old_smooth_alt)/dt;
        old_smooth_alt = smooth_alt;
        smooth_der_alt = ((1-tau_der)*old_smooth_der_alt + der_alt)/tau_der; 
        printf("%.2f %.2f\n",(double)(smooth_der_alt),(double)(smooth_alt));


	
		/*if ( softuart_kbhit() ) {
            c = softuart_getchar();
            putchar(c);
        }*/
	}	
	return 0; /* never reached */
}
