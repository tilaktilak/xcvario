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
void timer1_init(void){
    // TIMER 1 Config for timing while 1
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1C = 0;
    TCCR1B |=(1<<CS11);//prescaler=8 
}

void timer2_init(void){
    // TIMER 2 Config for PWM tone
    TCCR2A = (1<<WGM21) | (1<<WGM20);
    TCCR2B = (1<<CS20) | (0<<CS21) | (1<<CS22) |(1<<WGM22);
    OCR2A = 0x00;
    TIMSK2 = (1<<OCIE2A);

    // Init A3 pin (PC3)
    DDRC |= (1<<DDC3);
    PORTC |= (1<<PORTC3);
}
// Fmin = 250Hz
void timer2_set_freq(float freq){
    cli();
    OCR2A = (uint8_t)((1./freq)*8E6/128.f);
    // PERIOD = 1s -> 8E6/128f
    // PERIOD = 0.5s -> 
    sei();
}

volatile int32_t duration = 0;
volatile int32_t tim2_period = 0;//200;
volatile uint8_t mute = 0;

void timer2_set_duration(float d_sec){
    cli();
    duration = (int)(d_sec/2.f*(8E6/128.f)/(float)OCR2A);
    tim2_period = duration;//Launch new period
    sei();
}

int main(void)
{
	//char c;

    uart_init();
	softuart_init();
	softuart_turn_rx_on(); /* redundant - on by default */  
    i2c_init();
	
	sei();

	
    stdout = &uart_output;

    printf("%d\n",InitBMP280());
    float alt = 0.f;
    const float dt = powf(2,16)*(8.f/8E6);
    float smooth_alt = 0.f;
    float old_smooth_alt =0.f;
    float der_alt = 0.f;
    float smooth_der_alt = 0.0f;
    float old_smooth_der_alt = 0.f;
    const float tau_alt = 0.7f;
    const float tau_der = 0.8f;
    float freq = 300.f;
    float const low_level = -0.5f;
    float const high_level = 0.5f;
    float const low_gain = -50.f;
    float const low_offset = 300.f;
    float const high_gain = 150.f;
    float const high_offset = 1100.f;

    int const der_count_max = 3;
    int der_count = 0;

    int const prs_count_max = 10;
    int prs_count = 0;

    int const freq_count_max = 10;
    int freq_count = 0;
   
    timer1_init();

    timer2_init();
    timer2_set_freq(250.f);
    timer2_set_duration(3.f);
    
    smooth_alt = AltitudeBMP280();// Init
    old_smooth_alt = smooth_alt;
    alt = smooth_alt;

	for (;;) {
        while((TIFR1 & (1<<TOV1))!=(1<<TOV1)){// Wait until flag set
            //if ( softuart_kbhit() ) {
            //    c = softuart_getchar();
                //putchar(c);
            //}
        }
        TIFR1 |= (1 << TOV1);
        alt = AltitudeBMP280();
        smooth_alt = tau_alt*old_smooth_alt + (1.f - tau_alt)*alt;
        if(++der_count>=der_count_max){
            der_count = 0;
            der_alt=(smooth_alt-old_smooth_alt)/dt;
            old_smooth_alt = smooth_alt;
            //smooth_der_alt = ((1-tau_der)*old_smooth_der_alt + der_alt)/tau_der; 
            smooth_der_alt = tau_der*old_smooth_der_alt + (1-tau_der)*der_alt;

        }

        if(++prs_count>=prs_count_max){
            prs_count = 0;
            //printf("PRS %5x\n",(int)PressureBMP280());
        }

        if(freq_count>=freq_count_max){
            if(smooth_der_alt < low_level){
                mute = 0;
                freq = low_gain*fabs(smooth_der_alt) + low_offset;
            }
            else if(smooth_der_alt > high_level){
                mute = 0;
                freq = smooth_der_alt * high_gain + high_offset;
            }
            else {
                mute = 1;// Mute
            }
            freq_count = 0;
            // Constraint freq
            if(freq<250.f)freq = 250.f;

            timer2_set_freq(freq);
        }

        printf("%.2f %.2f\n",(double)(smooth_der_alt),
                (double)(smooth_alt));






    }	
    return 0; /* never reached */
}


void toggle_PC3(void){
    if(!mute){
        if(PORTC&(1<<PORTC3)){
            PORTC &= (0<<PORTC3);
        }
        else{
            PORTC |= (1<<PORTC3);
        }
    }
}

ISR(TIMER2_COMPA_vect){
static uint8_t state = 0;
    if(state == 0){// Noisy half period
        toggle_PC3();
        if((--tim2_period)<=0) state = 1;
    }
    if(state == 1){// Mute half period
        if(++tim2_period>=duration)state = 0;
    }
}


