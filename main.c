#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
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
#include "kalman.h"
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

//const float dt = powf(2,16)*(1.f/8E6);
const float dt = 0.008f;
float time = 0.f;
void timer1_init(void){
    // TIMER 1 Config for timing while 1
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1C = 0;
    TCCR1B |=(1<<CS10);//prescaler=1 
    //Enable Overflow Interrupt Enable
    //TIMSK1|=(1<<TOIE1);
}
/*
ISR(TIMER1_OVF_vect)
{
time += dt;
}
*/
void timer2_init(void){
    // TIMER 2 Config for PWM tone
    TCCR2A = (1<<WGM21) | (1<<WGM20);
    TCCR2B = (0<<CS20) | (1<<CS21) | (1<<CS22) |(1<<WGM22);
    //TCCR2B = (0<<CS21);
    OCR2A = 0x00;
    OCR2B = 0x00;
    TIMSK2 = (1<<OCIE2A);
    TIMSK2 |= (1<<OCIE2B);

    // Init A3 pin (PC3)
    DDRC |= (1<<DDC3);
    PORTC &= ~(1<<PORTC3);
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
volatile int32_t tim2_period = 0;
volatile uint8_t mute = 0;
volatile uint8_t state = 0;

void timer2_set_duration(float d_sec){
    cli();
    duration = (int)(d_sec/2.f*(8E6/128.f)/(float)OCR2A);
    tim2_period = duration;//Launch new period
    sei();
}

void timer2_set_volume(float volume){
    cli();
    OCR2B = (uint8_t)((OCR2A/2)*(volume/100.f));// Pourcent of OCR2A
    
    sei();
}

float altitude = 0.f;

int parse_nmea(uint8_t c){
    static int i = 0;
    static int i_comma = 0;
    static int nb_comma = 0;
    static uint8_t alt[10]={'0'};
    if( i==0){
        if(c == '$'){i++;}
    }
    else if(i==1){
        if(c == 'G'){i++;}
        else{i=0;}
    }
    else if(i==2){
        if(c == 'P'){i++;}
        else{i=0;}
    } 
    else if(i==3){
        if(c == 'G'){i++;}
        else{i=0;}
    } 
    else if(i==4){
        if(c == 'G'){i++;}
        else{i=0;}
    } 
    else if(i==5){
        if(c == 'A'){i++;}
        else{i=0;}
    } 
    else if(i==6){
        if(c == ','){i++;}
        else{i=0;}
    }
    else if(i>6){
        i++;
        if(c==','){nb_comma++;}
        if(nb_comma==8){
            i_comma++;
            if(i_comma>1){
                alt[i_comma-2]=c;
            }
        } 
        else if(nb_comma==9){
            alt[(i_comma<sizeof(alt)?i_comma:sizeof(alt))] = '\0';
            // Here set altitude global variable
            altitude = atof((const char*)alt);

            i_comma=0;
            nb_comma = 0;
            i=0;
        }
    }
    else{
        i = 0;
    }
    return 0;
}

volatile uint8_t tone_done = 0;

uint32_t b_int, a_int;

int main(void)
{
    char c = ' ';

    uart_init();
    softuart_init();
    softuart_turn_rx_on(); /* redundant - on by default */  
    i2c_init();

    sei();


    stdout = &uart_output;

    InitBMP280();

    float alt      = 0.f;
    float rate     = 0.f;

    float freq = 300.f;
    float const low_level = -0.5f;
    float const high_level = 0.5f;
    float const low_gain = -50.f;
    float const low_offset = 300.f;
    float const high_gain = 150.f;
    float const high_offset = 1100.f;

    int const prs_count_max = 5;
    int prs_count = 0;

    timer1_init();

    timer2_init();
    timer2_set_freq(250.f);
    timer2_set_duration(1.f);
    timer2_set_volume(20.f);

    alt = AltitudeBMP280();
    kalman_init(alt);
    
    for (;;) {
        while((TIFR1 & (1<<TOV1))!=(1<<TOV1)){// Wait until flag set
            while( softuart_kbhit() ) {
                c = softuart_getchar();
                //putchar(c);
            }
        }
        TIFR1 |= (1 << TOV1);
        time += dt;

        alt = AltitudeBMP280();
        kalman_predict(dt);
        kalman_update(alt);
        rate = X[1];
        //printf("%f,%f,%f,%f\r\n",(double)time, (double)alt,(double)X[1],(double) rate);
        //printf("B:%u - A:%u\r\n",(unsigned int)b_int,(unsigned int)a_int); 

        if(++prs_count>=prs_count_max && c=='\n'){ // Ensure NMEA end of line
            prs_count = 0;
            //printf("PRS %05x\r\n",(int)PressureBMP280());
        }

        rate = 1.f;
        if(tone_done==1){
            if(rate < low_level){
                mute = 0;
                freq = low_gain*fabs(rate) + low_offset;
            }
            else if(rate > high_level){
                mute = 0;
                freq = rate * high_gain + high_offset;
            }
            else {
                mute = 1;// Mute
            }
            // Constraint freq
            if(freq<250.f)freq = 250.f;

            timer2_set_freq(freq);
            timer2_set_duration(500.f/freq);
            timer2_set_volume(20.f);
            tone_done = 0;
        }


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
    else{
        PORTC &= (0<<PORTC3);
    }
}

ISR(TIMER2_COMPA_vect){
    a_int++;
    if(state == 0){// Noisy half period
        //toggle_PC3();
        if(!mute) PORTC |= (1<<PORTC3);
        if((--tim2_period)<=0) state = 1;
    }
    if(state == 1){// Mute half period
        if(++tim2_period>=duration){
            state = 0;
            tone_done = 1;
        }
    }
}

ISR(TIMER2_COMPB_vect){
    if(state == 0){
        if(!mute)PORTC &=(0<<PORTC3);
    }
}


