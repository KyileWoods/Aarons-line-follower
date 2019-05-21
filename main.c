#define F_CPU 16000000UL 
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/portpins.h>
#include <avr/interrupt.h>


void timer0_init(){
    //set up timer0 to output fast PWM, 256 prescaler, clear on compare match, set on top
    DDRB |= (1<<7);
    DDRD |= (1<<0);
    TCCR0A |= (1<<7)|(1<<5)|(1<<1)|1;
    TIMSK0 |= (1<<0); //enable overflow interrupt
    OCR0B = 0;
    TCCR0B |= (1<<2);
    
}

void timer1_init(){
    //set up timer1 to output fast PWM (8-bit), 256 prescaler, clear on compare match, set on top
    DDRB |= (1<<6)|(1<<5);
    TCCR1A |= (1<<7)|(1<<5)|1;
    TCCR1B |= (1<<3);
    TIMSK1 |= (1<<0); ////enable overflow interrupt
    OCR1B = 0;
    TCCR1B |= (1<<2);
}

void ADC_trim_init(){
    ADMUX = (0x20); //Sets the reference to the internal 2.56V and left adjusts the results
    ADCSRA = (0x87); // Enables the ADC conversion and sets the prescaler to 128
   DDRB |= (1<<3); //This and the next line turns on the IR LED of the sensor board.//This code isn't actually needed i don't think
   PORTB |= (1<<3); //because CHris has the VCC directly connected to sensors. I left it in because it didn't seem to create any conflict.
   DDRB |= (1<<1);
   DDRB |= (1<<2);
}

// Reads ADC4 which is connected to sensor 4 on the new board. 4rd line sensing LED from the LHS looking from the back
int adc_read4 (){
    ADCSRB =(ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000100);
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    //_delay_ms(100);
    return ADCH;
}

// Reads ADC5 which is connected to sensor 3 on the new board. 3rd line sensing LED from the LHS looking from the back
int adc_read3 (){
    ADCSRB =(ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000101);
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    //_delay_ms(100);
    return ADCH;
}

// Reads ADC6 which is connected to sensor 2 on the new board. 2nd line detecting LED from the LHS from the back
int adc_read2 (){
    ADCSRB =(ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000110);
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    return ADCH;
}

// Reads ADC7 which is connected to the RHS side marker LED
int RHS_sidemarkerADC (){
    ADCSRB =(ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000111);
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    return ADCH;
}

// Reads ADC11 which is connected to sensor 1 on the new board. That is the first line detecting LED on the LHS looking from the back
int adc_read1 (){
    ADCSRB |=(0b00100000); //Sets MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000011);
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    return ADCH;
}

// Reads ADC10 which is connected to the LHS side marker LED
int LHS_sidemarkerADC (){
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000010);
    ADCSRB |=(0b00100000); //Sets MUX5
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    return ADCH;
}


void setup(){
    timer0_init();
    timer1_init();
    ADC_trim_init();
    sei();
}

void side_markercheck(void){
    int value = LHS_sidemarkerADC(); //Reads from D7 which is the right sensor on the new board.
    if(value >= 180){ //Checks to see if it is on a white marker
        //side_markeron = 1; //If it is set marker on to 1
        PORTB |=(1<<1);
    }else{
        PORTB = (PORTB & 0b11111101);
    }
}


void run_PID (){
        int gain = 30;  // this equals a gain of 0.5 all gains need to be multiplied by 1000
        int Kd = 3; // This equals a Kp value of 0.5 same as above //
        long int error; int prev_error =0; long int PID;long int Kd_factor;
        int speed = 21;
        long int LED1 = adc_read1();int LED2 = adc_read2();int LED3 = adc_read3();int LED4 = adc_read4();
        int Sum_LED = LED1+LED2+LED3+LED4;
        long int weighted_error = ((LED1)+(LED2*2)+(LED3*3)+(LED4*4))*1000;

        error = (weighted_error/Sum_LED)-2500;
        Kd_factor = ((Kd *(error - prev_error)/100)); //This calculates the derivative side of the PD
        PID = ((error*gain)+Kd_factor)/1000; // Calculates the PID value.
        if(PID>speed){
            PID =speed-3;
        }
        if(PID<-speed){
            PID = -speed+3;
        }
            OCR0A = (speed+(-PID)); //this drives motor 2 or the left wheel
            OCR1A = (speed+PID); //this drives motor 1 or the right wheel
        prev_error = error; //Stores the last error value
        _delay_ms(5); //Delay for the derivative function
}

void finish_detect(void){
    int value1 = RHS_sidemarkerADC(); //Reads from D7 which is the right sensor on the new board.
    if(value1 >= 180){ //Checks to see if it is on a white marker
        //side_markeron = 1; //If it is set marker on to 1
        PORTB |=(1<<2);
        int Timer1 = 0;
        while(Timer1<=80){
            run_PID();
            Timer1++;
            _delay_ms(10);
        }
            OCR0A = 0;
            OCR1A = 0;
            _delay_ms(2000);
            PORTB = (PORTB & 0b11111011);
        }
}



void check_position(){
//This stops the robot if it runs off. This should work again now.
    while(adc_read1()<130 && adc_read2()<130 && adc_read3()<130 && adc_read4()<130){ 
        OCR0A = 0;
        OCR1A = 0;
    }
}


void main(){
    setup();
    _delay_ms(3000);
    int corner_count = 0;
    while(1){
        //check_position();
        run_PID();
        side_markercheck();
        finish_detect();
        }
}