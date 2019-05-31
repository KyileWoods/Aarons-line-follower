#define F_CPU 16000000UL 
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/portpins.h>
#include <avr/interrupt.h>
//Global Variables



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
   DDRB |= (1<<0);DDRB |= (1<<1);DDRB |= (1<<2); //This and the next line turns on the IR LED of the sensor board.//This code isn't actually needed i don't think
}



// Reads ADC4 which is connected to sensor 4 on the new board. 4rd line sensing LED from the LHS looking from the back
int adc_read4 (){
    ADCSRB =(ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000100);
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    return ADCH;

}



// Reads ADC5 which is connected to sensor 3 on the new board. 3rd line sensing LED from the LHS looking from the back
int adc_read3 (){
    ADCSRB =(ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000101);
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
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
}
//int corner_on;


// int side_markercheck(int corner_on){
//     int value = LHS_sidemarkerADC(); //Reads from D7 which is the right sensor on the new board.
//     if(value >= 100 &&corner_on==0){ //Checks to see if it is on a white marker
//         PORTB |=(1<<0); //Turns on an LED1
//         //corner_on = 0; //Some memory to know that it has been switched on.
//         return 1; //Return 1 so we can count
//     }else if(value<100 && corner_on == 3) {
//         PORTB = (PORTB & 0b11111110);
//         //corner_on = 1;
//         return 0;
//     }else{
//         return 3;
//     }
// }


//This is the code to speed it up on the straights. In reality it doesn't work exactly work like that. It does speed up if it is going straight enough
int corner_2 (int LED_2,int LED_3){
    long int diff;int speed;
    diff = (LED_3*1000)/LED_2;
    if(diff<500&&diff>-50){
    speed = 37;    
    
        PORTB |=(1<<1);
    }else{
    speed = 18;
    PORTB = (PORTB & 0b11111101);
        }
    if (LHS_sidemarkerADC()>180){
        speed = 12;
    }
    return speed;
}

long int run_PID (int prev_error){
        int gain = 75;  // this equals a gain of 0.5 all gains need to be multiplied by 1000
        int Kd = 4; // This equals a Kp value of 0.5 same as above //
        long int error; int PID;long int Kd_factor;
        int speed;
        long int LED1 = adc_read1();int LED2 = adc_read2();int LED3 = adc_read3();int LED4 = adc_read4();
        int Sum_LED = LED1+LED2+LED3+LED4;
        long int weighted_error = ((LED1)+(LED2*2)+(LED3*3)+(LED4*4))*1000;
        error = (weighted_error/Sum_LED)-2500;
        Kd_factor = (Kd *((error) - (prev_error)/100)); //This calculates the derivative side of the PD
        PID = ((error*gain)+Kd_factor)/1000; // Calculates the PID value.
        speed = corner_2(LED2,LED3);
        if(prev_error == 30000){
            speed=19;
        }
        if(PID>speed){
            PID =speed;
        }
        if(PID<-speed){
            PID = -speed;
        }
            OCR0A = (speed-PID); //this drives motor 2 or the left wheel
            OCR1A =(speed+PID); //this drives motor 1 or the right wheel
        _delay_ms(2); //Delay for the derivative function
        return error;
}



void check_position(){
//This stops the robot if it runs off. This should work again now.
    while(adc_read1()<50 && adc_read2()<50 && adc_read3()<50 && adc_read4()<50){ 
        OCR0A = 0;
        OCR1A = 0;
    }

}

int Start_finish(int count){
    int RHS = RHS_sidemarkerADC();
    int LHS = LHS_sidemarkerADC();
    if(RHS>200 && count<6 && LHS<100){
        count++;
    }else if(count<5){
        count=0;
    }
    if (RHS<110 && count>=5 && count<=30 && LHS<100){
        count++;
    }
    if(RHS>220&&count>=29&&LHS<100){
        count++;
    }
    if (count>=38){
        OCR0A = 0;
        OCR1A = 0;
        _delay_ms(2000);
        count = 0;
    }
    return count;

}

int Corner_marker(int count){
    int RHS = RHS_sidemarkerADC();
    int LHS = LHS_sidemarkerADC();
    if(LHS>150 && count<13 && RHS<150){
        count++;
        PORTB |=(1<<0);
    }
    if (LHS<110 && count>=12 && count<=30){
        count++;
        //PORTB = (PORTB & 0b11111110);
    }
    if(count>29){
        count = 0;
        PORTB = (PORTB & 0b11111110);
    }
    return count;

}


void main(){
    setup();
    _delay_ms(2000);
        int corner_mem = 0;int corner_count=0; int corner=0;
        int last_error;int error;int corner_on =0;int finish_count = 0;
        while(1){
            check_position();
            error = run_PID(last_error);
            last_error = error;
            finish_count = Start_finish(finish_count);
            corner_on = Corner_marker(corner_on);
            if(corner_on==28){
                corner++;
            }

            if(corner==15){
                //PORTB |=(1<<0);
                OCR0A = 0;
                OCR1A = 0;
                _delay_ms(15);
                while(LHS_sidemarkerADC()<150){
                    last_error=30000;
                    int crap = run_PID(last_error);
                }
            }else{
                //PORTB = (PORTB & 0b11111110);
            }
            if(finish_count ==0){
                corner=0;
                PORTB |=(1<<3);
            }else{
                PORTB = (PORTB & 0b11111011);
            }
            
        }
}