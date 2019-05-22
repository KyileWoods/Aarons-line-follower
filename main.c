#define F_CPU 16000000UL 
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/portpins.h>
#include <avr/interrupt.h>
//Global Variables



void timer0_init() {
    //set up timer0 to output fast PWM, 256 prescaler, clear on compare match, set on top
    DDRB |= (1 << 7);
    DDRD |= (1 << 0);
    TCCR0A |= (1 << 7) | (1 << 5) | (1 << 1) | 1;
    TIMSK0 |= (1 << 0); //enable overflow interrupt
    OCR0B = 0;
    TCCR0B |= (1 << 2);
}



void timer1_init() {
    //set up timer1 to output fast PWM (8-bit), 256 prescaler, clear on compare match, set on top
    DDRB |= (1 << 6) | (1 << 5);
    TCCR1A |= (1 << 7) | (1 << 5) | 1;
    TCCR1B |= (1 << 3);
    TIMSK1 |= (1 << 0); ////enable overflow interrupt
    OCR1B = 0;
    TCCR1B |= (1 << 2);
}



void ADC_init() {
    ADMUX = (0x20); //Sets the reference to the internal 2.56V and left adjusts the results
    ADCSRA = (0x87); // Enables the ADC conversion and sets the prescaler to 128
    DDRB |= (1 << 1); //This and the next line turns on the IR LED of the sensor board.//This code isn't actually needed i don't think
    DDRB |= (1 << 3); //because CHris has the VCC directly connected to sensors. I left it in because it didn't seem to create any conflict.
}



// Reads ADC4 which is connected to sensor 4 on the new board. 4rd line sensing LED from the LHS looking from the back
int adc_read4() {
    ADCSRB = (ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |= (0b00000100);
    ADCSRA |= (1 << 6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1 << 6)) {}
    return ADCH;

}



// Reads ADC5 which is connected to sensor 3 on the new board. 3rd line sensing LED from the LHS looking from the back
int adc_read3() {
    ADCSRB = (ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |= (0b00000101);
    ADCSRA |= (1 << 6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1 << 6)) {}
    return ADCH;

}



// Reads ADC6 which is connected to sensor 2 on the new board. 2nd line detecting LED from the LHS from the back

int adc_read2() {
    ADCSRB = (ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |= (0b00000110);
    ADCSRA |= (1 << 6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1 << 6)) {}
    return ADCH;
}



// Reads ADC7 which is connected to the RHS side marker LED

int RHS_sidemarkerADC() {
    ADCSRB = (ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |= (0b00000111);
    ADCSRA |= (1 << 6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1 << 6)) {}
    return ADCH;
}



// Reads ADC11 which is connected to sensor 1 on the new board. That is the first line detecting LED on the LHS looking from the back

int adc_read1() {
    ADCSRB |= (0b00100000); //Sets MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |= (0b00000011);
    ADCSRA |= (1 << 6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1 << 6)) {}
    return ADCH;
}



// Reads ADC10 which is connected to the LHS side marker LED

int LHS_sidemarkerADC() {
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |= (0b00000010);
    ADCSRB |= (0b00100000); //Sets MUX5
    ADCSRA |= (1 << 6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1 << 6)) {}
    return ADCH;
}

int error_value[10]; int corner;

int corner_detect(int error, int speed) {
    /*
    Counts to 12 overflow events
    */

    corner++; //Timer overflows 500 times in a second count 12 overflows to get 0.024s
    if (corner > 12) {
        corner = 0; int error_average = 0; int i, x; //resets our timer count and average
         // For loop shuffles the average values down one spot in the aray to clear a spot for the new value
        for (i = 9; i > 0; i--) {
            error_value[i] = error_value[i - 1];
        }
        error_value[0] = error; //Stores the latest error value
        for (x = 0; x < 10; x++) {
            error_average += error_value[x]; //Sums the stored PID values
        }
        error_average = error_average / 9; //Gets the average
        if (error_average > 2 || error_average < -2) { //The values in this if statement set the value that the average needs to be within to say it is on a straight.
            speed = 20;// gain = 100; Kd = 50;                   //Speed and PD values if it is deemed the robot is on a corner
            PORTB = (PORTB & 0b11111101);   //Turns the LED off if is on a corner
        }
        else {
            PORTB |= (1 << 1);     //IF it is on a Straight turn the LED on.
            speed = 30;//gain = 100;Kd = 50;  //SPeeds and PD values for the straight.
        }
    }
    return speed;
}


void count_down() {
    DDRB |= (1 << 0) | (1 << 1) | (1 << 2);
    DDRE |= (1 << 6);
    PORTB |= (1 << 0) | (1 << 1) | (1 << 2);
    PORTE |= (1 << 6);

    _delay_ms(1000);
    PORTE ^= (1 << 6);
    for (int i = 0; i <= 2; i++) {
        _delay_ms(700);
        PORTB ^= (1 << i);
    }
    

} 


void setup() {
    timer0_init();
    timer1_init();
    ADC_init();
    count_down();
}



int corner_2(int LED_2, int LED_3) {
    long int diff; int speed;
    diff = (LED_3 * 1000) / LED_2;
    if (diff<2050 && diff>-50) {
        speed = 35;
        PORTB |= (1 << 1);
    }
    else {
        speed = 19;
        PORTB = (PORTB & 0b11111101);
    }
    if (LHS_sidemarkerADC() > 180) {
        speed = 18;
    }
    return speed;
}

long int run_PID(int prev_error) {
    int gain = 30;  // this equals a gain of 0.5 all gains need to be multiplied by 1000
    int Kd = 5; // This equals a Kp value of 0.5 same as above //
    long int error; int PID; long int Kd_factor;
    int speed;
    long int LED1 = adc_read1(); int LED2 = adc_read2(); int LED3 = adc_read3(); int LED4 = adc_read4();
    int Sum_LED = LED1 + LED2 + LED3 + LED4;
    long int weighted_error = ((LED1)+(LED2 * 2) + (LED3 * 3) + (LED4 * 4)) * 1000;
    error = (weighted_error / Sum_LED) - 2500;
    Kd_factor = (Kd *((error)-(prev_error) / 100)); //This calculates the derivative side of the PD
    PID = ((error*gain) + Kd_factor) / 1000; // Calculates the PID value.
    speed = corner_2(LED2, LED3);
    if (PID > speed) {
        PID = speed;
    }
    if (PID < -speed) {
        PID = -speed;
    }
    OCR0A = (speed + (-PID)); //this drives motor 2 or the left wheel
    OCR1A = (speed + PID); //this drives motor 1 or the right wheel
    _delay_ms(5); //Delay for the derivative function
    return error;
}



void Emergency_brake() {
    //This stops the robot if it runs off. This should work again now.
    while (adc_read1() < 200 && adc_read2() < 50 && adc_read3() < 50 && adc_read4() < 200) {
        OCR0A = 0;
        OCR1A = 0;
    }

}





int main() {
    setup(); //Now includes a safety-pause and Christmas-tree style countdown.
    int corner_count = 0;
    int last_error;
    int error;
    while (1) {
        Emergency_brake();
        error = run_PID(last_error);
        last_error = error;
    }
    return 0;
}