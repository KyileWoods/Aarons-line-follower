#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/portpins.h>
#include <avr/interrupt.h>
//#include <adc.h>
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
   DDRB |= (1<<3); //This and the next line turns on the IR LED of the sensor board.
   PORTB |= (1<<3);
}

// Reads ADC4 which is connected to sensor 1 on sensor board
int adc_read1 (){
    ADCSRB =(ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000100);
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    //_delay_ms(100);
    return ADCH;
}

// Reads ADC5 which is connected to sensor 2 on sensor board
int adc_read2 (){
    ADCSRB =(ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000101);
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    //_delay_ms(100);
    return ADCH;
}

// Reads ADC6 which is connected to sensor 3 on sensor board
int adc_read3 (){
    ADCSRB =(ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000110);
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    return ADCH;
}

// Reads ADC7 which is connected to sensor 4 on sensor board
int adc_read4 (){
    ADCSRB =(ADCSRB & 0b11011111); //clears MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000111);
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    return ADCH;
}

// Reads ADC11 which is connected to sensor 5 on sensor board
int adc_read5 (){
    ADCSRB |=(0b00100000); //Sets MUX5
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000011);
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    return ADCH;
}

// Reads ADC10 which is connected to sensor 6 on sensor board
int adc_read6 (){
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000010);
    ADCSRB |=(0b00100000); //Sets MUX5
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    return ADCH;
}


// Reads ADC9 which is connected to sensor 7 on sensor board
int adc_read7 (){
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000001);
    ADCSRB |=(0b00100000);
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    return ADCH;
}

// Reads ADC8 which is connected to sensor 8 on sensor board
int adc_read8 (){
    ADMUX = (ADMUX & 0xF8); //clears the botom 3 bits of the ADMUX register before oring the channel to be converted
    ADMUX |=(0b00000000); // Not really necessary
    ADCSRB |=(0b00100000); //Sets MUX5
    ADCSRA |= (1<<6);   // sets a flag to start a single conversion of the value on ch ADC channel
    while (ADCSRA  & (1<<6)){}
    return ADCH;
}



int LED1_black;
int LED2_black;
int LED3_black;
int LED4_black;

void button_setup(void){
    DDRC = (0<<6); //Sets pin C6 as an input connected to SW1
    DDRB |= (1<<1); //Sets Pin 1 on PORTB as output.
}


int side_markeron=0; //Side marker is equal to zero when it is not on a side marker.

void setup(){
    timer0_init();
    timer1_init();
    ADC_trim_init();
    button_setup();
    sei(); //Enable interrupt service routines.

		DDRB |= (1<<0)|(1<<1)|(1<<2);
		DDRE |= (1<<6);
}


volatile int PID = 0;
volatile int speed;
volatile int gain;  // this equals a gain of 0.5 all gains need to be multiplied by 1000
volatile int Kd; // This equals a Kp value of 0.5 same as above //


uint8_t Timer_OVF = 0;
int PID_value[9];
int PID_average=0;

// This is the function that checks to see if the robot is moving mostly in a straight line and setting the speed and PD values accordingly.
//AVerages the last 10 PID values and checks to see if the average is near 0,if it is then we can assume that it is on a straight
ISR(TIMER_OVF_vect){
    Timer_OVF ++; //Timer overflows 500 times in a second count 12 overflows to get 0.024s
    if (Timer_OVF>12){
        Timer_OVF =0; PID_average=0;int i,x; //resets our timer count and average
         // For loop shuffles the average values down one spot in the aray to clear a spot for the new value
    for(i=9;i>=1;i--){
        PID_value[i] = PID_value[i-1];
    }
    PID_value[0]=PID; //Stores the latest PID value
    for(x=0;x<10;x++){
        PID_average+=PID_value[x]; //Sums the stored PID values
    }
    PID_average = PID_average/10; //Gets the average
    if(PID_average>12||PID_average<-12){ //The values in this if statement set the value that the average needs to be within to say it is on a straight.
    speed = 20; gain = 150; Kd = 50;                   //Speed and PD values if it is deemed the robot is on a corner
    PORTB = (PORTB & 0b11111101);   //Turns the LED off if is on a corner
    }else{
    PORTB |=(1<<1);     //IF it is on a Straight turn the LED on.
    speed = 32;gain = 150;Kd = 50;  //SPeeds and PD values for the straight.
    }
    }
}

int side_markercheck(void){
    int value = adc_read8(); //Reads from D7 which is the right sensor on the new board.
    if(side_markeron==0 && value <= 180){ //Checks to see if it is on a white marker
        side_markeron = 1; //If it is set marker on to 1
        //PORTB = (PORTB & 0b11111101);
        //PORTB |=(1<<1); //And turn on an LED to let us know
        return 1;
    }else if(value >=180){
        side_markeron=0;//robot LED is off the side marker set to 0 ready for next.
        //PORTB = (PORTB & 0b11111101); //Turn LED off
        return 0;
    }else{
        //PORTB = (PORTB & 0b11111101);
        return 0;
    }
    }

void run_PID (){
        long int error; int prev_error =0; long int Kd_factor;

        long int LED1 = adc_read7();//Why is LED7 a long intand everything else is an int ?
				int LED2 = adc_read6();int LED3 = adc_read4();int LED4 = adc_read3();
        int sum_LED = LED1+LED2+LED3+LED4;
        long int weighted_error = ((LED1)+(LED2*2)+(LED3*3)+(LED4*4))*1000;

        error = (weighted_error/sum_LED)-2500;
        Kd_factor = ((Kd *(error - prev_error)/10)); //This calculates the derivative side of the PD
        PID = ((error*gain)+Kd_factor)/1000; // Calculates the PID value.
        if(PID>speed){
            PID =speed;
        }
        if(PID<-speed){
            PID = -speed;
        }
            OCR0A = (speed-PID); //this drives motor 2 or the left wheel
            OCR1A =(speed+PID); //this drives motor 1 or the right wheel
        prev_error = error; //Stores the last error value
        _delay_ms(10); //Delay for the derivative function
				indicate_position(error);
}

void check_position(){
//This stops the robot if it runs off
    while(adc_read1()>215 && adc_read2()>215 && adc_read7()>215 && adc_read8()>215&& adc_read4()>215){
        OCR0A = 0;
        OCR1A = 0;
    }
}

void indicate_position(long int error){
	if(error<=-2){PORTE |=(1<<6);
	elseif(error>=-2 & error<=0){PORTB |=(1<<0);
	elseif(error>=0 & error<=2){PORTB |=(1<<0);
	elseif(error>=2){PORTB |=(1<<0);
	else{PORTB |= (1<<0)|(1<<1)|(1<<2);
			 PORTE |= (1<<6);
		 }


	}
}


void main(){
    setup();
    _delay_ms(3000);
    int corner_count = 0;
    while(1){
        check_position();
        run_PID();

        //corner_count += side_markercheck();
        }
}
