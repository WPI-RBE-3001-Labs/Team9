/*
 * main.c
 *
 *  Created on: Jan 18, 2017
 *      Author: vanc
 */
#include "RBELib/RBELib.h"
#include "RBELib/timer.h"
#include "RBELib/ADC.h"
#include "RBELib/DAC.h"
#include "RBELib/SPI.h"
#include <avr/io.h>
#include "RBELib/USARTDebug.h"
#include <string.h>
#include "main.h"


volatile char buf[50];
volatile unsigned short adcdatas[225];
volatile unsigned short timedatas[225];


volatile unsigned long counter0 = 0;
volatile unsigned long counter1 = 0;

volatile unsigned char timer_started = 0;
volatile char data_counter = 0;
volatile char data_done = 0;
int channel = 2;
unsigned short adc_val2;
int freq_div = 100;

int flag = 0; // flag for timer 0 interrupt

typedef struct {
	volatile unsigned int sec;
	volatile unsigned int min;
	volatile unsigned int hrs;
} SimpleClock;

SimpleClock sc;

void init_sc(){
	sc.sec = 0;
	sc.min = 0;
	sc.hrs = 0;
}

void update_sc(){
	if((counter0 % 225) == 0){
		sc.sec++;
	}
	if(sc.sec >= 59){
		sc.sec = 0;
		sc.min++;
	}
	if(sc.min>= 59){
		sc.min = 0;
		sc.hrs++;
	}
		sc.hrs = 0;
	if(sc.hrs >=23){
	}
}


int main(void){
	initRBELib();

	debugUSARTInit(115200);


	/** TRY RUNNING THIS ON THE OSCILLOSCOPE */

//	printf("--------------\r\n");
//	int center = 580;
//	while(1){
//		printf("Flag: %d\r\n", flag);
//		if (flag == 1){
//			int adcval = getADC(2);
//			float p = PID(center, adcval, .5, 0, 0);
//			printf("ADCVAL: %d, PID: %0.2f\r\n", adcval, p);
//			driveMotor0(p);
//			flag = 0;
//		}
//		//		setDAC(0, 0);
//		//		setDAC(1, 0);
//		//		setDAC(2, 0);
//		//		setDAC(3, 0);
//		//		int adcval = getADC(2);
//		//		float angle = (adcval - center)/1023.0 * 270;
//		//		printf("ADCVAL: %d, Voltage: %0.1f, Angle: %0.1f\r\n", adcval, adcval*5/1023.0, angle);
//		//		_delay_ms(100);
//
//	}

	//	buttonSM2();

	/* Lab 2b starts here */
	Point p;
	initADC(2);
	initADC(3);

	while (1){
		int adcv1 = getADC(3);
		int adcv2 = getADC(2);
		float a1 = toRadians(adcv1, 1);
		float a2 = toRadians(adcv2, 2);
		getEndPosition(&p, LINK_LENGTH_1, a1, LINK_LENGTH_2, a2);
		p.x = p.x + LINK_OFFSET;
		//printf("Angle 1 = %0.1f, Angle 2 = %0.1f, x = %0.1f, y = %0.1f\r\n", a1, a2, p.x, p.y);
		printf("%0.1f,%0.1f\r\n ", a1, a2);
		_delay_ms(100);
	}

	return 0;
}

void getEndPosition(Point* p, int l1, float a1, int l2, float a2){
	p->x = l1*cos(a1) + l2*cos(a2 + a1);
	p->y = l1*sin(a1) + l2*sin(a2 + a1);
}

float toRadians(int adcval, int link){
	if (link == 2)
		return (((adcval)/1024.0 * 240) - 150)*M_PI/180;
	return (((adcval)/1024.0 * 240) - 140)*M_PI/180;
}

void triangle(){
	for (int i = 0; i < 4096; i+=10){
		setDAC(0, i);
		setDAC(1, 4095 - i);
		_delay_ms(1);
	}
	for (int i = 4095; i >= 0; i-=10){
		setDAC(0, i);
		setDAC(1, 4095 - i);
		_delay_ms(1);
	}
}

float sum = 0;
float prev = 0;
float deltat = 0.01;
float PID(int setpoint, int curr, float kp, float ki, float kd){
	float error = curr - setpoint;
	float val = kp * error * deltat + ki*sum*deltat + kd * (error - prev) * deltat;

	sum += error;
	prev = error;
	return val;
}

void resetPID(){
	sum = 0;
	prev = 0;
}

// Signal must be between -1 and 1
void driveMotor0(float signal){
	if(signal >= 0){
		setDAC(0, signal*4096);
		setDAC(1, 0);
	}
	else{
		setDAC(1, -1*signal*4096);
		setDAC(0, 0);
	}
}


void printToSerial(char data[]){
	int i;
	for(i = 0; i<strlen(data); i++){
		putCharDebug(data[i]);
	}
	putCharDebug('\n');
	putCharDebug('\r');

}

int getCurrent(int val)
{
	//return 0;
	return ((getADC(val)-30) * 4.89) - 2500;
}

void buttonSM2()
{
	int state = 0;
	initADC(0);
	initADC(2);
	initSPI();
	timer0_init();
	DDRC &= ~((1 << DDC7)|
			(1 << DDC5)|
			(1 << DDC3)|
			(1 << DDC1));
	PORTC |= ((1 << DDC7)|
			(1 << DDC5)|
			(1 << DDC3)|
			(1 << DDC1));
	int degs = 0;
	while(1)
	{


		int button1 = (PINC>>PC7)&1;
		int button2 = (PINC>>PC6)&1;
		int button3 = (PINC>>PC5)&1;
		int button4 = (PINC>>PC4)&1;
		if( ((button1)==0) && ((button2)==1) &&((button3)==1) &&((button4)==1) ){
			state = 1;
		}
		if( ((button2)==0) && ((button1)==1) &&((button3)==1) &&((button4)==1)){
			state = 2;
		}
		if( ((button3)==0) && ((button1)==1) &&((button2)==1) &&((button4)==1)){
			state = 3;
		}
		if( ((button4)==0) && ((button1)==1) &&((button2)==1) &&((button3)==1)){
			state = 4;
		}

		switch(state){
		case 1:
			resetPID();
			degs = 950;
			break;
		case 2:
			resetPID();
			degs = 835;
			break;
		case 3:
			resetPID();
			degs = 735;
			break;
		case 4:
			resetPID();
			degs = 575;
			break;
		default:
			//state = 0;
			break;
		}
		//printf("Flag: %d\r\n", flag);
		if ( (flag == 1) & (state > 0)){
			int adcval = getADC(2);
			float p = PID(degs, adcval, .6, 0.1, 0.1);
			int a = 255-((adcval)/1023.0 * 270);
			int b = (30*(state-1));
			int c =  getCurrent(0);
			int d = p*4096;
			if(d > 4096) d = 4096;
			if(d < -4096) d = -4096;
			printf("%d, %d, %d, %d, %d\r\n", counter0, a, b, c, d);
			driveMotor0(p);
			flag = 0;
		}
	}
}

// isr setup
ISR(TIMER0_COMPA_vect) {
	if (counter0 >= 65535||counter0 < 0){
		counter0 = 0;
	}
	counter0++; // increment our counter
	flag = 1;
	update_sc();
}


ISR(TIMER2_COMPA_vect) {
	if (counter1 >= freq_div||counter1 < 0){
		counter1 = 0;
	}
	counter1++; // increment our counter
}


ISR(ADC_vect){
	adc_val2 = ADCH;
}


void timer0_init(){
	//cli();
	// Initialize timer count to 0
	TCNT0 = 0;
	TCCR0A |= (1 << WGM01);// set to CTC mode
	TCCR0A |= (1 << COM0A1); // Configure timer 0 for CTC mode

	TCCR0B |= (1<<CS02) | (1<<CS00); // prescale by 1024 for 18kHz
	OCR0A = 179; // divide by 180 -1  to get 100 Hz count

	//counter0 = 0; // initialize timercount
	TIMSK0 |= (1 << OCIE0A); // Enable CTC interrupt
	sei(); // Enable global interrupts
}


void ADCtimer_init(){
	//cli();
	// Initialize timer count to 0
	TCNT0 = 0;
	TCCR0A |= (1 << WGM01);// set to CTC mode
	TCCR0A |= (1 << COM0A1); // Configure timer 0 for CTC mode

	TCCR0B |= (1<<CS02) | (1<<CS00); // prescale by 1024 for 18kHz
	OCR0A = 79; // divide by 180 -1  to get 100 Hz count

	//counter0 = 0; // initialize timercount
	TIMSK0 |= (1 << OCIE0A); // Enable CTC interrupt
	sei(); // Enable global interrupts
}

void timer2_init(){
	TCNT2 = 0;
	TCCR2A |= (1 << WGM21);// set to CTC mode
	TCCR2A |= (1 << COM2A1); // Configure timer 0 for CTC mode

	TCCR2B |= (1<<CS22) | (1<<CS20);
	OCR2A = 14; // divide by 15-1  to get 10kHz count

	// 145/256 = 1k/x => x = 256k/145
	//counter0 = 0; // initialize timercount
	TIMSK2 |= (1 << OCIE2A); // Enable CTC interrupt
	//TIMSK1 |= (1 << TOIE1); // Enable CTC interrupt
	sei(); // Enable global interrupts
}
