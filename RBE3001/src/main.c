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
#include <math.h>


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
	//initADC(2);
	//	initSPI();
	//	while(1){
	//		triangle();
	//		printf("Flag: %d\r\n", flag);
	//		if (flag == 1){
	//			int adcval = getADC(2);
	//			float p = PID(center, adcval, .5, 0, 0);
	//			printf("ADCVAL: %d, PID: %0.2f\r\n", adcval, p);
	//			driveMotor0(p);
	//			flag = 0;
	//		}
	//setDAC(1, 4096);
	//		//		setDAC(1, 0);
	//		//		setDAC(2, 0);
	//		//		setDAC(3, 0);

	/* PART 1 OF LAB 2A HERE */
	//				int adcval = getADC(2);
	//				float angle = (adcval - 580)/1023.0 * 270;
	//				printf("ADCVAL: %d, Voltage: %0.1f, Angle: %0.1f\r\n", adcval, adcval*5/1024.0, angle);
	//				_delay_ms(100);

	//
	//	}

	buttonSM2();

	/* Lab 2b starts here */
	//printForwardPosition();
	//triangle1();
	// 410,0 top most position
	// 300,200

	//pointToPoint();
	//	printf("----------\r\n");
	//	while (1){
	//		_delay_ms(1);
	//		if (flag == 1){
	//			int c1 = getADC(3);
	//			int c2 = getADC(2);
	//			float v1 = PID(toADCValue(angles.x, 1), c1, 1.25, 0.01, 0.25, &pl1);
	//			float v2 = PID(toADCValue(angles.y, 2), c2, 1.25, 0.01, 0.25, &pl2);
	//
	//			driveMotor0(v1);
	//			driveMotor1(v2);
	//
	//			printf("Giving motors: %0.2f, %0.2f %d %d %d %d\r\n", v1, v2, toADCValue(angles.x, 1), toADCValue(angles.y, 2), c1, c2);
	//			flag = 0;
	//		}
	//
	//	}

	return 0;
}

void triangle1(){
	setupBoard();

	Point pts[3];
	pts[0].x = -1.45; pts[0].y = -1.35;
	pts[1].x = -1.06; pts[1].y = -0.45;
	pts[2].x = -0.12; pts[2].y = -1.03;

	makeTriangle(pts, 3);

}

void triangle2(){
	setupBoard();

	Point pts[12];
	pts[0].x = -0.12; pts[0].y = -1.03;
	pts[1].x = -0.46; pts[1].y = -1.25;

	pts[2].x = -0.85; pts[2].y = -1.20;

	pts[3].x = -0.9; pts[3].y = -.45;

	pts[4].x = -0.77; pts[4].y = -.5;

	pts[5].x = -0.70; pts[5].y = -0.57;

	pts[6].x = -0.60; pts[6].y = -0.65;

	pts[7].x = -0.50; pts[7].y = -0.75;

	pts[8].x = -0.30; pts[8].y = -0.88;
	pts[9].x = -0.12; pts[9].y = -1.03;

	makeTriangle(pts, 10);


}

void makeTriangle(Point* pts, int numPoints){
	printf("------\r\n");
	DDRC &= ~((1 << DDC7));
	PORTC |= ((1 << DDC7));
	while ((PINC >> PC7));
	//printf("Starting!\r\n");
	for (int i = 0; i < numPoints; i++){
		Point p = *(pts + i);
		//printf("Driving to next %0.2f %0.2f\r\n", p.x, p.y);
		driveTo(&p);
		//_delay_ms(1000);
		//counter0 = 0;
		flag = 0;
	}
	//printf("Finished\r\n");
}

void driveTo(Point* p){
	PIDVar pv;
	initPIDVar(&pv);

	PIDVar pv2;
	initPIDVar(&pv2);

	int c1 = getADC(3);
	int c2 = getADC(2);

	float pe1 = 2;
	float pe2 = 2;
	while (pe1 != 0 || pe2 != 0){
		if (flag == 1){
			c1 = getADC(3);
			c2 = getADC(2);
			pe1 = (pe1 == 0 ? 0 : PID(toADCValue(p->x, 1), c1, KP, KI, KD, &pv));
			pe2 = (pe2 == 0 ? 0 : PID(toADCValue(p->y, 2), c2, KP, KI, KD, &pv2));
			driveMotor0(pe1);
			driveMotor1(pe2);

			//printf("%d %d -> %d %d \r\n", toADCValue(p->x, 1), toADCValue(p->y, 2), c1, c2);
			printf("%ld, %0.2f, %0.2f\r\n", counter0, pe1, pe2);
			flag = 0;
		}
		_delay_ms(1);
	}

}

void pointToPoint(){
	int state = 0;
	initADC(0);
	setupBoard();
	DDRC &= ~((1 << DDC7)|
			(1 << DDC5)|
			(1 << DDC3)|
			(1 << DDC1));
	PORTC |= ((1 << DDC7)|
			(1 << DDC5)|
			(1 << DDC3)|
			(1 << DDC1));

	setDAC(0, 0);
	setDAC(1, 0);
	setDAC(2, 0);
	setDAC(3, 0);

	Point desired;
	desired.x = 410;
	desired.y = 0;
	Point angles;
	getAngles(&desired, &angles);
	//printf("Angle 1: %0.2f, Angle 2: %0.2f\r\n", angles.x, angles.y);
	PIDVar pl1;
	PIDVar pl2;
	initPIDVar(&pl1);
	initPIDVar(&pl2);
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
			desired.x = 410;
			desired.y = 0;
			getAngles(&desired, &angles);
			initPIDVar(&pl1);
			initPIDVar(&pl2);
			break;
		case 2:
			desired.x = 100;
			desired.y = -266;
			initPIDVar(&pl1);
			initPIDVar(&pl2);
			getAngles(&desired, &angles);
			break;
		case 3:
			desired.x = 270;
			desired.y = 200;
			initPIDVar(&pl1);
			initPIDVar(&pl2);
			getAngles(&desired, &angles);
			break;
		case 4:
			desired.x = 110;
			desired.y = -200;
			initPIDVar(&pl1);
			initPIDVar(&pl2);
			break;
		default:
			//state = 0;
			break;
		}
		if ( (flag == 1) & (state > 0)){
			getAngles(&desired, &angles);
			int c1 = getADC(3);
			int c2 = getADC(2);
			float v1 = PID(toADCValue(angles.x, 1), c1, KP, KI, KD, &pl1);
			float v2 = PID(toADCValue(angles.y, 2), c2, KP, KI, KD, &pl2);
			driveMotor0(v1);
			driveMotor1(v2);
			Point p;
			float a1 = toRadians(c1, 1);
			float a2 = toRadians(c2, 2);
			getEndPosition(&p, LINK_LENGTH_1, a1, LINK_LENGTH_2, a2);
			p.x = p.x + LINK_OFFSET;
			printf("Current: %0.1f, %0.1f. Desired: %0.1f, %0.1f\r\n", p.x, p.y, desired.x, desired.y);
			printf("Giving motors: %d %0.2f, %0.2f %d %d %0.1f %0.1f\r\n",
					state, v1, v2,
					toADCValue(angles.x, 1), toADCValue(angles.y, 2),
					angles.x, angles.y);
			flag = 0;
		}
	}
}

void getAngles(Point* desired, Point* angles){
	float x = desired->x - LINK_OFFSET;
	float y = desired->y;
	float l1 = LINK_LENGTH_1;
	float l2 = LINK_LENGTH_2;
	float t1 = (x*x + y*y - l1*l1 - l2*l2)/(2*l1*l2);
	float a2 = atan2(sqrt(1 - t1*t1), t1);

	float a1 = atan2(y, x)  - atan2(l2*sin(a2), l1 + l2*cos(a2));
	angles->x = a1;
	angles->y = a2;

}
void printForwardPosition(){
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
		//printf("Angle 1 = %0.2f, Angle 2 = %0.2f, x = %0.1f, y = %0.1f\r\n", a1, a2, p.x, p.y);
		printf("%0.2f,%0.2f\r\n ", a1, a2);

		_delay_ms(100);
	}
}
void getEndPosition(Point* p, int l1, float a1, int l2, float a2){
	p->x = l1*cos(a1) + l2*cos(a2 + a1);
	p->y = l1*sin(a1) + l2*sin(a2 + a1);
}
void initPIDVar(PIDVar* p){
	p->sum = 0;
	p->prev = 0;
	p->deltat = 0.01;
}
int toADCValue(float radians, int link){
	if (link == 2)
		return (int)((radians*180/M_PI + 131)*(1024/240));
	return (int)((radians*180/M_PI + 159)*(1024/240));
}
float toDegrees(int adcval, int link){
	if (link == 2)
		return (((adcval)/1024.0 * 240) - 131);
	return (((adcval)/1024.0 * 240) - 159);
}
float toRadians(int adcval, int link){
	return toDegrees(adcval, link)*M_PI/180;
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

float PID(int setpoint, int curr, float kp, float ki, float kd, PIDVar* var){
	float deltat = var->deltat;
	float sum = var->sum;
	float prev = var->prev;
	float error = setpoint - curr;
	if (error <= 10 && error >= -10){
		return 0;
	}

	float val = kp * error * deltat + ki*sum*deltat + kd * (error - prev) * deltat;
	var->sum += error;
	var->prev = error;
	return val;
}

// Signal must be between -1 and 1
void driveMotor0(float signal){
	if(signal >= 0){
		setDAC(2, signal*4096);
		setDAC(3, 0);
	}
	else{
		setDAC(3, -1*signal*4096);
		setDAC(2, 0);
	}
}


void driveMotor1(float signal){

	if(signal < 0){
		setDAC(0, -1*signal*4096);
		setDAC(1, 0);
	}
	else{
		setDAC(1, signal*4096);
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
	setupBoard();
	DDRC &= ~((1 << DDC7)|
			(1 << DDC5)|
			(1 << DDC3)|
			(1 << DDC1));
	PORTC |= ((1 << DDC7)|
			(1 << DDC5)|
			(1 << DDC3)|
			(1 << DDC1));
	int degs = 0;
	PIDVar pv;
	pv.sum = 0;
	pv.deltat = 0.001;
	pv.prev = 0;

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
			initPIDVar(&pv);
			degs = 950;
			break;
		case 2:
			initPIDVar(&pv);
			degs = 835;
			break;
		case 3:
			initPIDVar(&pv);
			degs = 735;
			break;
		case 4:
			initPIDVar(&pv);
			degs = 575;
			break;
		default:
			//state = 0;
			break;
		}
		//printf("Flag: %d\r\n", flag);
		if ( (flag == 1) & (state > 0)){
			int adcval = getADC(2);
			float p = PID(degs, adcval, .6, 0.1, 0.1, &pv);
			int a = toDegrees(adcval, 2);
			int b = (90 - 30*(state-1));
			int c =  getCurrent(0);
			int d = p*4096;
			if(d > 4096) d = 4096;
			if(d < -4096) d = -4096;
			printf("%ld, %d, %d, %d, %d\r\n", counter0, a, b, c, d);
			driveMotor1(p);
			flag = 0;
		}
	}
}

void setupBoard(){
	initSPI();
	initADC(2);
	initADC(3);
	timer0_init();

	setDAC(0, 0);
	setDAC(1, 0);
	setDAC(2, 0);
	setDAC(3, 0);
}
// isr setup
ISR(TIMER0_COMPA_vect) {
	if (counter0 >= 65535||counter0 < 0){
		counter0 = 0;
	}
	counter0++; // increment our counter
	flag = 1;
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
