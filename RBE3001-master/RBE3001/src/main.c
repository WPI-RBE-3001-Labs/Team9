/*
 * main.c
 *
 *  Created on: Jan 18, 2017
 *      Author: vanc
 */
#include "main.h"

volatile char buf[50];

volatile unsigned long counter0 = 0;
volatile unsigned long counter1 = 0;

volatile unsigned char timer_started = 0;
volatile char data_counter = 0;
volatile char data_done = 0;
int channel = 2;
unsigned short adc_val2;
int freq_div = 100;

int flag = 0; // flag for timer 0 interrupt

typedef enum{
	GO_HOME,
	WAIT_FOR_BLOCK,
	READ_IR,
	MOVE_TO_POSITION, // also opens gripped
	WAIT_FOR_TRIGGER, // reaches 2nd IR
	CLOSE_GRIPPER,
	PICKUP_BLOCK,
	MOVE_TO_1,
	MOVE_TO_2,
	DROP_BLOCK,
	END_STATE
} State;

State runState(State s);

// INITIALIZE THESE POINTS;
Point home;
Point midway;
Point heavy;
Point light;

int main(void){
	initRBELib();

	debugUSARTInit(115200);

	setupBoard();

	home.x = -1 * M_PI/6; // x1
	home.y = -1 * M_PI/6; // x2

	midway.x = -1 * M_PI/6;
	midway.y = -1*M_PI/3;

	heavy.x = 0;
	heavy.y = -1*M_PI/4;


	light.x = 0;
	light.y = M_PI/4;
	printf("We moving\r\n");
	//driveTo(&home);

	driveMotor0(0);
	driveMotor1(0);

	//printForwardPosition();

	Point p1;
	p1.x = 140; // 30, 45
	p1.y = -210; // 320,230			160, -205
	// 37, 240
	// 260, -190 MID WAY POSITION

	// 130 -208 BOTTOM POSITION
	Point angles;


	getAngles(&p1, &angles);

	printf("%0.2f %0.2f\r\n", angles.x, angles.y);
	//driveTo(&angles);



	State s = GO_HOME;
	//driveTo(&home);
	runBelt();
	while (1){
		_delay_ms(1);
		s = runState(s);
		//		printf("IR Value: %d\r\n", getADC(IR2));
		//		openGripper();
		//		_delay_ms(1000);
		//		closeGripper();
		//		_delay_ms(1000);
	}

	return 0;
}
double max = 0;
int count = 0;
int num = 0;
int st = 0;
int runs = 0;

State runState(State s){
	Point p1;
	Point angles;
	int temp = 0;
	long curr = 0;
	PIDVar pv;
	PIDVar pv2;
	float pe1 = 2;
	float pe2 = 2;
	int c1 = 0;
	int c2 = 0;


	switch(s){
	case GO_HOME:
		runBelt();
		openGripper();
		driveTo(&home);
		count = 0;
		return WAIT_FOR_BLOCK;
	case WAIT_FOR_BLOCK:
		openGripper();
		temp = getADC(IR1);
		max = 0;
		printf("IR1: %d\r\n", temp);
		if (temp > IR1THRESHOLD){
			_delay_ms(10);
			printf("detected\r\n");
			count++;
			if (count > 5){
				printf("switching state\r\n");
				count = 0;
				num = 0;
				return READ_IR;
			}
		}
		else
			count = 0;
		return WAIT_FOR_BLOCK;
	case READ_IR:
		temp = getADC(IR1);
		//printf("Reading %d %d\r\n", temp, max);
		//max = (temp > max) ? temp : max;
		num++;
		max += temp/10.0;
		if (temp < IR1THRESHOLD){
			max = max/num * 10.0;
			printf("MOVING ARM with AVG IR: %f\r\n", max);
			return MOVE_TO_POSITION;
		}
		return READ_IR;
	case MOVE_TO_POSITION:
		getPosition(&p1, max);
		getAngles(&p1, &angles);
		printf("Moving To: %0.1f, %0.1f, %0.1f, %0.1f\r\n", p1.x, p1.y, angles.x*180/M_PI, angles.y*180/M_PI);
		driveTo(&angles);
		count = 0;
		return WAIT_FOR_TRIGGER; // MAKE WAIT FOR TRIGGER AFTER
	case WAIT_FOR_TRIGGER:
		temp = getADC(IR2);
		if (temp > IR2THRESHOLD){
			count++;
			if (count > 5)
				return CLOSE_GRIPPER;
		}
		else
			count = 0;
		return WAIT_FOR_TRIGGER;
	case CLOSE_GRIPPER:
		// drop the arm slightly...
		_delay_ms(1000);


		//		driveMotor0(-0.5);
		//		_delay_ms(75); // dip
		//		driveMotor0(0);

		angles.x = angles.x - 0.35;
		driveTo(&angles);
		closeGripper();
		num = 0;
		_delay_ms(1000);
		return PICKUP_BLOCK;
	case PICKUP_BLOCK:
		curr = getADC(1);
		initPIDVar(&pv);

		initPIDVar(&pv2);

		c1 = getADC(3);
		c2 = getADC(2);

		pe1 = 2;
		pe2 = 2;
		while (pe1 != 0 || pe2 != 0){
			//THE TOLERANCE FOR THE FLOATS IS ALREADY ACCOUNTED FOR IN PID()
			if (flag == 1){
				c1 = getADC(3);
				c2 = getADC(2);
				pe1 = (pe1 == 0 ? 0 : PID(toADCValue(midway.x, 1), c1, KP, KI, KD, &pv));
				pe2 = (pe2 == 0 ? 0 : PID(toADCValue(midway.y, 2), c2, KP, KI, KD, &pv2));
				driveMotor0(pe1);
				driveMotor1(pe2);
				curr += getADC(1);
				num++;
				//printf("%d %d -> %d %d \r\n", toADCValue(p->x, 1), toADCValue(p->y, 2), c1, c2);
				//printf("%ld, %0.2f, %0.2f\r\n", counter0, pe1, pe2);
				flag = 0;
			}
			_delay_ms(1);
		}
		driveMotor0(0);
		driveMotor1(0);
		printf("Moved to Position\r\n");
		c1 = getADC(3);
		printf("Current: %d\r\n", c1);
		driveMotor1(0);
		_delay_ms(3000);
		c2 = getADC(3);

		driveMotor0(0);
		driveMotor1(0);
		printf("Final: %d\r\n", c2);
		if (c1 >= c2){
			printf("HEAVY!\r\n");
			return MOVE_TO_2;
		}
		else{
			printf("LIGHT!!\r\n");
			return MOVE_TO_1;
		}

		return END_STATE; // add current sense here
	case MOVE_TO_1:
		driveTo(&light);
		openGripper();
		return DROP_BLOCK;
	case MOVE_TO_2:
		driveTo(&heavy);
		openGripper();
		return DROP_BLOCK;
	case DROP_BLOCK:
		openGripper();
		runs++;
		if (runs == 2)
			return END_STATE;
		return GO_HOME;
	case END_STATE:
		break;
	}
	return s;
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
		//THE TOLERANCE FOR THE FLOATS IS ALREADY ACCOUNTED FOR IN PID()
		if (flag == 1){
			c1 = getADC(3);
			c2 = getADC(2);
			pe1 = (pe1 == 0 ? 0 : PID(toADCValue(p->x, 1), c1, KP, KI, KD, &pv));
			pe2 = (pe2 == 0 ? 0 : PID(toADCValue(p->y, 2), c2, KP, KI, KD, &pv2));
			driveMotor0(pe1);
			driveMotor1(pe2);

			//printf("%d %d -> %d %d \r\n", toADCValue(p->x, 1), toADCValue(p->y, 2), c1, c2);
			//printf("%ld, %0.2f, %0.2f\r\n", counter0, pe1, pe2);
			flag = 0;
		}
		_delay_ms(1);
	}
	driveMotor0(0);
	driveMotor1(0);

}

/**
 * if a is within b and c
 */
int IN_RANGE(float a, float b, float c){
	return a <= b && a >= c;
}

void initPIDVar(PIDVar* p){
	p->sum = 0;
	p->prev = 0;
	p->deltat = 0.01;
}


float PID(int setpoint, int curr, float kp, float ki, float kd, PIDVar* var){
	float deltat = var->deltat;
	float sum = var->sum;
	float prev = var->prev;
	float error = setpoint - curr;
	if (error <= PID_TOLERANCE && error >= -1*PID_TOLERANCE){
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
		setDAC(2, (int)(signal*4096));
		setDAC(3, 0);
	}
	else{
		setDAC(3, (int)(-1*signal*4096));
		setDAC(2, 0);
	}
}


void driveMotor1(float signal){

	if(signal < 0){
		setDAC(0, (int)(-1*signal*4096));
		setDAC(1, 0);
	}
	else{
		setDAC(1, (int)(signal*4096));
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


void setupBoard(){
	initSPI();
	initADC(1);
	initADC(2);
	initADC(3);
	initADC(7);
	initADC(6);
	timer0_init();

	//DDRD |= (1 << PD2) | (1 << PD3);

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
