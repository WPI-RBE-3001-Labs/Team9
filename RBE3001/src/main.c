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

void writeToSerial();
void timer0_init();

void timer2_init();

void init_sc();
void printToSerial(char data[]);
void readPot();
void sqWave(float dc);
void initFreqPin();

void buttonSM();

float PID(int setpoint, int curr, float kp, float ki, float kd);
void driveMotor0(float signal);

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
	initADC(2);
	initSPI();
	timer0_init();
	printf("--------------\r\n");
	int center = 580;
	while(1){
		printf("Flag: %d\r\n", flag);
		if (flag == 1){
			int adcval = getADC(2);
			float p = PID(center, adcval, .5, 0, 0);
			printf("ADCVAL: %d, PID: %0.2f\r\n", adcval, p);
			driveMotor0(p);
			flag = 0;
		}
		//		setDAC(0, 0);
		//		setDAC(1, 0);
		//		setDAC(2, 0);
		//		setDAC(3, 0);
		//		int adcval = getADC(2);
		//		float angle = (adcval - center)/1023.0 * 270;
		//		printf("ADCVAL: %d, Voltage: %0.1f, Angle: %0.1f\r\n", adcval, adcval*5/1023.0, angle);
		//		_delay_ms(100);

	}
	return 0;
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

void collectADC()
{
	DDRD &= ~((1<<DDD4));
	PORTD |= ((1<<PD4));
	initADC(channel);
	while((PIND>>PD4)&1 == 1){
		_delay_ms(5);
	}
	timer2_init();
	ADCtimer_init();
	volatile unsigned long last = counter0;

	while(counter0 < 225){
		if(counter0 > last){
			adcdatas[last] = getADC(channel);
			timedatas[last] = counter0;
			last = counter0;
		}
	}
	sprintf(buf, "---------------");
	printToSerial(buf);
	for(int i = 0; i < 225; i++){
		sprintf(buf,"%02d, %02d", timedatas[i], adcdatas[i]);
		printToSerial(buf);
	}

}

void writeToSerial(){

	timer0_init();
	sprintf(buf,"%02d:%02d:%02d;", (sc.hrs), (sc.min), (sc.sec));
	printToSerial(buf);
	_delay_ms(500);
}


void printToSerial(char data[]){
	int i;
	for(i = 0; i<strlen(data); i++){
		putCharDebug(data[i]);
	}
	putCharDebug('\n');
	putCharDebug('\r');

}

void buttonSM() {
	char state = 0;
	DDRBbits._P4 = OUTPUT;
	PORTD &= ~0X07;
	DDRD &= 0x00;
	init_sc();
	initADC(channel);
	//	timer0_init();
	timer2_init();
	while(1)
	{
		unsigned short adc_val = getADC(channel);
		int button1 = (PIND>>PD5)&1;
		int button2 = (PIND>>PD6)&1;
		int button3 = (PIND>>PD7)&1;
		float dc = adc_val/1023.0;
		if( ((button1)==0) && ((button2)==1) &&((button3)==1) ){
			state = 1;
		}
		if( ((button2)==0) && ((button1)==1) &&((button3)==1) ){
			state = 2;
		}
		if( ((button3)==0) && ((button1)==1) &&((button2)==1) ){
			state = 3;
		}


		switch(state){
		case 1:
			freq_div = 10000;
			break;
		case 2:
			freq_div = 500;
			break;
		case 3:
			freq_div = 100;
			break;
		default:
			freq_div = 1000;
		}
		sqWave(dc);
		int freq = 10000/freq_div;
		int output = 0;
		if(counter1  < (int) freq_div*dc)
			output = 1;
		sprintf(buf, "%.2f, %d, %d, %d",
				dc, freq, output, adc_val);
		printToSerial(buf);
	}

}


void readPot(){
	init_sc();
	initADC(channel);
	timer0_init();
	while(1){
		unsigned short adc_val = getADC(channel);
		printToSerial(buf);
		float mv = adc_val/1023.0 *5.0;
		float angle = adc_val/1023.0 *270;
		sprintf(buf,"%02d:%02d:%02d, %u, %.2f, %.2f",
				(sc.hrs), (sc.min), (sc.sec),adc_val, mv, angle);
		printToSerial(buf);
		_delay_ms(500);
	}
}

void sqWave(float dc){
	if(dc > 1.0) dc = 1.0;
	if(counter1  < (int) freq_div*dc){
		PORTB |= (1 << PB4);
	}
	else{
		PORTB &= ~(1 << PB4);
	}
}

void initFreqPin(){
	DDRAbits._P5 = OUTPUT;
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
