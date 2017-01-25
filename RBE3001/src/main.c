/*
 * main.c
 *
 *  Created on: Jan 18, 2017
 *      Author: vanc
 */
#include "RBELib/RBELib.h"
#include "RBELib/timer.h"
#include "RBELib/ADC.h"
#include <avr/io.h>
#include "RBELib/USARTDebug.h"
#include <string.h>
#include "main.h"

void writeToSerial();
void timer0_init();
void timer1_init();
void init_sc();
void printToSerial(char data[]);
void readPot();
void sqWave(float dc);
void initFreqPin();


volatile char buf[50];

volatile unsigned long counter0 = 0;
volatile unsigned long counter1 = 0;
int freq_div = 1000;

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
	if((counter0 % 100) == 0){
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
	if(sc.hrs >=23){
		sc.hrs = 0;
	}
}

// isr setup
ISR(TIMER0_COMPA_vect) {
	if (counter0 >= 65535||counter0 < 0){
		counter0 = 0;
	}
	counter0++; // increment our counter
	update_sc();
}

ISR(TIMER2_COMPA_vect) {
	if (counter1 >= freq_div||counter1 < 0){
		counter1 = 0;
	}
	counter1++; // increment our counter
}

int main(void){
	initRBELib();

	debugUSARTInit(115200);

	/** TRY RUNNING THIS ON THE OSCILLOSCOPE */

	DDRBbits._P4 = OUTPUT;
	PORTD &= ~0X07;
	DDRD &= 0x00;

	//timer1_init();
	//volatile unsigned long last = 0;
	sprintf(buf, "---------------");
	printToSerial(buf);
	PINBbits._P4 = 1;
	_delay_ms(100); //Delay .1 sec
	sprintf(buf, "%d", PINBbits._P4);
	printToSerial(buf);
	PINBbits._P4 = 0;
	_delay_ms(100);
	sprintf(buf, "%d", PINBbits._P4);
	printToSerial(buf);

	/*
	while(1){
		sprintf(buf, "%d", PINBbits._P4);
		//sqWave(0.5);
		//sprintf(buf, "%d,%d,%d", PIND&1, (PIND&2) >> 1, (PIND&4) >> 2);
		//last = counter1;
		printToSerial(buf);
//		PINBbits._P4 = 0;
	}
	 */
	return 0;
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

void timer1_init(){
	TCNT2 = 0;
	TCCR2A |= (1 << WGM21);// set to CTC mode
	TCCR2A |= (1 << COM2A1); // Configure timer 0 for CTC mode

	TCCR2B |= (1<<CS22) | (1<<CS20); // prescale by 1024 for 18kHz
	OCR2A = 144; // divide by 180 -1  to get 100 Hz count

	// 145/256 = 1k/x => x = 256k/145
	//counter0 = 0; // initialize timercount
	TIMSK2 |= (1 << OCIE2A); // Enable CTC interrupt
	//TIMSK1 |= (1 << TOIE1); // Enable CTC interrupt
	sei(); // Enable global interrupts
}

void readPot(){
	int channel = 7;
	init_sc();
	initADC(channel);
	timer0_init();
	while(1){
		unsigned short adc_val = getADC(channel);
		printToSerial(buf);
		float mv = adc_val/1024.0 *5.0;
		float angle = adc_val/1024.0 *270;
		sprintf(buf,"%02d:%02d:%02d, %u, %.2f, %.2f",
				(sc.hrs), (sc.min), (sc.sec),adc_val, mv, angle);
		printToSerial(buf);
		_delay_ms(500);
	}
}

void sqWave(float dc){
	if(counter1  < freq_div*dc){
		PINBbits._P4 = 1;
	}
	else{
		PINBbits._P4 = 0;
	}
}

void initFreqPin(){
	DDRAbits._P5 = OUTPUT;
}

