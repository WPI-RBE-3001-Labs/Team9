/*
 * main.c
 *
 *  Created on: Jan 18, 2017
 *      Author: vanc
 */
#include "RBELib/RBELib.h"
#include "RBELib/timer.h"
#include <avr/io.h>
#include "RBELib/USARTDebug.h"
#include <string.h>
#include "main.h"

void blinkTest();
void writeToSerial();
void turnOnLED();
void timer0_init();
void init_sc();
void printToSerial(char data[]);

volatile unsigned long counter0 = 0;

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
}

int main(void){

	int channel = 0;
	init_sc();
	initADC(channel);

	writeToSerial();
	return 0;
}

void turnOnLED(){
	initRBELib();
	debugUSARTInit(115200);
	DDRB = 0xFF;
	while (1){
		PORTB = 0xFF;
		_delay_ms(500);
		PORTB = 0x00;
		_delay_ms(500);
	}

}

volatile char buf[50];

void writeToSerial(){
	initRBELib();
	debugUSARTInit(115200);
	timer0_init();
	volatile unsigned short adcval = 0;
	while (1){
		update_sc();
		adcval = getADCval(4);
		sprintf(buf,"%02d:%02d:%02d;%08u", (sc.hrs), (sc.min), (sc.sec), adcval);
		printToSerial(buf);
		_delay_ms(500);
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

void blinkTest(){
	initRBELib();
	debugUSARTInit(115200);

	//initTimer(0, CTC, 100);

	DDRBbits._P4 = OUTPUT;
	while(1){
		PINBbits._P4 = 0;
		_delay_ms(100);
		PINBbits._P4 = 1;
		_delay_ms(100);


	}
}

void sig_gen(int internalDiv, int dutyCylce){
	((counter0 % (1/internalDiv)) == 0){

	}
}

void timer0_init(){
	//cli();
	// Initialize timer count to 0
	TCNT0 = 0;
	TCCR0A |= (1 << WGM01);// set to CTC mode
	TCCR0A |= (1 << COM0A1); // Configure timer 0 for CTC mode

	TCCR0B |= (1<<CS02) | (1<<CS00); // prescale by 1024 for 18kHz
	OCR0A = 179; // divide by 180 -1  to get 100 Hz counter

	//counter0 = 0; // initialize timercount
	TIMSK0 |= (1 << OCIE0A); // Enable CTC interrupt
	sei(); // Enable global interrupts
}

void timer1_init()
{
	TCNT1 = 0;
	TCCR1A |= (1 << WGM01);// set to CTC mode
	TCCR1A |= (1 << COM0A1); // Configure timer 1 for CTC mode

	TCCR1B |= (1<<CS11) | (1<<CS10); // prescale by 64 for 288kHz
	OCR1A = 287; // divide by 18 -1  to get 100 kHz counter

	//counter0 = 0; // initialize timercount
	TIMSK1 |= (1 << OCIE1A); // Enable CTC interrupt
	sei(); // Enable global interrupts
}


