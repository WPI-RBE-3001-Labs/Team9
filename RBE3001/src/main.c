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

void blinkTest();
void writeToSerial();
void turnOnLED();
void timer0_init();
void init_sc();

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
//	if (counter0 >= 65535||counter0 < 0){
//		counter0 = 0;
//	}
	counter0++; // increment our counter
	update_sc();
}

int main(void){
	init_sc();
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
	while (1){
		int n = sprintf(buf,"%d",(sc.sec));
		if (n ==1) putCharDebug('0');
		for(int i = 0; i < n; i++)
			putCharDebug(buf[i]);
		putCharDebug(':');
		n = sprintf(buf,"%d",(sc.min));
		if (n ==1) putCharDebug('0');
		for(int i = 0; i < n; i++)
			putCharDebug(buf[i]);
		putCharDebug(':');
		n = sprintf(buf,"%d",(sc.hrs));
		if (n ==1) putCharDebug('0');
		for(int i = 0; i < n; i++)
			putCharDebug(buf[i]);
		putCharDebug('\n');
		putCharDebug('\r');
		_delay_ms(500);
	}
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
