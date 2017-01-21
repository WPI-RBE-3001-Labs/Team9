/*
 * main.c
 *
 *  Created on: Jan 18, 2017
 *      Author: vanc
 */
#include "RBELib/RBELib.h"
#include "RBELib/timer.h"

void blinkTest();
void writeToSerial();
void turnOnLED();

int main(void){

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
void writeToSerial(){
	initRBELib();
	debugUSARTInit(115200);
	while (1){
		putCharDebug('a');
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
	sei();
	// Initialize timer count to 0
	TCNT0 = 0;
	TCCR0A = 0;
	TCCR0B = 0;
	// Sets timer to 15.625kHz





}
