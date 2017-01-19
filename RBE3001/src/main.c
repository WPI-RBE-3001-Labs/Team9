/*
 * main.c
 *
 *  Created on: Jan 18, 2017
 *      Author: vanc
 */
#include "RBELib/RBELib.h"
#include "RBELib/timer.h"

int main(void){


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
		char c = getCharDebug();
		putCharDebug(c);
	}
}
void blink(){
	initRBELib();
	debugUSARTInit(115200);

	initTimer(0, CTC, 100);

	DDRBbits._P4 = OUTPUT;
	while(1){
		PINBbits._P4 = 0;
		_delay_ms(100);
		PINBbits._P4 = 1;
		_delay_ms(100);
	}
}
