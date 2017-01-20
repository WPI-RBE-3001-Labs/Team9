/*
 * USART.c
 *
 *  Created on: Jan 26, 2015
 *      Author: joest
 */


#include "RBELib/RBELib.h"
//#include <avr/io.h>

void debugUSARTInit(unsigned long baudrate){
  UBRR0H = (unsigned char)(baudrate >> 8);
  UBRR0L = (unsigned char)(baudrate);
  UCSR0B = (1 << RXEN0)|(1 << TXEN0);
  UCSR0C = (1 << USBS0)|(3 << UCSZ00);
}

void putCharDebug(char byteToSend){
	while (!(UCSR0A & (1 < UDRE)));
	UDR0 = byteToSend;
 
}

unsigned char getCharDebug(void) {
   return 'a';
}
