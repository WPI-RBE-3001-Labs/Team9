/*
 * ADC.c
 *
 *  Created on: Jan 22, 2017
 *      Author: kacper
 */

#include "RBELib/RBELib.h"
#include <avr/io.h>

void init_ADC(char channel)
{
	//specify 128kHz sample rate using internal divider
	ADCSRA |= (1 << ADPS2)| (1<<ADPS1)| (1<<ADPS0);

	ADMUX |= 1 << REFS0; //set 5V reference voltage

	ADMUX &= 0xFB; //clear ADC
	ADMUX |= channel; //set multiplexer value

	ADCSRA |= 1 << ADATE; //set auto trigger for source selection (intrpt flag)
	ADCSRB &= ~((1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0)); // free running
	cli();
	ADCSRA |= 1 <<ADEN; //enable ADC
	ADCSRA |= 1 <<ADIE; //enable ADC interrupts
	sei();
	ADCSRA |= 1<<ADSC;
}

//ISR(ADC_vect){
//
//}

void getVal(char channel)
{
	ADMUX &= 0b11100000;
	ADMUX |= channel;
}

void clear(char channel)
{
	DDRA |= (1 << channel);

	//Enable digital input on this channel
	DIDR0 &= ~(1 << channel);

	//Clear ADC data registers
	ADCH = 0;
	ADCL = 0;
	ADC = 0;
}
