/*
 * ADC.c
 *
 *  Created on: Jan 22, 2017
 *      Author: kacper
 */

#include "RBELib/RBELib.h"
#include <avr/io.h>
#include "RBELib/USARTDebug.h"
#include "RBELib/ADC.h"

void initADC(int channel)
{
	DDRA &= ~(1<<channel);
	DIDR0 |= (1 << channel);

	//specify 128kHz sample rate using internal divider
	ADCSRA |= (1 << ADPS2)| (1<<ADPS1)| (1<<ADPS0);

	ADMUX |= 1 << REFS0; //set 5V reference voltage
	ADMUX |= ~(1 << REFS1); //set 5V reference voltage

	ADMUX &= 0x80; // clear channel select
	ADMUX |= channel; //set multiplexer value

	//ADCSRA |= 1 << ADATE; //set auto trigger for source selection (intrpt flag)
	//ADCSRB |= ((1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0)); // timer 0 triggered
	ADCSRB |= ((1<<ADTS1)|(1<<ADTS0)); // timer 0 triggered
	ADCSRB &= ~(1<<ADTS2);
	ADCSRA |= 1 <<ADEN; //enable ADC

//	ADCSRA |= 1 <<ADIE; //enable ADC interrupts

	//ADCSRA |= 1<<ADSC;
}


//ISR(ADC_vect){
//
//}

unsigned short getADC(int channel)
{
	// Channel select
	ADMUX &= 0b11100000;
	ADMUX |= channel;

	// Start conversion
	ADCSRA |= 1<<ADSC;

	while(ADCSRA & (1<<ADSC));
	return ADC;

}


void clearADC(int channel)
{
//	DDRA |= (1 << channel);

	//Enable digital input on this channel
//	DIDR0 &= ~(1 << channel);
	ADMUX &= 0b11100000;
	ADMUX |= channel;
	ADCSRA &= (1<<(ADEN) & 0) | (1<<ADSC) | (1<<ADATE) | (1<<ADIF) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	//Clear ADC data registers
	ADCH = 0;
	ADCL = 0;
	ADC = 0;
}

void changeADC(int channel){};
