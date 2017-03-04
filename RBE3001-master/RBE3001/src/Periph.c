/*
 * periph.c
 *
 *  Created on: Feb 11, 2017
 *      Author: vanc
 */

#include "RBELib/RBELib.h"
#include "RBELib/Periph.h"
#include "RBELib/SPI.h"

#define cntr_byte 0x20
#define clear_byte 0x00
#define read_byte 0x40


signed int getAccel(int axis){

	int result;
	PORTD &= ~(1 << PD7);

	spiTransceive(((1 << 4) | (1 << 3) | axis) << 2);

	result = (spiTransceive(0) << 4);

	result |= ((spiTransceive(0) >> 4) & 0x0F);

	PORTD |= (1 << PD7);
	return result;

}

void encInit(int chan)
{
	//direction selection
	if(chan ==1)
	{
		DDRC |= (OUTPUT << DDC5 );
		PORTC |= (HIGH << PC5 );
	} else
	{
		DDRC |= (OUTPUT << DDC4 );
		PORTC |= (HIGH << PC4);
	}
	//slave select
	if(chan == 1)
	{
		PORTC &= ~(HIGH << PC5);
	} else
	{
		PORTC &= ~(HIGH << PC4);
	}
	//clear cntr
	spiTransceive(clear_byte | cntr_byte);

	//slave de-select
	if(chan == 1)
	{
		PORTC |= (HIGH << PC5);
	} else
	{
		PORTC |= (HIGH << PC4);
	}

}

void resetEncCount(int chan)
{

	//slave de-select
	if(chan == 1)
	{
		PORTC |= (HIGH << PC5);
	} else
	{
		PORTC |= (HIGH << PC4);
	}
	//slave select
	if(chan == 1)
	{
		PORTC &= ~(HIGH << PC5);
	} else
	{
		PORTC &= ~(HIGH << PC4);
	}

	spiTransceive(clear_byte);
	spiTransceive(0x00);
	//slave de-select
	if(chan == 1)
	{
		PORTC |= (HIGH << PC5);
	} else
	{
		PORTC |= (HIGH << PC4);
	}
}

signed long encCount(int chan)
{
	signed long temps[4];
	//slave de-select
	if(chan == 1)
	{
		PORTC |= (HIGH << PC5);
	} else
	{
		PORTC |= (HIGH << PC4);
	}
	//slave select
	if(chan)
	{
		PORTC &= ~(HIGH << PC5);
	} else
	{
		PORTC &= ~(HIGH << PC4);
	}

	spiTransceive(read_byte | cntr_byte);

	for(int i = 0; i < 4; i++)
		temps[i] = spiTransceive(0x00);

	//slave de-select
	if(chan == 1)
	{
		PORTC |= (HIGH << PC5);
	} else
	{
		PORTC |= (HIGH << PC4);
	}

	signed long ans = (signed long) (temps[0] << 24) | (temps[1] << 16) | (temps[2] << 8) | (temps[3]);
	return (chan == 1) ? ans : ans*-1;
}

