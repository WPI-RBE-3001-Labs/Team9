/*
 * SPI.c
 *
 *  Created on: Jan 27, 2017
 *      Author: kacper
 */


#include "RBELib/RBELib.h"
#include <avr/io.h>
#include "RBELib/USARTDebug.h"
#include "RBELib/DAC.h"

void initSPI()
{
	DDRB =	((1 << DDB7	)| //SCLK
			(0 <<  DDB6	)| //MISO
			(1 <<  DDB5	)| //MOSI
			(1 <<  DDB4	)| //SS
			(0 <<  DDB3	)|
			(0 <<  DDB2	)|
			(0 <<  DDB1	)|
			(0 <<  DDB0	));

	DDRD = ((0 << DDD7	)| //CS for accel
			(0 <<  DDD6	)|
			(0 <<  DDD5	)|
			(1 <<  DDD4	)| //CS for DAC
			(0 <<  DDD3	)|
			(0 <<  DDD2	)|
			(0 <<  DDD1	)|
			(0 <<  DDD0	));


	PRR &= 	~(1 << PRSPI);	//power reduction settings to enable SPI
	SPCR = 	(1 << SPE)| 	//enable SPI
			(1 << MSTR);	//select Master SPI mode

	//	SPSR = 	(1 << SPI2X);	//double SPI speed

	PORTD |= (1 << PD4)	;	//chip select is low enable
}

unsigned char spiTransceive(BYTE data)
{
	SPDR = data;
	while(!(SPSR & (1 << SPIF)));
	unsigned char stat_reg = SPSR; //read these to clear interrupts
	unsigned char data_reg = SPDR;

	return data_reg; // return sent data

}

