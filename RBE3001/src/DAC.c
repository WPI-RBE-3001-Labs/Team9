/*
 * DAC.c
 *
 *  Created on: Jan 27, 2017
 *      Author: kacper
 */

#include "RBELib/RBELib.h"
#include <avr/io.h>
#include "RBELib/USARTDebug.h"
#include "RBELib/DAC.h"

void setDAC(int DACn, int SPIVal)
{

	if(SPIVal >= 4096) SPIVal = 4095;
	if(SPIVal < 0) SPIVal = 0;

	unsigned long shift = SPIVal;	// store the 16 bit value
	shift = shift << 4; // shift up the stored value
	BYTE c_a = 0x30;	// set up command and address info 0011 0000
	c_a |= DACn;		// select DAC


	BYTE upper, lower;	// set the data bytes

	lower = (shift & 0xFF00) >> 8;	// clear the upper byte, shift down 8 bits
	upper =  (shift & 0x00FF);		// clear lower byte

	PORTD &= ~(1 << PD4); // set slave select to 0

	spiTransceive(c_a);		// send command and address
	spiTransceive(lower);	// send lower byte
	spiTransceive(upper); 	// send upper byte

	PORTD |= (1 << PD4); // toggle slave select

}
