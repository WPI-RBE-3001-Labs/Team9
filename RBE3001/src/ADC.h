/*
 * ADC.h
 *
 *  Created on: Jan 23, 2017
 *      Author: kacper
 */

#ifndef ADC_H_
#define ADC_H_

void initADC(int channel);
unsigned long getADCval(int channel);
void clear(char channel);


#endif /* ADC_H_ */
