/*
 * main.h
 *
 *  Created on: Jan 24, 2017
 *      Author: kacper
 */

#ifndef MAIN_H_
#define MAIN_H_

#define LINK_LENGTH_1 150
#define LINK_LENGTH_2 120
#define LINK_OFFSET 140

#include "adc_custom.h"

typedef struct {
	float x;
	float y;
} Point;

void writeToSerial();
void timer0_init();

void timer2_init();

void init_sc();
void printToSerial(char data[]);

float toRadians(int adcval, int link);

float PID(int setpoint, int curr, float kp, float ki, float kd);
void driveMotor0(float signal);

void getEndPosition(Point* p, int l1, float a1, int l2, float a2);
void buttonSM2();



#endif /* MAIN_H_ */
