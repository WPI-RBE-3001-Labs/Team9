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

#define KP 2
#define KI 0.5
#define KD 0.5
#include "adc_custom.h"

typedef struct {
	float x;
	float y;
} Point;

typedef struct{
	float sum;
	float prev;
	float deltat;
} PIDVar;

void writeToSerial();
void timer0_init();

void timer2_init();

void init_sc();
void printToSerial(char data[]);

void triangle();

void printForwardPosition();
void getAngles(Point* desired, Point* angles);

float toDegrees(int adcval, int link);
float toRadians(int adcval, int link);
int toADCValue(float degrees, int link);
void initPIDVar(PIDVar* p);

//float PID(int setpoint, int curr, float kp, float ki, float kd);
float PID(int setpoint, int curr, float kp, float ki, float kd, PIDVar* var);
void driveMotor0(float signal);
void driveMotor1(float signal);

void getEndPosition(Point* p, int l1, float a1, int l2, float a2);
void buttonSM2();
void pointToPoint();


#endif /* MAIN_H_ */
