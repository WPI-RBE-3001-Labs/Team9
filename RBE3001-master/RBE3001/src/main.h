/*
 * main.h
 *
 *  Created on: Jan 24, 2017
 *      Author: kacper
 */

#ifndef MAIN_H_
#define MAIN_H_

#define PID_TOLERANCE 10

#define KP 1.0
#define KI 0.005
#define KD 0.05


#include "RBELib/RBELib.h"
#include "RBELib/timer.h"
#include "RBELib/ADC.h"
#include "RBELib/DAC.h"
#include "RBELib/SPI.h"
#include <avr/io.h>
#include "RBELib/USARTDebug.h"
#include <string.h>
#include <math.h>
#include "RBELib/Periph.h"
#include "Conversions.h"
#include "Arm.h"
#include "Actuators.h"

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


void printForwardPosition();

void initPIDVar(PIDVar* p);

//float PID(int setpoint, int curr, float kp, float ki, float kd);
void driveTo(Point* p);


void setupBoard();

float PID(int setpoint, int curr, float kp, float ki, float kd, PIDVar* var);
void driveMotor0(float signal);
void driveMotor1(float signal);


#endif /* MAIN_H_ */
