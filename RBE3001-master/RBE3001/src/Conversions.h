/*
 * Conversions.h
 *
 *  Created on: Feb 25, 2017
 *      Author: vanc
 */

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include "Arm.h"
#include <math.h>
#include "RBELib/ADC.h"

typedef struct {
	float x;
	float y;
} Point;


float toDegrees(int adcval, int link);
float toRadians(int adcval, int link);

int toADCValue(float degrees, int link);

// Inverse and Forward Kinematics
void getAngles(Point* desired, Point* angles);
void getEndPosition(Point* p, int l1, float a1, int l2, float a2);

int getPositionOnBelt(int belt);

// Get Arm Position from Weight Position wrt Belt
void getPosition(Point* arm, int belt);


#endif /* CONVERSIONS_H_ */
