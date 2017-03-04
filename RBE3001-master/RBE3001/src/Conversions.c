/*
 * Conversions.c
 *
 *  Created on: Feb 25, 2017
 *      Author: vanc
 */

#include "Conversions.h"


int getPositionOnBelt(int belt){
	int v = 250 - belt;
	// 50mm = 70 IR Val

	return v/60.0 * 50.0;
}

void getPosition(Point* arm, int belt){
	// 6.5
	arm->x = 160 + getPositionOnBelt(belt)/5.0;
	arm->y = -1*(220 + getPositionOnBelt(belt));
}

int toADCValue(float radians, int link){
	if (link == 2)
		return (int)((radians*180/M_PI)*(800.0/180) + LINK2_CENTER);
	return (int)((radians*180/M_PI)*(800.0/180) + LINK1_CENTER);
}

float toDegrees(int adcval, int link){
	if (link == 2)
		return (((adcval - LINK2_CENTER)/800.0 * 180));
	return (((adcval - LINK1_CENTER)/800.0 * 180));
}

float toRadians(int adcval, int link){
	return toDegrees(adcval, link)*M_PI/180;
}

//forward kinematics
void getEndPosition(Point* p, int l1, float a1, int l2, float a2){
	p->x = l1*cos(a1) + l2*cos(a2 + a1) + LINK_OFFSET;
	p->y = l1*sin(a1) + l2*sin(a2 + a1);
}

void getAngles(Point* desired, Point* angles){
	float x = desired->x - LINK_OFFSET;
	float y = desired->y;
	//	float l1 = LINK_LENGTH_1;
	//	float l2 = LINK_LENGTH_2;
	//	float t1 = (x*x + y*y - l1*l1 - l2*l2)/(2*l1*l2);
	//	float a2 = -1*(acos(t1) - M_PI/2);
	//			//atan2(sqrt(1 - t1*t1), t1);
	//
	//	float ap1 = atan2(y, x)  - acos((x*x + y*y + l1*l1 - l2*l2)/(2*l1*sqrt(x*x + y*y)));
	//	float ap2 = atan2(y, x)  + acos((x*x + y*y + l1*l1 - l2*l2)/(2*l1*sqrt(x*x + y*y)));
	//
	//	printf("1: %0.2f, 2: %0.2f theta2 = %0.2f\r\n", ap1, ap2*180/M_PI, a2*180/M_PI);
	//
	//	float a1 = -1*(ap2 - M_PI/2);

	float link1 = LINK_LENGTH_1; // mm
	float link2 = LINK_LENGTH_2; // mm
	float x1 = sqrt(1 - pow((pow(x, 2) + pow(y, 2) - pow(link1, 2) - pow(link2, 2)) / (2 * link1 * link2), 2));
	float x2 = (pow(x, 2) + pow(y, 2) - pow(link1, 2) - pow(link2, 2)) / (2 * link1 * link2);
	float t2pos = atan2(x1, x2);
	float t2neg = atan2(-x1, x2);
	float t1pos = atan2((float)y, (float)x) - atan2(link2 * sin(t2pos), link1 + link2 * cos(t2pos));
	float t1neg = atan2((float)y, (float)x) - atan2(link2 * sin(t2neg), link1 + link2 * cos(t2neg));

	if (IN_RANGE(t1pos, M_PI/2, -M_PI/2) && IN_RANGE(t2pos, M_PI/2, -M_PI/2) && IN_RANGE(t1neg, M_PI/2, -M_PI/2) && IN_RANGE(t2neg, M_PI/2, -M_PI/2))
	{
		if (y <= 0)
		{
			if (t1pos - t2pos >= 0){
				angles->x = t1pos;
				angles->y = t2pos;
			}
			else{
				angles->x = t1neg;
				angles->y = t2neg;
			}
		} else
		{
			if (t1pos - t2pos <= 0){
				angles->x = t1pos;
				angles->y = t2pos;
			}else{
				angles->x = t1neg;
				angles->y = t2neg;
			}
		}
	} else if (IN_RANGE(t1pos, M_PI/2, -M_PI/2) && IN_RANGE(t2pos, M_PI/2, -M_PI/2))
	{
		angles->x = t1pos;
		angles->y = t2pos;

	} else if (IN_RANGE(t1neg, M_PI/2, -M_PI/2) && IN_RANGE(t2neg, M_PI/2, -M_PI/2))
	{
		angles->x = t1neg;
		angles->y = t2neg;
	}
	else
	{
		angles->x = t1neg;
		angles->y = t2neg;
	}

}
