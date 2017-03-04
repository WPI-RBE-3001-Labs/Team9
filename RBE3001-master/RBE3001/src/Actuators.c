/*
 * Actuators.c
 *
 *  Created on: Feb 28, 2017
 *      Author: vanc
 */
#include "Actuators.h"


void closeGripper(){
	setServo(4, 180);
}

void openGripper(){
	setServo(4, 30);
}

void runBelt(){
	setServo(3, 10);
}

void stopBelt(){
	setServo(3, 90);
}
