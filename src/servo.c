/*
 * servo.c
 *
 *  Created on: Jan 3, 2014
 *      Author: carsten
 */
#include "stm32f10x.h"

#include "servo.h"

#include "pwm.h"


void updateServo(void);

int servoMax;
int servoMin;
int servoSpeed;
int servoPulse;

void servoInit(void) {
    int freq = pwmInit(50,PWM_RESOLUTION);

    servoMax = (2000 * freq) >> (20 - PWM_RESOLUTION);
    servoMin = (1000 * freq) >> (20 - PWM_RESOLUTION);
    servoSpeed = 1;
    servoPulse = servoMin;

    pwmInitIRQ(updateServo);
}

void updateServo(void) {
	static int servoDirection = 1;

	if (servoDirection) {
		servoPulse += servoSpeed;
		if (servoPulse >= servoMax) {
			servoDirection = 0;
			servoPulse = servoMax;
		}
	} else {
		servoPulse -= servoSpeed;
		if (servoPulse <= servoMin) {
			servoDirection = 1;
			servoPulse = servoMin;
		}
	}

	setPwm(1,servoPulse);
//	setPwm(2,(servoPulse - servoMin) << 4);
}

void toggleServoSpeed(void) {
	/* toogle servo speed */
	if (servoSpeed > 1000) {
		servoSpeed = 1;
	} else servoSpeed += servoSpeed;
}

uint16_t getServoPosition(void) {
	return (servoPulse - servoMin);
}

uint16_t getServoMinPosition(void) {
	return servoMin;
}

uint16_t getServoMaxPosition(void) {
	return servoMax;
}

uint8_t getServoSpeed(void) {
	return servoSpeed;
}

void setServoPosition(uint16_t value) {
	servoPulse = servoMin + value;
}

void setServoMinPosition(uint16_t value) {
	servoMin = value;
}

void setServoMaxPosition(uint16_t value) {
	servoMax = value;
}

void setServoSpeed(uint8_t value) {
	servoSpeed = value;
}


