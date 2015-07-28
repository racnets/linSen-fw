/*
 * servo.h
 *
 *  Created on: Jan 3, 2014
 *      Author: carsten
 */

#ifndef SERVO_H_
#define SERVO_H_

void servoInit(void);
void updateServo(void);
void toggleServoSpeed(void);

uint16_t getServoPosition(void);
uint16_t getServoMinPosition(void);
uint16_t getServoMaxPosition(void);
uint8_t getServoSpeed(void);
void setServoPosition(uint16_t value);
void setServoMinPosition(uint16_t value);
void setServoMaxPosition(uint16_t value);
void setServoSpeed(uint8_t value);

#define PWM_RESOLUTION 16

#endif /* SERVO_H_ */
