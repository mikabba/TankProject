/*
 * Motor.h
 *
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#include "gpio.h"
#include <math.h>
#include <stdbool.h>

volatile typedef struct Motor
{
	float LevelTarget;
	float LevelCurrent;
	float LevelDelta;

	int Dir;
	int DirSens;

	float PowerTarget;
	float PowerCurrent;
	float PowerMax;
	float PowerMin;

	volatile int ATotalCountTick;
	volatile int ATickForLevel;
	float Speed;

}Motor_t;

volatile typedef struct PI
{
	float Kp;
	float Ki;
	float Kd;
	float deltaT;
	float prev_error;
	float integral;
	float prev_output;
	float derivative;
	bool saturation;
}PI_Param_t;


void PI_Init(PI_Param_t* PI);

void Motor1_Init(Motor_t* Motor);

void actuation_motor1(PI_Param_t* PI, Motor_t* Motor);

// float Map(float x, float in_min, float in_max, float out_min, float out_max); /* Da cambiare con sensore*/

void MapL(float data1, float data2, float levelsensor);

#endif /* SRC_MOTOR_H_ */
