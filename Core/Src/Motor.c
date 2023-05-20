/*
 * Motor.c
 *
 */
/*
#include <Motor.h>
#include "tim.h"
#include <math.h>
#include <stdbool.h>
float epsilon = 1;

void actuation_motor1(PI_Param_t PI, Motor_t Motor)
{
	Motor->LevelDelta = Motor->LevelCurrent - Motor->LevelTarget ;
	if(fabsf(Motor->LevelDelta) > epsilon)
	{
		if(PI->saturation == false && (signbit(PI->prev_output) != signbit(Motor->LevelDelta)))
		{
			PI->integral += Motor->LevelDelta*PI->deltaT;
			PI->derivative += (Motor->LevelDelta - PI->prev_error)/PI->deltaT;
			Motor->PowerCurrent = (PI->Kp*Motor->LevelDelta) + (PI->Ki*PI->integral) + (PI->Kd*PI->derivative);
			if(Motor->PowerCurrent > Motor->PowerMax)
			{
				Motor->PowerCurrent = Motor->PowerMax;
				PI->saturation = true;
			}
			else if (Motor->PowerCurrent < Motor->PowerMin)
			{
				Motor->PowerCurrent = Motor->PowerMin;
				PI->saturation = true;
			}
			else
			{
				PI->saturation = false;
				PI->prev_output = Motor->PowerCurrent;

			}
			PI->prev_error = Motor->LevelDelta;
		}
		TIM2->CCR1 = Motor->PowerCurrent;

	}
	else
	{
		TIM2->CCR1 = 0;
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6 ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7 ,GPIO_PIN_RESET);
	}
}


// float Map(float x, float in_min, float in_max, float out_min, float out_max)
// {
//	return ((((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min);
//}
 *
 *//* Da cambiare con funzione acquisizione livello */

// void MapL(float data1, float data2, float levelsensor){
//	float *x1;
//	float *x2;
//	*x1 = levelsensor - data1;
//	*x2 = levelsensor - data2;
//}

