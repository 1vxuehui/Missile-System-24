/***********************************************************
文件名：Servo.h
描述：
		舵机控制实现
From:Qi-Q@Rjgawuie
***********************************************************/
#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "stm32f4xx_hal.h"


typedef struct
	{
		TIM_HandleTypeDef *timhandler;
		uint32_t Chn;
	}Servo_HandlerTypedef;

HAL_StatusTypeDef Servo_Init(Servo_HandlerTypedef *servohandler,TIM_HandleTypeDef *htimhandler, uint32_t Chn);
void Servo_Set_Angle(Servo_HandlerTypedef *servohandler,float angle);

#endif
