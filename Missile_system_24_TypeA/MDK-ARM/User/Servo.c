/***********************************************************
文件名：Servo.c
描述：
		舵机控制实现
From:Qi-Q@Rjgawuie
***********************************************************/
#include "Servo.h"

/***********************************************************
函数名：Servo_Init
功能：舵机PWM初始化，使用前调用
参数：servohandler 	舵机控制句柄地址
			htimhandler 	定时器句柄地址
			Chn 					通道号
返回值: 初始化是否成功
***********************************************************/
HAL_StatusTypeDef Servo_Init(Servo_HandlerTypedef *servohandler,TIM_HandleTypeDef *htimhandler, uint32_t Chn)
{
	servohandler->timhandler = htimhandler;
	servohandler->Chn = Chn;
	return HAL_TIM_PWM_Start(htimhandler,Chn);
}

/***********************************************************
函数名：Servo_Init
功能：舵机PWM初始化，使用前调用
参数：htimhandler 	定时器句柄地址
			Chn 					通道号
返回值: 初始化是否成功
***********************************************************/
void Servo_Set_Angle(Servo_HandlerTypedef *servohandler,float angle)
{
	uint16_t Cmp_v;
	Cmp_v = 500 + (uint16_t)(11.111111f * angle);
	__HAL_TIM_SetCompare(servohandler->timhandler,servohandler->Chn,Cmp_v);
}
