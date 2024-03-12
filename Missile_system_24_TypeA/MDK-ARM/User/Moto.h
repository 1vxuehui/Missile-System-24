/**
  ******************************************************************************
  * File Name          : Moto.h
  * Description        : C620,C610_FOC Driver 电调(电机)驱动
  ******************************************************************************
 **/
 
#ifndef __Moto_H
#define __Moto_H

#define CHASSIS_CAN hcan2
#define GIMBAL_CAN hcan1

#include "stm32f4xx_hal.h"

/* CAN send and receive ID */
#define    CAN_YAW_ALL_ID  0x200
#define    CAN_SHOOT_ALL_ID  0x1FF
#define    CAN_PULL_SPRING_ID  0x205
#define    CAN_RELOAD_ID  0x206
#define    CAN_MISSILE_SHOOT_MOTOR_ID  0x207
#define    CAN_YAW_MOTOR_ID  0x208

/**********************************
电机数据结构体
*************************************/
typedef struct
{
	uint16_t	angle;//机械角
	int16_t		speed; //电机转速
	int16_t		current; //实际电流
	uint8_t		temperate; //电机温度
	uint8_t		empty;  //空
	uint16_t	angle_last;//上一次机械角
	int32_t		angle_sum;//角度积分
} Moto_DataTypedef;

typedef struct
{
	float POS_GAOL;//目标位置
	float POS_ABS;//绝对位置0
	float POS_OFFSET;
	float eer;
	float eer_eer;
}ANGLE_TypeDef;

 __weak CAN_HandleTypeDef hcan2;          //弱定义，防止CAN2不用时报错
 extern CAN_HandleTypeDef hcan1,hcan2;    //HAL库CAN结构体声明

extern Moto_DataTypedef Moto_Data1[8];
extern Moto_DataTypedef Moto_Data2[8];

void CAN_CMD_MOTO(CAN_HandleTypeDef *CAN_ID,uint32_t MOTO_ID,int16_t data0, int16_t data1, int16_t data2, int16_t data3);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_Filter_Init(CAN_HandleTypeDef *h_can);

#endif
