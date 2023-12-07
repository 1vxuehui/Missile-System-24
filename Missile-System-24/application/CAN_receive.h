/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define LAUNCHER_CAN hcan1
#define SHOOT_CAN 	hcan2


/* CAN send and receive ID */
typedef enum
{
    CAN_LAUNCHER_ALL_ID = 0x200,
    CAN_YAW_3508_ID = 0x201,
    CAN_3508_ID = 0x202,
	
		CAN_SHOOT_ALL_ID = 0x200,
    CAN_SHOOT_MOTOR_ID = 0x201,
		CAN_RELOAD_MOTOR_ID = 0x204,

} can_msg_id_e;

//RM 电机数据
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;


/**
  * @brief          发送电机控制电流(0x201,0x202)
  * @param[in]      yaw: (0x205) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      spring: (0x206) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_launcher(int16_t yaw, int16_t spring);

/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      shoot: (0x201) 2006电机控制电流, 范围 [-16384,16384]
  * @param[in]      reload: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_shoot(int16_t shoot, int16_t reload);

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_motor_measure_point(void);

/**
  * @brief          返回弹簧 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_spring_motor_measure_point(void);

/**
  * @brief          返回换弹 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_reload_measure_point(void);

/**
  * @brief          返回发射 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_shoot_measure_point(void);

#endif
