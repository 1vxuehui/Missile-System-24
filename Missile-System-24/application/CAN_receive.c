/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"


#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
�������, 0:�����yaw���� 3508���,1:����ܵ��ɵ�� 3508�����  2:������� 3508���,3:������ 4 3508���;*/
motor_measure_t motor_launcher[10];
static CAN_TxHeaderTypeDef  launcher_tx_message;
static uint8_t              launcher_can_send_data[8];

static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];


fp32 angle;
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
		if(hcan==&hcan1)
		{	
			switch (rx_header.StdId)
			{
					case CAN_YAW_3508_ID:
					case CAN_3508_ID:
					{
							static uint8_t i = 0;
							//get motor id
							i = rx_header.StdId - CAN_YAW_3508_ID;
							get_motor_measure(&motor_launcher[i], rx_data);
							detect_hook(LAUNCHER_MOTOR1_TOE + i);
							break;
					}
					default:
					{
							break;
					}
			}
		}
		else if(hcan==&hcan2)
		{
			switch (rx_header.StdId)
			{
				case CAN_SHOOT_MOTOR_ID:
				{
						get_motor_measure(&motor_launcher[2], rx_data);
						detect_hook(SHOOT_MOTOR_TOE);
						break;
				}
				case CAN_RELOAD_MOTOR_ID:
				{
						get_motor_measure(&motor_launcher[3], rx_data);
						detect_hook(RELOAD_MOTOR_TOE);
						break;
				}
				default:
				{
						break;
				}
			}
		}
}


/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 3508������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      spring: (0x206) 3508������Ƶ���, ��Χ [-30000,30000]
  * @retval         none
  */
void CAN_cmd_launcher(int16_t yaw, int16_t spring)
{
    uint32_t send_mail_box;
    launcher_tx_message.StdId = CAN_LAUNCHER_ALL_ID;
    launcher_tx_message.IDE = CAN_ID_STD;
    launcher_tx_message.RTR = CAN_RTR_DATA;
    launcher_tx_message.DLC = 0x08;
    launcher_can_send_data[0] = (yaw >> 8);
    launcher_can_send_data[1] = yaw;
    launcher_can_send_data[2] = (spring >> 8);
    launcher_can_send_data[3] = spring;
    HAL_CAN_AddTxMessage(&LAUNCHER_CAN, &launcher_tx_message, launcher_can_send_data, &send_mail_box);
}

/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      shoot: (0x201) 2006������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      reload: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_shoot(int16_t shoot, int16_t reload)
{
    uint32_t send_mail_box;
    shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
    shoot_can_send_data[0] = shoot >> 8;
    shoot_can_send_data[1] = shoot;
    shoot_can_send_data[2] = reload >> 8;
    shoot_can_send_data[3] = reload;

    HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}


/**
  * @brief          ����yaw 3508�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_yaw_motor_measure_point(void)
{
    return &motor_launcher[0];
}

/**
  * @brief          ���ص��� 3508�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_spring_motor_measure_point(void)
{
    return &motor_launcher[1];
}

/**
  * @brief          ���ػ��� 3508�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_reload_measure_point(void)
{
    return &motor_launcher[2];
}

/**
  * @brief          ���ط��� 3508�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_shoot_measure_point(void)
{
    return &motor_launcher[3];
}
