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
�������, 0:�����yaw���� M15���, 1:����ܵ��ɵ�� 3508���,2:������ ����3508���,3�������� ����6020���*/
motor_measure_t motor_chassis[10];
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
					case CAN_M15_ID:
          {
							get_motor_measure(&motor_chassis[0], rx_data);
							detect_hook(LAUNCHER_M15_TOE);
							break;
					}
					case CAN_3508_ID:
					{
							get_motor_measure(&motor_chassis[1], rx_data);
							detect_hook(LAUNCHER_3508_TOE);
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
				case CAN_3508_SHOOT_ID:
				{
						get_motor_measure(&motor_chassis[2], rx_data);
						detect_hook(SHOOT_RELOAD_TOE);
						break;
				}
				case CAN_6020_RELOAD_ID:
				{
						get_motor_measure(&motor_chassis[3], rx_data);
						detect_hook(SHOOT_SHOOT_TOE);
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
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    launcher_tx_message.StdId = 0x700;
    launcher_tx_message.IDE = CAN_ID_STD;
    launcher_tx_message.RTR = CAN_RTR_DATA;
    launcher_tx_message.DLC = 0x08;
    launcher_can_send_data[0] = 0;
    launcher_can_send_data[1] = 0;
    launcher_can_send_data[2] = 0;
    launcher_can_send_data[3] = 0;
    launcher_can_send_data[4] = 0;
    launcher_can_send_data[5] = 0;
    launcher_can_send_data[6] = 0;
    launcher_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&LAUNCHER_CAN, &launcher_tx_message, launcher_can_send_data, &send_mail_box);
}

/**
  * @brief          ����M15���ģʽ
  * @param[in]      mode_value:ģʽֵ��������0x00����������0x01���ٶȻ���0x02��λ�û���0x03��
  * @retval         none
  */
void M15_set_mode(int16_t mode_value)
{
    uint32_t send_mail_box;
    launcher_tx_message.StdId = 0x105;
    launcher_tx_message.IDE = CAN_ID_STD;
    launcher_tx_message.RTR = CAN_RTR_DATA;
    launcher_tx_message.DLC = 0x08;
    launcher_can_send_data[0] = mode_value;
    launcher_can_send_data[1] = mode_value;
    launcher_can_send_data[2] = mode_value;
    launcher_can_send_data[3] = mode_value;
    launcher_can_send_data[4] = mode_value;
    launcher_can_send_data[5] = mode_value;
    launcher_can_send_data[6] = mode_value;
    launcher_can_send_data[7] = mode_value;

    HAL_CAN_AddTxMessage(&LAUNCHER_CAN, &launcher_tx_message, launcher_can_send_data, &send_mail_box);
}

/**
  * @brief          ����M15������Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) M15������Ƶ���, ��Χ [0,32767]
  * @retval         none
  */
void CAN_cmd_M15(int16_t motor1)
{
    uint32_t send_mail_box;
    launcher_tx_message.StdId = 0x32;
    launcher_tx_message.IDE = CAN_ID_STD;
    launcher_tx_message.RTR = CAN_RTR_DATA;
    launcher_tx_message.DLC = 0x08;
    launcher_can_send_data[0] = motor1 >> 8;
    launcher_can_send_data[1] = motor1;

    HAL_CAN_AddTxMessage(&LAUNCHER_CAN, &launcher_tx_message, launcher_can_send_data, &send_mail_box);
}

/**
  * @brief          ����3508������Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_3508(int16_t motor2)
{
    uint32_t send_mail_box;
    launcher_tx_message.StdId = CAN_LAUNCHER_ALL_ID;
    launcher_tx_message.IDE = CAN_ID_STD;
    launcher_tx_message.RTR = CAN_RTR_DATA;
    launcher_tx_message.DLC = 0x08;
    launcher_can_send_data[2] = motor2 >> 8;
    launcher_can_send_data[3] = motor2;

    HAL_CAN_AddTxMessage(&LAUNCHER_CAN, &launcher_tx_message, launcher_can_send_data, &send_mail_box);
}

/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      shoot: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      reload: (0x202) 6020������Ƶ���, ��Χ [-16384,16384]
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
  * @brief          ����yaw M15�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_yaw_M15_motor_measure_point(void)
{
    return &motor_chassis[0];
}

/**
  * @brief          ���ص��� 3508�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_spring_motor_measure_point(void)
{
    return &motor_chassis[1];
}

/**
  * @brief          ���ط��� 3508�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_can_2006_measure_point(void)
{
    return &motor_chassis[2];
}

/**
  * @brief          ���ػ��� 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_can_3508_left_measure_point(void)
{
    return &motor_chassis[3];
}
 
 