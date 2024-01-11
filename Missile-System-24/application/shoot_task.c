/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot_task.c/h
  * @brief      �������.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot_task.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "launcher_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "tim.h"
#include "stm32.h"

#define shoot_missile(speed) 	shoot_control.missile_shoot_speed_set = speed; 
#define reload_motor(speed)				shoot_control.reload_speed_set = -speed //�����������  
//�г̿���IO
#define BUTTEN_TRIG_PIN     HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)


static void shoot_init(void);

///**
//  * @brief          ģʽ�л�
//  * @param[in]      void
//  * @retval         void
//  */
//static void shoot_set_mode(void);

/**
  * @brief          ����ٶȼ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_control_loop(void);

/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void); 





shoot_control_t shoot_control;          //�������
static int16_t shoot_can_set_current = 0 ,reload_can_set_current = 0;
/**
  * @brief          �������
  * @param[in]      void
  * @retval         ����can����ֵ
  */
void shoot_task(void const *pvParameters)
{
		vTaskDelay(SHOOT_TASK_INIT_TIME);
		shoot_init();
		while(1)
		{
//				shoot_set_mode();
				shoot_feedback_update();
				shoot_control_loop();		 //���÷���������
			  if (!(toe_is_error(SHOOT_MOTOR_TOE) && toe_is_error(RELOAD_MOTOR_TOE)))
        {
            if (toe_is_error(DBUS_TOE))
            {
                CAN_cmd_shoot(0, 0);
            }
            else
            {
                CAN_cmd_shoot(shoot_can_set_current, reload_can_set_current);
            }
        }
				vTaskDelay(SHOOT_CONTROL_TIME_MS);
		}
}

/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */ 
void shoot_init(void)
{
		//���PID��ʼ��
		static const fp32 reload_speed_pid[3] = {RELOAD_ANGLE_PID_KP, RELOAD_ANGLE_PID_KI, RELOAD_ANGLE_PID_KD};
		static const fp32 missile_shoot_pid[3] = {MISSILE_SHOOT_MOTOR_SPEED_PID_KP, MISSILE_SHOOT_MOTOR_SPEED_PID_KI, MISSILE_SHOOT_MOTOR_SPEED_PID_KD};
		
    //ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();
    //���ָ��
    	shoot_control.shoot_motor_measure = get_shoot_measure_point();
		shoot_control.reload_motor_measure = get_reload_measure_point();
    //��ʼ��PID
		PID_init(&shoot_control.reload_pid, PID_POSITION, reload_speed_pid, RELOAD_READY_PID_MAX_OUT, RELOAD_READY_PID_MAX_IOUT);		
		PID_init(&shoot_control.missile_shoot_pid, PID_POSITION, missile_shoot_pid, MISSILE_SHOOT_MOTOR_SPEED_PID_MAX_OUT, MISSILE_SHOOT_MOTOR_SPEED_PID_MAX_IOUT);
    //��������
		shoot_control.shoot_flag = 0;
		shoot_control.shoot_continu_flag = 0;
		shoot_control.reverse_time = 0;
		shoot_control.shoot_time = 150;
		
		shoot_control.reload_given_current = 0;
    	shoot_control.reload_speed = 0.0f;
    	shoot_control.reload_speed_set = 0.0f;
		shoot_control.missile_shoot_speed = 0.0f;
    	shoot_control.missile_shoot_speed_set = 0.0f;
		
		shoot_control.reload_angle = 0;
		shoot_control.reload_angle_set = shoot_control.reload_angle;
}
///**
//  * @brief          ���״̬������
//  * @param[in]      void
//  * @retval         void
//  */
//int8_t R = 0;
//int s=2000,l;
//static void shoot_set_mode(void)
//{
////		static int8_t press_l_last_s = 0;
//		static uint16_t press_R_time = 0;
//		fp32 missile_speed;

//		//���̿��Ƴ���R������Ħ����
//		if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_R)
//		{
//				press_R_time ++;
//		}
//		if(press_R_time > 500) 
//		{
//				R=!R;
//				press_R_time = 0;
//		}

////		if ((switch_is_up(shoot_control.shoot_rc->rc.s[1]) || R) && robot_state.mains_power_shooter_output)
//			if (switch_is_up(shoot_control.shoot_rc->rc.s[1]))
//    {
//				laser_on();
//				reload_motor_turn_back();
//				//���ݲ���ϵͳ ���Ƶ���
//				switch (robot_state.shooter_id1_42mm_speed_limit)
//				{
//						case 10:
//						{
//								missile_speed = 4100;
//								break;
//						}					
//						case 16:
//						{
//								missile_speed = 5900;//14->4900  16->5900
//								break;
//						}
//						default:
//						{
//								missile_speed = 4900;
//								break;
//						}
//				}
//				shoot_missile(missile_speed);
////				
//				if(shoot_control.stuck_flag == 0)//�޿���
//				{
//						//����
//						if(!BUTTEN_TRIG_PIN)  	reload_motor(0.0f);
//						else if(BUTTEN_TRIG_PIN)		reload_motor(4.0f);
//					
//						if(shoot_control.shoot_rc->rc.ch[4] < 120)
//						{
//								shoot_control.bullet_flag = 1;
//						}
//						//����
//						if(shoot_control.bullet_flag == 1 && (shoot_control.shoot_rc->rc.ch[4] > 600) && !BUTTEN_TRIG_PIN)
////						if(shoot_control.bullet_flag == 1 && (shoot_control.shoot_rc->rc.ch[4] > 600 || (!press_l_last_s && shoot_control.press_l)) &&
////							!BUTTEN_TRIG_PIN && (robot_state.shooter_id1_42mm_cooling_limit - power_heat_data_t.shooter_id1_42mm_cooling_heat >= 100))  
//						{
//								shoot_control.shoot_flag = 1;
//								shoot_control.bullet_flag = 0;
//						}
//				}
//    }
//		else
//		{
//				laser_off();
//				shoot_missile(0);
//				third_missile(0);	
//				reload_motor(0);
//		}
//		
//		if(shoot_control.shoot_flag ==1)
//		{
//				shoot_control.shoot_time = 0;
//				shoot_control.shoot_flag = 0;
//		}
//		//����
//		if(shoot_control.shoot_time < 40)
//		{
//				reload_motor(6.0f);
//		}
//		
//	
//		
//		shoot_control.shoot_time++;
//		
//		if(shoot_control.shoot_time >= 150) shoot_control.shoot_time = 150;

//}

/**
  * @brief          ����ٶȼ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_control_loop(void)
{
			//����pid
			PID_calc(&shoot_control.reload_pid, shoot_control.reload_speed, shoot_control.reload_speed_set);
			PID_calc(&shoot_control.missile_shoot_pid, shoot_control.missile_shoot_speed, shoot_control.missile_shoot_speed_set);
			shoot_can_set_current = shoot_control.reload_pid.out;
			reload_can_set_current = shoot_control.missile_shoot_pid.out;
}

/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
	
    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.reload_speed = speed_fliter_3;
		
		//����ٶȸ���
		shoot_control.missile_shoot_speed = shoot_control.reload_motor_measure->speed_rpm;
		
    //��갴��
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
		
}

