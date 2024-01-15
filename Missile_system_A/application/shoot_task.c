/**
 * @file shoot_task.c
 * @date 2023-05-11
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "shoot_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
#include "bsp_missile_shoot.h"
#include "arm_math.h"
#include "bsp_servo_pwm.h"
#include "user_lib.h"
#include "referee.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "pid.h"
#include "stm32.h"
#include "tim.h"
/*----------------------------------�궨��---------------------------*/
#define shoot_laser_on() laser_on()                                              // ���⿪���궨��
#define shoot_laser_off() laser_off()                                            // ����رպ궨��
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin) // ΢������IO
#define trigger_motor(speed) trigger_control.trigger_speed_set = speed           // �����������
/*----------------------------------�ڲ�����---------------------------*/
/**
 * @brief   ���״̬��
 */
static void Shoot_Set_Mode(void);
/**
 * @brief   ������ݸ���
 */
static void Shoot_Feedback_Update(void);

/**
 * @brief �����̿���ѭ��
 *
 */
static void trigger_control_loop(Shoot_Motor_t *trigger_move_control_loop);

static void trigger2_control_loop(Shoot_Motor_t *trigger_move_control_loop);

static void Motor_Angle_Cal();
/**
 * @brief ���÷������ģʽ
 *
 */
static void shoot_set_control_mode(missile_shoot_move_t *missile_shoot_set_control);

/**
 * @brief ���ģʽ�л����ݹ���
 *
 */
static void shoot_mode_change_transit(void);


/**
 * @brief  �����ʼ��
 */
void shoot_init(void);
void SERIO_Control(void);
/*----------------------------------�ڲ�����---------------------------*/
fp32 missile_shoot;
int a = 0;
fp32 motor_last_angle = 0;
fp32 sum = 0;
int b = 0;
int turnback_flag = 0;
fp32 rc_speedset;
#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2000
int PWM=1130;
int turn_flag = 0;
uint8_t cnt = 1;
/*----------------------------------�ṹ��------------------------------*/
Shoot_Motor_t trigger_motor; // ����������
Shoot_Motor_t trigger_motor2; 
Shoot_Motor_t trigger_motor3; 
missile_shoot_move_t missile_shoot_move;       // �������
/*----------------------------------�ⲿ����---------------------------*/
extern ExtY_stm32 stm32_Y_shoot;
extern ext_power_heat_data_t power_heat_data_t; // �����˵�ǰ�Ĺ���״̬����Ҫ�ж�ǹ������
/*---------------------------------------------------------------------*/
// ����ģʽ
shoot_mode_e shoot_mode = SHOOT_STOP;                             // �˴����ģʽ
shoot_mode_e last_shoot_mode = SHOOT_STOP;                        // �ϴ����ģʽ
shoot_control_mode_e shoot_control_mode = SHOOT_STOP_CONTROL;     // �������ģʽ
shoot_init_state_e shoot_init_state = SHOOT_INIT_UNFINISH;        // �����ʼ��ö����
shoot_motor_control_mode_e missile_shoot_motor_mode = SHOOT_MOTOR_STOP;    
shoot_motor_control_mode_e trigger_motor_mode = SHOOT_MOTOR_STOP; 

/**
 * @brief          ������񣬼�� GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: ��
 * @retval         none
 */
void shoot_task(void const *pvParameters)
{
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    // �����ʼ��
    shoot_init();
    while (1)
    {
				SERIO_Control();
        // ���÷���ģʽ
        Shoot_Set_Mode();
        // ģʽ�л����ݹ���,��ҪPID�������ֹ���ݻ������������ת
        shoot_mode_change_transit();
        // �������ݸ���
        Shoot_Feedback_Update();
        // �������ѭ��
        shoot_control_loop();
        // ���Ϳ��Ƶ���
        //        if (!(toe_is_error(TRIGGER_MOTOR_TOE) && !toe_is_error(missile_shoot_LEFT_MOTOR_TOE) && !toe_is_error(missile_shoot_RIGHT_MOTOR_TOE)))
        //        {
        // ���Ϳ���ָ��
        CAN_cmd_shoot(-trigger_motor2.given_current, -trigger_motor.given_current, -trigger_motor3.given_current, 0);
			
        //        }-trigger_motor.given_current
        vTaskDelay(SHOOT_TASK_DELAY_TIME);
    }
}

int B_flag=0;
void SERIO_Control(void)
{
	if(missile_shoot_move.shoot_rc->rc.ch[3] >= 531)
	{
		PWM = missile_shoot_move.shoot_rc->rc.ch[3]*2.13;
	}
	else
	{
		PWM = 1130;
	}
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,PWM);
if(PWM == 1130)
{
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
}
else
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
}

/**
 * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
 * @param[in]      void
 * @retval         ���ؿ�
 */
void shoot_init(void)
{
    missile_shoot_move.laster_add = 0;
    trigger_motor.move_flag = 1;
    // ��ʼ��PID
    stm32_shoot_pid_init();
    // ������pid
    static const fp32 Trigger_speed_pid[3] = {900, 0, 100};
		static const fp32 Trigger_angle_pid[3] = {80, 0, 10};
		static const fp32 Trigger2_speed_pid[3] = {900, 0, 100};
		static const fp32 Trigger3_speed_pid[3] = {900, 0, 100};
		PID_Init(&trigger_motor.motor_pid_angle, PID_POSITION, Trigger_angle_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_Init(&trigger_motor.motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		PID_Init(&trigger_motor2.motor_pid, PID_POSITION, Trigger2_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		PID_Init(&trigger_motor3.motor_pid, PID_POSITION, Trigger3_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    // ����ָ���ȡ
    missile_shoot_move.shoot_rc = get_remote_control_point();
    trigger_motor3.shoot_motor_measure = get_trigger1_motor_measure_point();
    trigger_motor3.blocking_angle_set = 0;
		trigger_motor3.set_angle = 0;
    trigger_motor2.shoot_motor_measure = get_shoot_motor_measure_point(0);
		trigger_motor2.blocking_angle_set = 0;
		trigger_motor.set_angle = 0;
    trigger_motor.shoot_motor_measure = get_shoot_motor_measure_point(1);
		trigger_motor.blocking_angle_set = 0;
		trigger_motor.set_angle = 0;
    // �˲���ʼ��
    const static fp32 missile_shoot_1_order_filter[1] = {0.1666666667f};
    const static fp32 missile_shoot_2_order_filter[1] = {0.1666666667f};
    first_order_filter_init(&missile_shoot_move.missile_shoot1_cmd_slow_set_speed, SHOOT_CONTROL_TIME, missile_shoot_1_order_filter);
    first_order_filter_init(&missile_shoot_move.missile_shoot2_cmd_slow_set_speed, SHOOT_CONTROL_TIME, missile_shoot_2_order_filter);


    Shoot_Feedback_Update();
}

static void shoot_mode_change_transit(void)
{
    if (last_shoot_mode != shoot_mode)
    {
        // ģʽ�����л�,pid���
        stm32_step_shoot_pid_clear();
    }
}


/**
 * @brief          ������ݸ���
 * @param[in]      void
 * @retval         void
 */
static void Shoot_Feedback_Update(void)
{    
    trigger_motor.speed = trigger_motor.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED;
		Motor_Angle_Cal();
//    motor_last_angle = trigger_motor.angle;
//    
//    if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd > Half_ecd_range)
//		{
//			trigger_motor.ecd_count--;
//		}
//		else if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd < -Half_ecd_range)
//		{
//			trigger_motor.ecd_count++;
//		}
//		if (trigger_motor.ecd_count == FULL_COUNT)
//		{
//			trigger_motor.ecd_count = -(FULL_COUNT - 1);
//		}
//		else if (trigger_motor.ecd_count == -FULL_COUNT)
//		{
//			trigger_motor.ecd_count = FULL_COUNT - 1;
//		}
//		//���������Ƕ�
//		trigger_motor.angle = (trigger_motor.ecd_count * ecd_range + trigger_motor.shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE *10;
}

static void Motor_Angle_Cal()
{
	float  res1, res2;
//	int  res3, res4;
	static float pos, pos_old;
	
	if(cnt)
	{
		pos_old=trigger_motor.shoot_motor_measure->ecd*360/8192;
		cnt=0;
	}	
	pos =trigger_motor.shoot_motor_measure->ecd*360/8192;
	trigger_motor.ANGLE_rev.eer=pos - pos_old;
	
	if(trigger_motor.ANGLE_rev.eer>0) 	
	{
		res1=trigger_motor.ANGLE_rev.eer-360;//��ת���Լ�
		res2=trigger_motor.ANGLE_rev.eer;
	}
	else
	{
		res1=trigger_motor.ANGLE_rev.eer+360;//��ת���Լ�һ�����ڵĽǶ�ֵ��360��
		res2=trigger_motor.ANGLE_rev.eer;
	}
	
	if(ABS(res1)<ABS(res2)) //��������ת���϶���ת�ĽǶ�С���Ǹ������
	{
		trigger_motor.ANGLE_rev.eer_eer = res1;
	}
	else
	{
		trigger_motor.ANGLE_rev.eer_eer = res2;
	}
	trigger_motor.ANGLE_rev.POS_ABS += trigger_motor.ANGLE_rev.eer_eer;
	pos_old = pos;
	trigger_motor.angle = pos   ;
}


static void shoot_set_control_mode(missile_shoot_move_t *missile_shoot_set_control)
{

    // ����ģʽ

    // �жϳ�ʼ���Ƿ����
    if (shoot_control_mode == SHOOT_INIT_CONTROL)
    {
        static uint32_t init_time = 0;
        // �жϲ����Ƿ񲦵��µ�
        if (switch_is_down(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
        {
            // �����µ�ֹͣ��ʼ��
            init_time = 0;
        }
        else
        {
            // �ж��Ƿ��ʼ�����
            if (shoot_init_state == SHOOT_INIT_UNFINISH)
            {
                // ��ʼ��δ���

                // �жϳ�ʼ��ʱ���Ƿ����
                if (init_time >= SHOOT_TASK_S_TO_MS(SHOOT_TASK_MAX_INIT_TIME))
                {
                    // ��ʼ��ʱ����������г�ʼ������������ģʽ
                    init_time = 0;
                }
                else
                {
                        // ��ʼ��ģʽ����ԭ״����ʼ��ʱ������
                        init_time++;
                        return;
								}
							}
            else
            {
                // ��������ģʽ
                init_time = 0;
            }
        }
    }
    // ����ң�����������÷������ģʽ
    if (switch_is_up(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // ����ǰ��վģʽ
        shoot_control_mode = SHOOT_OUTPOST;
    }
    else if (switch_is_mid(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // ң��������ģʽ
        shoot_control_mode = SHOOT_RC_CONTROL;
    }
    else if (switch_is_down(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // �������ģʽ
        shoot_control_mode = SHOOT_BASE;
    }
    else if (toe_is_error(DBUS_TOE))
    {
        // ң����������
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }
    else
    {
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }

    
    // �жϽ����ʼ��ģʽ
    static shoot_control_mode_e last_shoot_control_mode = SHOOT_STOP_CONTROL;
    if (shoot_control_mode != SHOOT_STOP_CONTROL && last_shoot_control_mode == SHOOT_STOP_CONTROL)
    {
        // �����ʼ��ģʽ
        //        shoot_control_mode = SHOOT_INIT_CONTROL;
    }
    last_shoot_control_mode = shoot_control_mode;
}

/**
 * @brief          ���ģʽ����
 * @param[in]      void
 * @retval         ������
 */
static void Shoot_Set_Mode(void)
{

    // ���÷������ģʽ
    shoot_set_control_mode(&missile_shoot_move);
		
    // �����ϴ����ģʽ
    last_shoot_mode = shoot_mode;

    // ���µ�ǰ���ģʽ
	
	
	
}
/**
 * @brief          ������ѭ��
 * @param[in]      void
 * @retval         ������
 */
void shoot_control_loop(void)
{
	if(shoot_control_mode == SHOOT_RC_CONTROL)
	{		
		    if (fabs(missile_shoot_move.shoot_rc->rc.ch[4]) >= 1000)
    {
					trigger_motor3.speed_set = 5;				
    }
		else if (fabs(missile_shoot_move.shoot_rc->rc.ch[4]) == 660)
    {
					trigger_motor3.speed_set = -5;			
    }

				if (missile_shoot_move.shoot_rc->rc.ch[2] > 400)
    {
					trigger_motor2.speed_set = -1;				
    }
		else if (missile_shoot_move.shoot_rc->rc.ch[2] < -400)
    {
					trigger_motor2.speed_set = 1;				
    }

				if (missile_shoot_move.shoot_rc->rc.ch[1] > 200 && turn_flag == 0)
    {
					trigger_motor.set_angle += 2;
					turn_flag = 1;
    }
		else if (missile_shoot_move.shoot_rc->rc.ch[1] < -200 && turn_flag == 0)
    {
					trigger_motor.set_angle -= 2;	
					turn_flag = 1;
    }

		if (missile_shoot_move.shoot_rc->rc.ch[1] == 0)
		{		
			trigger_motor.set_angle += 0;
			turn_flag = 0;
		}

		if (missile_shoot_move.shoot_rc->rc.ch[2] == 0)
		{		
			trigger_motor2.speed_set = 0;
		}

		if (missile_shoot_move.shoot_rc->rc.ch[4] == 0)
		{		
			trigger_motor3.speed_set = 0;
		}

    // pid����
    trigger2_control_loop(&trigger_motor); // �������
		trigger_control_loop(&trigger_motor2); // ���ɿ���
		trigger_control_loop(&trigger_motor3); // �����̿���
	}
}

//�ٶȻ�
static void trigger_control_loop(Shoot_Motor_t *trigger_move_control_loop)
{
    trigger_move_control_loop->motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
    trigger_move_control_loop->motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
    PID_Calc(&trigger_move_control_loop->motor_pid, trigger_move_control_loop->speed, trigger_move_control_loop->speed_set); // ������
    trigger_move_control_loop->given_current = (int16_t)(trigger_move_control_loop->motor_pid.out);
}
	//�ǶȻ�
static void trigger2_control_loop(Shoot_Motor_t *trigger_move_control_loop)
{
    trigger_move_control_loop->motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
    trigger_move_control_loop->motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;	
		trigger_move_control_loop->motor_pid_angle.max_out = TRIGGER_BULLET_PID_MAX_OUT;
    trigger_move_control_loop->motor_pid_angle.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;	
		trigger_move_control_loop->given_angle = PID_Calc(&trigger_move_control_loop->motor_pid_angle, trigger_move_control_loop->angle, trigger_move_control_loop->set_angle);

		PID_Calc(&trigger_move_control_loop->motor_pid, trigger_move_control_loop->speed, trigger_move_control_loop->given_angle);
    trigger_move_control_loop->given_current = (int16_t)(trigger_move_control_loop->motor_pid.out)*2;
}

