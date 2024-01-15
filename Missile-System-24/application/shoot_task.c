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
/*----------------------------------宏定义---------------------------*/
#define shoot_laser_on() laser_on()                                              // 激光开启宏定义
#define shoot_laser_off() laser_off()                                            // 激光关闭宏定义
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin) // 微动开关IO
#define trigger_motor(speed) trigger_control.trigger_speed_set = speed           // 开启拨弹电机
/*----------------------------------内部函数---------------------------*/
/**
 * @brief   射击状态机
 */
static void Shoot_Set_Mode(void);
/**
 * @brief   射击数据更新
 */
static void Shoot_Feedback_Update(void);

/**
 * @brief 拨弹盘控制循环
 *
 */
static void trigger_control_loop(Shoot_Motor_t *trigger_move_control_loop);

static void trigger2_control_loop(Shoot_Motor_t *trigger_move_control_loop);

/**
 * @brief 设置发射控制模式
 *
 */
static void shoot_set_control_mode(missile_shoot_move_t *missile_shoot_set_control);

/**
 * @brief 射击模式切换数据过渡
 *
 */
static void shoot_mode_change_transit(void);


/**
 * @brief  射击初始化
 */
void shoot_init(void);
void SERIO_Control(void);
/*----------------------------------内部变量---------------------------*/
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
/*----------------------------------结构体------------------------------*/
Shoot_Motor_t trigger_motor; // 拨弹轮1数据
Shoot_Motor_t trigger_motor2; // 拨弹轮2数据
Shoot_Motor_t trigger_motor3; // 拨弹轮3数据
missile_shoot_move_t missile_shoot_move;       // 发射控制
/*----------------------------------外部变量---------------------------*/
extern ExtY_stm32 stm32_Y_shoot;
extern ext_power_heat_data_t power_heat_data_t; // 机器人当前的功率状态，主要判断枪口热量
/*---------------------------------------------------------------------*/
// 控制模式
shoot_mode_e shoot_mode = SHOOT_STOP;                             // 此次射击模式
shoot_mode_e last_shoot_mode = SHOOT_STOP;                        // 上次射击模式
shoot_control_mode_e shoot_control_mode = SHOOT_STOP_CONTROL;     // 射击控制模式
shoot_init_state_e shoot_init_state = SHOOT_INIT_UNFINISH;        // 射击初始化枚举体
shoot_motor_control_mode_e missile_shoot_motor_mode = SHOOT_MOTOR_STOP;    // 摩擦轮电机
shoot_motor_control_mode_e trigger_motor_mode = SHOOT_MOTOR_STOP; // 拨弹盘电机

/**
 * @brief          射击任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void shoot_task(void const *pvParameters)
{
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    // 射击初始化
    shoot_init();
    while (1)
    {
				SERIO_Control();
        // 设置发射模式
        Shoot_Set_Mode();
        // 模式切换数据过渡,主要PID清除，防止数据积累引发电机反转
        shoot_mode_change_transit();
        // 发射数据更新
        Shoot_Feedback_Update();
        // 射击控制循环
        shoot_control_loop();
        // 发送控制电流
        //        if (!(toe_is_error(TRIGGER_MOTOR_TOE) && !toe_is_error(missile_shoot_LEFT_MOTOR_TOE) && !toe_is_error(missile_shoot_RIGHT_MOTOR_TOE)))
        //        {
        // 发送控制指令
        CAN_cmd_shoot(-trigger_motor2.given_current, -trigger_motor3.given_current, -trigger_motor.given_current, 0);
			
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
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
void shoot_init(void)
{

    missile_shoot_move.laster_add = 0;
    trigger_motor.move_flag = 1;
    // 初始化PID
    stm32_shoot_pid_init();
    // 拨弹盘pid
    static const fp32 Trigger_speed_pid[3] = {900, 0, 100};
		static const fp32 Trigger2_speed_pid[3] = {900, 0, 100};
		static const fp32 Trigger3_speed_pid[3] = {900, 0, 100};
    PID_Init(&trigger_motor.motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		PID_Init(&trigger_motor2.motor_pid, PID_POSITION, Trigger2_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
		PID_Init(&trigger_motor3.motor_pid, PID_POSITION, Trigger3_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    // 数据指针获取
    missile_shoot_move.shoot_rc = get_remote_control_point();
   
    trigger_motor.shoot_motor_measure = get_trigger1_motor_measure_point();
    trigger_motor.blocking_angle_set = 0;
    trigger_motor2.shoot_motor_measure = get_shoot_motor_measure_point(0);
		trigger_motor2.blocking_angle_set = 0;
    trigger_motor3.shoot_motor_measure = get_shoot_motor_measure_point(1);
		trigger_motor3.blocking_angle_set = 0;
    // 滤波初始化
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
        // 模式发生切换,pid清除
        stm32_step_shoot_pid_clear();
    }
}


/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void Shoot_Feedback_Update(void)
{
    // 滤波
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    trigger_motor.speed = -speed_fliter_3;

    rc_speedset = missile_shoot_move.shoot_rc->rc.ch[1];

    motor_last_angle = trigger_motor.angle;

    // 电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd > Half_ecd_range)
    {
        trigger_motor.ecd_count--;
    }
    else if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd < -Half_ecd_range)
    {
        trigger_motor.ecd_count++;
    }
    if (trigger_motor.ecd_count == FULL_COUNT)
    {
        trigger_motor.ecd_count = -(FULL_COUNT - 1);
    }
    else if (trigger_motor.ecd_count == -FULL_COUNT)
    {
        trigger_motor.ecd_count = FULL_COUNT - 1;
    }
    // 计算输出轴角度
    trigger_motor.angle = (trigger_motor.ecd_count * ecd_range + trigger_motor.shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE;
//   
//		// 滤波
//    static fp32 speed_fliter_4 = 0.0f;
//    static fp32 speed_fliter_5 = 0.0f;
//    static fp32 speed_fliter_6 = 0.0f;
//    static const fp32 fliter_num2[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
//    speed_fliter_4 = speed_fliter_5;
//    speed_fliter_5 = speed_fliter_6;
//    speed_fliter_6 = speed_fliter_5 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor2.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
//    trigger_motor2.speed = -speed_fliter_6;

//    rc_speedset = missile_shoot_move.shoot_rc->rc.ch[1];

//    motor_last_angle = trigger_motor2.angle;

//    // 电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
//    if (trigger_motor2.shoot_motor_measure->ecd - trigger_motor2.shoot_motor_measure->last_ecd > Half_ecd_range)
//    {
//        trigger_motor2.ecd_count--;
//    }
//    else if (trigger_motor2.shoot_motor_measure->ecd - trigger_motor2.shoot_motor_measure->last_ecd < -Half_ecd_range)
//    {
//        trigger_motor2.ecd_count++;
//    }
//    if (trigger_motor2.ecd_count == FULL_COUNT)
//    {
//        trigger_motor2.ecd_count = -(FULL_COUNT - 1);
//    }
//    else if (trigger_motor2.ecd_count == -FULL_COUNT)
//    {
//        trigger_motor2.ecd_count = FULL_COUNT - 1;
//    }
//    // 计算输出轴角度
//    trigger_motor2.angle = (trigger_motor2.ecd_count * ecd_range + trigger_motor2.shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE;
//		
//		// 滤波
//    static fp32 speed_fliter_7 = 0.0f;
//    static fp32 speed_fliter_8 = 0.0f;
//    static fp32 speed_fliter_9 = 0.0f;
//    static const fp32 fliter_num3[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
//    speed_fliter_7 = speed_fliter_8;
//    speed_fliter_8 = speed_fliter_9;
//    speed_fliter_9 = speed_fliter_8 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor3.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
//    trigger_motor3.speed = -speed_fliter_9;

//    rc_speedset = missile_shoot_move.shoot_rc->rc.ch[1];

//    motor_last_angle = trigger_motor3.angle;

//    // 电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
//    if (trigger_motor3.shoot_motor_measure->ecd - trigger_motor3.shoot_motor_measure->last_ecd > Half_ecd_range)
//    {
//        trigger_motor3.ecd_count--;
//    }
//    else if (trigger_motor3.shoot_motor_measure->ecd - trigger_motor3.shoot_motor_measure->last_ecd < -Half_ecd_range)
//    {
//        trigger_motor3.ecd_count++;
//    }
//    if (trigger_motor3.ecd_count == FULL_COUNT)
//    {
//        trigger_motor3.ecd_count = -(FULL_COUNT - 1);
//    }
//    else if (trigger_motor3.ecd_count == -FULL_COUNT)
//    {
//        trigger_motor3.ecd_count = FULL_COUNT - 1;
//    }
//    // 计算输出轴角度
//    trigger_motor3.angle = (trigger_motor3.ecd_count * ecd_range + trigger_motor3.shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE;

}

static void shoot_set_control_mode(missile_shoot_move_t *missile_shoot_set_control)
{

    // 运行模式

    // 判断初始化是否完成
    if (shoot_control_mode == SHOOT_INIT_CONTROL)
    {
        static uint32_t init_time = 0;
        // 判断拨杆是否拨到下档
        if (switch_is_down(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
        {
            // 拨到下档停止初始化
            init_time = 0;
        }
        else
        {
            // 判断是否初始化完成
            if (shoot_init_state == SHOOT_INIT_UNFINISH)
            {
                // 初始化未完成

                // 判断初始化时间是否过长
                if (init_time >= SHOOT_TASK_S_TO_MS(SHOOT_TASK_MAX_INIT_TIME))
                {
                    // 初始化时间过长不进行初始化，进入其他模式
                    init_time = 0;
                }
                else
                {
                        // 初始化模式保持原状，初始化时间增加
                        init_time++;
                        return;
								}
							}
            else
            {
                // 进入其他模式
                init_time = 0;
            }
        }
    }
    // 根据遥控器开关设置发射控制模式
    if (switch_is_up(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // 击打前哨站模式
        shoot_control_mode = SHOOT_OUTPOST;
    }
    else if (switch_is_mid(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // 遥控器控制模式
        shoot_control_mode = SHOOT_RC_CONTROL;
    }
    else if (switch_is_down(missile_shoot_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // 击打基地模式
        shoot_control_mode = SHOOT_BASE;
    }
    else if (toe_is_error(DBUS_TOE))
    {
        // 遥控器报错处理
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }
    else
    {
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }

    // 云台在某些模式下发射停止
    //    if (gimbal_cmd_to_shoot_stop())
    //    {
    //        shoot_control_mode = SHOOT_STOP_CONTROL;
    //    }

    // 判断进入初始化模式
    static shoot_control_mode_e last_shoot_control_mode = SHOOT_STOP_CONTROL;
    if (shoot_control_mode != SHOOT_STOP_CONTROL && last_shoot_control_mode == SHOOT_STOP_CONTROL)
    {
        // 进入初始化模式
        //        shoot_control_mode = SHOOT_INIT_CONTROL;
    }
    last_shoot_control_mode = shoot_control_mode;
}

/**
 * @brief          射击模式设置
 * @param[in]      void
 * @retval         返回无
 */
static void Shoot_Set_Mode(void)
{

    // 设置发射控制模式
    shoot_set_control_mode(&missile_shoot_move);

    // 保留上次射击模式
    last_shoot_mode = shoot_mode;

    // 更新当前射击模式
	
	
	
}
/**
 * @brief          拨弹轮循环
 * @param[in]      void
 * @retval         返回无
 */
void shoot_control_loop(void)
{
    if (missile_shoot_move.shoot_rc->rc.s[0] == 3)
		{
        trigger_motor.speed_set = -TRIGGER_MOTOR_RUN_SPEED * (rc_speedset / 660);
				trigger_motor2.speed_set = -TRIGGER_MOTOR_RUN_SPEED * (rc_speedset / 660);
				trigger_motor3.speed_set = -TRIGGER_MOTOR_RUN_SPEED * (rc_speedset / 660);
		}
    else
		{
        trigger_motor.speed_set = 0; 
				trigger_motor.speed_set = 0; 
			  trigger_motor.speed_set = 0; 
		}

		
		    if (fabs(missile_shoot_move.shoot_rc->rc.ch[4]) >= 1000)
    {
					trigger_motor.speed_set = 10;				
    }
		else if (fabs(missile_shoot_move.shoot_rc->rc.ch[4]) == 660)
    {
					trigger_motor.speed_set = -15;				
    }

				if (missile_shoot_move.shoot_rc->rc.ch[2] > 400)
    {
					trigger_motor2.speed_set = -1;				
    }
		else if (missile_shoot_move.shoot_rc->rc.ch[2] < -400)
    {
					trigger_motor2.speed_set = 1;				
    }

				if (missile_shoot_move.shoot_rc->rc.ch[1] > 200)
    {
					trigger_motor3.speed_set = -5;				
    }
		else if (missile_shoot_move.shoot_rc->rc.ch[1] < -200)
    {
					trigger_motor3.speed_set = 5;				
    }

		if (missile_shoot_move.shoot_rc->rc.ch[4] == 0)
		{		
			trigger_motor.speed_set = 0;
		}

		if (missile_shoot_move.shoot_rc->rc.ch[2] == 0)
		{		
			trigger_motor2.speed_set = 0;
		}

		if (missile_shoot_move.shoot_rc->rc.ch[1] == 0)
		{		
			trigger_motor3.speed_set = 0;
		}

    // pid计算
    trigger_control_loop(&trigger_motor); // 发射控制
		trigger_control_loop(&trigger_motor2); // 拨弹盘控制
		trigger_control_loop(&trigger_motor3); // 弹簧控制

}

//速度环
static void trigger_control_loop(Shoot_Motor_t *trigger_move_control_loop)
{
    trigger_move_control_loop->motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
    trigger_move_control_loop->motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
    PID_Calc(&trigger_move_control_loop->motor_pid, trigger_move_control_loop->speed, trigger_move_control_loop->speed_set); // 拨弹盘
    trigger_move_control_loop->given_current = (int16_t)(trigger_move_control_loop->motor_pid.out);
}
	//角度环
static void trigger2_control_loop(Shoot_Motor_t *trigger_move_control_loop)
{
    trigger_move_control_loop->motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
    trigger_move_control_loop->motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;	
		trigger_move_control_loop->given_angle = (&trigger_move_control_loop->motor_pid, trigger_move_control_loop->angle, trigger_move_control_loop->set_angle);
		PID_Calc(&trigger_move_control_loop->motor_pid, trigger_move_control_loop->speed, trigger_move_control_loop->given_angle);
    trigger_move_control_loop->given_current = (int16_t)(trigger_move_control_loop->motor_pid.out);
}

