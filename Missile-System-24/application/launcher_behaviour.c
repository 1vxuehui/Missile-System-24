/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       launcher_task.c/h
  * @brief      launcher control task, because use the euler angle calculate by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.launcher has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成发射架控制任务，由于发射架使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。发射架主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
    add a launcher behaviour mode
    1. in launcher_behaviour.h , add a new behaviour name in launcher_behaviour_e
    erum
    {  
        ...
        ...
        launcher_XXX_XXX, // new add
    }launcher_behaviour_e,
    2. implement new function. launcher_xxx_xxx_control(fp32 *yaw, fp32 *spring, launcher_control_t *launcher_control_set);
        "yaw, spring" param is launcher movement contorl input. 
        first param: 'yaw' usually means  yaw axis move,usaully means increment angle.
            positive value means counterclockwise move, negative value means clockwise move.
        second param: 'spring' usually means spring axis move,usaully means increment angle.
            positive value means counterclockwise move, negative value means clockwise move.

        in this new function, you can assign set-point to "yaw" and "spring",as your wish
    3.  in "launcher_behavour_set" function, add new logical judgement to assign launcher_XXX_XXX to  "launcher_behaviour" variable,
        and in the last of the "launcher_behaviour_mode_set" function, add "else if(launcher_behaviour == launcher_XXX_XXX)" 
        choose a launcher control mode.
        four mode:
        launcher_MOTOR_RAW : will use 'yaw' and 'spring' as motor current set,  derectly sent to can bus.
        launcher_MOTOR_ENCONDE : 'yaw' and 'spring' are angle increment,  control enconde relative angle.
        launcher_MOTOR_GYRO : 'yaw' and 'spring' are angle increment,  control gyro absolute angle.
    4. in the last of "launcher_behaviour_control_set" function, add
        else if(launcher_behaviour == launcher_XXX_XXX)
        {
            launcher_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, launcher_control_set);
        }

        
    如果要添加一个新的行为模式
    1.首先，在launcher_behaviour.h文件中， 添加一个新行为名字在 launcher_behaviour_e
    erum
    {  
        ...
        ...
        launcher_XXX_XXX, // 新添加的
    }launcher_behaviour_e,

    2. 实现一个新的函数 launcher_xxx_xxx_control(fp32 *yaw, fp32 *spring, launcher_control_t *launcher_control_set);
        "yaw, spring" 参数是发射架运动控制输入量
        第一个参数: 'yaw' 通常控制yaw轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        第二个参数: 'spring' 通常控制spring轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        在这个新的函数, 你能给 "yaw"和"spring"赋值想要的参数
    3.  在"launcher_behavour_set"这个函数中，添加新的逻辑判断，给launcher_behaviour赋值成launcher_XXX_XXX
        在launcher_behaviour_mode_set函数最后，添加"else if(launcher_behaviour == launcher_XXX_XXX)" ,然后选择一种发射架控制模式
        3种:
        launcher_MOTOR_RAW : 使用'yaw' and 'spring' 作为电机电流设定值,直接发送到CAN总线上.
        launcher_MOTOR_ENCONDE : 'yaw' and 'spring' 是角度增量,  控制编码相对角度.
        launcher_MOTOR_GYRO : 'yaw' and 'spring' 是角度增量,  控制陀螺仪绝对角度.
    4.  在"launcher_behaviour_control_set" 函数的最后，添加
        else if(launcher_behaviour == launcher_XXX_XXX)
        {
            launcher_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, launcher_control_set);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "launcher_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "user_lib.h"
#include "referee.h"

//when launcher is in calibrating, set buzzer frequency and strenght
//当发射架在校准, 设置蜂鸣器频率和强度
#define launcher_warn_buzzer_on() buzzer_on(31, 20000)
#define launcher_warn_buzzer_off() buzzer_off()

extern ext_game_robot_state_t robot_state;

#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          remote control dealline solve,because the value of rocker is not zero in middle place,
  * @param          input:the raw channel value 
  * @param          output: the processed channel value
  * @param          deadline
  */
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


/**
  * @brief          judge if launcher reaches the limit by gyro
  * @param          gyro: rotation speed unit rad/s
  * @param          timing time, input "launcher_CALI_STEP_TIME"
  * @param          record angle, unit rad
  * @param          feedback angle, unit rad
  * @param          record ecd, unit raw
  * @param          feedback ecd, unit raw
  * @param          cali step, +1 by one step
  */
/**
  * @brief          通过判断角速度来判断发射架是否到达极限位置
  * @param          对应轴的角速度，单位rad/s
  * @param          计时时间，到达launcher_CALI_STEP_TIME的时间后归零
  * @param          记录的角度 rad
  * @param          反馈的角度 rad
  * @param          记录的编码值 raw
  * @param          反馈的编码值 raw
  * @param          校准的步骤 完成一次 加一
  */
#define launcher_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < launcher_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > launcher_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

/**
  * @brief          launcher behave mode set.
  * @param[in]      launcher_mode_set: launcher data
  * @retval         none
  */
/**
  * @brief          发射架行为状态机设置.
  * @param[in]      launcher_mode_set: 发射架数据指针
  * @retval         none
  */
static void launcher_behavour_set(launcher_control_t *launcher_mode_set);

/**
  * @brief          when launcher behaviour mode is launcher_ZERO_FORCE, the function is called
  *                 and launcher control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     spring: spring motor current set, it will be sent to CAN bus derectly.
  * @param[in]      launcher_control_set: launcher data
  * @retval         none
  */
/**
  * @brief          当发射架行为模式是launcher_ZERO_FORCE, 这个函数会被调用,发射架控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      spring:发送spring电机的原始值，会直接通过can 发送到电机
  * @param[in]      launcher_control_set: 发射架数据指针
  * @retval         none
  */
static void launcher_zero_force_control(fp32 *yaw, fp32 *spring, launcher_control_t *launcher_control_set);

/**
  * @brief          when launcher behaviour mode is launcher_INIT, the function is called
  *                 and launcher control mode is gyro mode. launcher will lift the spring axis
  *                 and rotate yaw axis.
  * @param[out]     yaw: yaw motor relative angle increment, unit rad.
  * @param[out]     spring: spring motor absolute angle increment, unit rad.
  * @param[in]      launcher_control_set: launcher data
  * @retval         none
  */
/**
  * @brief          发射架初始化控制，电机是陀螺仪角度控制，发射架先抬起spring轴，后旋转yaw轴
  * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     spring轴角度控制，为角度的增量 单位 rad
  * @param[in]      发射架数据指针
  * @retval         返回空
  */
static void launcher_init_control(fp32 *yaw, fp32 *spring, launcher_control_t *launcher_control_set);


/**
  * @brief          when launcher behaviour mode is launcher_RELATIVE_ANGLE, the function is called
  *                 and launcher control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment, unit rad
  * @param[out]     spring: spring axia relative angle increment,unit rad
  * @param[in]      launcher_control_set: launcher data
  * @retval         none
  */
/**
  * @brief          发射架编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      spring: spring轴角度控制，为角度的增量 单位 rad
  * @param[in]      launcher_control_set: 发射架数据指针
  * @retval         none
  */
static void launcher_relative_angle_control(fp32 *yaw, fp32 *spring, launcher_control_t *launcher_control_set);

//发射架行为状态机
launcher_behaviour_e launcher_behaviour = launcher_ZERO_FORCE;



/**
  * @brief          the function is called by launcher_set_mode function in launcher_task.c
  *                 the function set launcher_behaviour variable, and set motor mode.
  * @param[in]      launcher_mode_set: launcher data
  * @retval         none
  */
/**
  * @brief          被launcher_set_mode函数调用在launcher_task.c,发射架行为状态机以及电机状态机设置
  * @param[out]     launcher_mode_set: 发射架数据指针
  * @retval         none
  */

void launcher_behaviour_mode_set(launcher_control_t *launcher_mode_set)
{
    if (launcher_mode_set == NULL)
    {
        return;
    }
    //set launcher_behaviour variable
    //发射架行为状态机设置
    launcher_behavour_set(launcher_mode_set);

    //accoring to launcher_behaviour, set motor control mode
    //根据发射架行为状态机设置电机状态机
    if (launcher_behaviour == launcher_ZERO_FORCE)
    {
        launcher_mode_set->launcher_yaw_motor.launcher_motor_mode = launcher_MOTOR_RAW;
        launcher_mode_set->launcher_spring_motor.launcher_motor_mode = launcher_MOTOR_RAW;
    }
    else if (launcher_behaviour == launcher_INIT)
    {
        launcher_mode_set->launcher_yaw_motor.launcher_motor_mode = launcher_MOTOR_ENCONDE;
        launcher_mode_set->launcher_spring_motor.launcher_motor_mode = launcher_MOTOR_ENCONDE;
    }
		else if (launcher_behaviour == launcher_RELATIVE_ANGLE)
    {
        launcher_mode_set->launcher_yaw_motor.launcher_motor_mode = launcher_MOTOR_ENCONDE;
        launcher_mode_set->launcher_spring_motor.launcher_motor_mode = launcher_MOTOR_ENCONDE;
    }
    else if (launcher_behaviour == launcher_MOTIONLESS)
    {
        launcher_mode_set->launcher_yaw_motor.launcher_motor_mode = launcher_MOTOR_ENCONDE;
        launcher_mode_set->launcher_spring_motor.launcher_motor_mode = launcher_MOTOR_ENCONDE;
    }
}

/**
  * @brief          the function is called by launcher_set_contorl function in launcher_task.c
  *                 accoring to the launcher_behaviour variable, call the corresponding function
  * @param[out]     add_yaw:yaw axis increment angle, unit rad
  * @param[out]     add_spring:spring axis increment angle,unit rad
  * @param[in]      launcher_mode_set: launcher data
  * @retval         none
  */
/**
  * @brief          发射架行为控制，根据不同行为采用不同控制函数
  * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
  * @param[out]     add_spring:设置的spring角度增加值，单位 rad
  * @param[in]      launcher_mode_set:发射架数据指针
  * @retval         none
  */
void launcher_behaviour_control_set(fp32 *add_yaw, fp32 *add_spring, launcher_control_t *launcher_control_set)
{

    if (add_yaw == NULL || add_spring == NULL || launcher_control_set == NULL)
    {
        return;
    }

    if (launcher_behaviour == launcher_ZERO_FORCE)
    {
        launcher_zero_force_control(add_yaw, add_spring, launcher_control_set);
    }
    else if (launcher_behaviour == launcher_INIT)
    {
        launcher_init_control(add_yaw, add_spring, launcher_control_set);
    }
    else if (launcher_behaviour == launcher_RELATIVE_ANGLE)
    {
        launcher_relative_angle_control(add_yaw, add_spring, launcher_control_set);
    }
		
}

/**
  * @brief          in some launcher mode, need chassis keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          发射架在某些行为下，需要底盘不动
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t launcher_cmd_to_chassis_stop(void)
{
    if (launcher_behaviour == launcher_INIT || launcher_behaviour == launcher_MOTIONLESS || launcher_behaviour == launcher_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          in some launcher mode, need shoot keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          发射架在某些行为下，需要射击停止
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t launcher_cmd_to_shoot_stop(void)
{
    if (launcher_behaviour == launcher_INIT || launcher_behaviour == launcher_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief          launcher behave mode set.
  * @param[in]      launcher_mode_set: launcher data
  * @retval         none
  */
/**
  * @brief          发射架行为状态机设置.
  * @param[in]      launcher_mode_set: 发射架数据指针
  * @retval         none
  */
static void launcher_behavour_set(launcher_control_t *launcher_mode_set)
{
    if (launcher_mode_set == NULL)
    {
        return;
    }

		static int16_t last_key_G = 0;
		static int16_t move = 0;
		
		if(!last_key_G && launcher_mode_set->launcher_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G)
		{
				move=!move;
		}

		static int mode = 0;
		if(launcher_mode_set->launcher_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B )
		{
				mode = 1;
		}
		if(launcher_mode_set->launcher_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V )
		{
				mode = 2;
		}
		
    //开关控制 发射架状态
		if (!switch_is_down(launcher_mode_set->launcher_rc_ctrl->rc.s[1]))
		{
				// if (switch_is_down(launcher_mode_set->launcher_rc_ctrl->rc.s[0]))
				// {
				// 		launcher_behaviour = launcher_ABSOLUTE_ANGLE;
				// }
				// else if (switch_is_mid(launcher_mode_set->launcher_rc_ctrl->rc.s[0]))
				// {
				// 		launcher_behaviour = launcher_ABSOLUTE_ANGLE;
				// }
				// else if (switch_is_up(launcher_mode_set->launcher_rc_ctrl->rc.s[0]))
				// {
				// 		launcher_behaviour = launcher_ABSOLUTE_SPIN;
				// }
		}
		else if(switch_is_down(launcher_mode_set->launcher_rc_ctrl->rc.s[1]) && move)
		{
				// if (mode == 1)
				// {
				// 		launcher_behaviour = launcher_ABSOLUTE_ANGLE;
				// }
				// else if (mode == 2)
				// {
				// 		launcher_behaviour = launcher_ABSOLUTE_ANGLE;
										
				// 		if(launcher_mode_set->launcher_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT)
				// 		{
				// 				launcher_behaviour = launcher_ABSOLUTE_SPIN;
				// 		}
				// }
				// else
				// {
						launcher_behaviour = launcher_ZERO_FORCE;
				// }
		}
		else
		{
				robot_state.mains_power_chassis_output =0;
				launcher_behaviour = launcher_ZERO_FORCE;
		}

    if(toe_is_error(DBUS_TOE))
    {
        launcher_behaviour = launcher_ZERO_FORCE;
    }
		
		//继电器控制发射架电源
//    if(robot_state.mains_power_chassis_output == 0)
//		{
//        launcher_behaviour = launcher_ZERO_FORCE;
//		}

    //enter init mode
    //判断进入init状态机
    {
        static launcher_behaviour_e last_launcher_behaviour = launcher_ZERO_FORCE;
        if (last_launcher_behaviour == launcher_ZERO_FORCE && launcher_behaviour != launcher_ZERO_FORCE)
        {
            launcher_behaviour = launcher_INIT;
        }
        last_launcher_behaviour = launcher_behaviour;
    }
		
		last_key_G = launcher_mode_set->launcher_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G;
}

/**
  * @brief          when launcher behaviour mode is launcher_ZERO_FORCE, the function is called
  *                 and launcher control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     spring: spring motor current set, it will be sent to CAN bus derectly.
  * @param[in]      launcher_control_set: launcher data
  * @retval         none
  */
/**
  * @brief          当发射架行为模式是launcher_ZERO_FORCE, 这个函数会被调用,发射架控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      spring:发送spring电机的原始值，会直接通过can 发送到电机
  * @param[in]      launcher_control_set: 发射架数据指针
  * @retval         none
  */
static void launcher_zero_force_control(fp32 *yaw, fp32 *spring, launcher_control_t *launcher_control_set)
{
    if (yaw == NULL || spring == NULL || launcher_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *spring = 0.0f;
}
/**
  * @brief          when launcher behaviour mode is launcher_INIT, the function is called
  *                 and launcher control mode is gyro mode. launcher will lift the spring axis
  *                 and rotate yaw axis.
  * @param[out]     yaw: yaw motor relative angle increment, unit rad.
  * @param[out]     spring: spring motor absolute angle increment, unit rad.
  * @param[in]      launcher_control_set: launcher data
  * @retval         none
  */
/**
  * @brief          发射架初始化控制，电机是陀螺仪角度控制，发射架先抬起spring轴，后旋转yaw轴
  * @author         RM
  * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     spring轴角度控制，为角度的增量 单位 rad
  * @param[in]      发射架数据指针
  * @retval         返回空
  */
static void launcher_init_control(fp32 *yaw, fp32 *spring, launcher_control_t *launcher_control_set)
{
    if (yaw == NULL || spring == NULL || launcher_control_set == NULL)
    {
        return;
    }

    //初始化状态控制量计算
    if (fabs(INIT_spring_SET - launcher_control_set->launcher_spring_motor.relative_angle) > launcher_INIT_ANGLE_ERROR)
    {
        *spring = (INIT_spring_SET - launcher_control_set->launcher_spring_motor.relative_angle) * launcher_INIT_spring_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *spring = (INIT_spring_SET - launcher_control_set->launcher_spring_motor.relative_angle) * launcher_INIT_spring_SPEED;
        *yaw = (INIT_YAW_SET - launcher_control_set->launcher_yaw_motor.relative_angle) * launcher_INIT_YAW_SPEED;
    }
}

/**
  * @brief          when launcher behaviour mode is launcher_RELATIVE_ANGLE, the function is called
  *                 and launcher control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment, unit rad
  * @param[out]     spring: spring axia relative angle increment,unit rad
  * @param[in]      launcher_control_set: launcher data
  * @retval         none
  */
/**
  * @brief          发射架编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      spring: spring轴角度控制，为角度的增量 单位 rad
  * @param[in]      launcher_control_set: 发射架数据指针
  * @retval         none
  */
static void launcher_relative_angle_control(fp32 *yaw, fp32 *spring, launcher_control_t *launcher_control_set)
{
    if (yaw == NULL || spring == NULL || launcher_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, spring_channel = 0;

    rc_deadband_limit(launcher_control_set->launcher_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(launcher_control_set->launcher_rc_ctrl->rc.ch[spring_CHANNEL], spring_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN - launcher_control_set->launcher_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    *spring = spring_channel * spring_RC_SEN + launcher_control_set->launcher_rc_ctrl->mouse.y * spring_MOUSE_SEN;


}

/**
  * @brief          when launcher behaviour mode is launcher_MOTIONLESS, the function is called
  *                 and launcher control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment,  unit rad
  * @param[out]     spring: spring axia relative angle increment, unit rad
  * @param[in]      launcher_control_set: launcher data
  * @retval         none
  */
/**
  * @brief          发射架进入遥控器无输入控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      spring: spring轴角度控制，为角度的增量 单位 rad
  * @param[in]      launcher_control_set:发射架数据指针
  * @retval         none
  */
// static void launcher_motionless_control(fp32 *yaw, fp32 *spring, launcher_control_t *launcher_control_set)
// {
//     if (yaw == NULL || spring == NULL || launcher_control_set == NULL)
//     {
//         return;
//     }
//     *yaw = 0.0f;
//     *spring = 0.0f;
// }
