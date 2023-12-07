/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       launcher_task.c/h
  * @brief      launcher control task, because use the euler angle calculated by
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
#ifndef launcher_BEHAVIOUR_H
#define launcher_BEHAVIOUR_H
#include "struct_typedef.h"

#include "launcher_task.h"
typedef enum
{
  launcher_ZERO_FORCE = 0, 
  launcher_INIT,           
  launcher_CALI,           
  launcher_ABSOLUTE_ANGLE, 
  launcher_RELATIVE_ANGLE, 
  launcher_MOTIONLESS,     
	launcher_ABSOLUTE_SPIN,
} launcher_behaviour_e;

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

extern void launcher_behaviour_mode_set(launcher_control_t *launcher_mode_set);

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
extern void launcher_behaviour_control_set(fp32 *add_yaw, fp32 *add_spring, launcher_control_t *launcher_control_set);

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

extern bool_t launcher_cmd_to_chassis_stop(void);

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
extern bool_t launcher_cmd_to_shoot_stop(void);

#endif
