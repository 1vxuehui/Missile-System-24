/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       launcher_task.c/h
  * @brief      launcher control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.launcher has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             ��ɷ���ܿ����������ڷ����ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ������������Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
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

        
    ���Ҫ���һ���µ���Ϊģʽ
    1.���ȣ���launcher_behaviour.h�ļ��У� ���һ������Ϊ������ launcher_behaviour_e
    erum
    {  
        ...
        ...
        launcher_XXX_XXX, // ����ӵ�
    }launcher_behaviour_e,

    2. ʵ��һ���µĺ��� launcher_xxx_xxx_control(fp32 *yaw, fp32 *spring, launcher_control_t *launcher_control_set);
        "yaw, spring" �����Ƿ�����˶�����������
        ��һ������: 'yaw' ͨ������yaw���ƶ�,ͨ���ǽǶ�����,��ֵ����ʱ���˶�,��ֵ��˳ʱ��
        �ڶ�������: 'spring' ͨ������spring���ƶ�,ͨ���ǽǶ�����,��ֵ����ʱ���˶�,��ֵ��˳ʱ��
        ������µĺ���, ���ܸ� "yaw"��"spring"��ֵ��Ҫ�Ĳ���
    3.  ��"launcher_behavour_set"��������У�����µ��߼��жϣ���launcher_behaviour��ֵ��launcher_XXX_XXX
        ��launcher_behaviour_mode_set����������"else if(launcher_behaviour == launcher_XXX_XXX)" ,Ȼ��ѡ��һ�ַ���ܿ���ģʽ
        3��:
        launcher_MOTOR_RAW : ʹ��'yaw' and 'spring' ��Ϊ��������趨ֵ,ֱ�ӷ��͵�CAN������.
        launcher_MOTOR_ENCONDE : 'yaw' and 'spring' �ǽǶ�����,  ���Ʊ�����ԽǶ�.
        launcher_MOTOR_GYRO : 'yaw' and 'spring' �ǽǶ�����,  ���������Ǿ��ԽǶ�.
    4.  ��"launcher_behaviour_control_set" ������������
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
  * @brief          ��launcher_set_mode����������launcher_task.c,�������Ϊ״̬���Լ����״̬������
  * @param[out]     launcher_mode_set: ���������ָ��
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
  * @brief          �������Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
  * @param[out]     add_yaw:���õ�yaw�Ƕ�����ֵ����λ rad
  * @param[out]     add_spring:���õ�spring�Ƕ�����ֵ����λ rad
  * @param[in]      launcher_mode_set:���������ָ��
  * @retval         none
  */
extern void launcher_behaviour_control_set(fp32 *add_yaw, fp32 *add_spring, launcher_control_t *launcher_control_set);

/**
  * @brief          in some launcher mode, need chassis keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          �������ĳЩ��Ϊ�£���Ҫ���̲���
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
  * @brief          �������ĳЩ��Ϊ�£���Ҫ���ֹͣ
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
extern bool_t launcher_cmd_to_shoot_stop(void);

#endif
