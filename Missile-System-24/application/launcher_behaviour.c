/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       launcher_task.c/h
  * @brief      launcher control task, because use the euler angle calculate by
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

#include "launcher_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "user_lib.h"
#include "referee.h"

//when launcher is in calibrating, set buzzer frequency and strenght
//���������У׼, ���÷�����Ƶ�ʺ�ǿ��
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
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ��Ϊ0��
  * @param          �����ң����ֵ
  * @param          ��������������ң����ֵ
  * @param          ����ֵ
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
  * @brief          ͨ���жϽ��ٶ����жϷ�����Ƿ񵽴Ｋ��λ��
  * @param          ��Ӧ��Ľ��ٶȣ���λrad/s
  * @param          ��ʱʱ�䣬����launcher_CALI_STEP_TIME��ʱ������
  * @param          ��¼�ĽǶ� rad
  * @param          �����ĽǶ� rad
  * @param          ��¼�ı���ֵ raw
  * @param          �����ı���ֵ raw
  * @param          У׼�Ĳ��� ���һ�� ��һ
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
  * @brief          �������Ϊ״̬������.
  * @param[in]      launcher_mode_set: ���������ָ��
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
  * @brief          ���������Ϊģʽ��launcher_ZERO_FORCE, ��������ᱻ����,����ܿ���ģʽ��rawģʽ.ԭʼģʽ��ζ��
  *                 �趨ֵ��ֱ�ӷ��͵�CAN������,�������������������Ϊ0.
  * @param[in]      yaw:����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      spring:����spring�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      launcher_control_set: ���������ָ��
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
  * @brief          ����ܳ�ʼ�����ƣ�����������ǽǶȿ��ƣ��������̧��spring�ᣬ����תyaw��
  * @param[out]     yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[out]     spring��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ���������ָ��
  * @retval         ���ؿ�
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
  * @brief          ����ܱ���ֵ���ƣ��������ԽǶȿ��ƣ�
  * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      spring: spring��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      launcher_control_set: ���������ָ��
  * @retval         none
  */
static void launcher_relative_angle_control(fp32 *yaw, fp32 *spring, launcher_control_t *launcher_control_set);

//�������Ϊ״̬��
launcher_behaviour_e launcher_behaviour = launcher_ZERO_FORCE;



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

void launcher_behaviour_mode_set(launcher_control_t *launcher_mode_set)
{
    if (launcher_mode_set == NULL)
    {
        return;
    }
    //set launcher_behaviour variable
    //�������Ϊ״̬������
    launcher_behavour_set(launcher_mode_set);

    //accoring to launcher_behaviour, set motor control mode
    //���ݷ������Ϊ״̬�����õ��״̬��
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
  * @brief          �������Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
  * @param[out]     add_yaw:���õ�yaw�Ƕ�����ֵ����λ rad
  * @param[out]     add_spring:���õ�spring�Ƕ�����ֵ����λ rad
  * @param[in]      launcher_mode_set:���������ָ��
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
  * @brief          �������ĳЩ��Ϊ�£���Ҫ���̲���
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
  * @brief          �������ĳЩ��Ϊ�£���Ҫ���ֹͣ
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
  * @brief          �������Ϊ״̬������.
  * @param[in]      launcher_mode_set: ���������ָ��
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
		
    //���ؿ��� �����״̬
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
		
		//�̵������Ʒ���ܵ�Դ
//    if(robot_state.mains_power_chassis_output == 0)
//		{
//        launcher_behaviour = launcher_ZERO_FORCE;
//		}

    //enter init mode
    //�жϽ���init״̬��
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
  * @brief          ���������Ϊģʽ��launcher_ZERO_FORCE, ��������ᱻ����,����ܿ���ģʽ��rawģʽ.ԭʼģʽ��ζ��
  *                 �趨ֵ��ֱ�ӷ��͵�CAN������,�������������������Ϊ0.
  * @param[in]      yaw:����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      spring:����spring�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      launcher_control_set: ���������ָ��
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
  * @brief          ����ܳ�ʼ�����ƣ�����������ǽǶȿ��ƣ��������̧��spring�ᣬ����תyaw��
  * @author         RM
  * @param[out]     yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[out]     spring��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ���������ָ��
  * @retval         ���ؿ�
  */
static void launcher_init_control(fp32 *yaw, fp32 *spring, launcher_control_t *launcher_control_set)
{
    if (yaw == NULL || spring == NULL || launcher_control_set == NULL)
    {
        return;
    }

    //��ʼ��״̬����������
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
  * @brief          ����ܱ���ֵ���ƣ��������ԽǶȿ��ƣ�
  * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      spring: spring��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      launcher_control_set: ���������ָ��
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
  * @brief          ����ܽ���ң������������ƣ��������ԽǶȿ��ƣ�
  * @author         RM
  * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      spring: spring��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      launcher_control_set:���������ָ��
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
