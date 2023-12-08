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

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "launcher_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "launcher_behaviour.h"
#include "INS_task.h"
#include "shoot_task.h"
#include "pid.h"
#include "bsp_usart.h"
#include "referee.h"


//motor enconde value format, range[0-8191]
//�������ֵ���� 0��8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t launcher_high_water;
#endif

/**
  * @brief          ��ʼ��"launcher_control"����������pid��ʼ���� ң����ָ���ʼ��������ܵ��ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     init:"launcher_control"����ָ��.
  * @retval         none
  */
static void launcher_init(launcher_control_t *init);
/**
  * @brief          ���÷���ܿ���ģʽ����Ҫ��'launcher_behaviour_mode_set'�����иı�
  * @param[out]     launcher_set_mode:"launcher_control"����ָ��.
  * @retval         none
  */
static void launcher_set_mode(launcher_control_t *set_mode);
/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     launcher_feedback_update:"launcher_control"����ָ��.
  * @retval         none
  */
static void launcher_feedback_update(launcher_control_t *feedback_update);
/**
  * @brief          �����ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     mode_change:"launcher_control"����ָ��.
  * @retval         none
  */
static void launcher_mode_change_control_transit(launcher_control_t *mode_change);
/**
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
  * @brief          ���÷���ܿ����趨ֵ������ֵ��ͨ��launcher_behaviour_control_set�������õ�
  * @param[out]     launcher_set_control:"launcher_control"����ָ��.
  * @retval         none
  */
static void launcher_set_control(launcher_control_t *set_control);
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     launcher_control_loop:"launcher_control"����ָ��.
  * @retval         none
  */
static void launcher_control_loop(launcher_control_t *control_loop);

/**
  * @brief          ��launcher_MOTOR_ENCONDEģʽ�����ƽǶ��趨,��ֹ�������
  * @param[out]     launcher_motor:yaw�������spring���
  * @retval         none
  */
static void launcher_spring_relative_angle_limit(launcher_motor_t *launcher_motor, fp32 add);
static void launcher_yaw_relative_angle_limit(launcher_motor_t *launcher_motor, fp32 add);
/**
  * @brief          launcher calibration calculate
  * @param[in]      launcher_cali: cali data
  * @param[out]     yaw_offset:yaw motor middle place encode
  * @param[out]     spring_offset:spring motor middle place encode
  * @param[out]     max_yaw:yaw motor max machine angle
  * @param[out]     min_yaw: yaw motor min machine angle
  * @param[out]     max_spring: spring motor max machine angle
  * @param[out]     min_spring: spring motor min machine angle
  * @retval         none
  */
/**
  * @brief          �����У׼����
  * @param[in]      launcher_cali: У׼����
  * @param[out]     yaw_offset:yaw����������ֵ
  * @param[out]     spring_offset:spring ����������ֵ
  * @param[out]     max_yaw:yaw �������е�Ƕ�
  * @param[out]     min_yaw: yaw �����С��е�Ƕ�
  * @param[out]     max_spring: spring �������е�Ƕ�
  * @param[out]     min_spring: spring �����С��е�Ƕ�
  * @retval         none
  */
static void calc_launcher_cali(const launcher_step_cali_t *launcher_cali, uint16_t *yaw_offset, uint16_t *spring_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_spring, fp32 *min_spring);


//����ܿ��������������
launcher_control_t launcher_control;


extern ExtY_stm32 stm32_Y_yaw;
extern ExtY_stm32 stm32_Y_spring;
//�Ӿ�����
vision_rxfifo_t *vision_rx;
/**
  * @brief          launcher task, osDelay launcher_CONTROL_TIME (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          ��������񣬼�� launcher_CONTROL_TIME 1ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */

void launcher_task(void const *pvParameters)
{
    //�ȴ������������������������
    vTaskDelay(launcher_TASK_INIT_TIME);
    //����ܳ�ʼ��
    launcher_init(&launcher_control);
    //�жϵ���Ƿ�����
    while (toe_is_error(LAUNCHER_MOTOR1_TOE) || toe_is_error(LAUNCHER_MOTOR2_TOE))
    {
        vTaskDelay(launcher_CONTROL_TIME);
        launcher_feedback_update(&launcher_control);             //��������ݷ���
    }

    while (1)
    {
        launcher_set_mode(&launcher_control);                    //���÷���ܿ���ģʽ
        launcher_mode_change_control_transit(&launcher_control); //����ģʽ�л� �������ݹ���
        launcher_feedback_update(&launcher_control);             //��������ݷ���
        launcher_set_control(&launcher_control);                 //���÷���ܿ�����
        launcher_control_loop(&launcher_control);                //����ܿ���PID����
	

        if (!(toe_is_error(LAUNCHER_MOTOR1_TOE) && toe_is_error(LAUNCHER_MOTOR2_TOE)))
        {
            if (toe_is_error(DBUS_TOE))
            {
                CAN_cmd_launcher(0, 0);
            }
            else
            {
                CAN_cmd_launcher(launcher_control.launcher_yaw_motor.given_current, launcher_control.launcher_spring_motor.given_current);
            }
        }

        vTaskDelay(launcher_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        launcher_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          �����У׼���ã���У׼�ķ������ֵ�Լ���С����е��ԽǶ�
  * @param[in]      yaw_offse:yaw ��ֵ
  * @param[in]      spring_offset:spring ��ֵ
  * @param[in]      max_yaw:max_yaw:yaw �����ԽǶ�
  * @param[in]      min_yaw:yaw ��С��ԽǶ�
  * @param[in]      max_yaw:spring �����ԽǶ�
  * @param[in]      min_yaw:spring ��С��ԽǶ�
  * @retval         ���ؿ�
  * @waring         �������ʹ�õ�launcher_control ��̬�������º�������������ͨ��ָ�븴��
  */
void set_cali_launcher_hook(const uint16_t yaw_offset, const uint16_t spring_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_spring, const fp32 min_spring)
{
    launcher_control.launcher_yaw_motor.offset_ecd = yaw_offset;
    launcher_control.launcher_yaw_motor.max_relative_angle = max_yaw;
    launcher_control.launcher_yaw_motor.min_relative_angle = min_yaw;

    launcher_control.launcher_spring_motor.offset_ecd = spring_offset;
    launcher_control.launcher_spring_motor.max_relative_angle = max_spring;
    launcher_control.launcher_spring_motor.min_relative_angle = min_spring;
}

/**
  * @brief          �����У׼���㣬��У׼��¼����ֵ,��� ��Сֵ����
  * @param[out]     yaw ��ֵ ָ��
  * @param[out]     spring ��ֵ ָ��
  * @param[out]     yaw �����ԽǶ� ָ��
  * @param[out]     yaw ��С��ԽǶ� ָ��
  * @param[out]     spring �����ԽǶ� ָ��
  * @param[out]     spring ��С��ԽǶ� ָ��
  * @retval         ����1 ����ɹ�У׼��ϣ� ����0 ����δУ׼��
  * @waring         �������ʹ�õ�launcher_control ��̬�������º�������������ͨ��ָ�븴��
  */
bool_t cmd_cali_launcher_hook(uint16_t *yaw_offset, uint16_t *spring_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_spring, fp32 *min_spring)
{
    if (launcher_control.launcher_cali.step == 0)
    {
        launcher_control.launcher_cali.step             = launcher_CALI_START_STEP;
        //�������ʱ������ݣ���Ϊ��ʼ���ݣ����ж������Сֵ
        launcher_control.launcher_cali.max_spring        = launcher_control.launcher_spring_motor.absolute_angle;
        launcher_control.launcher_cali.max_spring_ecd    = launcher_control.launcher_spring_motor.launcher_motor_measure->ecd;
        launcher_control.launcher_cali.max_yaw          = launcher_control.launcher_yaw_motor.absolute_angle;
        launcher_control.launcher_cali.max_yaw_ecd      = launcher_control.launcher_yaw_motor.launcher_motor_measure->ecd;
        launcher_control.launcher_cali.min_spring        = launcher_control.launcher_spring_motor.absolute_angle;
        launcher_control.launcher_cali.min_spring_ecd    = launcher_control.launcher_spring_motor.launcher_motor_measure->ecd;
        launcher_control.launcher_cali.min_yaw          = launcher_control.launcher_yaw_motor.absolute_angle;
        launcher_control.launcher_cali.min_yaw_ecd      = launcher_control.launcher_yaw_motor.launcher_motor_measure->ecd;
        return 0;
    }
    else if (launcher_control.launcher_cali.step == launcher_CALI_END_STEP)
    {
        calc_launcher_cali(&launcher_control.launcher_cali, yaw_offset, spring_offset, max_yaw, min_yaw, max_spring, min_spring);
        (*max_yaw) -= launcher_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += launcher_CALI_REDUNDANT_ANGLE;
        (*max_spring) -= launcher_CALI_REDUNDANT_ANGLE;
        (*min_spring) += launcher_CALI_REDUNDANT_ANGLE;
        launcher_control.launcher_yaw_motor.offset_ecd              = *yaw_offset;
        launcher_control.launcher_yaw_motor.max_relative_angle      = *max_yaw;
        launcher_control.launcher_yaw_motor.min_relative_angle      = *min_yaw;
        launcher_control.launcher_spring_motor.offset_ecd            = *spring_offset;
        launcher_control.launcher_spring_motor.max_relative_angle    = *max_spring;
        launcher_control.launcher_spring_motor.min_relative_angle    = *min_spring;
        launcher_control.launcher_cali.step = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          calc motor offset encode, max and min relative angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     spring_offset:spring place encode
  * @param[out]     max_yaw:yaw max relative angle
  * @param[out]     min_yaw:yaw min relative angle
  * @param[out]     max_yaw:spring max relative angle
  * @param[out]     min_yaw:spring min relative angle
  * @retval         none
  */
/**
  * @brief          �����У׼���㣬��У׼��¼����ֵ,��� ��Сֵ
  * @param[out]     yaw ��ֵ ָ��
  * @param[out]     spring ��ֵ ָ��
  * @param[out]     yaw �����ԽǶ� ָ��
  * @param[out]     yaw ��С��ԽǶ� ָ��
  * @param[out]     spring �����ԽǶ� ָ��
  * @param[out]     spring ��С��ԽǶ� ָ��
  * @retval         none
  */
static void calc_launcher_cali(const launcher_step_cali_t *launcher_cali, uint16_t *yaw_offset, uint16_t *spring_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_spring, fp32 *min_spring)
{
}
/**
  * @brief          ����yaw �������ָ��
  * @param[in]      none
  * @retval         yaw���ָ��
  */
const launcher_motor_t *get_yaw_motor_point(void)
{
    return &launcher_control.launcher_yaw_motor;
}

/**
  * @brief          ����spring �������ָ��
  * @param[in]      none
  * @retval         spring
  */
const launcher_motor_t *get_spring_motor_point(void)
{
    return &launcher_control.launcher_spring_motor;
}

/**
  * @brief          ��ʼ��"launcher_control"����������pid��ʼ���� ң����ָ���ʼ��������ܵ��ָ���ʼ��
  * @param[out]     init:"launcher_control"����ָ��.
  * @retval         none
  */
static void launcher_init(launcher_control_t *init)
{
    //�������ָ���ȡ  
    init->launcher_yaw_motor.launcher_motor_measure = get_yaw_motor_measure_point();
    init->launcher_spring_motor.launcher_motor_measure = get_spring_motor_measure_point();
    //ң��������ָ���ȡ
    init->launcher_rc_ctrl = get_remote_control_point();
    //��ʼ�����ģʽ
    init->launcher_yaw_motor.launcher_motor_mode = init->launcher_yaw_motor.last_launcher_motor_mode = launcher_MOTOR_RAW;
    init->launcher_spring_motor.launcher_motor_mode = init->launcher_spring_motor.last_launcher_motor_mode = launcher_MOTOR_RAW;
		init->launcher_spring_motor.offset_ecd = 1600;//��ֵ
		init->launcher_yaw_motor.offset_ecd = 1350;//��ֵ
    //��ʼ�����pi
		stm32_pid_yaw_init();
		stm32_pid_spring_init();
    launcher_feedback_update(init);

    init->launcher_yaw_motor.absolute_angle_set = init->launcher_yaw_motor.absolute_angle;
    init->launcher_yaw_motor.relative_angle_set = init->launcher_yaw_motor.relative_angle;
    init->launcher_yaw_motor.motor_gyro_set = init->launcher_yaw_motor.motor_gyro;


    init->launcher_spring_motor.absolute_angle_set = init->launcher_spring_motor.absolute_angle;
    init->launcher_spring_motor.relative_angle_set = init->launcher_spring_motor.relative_angle;
    init->launcher_spring_motor.motor_gyro_set = init->launcher_spring_motor.motor_gyro;

		init->launcher_yaw_motor.max_relative_angle = 2.10f;
    init->launcher_yaw_motor.min_relative_angle = -2.60f;  
		
		init->launcher_spring_motor.max_relative_angle = 0.20f;
    init->launcher_spring_motor.min_relative_angle = -0.20f;   
		
}

/**
  * @brief          ���÷���ܿ���ģʽ����Ҫ��'launcher_behaviour_mode_set'�����иı�
  * @param[out]     launcher_set_mode:"launcher_control"����ָ��.
  * @retval         none
  */
static void launcher_set_mode(launcher_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    launcher_behaviour_mode_set(set_mode);
}

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     launcher_feedback_update:"launcher_control"����ָ��.
  * @retval         none
  */
static void launcher_feedback_update(launcher_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    //��������ݸ���
    feedback_update->launcher_spring_motor.absolute_angle = *(feedback_update->launcher_INT_angle_point + INS_spring_ADDRESS_OFFSET);

    feedback_update->launcher_spring_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->launcher_spring_motor.launcher_motor_measure->ecd,
                                                                                          feedback_update->launcher_spring_motor.offset_ecd);

    feedback_update->launcher_spring_motor.motor_gyro = *(feedback_update->launcher_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    feedback_update->launcher_yaw_motor.absolute_angle = *(feedback_update->launcher_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

    feedback_update->launcher_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->launcher_yaw_motor.launcher_motor_measure->ecd,
                                                                                        feedback_update->launcher_yaw_motor.offset_ecd);
    feedback_update->launcher_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->launcher_spring_motor.relative_angle) * (*(feedback_update->launcher_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - arm_sin_f32(feedback_update->launcher_spring_motor.relative_angle) * (*(feedback_update->launcher_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
}

/**
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
  * @brief          �����ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     launcher_mode_change:"launcher_control"����ָ��.
  * @retval         none
  */
static void launcher_mode_change_control_transit(launcher_control_t *launcher_mode_change)
{
    if (launcher_mode_change == NULL)
    {
        return;
    }
    //yaw���״̬���л���������
    if (launcher_mode_change->launcher_yaw_motor.last_launcher_motor_mode != launcher_MOTOR_RAW && launcher_mode_change->launcher_yaw_motor.launcher_motor_mode == launcher_MOTOR_RAW)
    {
        launcher_mode_change->launcher_yaw_motor.raw_cmd_current = launcher_mode_change->launcher_yaw_motor.current_set = launcher_mode_change->launcher_yaw_motor.given_current;
    }
    else if (launcher_mode_change->launcher_yaw_motor.last_launcher_motor_mode != launcher_MOTOR_GYRO && launcher_mode_change->launcher_yaw_motor.launcher_motor_mode == launcher_MOTOR_GYRO)
    {
        launcher_mode_change->launcher_yaw_motor.absolute_angle_set = launcher_mode_change->launcher_yaw_motor.absolute_angle;
    }
    else if (launcher_mode_change->launcher_yaw_motor.last_launcher_motor_mode != launcher_MOTOR_ENCONDE && launcher_mode_change->launcher_yaw_motor.launcher_motor_mode == launcher_MOTOR_ENCONDE)
    {
        launcher_mode_change->launcher_yaw_motor.relative_angle_set = launcher_mode_change->launcher_yaw_motor.relative_angle;
    }
    launcher_mode_change->launcher_yaw_motor.last_launcher_motor_mode = launcher_mode_change->launcher_yaw_motor.launcher_motor_mode;

    //spring���״̬���л���������
    if (launcher_mode_change->launcher_spring_motor.last_launcher_motor_mode != launcher_MOTOR_RAW && launcher_mode_change->launcher_spring_motor.launcher_motor_mode == launcher_MOTOR_RAW)
    {
        launcher_mode_change->launcher_spring_motor.raw_cmd_current = launcher_mode_change->launcher_spring_motor.current_set = launcher_mode_change->launcher_spring_motor.given_current;
    }
    else if (launcher_mode_change->launcher_spring_motor.last_launcher_motor_mode != launcher_MOTOR_GYRO && launcher_mode_change->launcher_spring_motor.launcher_motor_mode == launcher_MOTOR_GYRO)
    {
        launcher_mode_change->launcher_spring_motor.absolute_angle_set = launcher_mode_change->launcher_spring_motor.absolute_angle;
    }
    else if (launcher_mode_change->launcher_spring_motor.last_launcher_motor_mode != launcher_MOTOR_ENCONDE && launcher_mode_change->launcher_spring_motor.launcher_motor_mode == launcher_MOTOR_ENCONDE)
    {
        launcher_mode_change->launcher_spring_motor.relative_angle_set = launcher_mode_change->launcher_spring_motor.relative_angle;
    }

    launcher_mode_change->launcher_spring_motor.last_launcher_motor_mode = launcher_mode_change->launcher_spring_motor.launcher_motor_mode;
}

/**
  * @brief          ���÷���ܿ����趨ֵ������ֵ��ͨ��launcher_behaviour_control_set�������õ�
  * @param[out]     launcher_set_control:"launcher_control"����ָ��.
  * @retval         none
  */
static void launcher_set_control(launcher_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_spring_angle = 0.0f;

    launcher_behaviour_control_set(&add_yaw_angle, &add_spring_angle, set_control);
    //yaw���ģʽ����
    if (set_control->launcher_yaw_motor.launcher_motor_mode == launcher_MOTOR_RAW)
    {
        //rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        set_control->launcher_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->launcher_yaw_motor.launcher_motor_mode == launcher_MOTOR_ENCONDE)
    {
        //encondeģʽ�£��������Ƕȿ���
        launcher_yaw_relative_angle_limit(&set_control->launcher_yaw_motor, add_yaw_angle);
    }


    //spring���ģʽ����
    if (set_control->launcher_spring_motor.launcher_motor_mode == launcher_MOTOR_RAW)
    {
        //rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        set_control->launcher_spring_motor.raw_cmd_current = add_spring_angle;
    }
    else if (set_control->launcher_spring_motor.launcher_motor_mode == launcher_MOTOR_ENCONDE)
    {
        //encondeģʽ�£��������Ƕȿ���
        launcher_spring_relative_angle_limit(&set_control->launcher_spring_motor, add_spring_angle);
    }
}



/**
  * @brief          ����ܿ���ģʽ:launcher_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     launcher_motor:yaw�������spring���
  * @retval         none
  */
static void launcher_spring_relative_angle_limit(launcher_motor_t *launcher_motor, fp32 add)
{
    if (launcher_motor == NULL)
    {
        return;
    }
    launcher_motor->relative_angle_set += add;
    //�Ƿ񳬹���� ��Сֵ
    if (launcher_motor->relative_angle_set > 0.20f)
    {
        launcher_motor->relative_angle_set = 0.20f;
    }
    else if (launcher_motor->relative_angle_set < -0.20f)
    {
        launcher_motor->relative_angle_set = -0.20f;
    }
}
static void launcher_yaw_relative_angle_limit(launcher_motor_t *launcher_motor, fp32 add)
{
    if (launcher_motor == NULL)
    {
        return;
    }
    launcher_motor->relative_angle_set += add;
}


/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     launcher_control_loop:"launcher_control"����ָ��.
  * @retval         none
  */
static void launcher_control_loop(launcher_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    if (control_loop->launcher_yaw_motor.launcher_motor_mode == launcher_MOTOR_RAW)
    {
				if (&(control_loop->launcher_yaw_motor) == NULL)
				{
						return;
				}
				control_loop->launcher_yaw_motor.current_set = control_loop->launcher_yaw_motor.raw_cmd_current;
				control_loop->launcher_yaw_motor.given_current = (int16_t)(control_loop->launcher_yaw_motor.current_set);
    }
    else if (control_loop->launcher_yaw_motor.launcher_motor_mode == launcher_MOTOR_GYRO)
    {
				if (&(control_loop->launcher_yaw_motor) == NULL)
				{
						return;
				}
				stm32_step_yaw(control_loop->launcher_yaw_motor.absolute_angle_set,control_loop->launcher_yaw_motor.absolute_angle,0);
				control_loop->launcher_yaw_motor.given_current=stm32_Y_yaw.Out1;

    }
    else if (control_loop->launcher_yaw_motor.launcher_motor_mode == launcher_MOTOR_ENCONDE)
    {
				if (&(control_loop->launcher_yaw_motor) == NULL)
				{
						return;
				}
				stm32_step_yaw(control_loop->launcher_yaw_motor.relative_angle_set,control_loop->launcher_yaw_motor.relative_angle,0);
				control_loop->launcher_yaw_motor.given_current=stm32_Y_yaw.Out1;
    }

    if (control_loop->launcher_spring_motor.launcher_motor_mode == launcher_MOTOR_RAW)
    {
				if (&(control_loop->launcher_spring_motor) == NULL)
				{
						return;
				}
				control_loop->launcher_spring_motor.current_set = control_loop->launcher_spring_motor.raw_cmd_current;
				control_loop->launcher_spring_motor.given_current = (int16_t)(control_loop->launcher_spring_motor.current_set);
    }
    else if (control_loop->launcher_spring_motor.launcher_motor_mode == launcher_MOTOR_GYRO)
    {
				if (&(control_loop->launcher_spring_motor) == NULL)
				{
						return;
				}
				stm32_step_spring(control_loop->launcher_spring_motor.absolute_angle_set,control_loop->launcher_spring_motor.absolute_angle,0);
				control_loop->launcher_spring_motor.given_current=stm32_Y_spring.Out1;
    }
    else if (control_loop->launcher_spring_motor.launcher_motor_mode == launcher_MOTOR_ENCONDE)
    {
				if (&(control_loop->launcher_spring_motor) == NULL)
				{
						return;
				}
				stm32_step_spring(control_loop->launcher_spring_motor.relative_angle_set,control_loop->launcher_spring_motor.relative_angle,0);
				control_loop->launcher_spring_motor.given_current=stm32_Y_spring.Out1;

    }
}
