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
//电机编码值规整 0—8191
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
  * @brief          初始化"launcher_control"变量，包括pid初始化， 遥控器指针初始化，发射架电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"launcher_control"变量指针.
  * @retval         none
  */
static void launcher_init(launcher_control_t *init);
/**
  * @brief          设置发射架控制模式，主要在'launcher_behaviour_mode_set'函数中改变
  * @param[out]     launcher_set_mode:"launcher_control"变量指针.
  * @retval         none
  */
static void launcher_set_mode(launcher_control_t *set_mode);
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     launcher_feedback_update:"launcher_control"变量指针.
  * @retval         none
  */
static void launcher_feedback_update(launcher_control_t *feedback_update);
/**
  * @brief          发射架模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     mode_change:"launcher_control"变量指针.
  * @retval         none
  */
static void launcher_mode_change_control_transit(launcher_control_t *mode_change);
/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
  * @brief          设置发射架控制设定值，控制值是通过launcher_behaviour_control_set函数设置的
  * @param[out]     launcher_set_control:"launcher_control"变量指针.
  * @retval         none
  */
static void launcher_set_control(launcher_control_t *set_control);
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     launcher_control_loop:"launcher_control"变量指针.
  * @retval         none
  */
static void launcher_control_loop(launcher_control_t *control_loop);

/**
  * @brief          在launcher_MOTOR_ENCONDE模式，限制角度设定,防止超过最大
  * @param[out]     launcher_motor:yaw电机或者spring电机
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
  * @brief          发射架校准计算
  * @param[in]      launcher_cali: 校准数据
  * @param[out]     yaw_offset:yaw电机发射架中值
  * @param[out]     spring_offset:spring 电机发射架中值
  * @param[out]     max_yaw:yaw 电机最大机械角度
  * @param[out]     min_yaw: yaw 电机最小机械角度
  * @param[out]     max_spring: spring 电机最大机械角度
  * @param[out]     min_spring: spring 电机最小机械角度
  * @retval         none
  */
static void calc_launcher_cali(const launcher_step_cali_t *launcher_cali, uint16_t *yaw_offset, uint16_t *spring_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_spring, fp32 *min_spring);


//发射架控制所有相关数据
launcher_control_t launcher_control;


extern ExtY_stm32 stm32_Y_yaw;
extern ExtY_stm32 stm32_Y_spring;
//视觉数据
vision_rxfifo_t *vision_rx;
/**
  * @brief          launcher task, osDelay launcher_CONTROL_TIME (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          发射架任务，间隔 launcher_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */

void launcher_task(void const *pvParameters)
{
    //等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(launcher_TASK_INIT_TIME);
    //发射架初始化
    launcher_init(&launcher_control);
    //判断电机是否都上线
    while (toe_is_error(LAUNCHER_MOTOR1_TOE) || toe_is_error(LAUNCHER_MOTOR2_TOE))
    {
        vTaskDelay(launcher_CONTROL_TIME);
        launcher_feedback_update(&launcher_control);             //发射架数据反馈
    }

    while (1)
    {
        launcher_set_mode(&launcher_control);                    //设置发射架控制模式
        launcher_mode_change_control_transit(&launcher_control); //控制模式切换 控制数据过渡
        launcher_feedback_update(&launcher_control);             //发射架数据反馈
        launcher_set_control(&launcher_control);                 //设置发射架控制量
        launcher_control_loop(&launcher_control);                //发射架控制PID计算
	

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
  * @brief          发射架校准设置，将校准的发射架中值以及最小最大机械相对角度
  * @param[in]      yaw_offse:yaw 中值
  * @param[in]      spring_offset:spring 中值
  * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
  * @param[in]      min_yaw:yaw 最小相对角度
  * @param[in]      max_yaw:spring 最大相对角度
  * @param[in]      min_yaw:spring 最小相对角度
  * @retval         返回空
  * @waring         这个函数使用到launcher_control 静态变量导致函数不适用以上通用指针复用
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
  * @brief          发射架校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     yaw 中值 指针
  * @param[out]     spring 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     spring 最大相对角度 指针
  * @param[out]     spring 最小相对角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @waring         这个函数使用到launcher_control 静态变量导致函数不适用以上通用指针复用
  */
bool_t cmd_cali_launcher_hook(uint16_t *yaw_offset, uint16_t *spring_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_spring, fp32 *min_spring)
{
    if (launcher_control.launcher_cali.step == 0)
    {
        launcher_control.launcher_cali.step             = launcher_CALI_START_STEP;
        //保存进入时候的数据，作为起始数据，来判断最大，最小值
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
  * @brief          发射架校准计算，将校准记录的中值,最大 最小值
  * @param[out]     yaw 中值 指针
  * @param[out]     spring 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     spring 最大相对角度 指针
  * @param[out]     spring 最小相对角度 指针
  * @retval         none
  */
static void calc_launcher_cali(const launcher_step_cali_t *launcher_cali, uint16_t *yaw_offset, uint16_t *spring_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_spring, fp32 *min_spring)
{
}
/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
const launcher_motor_t *get_yaw_motor_point(void)
{
    return &launcher_control.launcher_yaw_motor;
}

/**
  * @brief          返回spring 电机数据指针
  * @param[in]      none
  * @retval         spring
  */
const launcher_motor_t *get_spring_motor_point(void)
{
    return &launcher_control.launcher_spring_motor;
}

/**
  * @brief          初始化"launcher_control"变量，包括pid初始化， 遥控器指针初始化，发射架电机指针初始化
  * @param[out]     init:"launcher_control"变量指针.
  * @retval         none
  */
static void launcher_init(launcher_control_t *init)
{
    //电机数据指针获取  
    init->launcher_yaw_motor.launcher_motor_measure = get_yaw_motor_measure_point();
    init->launcher_spring_motor.launcher_motor_measure = get_spring_motor_measure_point();
    //遥控器数据指针获取
    init->launcher_rc_ctrl = get_remote_control_point();
    //初始化电机模式
    init->launcher_yaw_motor.launcher_motor_mode = init->launcher_yaw_motor.last_launcher_motor_mode = launcher_MOTOR_RAW;
    init->launcher_spring_motor.launcher_motor_mode = init->launcher_spring_motor.last_launcher_motor_mode = launcher_MOTOR_RAW;
		init->launcher_spring_motor.offset_ecd = 1600;//中值
		init->launcher_yaw_motor.offset_ecd = 1350;//中值
    //初始化电机pi
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
  * @brief          设置发射架控制模式，主要在'launcher_behaviour_mode_set'函数中改变
  * @param[out]     launcher_set_mode:"launcher_control"变量指针.
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
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     launcher_feedback_update:"launcher_control"变量指针.
  * @retval         none
  */
static void launcher_feedback_update(launcher_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    //发射架数据更新
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
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
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
  * @brief          发射架模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     launcher_mode_change:"launcher_control"变量指针.
  * @retval         none
  */
static void launcher_mode_change_control_transit(launcher_control_t *launcher_mode_change)
{
    if (launcher_mode_change == NULL)
    {
        return;
    }
    //yaw电机状态机切换保存数据
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

    //spring电机状态机切换保存数据
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
  * @brief          设置发射架控制设定值，控制值是通过launcher_behaviour_control_set函数设置的
  * @param[out]     launcher_set_control:"launcher_control"变量指针.
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
    //yaw电机模式控制
    if (set_control->launcher_yaw_motor.launcher_motor_mode == launcher_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        set_control->launcher_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->launcher_yaw_motor.launcher_motor_mode == launcher_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        launcher_yaw_relative_angle_limit(&set_control->launcher_yaw_motor, add_yaw_angle);
    }


    //spring电机模式控制
    if (set_control->launcher_spring_motor.launcher_motor_mode == launcher_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        set_control->launcher_spring_motor.raw_cmd_current = add_spring_angle;
    }
    else if (set_control->launcher_spring_motor.launcher_motor_mode == launcher_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
        launcher_spring_relative_angle_limit(&set_control->launcher_spring_motor, add_spring_angle);
    }
}



/**
  * @brief          发射架控制模式:launcher_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     launcher_motor:yaw电机或者spring电机
  * @retval         none
  */
static void launcher_spring_relative_angle_limit(launcher_motor_t *launcher_motor, fp32 add)
{
    if (launcher_motor == NULL)
    {
        return;
    }
    launcher_motor->relative_angle_set += add;
    //是否超过最大 最小值
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
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     launcher_control_loop:"launcher_control"变量指针.
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
