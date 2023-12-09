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

#ifndef launcher_TASK_H
#define launcher_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "stm32.h"
#include "remote_control.h"
//spring �ٶȻ� PID�����Լ� PID���������������
#define spring_SPEED_PID_KP        500.0f
#define spring_SPEED_PID_KI        0.0f
#define spring_SPEED_PID_KD        0.0f
#define spring_SPEED_PID_MAX_OUT   10000.0f
#define spring_SPEED_PID_MAX_IOUT  8000.0f

//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_PID_KP        2000.0f
#define YAW_SPEED_PID_KI        5.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   20000.0f
#define YAW_SPEED_PID_MAX_IOUT  10000.0f

//spring �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define spring_ENCODE_RELATIVE_PID_KP 		150.0f
#define spring_ENCODE_RELATIVE_PID_KI 		0.0f
#define spring_ENCODE_RELATIVE_PID_KD 		0.0f

#define spring_ENCODE_RELATIVE_PID_MAX_OUT  100.0f
#define spring_ENCODE_RELATIVE_PID_MAX_IOUT 50.0f

//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP        0.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f


//�����ʼ�� ����һ��ʱ��
#define launcher_TASK_INIT_TIME 201
//yaw,spring����ͨ���Լ�״̬����ͨ��
#define YAW_CHANNEL   2
#define spring_CHANNEL 3

//��ͷ180 ����
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//��ͷ������ٶ�
#define TURN_SPEED    0.04f
//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_DEADBAND   10


#define YAW_RC_SEN    -0.000005f
#define spring_RC_SEN  -0.000005f //0.005

#define YAW_MOUSE_SEN   0.00006f
#define spring_MOUSE_SEN 0.00007f

#define YAW_ENCODE_SEN    0.01f
#define spring_ENCODE_SEN  0.01f

#define launcher_CONTROL_TIME 1

//����ܲ���ģʽ �궨�� 0 Ϊ��ʹ�ò���ģʽ
#define launcher_TEST_MODE 0

#define spring_TURN  0
#define YAW_TURN    0

//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//����ܳ�ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
#define launcher_INIT_ANGLE_ERROR     0.1f
#define launcher_INIT_STOP_TIME       100
#define launcher_INIT_TIME            6000
#define launcher_CALI_REDUNDANT_ANGLE 0.1f
//����ܳ�ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define launcher_INIT_spring_SPEED     0.004f
#define launcher_INIT_YAW_SPEED       0.005f

#define INIT_YAW_SET    0.0f
#define INIT_spring_SET  0.0f

//�����У׼��ֵ��ʱ�򣬷���ԭʼ����ֵ���Լ���תʱ�䣬ͨ���������ж϶�ת
#define launcher_CALI_MOTOR_SET   8000
#define launcher_CALI_STEP_TIME   2000
#define launcher_CALI_GYRO_LIMIT  0.1f

#define launcher_CALI_spring_MAX_STEP  1
#define launcher_CALI_spring_MIN_STEP  2
#define launcher_CALI_YAW_MAX_STEP    3
#define launcher_CALI_YAW_MIN_STEP    4

#define launcher_CALI_START_STEP  launcher_CALI_spring_MAX_STEP
#define launcher_CALI_END_STEP    5

//�ж�ң�����������ʱ���Լ�ң�����������жϣ����÷����yaw����ֵ�Է�������Ư��
#define launcher_MOTIONLESS_RC_DEADLINE 10
#define launcher_MOTIONLESS_TIME_MAX    3000

//�������ֵת���ɽǶ�ֵ
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
    launcher_MOTOR_RAW = 0, //���ԭʼֵ����
    launcher_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
} launcher_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} launcher_PID_t;

typedef struct
{
    const motor_measure_t *launcher_motor_measure;
    launcher_PID_t launcher_motor_relative_angle_pid;
    launcher_motor_mode_e launcher_motor_mode;
    launcher_motor_mode_e last_launcher_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

		ExtU_stm32 stm32_U_YAW;
		ExtU_stm32 stm32_U_spring;
} launcher_motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_spring;
    fp32 min_spring;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_spring_ecd;
    uint16_t min_spring_ecd;
    uint8_t step;
} launcher_step_cali_t;

typedef struct
{
    const RC_ctrl_t *launcher_rc_ctrl;
    launcher_motor_t launcher_yaw_motor;
    launcher_motor_t launcher_spring_motor;
		
} launcher_control_t;

/**
  * @brief          ����yaw �������ָ��
  * @param[in]      none
  * @retval         yaw���ָ��
  */
extern const launcher_motor_t *get_yaw_motor_point(void);

/**
  * @brief          ���ص��� �������ָ��
  * @param[in]      none
  * @retval         spring
  */
extern const launcher_motor_t *get_spring_motor_point(void);

/**
  * @brief          ��������񣬼�� launcher_CONTROL_TIME 1ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */

extern void launcher_task(void const *pvParameters);

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
extern void set_cali_launcher_hook(const uint16_t yaw_offset, const uint16_t spring_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_spring, const fp32 min_spring);
#endif
