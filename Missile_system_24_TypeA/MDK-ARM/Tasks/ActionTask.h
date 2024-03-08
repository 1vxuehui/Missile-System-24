
#ifndef __ACTIONTASK_H
#define __ACTIONTASK_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "struct_typedef.h"
#include "remote_control.h"
#include "pid.h"
#include "Moto.h"
#include "user_lib.h"


typedef enum
{
	ACTION_EMPTY = 0,
	ACTION_NO_FORCE,
  ACTION_RC,
	ACTION_KEYBOARD,

	uplift_init,
} action_mode_e;

typedef enum
{
	ACTION_MOVE,//上层移动
	ACTION_CHASSIS,//下层移动
	ACTION_BUG,//出现bug
	ACTION_INIT,
	ACTION_GOLD,
	ACTION_SILVER,
	ACTION_CATCH,
} action_key_mode_e;

typedef struct
{
	//速度环pid
  pid_type_def forward_pid[2];
  pid_type_def uplift_pid_up[2];
  pid_type_def transverse_pid;
  pid_type_def sucker_pitch_pid[2];
  pid_type_def sucker_roll_pid;
  pid_type_def outer;

  //角度环pid
  pid_type_def sucker_pitch_angelpid[2];
  pid_type_def sucker_roll_angelpid;
  pid_type_def uplift_angelpid_up[2];
  pid_type_def forward_anglepid[2];
} pid_init_struct;

typedef struct
{
  fp32 forward_speed[2];
  fp32 forward_speedset[2];		    //前伸电机 3508 can1 id 5—6
  fp32 forward_give_current[2];
  fp32 forward_angle_set;
  fp32 forward_give_angle[2];
  fp32 forward_angle[2];
	fp32 forward_last_angle[2];
	fp32 forward_angle_sum[2];
	fp32 rc_forward;
	fp32 forward_angle_mileage[2];//角度里程
	fp32 forward_mileage[2];//距离里程
	int32_t forward_init_time;//前伸初始化时间
	fp32 forward_limit_max[2];//向前最大里程
	fp32 forward_limit_min[2];//向后最大里程
	fp32 forward_interval[2];//运动区间
	int32_t limit_flag[2]; //限位标志位
	int32_t limit_fla1g[2]; //限位标志位
	int32_t limit_init_ready_f[2];//初始化结束标志位
	
} forward_struct;

typedef struct
{
  fp32 uplift_speed[2];        //抬升电机 3508 can1 id 7—8
  fp32 uplift_speedset[2];
  fp32 uplift_give_angle[2];
  fp32 uplift_angle[2];
  fp32 uplift_angleset[2];
	fp32 uplift_last_angleset[2];
  fp32 uplift_angleset_sum[2];
  fp32 uplift_give_current[2];
	fp32 rc_uplift;
	
	fp32 uplift_angle_mileage[2];
	fp32 uplift_mileage[2];
	fp32 uplift_limit_max[2];
	//uplift
	fp32 uplift_interval[2];
	
	int32_t uplift_limit_flag[2];
	int32_t limit_init_ready_u[2];//初始化结束标志位
	
	
	int32_t init_u;//初始化结束标志位
	
	
} uplift_struct;

typedef struct
{	
  fp32 sucker_pitch_speed;    //吸盘yaw轴电机 3508 can2 id 2—3
  fp32 sucker_pitch_speedset;	
  fp32 sucker_pitch_give_angle;
  fp32 sucker_pitch_give_current;
  fp32 sucker_pitch_angleset;
  fp32 sucker_pitch_angle;
	fp32 rc_pitch;
	
	
	fp32 sucker_pitch_last_angle;
	fp32 sucker_pitch_last_angleset;
	fp32 sucker_pitch_angleset_sum;
	fp32 sucker_pitch_angle_add;
	fp32 pitch_angle_mileage;
	int32_t pitch_limit_flag;
	int32_t limit_init_ready_p;
	
	fp32 pitch_limit_max;
	fp32 pitch_limit_min;
	fp32 pitch_max_angleset;
	
	
	fp32 sucker_pitch_angle_sum;
	fp32 sucker_pitch_angle_add1;
	int32_t limit_init_p;
	

} pitch_struct;

typedef struct
{
  fp32 sucker_roll_speed;        //吸盘pitch轴电机 6020 can2 id 5
  fp32 sucker_roll_speedset;
  fp32 sucker_roll_give_current;
  fp32 sucker_roll_angleset;
  fp32 sucker_roll_angle;
  fp32 sucker_roll_give_angle;
	fp32 rc_roll;
	fp32 roll_angle;
	int32_t roll_init_flag;
	
} roll_struct;
typedef enum
{
	GIMBAL_EMPTY = 0,
	GIMBAL_NO_FORCE,
  GIMBAL_RC,
	GIMBAL_KEYBOARD,
} gimbal_mode_e;

typedef struct
{
	//速度环pid
  pid_type_def gimbal_roll_pid;
  //角度环pid
  pid_type_def gimbal_roll_angelpid;
	fp32 gimbal_roll_speed;
	fp32 gimbal_roll_speed_set;
	fp32 gimbal_roll_angle;
	fp32 gimbal_roll_angle_set;
	
	fp32 rc_gimbal_roll;
	fp32 gimbal_roll_current;
	fp32 rc_gimbal_pitch;
	
	
	fp32 gimbal_roll_angle_mileage;
	fp32 gimbal_roll_mileage;
	int32_t gimbal_roll_limit_flag;
	fp32 gimbal_roll_limit_max;
	fp32 gimbal_roll_interval;
//	fp32 gimbal_roll_
//	fp32 gimbal_roll_
//	fp32 
	
} gimbal_move_t;

typedef struct
{
		
  int32_t time_up;
	int32_t time_up_ready;
	int32_t speed_up_time;
	int32_t time_forward;
	int32_t time_forward_ready;
	int32_t time_pitch;
	int32_t time_pitch_ready;
	int32_t speed_pitch_time;
	int32_t time_roll;
	int32_t time_roll_ready;
	
	int32_t time_pump;
	int32_t time_pump_ready;
	
	fp32 q;
	fp32 pitch_angle_cail;
	
	int32_t fast_switch_flag;
	int32_t mode_switch_flag;
	int32_t control_Aa;
	int32_t control_Ac;
	int32_t qwqwr;
	int32_t qw2wr;
	
	
	int32_t init_switch_flag;
	int32_t init_A;
	int32_t init_ready_flag;
	
	int32_t control_gold;
	int32_t control_silver;
	int32_t control_catch;
	
	
	int32_t catch_time;
	int32_t catch_time_over;
	int32_t gold_time;
	int32_t silver_time;
	
	int32_t speed_pitch_catch_time;
	int32_t speed_pitch_silver_time;
	fp32 pitch_angle_catch;
	
	fp32 pitch_angle_catch_limit;
	
	fp32 pitch_angle_silver;
	
	fp32 pitch_angle_silver_limit;
	
	int32_t pitch_angle_silver_limit_ready;
	int32_t pitch_angle_catch_limit_ready;
	
	
	
	int32_t help_switch_flag_A;
	int32_t help_A;
	int32_t help_ready_flag_A;
	
	
} action_keyboard;




#define GIMBAL_2006_P				5000.0f	//底盘3508
#define GIMBAL_2006_I				0.0f
#define GIMBAL_2006_D				0.1f
//吸盘
#define GIMBAL_2006_ANGEL_PID_KP				22.0f	//吸盘pitch 3508 速度环
#define GIMBAL_2006_ANGEL_PID_KI				0.0f
#define GIMBAL_2006_ANGEL_PID_KD				0.3f
#define M2006_MAX_OUT		10000.0f
#define M2006_MAX_IOUT	5000.0f

#define M2006_MOTOR_RPM_TO_ACTION				0.000415809748903494517209f	//m3508转化成底盘速度(m/s)的比例
#define GIMBAL_MOTOR_RPM_TO_VECTOR_SEN M2006_MOTOR_RPM_TO_ACTION








//前伸
#define FORWARD_3508_P				15000.0f	//底盘3508
#define FORWARD_3508_I				10.0f
#define FORWARD_3508_D				0.0f
//吸盘
#define SUCKER_PITCH_3508_P				30.0f	//吸盘pitch 3508 速度环
#define SUCKER_PITCH_3508_I				0.0f
#define SUCKER_PITCH_3508_D				0.8f

#define SUCKER_PITCH_ANGEL_PID_KP 60.8f   //吸盘pitch 3508 角度环
#define SUCKER_PITCH_ANGEL_PID_KI 0.00f
#define SUCKER_PITCH_ANGEL_PID_KD 1.3f
#define SUCKER_PITCH_ANGEL_PID_MAX_OUT 3.0f
#define SUCKER_PITCH_ANGEL_PID_MAX_IOUT 0.2f

#define SUCKER_ROLL_P			2900.0f		//吸盘roll 3508 速度环	
#define SUCKER_ROLL_I			0.0f
#define SUCKER_ROLL_D			0.1f

#define SUCKER_ROLL_ANGEL_PID_KP 8.0f   //吸盘roll 3508 角度环	
#define SUCKER_ROLL_ANGEL_PID_KI 0.0F
#define SUCKER_ROLL_ANGEL_PID_KD 0.0F
#define SUCKER_ROLL_ANGEL_PID_MAX_OUT 10.0f
#define SUCKER_ROLL_ANGEL_PID_MAX_IOUT 0.2f
////抬升 0.2f
//#define UPLIFT_3508_P				3.0f	//抬升 3508 速度环  8.5
//#define UPLIFT_3508_I				0.0f
//#define UPLIFT_3508_D				0.1f

//#define UPLIFT_ANGEL_PID_KP 45.0f   //抬升 3508 角度环
//#define UPLIFT_ANGEL_PID_KI 0.00f
//#define UPLIFT_ANGEL_PID_KD 0.09f
//#define UPLIFT_ANGEL_PID_MAX_OUT 30000.0f
//#define UPLIFT_ANGEL_PID_MAX_IOUT 0.4f


////抬升
//#define UPLIFT_3508_P				7.0f	//抬升 3508 速度环  8.5
//#define UPLIFT_3508_I				0.0f
//#define UPLIFT_3508_D				0.1f

//#define UPLIFT_ANGEL_PID_KP 33.0f   //抬升 3508 角度环
//#define UPLIFT_ANGEL_PID_KI 0.00f
//#define UPLIFT_ANGEL_PID_KD 0.1f
//#define UPLIFT_ANGEL_PID_MAX_OUT 30000.0f
//#define UPLIFT_ANGEL_PID_MAX_IOUT 0.4f

//抬升
#define UPLIFT_3508_P				7.0f	//抬升 3508 速度环  8.5
#define UPLIFT_3508_I				0.0f
#define UPLIFT_3508_D				0.1f

#define UPLIFT_ANGEL_PID_KP 33.0f   //抬升 3508 角度环
#define UPLIFT_ANGEL_PID_KI 0.00f
#define UPLIFT_ANGEL_PID_KD 0.1f
#define UPLIFT_ANGEL_PID_MAX_OUT 30000.0f
#define UPLIFT_ANGEL_PID_MAX_IOUT 0.4f
//横移
#define TRANSVERSE_P				5000.0f			//横移2006 速度环
#define TRANSVERSE_I				0.0f
#define TRANSVERSE_D				0.0f
#define CHASSIS_OVERTURN_P		108.0f			//翻矿2006
#define CHASSIS_OVERTURN_I	  0.0f
#define CHASSIS_OVERTURN_D		0.1f

//速度解算
#define ACTION_VX_MAX									4.0f	//三向速度最大值
#define ACTION_VY_MAX									4.0f
#define ACTION_WZ_MAX									10.0f
#define ACTION_VUP_MAX					        4.0f	
#define ACTION_VTRAN_MAX               3.0f
#define ACTION_VSY_MAX                 3.0F
#define ACTION_WZ_SET_SCALE						0.1f	//比例
//#define MOTOR_DISTANCE_TO_CENTER				0.2f	//轮中心距
//#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX	0.25f	//轮速转车速
//#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY	0.25f
//#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ	0.25f
#define M3508_MOTOR_RPM_TO_ACTION				0.000415809748903494517209f	//m3508转化成底盘速度(m/s)的比例
#define ACTION_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_ACTION

//遥控器
#define ACTION_RC_DEADLINE			 25.0f

#define CHASSIS_RC_CLAW_CH			 1

#define CHASSIS_RC_LIFTUP_CH	   3
#define CHASSIS_RC_TRANSVERSE_CH 0 
#define CHASSIS_RC_SUCKERYAW_CH  2
#define ACTION_RC_MODE_S2			 0
#define ACTION_RC_MODE_S1			 1
//电机

#define MOTOR_CLAW_L			Moto_Data1[4]//爪子左
#define MOTOR_CLAW_R			Moto_Data1[5]
#define MOTOR_UPLIFT_L	  Moto_Data1[6]//翻转左
#define MOTOR_UPLIFT_R	  Moto_Data1[7]
#define MOTOR_TRANSVERSE  Moto_Data2[0]//存矿左
#define MOTOR_SUCKER_YAW	Moto_Data2[1]

#define PUMP_OPEN_IO	GPIO_PIN_SET		//云台推出/收回时引脚状态
#define PUMP_OFF_IO	GPIO_PIN_RESET

#define PUMP_OPEN							HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,PUMP_OPEN_IO)//大臂前伸
#define PUMP_OFF							HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,PUMP_OFF_IO)

#define BUTTEN_TRIG_PIN1 HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin1)

#define BUTTEN_TRIG_PIN2 HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin2)

#define BUTTON_TRIG_Pin1 GPIO_PIN_9
#define BUTTON_TRIG_GPIO_Port GPIOI

#define BUTTON_TRIG_Pin2 GPIO_PIN_0
#define BUTTON_TRIG_GPIO_Port GPIOI



extern void ActionControlTask_Setup(void);
extern void ActionControlTask_Loop(void);
#endif
