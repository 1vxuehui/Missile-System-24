
#include "MotionTask.h"
#include "ActionTask.h"
#include "Servo.h"
#include "tim.h"
#include "Gimbal_Task.h"

#include "Servo.h"


//first_order_filter_type_t remote_vx_first_order_filter;
//first_order_filter_type_t remote_vy_first_order_filter;
//first_order_filter_type_t remote_vz_first_order_filter;

//函数声明
static void chassis_can_tx(void);
static void chassis_vector_to_mecanum_wheel_speed(void);
static void chassis_update(void);
static void chassis_control_set(void);
static void chassis_mode_set(void);
static void chassis_rc_controlup(void);
static void chassis_keyboard_control(void);


average_type gim_z;//均值滤波句柄,z轴方向
int16_t gim_z_buf[200];

chassis_mode_e chassis_mode;
obstacle_mode_e obstacle_mode;
chassis_key_mode_e key_mode_C;

chassis_move_t chassis_move;
chassis_keyboard Key_C;
obstacle_steer obstacle;

Servo_HandlerTypedef
	servo_rescue_right,
	servo_rescue_left;
	
	
	void Ave_Init(average_type *avetype,int16_t *p,uint16_t psize)
{
	avetype->buffer = p;
	avetype->size = psize;
	avetype->poi = 0;
}
fp32 Ave_cacl(average_type *avetype,int16_t In)
{
	fp32 sum = 0;
	avetype->buffer[avetype->poi] = In;
	avetype->poi++;
	if(avetype->poi >= avetype->size) avetype->poi = 0;
	for(int i = 0;i<avetype->size;i++)
	{
		sum=(avetype->buffer[i])+sum;
	}
	sum = ((fp32)sum)/avetype->size;
	return sum;
}
//底盘轮询
void MotionControlTask_Loop(void)
{
	chassis_mode_set();		//遥控器选择模式
	chassis_update();			//数据更新
	chassis_control_set();//控制量解算8

	chassis_can_tx();			//can发送


		//舵机回中


	
	
	osDelay(1);
}
//底盘初始化
int afdafawf;
void MotionControlTask_Setup(void)
{
	//PID初始化
	//速度环pid参数
	PID_Value_Typedef chassis_pidval = {CHASSIS_3508_P,CHASSIS_3508_I,CHASSIS_3508_D};							//底盘PID参数
  chassis_mode = CHASSIS_EMPTY;

	//速度环初始化
	for(int i=0;i<4;i++)
			PID_init(&chassis_move.chassis_pid[i],PID_POSITION,&chassis_pidval,16000.0f,8000.0f);//3508
//		Servo_Set_Angle(&servo_rescue_left,	180);		
//	Servo_Set_Angle(&servo_rescue_right,	0);
	//舵机初始化
	
	
	Servo_Init(&servo_rescue_left,&htim8,TIM_CHANNEL_1);
	Servo_Init(&servo_rescue_right,&htim8,TIM_CHANNEL_2);
	
  OBSTACLE_ENABLE;
	OBSTACLE_DISABLE;
	
		//斜坡函数初始化
	ramp_init(&chassis_move.wz_set_ramp_source_type,0.001,5.5,-5.5);
		Ave_Init(&gim_z,gim_z_buf,200);
}

static void chassis_update(void)
{
	uint8_t i = 0;
	
	for (i = 0; i < 4; i++)
	{
		chassis_move.chassis_speed[i] = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data1[i].speed;
	}
	chassis_move.vx = (-chassis_move.chassis_speed[0] + chassis_move.chassis_speed[1] + chassis_move.chassis_speed[2] - chassis_move.chassis_speed[3]) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_move.vy = (-chassis_move.chassis_speed[0] - chassis_move.chassis_speed[1] + chassis_move.chassis_speed[2] + chassis_move.chassis_speed[3]) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	chassis_move.wz = (-chassis_move.chassis_speed[0] - chassis_move.chassis_speed[1] - chassis_move.chassis_speed[2] - chassis_move.chassis_speed[3]) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
}
static void chassis_mode_set(void)
{
	if(switch_is_up(rc_ctrl.rc.s[CHASSIS_RC_MODE_S2]))
		chassis_mode = CHASSIS_NO_FORCE;
	if(switch_is_mid(rc_ctrl.rc.s[CHASSIS_RC_MODE_S2]))
		chassis_mode = CHASSIS_RC;
	if(switch_is_down(rc_ctrl.rc.s[CHASSIS_RC_MODE_S2]))
		chassis_mode = CHASSIS_KEYBOARD;
}

static void chassis_control_set(void)
{
	uint8_t i = 0;
	
	if(chassis_mode == CHASSIS_RC)//遥控器控制
	{
		if(switch_is_up(rc_ctrl.rc.s[CHASSIS_RC_MODE_S1]))
		chassis_rc_controlup();

	}
		
	else if(chassis_mode == CHASSIS_KEYBOARD)//键盘控制
		chassis_keyboard_control();
	
	//麦轮解算
	chassis_vector_to_mecanum_wheel_speed();
	
	//PID计算
	//底盘3508PID
	for (i = 0; i <= 3; i++)
		chassis_move.chassis_give_current[i] = PID_calc(&chassis_move.chassis_pid[i],chassis_move.chassis_speed[i],chassis_move.chassis_speedset[i]);

	//无力电流为0
	if(chassis_mode == CHASSIS_NO_FORCE)
	{
		memset(chassis_move.chassis_give_current,0.0f,sizeof(chassis_move.chassis_give_current));

	}
	
		fp32 max_vector = 0.0f, vector_rate = 0.0f;
	
    fp32 temp = 0.0f;
	    //calculate the max speed in four wheels, limit the max speed
    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        temp = fabs(chassis_move.chassis_speedset[i]);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move.chassis_speedset[i] *= vector_rate;
        }
    }
	
  
}
int qwrqwrqwr;
		int control1;
		int control2;		
		int asfaf;
		int sfhsf;	

static void chassis_rc_controlup(void)
{
	fp32 rc_x,rc_y,rc_z;
	//遥控器死限
	rc_deadband_limit((fp32)rc_ctrl.rc.ch[CHASSIS_RC_X_CH],rc_x,CHASSIS_RC_DEADLINE);
	rc_deadband_limit((fp32)rc_ctrl.rc.ch[CHASSIS_RC_Y_CH],rc_y,CHASSIS_RC_DEADLINE);
	rc_deadband_limit((fp32)rc_ctrl.rc.ch[CHASSIS_RC_Z_CH],rc_z,CHASSIS_RC_DEADLINE);
	
//  first_order_filter_cali(&remote_vx_first_order_filter, rc_x);
//  first_order_filter_cali(&remote_vy_first_order_filter, rc_y);
//  first_order_filter_cali(&remote_vz_first_order_filter, rc_z);
	//车体速度设置
	chassis_move.vx_set = rc_x/660.0f*CHASSIS_VX_MAX;
	chassis_move.vy_set = -rc_y/660.0f*CHASSIS_VY_MAX;
	chassis_move.wz_set = -rc_z/660.0f*CHASSIS_WZ_MAX;
	

}
fp32 sjdnvisd,iwrgunrwign,aiss;

static void chassis_keyboard_control(void)
{
	

	
				rc_deadband_limit((fp32)rc_ctrl.rc.ch[2],sjdnvisd,ACTION_RC_DEADLINE);
			rc_deadband_limit((fp32)rc_ctrl.rc.ch[3],iwrgunrwign,ACTION_RC_DEADLINE);
			rc_deadband_limit((fp32)rc_ctrl.rc.ch[1],aiss,ACTION_RC_DEADLINE);

	//初始状态为底盘移动模式
	//通过shift键切换到上层移动模式
	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT)
	{
//	if(sjdnvisd>30&&Key_C.fast_switch_flag_C == 0)
//	{
		Key_C.control_Ca++;
			Key_C.mode_switch_flag_C = 1;
		if(Key_C.control_Ca > 300)
		{
//		  Key_C.mode_switch_flag_C = 1;
		  key_mode_C = CHASSIS_ACTION;
		}
	}
	//通过cyrl键切换回到底盘移动模式
//	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL&&Key_C.fast_switch_flag_C == 0)
//	{
////	else if(sjdnvisd<-30&&Key_C.fast_switch_flag_C == 0)
////	{
//		Key_C.control_C++;
//			Key_C.mode_switch_flag_C = 1;
//		if(Key_C.control_C > 300)
//		{
////		  Key_C.mode_switch_flag_C = 0;
//		  key_mode_C = CHASSIS_MOVE;
//		}
//	}
	else 
			Key_C.mode_switch_flag_C = 0;
	
	
	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL)
	{
//		if(sjdnvisd<-30)
//	{
//	Key_C.mode_switch_flag_C = 1;
		
			Key_C.control_Cc++;

		Key_C.mode_switch_flag_C = 1;
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)
		{
//		if(iwrgunrwign>30)
//		{
			
			Key_C.control_Cc = 0;
			Key_C.control_gold_C++;
			if(Key_C.control_gold_C > 300)
		  {
//				Key_C.mode_switch_flag_C = 1;
		    Key_C.fast_switch_flag_C = 1;
		    key_mode_C = CHASSIS_GOLD;
	  	}
		}
		else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)
		{
//				else if(iwrgunrwign<-30)
//		{
			Key_C.control_Cc = 0;
			Key_C.control_silver_C++;
			if(Key_C.control_silver_C > 300)
		  {
//			  Key_C.mode_switch_flag_C = 1;
		    Key_C.fast_switch_flag_C = 2;
		    key_mode_C = CHASSIS_SILVER;
	  	}
		}
	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)
	{
//				else if(aiss>30)
//		{
			Key_C.control_Cc = 0;
			Key_C.control_catch_C++;
			if(Key_C.control_catch_C > 300)
		  {
//			  Key_C.mode_switch_flag_C = 1;
		    Key_C.fast_switch_flag_C = 3;
		    key_mode_C = CHASSIS_CATCH;
	  	}
		}
		else
		{
			Key_C.fast_switch_flag_C = 0;
			Key_C.control_gold_C = 0;
			Key_C.control_silver_C = 0;
			Key_C.control_catch_C = 0;		
			if(Key_C.control_Cc > 300)
		{
//		  Key_C.mode_switch_flag_C = 0;
		  key_mode_C = CHASSIS_MOVE;
		}
			
		}
	}
	else
	{
		Key_C.mode_switch_flag_C = 0;
  	Key_C.control_Cc = 0;
	}
	
	
	
	
	
	//判断舵机是否无力
	//舵机标志位为0，舵机上电（初始模式舵机上电）
	if(obstacle.obstacle_steer_flag == 0)
	{
		obstacle_mode = OBSTACLE_MOVE;
		afdafawf = 1;
  }
	//舵机标志位为1，舵机无力模式
	else if(obstacle.obstacle_steer_flag == 1)
	{
	  obstacle_mode = OBSTACLE_DOWN;
		afdafawf = 2;
	}
	

	
 //------------------------救急模式-------------------------//
	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT&&rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)
	{
		//R键切换到初始化模式
		Key_C.help_C++;
		if(Key_C.help_C > 100)
		{
		  Key_C.help_switch_flag_C = 1;
		}
	}
	else Key_C.help_C = 0;
  if(Key_C.help_switch_flag_C == 1)
  {
    
//		action_forward.limit_flag[0] = 0;
//		action_forward.limit_flag[1] = 0;
//		action_uplift.uplift_limit_flag[0] = 0;
//		action_uplift.uplift_limit_flag[1] = 0;
//		action_pitch.limit_init_p = 0;
			Key_C.help_switch_flag_C = 0;//退出初始化模式标志位
		chassis_mode = CHASSIS_EMPTY;
			Key_C.help_ready_flag_C = 1;
		
	}
	
	
	

  //------------------------底盘移动模式-------------------------//
//if(key_mode_C == CHASSIS_MOVE&&Key_C.mode_switch_flag_C == 0)
//{
if(key_mode_C == CHASSIS_MOVE)
{
//  //------------------------障碍块舵机运动量设定-------------------------//
//  if(obstacle_mode == OBSTACLE_MOVE)
//	{
//	obstacle.servo_left_cail = 59 + control1/50;
//  obstacle.servo_right_cail = 0 + control2/50;
//	
//	Servo_Set_Angle(&servo_rescue_left,	obstacle.servo_left_cail);
//	Servo_Set_Angle(&servo_rescue_right,obstacle.servo_right_cail);
//	}
//	else if(obstacle_mode == OBSTACLE_DOWN)
//	{
//	obstacle.obstacle_down = 1;
//	}
//	
//  //障碍块舵机
//	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)
//	{
//		obstacle.obstacle_steer_flag = 0;
//		OBSTACLE_ENABLE;
//		control1++;
//		control2--;
//		if(obstacle.servo_left_cail>59)
//		{
//			obstacle.servo_left_cail = 59;
//		  if(control1>0){control1 = 0;}
//		}
//		if(obstacle.servo_right_cail<0)
//		{
//			obstacle.servo_right_cail = 0;
//		  if(control2<0){control2 = 0;}
//		}
//	}
//	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)
//	{
//		OBSTACLE_ENABLE;
//		obstacle.obstacle_steer_flag = 0;
//		control1--;
//		control2++;
//		if(obstacle.servo_left_cail<5)
//    {
//		  obstacle.servo_left_cail = 5;
//		  if(control1<-2950)control1 = -2950;
//		}
//		if(obstacle.servo_right_cail>54)
//    {
//			obstacle.servo_right_cail = 54;
//		  if(control2>2950)control2 = 2950;
//		}
//	}	
//	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)
//	{
//		OBSTACLE_DISABLE;
//		obstacle.obstacle_steer_flag = 1;
//  }
		
//chassis_move.direction = 1;
	
  //底盘3508电机速度设置
	//变化速度设置
		  Key_C.speed_rate = 4 + Key_C.speed_up;
	  if(Key_C.speed_rate<2)Key_C.speed_rate = 1;
	  if(Key_C.speed_rate>6)Key_C.speed_rate = 7;
	
	//基础全向移动，可通过模式改变改变底盘运动坐标（方向）
	  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)//前
	  	chassis_move.vx_set = chassis_move.direction*CHASSIS_VX_MAX/Key_C.speed_rate;
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)//后
	  	chassis_move.vx_set = -chassis_move.direction*CHASSIS_VX_MAX/Key_C.speed_rate;
  	else 
	  	chassis_move.vx_set = 0.0f;
	
  	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)//左
	  	chassis_move.vy_set = chassis_move.direction*CHASSIS_VY_MAX/Key_C.speed_rate;
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)//右
	  	chassis_move.vy_set = -chassis_move.direction*CHASSIS_VY_MAX/Key_C.speed_rate;
	  else 
	  	chassis_move.vy_set = 0.0f;
	
	

	  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)//加速键
	  {
	  	Key_C.time_run++;
		
	  	Key_C.speed_up = Key_C.time_run*0.001;
	  	if(Key_C.speed_up >2)
				{
					Key_C.speed_up = 2;
					
				}
			if(Key_C.time_run > 2000)Key_C.time_run = 2000;
	  }
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)//减速键
	  {
	  	Key_C.time_run--;
		
	  	Key_C.speed_up = Key_C.time_run*0.001;
	  	if(Key_C.speed_up <-2)
			{
				Key_C.speed_up = -2;
			}			
			if(Key_C.time_run < -2000)Key_C.time_run = -2000;

	  }
	
	
  	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)//缓慢左旋
  		chassis_move.wz_set = CHASSIS_WZ_MAX/3;
  	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)//缓慢右旋
  		chassis_move.wz_set = -CHASSIS_WZ_MAX/3;
  	else 
	    chassis_move.wz_set = Ave_cacl(&gim_z,-rc_ctrl.mouse.x)*0.1f;
	
		
	  //与小云台一键掉头模式搭配，f键为默认方向（工程头为前）（默认头为前）
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)
		{
			chassis_move.direction = 1;
			
		}
	  //与小云台一键掉头模式搭配，g键为反向方向（工程尾为前）
		else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)
		{
			chassis_move.direction = -1;
			
		}	
//		else chassis_move.direction = 1;
}
  //------------------------上层移动模式-------------------------//
	else if(key_mode_C == CHASSIS_ACTION)
  {
//	else if(key_mode_C == CHASSIS_ACTION&&Key_C.mode_switch_flag_C == 0)
//  {
		chassis_move.vx_set = 0;
		chassis_move.vy_set = 0;
		chassis_move.wz_set = 0;
	}
  //------------------------bug模式须重新调节-------------------------//
		
	else if(key_mode_C == CHASSIS_CATCH)
	{
			  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)//前
	  	chassis_move.vx_set = CHASSIS_VX_MAX/7;
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)//后
	  	chassis_move.vx_set = -CHASSIS_VX_MAX/7;
  	else 
	  	chassis_move.vx_set = 0.0f;
	
  	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)//左
	  	chassis_move.vy_set = CHASSIS_VY_MAX/7;
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)//右
	  	chassis_move.vy_set = -CHASSIS_VY_MAX/7;
	  else 
	  	chassis_move.vy_set = 0.0f;
		
		
		
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)//缓慢左旋
  		chassis_move.wz_set = CHASSIS_WZ_MAX/5;
  	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)//缓慢右旋
  		chassis_move.wz_set = -CHASSIS_WZ_MAX/5;
  	else 
	    chassis_move.wz_set = Ave_cacl(&gim_z,-rc_ctrl.mouse.x)*0.1f;

	}
	else if(key_mode_C == CHASSIS_SILVER)
	{
			  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)//前
	  	chassis_move.vx_set = CHASSIS_VX_MAX/7;
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)//后
	  	chassis_move.vx_set = -CHASSIS_VX_MAX/7;
  	else 
	  	chassis_move.vx_set = 0.0f;
	
  	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)//左
	  	chassis_move.vy_set = CHASSIS_VY_MAX/7;
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)//右
	  	chassis_move.vy_set = -CHASSIS_VY_MAX/7;
	  else 
	  	chassis_move.vy_set = 0.0f;
		
		
		
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)//缓慢左旋
  		chassis_move.wz_set = CHASSIS_WZ_MAX/5;
  	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)//缓慢右旋
  		chassis_move.wz_set = -CHASSIS_WZ_MAX/5;
  	else 
	    chassis_move.wz_set = Ave_cacl(&gim_z,-rc_ctrl.mouse.x)*0.1f;

	}
	else if(key_mode_C == CHASSIS_GOLD)
	{
			  
		
		
		
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)//缓慢左旋
  		chassis_move.wz_set = CHASSIS_WZ_MAX/5;
  	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)//缓慢右旋
  		chassis_move.wz_set = -CHASSIS_WZ_MAX/5;
  	else 
	    chassis_move.wz_set = Ave_cacl(&gim_z,-rc_ctrl.mouse.x)*0.1f;

	}
	else key_mode_C = CHASSIS_BUG;	
	
	
	
	
	if(obstacle.obstacle_steer_flag == 0)
	{
		obstacle_mode = OBSTACLE_MOVE;
  }
	//舵机标志位为1，舵机无力模式
	else if(obstacle.obstacle_steer_flag == 1)
	{
	  obstacle_mode = OBSTACLE_DOWN;
	}
	
	
	  if(obstacle_mode == OBSTACLE_MOVE)//140-100
	{
	obstacle.servo_left_cail = 120 + control1/5;
	
	Servo_Set_Angle(&servo_rescue_left,	obstacle.servo_left_cail);
	}
	else if(obstacle_mode == OBSTACLE_DOWN)
	{
	obstacle.obstacle_down = 1;
	}
	
	
   if(rc_ctrl.mouse.z>1)
	{
		obstacle.obstacle_steer_flag = 0;
		OBSTACLE_ENABLE;
		control1++;
		if(obstacle.servo_left_cail>140)
		{
			obstacle.servo_left_cail = 140;
		  if(control1>100){control1 = 100;}
		}
	}
	else if(rc_ctrl.mouse.z<-1)
	{
		OBSTACLE_ENABLE;
		obstacle.obstacle_steer_flag = 0;
		control1--;
		if(obstacle.servo_left_cail<100)
    {
		  obstacle.servo_left_cail = 100;
		  if(control1<-100)control1 = -100;
		}
	}	
	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)
	{
		OBSTACLE_ENABLE;
		obstacle.obstacle_steer_flag = 0;
		control1 = 0;
  }
	else 
	{
		obstacle.obstacle_steer_flag = 1;	
	}
}
static void chassis_vector_to_mecanum_wheel_speed(void)
{
	chassis_move.chassis_speedset[0] = -chassis_move.vx_set - chassis_move.vy_set + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * chassis_move.wz_set;
	chassis_move.chassis_speedset[1] =  chassis_move.vx_set - chassis_move.vy_set + ( CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * chassis_move.wz_set;
	chassis_move.chassis_speedset[2] =  chassis_move.vx_set + chassis_move.vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * chassis_move.wz_set;
	chassis_move.chassis_speedset[3] = -chassis_move.vx_set + chassis_move.vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * chassis_move.wz_set;
}
static void chassis_can_tx(void)
{    
	

	
	
	//底盘电机
	CAN_CMD_MOTO(&hcan1,CAN_MOTO_ALL_ID_LOW,

	             (int16_t)chassis_move.chassis_give_current[0],
							 (int16_t)chassis_move.chassis_give_current[1],
							 
							 (int16_t)chassis_move.chassis_give_current[2],
							 (int16_t)chassis_move.chassis_give_current[3]);



}

