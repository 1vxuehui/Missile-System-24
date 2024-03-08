
#include "ActionTask.h"
#include "remote_control.h"
#include "Servo.h"
#include "tim.h"
#include "pid.h"
//#include "Gimbal_Task.h"


action_mode_e action_mode = ACTION_EMPTY;
action_key_mode_e key_mode_A = ACTION_CHASSIS;

pid_init_struct action_pid;
forward_struct action_forward;
uplift_struct action_uplift;
pitch_struct action_pitch;
roll_struct action_roll;

gimbal_move_t gimbal_move;


action_keyboard Key_A;
//gimbal_mode_e gimbal_mode = GIMBAL_EMPTY;

bool_t microswitch_uplift_bottoml;

bool_t microswitch_uplift_bottomr;
//函数声明
static void action_mode_set(void);//模式设置
static void action_update(void);//读取数据
static void action_control_set(void);//速度 角度设定
static void action_can_tx(void);//can通信发送
static void calibration(void);//校准函数
static void action_rc_controlup(void);//遥控器上拨杆控制
static void action_rc_controlmid(void);//遥控器中拨杆控制
static void action_rc_controldown(void);//遥控器下拨杆控制
static void action_keyboard_control(void);//键盘拨杆控制
static void forward_control(void);//前伸限位控制
static void uplift_control(void);//抬升限位控制
static void pitch_control(void);//pitch轴限位控制
static void roll_control(void);//roll轴限位控制
static void gimbal_roll_control(void);//云台roll轴限位控制

//底盘轮询
void ActionControlTask_Loop(void)
{
	action_mode_set();		//遥控器选择模式
	action_update();			//数据更新
	action_control_set();//控制量解算8

	action_can_tx();			//can发送

	osDelay(1);
	
	
}

//底盘初始化
void ActionControlTask_Setup(void)
{
	//PID初始化
	//速度环pid参数
	PID_Value_Typedef forward_pidval = {FORWARD_3508_P,FORWARD_3508_I,FORWARD_3508_D};									//夹爪PID参数
	PID_Value_Typedef uplift_pidval_up = {UPLIFT_3508_P,UPLIFT_3508_I,UPLIFT_3508_D};	//翻转机构PID参数
	PID_Value_Typedef sucker_pitch_pidval = {SUCKER_PITCH_3508_P,SUCKER_PITCH_3508_I,SUCKER_PITCH_3508_D};			//存矿机构PID参数
	PID_Value_Typedef sucker_roll_pidval = {SUCKER_ROLL_P,SUCKER_ROLL_I,SUCKER_ROLL_D};			//存矿机构PID参数
	PID_Value_Typedef gimbal_roll_pidval = {GIMBAL_2006_P,GIMBAL_2006_I,GIMBAL_2006_D};									//云台pitch轴PID参数
	//角度环pid参数
  PID_Value_Typedef sucker_roll_angelpidval = {SUCKER_ROLL_ANGEL_PID_KP, SUCKER_ROLL_ANGEL_PID_KI, SUCKER_ROLL_ANGEL_PID_KD};
	PID_Value_Typedef sucker_pitch_angelpidval = {SUCKER_PITCH_ANGEL_PID_KP, SUCKER_PITCH_ANGEL_PID_KI, SUCKER_PITCH_ANGEL_PID_KD};
	PID_Value_Typedef uplift_pidval_anglepidval_up = {UPLIFT_ANGEL_PID_KP,UPLIFT_ANGEL_PID_KI,UPLIFT_ANGEL_PID_KD};
	action_roll.sucker_roll_angleset = -6.85f;//初始化6020电机角度即让其处于特定编码值

	PUMP_OPEN;//开气泵
	PUMP_OFF;//关气泵
  BUTTEN_TRIG_PIN1;//微动开关初始化
	BUTTEN_TRIG_PIN2;//
	
	//速度环初始化
	PID_init(&gimbal_move.gimbal_roll_pid,PID_POSITION,&gimbal_roll_pidval,M2006_MAX_OUT,M2006_MAX_IOUT);//6020
	
	for(int i=0;i<2;i++)
		{
			PID_init(&action_pid.forward_pid[i],PID_POSITION,&forward_pidval,5000,2500);//3508
			PID_init(&action_pid.uplift_pid_up[i],PID_POSITION,&uplift_pidval_up,20000,2000);//3508		
	    PID_init(&action_pid.sucker_pitch_pid[i],PID_POSITION,&sucker_pitch_pidval,14000.0f,10000.0f);//3508	
		}
		  PID_init(&action_pid.sucker_roll_pid,PID_POSITION,&sucker_roll_pidval,20000.0f,5000.0f);//6020

	//角度环初始化
	for(int i=0;i<2;i++)
	{
  PID_init(&action_pid.sucker_pitch_angelpid[i],PID_POSITION,&sucker_pitch_angelpidval,2200,3000);
	PID_init(&action_pid.uplift_angelpid_up[i],PID_POSITION,&uplift_pidval_anglepidval_up,2200.0f,2000.0f);//3508			
  }
	PID_init(&action_pid.sucker_roll_angelpid,PID_POSITION,&sucker_roll_angelpidval,8000.0f,0.4f);//6020
  

}
fp32 asd,sdf,qwrw;
static void action_update(void)
{
	//电调读取数据
	action_roll.sucker_roll_speed = ACTION_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data2[4].speed;	//roll轴速度读取
	action_roll.sucker_roll_angle = Moto_Data2[4].angle;	//roll轴角度读取
	
	action_uplift.uplift_speed[0] = ACTION_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data2[1].speed;//左侧抬升速度读取
	action_uplift.uplift_speed[1] = ACTION_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data2[3].speed;//右侧抬升速度读取
	action_uplift.uplift_last_angleset[0] = action_uplift.uplift_angleset[0];//左侧抬升上一次角度读取
	action_uplift.uplift_last_angleset[1] = action_uplift.uplift_angleset[1];//左侧抬升上一次角度读取
	action_uplift.uplift_angle[0] = Moto_Data2[1].angle;//左侧抬升角度读取
	action_uplift.uplift_angle[1] = Moto_Data2[3].angle;//左侧抬升角度读取

	action_forward.forward_speed[0] = ACTION_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data1[4].speed;//左侧前伸速度读取
	action_forward.forward_speed[1] = ACTION_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data1[5].speed;//右侧前伸速度读取

	action_forward.forward_last_angle[0] = action_forward.forward_angle[0];//左侧前伸上一次角度读取
	action_forward.forward_last_angle[1] = action_forward.forward_angle[1];//右侧前伸上一次角度读取

	action_forward.forward_angle[0] = Moto_Data1[4].angle;//左侧前伸角度读取
	action_forward.forward_angle[1] = Moto_Data1[5].angle;//右侧前伸角度读取
	action_forward.forward_angle_sum[0] = (action_forward.forward_angle[0] - action_forward.forward_last_angle[0]);
	
	action_forward.forward_angle_sum[1] = (action_forward.forward_angle[1] - action_forward.forward_last_angle[1]);

	action_pitch.sucker_pitch_speed = ACTION_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data2[2].speed;	//pitch轴速度读取
	action_pitch.sucker_pitch_last_angle = Moto_Data2[2].angle_last;//pitch轴上一次角度读取
	action_pitch.sucker_pitch_angle = Moto_Data2[2].angle;//pitch轴上角度读取	
	action_pitch.sucker_pitch_angle_sum = (action_pitch.sucker_pitch_angle - action_pitch.sucker_pitch_last_angle);//计算角度变化量
	action_pitch.sucker_pitch_angle_add1+=action_pitch.sucker_pitch_angle_sum;//计算角度变化量
	action_pitch.sucker_pitch_last_angleset = action_pitch.sucker_pitch_angleset;

	sdf+=action_forward.forward_angle_sum[1];
	asd+=action_forward.forward_angle_sum[0];

	microswitch_uplift_bottoml = BUTTEN_TRIG_PIN1;
	microswitch_uplift_bottomr = BUTTEN_TRIG_PIN2;

	
	
	
	gimbal_move.gimbal_roll_speed = GIMBAL_MOTOR_RPM_TO_VECTOR_SEN * Moto_Data2[0].speed;
	gimbal_move.gimbal_roll_angle = Moto_Data2[0].angle_sum*360.0/8192.0/36.0;

}
static void action_mode_set(void)
{
	if(switch_is_up(rc_ctrl.rc.s[ACTION_RC_MODE_S2]))
		action_mode = ACTION_NO_FORCE;
	if(switch_is_mid(rc_ctrl.rc.s[ACTION_RC_MODE_S2]))
		action_mode = ACTION_RC;
	if(switch_is_down(rc_ctrl.rc.s[ACTION_RC_MODE_S2]))
		action_mode = ACTION_KEYBOARD;
}
fp32 highl,highr,highl_set,highr_set,b,c,d,error_l,error_r;
int miubl_cail,miubr_cail,miub_sum;
fp32 awdp,awd3,eifuweifu;
static void action_control_set(void)
{
	
	if(action_mode == ACTION_RC)//遥控器控制
	{
		if(switch_is_up(rc_ctrl.rc.s[ACTION_RC_MODE_S1]))
		action_rc_controlup();
	  if(switch_is_mid(rc_ctrl.rc.s[ACTION_RC_MODE_S1]))
		action_rc_controlmid();
		if(switch_is_down(rc_ctrl.rc.s[ACTION_RC_MODE_S1]))
		action_rc_controldown();
		
	}
	
	if(action_mode == ACTION_KEYBOARD)
	{
		action_keyboard_control();
	}
		
	

	//前伸PID
	action_forward.forward_give_current[0] = PID_calc(&action_pid.forward_pid[0],action_forward.forward_speed[0],action_forward.forward_speedset[0]);//
	action_forward.forward_give_current[1] = PID_calc(&action_pid.forward_pid[1],action_forward.forward_speed[1],action_forward.forward_speedset[1]);
	
	//抬升角度环PID
//	if((fp32)rc_ctrl.rc.ch[CHASSIS_RC_LIFTUP_CH]>0)
//	{
	action_uplift.uplift_give_angle[0] = PID_calc(&action_pid.uplift_angelpid_up[0],Moto_Data2[1].angle_sum*360.0/19.0/8192.0,action_uplift.uplift_angleset[0]);
  action_uplift.uplift_give_angle[1] = PID_calc(&action_pid.uplift_angelpid_up[1],Moto_Data2[3].angle_sum*360.0/19.0/8192.0,action_uplift.uplift_angleset[1]);
	action_uplift.uplift_give_current[0] = PID_calc(&action_pid.uplift_pid_up[0],action_uplift.uplift_speed[0],action_uplift.uplift_give_angle[0]);
	action_uplift.uplift_give_current[1] = PID_calc(&action_pid.uplift_pid_up[1],action_uplift.uplift_speed[1],action_uplift.uplift_give_angle[1]);
//	}
//	if((fp32)rc_ctrl.rc.ch[CHASSIS_RC_LIFTUP_CH]<0)
//	{	
//  uplift_give_angle[0] = PID_calc(&uplift_angelpid_up[0],Moto_Data2[1].angle_sum*360.0/19.0/8192.0,uplift_angleset[0]);
//  uplift_give_angle[1] = PID_calc(&uplift_angelpid_up[1],Moto_Data2[3].angle_sum*360.0/19.0/8192.0,uplift_angleset[1]);
//	uplift_give_current[0] = PID_calc(&uplift_pid_up[0],uplift_speed[0],uplift_give_angle[0]);
//	uplift_give_current[1] = PID_calc(&uplift_pid_up[1],uplift_speed[1],uplift_give_angle[1]);
//	}
  //pitch轴角度环PID
	action_pitch.sucker_pitch_give_angle = PID_calc(&action_pid.sucker_pitch_angelpid[1],Moto_Data2[2].angle_sum*360.0/19.0/8192.0,action_pitch.sucker_pitch_angleset);
	action_pitch.sucker_pitch_give_current = PID_calc(&action_pid.sucker_pitch_pid[1],action_pitch.sucker_pitch_speed,action_pitch.sucker_pitch_give_angle);
//first_order_filter_cali(&action_pitchone.chassis_cmd_slow_set_vx,action_pitchone.sucker_pitchone_give_current[1]);
//action_pitchone.sucker_pitchone_give_current[1] = action_pitchone.chassis_cmd_slow_set_vx.out;
  //roll轴角度环PID
  action_roll.sucker_roll_give_angle = PID_calc(&action_pid.sucker_roll_angelpid,Moto_Data2[4].angle_sum*360.0/19.0/8192.0,action_roll.sucker_roll_angleset);
	action_roll.sucker_roll_give_current = PID_calc(&action_pid.sucker_roll_pid,action_roll.sucker_roll_speed,action_roll.sucker_roll_give_angle);

	//无力电流为0
	
 gimbal_move.gimbal_roll_current = PID_calc(&gimbal_move.gimbal_roll_pid,gimbal_move.gimbal_roll_speed,gimbal_move.gimbal_roll_speed_set);

	if(action_mode == ACTION_NO_FORCE)
	{
		action_forward.forward_speedset[0] = 0.0f;
		action_forward.forward_speedset[1] = 0.0f;
		action_uplift.uplift_speed[0] = 0.0f;
		
		action_uplift.uplift_speed[1] = 0.0f;
	}

}
int uwygfuywf,whgbwber;
static void calibration(void)
{
		//前伸遥控器速度设置


	action_forward.forward_speedset[0] = -1.0;
	action_forward.forward_speedset[1] = 1.0;
//		whgbwber++;
//  if(Moto_Data1[4].current<-4800)
//	{
//		action_forward.limit_fla1g[0] = 2;
////		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
//		uwygfuywf++;
//	}
//	else 	action_forward.limit_fla1g[0] = 0;

//	//左侧电机最后堵转限位
//		if(Moto_Data1[5].current>4800)
//	{
//		action_forward.limit_fla1g[1] = 2;
////		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
//		whgbwber++;
//	}
//	else 	action_forward.limit_fla1g[1] = 0;


//	if(action_forward.limit_fla1g[0] == 2)
//	{
//		action_forward.forward_speedset[0] = 0;
////		whgbwber++;
//	}
//	if(action_forward.limit_fla1g[1] == 2)
//	{
//	  action_forward.forward_speedset[1] = 0;
//  }
	forward_control();
		
	
//  if(action_uplift.uplift_limit_flag[0] == 0||action_uplift.uplift_limit_max[1] == 0)
//	{
	  action_uplift.uplift_angleset[0] -= 0.2f;
	  action_uplift.uplift_angleset[1] += 0.2f;
//	}
	  uplift_control();
	  if(action_uplift.limit_init_ready_u[0] == 1&&action_uplift.limit_init_ready_u[1] == 1)
	 {
		  action_uplift.uplift_angleset[0] -= 0.0f;
	    action_uplift.uplift_angleset[1] += 0.0f;
	 }
	
	
	action_pitch.sucker_pitch_angleset += 0.1f;
	pitch_control();	
//  gimbal_move.gimbal_roll_speed_set = 
}
int wueinfiwef;
static void action_rc_controlup(void)
{
	//遥控器死限
	fp32 rc_r;
	//遥控器死限
	rc_deadband_limit((fp32)rc_ctrl.rc.ch[4],rc_r,20);
	
	//车体速度设置
  if(rc_r>40)
	{
		wueinfiwef = 1;
	}
	else if(rc_r<-40) wueinfiwef = 0;
	
	if(wueinfiwef == 1)
	{
		calibration();
	
  }
	if(action_forward.limit_init_ready_f[0]==1&&action_forward.limit_init_ready_f[1]==1&&action_uplift.limit_init_ready_u[0]==1&&action_uplift.limit_init_ready_u[1]==1&&action_pitch.pitch_limit_flag==1)
  {
		wueinfiwef = 0;//退出初始化模式标志位
	}
}

fp32 asd,sdf;
fp32 fjvghsif;
static void action_rc_controlmid(void)
{

	//遥控器死限

	rc_deadband_limit((fp32)rc_ctrl.rc.ch[CHASSIS_RC_CLAW_CH],action_forward.rc_forward,ACTION_RC_DEADLINE);
	rc_deadband_limit((fp32)rc_ctrl.rc.ch[CHASSIS_RC_LIFTUP_CH],action_uplift.rc_uplift,ACTION_RC_DEADLINE);
	rc_deadband_limit((fp32)rc_ctrl.rc.ch[CHASSIS_RC_SUCKERYAW_CH],action_pitch.rc_pitch,ACTION_RC_DEADLINE);
	//车体速度设置
	//前伸遥控器速度设置
	action_forward.forward_speedset[0] = action_forward.rc_forward/1320.0f*ACTION_VX_MAX;
	action_forward.forward_speedset[1] = -action_forward.rc_forward/1320.0f*ACTION_VX_MAX;
	//前伸限位控制
  forward_control();
	
	//吸盘pitch轴遥控器角度设置
	action_pitch.sucker_pitch_angleset += action_pitch.rc_pitch/5000.0f*ACTION_VSY_MAX;
	//吸盘pitch轴限位控制	
	pitch_control();
	
	if(action_uplift.init_u == 0)
	{
    action_uplift.uplift_angleset[0]+=0.2f;
  	if(action_uplift.uplift_angleset[0]>14.0f)
  	{
		  action_uplift.init_u = 1;
  	}
	}
	action_uplift.uplift_angleset[0] += action_uplift.rc_uplift/2000.0f*ACTION_VTRAN_MAX;//
	action_uplift.uplift_angleset[1] -= action_uplift.rc_uplift/2000.0f*ACTION_VTRAN_MAX;

	uplift_control();

}


fp32 ewgeqg;
int uyegfyuwe;
static void action_rc_controldown(void)
{
	fp32 rc_pump,rc_sucker_roll;
	//遥控器死限

	rc_deadband_limit((fp32)rc_ctrl.rc.ch[CHASSIS_RC_CLAW_CH],rc_pump,ACTION_RC_DEADLINE);
	rc_deadband_limit((fp32)rc_ctrl.rc.ch[0],rc_sucker_roll,ACTION_RC_DEADLINE);
	
	rc_deadband_limit((fp32)rc_ctrl.rc.ch[3],ewgeqg,ACTION_RC_DEADLINE);
	
	
			rc_deadband_limit((fp32)rc_ctrl.rc.ch[2],gimbal_move.rc_gimbal_roll,ACTION_RC_DEADLINE);
	//车体速度设置

	if(rc_sucker_roll>30)
		{action_roll.sucker_roll_angleset += 0.005f;}
	else if(rc_sucker_roll<-30)
		{action_roll.sucker_roll_angleset -= 0.005f;}
	else
		{action_roll.sucker_roll_angleset += 0.0f;}

		roll_control();

	
	if(rc_pump>400)
	{
		PUMP_OPEN;
		
	}
	if(rc_pump<-400)
	{
		PUMP_OFF;		
		
	}
	
	
	gimbal_move.gimbal_roll_speed_set = gimbal_move.rc_gimbal_roll/660*3.0f;
	gimbal_roll_control();
	if(gimbal_move.gimbal_roll_limit_flag == 1)
	{
		uyegfyuwe = 2;
		if(ewgeqg>30)
		{
			uyegfyuwe = 1;
//			if(gimbal_move.gimbal_roll_interval<176.0)
//			{
//			  gimbal_move.gimbal_roll_speed_set = 0.8;
//				if(gimbal_move.gimbal_roll_interval>176.0)
//			  {
//			  gimbal_move.gimbal_roll_speed_set = 0.0;
//				}
//			}
//			if(gimbal_move.gimbal_roll_interval>176.0)
//			{
//			  gimbal_move.gimbal_roll_speed_set = -0.8;
//				if(gimbal_move.gimbal_roll_interval<176.0)
//			  {
//			  gimbal_move.gimbal_roll_speed_set = 0.0;
//				}
//			}
			if(gimbal_move.gimbal_roll_interval<160.0f)
			{
			  gimbal_move.gimbal_roll_speed_set = 1.2f;
				if(gimbal_move.gimbal_roll_interval>160.0f)
			  {
			  gimbal_move.gimbal_roll_speed_set = 0.0f;
				}
			}
			if(gimbal_move.gimbal_roll_interval>192.0f)
			{
			  gimbal_move.gimbal_roll_speed_set = -1.2f;
				if(gimbal_move.gimbal_roll_interval<192.0f)
			  {
			  gimbal_move.gimbal_roll_speed_set = 0.0f;
				}
			}
		}
		else if(ewgeqg<-30)
		{
			gimbal_move.gimbal_roll_speed_set = 1.2f;
			if(gimbal_move.gimbal_roll_interval>458.0f)
			{
				gimbal_move.gimbal_roll_speed_set = 0;
			}
		}
	}

}

static void uplift_control(void)
{


	action_uplift.uplift_angleset_sum[0] = (action_uplift.uplift_angleset[0] - action_uplift.uplift_last_angleset[0]);
	awdp+=action_uplift.uplift_angleset_sum[0];
	action_uplift.uplift_angleset_sum[1] = (action_uplift.uplift_angleset[1] - action_uplift.uplift_last_angleset[1]);
	awd3+=action_uplift.uplift_angleset_sum[1];
	action_uplift.uplift_angle_mileage[0] = Moto_Data2[1].angle_sum*360.0f/19.0f/8192.0f; //角度里程 角度制
	action_uplift.uplift_angle_mileage[1] = Moto_Data2[3].angle_sum*360.0f/19.0f/8192.0f; //角度里程 角度制
	action_uplift.uplift_mileage[0] = Moto_Data2[1].angle_sum/19.0f/8192.0f*38.0f*3.14f; //距离里程 单位mm
	action_uplift.uplift_mileage[1] = Moto_Data2[3].angle_sum/19.0f/8192.0f*38.0f*3.14f; //距离里程 单位mm
	asd = Moto_Data2[3].angle_sum/19.0/8192.0*38*3.14/360.0;
	
	if(Moto_Data2[1].current<-3000)//通过电流值判断右侧是否堵转
	{
		action_uplift.uplift_limit_flag[0] = 1;//标志位为1
		action_uplift.uplift_limit_max[0] = action_uplift.uplift_mileage[0];//此时最大值为里程最大值 不为定值
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		action_uplift.uplift_limit_max[0] = action_uplift.uplift_limit_max[0];//确定最大值 为堵转位置
	}
	//左侧电机最前堵转限位
		if(Moto_Data2[3].current>3000)//通过电流值判断左侧是否堵转
	{
		action_uplift.uplift_limit_flag[1] = 1;
		action_uplift.uplift_limit_max[1] = action_uplift.uplift_mileage[1];//此时最大值为里程最大值 不为定值
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		action_uplift.uplift_limit_max[1] = action_uplift.uplift_limit_max[1];//确定最大值 为堵转位置
	}
	
	if(action_uplift.uplift_limit_flag[0] == 1)//通过标志为对运动区间限位
	{
		action_uplift.uplift_interval[0] = action_uplift.uplift_mileage[0] - action_uplift.uplift_limit_max[0];//计算运动区间
    if(action_uplift.uplift_interval[0]<3.0f)//右侧电机区间最小值
	  {
		  action_uplift.uplift_interval[0] = 3.0f;
		  if(action_uplift.uplift_angleset[0]<14.0f)//到达区间最小值时速度大于零 即不可向后加以速度
		  {
			  action_uplift.uplift_angleset[0] -= 0;
			  action_uplift.uplift_angleset[0] = 14.0f;
				action_uplift.limit_init_ready_u[0] = 1;
		  }
	  }
		else
				action_uplift.limit_init_ready_u[0] = 0;
		if(action_uplift.uplift_interval[0]>240.0f)
	  {
		  action_uplift.uplift_interval[0] = 240.0f;
		  if(action_uplift.uplift_angleset[0]>748.0f)
		  {
		  	action_uplift.uplift_angleset[0] = 748.0f;
		  }
	  }
		if(action_forward.limit_flag[0] == 2)
		{
			if(action_forward.forward_interval[0]<90.0f)
			{
				if(action_uplift.uplift_angleset[0]>404.0f)
				{
					action_uplift.uplift_angleset[0] = 404.0f;
					
				}
				
			}
			
		}
	}
	if(action_uplift.uplift_limit_flag[1] == 1)
	{
		action_uplift.uplift_interval[1] = action_uplift.uplift_mileage[1] - action_uplift.uplift_limit_max[1];
    if(action_uplift.uplift_interval[1]>-1.1f)//右侧电机区间最大值
	  {
	  	action_uplift.uplift_interval[1] = -1.1f;
	  	if(action_uplift.uplift_angleset[1]>0)//到达区间最大值时速度大于零 即不可向前加以速度
	  	{
			  action_uplift.uplift_angleset[1] += 0;
			  action_uplift.uplift_angleset[1] = 0;
				action_uplift.limit_init_ready_u[1] = 1;
	  	}
	  }	
			else
				action_uplift.limit_init_ready_u[1] = 0;
//		
		if(action_uplift.uplift_interval[1]<-240.0f)
	  {
		  action_uplift.uplift_interval[1] = -240.0f;
		  if(action_uplift.uplift_angleset[1]<-748.0f)
		  {
		  	action_uplift.uplift_angleset[1] = -748.0f;
		  }
	  }
//    if(action_uplift.uplift_interval[1]<0.5f)
//	  {
//	  	action_uplift.uplift_interval[1] = 0.5f;
//	  	if(action_uplift.uplift_angleset[1]<0)
//	  	{
//	  		action_uplift.uplift_angleset[1] = 0;
//	  	}
//	  }	
		if(action_forward.limit_flag[1] == 2)
		{
			if(action_forward.forward_interval[1]>-90.0f)
			{
				if(action_uplift.uplift_angleset[1]<-390.0f)
				{
					action_uplift.uplift_interval[1] = -390.0f;
					
				}
				
			}
			
		}		
		
  }
}
static void forward_control(void)
	


{

	action_forward.forward_angle_mileage[0] = Moto_Data1[4].angle_sum*360.0f/19.0f/8192.0f; //角度里程 角度制
	action_forward.forward_angle_mileage[1] = Moto_Data1[5].angle_sum*360.0f/19.0f/8192.0f; //角度里程 角度制
	action_forward.forward_mileage[0] = Moto_Data1[4].angle_sum/19.0f/8192.0f*38.0f*3.14f; //距离里程 单位mm
	action_forward.forward_mileage[1] = Moto_Data1[5].angle_sum/19.0f/8192.0f*38.0f*3.14f; //距离里程 单位mm
	
//	action_forward.forward_init_time++;
//	if(action_forward.forward_init_time<1000)
//	{
//		action_forward.forward_speedset[0] = action_forward.forward_speedset[1] = 0;
//		init_angle = Moto_Data1[4].angle;
//	}
//	else
//	{
//		init_angle = init_angle;
//		action_forward.forward_init_time = 1000;
//		
//	}
//	action_forward.forward_last_angle[0] = action_forward.forward_angle[0];
//	action_forward.forward_angle_sum[0] = (action_forward.forward_angle[0] - action_forward.forward_last_angle[0]);
//	asd+=action_forward.forward_angle_sum[0];
//	
//	action_forward.forward_last_angle[1] = action_forward.forward_angle[1];
//	action_forward.forward_angle_sum[1] = (action_forward.forward_angle[1] - action_forward.forward_last_angle[1]);
//	sdf+=action_forward.forward_angle_sum[1];//	asd+=action_forward.forward_angle_sum[0];
	//右侧电机最前堵转限位
	if(Moto_Data1[4].current>4900)//通过电流值判断右侧是否堵转
	{
		action_forward.limit_flag[0] = 1;//标志位为1
		action_forward.forward_limit_max[0] = action_forward.forward_mileage[0];//此时最大值为里程最大值 不为定值
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		action_forward.forward_limit_max[0] = action_forward.forward_limit_max[0];//确定最大值 为堵转位置
	}
	//左侧电机最前堵转限位
		if(Moto_Data1[5].current<-4900)//通过电流值判断左侧是否堵转
	{
		action_forward.limit_flag[1] = 1;
		action_forward.forward_limit_max[1] = action_forward.forward_mileage[1];//此时最大值为里程最大值 不为定值
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		action_forward.forward_limit_max[1] = action_forward.forward_limit_max[1];//确定最大值 为堵转位置
	}
	
	if(action_forward.limit_flag[1] == 1&&action_forward.limit_flag[0] == 1)//通过标志为对运动区间限位
	{
		action_forward.forward_interval[0] = action_forward.forward_mileage[0] - action_forward.forward_limit_max[0];//计算运动区间
		action_forward.forward_interval[1] = action_forward.forward_mileage[1] - action_forward.forward_limit_max[1];
    if(action_forward.forward_interval[0]<-438.0f)//右侧电机区间最小值
	  {
		  action_forward.forward_interval[0] = -438.0f;
		  if(action_forward.forward_speedset[0]<0)//到达区间最小值时速度大于零 即不可向后加以速度
		  {
			  action_forward.forward_speedset[0] = 0;
		  }
	  }
    if(action_forward.forward_interval[0]>-0.5f)//右侧电机区间最大值
	  {
	  	action_forward.forward_interval[0] = -0.5f;
	  	if(action_forward.forward_speedset[0]>0)//到达区间最大值时速度大于零 即不可向前加以速度
	  	{
	  		action_forward.forward_speedset[0] = 0;
	  	}
	  }	
		
		if(action_forward.forward_interval[1]>438.0f)
	  {
		  action_forward.forward_interval[1] = 438.0f;
		  if(action_forward.forward_speedset[1]>0)
		  {
		  	action_forward.forward_speedset[1] = 0;
		  }
	  }
    if(action_forward.forward_interval[1]<0.5f)
	  {
	  	action_forward.forward_interval[1] = 0.5f;
	  	if(action_forward.forward_speedset[1]<0)
	  	{
	  		action_forward.forward_speedset[1] = 0;
	  	}
	  }			
		
  }

	//右侧电机最后堵转限位
	if(Moto_Data1[4].current<-4800)
	{
		action_forward.limit_flag[0] = 2;
		action_forward.forward_limit_min[0] = action_forward.forward_mileage[0];
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		action_forward.forward_limit_min[0] = action_forward.forward_limit_min[0];
	}
	//左侧电机最后堵转限位
		if(Moto_Data1[5].current>4800)
	{
		action_forward.limit_flag[1] = 2;
		action_forward.forward_limit_min[1] = action_forward.forward_mileage[1];
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		action_forward.forward_limit_min[1] = action_forward.forward_limit_min[1];
	}
	if(action_forward.limit_flag[1] == 2&&action_forward.limit_flag[0] == 2)
	{
		action_forward.forward_interval[0] = action_forward.forward_mileage[0] - action_forward.forward_limit_min[0];
		action_forward.forward_interval[1] = action_forward.forward_mileage[1] - action_forward.forward_limit_min[1];
    if(action_forward.forward_interval[0]>438.0f)
	  {
		  action_forward.forward_interval[0] = 438.0f;
		  if(action_forward.forward_speedset[0]>0)
		  {
			  action_forward.forward_speedset[0] = 0;
		  }
	  }
    if(action_forward.forward_interval[0]<0.5f)
	  {
	  	action_forward.forward_interval[0] = 0.5f;
			action_forward.limit_init_ready_f[0] = 1;//限位到达最小位置后代表到达初始化位置
	  	if(action_forward.forward_speedset[0]<0)
	  	{
	  		action_forward.forward_speedset[0] = 0;
	  	}
	  }	
		else action_forward.limit_init_ready_f[0] = 0;//未到达初始化位置
		
		if(action_forward.forward_interval[1]<-438.0f)
	  {
		  action_forward.forward_interval[1] = -438.0f;
		  if(action_forward.forward_speedset[1]<0)
		  {
		  	action_forward.forward_speedset[1] = 0;
		  }
	  }
    if(action_forward.forward_interval[1]>-0.5f)
	  {
	  	action_forward.forward_interval[1] = -0.5f;
			action_forward.limit_init_ready_f[1] = 1;//限位到达最小位置后代表到达初始化位置
	  	if(action_forward.forward_speedset[1]>0)
	  	{
	  		action_forward.forward_speedset[1] = 0;
	  	}
	  }
		else action_forward.limit_init_ready_f[1] = 0;//未到达初始化位置
		
  }
		
	
	
	
}



static void pitch_control(void)
{
	
//	if(Moto_Data2[2].current>6200)
//	{
//		action_pitch.sucker_pitch_angleset = action_pitch.sucker_pitch_angleset;
//	}
	
//	action_pitch.sucker_pitch_last_angle
	action_pitch.pitch_angle_mileage = Moto_Data2[2].angle_sum*360.0f/19.0f/8192.0f; //角度里程 角度制
	
	
	action_pitch.sucker_pitch_angleset_sum = (action_pitch.sucker_pitch_angleset - action_pitch.sucker_pitch_last_angleset);
	
	action_pitch.sucker_pitch_angle_add+=action_pitch.sucker_pitch_angleset_sum;
  if(action_pitch.limit_init_p == 0)
	{
	  if(action_pitch.sucker_pitch_angle_add - action_pitch.pitch_angle_mileage>15||action_pitch.sucker_pitch_angle_add - action_pitch.pitch_angle_mileage<-15)
  	{
	  	action_pitch.pitch_limit_flag = 1;
	  	action_pitch.pitch_limit_max = action_pitch.pitch_angle_mileage;
	  	action_pitch.pitch_limit_min = action_pitch.pitch_limit_max - 278.0f;
//		action_pitch.pitch_limit_min = action_pitch.sucker_pitch_angleset;
//		action_pitch.pitch_limit_min = action_pitch.pitch_limit_max - 278.0f;

	  	
  		action_pitch.limit_init_p = 1;
    
  	}
  	else		
  	{
	  	action_pitch.pitch_limit_flag = 0;
//		action_pitch.pitch_limit_min = action_pitch.pitch_limit_min;
    }
    

  }
	if(action_pitch.pitch_limit_flag == 1)
	  {
	 		action_pitch.pitch_max_angleset = action_pitch.sucker_pitch_angleset;
	  	if(action_pitch.sucker_pitch_angle_add > action_pitch.pitch_angle_mileage)
	  	{
	  		action_pitch.sucker_pitch_angle_add = action_pitch.pitch_angle_mileage;
//	  		action_pitch.sucker_pitch_angleset = action_pitch.pitch_angle_mileage;
				action_pitch.limit_init_ready_p = 1;
	  	}
	  }
		  if(action_pitch.pitch_angle_mileage<action_pitch.pitch_limit_max-5.0f)
	  {
	  	action_pitch.limit_init_ready_p = 0;
	  }
	if(action_pitch.limit_init_p == 1)
	{
	if(action_pitch.sucker_pitch_angleset<action_pitch.pitch_limit_min)
	{
		action_pitch.sucker_pitch_angleset-=0;
		action_pitch.sucker_pitch_angleset = action_pitch.pitch_limit_min;
	}
	if(action_pitch.sucker_pitch_angleset>action_pitch.pitch_limit_max)
	{
		action_pitch.sucker_pitch_angleset+=0;
		action_pitch.sucker_pitch_angleset = action_pitch.pitch_limit_max;
	}
  }
}



static void gimbal_roll_control(void)
{
	
	
	gimbal_move.gimbal_roll_angle_mileage = Moto_Data2[0].angle_sum*360.0f/36.0f/8192.0f; //角度里程 角度制
	gimbal_move.gimbal_roll_mileage = Moto_Data2[0].angle_sum/19.0f/8192.0f*100.0f*3.14f; //距离里程 单位mm
	
	//右侧电机最前堵转限位
	if(Moto_Data2[0].current<-3000)//通过电流值判断右侧是否堵转
	{
		gimbal_move.gimbal_roll_limit_flag = 1;//标志位为1
		gimbal_move.gimbal_roll_limit_max = gimbal_move.gimbal_roll_mileage;//此时最大值为里程最大值 不为定值
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		gimbal_move.gimbal_roll_limit_max = gimbal_move.gimbal_roll_limit_max;//确定最大值 为堵转位置
	}

	if(gimbal_move.gimbal_roll_limit_flag == 1)//通过标志为对运动区间限位
	{
		gimbal_move.gimbal_roll_interval = gimbal_move.gimbal_roll_mileage - gimbal_move.gimbal_roll_limit_max;//计算运动区间
    if(gimbal_move.gimbal_roll_interval>520.0f)//右侧电机区间最小值
	  {
		  gimbal_move.gimbal_roll_interval = 520.0f;
		  if(gimbal_move.gimbal_roll_speed_set>0)//到达区间最小值时速度大于零 即不可向后加以速度
		  {
			  gimbal_move.gimbal_roll_speed_set = 0.0f;
		  }
	  }
    if(gimbal_move.gimbal_roll_interval<0.6f)//右侧电机区间最大值
	  {
	  	gimbal_move.gimbal_roll_interval = 0.6f;
	  	if(gimbal_move.gimbal_roll_speed_set<0)//到达区间最大值时速度大于零 即不可向前加以速度
	  	{
	  		gimbal_move.gimbal_roll_speed_set = 0.0f;
	  	}
	  }	

		
  }
	
	
	
		if(Moto_Data2[0].current>3000)//通过电流值判断右侧是否堵转
	{
		gimbal_move.gimbal_roll_limit_flag = 2;//标志位为1
		gimbal_move.gimbal_roll_limit_max = gimbal_move.gimbal_roll_mileage;//此时最大值为里程最大值 不为定值
//		(Moto_Data1[4].angle_sum*360.0/19.0/8192.0) = 0;
	}
	else
	{
		gimbal_move.gimbal_roll_limit_max = gimbal_move.gimbal_roll_limit_max;//确定最大值 为堵转位置
	}

	if(gimbal_move.gimbal_roll_limit_flag == 2)//通过标志为对运动区间限位
	{
		gimbal_move.gimbal_roll_interval = gimbal_move.gimbal_roll_mileage - gimbal_move.gimbal_roll_limit_max;//计算运动区间
    if(gimbal_move.gimbal_roll_interval<-520.0f)//右侧电机区间最小值
	  {
		  gimbal_move.gimbal_roll_interval = -520.0f;
		  if(gimbal_move.gimbal_roll_speed_set<0)//到达区间最小值时速度大于零 即不可向后加以速度
		  {
			  gimbal_move.gimbal_roll_speed_set = 0.0f;
		  }
	  }
    if(gimbal_move.gimbal_roll_interval>0.6f)//右侧电机区间最大值
	  {
	  	gimbal_move.gimbal_roll_interval = 0.6f;
	  	if(gimbal_move.gimbal_roll_speed_set>0)//到达区间最大值时速度大于零 即不可向前加以速度
	  	{
	  		gimbal_move.gimbal_roll_speed_set = 0.0f;
	  	}
	  }	

		
  }
}



static void roll_control(void)
{
		action_roll.roll_angle = action_roll.sucker_roll_angleset*19;
		
	if(action_roll.roll_angle>50)
	{
		action_roll.sucker_roll_angleset=50.0f/19.0f;
	}	
	if(action_roll.roll_angle<-310)
	{
		action_roll.sucker_roll_angleset=-310.0f/19.0f;
	}
	
	
}
	fp32 aiuf,uawhfuaf,qegqeg;
fp32 iqufhi;
static void action_keyboard_control(void)
{

			rc_deadband_limit((fp32)rc_ctrl.rc.ch[2],aiuf,ACTION_RC_DEADLINE);
			rc_deadband_limit((fp32)rc_ctrl.rc.ch[3],uawhfuaf,ACTION_RC_DEADLINE);
			rc_deadband_limit((fp32)rc_ctrl.rc.ch[1],qegqeg,ACTION_RC_DEADLINE);
  //------------------------模式切换-------------------------//
//	if(aiuf<-40&&Key_A.mode_switch_flag == 0)
//	{
////	if(gimbal_move.rc_gimbal_roll>30&&Key_A.mode_switch_flag == 0)
////	{	
//		//SHIFT键切换到上层移动模式
//		Key_A.control_A++;
//		if(Key_A.control_A > 300)
//		{
//		  Key_A.mode_switch_flag = 1;
//		  key_mode_A = ACTION_MOVE;
//		}
//	}
//	else if(aiuf>40)
//	{
////	else if(gimbal_move.rc_gimbal_roll<-30&&Key_A.mode_switch_flag == 1)
////	{
//		//CTRL键切换到下层移动模式
//		Key_A.control_A++;
//		if(Key_A.control_A > 300)
//		{
////		  Key_A.mode_switch_flag = 0;
//		  key_mode_A = ACTION_CHASSIS;
//		}
//		
//		
//		if(uawhfuaf>40)
//		{
//			Key_A.control_A = 0;
//			Key_A.control_gold++;
//			if(Key_A.control_gold > 300)
//		  {
//		    Key_A.mode_switch_flag = 2;
//		    key_mode_A = ACTION_GOLD;
//	  	}
//		}
//		else if(uawhfuaf<-40)
//		{
//			Key_A.control_A = 0;
//			Key_A.control_silver++;
//			if(Key_A.control_silver > 300)
//		  {
//		     Key_A.mode_switch_flag = 3;
//		    key_mode_A = ACTION_SILVER;
//	  	}
//		}
//		else if(qegqeg>200)
//		{
//			Key_A.control_A = 0;
//			Key_A.control_catch++;
//			if(Key_A.control_catch > 300)
//		  {
//		     Key_A.mode_switch_flag = 4;
//		    key_mode_A = ACTION_CATCH;
//	  	}
//		}
//		else
//		{
//			Key_A.control_gold = 0;
//			Key_A.control_silver = 0;
//			Key_A.control_catch = 0;
//			
//		}
//	}
//	else Key_A.control_A = 0;
	
//  //------------------------模式切换-------------------------//
	if(uawhfuaf>40)
	{
		
		  key_mode_A = ACTION_CATCH;
		
	}
	
//		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT&&Key_A.mode_switch_flag == 0)
//	{
//	if(aiuf>30&&Key_A.fast_switch_flag == 0)
//	{	
	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT)
	{	
		//SHIFT键切换到上层移动模式
		Key_A.control_Aa++;
		Key_A.mode_switch_flag = 1;
		if(Key_A.control_Aa > 300)
		{
		  key_mode_A = ACTION_MOVE;
		}
	}
//	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL&&Key_A.fast_switch_flag == 0)
//	{
////	else if(aiuf<-30&&Key_A.fast_switch_flag == 0)
////	{
////	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL&&Key_A.mode_switch_flag == 0)
////	{
//		//CTRL键切换到下层移动模式
//		Key_A.control_A++;
//		Key_A.mode_switch_flag = 1;
//		if(Key_A.control_A > 300)
//		{
//		  key_mode_A = ACTION_CHASSIS;
//		}
//	}
	else {Key_A.control_Aa = 0;
		Key_A.mode_switch_flag = 0;}
	
	
	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL)
	{
//		if(aiuf<-30)
//	{
//		Key_A.mode_switch_flag = 0;
//		Key_A.fast_switch_flag = 0;
    Key_A.control_Ac++;
		Key_A.mode_switch_flag = 1;
    if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)
		{
//		if(uawhfuaf>30)
//		{
			
			Key_A.control_Ac = 0;
			Key_A.control_gold++;
			if(Key_A.control_gold > 300)
		  {
//				Key_A.mode_switch_flag = 1;
		    Key_A.fast_switch_flag = 1;
		    key_mode_A = ACTION_GOLD;
	  	}
		}
		else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)
		{
//				else if(uawhfuaf<-30)
//		{
			Key_A.control_Ac = 0;
			Key_A.control_silver++;
			if(Key_A.control_silver > 300)
		  {
//			  Key_A.mode_switch_flag = 1;
		    Key_A.fast_switch_flag = 2;
		    key_mode_A = ACTION_SILVER;
	  	}
		}
	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)
	{
//				else if(qegqeg>30)
//		{
			Key_A.control_Ac = 0;
			Key_A.control_catch++;
			if(Key_A.control_catch > 300)
		  {
//			  Key_A.mode_switch_flag = 1;
		    Key_A.fast_switch_flag = 3;
		    key_mode_A = ACTION_CATCH;
	  	}
		}
	
		else
		{
			Key_A.fast_switch_flag = 0;
			Key_A.control_gold = 0;
			Key_A.control_silver = 0;
			Key_A.control_catch = 0;
					
		if(Key_A.control_Ac > 300)
		{
		  key_mode_A = ACTION_CHASSIS;
		}
		}
	}
	else
	{Key_A.mode_switch_flag = 0;
  	Key_A.control_Ac = 0;
	}
	
	
	 //------------------------初始化模式-------------------------//
	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)
	{
		//R键切换到初始化模式
		Key_A.init_A++;
		if(Key_A.init_A > 300)
		{
		  Key_A.init_switch_flag = 1;
		}
		 
	}
	else 
	{
		Key_A.init_A = 0;
    Key_A.init_switch_flag = 0;
	}
	
	if(Key_A.init_switch_flag == 1)
    {
		  calibration();
		  forward_control();
		  pitch_control();
		  uplift_control();
      if(action_forward.limit_init_ready_f[0]==1&&action_forward.limit_init_ready_f[1]==1&&action_uplift.limit_init_ready_u[0]==1&&action_uplift.limit_init_ready_u[1]==1&&action_pitch.limit_init_ready_p==1)
      {
		  	Key_A.init_switch_flag = 0;//退出初始化模式标志位
		  	Key_A.init_ready_flag = 1;
		  }
	  }
	
	
// //------------------------救急模式-------------------------//
	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT&&rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)
	{
//	if(aiuf>30)
//	{
		//R键切换到初始化模式
		Key_A.help_A++;
		if(Key_A.help_A > 100)
		{
		  Key_A.help_switch_flag_A = 1;
		}
	}
	else Key_A.help_A = 0;
  if(Key_A.help_switch_flag_A == 1)
  {
    
		action_forward.limit_flag[0] = 0;
		action_forward.limit_flag[1] = 0;
		action_uplift.uplift_limit_flag[0] = 0;
		action_uplift.uplift_limit_flag[1] = 0;
		action_pitch.pitch_limit_flag = 0;
		action_pitch.limit_init_p = 0;
			Key_A.help_switch_flag_A = 0;//退出初始化模式标志位
		action_mode = ACTION_EMPTY;
			Key_A.help_ready_flag_A = 1;
		
	}


  //------------------------上层移动模式-------------------------//
  if(key_mode_A == ACTION_MOVE&&Key_A.init_switch_flag == 0)
  {
//  if(key_mode_A == ACTION_MOVE&&Key_A.mode_switch_flag == 0)
//  {
  //------------------------抬升-------------------------//
	  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_W){
	    Key_A.time_up++;
	  	if(Key_A.time_up > 300)
	  	Key_A.time_up_ready = 1;}
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_S){
	    Key_A.time_up++;
	  	if(Key_A.time_up > 300)
	  	Key_A.time_up_ready = -1;}
	  else
	    {Key_A.time_up = 0;
  		Key_A.time_up_ready = 0;}
  //-------------------------------------------------//	
  	 	if(Key_A.time_up_ready  == 1)
  	{
  		Key_A.speed_up_time++;
  		if(Key_A.speed_up_time>1200)Key_A.speed_up_time=1200;
	  	Key_A.q=Key_A.speed_up_time*0.0003f;
	  	action_uplift.uplift_angleset[0] += Key_A.q;
     action_uplift.uplift_angleset[1] -= Key_A.q;}
	  else if(Key_A.time_up_ready  == -1)
	  {Key_A.speed_up_time++;
  		if(Key_A.speed_up_time>1200)Key_A.speed_up_time=1200;
  		Key_A.q=Key_A.speed_up_time*0.0003f;
  		action_uplift.uplift_angleset[0] -= Key_A.q;
     action_uplift.uplift_angleset[1] += Key_A.q;}
	  else 
	  {Key_A.speed_up_time=0;
	  	Key_A.q=0;
		  action_uplift.uplift_angleset[0] += 0;
      action_uplift.uplift_angleset[1] -= 0;}
	uplift_control();

	 
  //------------------------前伸-------------------------//
  	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)
  	  {Key_A.time_forward++;
  		if(Key_A.time_forward > 300)
	  	Key_A.time_forward_ready = 1;}
  	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)
  	  {Key_A.time_forward++;
	  	if(Key_A.time_forward > 300)
	  	Key_A.time_forward_ready = -1;}
	  else
	    {Key_A.time_forward = 0;
	  	Key_A.time_forward_ready = 0;}
  //-------------------------------------------------//	
	  if(Key_A.time_forward_ready  == 1)
	  {action_forward.forward_speedset[0] = ACTION_VX_MAX/5;//
     action_forward.forward_speedset[1] = -ACTION_VX_MAX/5;}
	  else if(Key_A.time_forward_ready  == -1)
	  {action_forward.forward_speedset[0] = -ACTION_VX_MAX/5;//
     action_forward.forward_speedset[1] = ACTION_VX_MAX/5;}
    else {action_forward.forward_speedset[0] = 0;//
     action_forward.forward_speedset[1] = 0;}	 
		
		 forward_control();
	
	 
  //------------------------pitch轴-------------------------//
  	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)
  	  {Key_A.time_pitch++;
  		if(Key_A.time_pitch > 300)
  		Key_A.time_pitch_ready = 1;}
  	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)
  	  {Key_A.time_pitch++;
  		if(Key_A.time_pitch > 300)
  		Key_A.time_pitch_ready = -1;}
  	else
  	  {Key_A.time_pitch = 0;
  		Key_A.time_pitch_ready = 0;}
  //-------------------------------------------------//	
  	if(Key_A.time_pitch_ready  == 1)
  	  {Key_A.speed_pitch_time++;
  		if(Key_A.speed_pitch_time>1600)Key_A.speed_pitch_time=1600;
  		Key_A.pitch_angle_cail=Key_A.speed_pitch_time*0.00015f;
	  	action_pitch.sucker_pitch_angleset += Key_A.pitch_angle_cail;}
  	else if(Key_A.time_pitch_ready  == -1)
	    {Key_A.speed_pitch_time++;
	  	if(Key_A.speed_pitch_time>1600)Key_A.speed_pitch_time=1600;
  		Key_A.pitch_angle_cail=Key_A.speed_pitch_time*0.00015f;
  		action_pitch.sucker_pitch_angleset -= Key_A.pitch_angle_cail;}
  	else 
  	  {Key_A.speed_pitch_time=0;
    	Key_A.pitch_angle_cail=0;
	    action_pitch.sucker_pitch_angleset += 0;}	 
	pitch_control();
	 
  //------------------------roll轴-------------------------//
	  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)
	    {Key_A.time_roll++;
	  	if(Key_A.time_roll > 100)
  		Key_A.time_roll_ready = 1;}
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)
  	  {Key_A.time_roll++;
	  	if(Key_A.time_roll > 100)
	  	Key_A.time_roll_ready = -1;}
	  else
	    {Key_A.time_roll = 0;
	  	Key_A.time_roll_ready = 0;}
  //-------------------------------------------------//	
	  if(Key_A.time_roll_ready  == 1)
	  {action_roll.sucker_roll_angleset += 0.005f;}
	  else if(Key_A.time_roll_ready  == -1)
	  {action_roll.sucker_roll_angleset -= 0.005f;}
	  else 
	  {action_roll.sucker_roll_angleset += 0.0f;}	 
	 
		roll_control();
		
		
  ////------------------------气泵-------------------------//
  	if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)
  	{
			Key_A.time_pump++;
  		if(Key_A.time_pump>100)
			{
				PUMP_OPEN;
			}
  	}	
  	else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)
  	{
			Key_A.time_pump++;
  		if(Key_A.time_pump>100)
			{
				PUMP_OFF;
			}
  	}	
    else Key_A.time_pump = 0;
	
	
//  		gimbal_move.gimbal_roll_speed_set = -(fp32)rc_ctrl.mouse.x * 0.01f;//TTT
  	if((fp32)rc_ctrl.mouse.x<-20)
		{
			gimbal_move.gimbal_roll_speed_set = 0.2f;
		}
  	else if((fp32)rc_ctrl.mouse.x>20)
		{
			gimbal_move.gimbal_roll_speed_set = -0.2f;
		}
		else gimbal_move.gimbal_roll_speed_set = 0.0f;
		
  	if(gimbal_move.gimbal_roll_speed_set>2.0f)
  	{
  		gimbal_move.gimbal_roll_speed_set = 2.0f;
  	}
  	if(gimbal_move.gimbal_roll_speed_set<-2.0f)
  	{
  		gimbal_move.gimbal_roll_speed_set = -2.0f;
  	}
	
//  	gimbal_roll_control();
 


		Key_A.qw2wr = 1;


  }
  //------------------------下层移动模式-------------------------//
	else if(key_mode_A == ACTION_CHASSIS&&Key_A.init_switch_flag == 0)
	{
//  else if(key_mode_A == ACTION_CHASSIS&&Key_A.mode_switch_flag == 0)
//  {

		action_forward.forward_speedset[0] = 0;
		action_forward.forward_speedset[0] = 0;
		
		action_uplift.uplift_angleset[0] += 0;
		action_uplift.uplift_angleset[0] -= 0;
		
		action_pitch.sucker_pitch_angleset += 0;
	  action_roll.sucker_roll_angleset += 0;
		
		//一键掉头
	  gimbal_roll_control();
	  if(gimbal_move.gimbal_roll_limit_flag == 1)
	  {
	    if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)
	    {
		  Key_A.qw2wr = 2;
		   if(gimbal_move.gimbal_roll_interval<160.0f)
		  	{
		  	  gimbal_move.gimbal_roll_speed_set = 1.0f;
		  		if(gimbal_move.gimbal_roll_interval>160.0f)
		  	  {
		  	  gimbal_move.gimbal_roll_speed_set = 0.0f;
		  		}
		  	}
			  if(gimbal_move.gimbal_roll_interval>192.0f)
			  {
			    gimbal_move.gimbal_roll_speed_set = -1.0f;
			  	if(gimbal_move.gimbal_roll_interval<192.0f)
			    {
			    gimbal_move.gimbal_roll_speed_set = 0.0f;
			  	}
			  }
		   }
		  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)
	   {
		  Key_A.qw2wr = 2;
		  	gimbal_move.gimbal_roll_speed_set = 1.0f;
		  	if(gimbal_move.gimbal_roll_interval>458.0f)
		  	{
		  		gimbal_move.gimbal_roll_speed_set = 0;
		  	}
		  }
	   else {gimbal_move.gimbal_roll_speed_set = 0;}
	   }
	 }
	else if(key_mode_A == ACTION_CATCH)
	{
		
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)//
		{
//	  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)//缓慢左旋
//		{
			if(action_uplift.uplift_angleset[0]<14.0f)
			{
				action_uplift.uplift_angleset[0] = 14.0f;
			}
			action_uplift.uplift_angleset[0]+=0.15f;
			action_uplift.uplift_angleset[1]-=0.15f;
			uplift_control();
//			if(action_uplift.uplift_interval[0]>22.0f)
//			{
				if(action_uplift.uplift_angleset[0]>75.0f)
				{
					action_uplift.uplift_angleset[0] += 0;
				action_uplift.uplift_angleset[0] = 75.0f;
				Key_A.catch_time++;}
//			}
//			if(action_uplift.uplift_interval[1]<-20.0)
//			{
				if(action_uplift.uplift_angleset[1]<-60.0f)
				{
				action_uplift.uplift_angleset[1] -= 0;
				action_uplift.uplift_angleset[1] = -60.0f;}	
				
//			}
			if(Key_A.catch_time>100)
			{
				 Key_A.speed_pitch_catch_time++;
			 	 Key_A.pitch_angle_catch_limit = action_pitch.pitch_limit_max-90.0f;
  		   if(Key_A.speed_pitch_catch_time>1400)
				 {
					 Key_A.speed_pitch_catch_time=1400;
  		     Key_A.pitch_angle_catch=Key_A.speed_pitch_catch_time*0.00015f;
	     	   action_pitch.sucker_pitch_angleset += Key_A.pitch_angle_catch;
					 
					 if(action_pitch.sucker_pitch_angleset>Key_A.pitch_angle_catch_limit)
					 {action_pitch.sucker_pitch_angleset = Key_A.pitch_angle_catch_limit;
						 Key_A.pitch_angle_catch_limit_ready = 1;}
					 
					 action_forward.forward_speedset[0] = -1.0f;
           action_forward.forward_speedset[1] = 1.0f;
					 forward_control();
					 

				 }
				 
			}

	 }
					if(Key_A.pitch_angle_catch_limit_ready == 1)
				 {
				 	 action_uplift.uplift_angleset[0]-=0.15f;
			     action_uplift.uplift_angleset[1]+=0.15f;
					 uplift_control();
					 if(action_uplift.limit_init_ready_u[0] == 1&&action_uplift.limit_init_ready_u[1] == 1)
					 {
						 Key_A.pitch_angle_catch_limit_ready = 0;
						 Key_A.speed_pitch_catch_time = 0;
						 Key_A.catch_time = 0;
					 }
				 }
					
		
		
	}
	else if(key_mode_A == ACTION_GOLD)
	{
		
	  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)//缓慢左旋
		{
			
			action_uplift.uplift_angleset[1] -= 0.1f;
		}
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)//缓慢左旋
		{
			
			action_uplift.uplift_angleset[0] += 0.1f;
		}
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)//缓慢左旋
		{
			
			action_uplift.uplift_angleset[1] += 0.1f;
		}
	  else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)//缓慢左旋
		{
			
			action_uplift.uplift_angleset[0] -= 0.1f;
			action_uplift.uplift_angleset[1] -= 0.1f;
		}
		else 
		{
			action_uplift.uplift_angleset[0] += 0.0f;
			action_uplift.uplift_angleset[1] += 0.0f;
		}
				
	}
	else if(key_mode_A == ACTION_SILVER)
	{
				 int silver_limit_flag;
		int time_sliver;
		if(action_uplift.uplift_angleset[0]<14.0f)
		{
			action_uplift.uplift_angleset[0] = 14.0f;
			action_uplift.uplift_angleset[0] -= 0.0f;
		}		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)//
		{
//		  time_sliver++;
//			if(time_sliver>200)
//			{
//	  if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)//缓慢左旋
//		{

			uplift_control();
//			if(action_uplift.uplift_interval[0]>22.0f)
//			{
			if(action_uplift.uplift_angleset[0]<465.0f)
			{
			  action_uplift.uplift_angleset[0]+=0.15f;
				if(action_uplift.uplift_angleset[0]>465.0f)
				{
					iqufhi = 1;
					action_uplift.uplift_angleset[0] += 0;
				action_uplift.uplift_angleset[0] = 465.0f;
				}
      }
//			}
//			if(action_uplift.uplift_interval[1]<-20.0)
//			{
			if(action_uplift.uplift_angleset[1]>-451.0f)
			{
			  action_uplift.uplift_angleset[1]-=0.15f;
				if(action_uplift.uplift_angleset[1]<-451.0f)
				{
				action_uplift.uplift_angleset[1] -= 0;
				action_uplift.uplift_angleset[1] = -451.0f;}	
			}
			
			
			if(action_uplift.uplift_angleset[0]>465.0f)
			{
			  action_uplift.uplift_angleset[0]-=0.15f;
				if(action_uplift.uplift_angleset[0]<465.0f)
				{
					iqufhi = 2;
					action_uplift.uplift_angleset[0] -= 0;
				action_uplift.uplift_angleset[0] = 465.0f;
				}
      }
//			}
//			if(action_uplift.uplift_interval[1]<-20.0)
//			{
			if(action_uplift.uplift_angleset[1]<-451.0f)
			{
			  action_uplift.uplift_angleset[1]+=0.15f;
				if(action_uplift.uplift_angleset[1]>-451.0f)
				{
				action_uplift.uplift_angleset[1] += 0;
				action_uplift.uplift_angleset[1] = -451.0f;}	
			}			
			
			if(iqufhi == 1)
			{
				Key_A.catch_time++;
			}
			else if(iqufhi == 2)
			{
				Key_A.catch_time++;
			}
			else Key_A.catch_time = 0;
//			}
			if(Key_A.catch_time>100)
			{
				 Key_A.speed_pitch_silver_time++;
			 	 Key_A.pitch_angle_silver_limit = action_pitch.pitch_limit_max-176.0f;
  		     Key_A.pitch_angle_silver=Key_A.speed_pitch_silver_time*0.00015f;
	     	   action_pitch.sucker_pitch_angleset -= Key_A.pitch_angle_silver;
  		   if(Key_A.speed_pitch_silver_time>1200)
				 {
					 Key_A.speed_pitch_silver_time=1200;		 
			      action_roll.sucker_roll_angleset = -6.8f;
//					 if(action_roll.sucker_roll_angleset<-6.8)
//					 action_forward.forward_speedset[0] = -1.0;
//           action_forward.forward_speedset[1] = 1.0;
//					 forward_control();
					 silver_limit_flag = 1;
				 }
				 
				 if(action_pitch.sucker_pitch_angleset<Key_A.pitch_angle_silver_limit&&silver_limit_flag == 1)
					 {action_pitch.sucker_pitch_angleset = Key_A.pitch_angle_silver_limit;
						 Key_A.pitch_angle_silver_limit_ready = 1;}
				 
			}
			else Key_A.speed_pitch_silver_time = 0;
//		 }

	 }
		else {Key_A.catch_time = 0;
	 iqufhi = 0;}
		if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_E&&Key_A.pitch_angle_silver_limit_ready == 1)
		{
			silver_limit_flag = 0;
			action_uplift.uplift_angleset[0]+=0.15f;
			action_uplift.uplift_angleset[1]-=0.15f;

				
			if(action_uplift.uplift_angleset[0]>660.0f)
				{
					action_uplift.uplift_angleset[0] += 0;
				action_uplift.uplift_angleset[0] = 660.0f;
				Key_A.catch_time_over++;}
//			}
//			if(action_uplift.uplift_interval[1]<-20.0)
//			{
				if(action_uplift.uplift_angleset[1]<-646.0f)
				{
				action_uplift.uplift_angleset[1] -= 0;
				action_uplift.uplift_angleset[1] = -646.0f;}	
				
//			}
			if(Key_A.catch_time_over>100)
			{
				if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_R&&Key_A.init_switch_flag == 0)
	      {
	     	//R键切换到初始化模式
	    	  Key_A.init_A++;
		      if(Key_A.init_A > 300)
		      {
		      Key_A.init_switch_flag = 1;
		      } 
	     }
	     else Key_A.init_A = 0;
       if(Key_A.init_switch_flag == 1)
       {
		     calibration();
         if(action_forward.limit_init_ready_f[0]==1&&action_forward.limit_init_ready_f[1]==1&&action_uplift.limit_init_ready_u[0]==1&&action_uplift.limit_init_ready_u[1]==1&&action_pitch.limit_init_ready_p==1)
         {
		     	Key_A.init_switch_flag = 0;//退出初始化模式标志位
		     	Key_A.init_ready_flag = 1;
	    	  }
       }


		 }				 
			
			
			
		}
		else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)
		{
			if(action_roll.sucker_roll_angleset<-12.1f)
			{
				action_roll.sucker_roll_angleset += 0.005f;
				if(action_roll.sucker_roll_angleset>-12.1f)
				{
					action_roll.sucker_roll_angleset=-12.1f;
				}
			}
			else if(action_roll.sucker_roll_angleset>-12.1f)
			{
				action_roll.sucker_roll_angleset -= 0.005f;
				if(action_roll.sucker_roll_angleset<-12.1f)
				{
					action_roll.sucker_roll_angleset=-12.1f;
				}				
			}
			else action_roll.sucker_roll_angleset += 0.0f;
			
			
			
		}
		else if(rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)
		{
			if(action_roll.sucker_roll_angleset<-6.8f)
			{
				action_roll.sucker_roll_angleset += 0.005f;
				if(action_roll.sucker_roll_angleset>-6.8f)
				{
					action_roll.sucker_roll_angleset=-6.8f;
				}
			}
			else if(action_roll.sucker_roll_angleset>-6.8f)
			{
				action_roll.sucker_roll_angleset -= 0.005f;
				if(action_roll.sucker_roll_angleset<-6.8f)
				{
					action_roll.sucker_roll_angleset=-6.8f;
				}				
			}
			else action_roll.sucker_roll_angleset += 0.0f;
			
			
			
		}
		else  Key_A.catch_time_over=0;
		
	}
	else key_mode_A = ACTION_BUG;
	
//	
	
	if((fp32)rc_ctrl.mouse.press_l==1)
	{
		
		PUMP_OPEN;
	}
	else if((fp32)rc_ctrl.mouse.press_r==1)
	{
		PUMP_OFF;
	}
		 
	
	
	
	
	

}

static void action_can_tx(void)
{
	//底盘电机

	//爪子
	CAN_CMD_MOTO(&hcan1,CAN_MOTO_ALL_ID_HIGH,
							 (int16_t)action_forward.forward_give_current[0],
							 (int16_t)action_forward.forward_give_current[1],
//	   0,
//	0,
							 0,
							 0);

	//反转	存矿
	CAN_CMD_MOTO(&hcan2,CAN_MOTO_ALL_ID_LOW,
//#if OVERTURN_POWER == 1
							 (int16_t)gimbal_move.gimbal_roll_current,
//	0,
							 (int16_t)action_uplift.uplift_give_current[0],
//#else
//							 0,
//							 0,
//#endif
//#if DEPOSIT_POWER == 1
							 (int16_t)action_pitch.sucker_pitch_give_current,
							 (int16_t)action_uplift.uplift_give_current[1]);
							 
//#else
//							 0,
//							 0);
//#endif
	CAN_CMD_MOTO(&hcan2,CAN_MOTO_ALL_ID_HIGH,
//#if OVERTURN_POWER == 1
							 (int16_t)action_roll.sucker_roll_give_current, 
							 
							 0,
							 0,
               0);
							 

}



