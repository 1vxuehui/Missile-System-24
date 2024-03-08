
#include "UItask.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "RM_Cilent_UI.h"
#include "string.h"
#include "stdio.h"
#include "remote_control.h"
#include "referee.h"
#include "MotionTask.h"
#include "ActionTask.h"
extern int anglesr;
extern ext_game_robot_state_t robot_state;
extern ext_bullet_remaining_t bullet_remaining_t;
extern chassis_move_t chassis_move;

Graph_Data Aim[7];
String_Data strSPEED;
void UIRefreshTask_Loop(void)
{
			char tmp1[30]={0},tmp2[30]={0},tmp3[30]={0};
			memset(&strSPEED,0,sizeof(strSPEED));
			for(int k=0;k<6;k++)
			{
				memset(&Aim[k],0,sizeof(Aim[k]));
			} 										//清空图形数据
			Line_Draw(&Aim[0],"AL1",UI_Graph_ADD,5,UI_Color_Green,3,960,300,960,540);
			Line_Draw(&Aim[1],"AL2",UI_Graph_ADD,5,UI_Color_Green,2,940,520,980,520);
			Line_Draw(&Aim[2],"AL3",UI_Graph_ADD,5,UI_Color_Black,3,930,478,990,478);
			Line_Draw(&Aim[3],"AL4",UI_Graph_ADD,5,UI_Color_Yellow,2,920,380,1000,380);
			//可通过宽度
			Line_Draw(&Aim[4],"AL5",UI_Graph_ADD,6,UI_Color_Green,2,440,0,627,443);
			Line_Draw(&Aim[5],"AL6",UI_Graph_ADD,6,UI_Color_Green,2,1480,0,1293,443);
			Line_Draw(&Aim[6],"AL7",UI_Graph_ADD,5,UI_Color_Orange,3,920,400,1000,400);
			Char_Draw(&strSPEED,"CAP",UI_Graph_ADD,8,UI_Color_Green,20,strlen(tmp1),2,860,100,tmp1);
			Char_ReFresh(strSPEED);
			vTaskDelay(50);
			UI_ReFresh(7,Aim[0],Aim[1],Aim[2],Aim[3],Aim[4],Aim[5],Aim[6]);
				
	while(1)
	{
		if(rc_ctrl.key.v&KEY_PRESSED_OFFSET_Z)
		{
						Line_Draw(&Aim[0],"AL1",UI_Graph_ADD,5,UI_Color_Green,3,960,300,960,540);
			Line_Draw(&Aim[1],"AL2",UI_Graph_ADD,5,UI_Color_Green,2,940,520,980,520);
			Line_Draw(&Aim[2],"AL3",UI_Graph_ADD,5,UI_Color_Black,3,930,478,990,478);
			Line_Draw(&Aim[3],"AL4",UI_Graph_ADD,5,UI_Color_Yellow,2,920,380,1000,380);
			//可通过宽度
			Line_Draw(&Aim[4],"AL5",UI_Graph_ADD,6,UI_Color_Green,2,440,0,627,443);
			Line_Draw(&Aim[5],"AL6",UI_Graph_ADD,6,UI_Color_Green,2,1480,0,1293,443);
			Line_Draw(&Aim[6],"AL7",UI_Graph_ADD,5,UI_Color_Orange,3,920,400,1000,400);
			Char_Draw(&strSPEED,"CAP",UI_Graph_ADD,8,UI_Color_Green,20,strlen(tmp1),2,860,100,tmp1);
			Char_ReFresh(strSPEED);
			vTaskDelay(50);
			UI_ReFresh(7,Aim[0],Aim[1],Aim[2],Aim[3],Aim[4],Aim[5],Aim[6]);
		}
  }
}

void UIRefreshTask_Setup(void)
{
	
	
}




