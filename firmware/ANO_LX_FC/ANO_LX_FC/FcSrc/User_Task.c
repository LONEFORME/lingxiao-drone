#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "ANO_DT_LX.h"
#include "Drv_AnoOf.h"
#include "math.h"

#define speed_x 20
#define speed_y 20
#define speed_z 20
#define speed_yaw 20

u16 pid_speed=0;
u8 mission_stage=0;//用于指示当前任务阶段
u8 mission_done_flag=0;//当前任务用于切换树莓派状态

void UserTask_OneKeyCmd(void)//一键任务
{
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    static u8 mission_step,eme_stop=1,pi_start_f=0,now_task_mode=0;
	
  //////////////////////////////////////////////////////////////////////
	//一键锁桨 第8通道拨杆位置1700<CH_8<2200
	
	if (rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1700 &&rc_in.rc_ch.st_data.ch_[ch_8_aux4] < 2200) 
	{
		if (eme_stop == 0) 
		{
			eme_stop = 1;
			//执行一键锁桨
			FC_Lock();
			pwm_to_esc.pwm_m1 = 0;
			pwm_to_esc.pwm_m2 = 0;
			pwm_to_esc.pwm_m3 = 0; 
			pwm_to_esc.pwm_m4 = 0;
		}
	} 
	else 
	{
		eme_stop = 0;
	}
	//////////////////////////////////////////////////////////////////////
	//任务启动 第7通道拨杆位置1700<CH_7<2200 或者 树莓派发送起飞指令
	if((rc_in.rc_ch.st_data.ch_[ch_7_aux3]>1700 && rc_in.rc_ch.st_data.ch_[ch_7_aux3]<2200)||(received_data.task_sta==1))//树莓派远程起飞再加判断
		{
			//还没有执行
			if(one_key_mission_f ==0)
			{
				//标记已经执行
				one_key_mission_f = 1;
				//开始流程
				mission_step = 0;
			}
		}
		else
		{
			//复位标记，以便再次执行
			if(one_key_mission_f==1)OneKey_Land();		
			one_key_mission_f = 0;		
		}
	///////////////////////////////////////////////////////////////////////
	//任务列表
		if(one_key_mission_f==1)
		{
			static u16 time_dly_cnt_ms;
			static s16 integ_x,integ_y;
			static s16 integ_x_base,integ_y_base;
			static s32 pos_x_base,pos_y_base;
			static u16 icount=0;
			mission_stage=mission_step;
			//
			switch(mission_step)
			{
				case 0:
				{
					//reset
					time_dly_cnt_ms = 0;
					mission_step +=1;
				}
				break;
				case 1://解锁
					{
						PID_init();
						
						mission_step +=FC_Unlock();
					}
				break;
				case 2://十字处起飞 默认120cm
				{
					if(time_dly_cnt_ms<3500)//转桨延时
					{
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += OneKey_Takeoff(120);
						one_key_takeoff_f=1;
					}
				}
				break;
				case 3://等三秒
				{
					if(time_dly_cnt_ms<5000)//任务延时
					{
						time_dly_cnt_ms+=20;//ms
					}
					else
					{
						time_dly_cnt_ms = 0;
						mission_step += 1;
					}
				}
				break;
				case 4://延时2s
				{
					pid_speed=0;
					PID_init();
					time_dly_cnt_ms = 0;
					mission_step += 1;
				}	
				break;
				case 5://视觉控制阶段
				{
					if(received_data.next_task_sign==0)
					{
						// 高度：用 视觉模块的高度 + 目标高度
						tar_setdata(received_data.com_x,received_data.com_y,height_set(ano_of.of_alt_cm,received_data.com_z),received_data.com_yaw);
					}
					else if(received_data.next_task_sign==1)
					{
						time_dly_cnt_ms = 0;
						mission_step = 6;
					}
					else
					{
						mission_step = 101;
					}
						
				}
				break;
				case 6://光流控制阶段
				{
					if(received_data.next_task_sign==1)
					{
						// 高度：直接用上位机给的高度
						tar_setdata(received_data.com_x,received_data.com_y,height_set(received_data.com_z,received_data.com_z),received_data.com_yaw);
					}
					else if(received_data.next_task_sign==0)
					{
						time_dly_cnt_ms = 0;
						mission_step =5;
					}
					else
					{
						mission_step = 101;
					}
				}
				break;
				default:
				{
					OneKey_Land();
				}
				break;
			}
			
		}
		else
		{
			PID_init();
			mission_step = 0;
			mission_stage=0;
			tar_setdata(0,0,0,0);
			one_key_takeoff_f=0;
		}
		
}



