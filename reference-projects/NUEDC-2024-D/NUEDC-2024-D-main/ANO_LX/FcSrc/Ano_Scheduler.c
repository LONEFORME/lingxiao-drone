/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：任务调度
**********************************************************************************/
#include "Ano_Scheduler.h"
#include "User_Task.h"
#include "User_Oled.h"
#include "ANO_LX.h"
#include "User_T265.h"
#include "User_Ctrl.h"
#include "User_ComBoard.h"
//////////////////////////////////////////////////////////////////////
//用户程序调度器
//////////////////////////////////////////////////////////////////////

int i=0,i_last=0;//调试用

static void Loop_1000Hz(void) //1ms执行一次
{
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
}

static void Loop_500Hz(void) //2ms执行一次
{
	//////////////////////////////////////////////////////////////////////
//Servo[0]=1;
    
    
    
	//////////////////////////////////////////////////////////////////////
}

static void Loop_200Hz(void) //5ms执行一次
{
	//////////////////////////////////////////////////////////////////////
	if(LX_FC.CtrlState==0)//普通位置环控制模式
	{
		User_Loc_Ctrl(t265.pos_x_f,t265.pos_y_f,Loc_Exp_Xcm,Loc_Exp_Ycm);//位置环控制
		User_Alt_Ctrl(ano_of.of_alt_cm,Alt_Exp_Zcm);//高度环控制
		User_Ang_Ctrl(t265.ya_f,Ang_Exp_Deg);//角度环控制
		
//	UserVisualAlignment(openmv.err_x,openmv.err_y,VisualAlignmentExpX,VisualAlignmentExpY,1);//对齐
		
//		User_Ang_Ctrl(User_Yaw,Ang_Exp_Deg);//角度环控制 滤波后响应慢
		User_RaoganSpedXcm=0;//绕杆控制量置0
		User_RaoganSpedYcm=0;//绕杆控制量置0
		User_RaoganAngDeg=0;//绕杆控制量置0
		VisualAlignmentOutXcm=0;//视觉对齐置0
		VisualAlignmentOutYcm=0;//视觉对齐置0
		SpdOutXcmFor2019=0;
		SpdOutYcmFor2019=0;
		SpdOutZcmFor2019=0;
		Fb_LocCtrlOutXcm=0;//现场编程控制量置0
		Fb_LocCtrlOutYcm=0;
		Fb_AltCtrlOutZcm=0;
		
		Lf_LocCtrlOutXcm=0;//巡线控制量输出置0
		Lf_AngCtrlOutDeg=0;
	}else
	if(LX_FC.CtrlState==1)//绕杆模式
	{
		User_Alt_Ctrl(ano_of.of_alt_cm,Alt_Exp_Zcm);//高度环控制
		User_RaoGan(openmv.distance_cm,openmv.ang,RaoGanExpRcm,RaoGanExpYcm ,RaoGanExpYawDeg,RaoGanMode);
		User_LocCtrlOutXcm=0;//位置环控制量置0
		User_LocCtrlOutYcm=0;//位置环控制量置0
		User_AngCtrlOutDeg=0;//位置环控制量置0
		VisualAlignmentOutXcm=0;//视觉对齐置0
		VisualAlignmentOutYcm=0;//视觉对齐置0
		SpdOutXcmFor2019=0;
		SpdOutYcmFor2019=0;
		SpdOutZcmFor2019=0;
		Fb_LocCtrlOutXcm=0;//现场编程控制量置0
		Fb_LocCtrlOutYcm=0;
		Fb_AltCtrlOutZcm=0;
		
		Lf_LocCtrlOutXcm=0;//巡线控制量输出置0
		Lf_AngCtrlOutDeg=0;
	}else
	if(LX_FC.CtrlState==2)//找点模式
	{
		User_Alt_Ctrl(ano_of.of_alt_cm,Alt_Exp_Zcm);//高度环控制
		User_Ang_Ctrl(t265.ya_f,Ang_Exp_Deg);//角度环控制
		User_Loc_Ctrl(t265.pos_x_f,t265.pos_y_f,Loc_Exp_Xcm,Loc_Exp_Ycm);//位置环控制{Y}
		UserVisualAlignment(openmv.err_x,openmv.err_y,VisualAlignmentExpX,VisualAlignmentExpY,1);//对齐
		User_LocCtrlOutXcm=0;//位置环控制量置0
//		User_LocCtrlOutYcm=0;//位置环控制量置0
		VisualAlignmentOutYcm=0;//视觉对齐置0
		User_RaoganSpedXcm=0;//绕杆控制量置0
		User_RaoganSpedYcm=0;//绕杆控制量置0
		User_RaoganAngDeg=0;//绕杆控制量置0
		SpdOutXcmFor2019=0;
		SpdOutYcmFor2019=0;
		SpdOutZcmFor2019=0;
		Fb_LocCtrlOutXcm=0;//现场编程控制量置0
		Fb_LocCtrlOutYcm=0;
		Fb_AltCtrlOutZcm=0;
		
		Lf_LocCtrlOutXcm=0;//巡线控制量输出置0
		Lf_AngCtrlOutDeg=0;
	}else
	if
	(LX_FC.CtrlState==3)//2019年限定巡线模式
	{
		User_Ang_Ctrl(t265.ya_f,Ang_Exp_Deg);//角度环控制
		UserVisualYCtrl(openmv.line_err_y,300,LineCtrlEnable);
		UserVisualZCtrl(0,0,0);
		User_Alt_Ctrl(ano_of.of_alt_cm,Alt_Exp_Zcm);//高度环控制
		User_LocCtrlOutXcm=0;//位置环控制量置0
		User_LocCtrlOutYcm=0;//位置环控制量置0
//		User_AltCtrlOutZcm=0;//位置环控制量置0
		User_RaoganSpedXcm=0;//绕杆控制量置0
		User_RaoganSpedYcm=0;//绕杆控制量置0
		User_RaoganAngDeg=0;//绕杆控制量置0
		VisualAlignmentOutXcm=0;//视觉对齐置0
		VisualAlignmentOutYcm=0;//视觉对齐置0
		Fb_LocCtrlOutXcm=0;//现场编程控制量置0
		Fb_LocCtrlOutYcm=0;
		Fb_AltCtrlOutZcm=0;
		
		Lf_LocCtrlOutXcm=0;//巡线控制量输出置0
		Lf_AngCtrlOutDeg=0;
	}else
	if
	(LX_FC.CtrlState==4)//现场编程倾斜起飞模式
	{
		User_Ang_Ctrl(t265.ya_f,Ang_Exp_Deg);//角度环控制
		/*倾斜起飞控制*/
		FbOneKeyTakeOff(t265.pos_x_f,t265.pos_y_f,t265.pos_z_f,Fb_Loc_Exp_Xcm,Fb_Loc_Exp_Ycm,Fb_Alt_Exp_Zcm,FbCtrlEnable);
		

		User_LocCtrlOutXcm=0;//位置环控制量置0
		User_LocCtrlOutYcm=0;//位置环控制量置0
		User_AltCtrlOutZcm=0;//位置环控制量置0
		User_RaoganSpedXcm=0;//绕杆控制量置0
		User_RaoganSpedYcm=0;//绕杆控制量置0
		User_RaoganAngDeg=0;//绕杆控制量置0
		VisualAlignmentOutXcm=0;//视觉对齐置0
		VisualAlignmentOutYcm=0;//视觉对齐置0
		SpdOutXcmFor2019=0;//2019巡线控制量置0
		SpdOutYcmFor2019=0;//
		SpdOutZcmFor2019=0;//
		
		Lf_LocCtrlOutXcm=0;//巡线控制量输出置0
		Lf_AngCtrlOutDeg=0;
		
	}else
	if(LX_FC.CtrlState==5)//巡线模式
	{
		LineFollowingCtrl(openmv.line_err_yaw,Lf_Ang_Exp_Deg,openmv.line_err_y,0,Lf_Spd_Exp_Xcm,Lf_Load_Mode,Lf_Ctrl_Enable);
		User_Alt_Ctrl(ano_of.of_alt_cm,Alt_Exp_Zcm);//高度环控制
		
		User_LocCtrlOutXcm=0;//位置环控制量置0
		User_LocCtrlOutYcm=0;//位置环控制量置0
		User_RaoganSpedXcm=0;//绕杆控制量置0
		User_RaoganSpedYcm=0;//绕杆控制量置0
		User_RaoganAngDeg=0;//绕杆控制量置0
		VisualAlignmentOutXcm=0;//视觉对齐置0
		VisualAlignmentOutYcm=0;//视觉对齐置0
		SpdOutXcmFor2019=0;//2019巡线控制量置0
		SpdOutYcmFor2019=0;//
		SpdOutZcmFor2019=0;//
		Fb_LocCtrlOutXcm=0;//现场编程控制量置0
		Fb_LocCtrlOutYcm=0;
		Fb_AltCtrlOutZcm=0;

		
		
		
	}
	
///////////////////////////////////////////
		if(AvoidingEnable==0)
		{
			AvoidingXcmps=0;
			AvoidingYcmps=0;
		}
////////////////////////////////////////////////////
		//多环叠加
		OutXcm=User_LocCtrlOutXcm+User_RaoganSpedXcm+VisualAlignmentOutXcm+SpdOutXcmFor2019+AvoidingXcmps+Fb_LocCtrlOutXcm+Lf_LocCtrlOutXcm;//X输出
		OutYcm=User_LocCtrlOutYcm+User_RaoganSpedYcm+VisualAlignmentOutYcm+SpdOutYcmFor2019+AvoidingYcmps+Fb_LocCtrlOutYcm+Lf_LocCtrlOutYcm;//Y输出
		OutDeg=User_AngCtrlOutDeg+User_RaoganAngDeg+Lf_AngCtrlOutDeg;//YAW输出
		OutZcm=User_AltCtrlOutZcm+SpdOutZcmFor2019+Fb_AltCtrlOutZcm;//Z输出
		
		OutXcm=LIMIT(OutXcm,-60,60);
		OutYcm=LIMIT(OutYcm,-60,60);
		OutDeg=LIMIT(OutDeg,-60,60);
		OutZcm=LIMIT(OutZcm,-60,60);
		
		Program_Ctrl_User_Set_Speed(OutXcm,OutYcm,OutZcm,OutDeg);
	//////////////////////////////////////////////////////////////////////
}

static void Loop_100Hz(void) //10ms执行一次
{
	//////////////////////////////////////////////////////////////////////

		//debug
//	goods[0]=16;
//	goods[1]=0xa1;
//	goods[9]=0xd1;
//	goods[16]=0xc4;
//	goods[17]=0xc2;
//	goods[24]=0xb2;
//	goods[25]=0xB2;
	//////////////////////////////////////////////////////////////////////
}

static void Loop_50Hz(void) //20ms执行一次
{
	//////////////////////////////////////////////////////////////////////
	UserTask_OneKeyCmd();//飞行任务函数
	//////////////////////////////////////////////////////////////////////
}

static void Loop_20Hz(void) //50ms执行一次
{
	//////////////////////////////////////////////////////////////////////

    if(preparetopoint==1&&preparetopoint_time==0)
    {
        preparetopoint_time++;
    }
    
    if(preparetopoint_time>0)
        preparetopoint_time++;
    
    if(preparetopoint_time==25)
    {
        FcDataToT265();
        preparetopoint_time=0;
        preparetopoint=0;
    }
		
		
		
		if(rc_in.rc_ch.st_data.ch_[ch_7_aux3]>1500)
		{
			goods[27]=0;//货物
			goods[28]=0;//位置
		}
		if(taskmode==1&&goods[27]==0&&openmv.ifget==1)
		{
			goods[27]=openmv.element;//货物
			goods[28]=goods[openmv.element];//位置
			
		}
		
	//////////////////////////////////////////////////////////////////////
}

static void Loop_2Hz(void) //500ms执行一次
{
	
//	if(1)
//	{
//		goods[1]=0xa1;
//		goods[2]=0xa2;
//		goods[3]=0xa3;
//		goods[4]=0xa4;
//		goods[5]=0xa5;
//		goods[6]=0xa6;
//		
//	}
    

    if(rc_in.rc_ch.st_data.ch_[ch_6_aux2]<1250)
    {
			goods[26]=0;//清0标志位
		
    }else
    if(rc_in.rc_ch.st_data.ch_[ch_6_aux2]>1250&&rc_in.rc_ch.st_data.ch_[ch_6_aux2]<1750)
    {

			goods[26]=1;
			for(int i=0;i<26;i++)
			{
				goods[i]=0;
			}
			for(int i=0;i<24;i++)
			{
				goods_pos[i]=0;
			}
			
    }else if(rc_in.rc_ch.st_data.ch_[ch_6_aux2]>1750)
		{
				goods[1]=0xa3;
				goods[2]=0xc1;
				goods[3]=0xc3;
				goods[4]=0xd3;
				goods[5]=0xb6;
				goods[6]=0xb1;
				goods[7]=0xc5;
				goods[8]=0xd4;
				goods[9]=0xc2;
				goods[10]=0xb4;
				goods[11]=0xb2;
				goods[12]=0xa6;
				goods[13]=0xd2;
				goods[14]=0xa2;
				goods[15]=0xa5;
				goods[16]=0xc4;
				goods[17]=0xb5;
				goods[18]=0xa4;
				goods[19]=0xb3;
				goods[20]=0xd1;
				goods[21]=0xa1;
				goods[22]=0xd6;
				goods[23]=0xd5;
				goods[24]=0xc6;
		}
		
		
		
}
//////////////////////////////////////////////////////////////////////
//调度器初始化
//////////////////////////////////////////////////////////////////////
//系统任务配置，创建不同执行频率的“线程”
static sched_task_t sched_tasks[] =
	{
		{Loop_1000Hz, 1000, 0, 0},
		{Loop_500Hz, 500, 0, 0},
		{Loop_200Hz, 200, 0, 0},
		{Loop_100Hz, 100, 0, 0},
		{Loop_50Hz, 50, 0, 0},
		{Loop_20Hz, 20, 0, 0},
		{Loop_2Hz, 2, 0, 0},
};
//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks) / sizeof(sched_task_t))

void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//初始化任务表
	for (index = 0; index < TASK_NUM; index++)
	{
		//计算每个任务的延时周期数
		sched_tasks[index].interval_ticks = TICK_PER_SECOND / sched_tasks[index].rate_hz;
		//最短周期为1，也就是1ms
		if (sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}
//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//循环判断所有线程，是否应该执行

	for (index = 0; index < TASK_NUM; index++)
	{
		//获取系统当前时间，单位MS
		uint32_t tnow = GetSysRunTimeMs();
		//进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
		if (tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{

			//更新线程的执行时间，用于下一次判断
			sched_tasks[index].last_run = tnow;
			//执行线程函数，使用的是函数指针
			sched_tasks[index].task_func();
		}
	}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
