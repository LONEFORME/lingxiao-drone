/*==========================================================================
 * 描述    ：用户程控函数
 * 更新时间：2023年3月26日
 * 作者		 ：	LHB
===========================================================================*/

#include "User_Ctrl.h"

#include "Drv_Uart.h"
#include "uart.h"
#include "hw_ints.h"
#include "hw_gpio.h"
#include "hw_types.h"
#include "Ano_DT_LX.h"
#include "Drv_UbloxGPS.h"
#include "Drv_AnoOf.h"
#include "Drv_AnoOf.h"
#include "User_T265.h"
#include "Ano_Math.h"
#include "User_Task.h"
#include "LX_FC_Fun.h"
#include "LX_FC_State.h"
#include "User_Opmv.h"

s16 CMDvalue[4]={0}; //实时控制帧x y z deg发送数据  
////////////////////////////////
s16 User_Yaw=0;//处理后的YAW角数据 范围负无穷到正无穷
s16 User_Yaw_OF=0;//YAW角溢出次数
s16 User_Yaw_Zero=0;//yaw角参考零点
s16 User_Yaw_attnum=0;//yaw角确实到达范围内次数，多次判断
///////////////////////////////
s16 RaoGanExpRcm=0;//绕杆半径
s16 RaoGanExpYcm=0;//绕杆切向速度
s16 RaoGanExpYawDeg=0;//绕杆期望角度
s16 RaoGanMode=0;//绕杆模式
s16 RaoGanNum=0;//次数
/////////////////////////////////////

s16 VisualAlignmentMode=0;   //视觉对齐模式
s16 VisualAlignmentElement=0;//视觉对齐元素
s16 VisualAlignmentOutXcm=0; //位置环控制输出
s16 VisualAlignmentOutYcm=0; //位置环控制输出

s16 VisualAlignmentExpX=0; //视觉对齐期望值
s16 VisualAlignmentExpY=0; //视觉对齐期望值
//////////////////////
//标志位
u8 ProgramAutoLand =0;//程控自动降落

u8 LocCtrlEnable =0;
u8 AngCtrlEnable =0;
u8 AttCtrlEnable =0;
/*控制使能标志位0不控制 1 控制*/
////////////////////////////////

//期望位置设定，在这里赋值
s16 Loc_Exp_Xcm =0;
s16 Loc_Exp_Ycm =0;
s16 Alt_Exp_Zcm =0;
s16 Ang_Exp_Deg =0;

///////////////////////////////

s16 User_LocCtrlOutXcm=0;//位置环控制输出
s16 User_LocCtrlOutYcm=0;//位置环控制输出
s16 User_AngCtrlOutDeg=0;//角度环控制输出
s16 User_AltCtrlOutZcm=0;//高度环控制输出

///////////////////////////////////////////////
/*现场编程角度起飞降落控制量输出*/
s16 Fb_LocCtrlOutXcm=0;//位置环控制输出
s16 Fb_LocCtrlOutYcm=0;//位置环控制输出
s16 Fb_AngCtrlOutDeg=0;//角度环控制输出（螺旋起飞会用到？）
s16 Fb_AltCtrlOutZcm=0;//高度环控制输出

u8 FbCtrlEnable =0;    //开启控制
u8 FbAutoLand =0 ;
//期望位置设定，在这里赋值
s16 Fb_Loc_Exp_Xcm =0;
s16 Fb_Loc_Exp_Ycm =0;
s16 Fb_Alt_Exp_Zcm =0;
s16 Fb_Ang_Exp_Deg =0;
s16 Fb_StartPos_Xcm = 0;
s16 Fb_StartPos_Ycm = 0;

////////////////////////////////////////
s16 Lf_LocCtrlOutXcm=0;//向前速度
s16 Lf_LocCtrlOutYcm=0;//速度
s16 Lf_AngCtrlOutDeg=0;//角度环控制输出
s16 Lf_Ang_Exp_Deg =0;
s16 Lf_Spd_Exp_Xcm =0;
u8 	Lf_Load_Mode =0;//巡线路口情况
u8  Lf_Ctrl_Enable=0;//使能控制


////////////////////////////////////////
/*最终控制量输出*/
s16 OutXcm=0;//X输出
s16 OutYcm=0;//Y输出
s16 OutDeg=0;//YAW输出
s16 OutZcm=0;//Z输出

///////////////////////////////////////////
s16 User_RaoganSpedXcm=0;//绕杆
s16 User_RaoganSpedYcm=0;
s16 User_RaoganAngDeg=0;
//////////////////////////////////////////
/*2019年 XYZ控制量输出*/

s16 SpdOutXcmFor2019 = 0;
s16 SpdOutYcmFor2019 = 0;
s16 SpdOutZcmFor2019 = 0;

u8 LineCtrlEnable = 0 ;
////////////////////////////////////////////
u8  AvoidingEnable = 0; //避障使能
s16 AvoidingXcmps = 0;
s16 AvoidingYcmps = 0;
//////////////////////////////////////
/**********************************************************************************************************
*函 数 名: Program_Ctrl_User_Set_Speed()
*功能说明: 程控功能，航向水平坐标系下速度设定（实时控制）
*参    数: X速度（厘米每秒，正为前进，负为后退，Y速度（厘米每秒，正为左移，负为右移）
*返 回 值: 无
*作    者: LHB
**********************************************************************************************************/
void Program_Ctrl_User_Set_Speed(s16 xpcm, s16 ypcm, s16 zpcm, s16 degpres)
{
		CMDvalue[0] = degpres;  //航向转动角速度，度每秒，逆时针为正
		CMDvalue[1] = xpcm;    //头向速度，厘米每秒
		CMDvalue[2] = ypcm;    //左向速度，厘米每秒
		CMDvalue[3] = zpcm;	 //天向速度，厘米每秒
}

/**********************************************************************************************************
*函 数 名: User_Loc_Ctrl
*功能说明:	单独控制XY方向上的速度，输出保存到全局变量
*参    数: 实际位置 期望位置 
*返 回 值: 无
*作    者: LHB
**********************************************************************************************************/
void User_Loc_Ctrl(s16 true_x_cm,s16 true_y_cm,s16 exp_x_cm,s16 exp_y_cm)
{
	static float err_old_x_cm=0,err_old_y_cm=0;
	static float err_new_x_cm=0,err_new_y_cm=0;
	static float err_add_x_cm=0,err_add_y_cm=0;
	static s16 spd_out_x_cm=0,spd_out_y_cm=0;
	
	///////////////////////////////////////////////////////////////////////////////
	/*PID计算*/
	err_new_x_cm=exp_x_cm-true_x_cm;
	err_new_y_cm=exp_y_cm-true_y_cm;

	
	err_add_x_cm+=err_new_x_cm;
	err_add_y_cm+=err_new_y_cm;

	spd_out_x_cm=(s16)(loc_ctrl_dis_p*err_new_x_cm+LIMIT(loc_ctrl_dis_i*err_add_x_cm,-10,10)+loc_ctrl_dis_d*(err_new_x_cm-err_old_x_cm));
	spd_out_y_cm=(s16)(loc_ctrl_dis_p*err_new_y_cm+LIMIT(loc_ctrl_dis_i*err_add_y_cm,-10,10)+loc_ctrl_dis_d*(err_new_y_cm-err_old_y_cm));

	err_old_x_cm=err_new_x_cm;
	err_old_y_cm=err_new_y_cm;

	//////////////////////////////////////////
	/*限速*/
	spd_out_x_cm=LIMIT(spd_out_x_cm,-LOCMAXSPD,LOCMAXSPD);
	spd_out_y_cm=LIMIT(spd_out_y_cm,-LOCMAXSPD,LOCMAXSPD);
	///////////////////////////////////////////////////////////////////////////////

	
	////////////////////////////////////////////////////////////////////////////////////
	if(LocCtrlEnable==0)
	{
		spd_out_x_cm=0;
		spd_out_y_cm=0;
	}

	//////////////////////////////////////////////////////////////////
	User_LocCtrlOutXcm=spd_out_x_cm;
	User_LocCtrlOutYcm=spd_out_y_cm;
	
//////////////////////////////////////////////////////////////////////

}


/**********************************************************************************************************
*函 数 名: User_Alt_Ctrl(s16 true_z_cm,s16 exp_z_cm)
*功能说明:高度环+自动降落
*参    数: 实际高度 期望高度
*返 回 值: 无
*作    者: LHB
**********************************************************************************************************/
void User_Alt_Ctrl(s16 true_z_cm,s16 exp_z_cm)
{
	static float err_old_z_cm=0;
	static float err_new_z_cm=0;
	static float err_add_z_cm=0;
	static s16 spd_out_z_cm=0;
	
	static s16 LandTime =0;
	///////////////////////////////////////////////////////////////////////////////
	/*PID计算*/
	err_new_z_cm=exp_z_cm-true_z_cm;
	
	err_add_z_cm+=err_new_z_cm;
	
	spd_out_z_cm=(s16)(loc_ctrl_att_p*err_new_z_cm\
	+LIMIT(loc_ctrl_att_i*err_add_z_cm,-10,10)\
	+loc_ctrl_att_d*(err_new_z_cm-err_old_z_cm));
	
	err_old_z_cm=err_new_z_cm;
	//////////////////////////////////////////
	/*限速*/
	spd_out_z_cm=LIMIT(spd_out_z_cm,-35,35);
	///////////////////////////////////////////////////////////////////////////////

	
	////////////////////////////////////////////////////////////////////////////////////
	
	if(AttCtrlEnable==0)
	{
		spd_out_z_cm=0;
	}
	//////////////////////////////////////////////////////////////////
	if(ProgramAutoLand==1)//执行自动降落
	{
		LandTime+=5;
		
		if(LandTime>=15*1000||true_z_cm<=6)//超时或者高度达到就上锁
		{
			spd_out_z_cm=0;
			ProgramAutoLand=0;//自动降落标志位复位
			LandTime=0;
			FC_Lock();
		}
		
		if(true_z_cm>=100)
			spd_out_z_cm=-30;
		else if(true_z_cm>=30)
			spd_out_z_cm=-20;
		else 
			spd_out_z_cm=-8;
	}
//////////////////////////////////////////////////////////////////////
	User_AltCtrlOutZcm=spd_out_z_cm;//高度环控制输出
}
/**********************************************************************************************************
*函 数 名: void User_Ang_Ctrl(s16 true_yaw_deg,s16 exp_yaw_deg)
*功能说明:
*参    数: 
*返 回 值: 无
*作    者: LHB
**********************************************************************************************************/
void User_Ang_Ctrl(s16 true_yaw_deg,s16 exp_yaw_deg)
{
	static float err_old_yaw_deg=0;
	static float err_new_yaw_deg=0;
	static float err_add_yaw_deg=0;
	static s16 spd_out_yaw_deg=0;
	///////////////////////////////////////////////////////////////////////////////
	/*PID计算*/
	err_new_yaw_deg=exp_yaw_deg-true_yaw_deg;	
	err_add_yaw_deg+=err_new_yaw_deg;
	spd_out_yaw_deg=(s16)(loc_ctrl_ang_p*err_new_yaw_deg\
	+LIMIT(loc_ctrl_ang_i*err_add_yaw_deg,-10,10)\
	+loc_ctrl_ang_d*(err_new_yaw_deg-err_old_yaw_deg));
	err_old_yaw_deg=err_new_yaw_deg;
	//////////////////////////////////////////
	/*限速*/
	spd_out_yaw_deg=LIMIT(spd_out_yaw_deg,-35,35);
	///////////////////////////////////////////////////////////////////////////////
	if(AngCtrlEnable==0)
		spd_out_yaw_deg=0;
	//////////////////////////////////////////////////////////////////
	User_AngCtrlOutDeg=-spd_out_yaw_deg;//控制量输出
}




void User_Yaw_Calu(void)//YAW转换到连续的区间
{
//	static s16 temp_yaw_old=0,temp_yaw_new=0,User_Yaw_new=0;
//	temp_yaw_new=(s16)fc_att.st_data.yaw_x100/100;
//	if(temp_yaw_old<=-110&&temp_yaw_new>=110)//发生一次跳变，转换连续区间上
//		User_Yaw_OF--;
//	if(temp_yaw_old>=110&&temp_yaw_new<=-110)//发生一次跳变，转换连续区间上
//		User_Yaw_OF++;
//	
//	User_Yaw_new=temp_yaw_new+User_Yaw_OF*360;
//	
//	temp_yaw_old=temp_yaw_new;
//	User_Yaw=0.1*User_Yaw_new+0.9*User_Yaw;//低通滤波
	
	static s16 temp_yaw_old=0,temp_yaw_new=0,User_Yaw_new=0;
	temp_yaw_new=(s16)t265.ya_f;
	if(temp_yaw_old<=-110&&temp_yaw_new>=110)//发生一次跳变，转换连续区间上
		User_Yaw_OF--;
	if(temp_yaw_old>=110&&temp_yaw_new<=-110)//发生一次跳变，转换连续区间上
		User_Yaw_OF++;
	
	User_Yaw_new=temp_yaw_new+User_Yaw_OF*360;
	
	temp_yaw_old=temp_yaw_new;
	User_Yaw=0.1*User_Yaw_new+0.9*User_Yaw;//低通滤波
	
	
	
}


/**********************************************************************************************************
*函 数 名: void User_RaoGan(s16 true_r_cm,s16 true_yaw_deg ,s16 exp_r_cm,s16 exp_y_cm,s16 exp_yaw_deg,u8 mode)
*功能说明:	 绕杆飞行控制
*参    数: 实际距杆距离 实际偏差角度 预期与杆距离 预期绕杆速度 期望与杆角度 绕杆模式0不绕 1自转 2调整角度 3调整距离 4开始绕杆
*返 回 值: 无
*作    者: LHB
**********************************************************************************************************/
void User_RaoGan(s16 true_r_cm,s16 true_yaw_deg ,s16 exp_r_cm,s16 exp_y_cm,s16 exp_yaw_deg,u8 mode)
{
	static float err_old_r_cm=0,err_old_yaw_deg=0;
	static float err_new_r_cm=0,err_new_yaw_deg=0;
	static float err_add_r_cm=0,err_add_yaw_deg=0;
	static s16   spd_out_r_cm=0,spd_out_y_cm=0,spd_out_yaw_deg=0;

	///////////////////////////////////////////////////////////////////////////////
	/*PID计算*/
	err_new_r_cm=exp_r_cm-true_r_cm;
	err_new_yaw_deg=exp_yaw_deg-true_yaw_deg;//期望角度0
	
	err_add_r_cm+=err_new_r_cm;
	err_add_yaw_deg+=err_new_yaw_deg;
	
	spd_out_r_cm=			(s16)(raogan_ctrl_r_p*err_new_r_cm\
	+LIMIT(raogan_ctrl_r_i*err_add_r_cm,-10,10)\
	+raogan_ctrl_r_d*(err_new_r_cm-err_old_r_cm));
	
	spd_out_yaw_deg=  (s16)(raogan_ctrl_ang_p*err_new_yaw_deg\
	+LIMIT(raogan_ctrl_ang_i*err_add_yaw_deg,-10,10)\
	+raogan_ctrl_ang_d*(err_new_yaw_deg-err_old_yaw_deg));
	
	err_old_r_cm=err_new_r_cm;
	err_old_yaw_deg=err_new_yaw_deg;
	//////////////////////////////////////////

	//////////////////////////////////////////
	/*限速*/
	spd_out_r_cm=LIMIT(spd_out_r_cm,-50,50);
	spd_out_y_cm=LIMIT(spd_out_y_cm,-50,50);
	spd_out_yaw_deg=LIMIT(spd_out_yaw_deg,-35,35);
	///////////////////////////////////////////////////////////////////////////////
	/*控制量输出*/
	if(mode==0)//不调整
	{
		spd_out_r_cm=0;
		spd_out_y_cm=0;
		spd_out_yaw_deg=0;
	}else if(mode==1)//自转
	{
		spd_out_r_cm=0;
		spd_out_y_cm=0;
		spd_out_yaw_deg=exp_yaw_deg;
	}else if(mode==2)//只调整角度
	{
		spd_out_r_cm=0;
		spd_out_y_cm=0;
		spd_out_yaw_deg=spd_out_yaw_deg;
		
	}else if(mode==3)//调整距离和角度 但不绕杆
	{
		spd_out_y_cm=0;
		spd_out_yaw_deg=spd_out_yaw_deg;
		spd_out_r_cm=spd_out_r_cm;//摄像头朝后，调整单位
	}else if(mode==4)//开始绕杆
	{
		spd_out_y_cm=exp_y_cm;
		spd_out_yaw_deg=spd_out_yaw_deg;
		spd_out_r_cm=spd_out_r_cm;//摄像头朝后，调整单位
	}
	User_RaoganAngDeg=-spd_out_yaw_deg;
	User_RaoganSpedYcm=-spd_out_r_cm;
	User_RaoganSpedXcm=spd_out_y_cm;
}

/**********************************************************************************************************
*函 数 名: UserVisualAlignment(s16 true_x_cm,s16 true_y_cm ,s16 exp_x_cm,s16 exp_y_cm,s16 mode)
*功能说明:	 视觉对齐或激光雷达对齐 前馈加PD控制
*参    数:
*返 回 值: 无
*作    者: LHB
**********************************************************************************************************/

void UserVisualAlignment(s16 true_x_cm,s16 true_y_cm ,s16 exp_x_cm,s16 exp_y_cm,s16 mode)
{
	static float err_old_x_cm=0,err_old_y_cm=0;
	static float err_new_x_cm=0,err_new_y_cm=0;
	static float err_add_x_cm=0,err_add_y_cm=0;
	static s16 spd_out_x_cm=0,spd_out_y_cm=0;
	
	static s16 target_spd_x_cmps = 0,target_spd_y_cmps = 0 ;
	
	
	///////////////////////////////////////////////////////////////////////////////
	/*PID计算*/
	err_new_x_cm=exp_x_cm-true_x_cm;
	err_new_y_cm=exp_y_cm-true_y_cm;
	
/*1m标定*/
	openmv.target_spd_x_cmps=(200.0f)*(err_new_x_cm-err_old_x_cm);//t265.v_x_f+
	openmv.target_spd_y_cmps=(200.0f)*(err_new_y_cm-err_old_y_cm);//t265.v_x_f+
	
	/*求出目标速度 飞机速度加微分速度 未完成*/
	target_spd_x_cmps=t265.v_x_f;
	target_spd_y_cmps=t265.v_y_f;
	
	err_add_x_cm+=err_new_x_cm;
	err_add_y_cm+=err_new_y_cm;

	spd_out_x_cm=(s16)( VisualAlignment_KF*target_spd_x_cmps + VisualAlignment_P*err_new_x_cm+LIMIT(VisualAlignment_I*err_add_x_cm,-10,10)+VisualAlignment_D*(err_new_x_cm-err_old_x_cm));
	spd_out_y_cm=(s16)( VisualAlignment_KF*target_spd_y_cmps + VisualAlignment_P*err_new_y_cm+LIMIT(VisualAlignment_I*err_add_y_cm,-10,10)+VisualAlignment_D*(err_new_y_cm-err_old_y_cm));

	err_old_x_cm=err_new_x_cm;
	err_old_y_cm=err_new_y_cm;

	//////////////////////////////////////////
	/*限速*/
	spd_out_x_cm=LIMIT(spd_out_x_cm,-30,30);
	spd_out_y_cm=LIMIT(spd_out_y_cm,-30,30);
	///////////////////////////////////////////////////////////////////////////////

	if(mode==0)
	{
	spd_out_x_cm=0;
	spd_out_y_cm=0;
	}

	//////////////////////////////////////////////////////////////////

	VisualAlignmentOutXcm=-spd_out_x_cm;
	VisualAlignmentOutYcm=spd_out_y_cm;
}

//void UserVisualAlignmentCalu(s16 true_x_cm,s16 true_y_cm)
//{
//	static float err_old_x_cm=0,err_old_y_cm=0;
//	static float err_new_x_cm=0,err_new_y_cm=0;
//	static float err_add_x_cm=0,err_add_y_cm=0;
//	static s16 spd_out_x_cm=0,spd_out_y_cm=0;
//	
//	static s16 target_spd_x_cmps = 0,target_spd_y_cmps = 0 ;
//	/*1m标定*/
//	openmv.target_spd_x_cmps=(2000.0f)*(err_new_x_cm-err_old_x_cm);//t265.v_x_f+
//	openmv.target_spd_y_cmps=(2000.0f)*(err_new_y_cm-err_old_y_cm);//t265.v_x_f+
//	
//	target_spd_x_cmps=openmv.target_spd_x_cmps+t265.v_x_f;
//	target_spd_y_cmps=openmv.target_spd_y_cmps+t265.v_y_f;
//	
//	///////////////////////////////////////////////////////////////////////////////
//	/*PID计算*/
//	err_new_x_cm=true_x_cm;
//	err_new_y_cm=true_y_cm;

//	
//	
//	err_old_x_cm=err_new_x_cm;
//	err_old_y_cm=err_new_y_cm;


//}


void UserVisualZCtrl(s16 true_z_cm,s16 exp_z_cm,s16 mode)// for 2019
{
	static float err_old_z_cm=0;
	static float err_new_z_cm=0;
	static float err_add_z_cm=0;
	static s16 spd_out_z_cm=0;
	///////////////////////////////////////////////////////////////////////////////
	/*PID计算*/
	err_new_z_cm=exp_z_cm-true_z_cm;
	
	err_add_z_cm+=err_new_z_cm;
	
	spd_out_z_cm=(s16)(loc_ctrl_att_p*err_new_z_cm+LIMIT(loc_ctrl_att_i*err_add_z_cm,-10,10)+loc_ctrl_att_d*(err_new_z_cm-err_old_z_cm));
	
	err_old_z_cm=err_new_z_cm;
	//////////////////////////////////////////
	/*限速*/
	spd_out_z_cm=LIMIT(spd_out_z_cm,-10,10);
	///////////////////////////////////////////////////////////////////////////////	
	if(AttCtrlEnable==0)
	{
		spd_out_z_cm=0;
	}
	SpdOutZcmFor2019=spd_out_z_cm;//高度环控制输出
}
void UserVisualYCtrl(s16 true_y_cm,s16 exp_y_cm,s16 mode)// for 2019
{
	static float err_old_y_cm=0;
	static float err_new_y_cm=0;
	static float err_add_y_cm=0;
	static s16 spd_out_y_cm=0;
	
	///////////////////////////////////////////////////////////////////////////////
	/*PID计算*/
	err_new_y_cm=exp_y_cm-true_y_cm;

	
	err_add_y_cm+=err_new_y_cm;

	spd_out_y_cm=(s16)(VisualAlignment_P_2019*err_new_y_cm+LIMIT(VisualAlignment_I_2019*err_add_y_cm,-10,10)+VisualAlignment_D_2019*(err_new_y_cm-err_old_y_cm));

	err_old_y_cm=err_new_y_cm;

	//////////////////////////////////////////
	/*限速*/
	spd_out_y_cm=LIMIT(spd_out_y_cm,-15,15);
	///////////////////////////////////////////////////////////////////////////////

	if(mode==0)
	{
	spd_out_y_cm=0;
	}

	//////////////////////////////////////////////////////////////////

	SpdOutYcmFor2019=-spd_out_y_cm;
}
/*检测是否进入局部最优解陷阱*/
void LosTrapCheck(u8 t)
{
	static int time = 0;//卡死时间
	if(AvoidingEnable==1)
	{
		if(ABS(t265.v_x_f)<4&&ABS(t265.v_y_f)<4)
		{
			time+=t;
			if(time==2000)//2000毫秒
			{
				time=0;
				/*告知JETSON*/
//				openmv.c_mode=1;
				
			}
		}else
		{
			time=0;
		}
			
	}
}


/**********************************************************************************************************
*函 数 名: FbOneKeyTakeOff(s16 true_x_cm,s16 true_y_cm,s16 true_z_cm,
						s16 exp_x_cm,s16 exp_y_cm,s16 exp_z_cm,u8 CtrlEnable)
*功能说明: 倾斜起飞控制
*参    数: 与Y轴夹角 与X轴夹角 目标起飞高度 控制与否
*返 回 值: 无
*作    者: LHB
**********************************************************************************************************/


void FbOneKeyTakeOff(s16 true_x_cm,s16 true_y_cm,s16 true_z_cm,\
s16 exp_x_cm,s16 exp_y_cm,s16 exp_z_cm,u8 CtrlEnable)
{
	static float err_old_x_cm=0,err_old_y_cm=0,err_old_z_cm=0;
	static float err_new_x_cm=0,err_new_y_cm=0,err_new_z_cm=0;
	static float err_add_x_cm=0,err_add_y_cm=0,err_add_z_cm=0;
	static s16 spd_out_x_cm=0,spd_out_y_cm=0,spd_out_z_cm=0;
	static float takeoff_k = 0;//当前高度与目标高度比值
	static s16 LandTime =0;
	/*主动控制Z轴速度，X,Y轴为从动，以达到倾斜起飞目的*/
	/////////////////////////////////////////////////////////
	if(FbAutoLand==0)
{
	takeoff_k=(float)((float)true_z_cm/(float)exp_z_cm);//计算比例
    if(ABS(true_z_cm-exp_z_cm)<10)
    {
      takeoff_k=1;
    }
}
	else if(FbAutoLand==1)
{
    takeoff_k=(float)((float)(100-true_z_cm)/90);//假定从一米高度降落
    if(true_z_cm<10)
    takeoff_k=1;
    takeoff_k=LIMIT(takeoff_k,0.1f,1);
}
	///////////////////////////////////////////////////////////////////////////////
	/*PID计算*/
	err_new_z_cm=exp_z_cm-true_z_cm;
	
	err_add_z_cm+=err_new_z_cm;
	
	spd_out_z_cm=(s16)(loc_ctrl_att_p*err_new_z_cm\
	+LIMIT(loc_ctrl_att_i*err_add_z_cm,-10,10)\
	+loc_ctrl_att_d*(err_new_z_cm-err_old_z_cm));
	
	err_old_z_cm=err_new_z_cm;
	//////////////////////////////////////////
	/*限速*/
	spd_out_z_cm=LIMIT(spd_out_z_cm,-20,20);
	
	////////////////////////////////////////////////
	
	if(FbAutoLand==1)//执行自动降落
	{
		LandTime+=5;
		
		if(LandTime>=15*1000||true_z_cm<=8)//超时或者高度达到就上锁
		{
			spd_out_z_cm=0;
			FbAutoLand=0;//自动降落标志位复位
			LandTime=0;
			FC_Lock();
		}
		
		if(true_z_cm>=100)
			spd_out_z_cm=-10;
		else if(true_z_cm>=30)
			spd_out_z_cm=-10;
		else 
			spd_out_z_cm=-8;
	}
	
	///////////////////////////////////////////////////////////////////////////////
	/*PID计算*/

	err_new_x_cm=Fb_StartPos_Xcm+takeoff_k*(exp_x_cm-Fb_StartPos_Xcm)-true_x_cm;//设置期望位置
	err_new_y_cm=Fb_StartPos_Ycm+takeoff_k*(exp_y_cm-Fb_StartPos_Ycm)-true_y_cm;

	
	err_add_x_cm+=err_new_x_cm;
	err_add_y_cm+=err_new_y_cm;

	spd_out_x_cm=(s16)(loc_ctrl_dis_p*err_new_x_cm\
	+LIMIT(loc_ctrl_dis_i*err_add_x_cm,-10,10)\
	+loc_ctrl_dis_d*(err_new_x_cm-err_old_x_cm));
	
	spd_out_y_cm=(s16)(loc_ctrl_dis_p*err_new_y_cm\
	+LIMIT(loc_ctrl_dis_i*err_add_y_cm,-10,10)\
	+loc_ctrl_dis_d*(err_new_y_cm-err_old_y_cm));

	err_old_x_cm=err_new_x_cm;
	err_old_y_cm=err_new_y_cm;

	//////////////////////////////////////////
	/*限速*/
	spd_out_x_cm=LIMIT(spd_out_x_cm,-40,40);
	spd_out_y_cm=LIMIT(spd_out_y_cm,-40,40);
	///////////////////////////////////////////////////////////////////////////////

	
	////////////////////////////////////////////////////////////////////////////////////
	if(CtrlEnable==0)
	{
		spd_out_x_cm=0;
		spd_out_y_cm=0;
		spd_out_z_cm=0;
	}
	
	//////////////////////////////////////////////////////////////////
	Fb_LocCtrlOutXcm=spd_out_x_cm;
	Fb_LocCtrlOutYcm=spd_out_y_cm;
	Fb_AltCtrlOutZcm=spd_out_z_cm;
	
}





void LineFollowingCtrl(s16 true_yaw_deg,s16 exp_yaw_deg ,s16 true_y_cm,s16 exp_y_cm,s16 exp_vx_cmps,u8 load_mode,u8 CtrlEnable)
{
	
	static float err_old_y_cm=0;
	static float err_new_y_cm=0;
	static float err_add_y_cm=0;
	static s16 	spd_out_y_cm=0;

	static float err_old_yaw_deg=0;
	static float err_new_yaw_deg=0;
	static float err_add_yaw_deg=0;
	static s16 spd_out_yaw_deg=0, spd_out_x_cmps=0;
//	static s16 dt=0;/*积分时间*/
//	static u8 turn_mode=0;
	spd_out_x_cmps=exp_vx_cmps;
//	dt=dt;
//	if(load_mode!=0&&turn_mode==0)
//	{
//		turn_mode=load_mode;
//	}
//	if(turn_mode!=0)
//	{
//		dt+=5;
//		if(turn_mode==1)//左转90
//		{
//			if(dt>=LF_TURN_90)
//			{
//				turn_mode=3;
//				dt=0;
//			}
//		}else if(turn_mode==2)//y右转90
//		{
//			if(dt>=LF_TURN90)
//			{
//				turn_mode=3;
//				dt=0;
//			}
//		}else if(turn_mode==3)
//		{
//			if(dt>=LF_GO)
//			{
//				turn_mode=0;
//				dt=0;
//			}
//		}
//		
//	}
	
	
	///////////////////////////////////////////////////////////////////////////////
	/*PID计算*/
	err_new_y_cm=exp_y_cm-true_y_cm;
	err_add_y_cm+=err_new_y_cm;
	spd_out_y_cm=(s16)(lf_ctrl_dis_p*err_new_y_cm\
	+LIMIT(lf_ctrl_dis_i*err_add_y_cm,-10,10)\
	+lf_ctrl_dis_d*(err_new_y_cm-err_old_y_cm));
	err_old_y_cm=err_new_y_cm;
	
	
	err_new_yaw_deg=exp_yaw_deg-true_yaw_deg;	
	
	if(ABS(err_new_yaw_deg)>60)//转角减速40%
	{
		spd_out_x_cmps=0.40f*spd_out_x_cmps;
	}else
    if(ABS(err_new_yaw_deg)>20)//转角减速40%
	{
		spd_out_x_cmps=0.60f*spd_out_x_cmps;
	}
	
	
	err_add_yaw_deg+=err_new_yaw_deg;
	spd_out_yaw_deg=(s16)(lf_ctrl_ang_p*err_new_yaw_deg\
	+LIMIT(lf_ctrl_ang_i*err_add_yaw_deg,-10,10)\
	+lf_ctrl_ang_d*(err_new_yaw_deg-err_old_yaw_deg));
	err_old_yaw_deg=err_new_yaw_deg;
	//////////////////////////////////////////
	/*限速*/
	spd_out_yaw_deg=LIMIT(spd_out_yaw_deg,-45,45);
		/*限速*/
	spd_out_y_cm=LIMIT(spd_out_y_cm,-40,40);
	///////////////////////////////////////////////////////////////////////////////
	if(CtrlEnable==0)
	{
		spd_out_yaw_deg=0;
		spd_out_x_cmps=0;
		spd_out_y_cm=0;
	}
	
//	if(turn_mode==1)
//	{
//		spd_out_yaw_deg=-30;
//		spd_out_x_cmps=0;
//		spd_out_y_cm=25;
//	}else if(turn_mode==2)
//	{
//		spd_out_yaw_deg=30;
//		spd_out_x_cmps=0;
//		spd_out_y_cm=-25;
//	}else if(turn_mode==3)
//	{
//		spd_out_yaw_deg=0;
//		spd_out_x_cmps=15;
//		spd_out_y_cm=0;
//	}
	
	//////////////////////////////////////////////////////////////////
	Lf_AngCtrlOutDeg=-spd_out_yaw_deg;//控制量输出
	Lf_LocCtrlOutXcm=spd_out_x_cmps;
	Lf_LocCtrlOutYcm=spd_out_y_cm;
//	Lf_LocCtrlOutYcm=0;
}

