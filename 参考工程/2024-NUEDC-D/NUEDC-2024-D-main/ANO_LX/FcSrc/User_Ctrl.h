/*==========================================================================
 * 描述    ：用户程控函数
 * 更新时间：2023年3月26日
 * 作者		 ：	LHB
===========================================================================*/
#ifndef __USER_CTRL_
#define __USER_CTRL_


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

////////////////////////////////////////
/*位置角度环PID参数定义*/
#define loc_ctrl_dis_p 1.1f
#define loc_ctrl_dis_i 0.0f
#define loc_ctrl_dis_d 0.2f  

#define loc_ctrl_ang_p 1.1f
#define loc_ctrl_ang_i 0.0f
#define loc_ctrl_ang_d 0.2f

#define loc_ctrl_att_p 1.0f
#define loc_ctrl_att_i 0.0f
#define loc_ctrl_att_d 0.1f

//////////////////////////////////////////
/*视觉对齐PID定义*/
/* 植保无人机对齐杆 */
#define VisualAlignment_P  0.05f
#define VisualAlignment_I  0.0f
#define VisualAlignment_D  0.01f  
#define VisualAlignment_KF 0.0f  

////////////////////////////////////////

#define VisualAlignment_P_2019 0.4f
#define VisualAlignment_I_2019 0.0f
#define VisualAlignment_D_2019 0.1f  


////////////////////////////////////////
/*绕杆PID参数定义*/
#define raogan_ctrl_r_p 0.8f 
#define raogan_ctrl_r_i 0.0f
#define raogan_ctrl_r_d 0.01f//半径控制

#define raogan_ctrl_att_p 0.6f 
#define raogan_ctrl_att_i 0.0f
#define raogan_ctrl_att_d 0.0f//高度控制

#define raogan_ctrl_ang_p 1.0f 
#define raogan_ctrl_ang_i 0.0f
#define raogan_ctrl_ang_d 0.0f//角度控制
/////////////////////////////////////////////
/*延迟转弯时间*/
#define LF_TURN90  2500
#define LF_TURN_90 2500
#define LF_TURN180 0
#define LF_GO 1500

#define lf_ctrl_ang_p 0.7f 
#define lf_ctrl_ang_i 0.0f
#define lf_ctrl_ang_d 0.0f//角度控制

#define lf_ctrl_dis_p 0.2f
#define lf_ctrl_dis_i 0.0f
#define lf_ctrl_dis_d 0.0f  

#define LOCMAXSPD 28

////////////////////////////////////////
extern s16 CMDvalue[4];//实时控制帧发送数据

extern u8 LocCtrlEnable;
extern u8 AngCtrlEnable;
extern u8 AttCtrlEnable;

extern u8 ProgramAutoLand;//程控自动降落

extern s16 Loc_Exp_Xcm ;
extern s16 Loc_Exp_Ycm ;
extern s16 Alt_Exp_Zcm ;
extern s16 Ang_Exp_Deg ;

extern s16 User_LocCtrlOutXcm;//位置环控制输出
extern s16 User_LocCtrlOutYcm;//位置环控制输出
extern s16 User_AngCtrlOutDeg;//角度环控制输出
extern s16 User_AltCtrlOutZcm;//高度环控制输出

///////////////////////////////////////////////
/*现场编程角度起飞降落控制量输出*/
extern s16 Fb_LocCtrlOutXcm;//位置环控制输出
extern s16 Fb_LocCtrlOutYcm;//位置环控制输出
extern s16 Fb_AngCtrlOutDeg;//角度环控制输出（螺旋起飞会用到？）
extern s16 Fb_AltCtrlOutZcm;//高度环控制输出

extern u8 FbCtrlEnable;    //开启控制

extern u8 FbAutoLand ;
extern s16 Fb_Loc_Exp_Xcm ;
extern s16 Fb_Loc_Exp_Ycm ;
extern s16 Fb_Alt_Exp_Zcm ;
extern s16 Fb_Ang_Exp_Deg ;

extern s16 Fb_StartPos_Xcm;
extern s16 Fb_StartPos_Ycm;

extern s16 User_RaoganSpedXcm;//绕杆
extern s16 User_RaoganSpedYcm;
extern s16 User_RaoganAngDeg;

extern s16 RaoGanExpRcm;//绕杆半径
extern s16 RaoGanExpYcm;//绕杆切向速度
extern s16 RaoGanExpYawDeg;//绕杆期望角度
extern s16 RaoGanMode;//绕杆模式
extern s16 RaoGanNum;//次数

extern s16 OutXcm;//X输出
extern s16 OutYcm;//Y输出
extern s16 OutDeg;//YAW输出
extern s16 OutZcm;//Z输出

extern s16 VisualAlignmentMode;   //视觉对齐模式
extern s16 VisualAlignmentElement;//视觉对齐元素
extern s16 VisualAlignmentOutXcm; //位置环控制输出
extern s16 VisualAlignmentOutYcm; //位置环控制输出

extern s16 VisualAlignmentExpX; //视觉对齐期望值
extern s16 VisualAlignmentExpY; //视觉对齐期望值

extern s16 SpdOutXcmFor2019 ;
extern s16 SpdOutYcmFor2019 ;
extern s16 SpdOutZcmFor2019 ;

extern u8 LineCtrlEnable ;


extern s16 Lf_LocCtrlOutXcm;//向前速度
extern s16 Lf_LocCtrlOutYcm;//速度
extern s16 Lf_AngCtrlOutDeg;//角度环控制输出
extern s16 Lf_Ang_Exp_Deg ;
extern u8  Lf_Load_Mode ;//巡线路口情况
extern u8  Lf_Ctrl_Enable;//使能控制
extern s16 Lf_Spd_Exp_Xcm;

extern s16 User_Yaw;//处理后的YAW角数据 范围负无穷到正无穷
extern s16 User_Yaw_OF;//YAW角溢出次数
extern s16 User_Yaw_Zero;//yaw角参考零点
extern s16 User_Yaw_attnum;//yaw角确实到达范围内次数，多次判断 过滤摄像头误识别

extern u8  AvoidingEnable ; //避障使能
extern s16 AvoidingXcmps ;	
extern s16 AvoidingYcmps ;	

void Program_Ctrl_User_Set_Speed(s16 xpcm, s16 ypcm, s16 zpcm, s16 degpres);//向飞控发送期望速度
void User_Loc_Ctrl(s16 true_x_cm,s16 true_y_cm,s16 exp_x_cm,s16 exp_y_cm);//位置环控制
void User_Alt_Ctrl(s16 true_z_cm,s16 exp_z_cm);//高度环控制
void User_Ang_Ctrl(s16 true_yaw_deg,s16 exp_yaw_deg);//角度环控制
void User_RaoGan(s16 true_r_cm,s16 true_yaw_deg ,s16 exp_r_cm,s16 exp_y_cm,s16 exp_yaw_deg,u8 mode);//绕杆控制
void User_Yaw_Calu(void);//YAW转换到连续的区间
void UserVisualAlignment(s16 true_x_cm,s16 true_y_cm ,s16 exp_x_cm,s16 exp_y_cm,s16 mode);//视觉跟踪
void UserVisualZCtrl(s16 true_z_cm,s16 exp_z_cm,s16 mode);// for 2019
void UserVisualYCtrl(s16 true_y_cm,s16 exp_y_cm,s16 mode);// for 2019
void LosTrapCheck(u8 t);

void FbOneKeyTakeOff(s16 true_x_cm,s16 true_y_cm,s16 true_z_cm,\
s16 exp_x_cm,s16 exp_y_cm,s16 exp_z_cm,u8 CtrlEnable);

//void FbOneKeyLand(s16 true_x_cm,s16 true_y_cm,s16 true_z_cm,\
//s16 exp_x_cm,s16 exp_y_cm,s16 exp_z_cm,u8 CtrlEnable);

void LineFollowingCtrl(s16 true_yaw_deg,s16 exp_yaw_deg ,s16 true_y_cm,s16 exp_y_cm,s16 exp_vx_cmps,u8 load_mode,u8 CtrlEnable);

#endif

