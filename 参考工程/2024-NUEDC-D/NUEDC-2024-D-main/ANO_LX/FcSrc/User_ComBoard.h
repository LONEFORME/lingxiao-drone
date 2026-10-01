/*==========================================================================
 * 描述    ：串口拓展板数据处理
 * 更新时间：2023年4月5日
 * 作者		 ：	LHB
===========================================================================*/

#ifndef __USER_COMBOARD_H
#define __USER_COMBOARD_H

#include "ANO_LX.h"
#include "Drv_RcIn.h"
#include "ANO_DT_LX.h"
#include "ANO_Math.h"
#include "Drv_PwmOut.h"
#include "LX_FC_State.h"
#include "LX_FC_EXT_Sensor.h"
#include "Drv_AnoOf.h"
#include "Drv_adc.h"
#include "Drv_led.h"
#include "Drv_UbloxGPS.h"
#include "LX_FC_Fun.h"
#include "Drv_Uart.h"


#include "User_RC.h"
#include "User_Ctrl.h"
#include "User_T265.h"
#include "User_Opmv.h"
typedef struct
{
	int16_t fly_state;			//当前飞行状态 0上锁 1进入起飞倒计时 2解锁起飞 3开始程控任务 4开始降落
	int16_t LxFcState;			//凌霄飞控状态 1姿态控制模式 2定点模式 3程控模式
	int16_t bat_v100;				//电池电压 单位10MV
	int16_t CtrlState;			//控制模式 0位置环 1绕杆 2自动降落 3程序单独控制速度
	int16_t sped_state;			//速度传感器来源 0 光流 1 T265 2 保留
	int16_t time_s;					//代码运行时间 单位 秒
	int16_t pos_x;					//位置 单位CM
	int16_t pos_y;					//位置 单位CM
	int16_t pos_z;					//位置 单位CM
	int16_t yaw;						//偏航角 单位度
	
}FC_data;

extern FC_data LX_FC;
extern u8 Servo[4];			//舵机控制
extern u8 playvoice;//播报
extern u8 preparetopoint,preparetopoint_time;
extern u8 lazer[2];
extern u8 goods[30];//货物
extern u8 goods_pos[24];//货物
void FC_DateUpdate(void);//飞行状态更新
void BD_GetOneByte(uint8_t data);//下位机数据获取
void BD_DataAnl(void);//下位机数据解析
void BD_DataSend(void);//上位机数据打包发送

#endif
