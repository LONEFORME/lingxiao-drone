/*==========================================================================
 * 描述    ：OPENMV数处理
 * 更新时间：2023年4月21日
 * 作者		 ：	LHB
===========================================================================*/

#ifndef __USER_OPMV_H
#define __USER_OPMV_H


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

typedef struct
{
	int16_t ifget;							//是否找到杆子
    int16_t ifget_temp;					    //是否找到杆子
    int16_t ifget_times;					    //是否找到杆子
	int16_t distance_cm;				//超声波测距
	int16_t posx_cm;						//杆相对水平的位置
	int16_t color;							//杆的颜色
	int16_t err_x;							//误差X
	int16_t err_y;							//误差Y
	int16_t element;						//元素
	int16_t element_last;				//上一次元素
	int16_t code;
	int16_t ang;
	
	
	int16_t point0_x;          //距离较近的点
	int16_t point0_y;					 //
	int16_t point1_x;					 //距离较远的点
	int16_t point1_y;				   //
	int16_t c_mode;            // 1 呼啦圈只找到一个点  2 呼啦圈找到两个点 
	
	int16_t line_err_y;
	int16_t line_err_k;
    int16_t line_err_yaw;
	
	int16_t temp1;
	int16_t temp2;
	
	int16_t target_spd_x_cmps;
	int16_t target_spd_y_cmps;
	
}opmv_data;

extern opmv_data openmv;
extern uint8_t find_color;//寻找杆的颜色 0不找 1找红的杆子 2找绿的杆子

void MV_GetOneByte(uint8_t data);//OPENMV数据获取
void MV_DataAnl(void);//OPENMV数据解析
void MV_DataSend(void);//上位机数据打包发送

#endif

