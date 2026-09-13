/*==========================================================================
 * 描述    ：路径规划与飞行任务
 * 更新时间：2023年4月05日
 * 作者		 ：	LHB
===========================================================================*/

#ifndef __USER_TASK_H
#define __USER_TASK_H

#include "SysConfig.h"

//==定义
typedef struct
{

		//坐标
	int16_t pos_x_exp;
	int16_t pos_y_exp;
	int16_t pos_z_exp;
	//欧拉角
	int16_t ya_f;
	
	uint8_t arrive;
	
}Fly_point_typedef;//定义航点结构体


extern int32_t user_program_time;//程序运行时间 全局变量
extern int16_t taskmode;//任务选择
extern int16_t mission_start;
extern int32_t takeoff_prepare;//起飞倒计时
extern Fly_point_typedef fly_point[100];//航点结构体数组 全局变量
extern Fly_point_typedef calibration[100];//T265标定点数组 全局变量
extern Fly_point_typedef fp_point[100];//现场编程航点
extern int16_t zhuanquan_errx ;//转呼啦圈
extern int16_t zhuanquan_erry ;
extern int16_t zhuanquan_k    ;

extern int16_t findfire,findfire_time;//是否找到


void UserTask_OneKeyCmd(void);
void FlyPointUpdate(void);//标定点更新
void FlagAllInit(void);

#endif
