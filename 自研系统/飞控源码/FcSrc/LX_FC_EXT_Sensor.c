/*==========================================================================
 * 文件名   外部扩展传感器数据
 * 创建时间：2020-02-06
 * 作者	 匿名科创-Jyoun
 * 网站   www.anotc.com
 * 淘宝   anotc.taobao.com
 * 技术Q群 190169595
 * 固件热线：18084888982，18061373080
============================================================================
 * 匿名科创团队感谢大家的支持，欢迎大家进群多交流、讨论、学习。
 * 若程序中有做得不好的地方，欢迎大家拍砖、指正。
 * 如果您觉得固件对你有用，希望您可以赞助并支持我们。
 * 本着开源精神欢迎广大爱好者自由使用、修改、再发布，但希望使用时间注明出处。
 * 鄙人坦荡荡，小人常戚戚，产品地水军水手，也从未有过抹黑同行的行为。
 * 开源靠大家，在此谢过各位，希望新的一年大家相互尊重、互惠互助、共同进步。
 * 只要大家支持，我们一定把固件做得更好。
===========================================================================*/
#include "LX_FC_EXT_Sensor.h"
#include "Drv_AnoOf.h"
#include "ANO_DT_LX.h"
#include "ANO_LX.h"
#include "my_protocol.h"
extern volatile u8 pi_ctrl_mode;
extern volatile s16 t265_vel_x, t265_vel_y;

_fc_ext_sensor_st ext_sens;

//通用外部传感器数据通过速度传感器融合
static inline void General_Velocity_Data_Handle()
{
	static u8 of_update_cnt, of_alt_update_cnt;
	static u8 dT_ms = 0;
	//每一次给dT_ms+1，再判断是否超时复位
	if (dT_ms != 255)
	{
		dT_ms++;
	}
	// T265 模式：T265 速度为主信号（无漂移），光流辅助高频响应
	if (pi_ctrl_mode == 1)
	{
		s16 vx = t265_vel_x;
		s16 vy = t265_vel_y;

		// // 光流新数据到达时做互补融合，平滑过渡
		// if (of_update_cnt != ano_of.of_update_cnt)
		// {
		// 	of_update_cnt = ano_of.of_update_cnt;
		// 	if (ano_of.of1_sta && ano_of.work_sta) //光流有效
		// 	{
		// 		vx = (s16)(t265_vel_x * 0.7f + ano_of.of1_dx * 0.3f);
		// 		vy = (s16)(t265_vel_y * 0.7f + ano_of.of1_dy * 0.3f);
		// 	}
		// }

		ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = vx;
		ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = vy;
		// T265 yaw complementary filter (~10Hz, gentle correction)
		{
			static u8 yaw_corr_cnt = 0;
			yaw_corr_cnt++;
			if (yaw_corr_cnt >= 100)
			{
				yaw_corr_cnt = 0;
				s32 diff = (s32)t265_yaw_angle - (s32)fc_att.st_data.yaw_x100;
				if (diff > 18000) diff -= 36000;
				else if (diff < -18000) diff += 36000;
				fc_att.st_data.yaw_x100 += (s16)(diff * 0.03f);
			}
		}
	}
	//非 T265 模式：光流作为唯一速度源
	else if (of_update_cnt != ano_of.of_update_cnt)
	{
		of_update_cnt = ano_of.of_update_cnt;
		if (ano_of.of1_sta && ano_of.work_sta) //光流有效
		{
			ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = ano_of.of1_dx;
			ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = ano_of.of1_dy;
		}
		else //无效
		{
			ext_sens.gen_vel.st_data.hca_velocity_cmps[0] = 0x8000;
			ext_sens.gen_vel.st_data.hca_velocity_cmps[1] = 0x8000;
		}
	}
	if (of_alt_update_cnt != ano_of.alt_update_cnt)
	{
		//
		of_alt_update_cnt = ano_of.alt_update_cnt;
		//重置Z轴速度，将Z速度赋值为无效
		ext_sens.gen_vel.st_data.hca_velocity_cmps[2] = 0x8000;
		//触发数据更新
		dt.fun[0x33].WTS = 1;
		//reset
		dT_ms = 0;
	}
}

static inline void General_Distance_Data_Handle()
{
	static u8 of_alt_update_cnt;
	if (of_alt_update_cnt != ano_of.alt_update_cnt)
	{
		//
		of_alt_update_cnt = ano_of.alt_update_cnt;
		//
		ext_sens.gen_dis.st_data.direction = 0;
		ext_sens.gen_dis.st_data.angle_100 = 270;
		ext_sens.gen_dis.st_data.distance_cm = ano_of.of_alt_cm;
		//触发数据更新
		dt.fun[0x34].WTS = 1;
	}
}

void LX_FC_EXT_Sensor_Task(float dT_s) //1ms
{
	//
	General_Velocity_Data_Handle();
	//
	General_Distance_Data_Handle();
}
