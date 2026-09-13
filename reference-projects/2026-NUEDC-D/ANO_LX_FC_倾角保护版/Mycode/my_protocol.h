#include "Drv_Uart.h"
#include "stm32f4xx.h"
#include "Drv_AnoOf.h"
#include "angle_protect.h"
#include "LX_FC_State.h"
#ifndef _MY_PROTOCOL_H_
#define _MY_PROTOCOL_H_

// ========== 打包/解包宏 ==========
// 小端序（低字节在前）
#define W8(buf, v)       do { (buf)[0] = (u8)(v); } while(0)
#define W16_LE(buf, v)   do { (buf)[0]=(u8)(v); (buf)[1]=(u8)((v)>>8); } while(0)
#define W32_LE(buf, v)   do { (buf)[0]=(u8)(v); (buf)[1]=(u8)((v)>>8); \
                              (buf)[2]=(u8)((v)>>16); (buf)[3]=(u8)((v)>>24); } while(0)
#define R16_LE(buf) ((s16)((u16)(buf)[0] | ((u16)(buf)[1] << 8)))
#define R32_LE(buf) ((s32)((u32)(buf)[0] | ((u32)(buf)[1] << 8) | \
                           ((u32)(buf)[2] << 16) | ((u32)(buf)[3] << 24)))

///////////////////////////////////////数据结构
struct sdata
{
	s16 com_x;//x位置指令
	s16 com_y;//y位置指令
	s16 com_z;//z位置指令
	s16 com_yaw;//yaw指令
	u8 task_sta;//任务状态
	u8 next_task_sign;//阶段切换指令
	s16 sp_side;
};
struct PID_inc
{
	s16 target;
	s16 actual;
	float p;
	float i;
	float d;
	s32 err_current;
	s32 err_last;
	s32 err_previous;
};
extern u8 RxBuffer[256];
extern u8 LidarBuffer[256];
extern u8 pi_receive_done_sign;
extern u8 lidar_receive_done_sign;
extern u8 task_mode;
extern s16 CSPX,CSPY;
extern volatile s16 t265_vel_x, t265_vel_y;
extern volatile s16 t265_yaw_angle; // T265 偏航角，单位 0.01°，范围 [-18000,18000]
extern volatile s32 t265_pos_x, t265_pos_y, t265_pos_z; // T265 位置，单位cm（简化版：未做解锁时机头对齐）
///////////////////////////////////////结构体
struct lidar_data
{
	u16 lidar_speed;
	u16 start_angle;
	u16 end_angle;
	u16 point_dis[12];
	u16 point_credit[12];
	u16 timestamp;
	u8 crc;
};
///////////////////////////////////////函数
void my_spcal(s16,s16);
void pi_receive( u8 );
void pi_send();
void pi_send_debug();
void Send_str_by_len(USART_TypeDef * USARTx,u8 *s,u16 len);
void PID_init();
s16 height_set(u32 height, u16 height_target);
s16 xypid_set(s32 ,s16 ,u16 );
// ========== 灵活帧 API ==========
void flex_send(u8 id, const u8 *data, u8 len);
void flex_send_t265_vel(void);
void flex_send_guangliu_vel(void);
#endif
