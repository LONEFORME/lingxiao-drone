#include "my_protocol.h"
#include "User_Task.h"
#include "Ano_Math.h"
#include "ANO_LX.h"

/* Height positional PID state */
static s16 s_height_integral = 0;
static s16 s_height_err_last  = 0;
volatile struct sdata received_data={0,0,140,0,0,0};
struct PID_inc height_PID;
struct PID_inc xy_PID;
u8 RxBuffer[256];//树莓派接收数据缓存
u8 LidarBuffer[256];//激光雷达数据缓存
u8 pi_receive_done_sign=0;//树莓派接收完成标志位
u8 lidar_receive_done_sign=0;//激光雷达接收完成标志位
s16 CSPX=0,CSPY=0;
u8 x_high=0;
u8 x_low=0;
u8 y_high=0;
u8 y_low=0;
u8 rol_h=0;
u8 rol_l=0;
u8 pit_h=0;
u8 pit_l=0;
u8 yaw_h=0;
u8 yaw_l=0;
u8 att_state=0;

// 发送控制
// 帧1/帧2整帧一次性阻塞发送(Send_str_by_len)，不再逐字节跨tick分段，
// 29字节@460800波特率实际传输仅需约0.6ms，相对20ms调度周期可忽略不计，
// 避免了"分段发送状态机互相抢占USART2导致数据交织"以及"完整帧实际到达率远低于调度频率"两个问题
static u8 tx_debug_pending = 0; // 帧2发送请求标志，由 pi_send_debug() 置位
extern _ano_of_st ano_of; // 激光测距高度通过光流串口回传


s16 integ_side=0x4000;
volatile s16 t265_vel_x = 0, t265_vel_y = 0;
volatile s16 t265_yaw_angle = 0; // T265 偏航角，单位 0.01°
volatile s32 t265_pos_x = 0, t265_pos_y = 0, t265_pos_z = 0; // T265 位置，单位cm（简化版：未做解锁时机头对齐）

// ========== 灵活帧协议 ==========
// 帧格式: AA FF | ID(0xF1~0xFA) | LEN(1~40) | DATA[LEN] | SC | AC
// 校验使用小端序（低字节在前）
void flex_send(u8 id,const u8 *data, u8 len)
{
    u8 buf[2+1+1+40+2];  // 最大帧 46 字节
    u8 cnt = 0;

    buf[cnt++] = 0xAA;
    buf[cnt++] = 0xFF;
    buf[cnt++] = id;        // 数据帧 0xF1~0xFA
    buf[cnt++] = len;       // 数据字节数

    for (u8 i = 0; i < len; i++)
        buf[cnt++] = data[i];

    // SC/AC 校验和（Fletcher-8）
    u8 sc = 0, ac = 0;
    for (u8 i = 0; i < cnt; i++)
    {
        sc += buf[i];
        ac += sc;
    }
    buf[cnt++] = sc;
    buf[cnt++] = ac;

    UartSendLXIMU(buf, cnt);
}

// 发送 T265 速度数据，通过灵活帧 0xF1 转发到 IMU 口
void flex_send_t265_vel(void)
{
    u8 data[4];
    W16_LE(data,   t265_vel_x);
    W16_LE(data+2, t265_vel_y);
    flex_send(0xF1, data, 4);
}

// 发送 光流 速度数据，通过灵活帧 0xF2 转发到 IMU 口
void flex_send_guangliu_vel(void)
{
    u8 data[4];
    W16_LE(data,   ano_of.of1_dx);
    W16_LE(data+2, ano_of.of1_dy);
    flex_send(0xF2, data, 4);
}

void Send_str_by_len(USART_TypeDef * USARTx,u8 *s,u16 len)//用于发送函数
{
	u16 i=0;
	while(i<len)
	{
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TC )==RESET);
		USART_SendData(USARTx,*s);
		s++;
		i++;
	}
}
void pi_receive(u8 data)//树莓派接收协议 串口2
{
	static u8 state_1 = 0;
	if(state_1==0&&data==0xAA)	//帧头0xAA
	{
		state_1=1;
		RxBuffer[0]=data;
	}
	else if(state_1==1)	//帧第二字节
	{
		RxBuffer[1]=data;
		if(data == 0x01)		//T265速度帧: AA 01 vx_h vx_l vy_h vy_l yaw_h yaw_l CK FF
			state_1 = 2;
		else if(data == 0x02)	//飞行指令帧: AA 02 task_sta com_x com_y com_z com_yaw next_task sp_side CK FF
			state_1 = 10;
		else if(data == 0x03)	//T265位置帧: AA 03 x0..x3 y0..y3 z0..z3 CK FF (小端序，单位cm)
			state_1 = 30;
		else
			state_1 = 0;		//未知帧放弃
	}
	//--- T265速度帧 (0x01) ---
	else if(state_1==2)		{RxBuffer[2]=data; state_1=3;}	// vx_h
	else if(state_1==3)		{RxBuffer[3]=data; state_1=4;}	// vx_l
	else if(state_1==4)		{RxBuffer[4]=data; state_1=5;}	// vy_h
	else if(state_1==5)		{RxBuffer[5]=data; state_1=6;}	// vy_l
	else if(state_1==6)		{RxBuffer[6]=data; state_1=7;}	// yaw_h
	else if(state_1==7)		{RxBuffer[7]=data; state_1=8;}	// yaw_l
	else if(state_1==8)		{RxBuffer[8]=data; state_1=9;}	// CK
	else if(state_1==9&&data==0xFF)	//帧尾
	{
		RxBuffer[9]=data;
		u8 ck = 0;
		for(u8 i=1; i<8; i++) ck ^= RxBuffer[i];
		if(ck == RxBuffer[8])
		{
			t265_vel_x = ((s16)RxBuffer[2] << 8) | RxBuffer[3];
			t265_vel_y = ((s16)RxBuffer[4] << 8) | RxBuffer[5];
			t265_yaw_angle = ((s16)RxBuffer[6] << 8) | RxBuffer[7];
		}
		state_1 = 0;
	}
	//--- 飞行指令帧 (0x02) ---
	else if(state_1==10)	{RxBuffer[2]=data; state_1=11;}	// task_sta
	else if(state_1==11)	{RxBuffer[3]=data; state_1=12;}	// com_x
	else if(state_1==12)	{RxBuffer[4]=data; state_1=13;}	// com_y
	else if(state_1==13)	{RxBuffer[5]=data; state_1=14;}	// com_z
	else if(state_1==14)	{RxBuffer[6]=data; state_1=15;}	// com_yaw
	else if(state_1==15)	{RxBuffer[7]=data; state_1=16;}	// next_task
	else if(state_1==16)	{RxBuffer[8]=data; state_1=17;}	// sp_side
	else if(state_1==17)	{RxBuffer[9]=data; state_1=18;}	// CK
	else if(state_1==18&&data==0xFF)	//帧尾
	{
		RxBuffer[10]=data;
		u8 ck = 0;
		for(u8 i=1; i<9; i++) ck ^= RxBuffer[i];
		if(ck == RxBuffer[9])
		{
			received_data.sp_side   = RxBuffer[8];
			received_data.task_sta  = RxBuffer[2];
			received_data.com_x     = RxBuffer[3] - received_data.sp_side;
			received_data.com_y     = RxBuffer[4] - received_data.sp_side;
			received_data.com_z     = RxBuffer[5];
			received_data.com_yaw   = RxBuffer[6] - received_data.sp_side;
			received_data.next_task_sign = RxBuffer[7];
			pi_receive_done_sign    = 1;
		}
		state_1 = 0;
	}
	//--- T265位置帧 (0x03) ---
	else if(state_1==30)	{RxBuffer[2]=data; state_1=31;}	// x0
	else if(state_1==31)	{RxBuffer[3]=data; state_1=32;}	// x1
	else if(state_1==32)	{RxBuffer[4]=data; state_1=33;}	// x2
	else if(state_1==33)	{RxBuffer[5]=data; state_1=34;}	// x3
	else if(state_1==34)	{RxBuffer[6]=data; state_1=35;}	// y0
	else if(state_1==35)	{RxBuffer[7]=data; state_1=36;}	// y1
	else if(state_1==36)	{RxBuffer[8]=data; state_1=37;}	// y2
	else if(state_1==37)	{RxBuffer[9]=data; state_1=38;}	// y3
	else if(state_1==38)	{RxBuffer[10]=data; state_1=39;}	// z0
	else if(state_1==39)	{RxBuffer[11]=data; state_1=40;}	// z1
	else if(state_1==40)	{RxBuffer[12]=data; state_1=41;}	// z2
	else if(state_1==41)	{RxBuffer[13]=data; state_1=42;}	// z3
	else if(state_1==42)	{RxBuffer[14]=data; state_1=43;}	// CK
	else if(state_1==43&&data==0xFF)	//帧尾
	{
		RxBuffer[15]=data;
		u8 ck = 0;
		for(u8 i=1; i<14; i++) ck ^= RxBuffer[i];
		if(ck == RxBuffer[14])
		{
			t265_pos_x = ((s32)RxBuffer[2]) | ((s32)RxBuffer[3]<<8) | ((s32)RxBuffer[4]<<16) | ((s32)RxBuffer[5]<<24);
			t265_pos_y = ((s32)RxBuffer[6]) | ((s32)RxBuffer[7]<<8) | ((s32)RxBuffer[8]<<16) | ((s32)RxBuffer[9]<<24);
			t265_pos_z = ((s32)RxBuffer[10]) | ((s32)RxBuffer[11]<<8) | ((s32)RxBuffer[12]<<16) | ((s32)RxBuffer[13]<<24);
		}
		state_1 = 0;
	}
	else
	{
		state_1=0;
	}
}
void PID_init()
{
	height_PID.p=0.8;
	height_PID.i=0.3;
	height_PID.d=0.2;
	height_PID.actual=0;
	height_PID.target=0;
	height_PID.err_current=0;
	height_PID.err_last=0;
	height_PID.err_previous=0;
	xy_PID.p=1.5;
	xy_PID.i=0.9;
	xy_PID.d=0.6;
	xy_PID.actual=0;
	xy_PID.target=0;
	xy_PID.err_current=0;
	xy_PID.err_last=0;
	xy_PID.err_previous=0;
	/* Reset height PID state */
	s_height_integral = 0;
	s_height_err_last  = 0;
}

s16 height_set(u32 height, u16 height_target)
{
	float rol_deg, pit_deg, tilt_deg, tilt_rad;
	s16 err, i_term, output;

	/* Tilt compensation: convert slant range to vertical height */
	rol_deg = fc_att.st_data.rol_x100 / 100.0f;
	pit_deg = fc_att.st_data.pit_x100 / 100.0f;
	tilt_deg = my_sqrt(rol_deg * rol_deg + pit_deg * pit_deg);
	if (tilt_deg > 45.0f) tilt_deg = 45.0f;
	tilt_rad = tilt_deg * 0.0174533f;
	height = (u32)((float)height * my_cos(tilt_rad));

	err = (s16)height_target - (s16)height;

	/* Integral separation: disable integral when |err| > 200 cm */
	if (err > 200 || err < -200) {
		i_term = 0;
		s_height_integral = 0;
	} else {
		s_height_integral += err;
		if (s_height_integral >  100) s_height_integral =  100;
		if (s_height_integral < -100) s_height_integral = -100;
		i_term = (s16)(0.05f * s_height_integral);
	}

	/* Positional PID: Kp=0.8, Ki=0.05, Kd=0.2 */
	output = (s16)(0.8f * err + i_term + 0.2f * (err - s_height_err_last));
	s_height_err_last = err;

	if (output >  30) output =  30;
	if (output < -30) output = -30;
	return output;
}

//s16 height_set(u32 height,u16 height_set)
//{
//	s16 output=0;
//	// 倾斜补偿：将测距仪斜距转为垂直高度，避免水平移动时高度波动
//	{
//		float rol_deg = fc_att.st_data.rol_x100 / 100.0f;
//		float pit_deg = fc_att.st_data.pit_x100 / 100.0f;
//		float tilt_deg = my_sqrt(rol_deg * rol_deg + pit_deg * pit_deg);
//		if (tilt_deg > 45.0f) tilt_deg = 45.0f;
//		float tilt_rad = tilt_deg * 0.0174533f;
//		height = (u32)((float)height * my_cos(tilt_rad));
//	}
//	height_PID.actual=height;
//	height_PID.target=height_set;
//	height_PID.err_current=height_PID.target-height_PID.actual;

//	// 积分分离 + 真积分累积
//	static s16 height_integral = 0;
//	s16 i_term;
//	if (height_PID.err_current > 200 || height_PID.err_current < -200)
//	{
//		i_term = 0;
//		height_integral = 0;  // 误差过大时清积分防饱和
//	}
//	else
//	{
//		height_integral += height_PID.err_current;
//		if (height_integral > 100) height_integral = 100;
//		if (height_integral < -100) height_integral = -100;
//		i_term = (s16)(height_PID.i * height_integral);
//	}

//	output = height_PID.p * height_PID.err_current
//	       + i_term
//	       + height_PID.d * (height_PID.err_current - height_PID.err_last);

//	height_PID.err_previous = height_PID.err_last;
//	height_PID.err_last = height_PID.err_current;

//	if (output > 30) output = 30;
//	else if (output < -30) output = -30;
//	return output;
//}

// 校验和：字节1~12累加
static u8 calc_checksum(u8 *buf, u8 count)
{
    u16 sum = 0;
    for(u8 i=1; i<=count; i++) sum += buf[i];
    return (u8)(sum & 0xFF);
}

void pi_send(void)
{
    // 帧1/帧2共用同一条USART2，整帧一次性阻塞发送(Send_str_by_len)，
    // 任意时刻只会有一个完整帧在发送，不存在两帧交织的问题；
    // 29字节@460800波特率实际传输约0.6ms，对20ms调度周期可忽略不计。
    if(tx_debug_pending)
    {
        // ========== 打包并整帧发送 (ID=0x02: 调试扩展帧) ==========
        u8 buf2[24];
        tx_debug_pending = 0;

        buf2[0] = 0xAA;
        buf2[1] = 0x02;  // 帧类型: 调试扩展帧
        buf2[2] = 19;    // 数据长度

        // 飞控（凌霄IMU）速度估计
        buf2[3] = (fc_vel.st_data.vel_x >> 0) & 0xFF;
        buf2[4] = (fc_vel.st_data.vel_x >> 8) & 0xFF;
        buf2[5] = (fc_vel.st_data.vel_y >> 0) & 0xFF;
        buf2[6] = (fc_vel.st_data.vel_y >> 8) & 0xFF;
        buf2[7] = (fc_vel.st_data.vel_z >> 0) & 0xFF;
        buf2[8] = (fc_vel.st_data.vel_z >> 8) & 0xFF;

        // 光流模块自带 IMU 原始数据（振动/交叉验证诊断用）
        buf2[9]  = (ano_of.acc_data_x >> 0) & 0xFF;
        buf2[10] = (ano_of.acc_data_x >> 8) & 0xFF;
        buf2[11] = (ano_of.acc_data_y >> 0) & 0xFF;
        buf2[12] = (ano_of.acc_data_y >> 8) & 0xFF;
        buf2[13] = (ano_of.acc_data_z >> 0) & 0xFF;
        buf2[14] = (ano_of.acc_data_z >> 8) & 0xFF;
        buf2[15] = (ano_of.gyr_data_x >> 0) & 0xFF;
        buf2[16] = (ano_of.gyr_data_x >> 8) & 0xFF;
        buf2[17] = (ano_of.gyr_data_y >> 0) & 0xFF;
        buf2[18] = (ano_of.gyr_data_y >> 8) & 0xFF;
        buf2[19] = (ano_of.gyr_data_z >> 0) & 0xFF;
        buf2[20] = (ano_of.gyr_data_z >> 8) & 0xFF;

        // 电机PWM非零位掩码(bit0~3=m1~m4)：诊断unlock_sta(来自凌霄IMU CMD0x06)
        // 与实际PWM输出(来自CMD0x20，pwm_to_esc)是否同步，排查"确认已上锁但电机未停转"问题
        buf2[21] = (pwm_to_esc.pwm_m1 != 0 ? 0x01 : 0) |
                   (pwm_to_esc.pwm_m2 != 0 ? 0x02 : 0) |
                   (pwm_to_esc.pwm_m3 != 0 ? 0x04 : 0) |
                   (pwm_to_esc.pwm_m4 != 0 ? 0x08 : 0) |
                   (land_timeout_gaveup_f != 0 ? 0x10 : 0);  // 2026-07-12新增:bit4=纯超时兜底是否已放弃自动锁定(高度仍偏高)

        // 校验 + 帧尾（checksum覆盖buf2[1]~buf2[21]）
        buf2[22] = calc_checksum(buf2, 21);
        buf2[23] = 0xFF;

        Send_str_by_len(USART2, buf2, 24);
        return;
    }

    // ========== 打包并整帧发送 (ID=0x01: 飞行关键帧) ==========
    u8 buf[29];

    buf[0] = 0xAA;  // 帧头
    buf[1] = 0x01;  // 帧类型: 飞行关键帧
    buf[2] = 24;    // 数据长度

    buf[3] = mission_stage;

    // 姿态角
    buf[4] = (fc_att.st_data.rol_x100 >> 0) & 0xFF;
    buf[5] = (fc_att.st_data.rol_x100 >> 8) & 0xFF;
    buf[6] = (fc_att.st_data.pit_x100 >> 0) & 0xFF;
    buf[7] = (fc_att.st_data.pit_x100 >> 8) & 0xFF;
    buf[8] = (fc_att.st_data.yaw_x100 >> 0) & 0xFF;
    buf[9] = (fc_att.st_data.yaw_x100 >> 8) & 0xFF;
    buf[10] = fc_att.st_data.state;   // 姿态融合状态

    buf[11] = fc_sta.unlock_sta;      // 解锁状态（真实值，来自凌霄IMU CMD 0x06）

    // X/Y 积分（光流）
    s16 x = ano_of.intergral_x + 0x4000;
    s16 y = ano_of.intergral_y + 0x4000;
    buf[12] = (x >> 0) & 0xFF;
    buf[13] = (x >> 8) & 0xFF;
    buf[14] = (y >> 0) & 0xFF;
    buf[15] = (y >> 8) & 0xFF;

    // 激光测距高度（cm），通过光流串口回传
    u32 h = ano_of.of_alt_cm;
    buf[16] = (h >> 0) & 0xFF;
    buf[17] = (h >> 8) & 0xFF;
    buf[18] = (h >> 16) & 0xFF;
    buf[19] = (h >> 24) & 0xFF;

    // 光流融合速度 (of1_dx/dy)
    buf[20] = (ano_of.of1_dx >> 0) & 0xFF;
    buf[21] = (ano_of.of1_dx >> 8) & 0xFF;
    buf[22] = (ano_of.of1_dy >> 0) & 0xFF;
    buf[23] = (ano_of.of1_dy >> 8) & 0xFF;

    // 光流质量/连接状态/工作状态
    buf[24] = ano_of.of_quality;
    buf[25] = ano_of.link_sta;
    buf[26] = ano_of.work_sta;

    // 校验 + 帧尾（checksum覆盖buf[1]~buf[26]）
    buf[27] = calc_checksum(buf, 26);
    buf[28] = 0xFF;

    Send_str_by_len(USART2, buf, 29);
}

void pi_send_debug(void)
{
    // 帧2(23字节)整帧发送耗时约0.4ms(可忽略)，但仍不宜太频繁触发调试数据，
    // 这里做降频：每调用5次才真正触发一次(约2.5秒一次)。
    static u8 div_cnt = 0;
    div_cnt++;
    if (div_cnt >= 5)
    {
        div_cnt = 0;
        tx_debug_pending = 1;
    }
}

//void pi_send()
//{
//	static u8 stage=0;
//	if(stage==0)
//	{
//		USART_SendData(USART2,0xAA);
//		stage=1;
//		s16 num = ano_of.intergral_x + 0x4000;
//		x_high=(num >> 8) & 0xFF;
//		x_low=num & 0xFF;
//		s16 ynum = ano_of.intergral_y + 0x4000;
//		y_high=(ynum >> 8) & 0xFF;
//		y_low=ynum & 0xFF;
//	}
//	else if(stage==1)
//	{
//		USART_SendData(USART2,mission_stage);
//		stage=2;
//	}
//	else if(stage==2)
//	{

//		USART_SendData(USART2,x_high);
//		//USART_SendData(USART2,0x05);
//		stage=3;
//	}
//	else if(stage==3)
//	{
//		USART_SendData(USART2,x_low);
//		stage=4;
//	}
//	else if(stage==4)
//	{

//		USART_SendData(USART2,y_high);
//		//USART_SendData(USART2,0x05);
//		stage=5;
//	}
//	else if(stage==5)
//	{
//		USART_SendData(USART2,y_low);
//		stage=6;
//	}
//	else if(stage==6)
//	{
//		USART_SendData(USART2,0xFF);
//		stage=0;
//	}
//	else stage=0;
//}

void my_spcal(s16 x,s16 y)
{
        CSPX = y*0.3 ;
        CSPY = x*0.3 ;
	if(CSPX>30) CSPX=30;
	if(CSPX<-30) CSPX=-30;
	if(CSPY>30) CSPY=30;
	if(CSPY<-30) CSPY=-30;
}
s16 xypid_set(s32 xy,s16 xy_set,u16 speedmax)
{
	s16 output=0;
	xy_PID.actual=xy;
	xy_PID.target=xy_set;
	xy_PID.err_current=xy_PID.target-xy_PID.actual;
	output=xy_PID.p*(xy_PID.err_current-xy_PID.err_last)+xy_PID.i*xy_PID.err_current+xy_PID.d*(xy_PID.err_current-2*xy_PID.err_last+xy_PID.err_previous);
	xy_PID.err_previous=xy_PID.err_last;
	xy_PID.err_last=xy_PID.err_current;
	if (output>speedmax ) output=speedmax;
	else if(output<-speedmax) output=-speedmax;
	return output;
}
