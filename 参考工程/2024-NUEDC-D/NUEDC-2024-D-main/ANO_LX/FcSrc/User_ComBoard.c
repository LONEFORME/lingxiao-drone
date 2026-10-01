/*==========================================================================
 * 描述    ：串口拓展板数据处理
 * 更新时间：2023年4月5日
 * 作者		 ：	LHB
===========================================================================*/

#include "User_ComBoard.h"
#include "Drv_Uart.h"
#include "Drv_UbloxGPS.h"
#include "ANO_DT_LX.h"
#include "Drv_AnoOf.h"
#include "User_Task.h"
#include "User_Opmv.h"

u8 preparetopoint=0,preparetopoint_time=0;

u8 data_to_bd[30]={0};//发送数据缓冲
u8 data_fr_bd[30]={0};//接收数据缓冲
u8 lazer[2]={0};
u8 goods[30]={0};//货物 0:当前货物 25:当前位置 26:初始化标志位
u8 goods_pos[24]={0};//货物是否已存
u8 playvoice = 0;//播报
u8 Servo[4]={0};			//舵机控制
FC_data LX_FC={0};

void FC_DateUpdate(void)//飞行状态更新
{
	LX_FC.LxFcState=fc_sta.fc_mode_sta;//凌霄飞控状态更新
	LX_FC.bat_v100=fc_bat.st_data.voltage_100;//电池电压更新
	LX_FC.time_s=user_program_time/1000;//程序运行时间
	
	LX_FC.pos_x=t265.pos_x_f;
	LX_FC.pos_y=t265.pos_y_f;
	LX_FC.pos_z=ano_of.of_alt_cm;      
//    LX_FC.pos_z=t265.pos_z_f;                                     
	LX_FC.yaw=t265.ya_f;
//    LX_FC.yaw=t265.ro_f;
}
void BD_GetOneByte(uint8_t data)//下位机数据获取
{
		static uint8_t rec_sta = 0;

	data_fr_bd[rec_sta] = data;
    
	if(rec_sta==0)
	{
		if(data==0X15)/*帧头0X15*/
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}
	}
	else if(rec_sta==1)
	{
		if(data==0X79)/*帧头0X79*/
		{
			rec_sta++;
		}	
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==2)
	{
		if(data==0X78)/*帧头0X78*/
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}		
	}
	else if(rec_sta==3)
	{
		/*功能字：00到09*/
		if(data<=0x09)
		{
			rec_sta++;
		}
		else
		{
			rec_sta=0;
		}	
	}
	else if(rec_sta==4)
	{
			rec_sta++;
	}
    else if(rec_sta==5)
	{
			rec_sta++;		
	}
	else if(rec_sta==6)
	{
			rec_sta++;		
	}
	else if(rec_sta==7)
	{
			rec_sta++;		
	}
	else if(rec_sta==8)
	{
			rec_sta++;		
	}
	else if(rec_sta==9)
	{
			rec_sta++;		
	}
	else if(rec_sta==10)
	{
			rec_sta++;		
	}
	else if(rec_sta==11)
	{
			rec_sta++;		
	}
	else if(rec_sta==12)
	{
			rec_sta++;		
	}
	else if(rec_sta==13)
	{
			rec_sta++;		
	}
	else if(rec_sta==14)/*帧尾*/
	{
        if( data == 0X05 )/*正确的位置接收到帧尾，开始解析*/
        {           
						BD_DataAnl();
            rec_sta=0;
        }
				else
				{
					rec_sta=0;
				}		
	}
	else
	{
		rec_sta=0;
	}
}
void BD_DataAnl(void)//下位机数据解析
{
	if(data_fr_bd[3]==0x00)//起飞准备模式
	{
		//一键启动板发送起飞指令，飞控返回校验同时清除标志位，这是为了确保起飞指令确实收到了
		if((s16)((data_fr_bd[4]<<8)|data_fr_bd[5])==1)
		{
			LX_FC.fly_state=1;//标记等待解锁起飞
		}
        preparetopoint=data_fr_bd[6];
	}else
	if(data_fr_bd[3]==0x01)//位置标定模式 地点1,2
	{
		calibration[0].pos_x_exp=(int16_t)((data_fr_bd[4]<<8)|data_fr_bd[5]);
		calibration[0].pos_y_exp=(int16_t)((data_fr_bd[6]<<8)|data_fr_bd[7]);
		calibration[1].pos_x_exp=(int16_t)((data_fr_bd[8]<<8)|data_fr_bd[9]);
		calibration[1].pos_y_exp=(int16_t)((data_fr_bd[10]<<8)|data_fr_bd[11]);
	}else
	if(data_fr_bd[3]==0x02)//位置标定模式 地点3,4
	{
		calibration[2].pos_x_exp=(int16_t)((data_fr_bd[4]<<8)|data_fr_bd[5]);
		calibration[2].pos_y_exp=(int16_t)((data_fr_bd[6]<<8)|data_fr_bd[7]);
		calibration[3].pos_x_exp=(int16_t)((data_fr_bd[8]<<8)|data_fr_bd[9]);
		calibration[3].pos_y_exp=(int16_t)((data_fr_bd[10]<<8)|data_fr_bd[11]);
	}else
	if(data_fr_bd[3]==0x03)//位置标定模式 地点5,6
	{
		calibration[4].pos_x_exp=(int16_t)((data_fr_bd[4]<<8)|data_fr_bd[5]);
		calibration[4].pos_y_exp=(int16_t)((data_fr_bd[6]<<8)|data_fr_bd[7]);
		calibration[5].pos_x_exp=(int16_t)((data_fr_bd[8]<<8)|data_fr_bd[9]);
		calibration[5].pos_y_exp=(int16_t)((data_fr_bd[10]<<8)|data_fr_bd[11]);
	}else
	if(data_fr_bd[3]==0x04)//位置标定模式 地点7,8
	{
		calibration[6].pos_x_exp=(int16_t)((data_fr_bd[4]<<8)|data_fr_bd[5]);
		calibration[6].pos_y_exp=(int16_t)((data_fr_bd[6]<<8)|data_fr_bd[7]);
		calibration[7].pos_x_exp=(int16_t)((data_fr_bd[8]<<8)|data_fr_bd[9]);
		calibration[7].pos_y_exp=(int16_t)((data_fr_bd[10]<<8)|data_fr_bd[11]);
		
	}else
	if(data_fr_bd[3]==0x05)//位置标定模式 地点9,10
	{
		calibration[8].pos_x_exp=(int16_t)((data_fr_bd[4]<<8)|data_fr_bd[5]);
		calibration[8].pos_y_exp=(int16_t)((data_fr_bd[6]<<8)|data_fr_bd[7]);
		calibration[9].pos_x_exp=(int16_t)((data_fr_bd[8]<<8)|data_fr_bd[9]);
		calibration[9].pos_y_exp=(int16_t)((data_fr_bd[10]<<8)|data_fr_bd[11]);
	}
	else
	if(data_fr_bd[3]==0x06)//位置标定模式 地点11,12
	{
		calibration[10].pos_x_exp=(int16_t)((data_fr_bd[4]<<8)|data_fr_bd[5]);
		calibration[10].pos_y_exp=(int16_t)((data_fr_bd[6]<<8)|data_fr_bd[7]);
		calibration[11].pos_x_exp=(int16_t)((data_fr_bd[8]<<8)|data_fr_bd[9]);
		calibration[11].pos_y_exp=(int16_t)((data_fr_bd[10]<<8)|data_fr_bd[11]);
	}else	
	if(data_fr_bd[3]==0x07)//位置标定模式 地点13,14
	{
		calibration[12].pos_x_exp=(int16_t)((data_fr_bd[4]<<8)|data_fr_bd[5]);
		calibration[12].pos_y_exp=(int16_t)((data_fr_bd[6]<<8)|data_fr_bd[7]);
		calibration[13].pos_x_exp=(int16_t)((data_fr_bd[8]<<8)|data_fr_bd[9]);
		calibration[13].pos_y_exp=(int16_t)((data_fr_bd[10]<<8)|data_fr_bd[11]);
	}else	
	if(data_fr_bd[3]==0x08)//位置标定模式 地点15,16
	{
		calibration[14].pos_x_exp=(int16_t)((data_fr_bd[4]<<8)|data_fr_bd[5]);
		calibration[14].pos_y_exp=(int16_t)((data_fr_bd[6]<<8)|data_fr_bd[7]);
		calibration[15].pos_x_exp=(int16_t)((data_fr_bd[8]<<8)|data_fr_bd[9]);
		calibration[15].pos_y_exp=(int16_t)((data_fr_bd[10]<<8)|data_fr_bd[11]);
	}else	
	if(data_fr_bd[3]==0x09)//飞行任务选择模式`
	{
		taskmode=(int16_t)(data_fr_bd[4]);//任务模式选择
	}
	
	
}

void BD_DataSend(void)//上位机数据打包发送
{
	static int8_t mode=0;//模式选择
	u8 i=0;//发送计数
	mode++;
	mode%=16;
	////////////////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==0)//基本飞行状态
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x00;//功能字00 基本飞行状态
	data_to_bd[i++]=BYTE1(LX_FC.fly_state);
	data_to_bd[i++]=BYTE0(LX_FC.fly_state);
	data_to_bd[i++]=BYTE1(LX_FC.pos_x);
	data_to_bd[i++]=BYTE0(LX_FC.pos_x);
	data_to_bd[i++]=BYTE1(LX_FC.pos_y);
	data_to_bd[i++]=BYTE0(LX_FC.pos_y);
	data_to_bd[i++]=BYTE1(LX_FC.pos_z);
	data_to_bd[i++]=BYTE0(LX_FC.pos_z);
	data_to_bd[i++]=BYTE1(LX_FC.yaw);
	data_to_bd[i++]=BYTE0(LX_FC.yaw);
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}else
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==1)//航点确认 航点1
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x01;//功能字01 航点确认 航点1,2
	data_to_bd[i++]=BYTE1(calibration[0].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[0].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[0].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[0].pos_y_exp);
	data_to_bd[i++]=BYTE1(calibration[1].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[1].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[1].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[1].pos_y_exp);
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}else
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==2)//航点确认 航点2
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x02;//功能字02 航点确认 航点3,4
	data_to_bd[i++]=BYTE1(calibration[2].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[2].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[2].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[2].pos_y_exp);
	data_to_bd[i++]=BYTE1(calibration[3].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[3].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[3].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[3].pos_y_exp);
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}else
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==3)//航点确认 航点3
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x03;//功能字03 航点确认 航点5,6
	data_to_bd[i++]=BYTE1(calibration[4].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[4].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[4].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[4].pos_y_exp);
	data_to_bd[i++]=BYTE1(calibration[5].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[5].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[5].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[5].pos_y_exp);
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}else
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==4)//航点确认 航点4
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x04;//功能字04 航点确认 航点7,8
	data_to_bd[i++]=BYTE1(calibration[6].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[6].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[6].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[6].pos_y_exp);
	data_to_bd[i++]=BYTE1(calibration[7].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[7].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[7].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[7].pos_y_exp);
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}else
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==5)//航点确认 航点5
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x05;//功能字05 航点确认 航点9,10
	data_to_bd[i++]=BYTE1(calibration[8].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[8].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[8].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[8].pos_y_exp);
	data_to_bd[i++]=BYTE1(calibration[9].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[9].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[9].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[9].pos_y_exp);
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}else
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==6)//航点确认 航点6
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x06;//功能字06 航点确认 航点11,12
	data_to_bd[i++]=BYTE1(calibration[10].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[10].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[10].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[10].pos_y_exp);
	data_to_bd[i++]=BYTE1(calibration[11].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[11].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[11].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[11].pos_y_exp);
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}else
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==7)//航点确认 航点7
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x07;//功能字07 航点确认 航点13,14
	data_to_bd[i++]=BYTE1(calibration[12].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[12].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[12].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[12].pos_y_exp);
	data_to_bd[i++]=BYTE1(calibration[13].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[13].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[13].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[13].pos_y_exp);
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}else
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==8)//航点确认 航点8
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x08;//功能字08 航点确认 航点15,16
	data_to_bd[i++]=BYTE1(calibration[14].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[14].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[14].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[14].pos_y_exp);
	data_to_bd[i++]=BYTE1(calibration[15].pos_x_exp);
	data_to_bd[i++]=BYTE0(calibration[15].pos_x_exp);
	data_to_bd[i++]=BYTE1(calibration[15].pos_y_exp);
	data_to_bd[i++]=BYTE0(calibration[15].pos_y_exp);
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}else
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==9)//基本飞行状态发送
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x09;//功能字09 基本飞行状态发送
	data_to_bd[i++]=BYTE1(LX_FC.LxFcState);
	data_to_bd[i++]=BYTE0(LX_FC.LxFcState);
	data_to_bd[i++]=BYTE1(LX_FC.bat_v100);
	data_to_bd[i++]=BYTE0(LX_FC.bat_v100);
	data_to_bd[i++]=BYTE1(LX_FC.time_s);
	data_to_bd[i++]=BYTE0(LX_FC.time_s);
	data_to_bd[i++]=BYTE1(taskmode);
	data_to_bd[i++]=BYTE0(taskmode);
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}else
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==10)//openmv数据发送
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x10;//功能字10 openmv数据发送
	data_to_bd[i++]=BYTE1(openmv.err_x);
	data_to_bd[i++]=BYTE0(openmv.err_x);
	data_to_bd[i++]=BYTE1(openmv.err_y);
	data_to_bd[i++]=BYTE0(openmv.err_y);
	data_to_bd[i++]=BYTE1(openmv.ifget);
	data_to_bd[i++]=BYTE0(openmv.ifget);
	data_to_bd[i++]=BYTE1(openmv.element);
	data_to_bd[i++]=BYTE0(openmv.element);
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}else
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==11)//下视镜头数据发送
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x11;//功能字11 下视镜头数据发送
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}else
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==12)//外设控制
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x12;//功能字12 外设控制数据发送
	data_to_bd[i++]=Servo[0];
	data_to_bd[i++]=lazer[0];
	data_to_bd[i++]=lazer[1];
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0;
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}
	////////////////////////////////////////
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==13)//外设控制
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x13;//功能字13
	data_to_bd[i++]=goods[1];
	data_to_bd[i++]=goods[2];
	data_to_bd[i++]=goods[3];
	data_to_bd[i++]=goods[4];
	data_to_bd[i++]=goods[5];
	data_to_bd[i++]=goods[6];
	data_to_bd[i++]=goods[7];
	data_to_bd[i++]=goods[8];
	data_to_bd[i++]=goods[0];
	data_to_bd[i++]=goods[25];
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}
	////////////////////////////////////////
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==14)//外设控制
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x14;//功能字14
	data_to_bd[i++]=goods[9];
	data_to_bd[i++]=goods[10];
	data_to_bd[i++]=goods[11];
	data_to_bd[i++]=goods[12];
	data_to_bd[i++]=goods[13];
	data_to_bd[i++]=goods[14];
	data_to_bd[i++]=goods[15];
	data_to_bd[i++]=goods[16];
	data_to_bd[i++]=goods[26];
	data_to_bd[i++]=goods[27];
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}
	////////////////////////////////////////
	////////////////////////////////////////
	//时刻注意这是飞控发给串口拓展板的代码
	if(mode==15)//外设控制
	{
	data_to_bd[i++]=0x15;//帧头15
	data_to_bd[i++]=0x79;//帧头79
	data_to_bd[i++]=0x78;//帧头78
	data_to_bd[i++]=0x15;//功能字15
	data_to_bd[i++]=goods[17];
	data_to_bd[i++]=goods[18];
	data_to_bd[i++]=goods[19];
	data_to_bd[i++]=goods[20];
	data_to_bd[i++]=goods[21];
	data_to_bd[i++]=goods[22];
	data_to_bd[i++]=goods[23];
	data_to_bd[i++]=goods[24];
	data_to_bd[i++]=goods[28];
	data_to_bd[i++]=goods[29];
	data_to_bd[i++]=0x05;//帧尾
	//触发一次发送
	DrvUart3SendBuf(data_to_bd,i);	
	}
	////////////////////////////////////////
	///////////////////////////////
	
}




