/*==========================================================================
 * 描述    ：OPENMV数据处理
 * 更新时间：2023年4月21日
 * 作者		 ：	LHB
===========================================================================*/

#include "User_Opmv.h"
#include "User_Task.h"
#include "User_ComBoard.h"
/*接收数据缓冲*/
uint8_t openmv_buf[20];
uint8_t bd_to_op[20]={0};//飞控向OPENMV发送数据缓冲
uint8_t find_color=0;//寻找杆的颜色 0不找 1找红的杆子 2找绿的杆子
opmv_data openmv={0};

void MV_GetOneByte(uint8_t data)//OPENMV数据获取
{
	
	static uint8_t rec_sta = 0;

	openmv_buf[rec_sta] = data;
    
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
	}else if(rec_sta==9)
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
	else if(rec_sta==14)
	{
		if( data == 0X05 )/*正确的位置接收到帧尾，开始解析*/
        {           
						MV_DataAnl();
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
void MV_DataAnl(void)//OPENMV数据解析
{
	if(openmv_buf[3]==0x00)//起飞准备模式
	{
		//一键启动板发送起飞指令，飞控返回校验同时清除标志位，这是为了确保起飞指令确实收到了
		if((s16)((openmv_buf[4]<<8)|openmv_buf[5])==1)
		{
			LX_FC.fly_state=1;//标记等待解锁起飞
		}
//        taskmode=openmv_buf[6];
        
	}
}
void MV_DataSend(void)//上位机数据打包发送
{

	static int time=0;
    time++;
    time%=500;
    
    if(time==0)
    {
	bd_to_op[0]=0x15;//帧头15
	bd_to_op[1]=BYTE1(LX_FC.fly_state);//帧头79
	bd_to_op[2]=BYTE0(LX_FC.fly_state);//帧头78
	bd_to_op[3]=BYTE1(LX_FC.pos_x);
	bd_to_op[4]=BYTE0(LX_FC.pos_x);
	bd_to_op[5]=BYTE1(LX_FC.pos_y);
	bd_to_op[6]=BYTE0(LX_FC.pos_y);
	bd_to_op[7]=BYTE0(t265_data[0]);
	bd_to_op[8]=BYTE0(t265_data[1]);
	bd_to_op[9]=BYTE0(t265_data[2]);
	bd_to_op[10]=BYTE0(t265_data[3]);
	bd_to_op[11]=0x00;
	bd_to_op[12]=0x00;
	bd_to_op[13]=0x00;
	bd_to_op[14]=0x05;//帧尾

    DrvUart2SendBuf(bd_to_op,15);
    }
}

