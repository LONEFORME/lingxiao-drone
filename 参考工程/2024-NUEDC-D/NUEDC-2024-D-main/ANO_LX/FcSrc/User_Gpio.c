/*==========================================================================
 * 描述    ：板载GPIO
 * 更新时间：2023年7月31日
 * 作者		 ：	LHB
======javascript:;=====================================================================*/

#include "User_Task.h"
#include "Drv_RcIn.h"
#include "LX_FC_Fun.h"
#include "User_ComBoard.h"
#include "User_Opmv.h"
#include "User_T265.h"
#include "User_Gpio.h"

			
void User_GpioInit(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_GPIOPinTypeGPIOInput(GPIOB_BASE, GPIO_PIN_0);
	ROM_GPIOPinTypeGPIOOutput(GPIOB_BASE, GPIO_PIN_1);
    
}

void User_GpioCheck(void)
{
    static int time1=0,flag=0;
    
    if(ROM_GPIOPinRead(GPIOB_BASE,GPIO_PIN_0) == 0&&flag==0)
    {
         flag=1;
    }
    
    if(flag==1)
    {
        time1++;
        if(time1==2000)
        {
          time1=0;
          flag=0;
          LX_FC.fly_state=1;
        }
    }
    
    if(flag==1)
    {
        ROM_GPIOPinWrite(GPIOB_BASE,GPIO_PIN_1,GPIO_PIN_1);
    }else
   {
       ROM_GPIOPinWrite(GPIOB_BASE,GPIO_PIN_1,0);
   }
}
