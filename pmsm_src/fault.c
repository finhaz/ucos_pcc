/*
 * fault.c
 *
 *
 */

#include "DSP28x_Project.h"

//-----------------------------------/快速故障判断---用于PWM中断判断进行过流保护，过压保护，欠压保护策略
//过流保护
//#define vmax_current 434//180  //325//271//180  //3*0.05*1.2k/(1.2k+174)*4096/3.3
//#define  //2470 //3087 //  360    1300 //120//     x/201*4096/3.3
//#define min_voltage 300

int32  vmax_current=744;
int32 max_voltage=2770;



//过流保护
void CurrentOverFault( )
{
    //U相过流判断
	if(ADCresultNEW[3]>vmax_current)//Paramet[current_max]由SCI通讯下发
	{
		//countover_Iu = countover_Iu+1 ;
		//if(countover_Iu>=2)
		//{
			overcurrent_U = 1 ;
			SYSTEMoff();
			//countover_Iu = 0 ;
		//}
	}
	else
	{
		overcurrent_U = 0 ;
		//countover_Iu = 0 ;
	}
	//W相过流判断
	if(ADCresultNEW[5] > vmax_current)
	{
		//countover_Iw = countover_Iw +1 ;
		//if(countover_Iw>=2)//防止误判
		//{
			overcurrent_W = 1 ;
			SYSTEMoff();
			//countover_Iw = 0;
		//}
	}
	else
	{
		overcurrent_W = 0 ;
		//countover_Iw = 0;
	}
	//V相过流判断
	if(ADCresultNEW[4] > vmax_current)
	{
		//countover_Iv = countover_Iv +1 ;
		//if(countover_Iv>=2)
		//{
			overcurrent_V = 1 ;
			SYSTEMoff();
			//countover_Iv = 0;
		//}
	}
	else
	{
		overcurrent_V = 0 ;
		//countover_Iv = 0;
	}
	/*
	//IDCLINK过流判断
	if(Adcget.Idc >3.3)
	{
		countover_IDCLINK = countover_IDCLINK +1 ;
		if(countover_IDCLINK>=2)
		{
			overcurrent_IDCLINK = 1 ;
			countover_IDCLINK = 0;
		}
	}
	else
	{
		overcurrent_IDCLINK = 0 ;
		countover_IDCLINK = 0;
	}
	*/
	//过流标志位
    if((overcurrent_U == 1)||(overcurrent_W == 1)||(overcurrent_V == 1))//||(overcurrent_IDCLINK == 1)
    {
    	FlagRegs.flagfault.bit.overcurrent = 1 ;
    	Paramet[overcurrentt]=1;
    }
    else
    {
    	FlagRegs.flagfault.bit.overcurrent = 0 ;
    }
}

//过压保护
void VoltageOverFault( )
{
	//过压判断
	if(ADCresultNEW[1]  >max_voltage)//过压判断Paramet[voltage_max]120Paramet[voltage_max]
	{
	  FlagRegs.flagfault.bit.highvoltage_dc = 1 ;
	 // Paramet[stop_moder]=1;
	  moder_of_stop=1;
	  SYSTEMoff();
	 // Paramet[overvoltagee]=1;
	}
	else
	{
	  FlagRegs.flagfault.bit.highvoltage_dc = 0 ;
	 // moder_of_stop=0;
	}
}

//欠压保护
/*
void VoltageUnderFault( )
{
	//欠压判断
	if(ADCresultNEW[1]<min_voltage)//低压判断Paramet[voltage_min]
	{
	 // FlagRegs.flagfault.bit.lowvoltage_dc = 1 ;
	 // Paramet[voltage_min]=1;
	}
	else
	{
		FlagRegs.flagfault.bit.lowvoltage_dc = 0 ;
	}
}
**/
//////快速故障保护
void FastFaultTest( )
{

	CurrentOverFault( );//过流保护
	VoltageOverFault( );//过压保护
	//VoltageUnderFault( );//欠压保护
	//故障码确认
	Paramet[flagfault_run] = FlagRegs.flagfault.all ;//显示
	//判断是否发生故障，即只要发生：过压，欠压，软件过流，硬件过流，过载中的任一故障，即确认故障发生。
	if(FlagRegs.flagfault.all>0)
	{
	    FlagRegs.flagsystem.bit.faultoccur = 1 ;//表示有错误发生，需要停机
	}
		else
	{
		FlagRegs.flagsystem.bit.faultoccur = 0 ;
	}
	Paramet[faultoccurr] = FlagRegs.flagsystem.bit.faultoccur;
}

void faultzero()
{
    //故障检测及复位
    //
	//countover_Iu=0;
	//countover_Iv=0;
	//countover_Iw=0;
    //
	overcurrent_U=0;
	overcurrent_V=0;
	overcurrent_W=0;


}


