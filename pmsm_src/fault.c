/*
 * fault.c
 *
 *
 */

#include "DSP28x_Project.h"

//-----------------------------------/���ٹ����ж�---����PWM�ж��жϽ��й�����������ѹ������Ƿѹ��������
//��������
//#define vmax_current 434//180  //325//271//180  //3*0.05*1.2k/(1.2k+174)*4096/3.3
//#define  //2470 //3087 //  360    1300 //120//     x/201*4096/3.3
//#define min_voltage 300

int32  vmax_current=744;
int32 max_voltage=2770;



//��������
void CurrentOverFault( )
{
    //U������ж�
	if(ADCresultNEW[3]>vmax_current)//Paramet[current_max]��SCIͨѶ�·�
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
	//W������ж�
	if(ADCresultNEW[5] > vmax_current)
	{
		//countover_Iw = countover_Iw +1 ;
		//if(countover_Iw>=2)//��ֹ����
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
	//V������ж�
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
	//IDCLINK�����ж�
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
	//������־λ
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

//��ѹ����
void VoltageOverFault( )
{
	//��ѹ�ж�
	if(ADCresultNEW[1]  >max_voltage)//��ѹ�ж�Paramet[voltage_max]120Paramet[voltage_max]
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

//Ƿѹ����
/*
void VoltageUnderFault( )
{
	//Ƿѹ�ж�
	if(ADCresultNEW[1]<min_voltage)//��ѹ�ж�Paramet[voltage_min]
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
//////���ٹ��ϱ���
void FastFaultTest( )
{

	CurrentOverFault( );//��������
	VoltageOverFault( );//��ѹ����
	//VoltageUnderFault( );//Ƿѹ����
	//������ȷ��
	Paramet[flagfault_run] = FlagRegs.flagfault.all ;//��ʾ
	//�ж��Ƿ������ϣ���ֻҪ��������ѹ��Ƿѹ�����������Ӳ�������������е���һ���ϣ���ȷ�Ϲ��Ϸ�����
	if(FlagRegs.flagfault.all>0)
	{
	    FlagRegs.flagsystem.bit.faultoccur = 1 ;//��ʾ�д���������Ҫͣ��
	}
		else
	{
		FlagRegs.flagsystem.bit.faultoccur = 0 ;
	}
	Paramet[faultoccurr] = FlagRegs.flagsystem.bit.faultoccur;
}

void faultzero()
{
    //���ϼ�⼰��λ
    //
	//countover_Iu=0;
	//countover_Iv=0;
	//countover_Iw=0;
    //
	overcurrent_U=0;
	overcurrent_V=0;
	overcurrent_W=0;


}


