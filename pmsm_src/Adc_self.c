/*
 * Adc_self.c
 *   ����
 */

#include "DSP28x_Project.h"


void  Adcread()
{
	   ADCresult[0]=AdcMirror.ADCRESULT0;//Ia
	   ADCresult[1]=AdcMirror.ADCRESULT1;//Ib
	   ADCresult[2]=AdcMirror.ADCRESULT2;//Ic
	   ADCresult[3]=AdcMirror.ADCRESULT3;//Ua
	   ADCresult[4]=AdcMirror.ADCRESULT4;//Ub
	   ADCresult[5]=AdcMirror.ADCRESULT5;//Uc
	   ADCresult[6]=AdcMirror.ADCRESULT6;//Ioa
	   ADCresult[7]=AdcMirror.ADCRESULT7;//Iob
	   ADCresult[8]=AdcMirror.ADCRESULT8;//Iob
	   ADCresult[9]=AdcMirror.ADCRESULT9;//Uoa
	   ADCresult[10]=AdcMirror.ADCRESULT10;//Uob
	   ADCresult[11]=AdcMirror.ADCRESULT11;//Uoc
}

//adc���ݴ���

void  Adcdeal()
{
	 //AD�����ļ��㼰У��
    if(FlagRegs.flagsystem.bit.AC0CheckFinished==1)  //1�������������Ҳ����ﵽҪ��ֵ
    {
        //Udc=ADCresultNEW[1]*26532/10;
        //IAD=I*0.025*4095/3
        //UDC=U*0.004*4095/3
    	//����Ҫ���ɵ��ĵ�ѹ�����ź�ͨ��ת���õ����뵽DSP���źţ�����ѹ��Χ���Ƶ�0~3V
    	//����ֱ��ƫ�õ���

        //��е�������Ia
     	   ADCresultNEW[0]= ADCresult[0]-AC_ZeroMean[0];//ע�⣺������������·ǰ��Ҫ��ƫ�����ﵽAC_ZeroMean[1]
    	   Adcget.Ia=ADCresultNEW[0]*ADC_I;

       //��е�������Ib
    	   ADCresultNEW[1]= ADCresult[1]-AC_ZeroMean[1];//ע�⣺������������·ǰ��Ҫ��ƫ�����ﵽAC_ZeroMean[1]
    	   Adcget.Ib=ADCresultNEW[1]*ADC_I;

       //��е�������Ic
    	   ADCresultNEW[2]= ADCresult[2]-AC_ZeroMean[2];//ע�⣺������������·ǰ��Ҫ��ƫ�����ﵽAC_ZeroMean[1]
    	   Adcget.Ic=ADCresultNEW[2]*ADC_I;

       //���ݵ�ѹ����Ua
    	   ADCresultNEW[3]= ADCresult[3]- AC_ZeroMean[3];//
    	   Adcget.Ua=ADCresultNEW[3]*ADC_U;

       //���ݵ�ѹ����Ub
    	   ADCresultNEW[4]= ADCresult[4]- AC_ZeroMean[4];//
    	   Adcget.Ub=ADCresultNEW[4]*ADC_U;

       //���ݵ�ѹ����Uc
    	  ADCresultNEW[5]= ADCresult[5]- AC_ZeroMean[5];//
   	      Adcget.Uc=ADCresultNEW[5]*ADC_U;

        //PCC����Ioa
       	  ADCresultNEW[6]= ADCresult[6]- AC_ZeroMean[6];//
   	      Adcget.Ioa=ADCresultNEW[6]*ADC_I;

        //PCC����Iob
       	  ADCresultNEW[7]= ADCresult[7]- AC_ZeroMean[7];//
   	      Adcget.Iob=ADCresultNEW[7]*ADC_I;

        //PCC����Ioc
       	  ADCresultNEW[8]= ADCresult[8]- AC_ZeroMean[8];//
   	      Adcget.Ioc=ADCresultNEW[8]*ADC_I;

        //PCC��ѹUoa
       	  ADCresultNEW[9]= ADCresult[9]- AC_ZeroMean[9];//
   	      Adcget.Uoa=ADCresultNEW[9]*ADC_U;

        //PCC��ѹUob
       	  ADCresultNEW[10]= ADCresult[10]- AC_ZeroMean[10];//
   	      Adcget.Uob=ADCresultNEW[10]*ADC_U;

        //PCC��ѹUoc
       	  ADCresultNEW[11]= ADCresult[11]- AC_ZeroMean[11];//
   	      Adcget.Uoc=ADCresultNEW[11]*ADC_U;


    }
    else
    {

       Uint16 i;
       Uint16 j;


       ADCZeroCNT0 +=1;
       for(i=0;i<12;i++)//12��������
        {
    	  AC_Zero[i][CounterRegs.count.bit.ADC_ZeroCNTloop] += ADCresult[i];
    	}
       if(  ADCZeroCNT0 == ADCZeroCNT0num )//600��ѭ��
    	{
    		ADCZeroCNT0 = 0;
    	    for(i=0;i<12;i++)
    		{
    	     //ȡ����
    		AC_Zero_yu[i][CounterRegs.count.bit.ADC_ZeroCNTloop] = AC_Zero[i][CounterRegs.count.bit.ADC_ZeroCNTloop] % ADCZeroCNT0num;
    		//��������
    		AC_Zero[i][CounterRegs.count.bit.ADC_ZeroCNTloop] = AC_Zero[i][CounterRegs.count.bit.ADC_ZeroCNTloop]/ ADCZeroCNT0num;
    		}
    		CounterRegs.count.bit.ADC_ZeroCNTloop += 1;
    	}

      if(CounterRegs.count.bit.ADC_ZeroCNTloop == ADCZeroCNTloopnum )	//8��ѭ��������һ��ѭ����200����
       {
            for(i=0;i<12;i++)
    	   {
            for(j=0;j<ADCZeroCNTloopnum ;j++)
    		{
              AC_Zero_yuz[i] =AC_Zero_yuz[i] + AC_Zero_yu[i][j];//�ۼ�����
    		  AC_ZeroMean[i] = AC_ZeroMean[i] + AC_Zero[i][j] ;//�ۼ�������
    		}
            AC_Zero_yuz[i] = AC_Zero_yuz[i]/ADCZeroCNT0num;
    		AC_ZeroMean[i] = (AC_ZeroMean[i]+AC_Zero_yuz[i])/ADCZeroCNTloopnum ;
    	}

     FlagRegs.flagsystem.bit.AC0CheckFinished = 1;
     CounterRegs.count.bit.ADC_ZeroCNTloop = 0;
       }
    //��ʾ
     // Paramet[pianzhi_U_run]= AC_ZeroMean[0];
     //Paramet[pianzhi_W_run]= AC_ZeroMean[1];


    }

}

void ADCzero()
{
    //ADC������ʼ��
	     Adcget.Ia=0;
	     Adcget.Ib=0;
	     Adcget.Ic=0;
	     Adcget.Ua=0;
	     Adcget.Ub=0;
	     Adcget.Uc=0;
	     Adcget.Ioa=0;
	     Adcget.Iob=0;
	     Adcget.Ioc=0;
	     Adcget.Uoa=0;
	     Adcget.Uob=0;
	     Adcget.Uoc=0;


	     CounterRegs.count.bit.ADC_ZeroCNTloop = 0;//У���������xѭ��
	     ADCZeroCNT0=0;//У�������������
	     int i , j ;
         for(i=0;i<12;i++)
        {
        	 ADCresultNEW[0]=0;
        	 ADCresultNEW[1]=0;
        	 ADCresultNEW[2]=0;
             ADCresultNEW[3]=0;
             ADCresultNEW[4]=0;
        	 ADCresultNEW[5]=0;
        	 ADCresultNEW[6]=0;
        	 ADCresultNEW[7]=0;
        	 ADCresultNEW[8]=0;
        	 ADCresultNEW[9]=0;
        	 ADCresultNEW[10]=0;
        	 ADCresultNEW[11]=0;

		        for(j=0;j<8;j++)
		       {
				      AC_Zero[i][j] = 0 ;
				      AC_Zero_yu[i][j] = 0 ;
		       }
        }
         for(i=0;i<12;i++)
	    {
		       AC_ZeroMean[i]=0;
		       AC_Zero_yuz[i]= 0 ;
	    }

}

