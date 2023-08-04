/*
 * Vector_control.c
 *
 */

#include "DSP28x_Project.h"


/*////////////////////////////////////////////////////////////////////////////////////////////////////////
 * ----------------------坐标变换部分------------------------------------------------------------------------
 */

//正序abc to dq
void abc_dq0p(ABC_PARK *p)
{

//	 p->Sine = _IQsin(p->Angle);
//	 p->Cosine = _IQcos(p->Angle);
//
//
//	 p->Ds =_IQmpy(p->Alpha,p->Cosine) + _IQmpy(p->Beta,p->Sine);
//	 p->Qs =_IQmpy(p->Beta,p->Cosine) - _IQmpy(p->Alpha,p->Sine);

     p->Sine0 = sin(p->Angle);
     p->Sine1 = sin(p->Angle-TWObyTHREE*PI);
     p->Sine2 = sin(p->Angle+TWObyTHREE*PI);
	 p->Cosine0 = cos(p->Angle);
	 p->Cosine1 = cos(p->Angle-TWObyTHREE*PI);
	 p->Cosine2 = cos(p->Angle+TWObyTHREE*PI);

	 p->Ds =TWObyTHREE*(p->As*p->Cosine0+p->Bs*p->Cosine1+p->Cs*p->Cosine2);
	 p->Qs =TWObyTHREE*(p->As*p->Sine0+p->Bs*p->Sine1+p->Cs*p->Sine2);
	 p->V0 =ONEbyTHREE*(p->As+p->Bs+p->Cs);

}
//正序dq to abc
void iabc_dq0p(ABC_PARK *p)
{

//	 p->Sine = _IQsin(p->Angle);
//	 p->Cosine = _IQcos(p->Angle);
//
//
//	 p->Ds =_IQmpy(p->Alpha,p->Cosine) + _IQmpy(p->Beta,p->Sine);
//	 p->Qs =_IQmpy(p->Beta,p->Cosine) - _IQmpy(p->Alpha,p->Sine);

     p->Sine0 = sin(p->Angle);
     p->Sine1 = sin(p->Angle-TWObyTHREE*PI);
     p->Sine2 = sin(p->Angle+TWObyTHREE*PI);
	 p->Cosine0 = cos(p->Angle);
	 p->Cosine1 = cos(p->Angle-TWObyTHREE*PI);
	 p->Cosine2 = cos(p->Angle+TWObyTHREE*PI);

	 p->As =p->Ds*p->Cosine0+p->Qs*p->Sine0+p->V0;
	 p->Bs =p->Ds*p->Cosine1+p->Qs*p->Sine1+p->V0;
	 p->Cs =p->Ds*p->Cosine2+p->Qs*p->Sine2+p->V0;

}
//负序abc to dq
void abc_dq0n(ABC_PARK *p)
{

//	 p->Sine = _IQsin(p->Angle);
//	 p->Cosine = _IQcos(p->Angle);
//
//
//	 p->Ds =_IQmpy(p->Alpha,p->Cosine) + _IQmpy(p->Beta,p->Sine);
//	 p->Qs =_IQmpy(p->Beta,p->Cosine) - _IQmpy(p->Alpha,p->Sine);

     p->Sine0 = sin(p->Angle);
     p->Sine1 = sin(p->Angle-TWObyTHREE*PI);
     p->Sine2 = sin(p->Angle+TWObyTHREE*PI);
	 p->Cosine0 = cos(p->Angle);
	 p->Cosine1 = cos(p->Angle-TWObyTHREE*PI);
	 p->Cosine2 = cos(p->Angle+TWObyTHREE*PI);

	 p->Ds =TWObyTHREE*(p->As*p->Cosine0+p->Bs*p->Cosine1+p->Cs*p->Cosine2);
	 p->Qs =-TWObyTHREE*(p->As*p->Sine0+p->Bs*p->Sine1+p->Cs*p->Sine2);
	 p->V0 =ONEbyTHREE*(p->As+p->Bs+p->Cs);

}
//负序dq to abc
void iabc_dq0n(ABC_PARK *p)
{

//	 p->Sine = _IQsin(p->Angle);
//	 p->Cosine = _IQcos(p->Angle);
//
//
//	 p->Ds =_IQmpy(p->Alpha,p->Cosine) + _IQmpy(p->Beta,p->Sine);
//	 p->Qs =_IQmpy(p->Beta,p->Cosine) - _IQmpy(p->Alpha,p->Sine);

     p->Sine0 = sin(p->Angle);
     p->Sine1 = sin(p->Angle-TWObyTHREE*PI);
     p->Sine2 = sin(p->Angle+TWObyTHREE*PI);
	 p->Cosine0 = cos(p->Angle);
	 p->Cosine1 = cos(p->Angle-TWObyTHREE*PI);
	 p->Cosine2 = cos(p->Angle+TWObyTHREE*PI);

	 p->As =p->Ds*p->Cosine0-p->Qs*p->Sine0+p->V0;
	 p->Bs =p->Ds*p->Cosine1-p->Qs*p->Sine1+p->V0;
	 p->Cs =p->Ds*p->Cosine2-p->Qs*p->Sine2+p->V0;

}
//
//---------------------------------pi---------------------------------------------------------------
void PI_CONTROL_CALC( PI_CONTROL*p)
{
   //计算误差
   //Error  = Reference - Measurement
    p->currentError =p->qInRef-p->qInMeas;// pParm->qInRef - pParm->qInMeas;
    //计算积分
    //U = Ki * Err
    //U =  _IQmpy(currentError ,p->qKi) ;
    p->U = p->currentError  *p->qKi;
    //p->U=_IQmpy(p->currentError ,p->qKi);

	//Sum = Sum + U = Sum + Ki * Err - Kc * Exc
    p->qdSum=p->qdSum+p->U;// sum = sum + U;累加积分值
   //积分限幅
    if(p->qdSum>p->qOutMax)
       p->qdSum=p->qOutMax;
    if(p->qdSum<p->qOutMin)
       p->qdSum=p->qOutMin;
      //计算比例
    //U  = Sum + Kp * Error
    //U =   _IQmpy(currentError ,p->qKp) ;
    p->U = p->currentError *p->qKp;
    //p->U=_IQmpy( p->currentError,p->qKp);
    p->U = p->qdSum+p->U; // U = U + sum;
    //输出限幅
    if(  p->U >p->qOutMax)
    	p->qOut=p->qOutMax;

    else if( p->U <p->qOutMin)
    	p->qOut=p->qOutMin;
    else
    	p->qOut=p->U;
}

//------------------------------------------------滤波器
void FILTRATE_CALC(FILTRATE*p)
{
//	X_last1=X_last;
//	X_last=X;
//	X=X_in;
//	Y_last1=Y_last;
//	Y_last=Y;
//	Y=a1*X+a2*X_last+a3*X_last1-b1*Y_last-b2*Y_last1;
//	float tmp1,tmp2,tmp3,tmp4,tmp5;


	p->X_last1=p->X_last;
	p->X_last=p->X;
	p->X=p->X_in;
	p->Y_last1=p->Y_last;
	p->Y_last=p->Y;

	p->Y=p->a1*p->X+p->a2*p->X_last+p->a3*p->X_last1-p->b1*p->Y_last-p->b2*p->Y_last1;

}

//------------------------------------------------解耦
void JIEOU_CALC(JIEOU*p)
{
//	Udout=Ud-Udmean*cos(2*Angle)-Uqmean*sin(2*Angle);
//  Uqout=Uq-Udmean*sin(2*Angle)+Uqmean*cos(2*Angle);

	p->Udout=p->Ud-p->Udmean*cos(2*p->Angle)-p->Uqmean*sin(2*p->Angle);
	p->Uqout=p->Uq-p->Udmean*sin(2*p->Angle)+p->Uqmean*cos(2*p->Angle);

}

//------------------------------------------------DDSRF_PLL
void DDSRF_PLL_CALC(DDSRF_PLL*p)
{
	p->Udpmean=p->Udp_filtrate.Y;
	p->Uqpmean=p->Uqp_filtrate.Y;
	p->Udnmean=p->Udn_filtrate.Y;
	p->Uqnmean=p->Uqn_filtrate.Y;

	jieou_positive.Ud=p->Udp;
	jieou_positive.Uq=p->Uqp;
	jieou_positive.Udmean=p->Udnmean;
	jieou_positive.Uqmean=p->Uqnmean;
	jieou_positive.Angle=theta_fan;
	JIEOU_CALC(&jieou_positive);
	p->Udp_filtrate.X_in=jieou_positive.Udout;
	FILTRATE_CALC(&p->Udp_filtrate);
	p->Uqp_filtrate.X_in=jieou_positive.Uqout;
	FILTRATE_CALC(&p->Uqp_filtrate);

	jieou_negative.Ud=p->Udn;
	jieou_negative.Uq=p->Uqn;
	jieou_negative.Udmean=p->Udpmean;
	jieou_negative.Uqmean=p->Uqpmean;
	jieou_negative.Angle=theta_fan;
	JIEOU_CALC(&jieou_negative);
	p->Udn_filtrate.X_in=jieou_negative.Udout;
	FILTRATE_CALC(&p->Udn_filtrate);
	p->Uqn_filtrate.X_in=jieou_negative.Uqout;
	FILTRATE_CALC(&p->Uqn_filtrate);

	p->Udpout=p->Udp_filtrate.Y;
	p->Uqpout=p->Uqp_filtrate.Y;
	p->Udnout=p->Udn_filtrate.Y;
	p->Uqnout=p->Uqn_filtrate.Y;

}


void Paramet_Init(void)//赋初值，在上位机没有给参数的时候就先这个参数开环跑
{

    Paramet[P_0]=0;
    Paramet[Q_0]=0;

    Paramet[kp_I_p]=0.15;//0.2
    Paramet[ki_I_p]=0;
    Paramet[kp_I_n]=1;
    Paramet[ki_I_n]=0;
    Paramet[kp_u_p]=0.002;
    Paramet[ki_u_p]=0.015;
    Paramet[kp_u_n]=0.45;
    Paramet[ki_u_n]=0.005;
    Paramet[kp_pcc]=0.1;
    Paramet[ki_pcc]=0.05;

    Paramet[PI_I_max]=1000;
}

void Initparameter(void)//其他的一些参数
{
         FlagRegs.flagsystem.bit.sysonoff = 0;//停机
         FlagRegs.flagfault.all = 0 ;//初始化时，0000设置为：故障发生、电机温度、电路板温度、踏板信号、直流欠压、直流过压、w过流、v过流、u过流
         FlagRegs.flagsystem.bit.faultoccur = 0 ;//故障未发生
         FlagRegs.flagsystem.bit.pre_dir_sign = 0 ;//先预定位
        // FlagRegs.flagsystem.bit.if_start = 1 ;//预定位计数未结束
         FlagRegs.flagsystem.bit.AC0CheckFinished=0;//采样校零未完成
         Switchsystem=0;//停机
         moder_of_stop=0;
         stage=0;//ceshi
      //系统参数数组初始化

         int i ;
         for(i=0;i<44;i++)
        {
            Paramet[i]=0;
        }
        ADCzero();
        faultzero();
        Paramet_Init();
       // PIZero();//
       // VectorControl_zero();
}


void PIZero()//PI调节器清零,主要是积分终值要清零，而dq变换那个函数是实时计算的就不用清零。
{
//电压d轴
PI_Ud.U=0;
PI_Ud.qInMeas=0;
PI_Ud.qInRef=0;
PI_Ud.qOut=0;
PI_Ud.qdSum=0;

//电压q轴
PI_Uq.U=0;
PI_Uq.qInMeas=0;
PI_Uq.qInRef=0;
PI_Uq.qdSum=0;
PI_Uq.qOut=0;
//电流d轴
PI_Id.U=0;
PI_Id.qInMeas=0;
PI_Id.qInRef=0;
PI_Id.qdSum=0;
PI_Id.qOut=0;
//电流q轴
PI_Iq.U=0;
PI_Iq.qInRef=0;
PI_Iq.qInMeas=0;
PI_Iq.qdSum=0;
PI_Iq.qOut=0;
//----------------------负序
//电压d轴
PI_Udn.U=0;
PI_Udn.qInMeas=0;
PI_Udn.qInRef=0;
PI_Udn.qOut=0;
PI_Udn.qdSum=0;

//电压q轴
PI_Uqn.U=0;
PI_Uqn.qInMeas=0;
PI_Uqn.qInRef=0;
PI_Uqn.qdSum=0;
PI_Uqn.qOut=0;
//电流d轴
PI_Idn.U=0;
PI_Idn.qInMeas=0;
PI_Idn.qInRef=0;
PI_Idn.qdSum=0;
PI_Idn.qOut=0;
//电流q轴
PI_Iqn.U=0;
PI_Iqn.qInRef=0;
PI_Iqn.qInMeas=0;
PI_Iqn.qdSum=0;
PI_Iqn.qOut=0;
//PCC
PI_PCC.U=0;
PI_PCC.qInRef=0;
PI_PCC.qInMeas=0;
PI_PCC.qdSum=0;
PI_PCC.qOut=0;

}

//归零初始化
void VectorControl_zero()
{


	//Ki=163;
	//Kp=16384;

	stage=0;//ceshi


	FlagRegs.flagsystem.bit.pre_dir_sign=0;
	FlagRegs.flagsystem.bit.if_start=0;


	P=0;
	Q=0;

//	 Ud=0;
//	 Uq=0;
//	 Id=0;
//	 Iq=0;
	Idp=0;
	Iqp=0;
	Idn=0;
	Iqn=0;

	Udp=0;
	Uqp=0;
	Udn=0;
	Uqn=0;

	Uodp=0;
	Uoqp=0;
	Uodn=0;
	Uoqn=0;

	Idpout=0;
	Iqpout=0;
	Idnout=0;
	Iqnout=0;

	Udpout=0;
	Uqpout=0;
	Udnout=0;
	Uqnout=0;

	Uodpout=0;
	Uoqpout=0;
	Uodnout=0;
	Uoqnout=0;

	 theta_fan=0;
	VUFout=0;
	VUFpcc=0;
	VUFpccmean=0;
	 gain=0;

     Ua_ref=0;
     Ub_ref=0;
     Uc_ref=0;

 	Ud_ref=0;
 	Uq_ref=0;
//    delt_Ud=0;
//    delt_Uq=0;
    Id_ref=0;
    Iq_ref=0;
//    delt_Id=0;
//    delt_Iq=0;

    n_count=0;
    n_count1=0;

	 N_stage=0;
	 N_stage1=0;
	 N_stage2=0;

	 Udn_ref=0;
	 Uqn_ref=0;

//	 Udp_filtrate.X=0;
//	 Udp_filtrate.X_last=0;
//	 Udp_filtrate.X_last1=0;
//	 Udp_filtrate.X_in=0;
//	 Udp_filtrate.Y=0;
//	 Udp_filtrate.Y_last=0;
//	 Udp_filtrate.Y_last1=0;
//
//	 Uqp_filtrate.X=0;
//	 Uqp_filtrate.X_last=0;
//	 Uqp_filtrate.X_last1=0;
//	 Uqp_filtrate.X_in=0;
//	 Uqp_filtrate.Y=0;
//	 Uqp_filtrate.Y_last=0;
//	 Uqp_filtrate.Y_last1=0;
//
//	 Udn_filtrate.X=0;
//	 Udn_filtrate.X_last=0;
//	 Udn_filtrate.X_last1=0;
//	 Udn_filtrate.X_in=0;
//	 Udn_filtrate.Y=0;
//	 Udn_filtrate.Y_last=0;
//	 Udn_filtrate.Y_last1=0;
//
//	 Uqn_filtrate.X=0;
//	 Uqn_filtrate.X_last=0;
//	 Uqn_filtrate.X_last1=0;
//	 Uqn_filtrate.X_in=0;
//	 Uqn_filtrate.Y=0;
//	 Uqn_filtrate.Y_last=0;
//	 Uqn_filtrate.Y_last1=0;



//	 test_filtrate.X=0;
//	 test_filtrate.X_last=0;
//	 test_filtrate.X_last1=0;
//	 test_filtrate.X_in=0;
//	 test_filtrate.Y=0;
//	 test_filtrate.Y_last=0;
//	 test_filtrate.Y_last1=0;

//ceshi=0.000000003947490990434116;
}

//--------------------下垂控制---------------------------------------
void droop()
{
	//////////////////
//	Ud=311;
//	Uq=0;

	////////////////////////开环测试
//
//	Id=20;
//	Iq=-2;

	////////////////////////////
      I_DDSRF_PLL.Udp=Idp;
      I_DDSRF_PLL.Uqp=Iqp;
      I_DDSRF_PLL.Udn=Idn;
      I_DDSRF_PLL.Uqn=Iqn;
      I_DDSRF_PLL.Angle=theta_fan;
      DDSRF_PLL_CALC(&I_DDSRF_PLL);
      Idpout=I_DDSRF_PLL.Udpout;
      Iqpout=I_DDSRF_PLL.Uqpout;
      Idnout=I_DDSRF_PLL.Udnout;
      Iqnout=I_DDSRF_PLL.Uqnout;

      pso_t[0]=Idnout;
      pso_t[1]=Iqnout;
      pso_t[2]=0;
      pso_t[3]=0;



      U_DDSRF_PLL.Udp=Udp;
      U_DDSRF_PLL.Uqp=Uqp;
      U_DDSRF_PLL.Udn=Udn;
      U_DDSRF_PLL.Uqn=Uqn;
      U_DDSRF_PLL.Angle=theta_fan;
      DDSRF_PLL_CALC(&U_DDSRF_PLL);
      Udpout=U_DDSRF_PLL.Udpout;
      Uqpout=U_DDSRF_PLL.Uqpout;
      Udnout=U_DDSRF_PLL.Udnout;
      Uqnout=U_DDSRF_PLL.Uqnout;

      Uo_DDSRF_PLL.Udp=Uodp;
      Uo_DDSRF_PLL.Uqp=Uoqp;
      Uo_DDSRF_PLL.Udn=Uodn;
      Uo_DDSRF_PLL.Uqn=Uoqn;
      Uo_DDSRF_PLL.Angle=theta_fan;
      DDSRF_PLL_CALC(&Uo_DDSRF_PLL);
      Uodpout=Uo_DDSRF_PLL.Udpout;
      Uoqpout=Uo_DDSRF_PLL.Uqpout;
      Uodnout=Uo_DDSRF_PLL.Udnout;
      Uoqnout=Uo_DDSRF_PLL.Uqnout;
//-------------------------------------
//----------------------------------------计算不平衡度
      if(n_count<5)
      //      if(n_count<10)
      {
    	  VUFout=0;
    	  VUFpcc=0;
    	  n_count++;
      }
      else
      {
    	  VUFout=sqrt(Udnout*Udnout+Uqnout*Uqnout)/sqrt(Udpout*Udpout+Uqpout*Uqpout);
    	  VUFpcc=sqrt(Uodnout*Uodnout+Uoqnout*Uoqnout)/sqrt(Uodpout*Uodpout+Uoqpout*Uoqpout);
      }

//--------------------------------------------droop
      P= 1.5*(Udpout*Idpout+Uqpout*Iqpout);
      Q= -1.5*(Uqpout*Idpout-Udpout*Iqpout);
      P_last=0.99*P_last+0.01*P;
      Q_last=0.99*Q_last+0.01*Q;

//            P_last=oneP_filtrate.Y;
//            Q_last=oneQ_filtrate.Y;

          w=w0-m*(P_last-P0);
          /////////开环测试
      //    w=314;

          U=U0-n*(Q_last-Q0);

 }

//--------------------电流环电压环---------------------------------------
void neiwaihuan()
{
	pso_t[4]=w;
	gain=w*T;
	theta_fan=theta_fan+gain;
	if(theta_fan>=twopi)       //取余
		 {
		theta_fan=theta_fan-twopi;
		 }

//下垂控制得到的参考电压
    Ua_ref=U*cos(theta_fan);
    Ub_ref=U*cos(theta_fan-TWObyTHREE*PI);
    Uc_ref=U*cos(theta_fan+TWObyTHREE*PI);
    Uref_conversion.As=Ua_ref;
    Uref_conversion.Bs=Ub_ref;
    Uref_conversion.Cs=Uc_ref;
    Uref_conversion.Angle=theta_fan;
    abc_dq0p(&Uref_conversion);
//------------------电压环--------------------正序

    Ud_ref=Uref_conversion.Ds;
    Uq_ref=Uref_conversion.Qs;

//    delt_Ud=Ud_ref-Udpout;
//    delt_Uq=Uq_ref-Uqpout;

//------------------正序电压环PI--------------------
    //给定d轴调节
        PI_Ud.qKp=kp_voltage_dqp;
        PI_Ud.qKi=ki_voltage_dqp;
        PI_Ud.qOutMax=max_current;
        PI_Ud.qOutMin=min_current;

    //给定q轴调节
        PI_Uq.qKp=kp_voltage_dqp;
        PI_Uq.qKi=ki_voltage_dqp;
        PI_Uq.qOutMax=max_current;
        PI_Uq.qOutMin=min_current;
   //d轴调节
     PI_Ud.qInRef=Ud_ref;
     PI_Ud.qInMeas=Udpout;
     PI_CONTROL_CALC(&PI_Ud);//输出d轴调制电压Ud
   //q轴调节
     PI_Uq.qInRef=Uq_ref;
     PI_Uq.qInMeas=Uqpout;//
   	 PI_CONTROL_CALC(&PI_Uq);//输出q轴调制电压Uq

   //Ud,Uq可以用于弱磁，也可以用于保存本次的PI输出值
   	 PIout_Ud=PI_Ud.qOut;
   	 PIout_Uq=PI_Uq.qOut;

//------------------电流环--------------------正序
     Id_ref=PIout_Ud;
     Iq_ref=PIout_Uq;

//     delt_Id=Id_ref-Idpout;
//     delt_Iq=Iq_ref-Iqpout;

//------------------正序电流环PI--------------------
    //给定d轴调节
        PI_Id.qKp=kp_current_dqp;
        PI_Id.qKi=ki_current_dqp;
        PI_Id.qOutMax=max_current;
        PI_Id.qOutMin=min_current;

    //给定q轴调节
        PI_Iq.qKp=kp_current_dqp;
        PI_Iq.qKi=ki_current_dqp;
        PI_Iq.qOutMax=max_current;
        PI_Iq.qOutMin=min_current;

    //d轴调节
      PI_Id.qInRef=Id_ref;
      PI_Id.qInMeas=Idpout;
      PI_CONTROL_CALC(&PI_Id);//输出q轴调制电压Uq
    //q轴调节
      PI_Iq.qInRef=Iq_ref;
      PI_Iq.qInMeas=Iqpout;
      PI_CONTROL_CALC(&PI_Iq);//输出d轴调制电压Ud

      PIout_Id=PI_Id.qOut;
      PIout_Iq=PI_Iq.qOut;


//------------------电压环--------------------补负序
      if(n_count1<vn_comp)
      //if(n_count1<3500)
      {
//    	  delt_Udn=0;
//    	  delt_Uqn=0;
    	  PI_Udn.qInMeas=0;
    	  PI_Udn.qInRef=0;
    	  PI_Uqn.qInMeas=0;
    	  PI_Uqn.qInRef=0;
    	  n_count1++;
      }
      else
      {
    	  Udn_ref=PSO_g[0];
    	  Uqn_ref=PSO_g[1];
    	  PI_Udn.qInMeas=Udnout;
    	  PI_Udn.qInRef=Udn_ref;
    	  PI_Uqn.qInMeas=Uqnout;
    	  PI_Uqn.qInRef=Uqn_ref;
//    	  delt_Udn=Udn_ref-Udnout;
//    	  delt_Uqn=Uqn_ref-Uqnout;
      }
//------------------负序电压环PI--------------------
  //给定d轴调节
	  PI_Udn.qKp=kp_voltage_dqn;
	  PI_Udn.qKi=ki_voltage_dqn;
	  PI_Udn.qOutMax=max_current;
	  PI_Udn.qOutMin=min_current;

  //给定q轴调节
	  PI_Uqn.qKp=kp_voltage_dqn;
	  PI_Uqn.qKi=ki_voltage_dqn;
	  PI_Uqn.qOutMax=max_current;
	  PI_Uqn.qOutMin=min_current;
 //d轴调节
//      PI_Udn.qInMeas=delt_Udn;
//	  PI_Udn.qInMeas=Udnout;
//	  PI_Udn.qInRef=Udn_ref;
      PI_CONTROL_CALC(&PI_Udn);//输出d轴调制电压Ud
 //q轴调节
//      PI_Uqn.qInMeas=delt_Uqn;//
//	  PI_Uqn.qInMeas=Uqnout;
//	  PI_Uqn.qInRef=Uqn_ref;
	  PI_CONTROL_CALC(&PI_Uqn);//输出q轴调制电压Uq

	   PIout_Udn=PI_Udn.qOut;
	   PIout_Uqn=PI_Uqn.qOut;
//------------------电流环--------------------补负序
//	  Idn_ref=0;
//	  Iqn_ref=0;
	  Idn_ref=PIout_Udn;
	  Iqn_ref=PIout_Uqn;

//	  delt_Idn=Idn_ref-Idnout;
//	  delt_Iqn=Iqn_ref-Iqnout;

//------------------负序电流环PI--------------------
   //给定d轴调节
	   PI_Idn.qKp=kp_current_dqn;
	   PI_Idn.qKi=ki_current_dqn;
	   PI_Idn.qOutMax=max_current;
	   PI_Idn.qOutMin=min_current;

   //给定q轴调节
	   PI_Iqn.qKp=kp_current_dqn;
	   PI_Iqn.qKi=ki_current_dqn;
	   PI_Iqn.qOutMax=max_current;
	   PI_Iqn.qOutMin=min_current;

   //d轴调节
	   PI_Idn.qInMeas=Idnout;
	   PI_Idn.qInRef=Idn_ref;
	   PI_CONTROL_CALC(&PI_Idn);//输出q轴调制电压Uq
   //q轴调节
	   PI_Iqn.qInMeas=Iqnout;
	   PI_Iqn.qInRef=Iqn_ref;
	   PI_CONTROL_CALC(&PI_Iqn);//输出d轴调制电压Ud

	   PIout_Idn=PI_Idn.qOut;
	   PIout_Iqn=PI_Iqn.qOut;
//-----------------------------------------------------PI环到这结束

  	//下面的都是开环测试
//      Uout_conversion.Ds=311;
//      Uout_conversion.Qs=0;

//      test_conversion.As=311*cos(theta_fan);
//      test_conversion.Bs=311*cos(theta_fan+TWObyTHREE*PI);
//      test_conversion.Cs=311*cos(theta_fan-TWObyTHREE*PI);
//      test_conversion.Angle=-theta_fan;
//      abc_dq0n(&test_conversion);

//      test_conversion.Ds=311;
//      test_conversion.Qs=0;
//      test_conversion.Angle=theta_fan;
//      iabc_dq0p(&test_conversion);
//      test_conversion.As=311*cos(theta_fan);

      ///////////////////滤波器测试

//      test_filtrate.X_in=100+10*cos(theta_fan)+5*cos(3*theta_fan)+50*cos(6*theta_fan);
////      U_filtrate.a1=0.000039130205399144361;//给X
////      U_filtrate.a2=0.000078260410798288723;//给X_last
////      U_filtrate.a3=0.000039130205399144361;//给X_last1
////      U_filtrate.b1=-1.9822289297925286;//给Y_last
////      U_filtrate.b2=0.9823854506141253;
//      FILTRATE_CALC(&test_filtrate);
//      test_Uout=test_filtrate.Y;

      ///////////////////DDSRF_PLL测试
//      test_conversion.As=311*cos(theta_fan)+5*cos(theta_fan);
//      test_conversion.Bs=311*cos(theta_fan-TWObyTHREE*PI)+5*cos(theta_fan+TWObyTHREE*PI);
//      test_conversion.Cs=311*cos(theta_fan+TWObyTHREE*PI)+5*cos(theta_fan-TWObyTHREE*PI);
//      test_conversion.Angle=theta_fan;
//      abc_dq0p(&test_conversion);
//
//      test2_conversion.As=311*cos(theta_fan)+5*cos(theta_fan);
//      test2_conversion.Bs=311*cos(theta_fan-TWObyTHREE*PI)+5*cos(theta_fan+TWObyTHREE*PI);
//      test2_conversion.Cs=311*cos(theta_fan+TWObyTHREE*PI)+5*cos(theta_fan-TWObyTHREE*PI);
//      test2_conversion.Angle=-theta_fan;
//      abc_dq0n(&test2_conversion);
//
//      test_DDSRF_PLL.Udp=test_conversion.Ds;
//      test_DDSRF_PLL.Uqp=test_conversion.Qs;
//      test_DDSRF_PLL.Udn=test2_conversion.Ds;
//      test_DDSRF_PLL.Uqn=test2_conversion.Qs;
//      test_DDSRF_PLL.Angle=theta_fan;
//      DDSRF_PLL_CALC(&test_DDSRF_PLL);
//---------------------------------------------------存数据用于MATLAB画图
//      if(i_a<FilterNumber)
//      {
//		  a_filter[i_a]=test_DDSRF_PLL.Uqnout;
//		  i_a++;
//      }


//		a_filter[i_a]=test_conversion.As;
//		i_a++;
//		if(i_a==FilterNumber)
//		{
//			i_a=0;
//		}

//      int i=0;
//      for (i=0;i<100;i++)
//      {
//    	  a_filter[i_a+i]=a1_filter[i];
//      }
//      i_a=sizeof(a1_filter);
//----------------------------------------------------------------------


//////////////////////////////////////////////////////////////////////////////上面是测试部分
      Uout_conversion.Ds=PIout_Id;
      Uout_conversion.Qs=PIout_Iq;
      Uout_conversion.Angle=theta_fan;
	 iabc_dq0p(&Uout_conversion);//得到ua,ub,uc（正序的）
     Uoutn_conversion.Ds=PIout_Idn;
     Uoutn_conversion.Qs=PIout_Iqn;
     Uoutn_conversion.Angle=-theta_fan;
	 iabc_dq0n(&Uoutn_conversion);//得到ua,ub,uc（正序的）

 }


//void test_DDSRF_PLL_init(void)
//{
//	test_DDSRF_PLL.Udp=0;
//
//	test_DDSRF_PLL.Udn_filtrate.X=0;
//
//
//}
