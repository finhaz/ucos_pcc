#include  <app_cfg.h>
#include  <ucos_ii.h>
#include  <cpu_core.h>
#include  <lib_def.h>
#include  <os_cpu.h>
#include "DSP28x_Project.h"

#define LED0        GpioDataRegs.GPBTOGGLE.bit.GPIO34
CPU_STK_SIZE  App_TaskStartStk[APP_CFG_TASK_STK_SIZE];
CPU_STK_SIZE  App_TaskPendStk[APP_CFG_TASK_STK_SIZE];
CPU_STK_SIZE  App_TaskPostStk[APP_CFG_TASK_STK_SIZE];

static  OS_EVENT    *AppTaskObjSem;

static  void  App_TaskStart(void  *p_arg);
static  void  App_TaskPing (void  *p_arg);
static  void  App_TaskPong (void  *p_arg);

interrupt void cpu_timer0_isr(void);  // �ж�����
void BSP_Tick_Init(void);

void BSP_Tick_Init(void)
{
    EALLOW;
    PieVectTable.TINT0 = &cpu_timer0_isr; // ��ʱ���жϵ�ַ
    PieVectTable.SCIRXINTC = &scirxintab_isr;//SCI�ж��ӳ����ַ
    PieVectTable.ADCINT=&adca1_interrupt_isr;//��׽�ж��ӳ����ַ
    PieVectTable.XINT1=&xint1_isr;//�ⲿ�ж�2018/1/3

    PieVectTable.OS_CPU_RTOSINT = &OS_CPU_RTOSINT_Handler;  // RTOS


    // MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    // InitFlash();  //��ʼ��FLASH
    EDIS;
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 150, 1000);  // test 500000
    CpuTimer0Regs.TCR.all = 0x4001; // ���� TSS bit = 0
    IER |=M_INT8;//SCI�ж�
    IER |= M_INT1;   // CPU-Timer 0����CPU INT1��ʹ��:

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;//ʹ��PIE��Adcint1���ж�
    //PieCtrlRegs.PIEIER9.bit.INTx1 = 1;//ʹ��PIE��Sci�����ж�
    PieCtrlRegs.PIEIER8.bit.INTx5 = 1;//ʹ��PIE��Sci�����ж�
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;//ʹ��PIE���ⲿ�ж�

    EINT;   // ʹ��INTM�ж�
    ERTM;   // ʹ��DBGM�ж�
}
void main(void)
{
   InitSysCtrl();
   // InitGpio();  // ����δ�õ�  // �ڶ�������ʼ��GPIO:
   DINT;
   InitPieCtrl();
   IER = 0x0000;
   IFR = 0x0000;
   InitPieVectTable();

   EALLOW;
   GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;  // LED0
   GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
   EDIS;

   InitUseGpio();
   ConfigureEPwm();
   ConfigureSci();
   ConfigureADC();
   ConfigureXint();
   Initparameter();

   EINT;   // ʹ��INTM�ж�
   ERTM;   // ʹ��DBGM�ж�

   OSInit();
   OSTaskCreateExt(App_TaskStart,
                   (void    *)0,
                   (CPU_STK *)&App_TaskStartStk[0],
                   (INT8U    )APP_CFG_TASK_START_PRIO,
                   (INT16U   )APP_CFG_TASK_START_PRIO,
                   (CPU_STK *)&App_TaskStartStk[APP_CFG_TASK_STK_SIZE - 1u],
                   (INT32U   )APP_CFG_TASK_STK_SIZE,
                   (void    *)0,
                   (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
   OSStart();
   while (DEF_TRUE) {
       ;
   }
}

static  void  App_TaskStart (void *p_arg)
{
    volatile CPU_INT08U  os_err;  // test
    (void)&p_arg;
    BSP_Tick_Init();
    AppTaskObjSem = OSSemCreate(0);

    OSTaskCreateExt(App_TaskPing,
                    (void    *)0,
                    (CPU_STK *)&App_TaskPendStk[0],
                    (INT8U    )APP_CFG_TASK_PEND_PRIO,
                    (INT16U   )APP_CFG_TASK_PEND_PRIO,
                    (CPU_STK *)&App_TaskPendStk[APP_CFG_TASK_STK_SIZE - 1u],
                    (INT32U   )APP_CFG_TASK_STK_SIZE,
                    (void    *)0,
                    (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    OSTaskCreateExt(App_TaskPong,
                    (void    *)0,
                    (CPU_STK *)&App_TaskPostStk[0],
                    (INT8U    )APP_CFG_TASK_POST_PRIO,
                    (INT16U   )APP_CFG_TASK_POST_PRIO,
                    (CPU_STK *)&App_TaskPostStk[APP_CFG_TASK_STK_SIZE - 1u],
                    (INT32U   )APP_CFG_TASK_STK_SIZE,
                    (void    *)0,
                    (INT16U   )(OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
    while (DEF_TRUE) {
        os_err = OSSemPost(AppTaskObjSem);
        OSTimeDlyHMSM(0, 0, 0, 1);
    }
}
static  void  App_TaskPing (void *p_arg)
{
    CPU_INT08U  os_err;
   (void)&p_arg;
    while (DEF_TRUE)
    {
        OSSemPend( AppTaskObjSem,
                   0,
                  &os_err);
    }
}
static  void  App_TaskPong (void *p_arg)
{
   (void)&p_arg;
    while (DEF_TRUE)
    {
        OSTimeDlyHMSM(0, 0, 0, 300);   // test   before is 300
        LED0=1;
        CpuTimer0.InterruptCount++;    // test  ���������
        Checkdata();//У��
        SciSend();//�յ�������
        Paramet[0]=Ua;//ֵ����λ��
        Paramet[1]=Ub;
        Paramet[2]=Uc;
        Paramet[3]=P;
        Paramet[4]=Q;
        Paramet[5]=w;
        Paramet[6]=theta_fan;
///////////////////////////////////////////////////////////��λ���·�����
        P0=Paramet[P_0];
        Q0=Paramet[Q_0];
        kp_current_dqp=Paramet[kp_I_p];
        ki_current_dqp=Paramet[ki_I_p];
        kp_current_dqn=Paramet[kp_I_n];
        ki_current_dqn=Paramet[ki_I_n];
        kp_voltage_dqp=Paramet[kp_u_p];
        ki_voltage_dqp=Paramet[ki_u_p];
        kp_voltage_dqn=Paramet[kp_u_n];
        ki_voltage_dqn=Paramet[ki_u_n];
        max_current=Paramet[PI_I_max];
        kp_pcc_degree=Paramet[kp_pcc];
        ki_pcc_degree=Paramet[ki_pcc];
        min_current=-max_current;

///////////////////////////////////////////////////////////

        //�Ƿ�ͣ��
        if( FlagRegs.flagsystem.bit.sysonoff == 0)//ͣ��
        {
            SYSTEMoff();
        }
        else
        {
            if(N_stage2==0)
            {
                PWMopen();
            }
            else
            {
                PWMoff();//����PWM
            }
        }
    }
}
interrupt void cpu_timer0_isr(void)
{
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
   // LED0=1;     // test
   // CpuTimer0.InterruptCount++;  //test
   OSIntEnter();
   OSTimeTick();
   OSIntExit();
}


//SCI�����жϷ����ӳ���
interrupt void scirxintab_isr(void)
{
	OSIntEnter();
    //�жϳ��򲿷�
    SciRecieve();
    //���жϱ�־λ
    //PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;//��PIEACK�Ĵ����ĵھ�λд1�������PIE�ھ�λ���ж�����
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;//��PIEACK�Ĵ����ĵڰ�λд1�������PIE�ڰ�λ���ж�����
    EINT;
    OSIntExit();
}
////////////////////////////////�о����Ӧ�ò��ö��ɡ���

//�ⲿ�ж�
interrupt void xint1_isr(void)
{
	OSIntEnter();
  Switchsystem=0;
  Paramet[ocfaultt]=1;//18
    //���жϱ�־λ
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
  OSIntExit();
}
////////////////////////////////�����Ҳû�ж�




interrupt void adca1_interrupt_isr(void)
{
	OSIntEnter();
    GpioDataRegs.GPADAT.bit.GPIO26 = 1;
    Adcread();//�����ӳ���
    Adcdeal();




    //--------------------------------
    //�Ƿ�У�����
     if(FlagRegs.flagsystem.bit.AC0CheckFinished==1)//����У���Ѿ����
     {
       //����
//       FastFaultTest();//������ѹ���ٹ��ϼ��
       //ͣ���ж�
//       if (FlagRegs.flagsystem.bit.faultoccur ==1) //�����ϣ���ͣ��������ͣ�������ȼ�����
//       {
//         Switchsystem = 0;//��ʾ���ش򿪣���Ҫͣ��
//         SYSTEMoff();//�ػ�
//       }
        FlagRegs.flagsystem.bit.sysonoff=Switchsystem;//��λ����ͣ������(��λ����Switchsystem����SCI�����д����)����
        //�Ƿ�ͣ��
        if(FlagRegs.flagsystem.bit.sysonoff==1)////������
        {

        //---------------------������-----------------------------------
        /////////////��������
//        	theta_fan=0;
        //------------------------------------��е�������任
        I_conversion.As=Adcget.Ia;
        I_conversion.Bs=Adcget.Ib;
        I_conversion.Cs=Adcget.Ic;
        I_conversion.Angle=theta_fan;
        abc_dq0p(&I_conversion);
        Idp=I_conversion.Ds;
        Iqp=I_conversion.Qs;
        I_conversion.Angle=-theta_fan;
        abc_dq0n(&I_conversion);
        Idn=I_conversion.Ds;
        Iqn=I_conversion.Qs;


        //------------------------------------������˿ڵ�ѹ
        U_conversion.As=Adcget.Ua;
        U_conversion.Bs=Adcget.Ub;
        U_conversion.Cs=Adcget.Uc;
        U_conversion.Angle=theta_fan;
        abc_dq0p(&U_conversion);
        Udp=U_conversion.Ds;
        Uqp=U_conversion.Qs;
        U_conversion.Angle=-theta_fan;
        abc_dq0n(&U_conversion);
        Udn=U_conversion.Ds;
        Uqn=U_conversion.Qs;

//        //--------------------------------------PCC����
//        Io_conversion.As=Adcget.Ioa;
//        Io_conversion.Bs=Adcget.Iob;
//        Io_conversion.Cs=Adcget.Ioc;
//        Io_conversion.Angle=theta_fan;
//        abc_dq0p(&Io_conversion);

        //--------------------------------------PCC��ѹ
        Uo_conversion.As=Adcget.Uoa;
        Uo_conversion.Bs=Adcget.Uob;
        Uo_conversion.Cs=Adcget.Uoc;
        Uo_conversion.Angle=theta_fan;
        abc_dq0p(&Uo_conversion);
        Uodp=Uo_conversion.Ds;
        Uoqp=Uo_conversion.Qs;
        Uo_conversion.Angle=-theta_fan;
        abc_dq0n(&Uo_conversion);
        Uodn=Uo_conversion.Ds;
        Uoqn=Uo_conversion.Qs;

        droop();
        neiwaihuan();


        //---------------------SPWM����------------------------------------

        //////////////����������
//        Ua=311;
//        Ub=-311/2;
//        Uc=-311/2;
//        Uout_conversion.As=0;
//        Uout_conversion.Bs=0;
//        Uout_conversion.Cs=0;
//        Uoutn_conversion.As=0;
//        Uoutn_conversion.Bs=0;
//        Uoutn_conversion.Cs=0;
        /////////////
        Ua=311*cos(theta_fan)+Uout_conversion.As;
        Ub=311*cos(theta_fan-TWObyTHREE*PI)+Uout_conversion.Bs;
        Uc=311*cos(theta_fan+TWObyTHREE*PI)+Uout_conversion.Cs;
//        Ua=311*cos(theta_fan)+Uout_conversion.As+Uoutn_conversion.As;
//        Ub=311*cos(theta_fan-TWObyTHREE*PI)+Uout_conversion.Bs+Uoutn_conversion.Bs;
//        Uc=311*cos(theta_fan+TWObyTHREE*PI)+Uout_conversion.Cs+Uoutn_conversion.Cs;

//-----------------------------------------
        a_graph[n_graph]=Ua;
        b_graph[n_graph]=Ub;
        c_graph[n_graph]=Uc;
//        a_graph[n_graph]=theta_fan;
        n_graph++;
        if(n_graph==graphNumber)
        {

        	n_graph=0;
        }

        //
        if(Ua>M)//M
        {
            Ua=M;
        }
        if(Ua<-M)
        {
            Ua=-M;
        }
        //
        if(Ub>M)
        {
            Ub=M;
        }

        if(Ub<-M)
        {
            Ub=-M;
        }
        //
        if(Uc>M)
        {
            Uc=M;
        }

        if(Uc<-M)
        {
            Uc=-M;
        }
        m_sin_a=Ua/500;
        m_sin_b=Ub/500;
        m_sin_c=Uc/500;

//        Tcmpa=3750*(1-m_sin_a);
//        Tcmpb=3750*(1-m_sin_b);
//        Tcmpc=3750*(1-m_sin_c);
        //T/4

        //5kHz
        Tcmpa=7500*(1-m_sin_a);
        Tcmpb=7500*(1-m_sin_b);
        Tcmpc=7500*(1-m_sin_c);


        EPwm1Regs.CMPA.half.CMPA=Tcmpa;
        EPwm2Regs.CMPA.half.CMPA=Tcmpb;
        EPwm3Regs.CMPA.half.CMPA=Tcmpc;

//			if(n_count1==vn_comp)
//			{
//				n_pso++;
//				if(n_pso==200)
//				{
//					n_pso=0;
//					PSOSENDF=1;
//					PSOsend(pso_t);
//				}
//			}

        }
        else
        {
            SYSTEMoff();//�ػ�
        }
    }

     // Reinitialize for next ADC sequence
    AdcRegs.ADCTRL2.bit.RST_SEQ1=1;
    AdcRegs.ADCST.bit.INT_SEQ1_CLR=1;//make sure INT1 flag is cleared
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    GpioDataRegs.GPADAT.bit.GPIO26 = 0;//�����ʱ��
    OSIntExit();
}
