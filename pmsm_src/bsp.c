/*
 *     bsp.c
 *
 *     所有相关的外设驱动初始化配置函数在此文件编写
 *
 *
 */
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File



void InitBoardGpio(void)
{
    //
    // Initialize GPIOs for the LEDs and turn them off
    //
    EALLOW;

    GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;
    GpioDataRegs.GPADAT.bit.GPIO26 = 0;

    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;
    GpioDataRegs.GPADAT.bit.GPIO27 = 0;

    EDIS;
}


void ConfigureADC(void)
{


	EALLOW;
	SysCtrlRegs.HISPCP.all = 0x3;//HSPCLK = SYSCLKOUT/2*HISPCP = 150/(2*3)   = 25.0 MHz
//	SysCtrlRegs.HISPCP.all = 0x4;//HSPCLK = SYSCLKOUT/2*HISPCP = 150/(2*4)   = 18.75 MHz
//	SysCtrlRegs.HISPCP.all = 0x7;//HSPCLK = SYSCLKOUT/2*HISPCP = 150/(2*7)   = 10.71 MHz
	EDIS;
	InitAdc();  // For this example, init the ADC

   //CHANNEL
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 0x1;//0-dual, 1-Cascaded
    AdcRegs.ADCMAXCONV.all = 0x000F;       // Setup 16 conv's on SEQ
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0; // Setup ADCINA0 as 1st SEQ conv.
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; // Setup ADCINA1 as 2nd SEQ conv.
    AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2; // Setup ADCINA2 as 3rd SEQ conv.
    AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3; // Setup ADCINA3 as 4th SEQ conv.
    AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4; // Setup ADCINA4 as 5th SEQ conv.
    AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5; // Setup ADCINA5 as 6th SEQ conv.
    AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6; // Setup ADCINA6 as 7th SEQ conv.
    AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x7; // Setup ADCINA7 as 8th SEQ conv.
    AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 0x8; // Setup ADCINB0 as 9th SEQ conv.
    AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 0x9; // Setup ADCINB1 as 10th SEQ conv.
    AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 0xA; // Setup ADCINB2 as 11th SEQ conv.
    AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 0xB; // Setup ADCINB3 as 12th SEQ conv.
    AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 0xC; // Setup ADCINB4 as 13th SEQ conv.
    AdcRegs.ADCCHSELSEQ4.bit.CONV13 = 0xD; // Setup ADCINB5 as 14th SEQ conv.
    AdcRegs.ADCCHSELSEQ4.bit.CONV14 = 0xE; // Setup ADCINB6 as 15th SEQ conv.
    AdcRegs.ADCCHSELSEQ4.bit.CONV15 = 0xF; // Setup ADCINB7 as 16th SEQ conv.


//    AdcRegs.ADCTRL1.bit.ACQ_PS=12;
    AdcRegs.ADCTRL1.bit.ACQ_PS=7;

    //触发源设置
//    AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;//软触发（用于级联、双通道1#）
//    AdcRegs.ADCTRL2.bit.SOC_SEQ2 = 1;//软触发（用于双通道2#）
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;//ePWM触发（用于双通道1#）
//    AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ2 = 1;//ePWM触发（用于双通道2#）
//    AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ = 1;//ePWM触发（用于级联）
    //中断源设置
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1; //用于级联、双通道1#
//    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ2 = 1; //双通道2#


}


void ConfigureEPwm(void)
{
    //EPWM1-3 DRIVE

    //Note that the default/2 divider for ePWMs and EMIFs
    //EPWMCLK=SYSCLKOUT=150MHZ
    //ePWM模块时基设置  // TBCLK = EPWMCLK/(HSPCLKDIV*CLKDIV)
    //PWM周期=15000个TBCLK周期100us
//    Uint16 Prd=7500;
    //5K
    Uint16 Prd=15000;
    //Uint16 Prd_da=7500;
    // Initialize EPwm1/2/3/4/5/6/7
    //自增配置程序------------->>>>>>>>>>>>>
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;
    //EPWM Moudle 1 配置
    //TB
    EPwm1Regs.TBCTL.bit.HSPCLKDIV=0;
    EPwm1Regs.TBCTL.bit.CLKDIV=0;
    EPwm1Regs.TBPRD=Prd;
    EPwm1Regs.TBPHS.half.TBPHS=0;//将相位寄存器值清零
    EPwm1Regs.TBCTR=0x0000;
    EPwm1Regs.TBCTL.bit.CTRMODE=TB_COUNT_UPDOWN;//增减模式
    EPwm1Regs.TBCTL.bit.PHSEN=TB_DISABLE;//禁止相位装载
    EPwm1Regs.TBCTL.bit.PRDLD=TB_SHADOW;//当CTR=0时，将映射寄存器中的数据装载到当前寄存器
    EPwm1Regs.TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;//CTR=0时发出同步信号
    //CC
    EPwm1Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;//CMPA寄存器工作在 映射模式
    EPwm1Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;//CMPB寄存器工作在 映射模式
    EPwm1Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;//在CTR=0时装载
    EPwm1Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;//在CTR=0时装载
    //AQ
    EPwm1Regs.AQCTLA.bit.CAU=AQ_SET;//当时间基准计数器的值等于CMPA的值，且正在递增计数，使EPWM1A为高电平
    EPwm1Regs.AQCTLA.bit.CAD=AQ_CLEAR;//当时间基准计数器的值等于CMPA的值，且正在递减计数，使EPWM1A为低电平

     //DB
    EPwm1Regs.AQCSFRC.all = 0x0005 ;//软件强制PWM输出为低,2017/12/26
    //EPwm1Regs.DBCTL.bit.OUT_MODE=DB_FULL_ENABLE;//使能上升沿及下降沿延时信号
    EPwm1Regs.DBCTL.bit.IN_MODE=DBA_ALL;//默认EPWM1A作为上升沿及下降沿延时的信号源
    EPwm1Regs.DBCTL.bit.POLSEL=DB_ACTV_HIC;//AHC\EPWM1B反转极性
    EPwm1Regs.DBFED=200;//下降沿延时FED=200个TBCLK
    EPwm1Regs.DBRED=200;//上升沿延时RED=200个TBCLK

    //事件触发设置ADC
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;         // 使能SOCA转换器工作
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;//计数器为0时触发转换
    EPwm1Regs.ETPS.bit.SOCAPRD  = ET_1ST; // Generate pulse on 1st event
//    EPwm1Regs.ETSEL.bit.SOCBEN  = 1;         // 使能SOCB转换器工作
//    EPwm1Regs.ETSEL.bit.SOCBSEL = ET_CTRD_CMPA;//计数器为0时触发转换
//    EPwm1Regs.ETPS.bit.SOCBPRD  = ET_1ST;
    //EPwm1Regs.CMPA.bit.CMPA  = 100;   // Set compare A value
    //EPwm1Regs.TBCTL.bit.CTRMODE=0;

    //EPWM Moudle 2 配置
    //TB
    EPwm2Regs.TBCTL.bit.HSPCLKDIV=0;
    EPwm2Regs.TBCTL.bit.CLKDIV=0;
    EPwm2Regs.TBPRD=Prd;
    EPwm2Regs.TBPHS.half.TBPHS=0;//将相位寄存器值清零
    EPwm2Regs.TBCTR=0x0000;
    EPwm2Regs.TBCTL.bit.CTRMODE=TB_COUNT_UPDOWN;//增减模式
    EPwm2Regs.TBCTL.bit.PHSEN=TB_DISABLE;//禁止相位装载
    EPwm2Regs.TBCTL.bit.PRDLD=TB_SHADOW;//当CTR=0时，将映射寄存器中的数据装载到当前寄存器
    EPwm2Regs.TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;//CTR=0时发出同步信号
    //CC
    EPwm2Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;//CMPA寄存器工作在 映射模式
    EPwm2Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;//CMPB寄存器工作在 映射模式
    EPwm2Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;//在CTR=0时装载
    EPwm2Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;//在CTR=0时装载
    //AQ
    EPwm2Regs.AQCTLA.bit.CAU=AQ_SET;//当时间基准计数器的值等于CMPA的值，且正在递增计数，使EPWM1A为高电平
    EPwm2Regs.AQCTLA.bit.CAD=AQ_CLEAR;//当时间基准计数器的值等于CMPA的值，且正在递减计数，使EPWM1A为低电平

     //DB
    EPwm2Regs.AQCSFRC.all = 0x0005 ;//软件强制PWM输出为低,2017/12/26
    //EPwm1Regs.DBCTL.bit.OUT_MODE=DB_FULL_ENABLE;//使能上升沿及下降沿延时信号
    EPwm2Regs.DBCTL.bit.IN_MODE=DBA_ALL;//默认EPWM1A作为上升沿及下降沿延时的信号源
    EPwm2Regs.DBCTL.bit.POLSEL=DB_ACTV_HIC;//AHC\EPWM1B反转极性
    EPwm2Regs.DBFED=200;//下降沿延时FED=200个TBCLK
    EPwm2Regs.DBRED=200;//上升沿延时RED=200个TBCLK

    //EPWM Moudle 3 配置
    //TB
    EPwm3Regs.TBCTL.bit.HSPCLKDIV=0;
    EPwm3Regs.TBCTL.bit.CLKDIV=0;
    EPwm3Regs.TBPRD=Prd;
    EPwm3Regs.TBPHS.half.TBPHS=0;//将相位寄存器值清零
    EPwm3Regs.TBCTR=0x0000;
    EPwm3Regs.TBCTL.bit.CTRMODE=TB_COUNT_UPDOWN;//增减模式
    EPwm3Regs.TBCTL.bit.PHSEN=TB_DISABLE;//禁止相位装载
    EPwm3Regs.TBCTL.bit.PRDLD=TB_SHADOW;//当CTR=0时，将映射寄存器中的数据装载到当前寄存器
    EPwm3Regs.TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;//CTR=0时发出同步信号
    //CC
    EPwm3Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;//CMPA寄存器工作在 映射模式
    EPwm3Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;//CMPB寄存器工作在 映射模式
    EPwm3Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;//在CTR=0时装载
    EPwm3Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;//在CTR=0时装载
    //AQ
    EPwm3Regs.AQCTLA.bit.CAU=AQ_SET;//当时间基准计数器的值等于CMPA的值，且正在递增计数，使EPWMA为高电平
    EPwm3Regs.AQCTLA.bit.CAD=AQ_CLEAR;//当时间基准计数器的值等于CMPA的值，且正在递减计数，使EPWMA为低电平

     //DB
    EPwm3Regs.AQCSFRC.all = 0x0005 ;//软件强制PWM输出为低,2017/12/26
    //EPwm3Regs.DBCTL.bit.OUT_MODE=DB_FULL_ENABLE;//使能上升沿及下降沿延时信号
    EPwm3Regs.DBCTL.bit.IN_MODE=DBA_ALL;//默认EPWMA作为上升沿及下降沿延时的信号源
    EPwm3Regs.DBCTL.bit.POLSEL=DB_ACTV_HIC;//AHC\EPWM1B反转极性
    EPwm3Regs.DBFED=200;//下降沿延时FED=200个TBCLK
    EPwm3Regs.DBRED=200;//上升沿延时RED=200个TBCLK


//    //EPWM7-8 DAC
//    //EPWM Moudle 7 配置
//    //TB
//    EPwm7Regs.TBCTL.bit.HSPCLKDIV=0;
//    EPwm7Regs.TBCTL.bit.CLKDIV=0;
//    EPwm7Regs.TBPRD=Prd_da;
//    EPwm7Regs.TBPHS.bit.TBPHS=0;//将相位寄存器值清零
//    EPwm7Regs.TBCTR=0x0000;
//    EPwm7Regs.TBCTL.bit.CTRMODE=TB_COUNT_UPDOWN;//增减模式
//    EPwm7Regs.TBCTL.bit.PHSEN=TB_DISABLE;//禁止相位装载
//    EPwm7Regs.TBCTL.bit.PRDLD=TB_SHADOW;//当CTR=0时，将映射寄存器中的数据装载到当前寄存器
//    EPwm7Regs.TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;//CTR=0时发出同步信号
//    //CC
//    EPwm7Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;//CMPA寄存器工作在 映射模式
//    EPwm7Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;//CMPB寄存器工作在 映射模式
//    EPwm7Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;//在CTR=0时装载
//    EPwm7Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;//在CTR=0时装载
//    //AQ
//    EPwm7Regs.AQCTLA.bit.CAU=AQ_SET;//当时间基准计数器的值等于CMPA的值，且正在递增计数，使EPWMA为高电平
//    EPwm7Regs.AQCTLA.bit.CAD=AQ_CLEAR;//当时间基准计数器的值等于CMPA的值，且正在递减计数，使EPWMA为低电平
//    EPwm7Regs.AQCTLB.bit.CAU=AQ_SET;
//    EPwm7Regs.AQCTLB.bit.CAD=AQ_CLEAR;
//     //DB
//    //软件强制PWM输出为低
//    EPwm7Regs.AQCSFRC.bit.CSFA=AQ_CLEAR;
//    EPwm7Regs.AQCSFRC.bit.CSFB=AQ_CLEAR;
//    EPwm7Regs.DBCTL.bit.OUT_MODE=DB_DISABLE;//禁止死区模块
//
//
//    //EPWM Moudle 8 配置
//    //TB
//    EPwm8Regs.TBCTL.bit.HSPCLKDIV=0;
//    EPwm8Regs.TBCTL.bit.CLKDIV=0;
//    EPwm8Regs.TBPRD=Prd_da;
//    EPwm8Regs.TBPHS.bit.TBPHS=0;//将相位寄存器值清零
//    EPwm8Regs.TBCTR=0x0000;
//    EPwm8Regs.TBCTL.bit.CTRMODE=TB_COUNT_UPDOWN;//增减模式
//    EPwm8Regs.TBCTL.bit.PHSEN=TB_DISABLE;//禁止相位装载
//    EPwm8Regs.TBCTL.bit.PRDLD=TB_SHADOW;//当CTR=0时，将映射寄存器中的数据装载到当前寄存器
//    EPwm8Regs.TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;//CTR=0时发出同步信号
//    //CC
//    EPwm8Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;//CMPA寄存器工作在 映射模式
//    EPwm8Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;//CMPB寄存器工作在 映射模式
//    EPwm8Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;//在CTR=0时装载
//    EPwm8Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;//在CTR=0时装载
//    //AQ
//    EPwm8Regs.AQCTLA.bit.CAU=AQ_SET;//当时间基准计数器的值等于CMPA的值，且正在递增计数，使EPWMA为高电平
//    EPwm8Regs.AQCTLA.bit.CAD=AQ_CLEAR;//当时间基准计数器的值等于CMPA的值，且正在递减计数，使EPWMA为低电平
//    EPwm8Regs.AQCTLB.bit.CAU=AQ_SET;
//    EPwm8Regs.AQCTLB.bit.CAD=AQ_CLEAR;
//     //DB
//    //软件强制PWM输出为低
//    EPwm8Regs.AQCSFRC.bit.CSFA=AQ_CLEAR;
//    EPwm8Regs.AQCSFRC.bit.CSFB=AQ_CLEAR;
//    EPwm8Regs.DBCTL.bit.OUT_MODE=DB_DISABLE;//使能上升沿及下降沿延时信号




    //TBD
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}



//#define CPU_FREQ           150E6
//#define LSPCLK_FREQ     CPU_FREQ/4
//#define SCI_FREQ        9600
//#define SCI_PRD         ((LSPCLK_FREQ/(SCI_FREQ*8))-1)
//BRR =SCI_PRD;

void InitSciGpio_a(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled disabled by the user.
    // This will enable the pullups for the specified pins.
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO36 = 0;  // Enable pull-up for GPIO36 (SCIRXDA)
    GpioCtrlRegs.GPBPUD.bit.GPIO35 = 0;	 // Enable pull-up for GPIO35 (SCITXDA)

    //
    // Set qualification for selected pins to asynch only
    // Inputs are synchronized to SYSCLKOUT by default.
    // This will select asynch (no qualification) for the selected pins.
    //
    GpioCtrlRegs.GPBQSEL1.bit.GPIO36 = 3;  // Asynch input GPIO36 (SCIRXDA)

    //
    // Configure SCI-A pins using GPIO regs
    // This specifies which of the possible GPIO pins will be SCI functional
    // pins.
    //
    GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 1;   // Configure GPIO36 to SCIRXDA
    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 1;   // Configure GPIO35 to SCITXDA


    EDIS;
}

void ConfigureSci(void)
{
    EALLOW;
    //SysCtrlRegs.PCLKCR0.bit.SCIAENCLK= 1;
    SysCtrlRegs.PCLKCR0.bit.SCICENCLK= 1;
    //SciaRegs.SCIFFTX.all=0xE040;
    //SciaRegs.SCIFFRX.all=0x2044;
    //SciaRegs.SCIFFCT.all=0x0;

    //SCIA

//    SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
//                                   // No parity,8 char bits,
//                                   // async mode, idle-line protocol
//    SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
//                                   // Disable RX ERR, SLEEP, TXWAKE
//    SciaRegs.SCICTL2.all =0x0003;
//    SciaRegs.SCICTL2.bit.TXINTENA =1;
//    SciaRegs.SCICTL2.bit.RXBKINTENA =1;
//
//    SciaRegs.SCIHBAUD=0x0001;  // 9600 baud @LSPCLK = 37.5MHz
//                                         //(150 MHz SYSCLK).
//    SciaRegs.SCILBAUD =0x00E7;
//
//    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

    //SCIC
    ScicRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    ScicRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    ScicRegs.SCICTL2.all =0x0003;
    ScicRegs.SCICTL2.bit.TXINTENA =1;
    ScicRegs.SCICTL2.bit.RXBKINTENA =1;

    ScicRegs.SCIHBAUD=0x0001;  // 9600 baud @LSPCLK = 37.5MHz
                                         //(150 MHz SYSCLK).
    ScicRegs.SCILBAUD =0x00E7;

    ScicRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset


    EDIS;

}

//外部中断，进行过流保护
void ConfigureXint(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO25=0;
    GpioCtrlRegs.GPADIR.bit.GPIO25=0;
    //由于下降沿触发XINT，需要加上拉，但如果外电路有上拉，则可以不用内部上拉
    GpioCtrlRegs.GPAPUD.bit.GPIO25=0;
    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL=25;//XINT1=gpio25
    EDIS;
// Configure XINT1 Falling edge interrupt
    XIntruptRegs.XINT1CR.bit.POLARITY=0;
// Enable XINT1
    XIntruptRegs.XINT1CR.bit.ENABLE=1;



}

void InitUseGpio(void)
{
	InitBoardGpio();

    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();

    InitScicGpio();
}
