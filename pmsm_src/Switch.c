/*
 * Switch.c
 *
 */


#include "DSP28x_Project.h"


//PWM封锁
void PWMoff(void)
{
    //开关机判断
    //ePWM1封锁
    EPwm1Regs.AQSFRC.bit.RLDCSF = 3 ;//选择直接加载
    EPwm1Regs.DBCTL.bit.OUT_MODE=0;
    //ePWM2封锁
    EPwm2Regs.AQSFRC.bit.RLDCSF = 3 ;//选择直接加载
    EPwm2Regs.DBCTL.bit.OUT_MODE=0;
    //ePWM3封锁
    EPwm3Regs.AQSFRC.bit.RLDCSF = 3 ;//选择直接加载
    EPwm3Regs.DBCTL.bit.OUT_MODE=0;
//    //ePWM7封锁
//    EPwm7Regs.AQSFRC.bit.RLDCSF = 3 ;//选择直接加载
//    EPwm7Regs.DBCTL.bit.OUT_MODE=0;
//    //ePWM8封锁
//    EPwm8Regs.AQSFRC.bit.RLDCSF = 3 ;//选择直接加载
//    EPwm8Regs.DBCTL.bit.OUT_MODE=0;

    if(moder_of_stop==1)//
    {
//        EPwm1Regs.AQCSFRC.all = 0x0009 ;//软件强制PWMA输出为低,PWMB输出为高
//        EPwm2Regs.AQCSFRC.all = 0x0009 ;//软件强制PWMA输出为低,PWMB输出为高
//        EPwm3Regs.AQCSFRC.all = 0x0009;//软件强制PWMA输出为低,PWMB输出为高

        //软件强制PWMA输出为低,PWMB输出为高
        EPwm1Regs.AQCSFRC.bit.CSFA=AQ_CLEAR;
        EPwm1Regs.AQCSFRC.bit.CSFB=AQ_SET;
        EPwm2Regs.AQCSFRC.bit.CSFA=AQ_CLEAR;
        EPwm2Regs.AQCSFRC.bit.CSFB=AQ_SET;
        EPwm3Regs.AQCSFRC.bit.CSFA=AQ_CLEAR;
        EPwm3Regs.AQCSFRC.bit.CSFB=AQ_SET;
    }
    else
    {
//        EPwm1Regs.AQCSFRC.all = 0x0005 ;//软件强制PWM输出为低
//        EPwm2Regs.AQCSFRC.all = 0x0005 ;//软件强制PWM输出为低
//        EPwm3Regs.AQCSFRC.all = 0x0005;//软件强制PWM输出为低

        //软件强制PWM输出为低
        EPwm1Regs.AQCSFRC.bit.CSFA=AQ_CLEAR;
        EPwm1Regs.AQCSFRC.bit.CSFB=AQ_CLEAR;
        EPwm2Regs.AQCSFRC.bit.CSFA=AQ_CLEAR;
        EPwm2Regs.AQCSFRC.bit.CSFB=AQ_CLEAR;
        EPwm3Regs.AQCSFRC.bit.CSFA=AQ_CLEAR;
        EPwm3Regs.AQCSFRC.bit.CSFB=AQ_CLEAR;

    }

//    EPwm7Regs.AQCSFRC.bit.CSFA=AQ_CLEAR;
//    EPwm7Regs.AQCSFRC.bit.CSFB=AQ_CLEAR;
//    EPwm8Regs.AQCSFRC.bit.CSFA=AQ_CLEAR;
//    EPwm8Regs.AQCSFRC.bit.CSFB=AQ_CLEAR;

}

//打开PWM
void PWMopen(void)
{
    //开关机判断
    //ePWM1封锁
    EPwm1Regs.AQSFRC.bit.RLDCSF = 0;//
    EPwm1Regs.AQCSFRC.all = 0x0000 ;//无动作
    EPwm1Regs.DBCTL.bit.OUT_MODE=3;
    //ePWM2封锁
    EPwm2Regs.AQSFRC.bit.RLDCSF = 0 ;//
    EPwm2Regs.AQCSFRC.all = 0x0000 ;//无动作
    EPwm2Regs.DBCTL.bit.OUT_MODE=3;
    //ePWM3封锁
    EPwm3Regs.AQSFRC.bit.RLDCSF = 0 ;//
    EPwm3Regs.AQCSFRC.all = 0x0000 ;//无动作
    EPwm3Regs.DBCTL.bit.OUT_MODE=3;
//    //ePWM7封锁
//    EPwm7Regs.AQSFRC.bit.RLDCSF = 0 ;//
//    EPwm7Regs.AQCSFRC.all = 0x0000 ;//无动作
//    //ePWM8封锁
//    EPwm8Regs.AQSFRC.bit.RLDCSF = 0 ;//
//    EPwm8Regs.AQCSFRC.all = 0x0000 ;//无动作

}

void SYSTEMoff(void)
{
   PWMoff();//封锁PWM，
   PIZero();
   VectorControl_zero();
}








