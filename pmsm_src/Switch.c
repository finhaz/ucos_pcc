/*
 * Switch.c
 *
 */


#include "DSP28x_Project.h"


//PWM����
void PWMoff(void)
{
    //���ػ��ж�
    //ePWM1����
    EPwm1Regs.AQSFRC.bit.RLDCSF = 3 ;//ѡ��ֱ�Ӽ���
    EPwm1Regs.DBCTL.bit.OUT_MODE=0;
    //ePWM2����
    EPwm2Regs.AQSFRC.bit.RLDCSF = 3 ;//ѡ��ֱ�Ӽ���
    EPwm2Regs.DBCTL.bit.OUT_MODE=0;
    //ePWM3����
    EPwm3Regs.AQSFRC.bit.RLDCSF = 3 ;//ѡ��ֱ�Ӽ���
    EPwm3Regs.DBCTL.bit.OUT_MODE=0;
//    //ePWM7����
//    EPwm7Regs.AQSFRC.bit.RLDCSF = 3 ;//ѡ��ֱ�Ӽ���
//    EPwm7Regs.DBCTL.bit.OUT_MODE=0;
//    //ePWM8����
//    EPwm8Regs.AQSFRC.bit.RLDCSF = 3 ;//ѡ��ֱ�Ӽ���
//    EPwm8Regs.DBCTL.bit.OUT_MODE=0;

    if(moder_of_stop==1)//
    {
//        EPwm1Regs.AQCSFRC.all = 0x0009 ;//���ǿ��PWMA���Ϊ��,PWMB���Ϊ��
//        EPwm2Regs.AQCSFRC.all = 0x0009 ;//���ǿ��PWMA���Ϊ��,PWMB���Ϊ��
//        EPwm3Regs.AQCSFRC.all = 0x0009;//���ǿ��PWMA���Ϊ��,PWMB���Ϊ��

        //���ǿ��PWMA���Ϊ��,PWMB���Ϊ��
        EPwm1Regs.AQCSFRC.bit.CSFA=AQ_CLEAR;
        EPwm1Regs.AQCSFRC.bit.CSFB=AQ_SET;
        EPwm2Regs.AQCSFRC.bit.CSFA=AQ_CLEAR;
        EPwm2Regs.AQCSFRC.bit.CSFB=AQ_SET;
        EPwm3Regs.AQCSFRC.bit.CSFA=AQ_CLEAR;
        EPwm3Regs.AQCSFRC.bit.CSFB=AQ_SET;
    }
    else
    {
//        EPwm1Regs.AQCSFRC.all = 0x0005 ;//���ǿ��PWM���Ϊ��
//        EPwm2Regs.AQCSFRC.all = 0x0005 ;//���ǿ��PWM���Ϊ��
//        EPwm3Regs.AQCSFRC.all = 0x0005;//���ǿ��PWM���Ϊ��

        //���ǿ��PWM���Ϊ��
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

//��PWM
void PWMopen(void)
{
    //���ػ��ж�
    //ePWM1����
    EPwm1Regs.AQSFRC.bit.RLDCSF = 0;//
    EPwm1Regs.AQCSFRC.all = 0x0000 ;//�޶���
    EPwm1Regs.DBCTL.bit.OUT_MODE=3;
    //ePWM2����
    EPwm2Regs.AQSFRC.bit.RLDCSF = 0 ;//
    EPwm2Regs.AQCSFRC.all = 0x0000 ;//�޶���
    EPwm2Regs.DBCTL.bit.OUT_MODE=3;
    //ePWM3����
    EPwm3Regs.AQSFRC.bit.RLDCSF = 0 ;//
    EPwm3Regs.AQCSFRC.all = 0x0000 ;//�޶���
    EPwm3Regs.DBCTL.bit.OUT_MODE=3;
//    //ePWM7����
//    EPwm7Regs.AQSFRC.bit.RLDCSF = 0 ;//
//    EPwm7Regs.AQCSFRC.all = 0x0000 ;//�޶���
//    //ePWM8����
//    EPwm8Regs.AQSFRC.bit.RLDCSF = 0 ;//
//    EPwm8Regs.AQCSFRC.all = 0x0000 ;//�޶���

}

void SYSTEMoff(void)
{
   PWMoff();//����PWM��
   PIZero();
   VectorControl_zero();
}








