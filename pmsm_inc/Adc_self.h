/*
 * Adc_self.h
 *
 *  Created on: 2017-1-6
 *      Author: naiyangui
 */

#ifndef ADC_SELF_H_
#define ADC_SELF_H_

#define ADCZeroCNT0num 600//����У��ÿѭ����������
#define ADCZeroCNTloopnum 8 //����У��ѭ����

typedef struct{float Ia;
               float Ib;
               float Ic;
               float Ua;
               float Ub;
               float Uc;

               float Ioa;
               float Iob;
               float Ioc;
               float Uoa;
               float Uob;
               float Uoc;

              }ADC_VOLT_CURRENT_GET;
//-----------------------------------------------------------------------------
//����ADC_VOLT_CURRENT_GEThandleΪADC_VOLT_CURRENT_GETָ������
//typedef ADC_VOLT_CURRENT_GET*ADC_VOLT_CURRENT_GET_handle;
//-----------------------------------------------------------------------------
//����ADCģ��ĳ�ʼֵ
//-----------------------------------------------------------------------------
#define ADC_VOLT_CURRENT_GET_DEFAULTS {0,0,0,\
	                                  0,0,0,\
	                                  0,0,0,\
	                                  0,0,0}

//�ṹ�����
 extern ADC_VOLT_CURRENT_GET Adcget;
 extern void Adcdeal(void);//ADC��������
 extern void Adcread(void);//ADC������ȡ
 extern void ADCzero(void);

#endif /* ADC_SELF_H_ */
