/*
 * Adc_self.h
 *
 *  Created on: 2017-1-6
 *      Author: naiyangui
 */

#ifndef ADC_SELF_H_
#define ADC_SELF_H_

#define ADCZeroCNT0num 600//采样校零每循环采样点数
#define ADCZeroCNTloopnum 8 //采样校零循环数

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
//声明ADC_VOLT_CURRENT_GEThandle为ADC_VOLT_CURRENT_GET指针类型
//typedef ADC_VOLT_CURRENT_GET*ADC_VOLT_CURRENT_GET_handle;
//-----------------------------------------------------------------------------
//定义ADC模块的初始值
//-----------------------------------------------------------------------------
#define ADC_VOLT_CURRENT_GET_DEFAULTS {0,0,0,\
	                                  0,0,0,\
	                                  0,0,0,\
	                                  0,0,0}

//结构体变量
 extern ADC_VOLT_CURRENT_GET Adcget;
 extern void Adcdeal(void);//ADC采样处理
 extern void Adcread(void);//ADC采样读取
 extern void ADCzero(void);

#endif /* ADC_SELF_H_ */
