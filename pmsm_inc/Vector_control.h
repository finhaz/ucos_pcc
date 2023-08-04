/*
 * Vector_control.h
 *
 *  Created on: 2017-1-6
 *      Author: naiyangui
 */

#ifndef VECTOR_CONTROL_H_
#define VECTOR_CONTROL_H_


//--------------------定义结构体-----------------------------------

//坐标变换

typedef struct {  float  As;
				  float  Bs;
				  float  Cs;
				  float  Sine0;
				  float  Sine1;
				  float  Sine2;
				  float  Cosine0;
				  float  Cosine1;
				  float  Cosine2;
				  float  Angle;
				  float  Ds;
				  float  Qs;
				  float  V0;
		 	 	} ABC_PARK;

//----------------------------------------------------------------
//typedef struct {_iq  As;  		// Input: phase-a stator variable
//				_iq  Bs;			// Input: phase-b stator variable
//				_iq  Cs;			// Input: phase-c stator variable
//				_iq  Alpha;		// Output: stationary d-axis stator variable
//				_iq  Beta;		// Output: stationary q-axis stator variable
//				_iq  Angle;		// Input: rotating angle (pu)
//				_iq  Ds;			// Output: rotating d-axis stator variable
//				_iq  Qs;			// Output: rotating q-axis stator variable
//				_iq  Sine;
//				_iq  Cosine;
//		 	 	} CLARKE_PARK;


//声明CLARKE_PARK_handle为CLARKE_PARK指针类型
typedef ABC_PARK*CLARKE_PARK_handle;

//初始化
#define CLARKE_PARK_DEFAULTS {0, \
	           				  0, \
	           				  0, \
	           				  0, \
	           				  0, \
	           				  0, \
	           				  0, \
	           				  0, \
	           				  0, \
	           				  0, \
	           				  0, \
	           				  0, \
	           				  0, \
	           				}

//参数
//  1/sqrt(3) = 0.57735026918963
#define  ONEbySQRT3   0.57735026918963    /* 1/sqrt(3) */
#define  SQRT3byTWO   0.8660254    /* sqrt(3)/2 */
#define  TWObyTHREE   0.66666666666667    /* 2/3 */
#define  ONEbyTHREE   0.33333333333333    /* 1/3 */
#define  PI           3.141592654
#define  PIby180      0.01745329
#define  c180byPI      57.29578
//void abc_dq_clac(CLARKE_PARK_handle);
void abc_dq0p(CLARKE_PARK_handle);
void iabc_dq0p(CLARKE_PARK_handle);
void abc_dq0n(CLARKE_PARK_handle);
void iabc_dq0n(CLARKE_PARK_handle);
//
////------------------------------------------pi调节器
//typedef struct {
//	_iq   qdSum;		//Integrator sum; 1.31 format
//	_iq   currentError;
//	_iq   U;
//	_iq   qKp;			//Proportional Gain
//	_iq   qKi;			//Integral Gain
//	_iq   qKc;			//Anti-windup Gain
//	_iq   qOutMax;		//PI Output maximum limit
//	_iq   qOutMin;		//PI Output minimum limit
//	_iq   qInRef; 		//给定参考值
//	_iq   qInMeas;		//反馈值
//	_iq   qOut;			//PI Output; 1.15 format
//    }PI_CONTROL;
typedef struct {
    float   qdSum;        //Integrator sum; 1.31 format
    float   currentError;
    float   U;
    float   qKp;          //Proportional Gain
    float   qKi;          //Integral Gain
    float   qKc;          //Anti-windup Gain
    float   qOutMax;      //PI Output maximum limit
    float   qOutMin;      //PI Output minimum limit
    float   qInRef;       //给定参考值
    float   qInMeas;      //反馈值
    float   qOut;         //PI Output; 1.15 format
    }PI_CONTROL;

//声明PI_CONTROL_handle为PI_CONTROL指针类型
typedef PI_CONTROL*PI_CONTROL_handle;
#define PI_CONTROL_DEFAULTS {0,0,0,\
	                         0,0,0,\
	                         0,0,0,\
	                         0,0,\
	                         }
//void InitPI(PI_CONTROL_handle);
void PI_CONTROL_CALC(PI_CONTROL_handle);
extern void PIZero();//变量声明的外部引用，为了变量跨文件使用

////------------------------------------------滤波filtrate
typedef struct {  float  X;
				  float  X_last;
				  float  X_last1;
				  float  X_in;
				  float  Y;
				  float  Y_last;
				  float  Y_last1;
				  float  a1;//给X
				  float  a2;//给X_last
				  float  a3;//给X_last1
				  float  b1;//给Y_last
				  float  b2;//给Y_last1
		 	 	} FILTRATE;
//#define  a1   0.000039130205399144361             //滤波器参数
//#define  a2   0.000078260410798288723
//#define  a3   0.000039130205399144361
//#define  b1   -1.9822289297925286
//#define  b2    0.9823854506141253
//声明FILTRATE_handle为FILTRATE指针类型
typedef FILTRATE*FILTRATE_handle;
//#define FILTRATE_DEFAULTS_20Hz {0,0,0,0,0,0,0,\
//	                       0.000039130205399144361,\
//	                       0.000078260410798288723,\
//	                       0.000039130205399144361,\
//	                       -1.9822289297925286,\
//	                       0.9823854506141253}//fs=10k
#define FILTRATE_DEFAULTS_20Hz {0,0,0,0,0,0,0,\
		                   0.00015514842347569903,\
		                   0.00031029684695139806,\
		                   0.00015514842347569903,\
		                   -1.964460580205232,\
		                   0.96508117389913495}//fs=5k
//void InitPI(FILTRATE_handle);
void FILTRATE_CALC(FILTRATE_handle);

//------------------------------------------解耦
typedef struct {  float  Ud;
				  float  Uq;
				  float  Udmean;
				  float  Uqmean;
				  float  Angle;
				  float  Udout;
				  float  Uqout;
		 	 	} JIEOU;

//声明FILTRATE_handle为FILTRATE指针类型
typedef JIEOU*JIEOU_handle;
#define JIEOU_DEFAULTS {0,0,0,0,0,0,0}
//void InitPI(FILTRATE_handle);
void JIEOU_CALC(JIEOU_handle);

////------------------------------------------DDSRF_PLL结构
typedef struct {  float  Udp;
				  float  Uqp;
				  float  Udn;
				  float  Uqn;
				  float  Udpmean;
				  float  Uqpmean;
				  float  Udnmean;
				  float  Uqnmean;
				  float  Angle;
				  float  Udpout;
				  float  Uqpout;
				  float  Udnout;
				  float  Uqnout;
				  FILTRATE Udp_filtrate;
				  FILTRATE Uqp_filtrate;
				  FILTRATE Udn_filtrate;
				  FILTRATE Uqn_filtrate;
		 	 	} DDSRF_PLL;

//声明FILTRATE_handle为FILTRATE指针类型
typedef DDSRF_PLL*DDSRF_PLL_handle;
#define DDSRF_PLL_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,FILTRATE_DEFAULTS_20Hz,FILTRATE_DEFAULTS_20Hz,FILTRATE_DEFAULTS_20Hz,FILTRATE_DEFAULTS_20Hz}
//void InitPI(FILTRATE_handle);
void DDSRF_PLL_CALC(DDSRF_PLL_handle);






//
//结构体变量



//
//函数声明
void predir(void);
void predir_IFstart(void);
void breaking(void);
void droop(void);
void neiwaihuan(void);
void VectorControl_zero(void);
void Initparameter(void);


#endif /* VECTOR_CONTROL_H_ */
