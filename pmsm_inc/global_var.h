#ifndef __GOF__
#define __GOF__


#include <string.h>
#include <stdint.h>
#include <stdlib.h>
//#include "IQmathLib.h"
#include "Adc_self.h"
#include "flag.h"
#include"fault.h"
#include "Vector_control.h"
#include "Switch.h"
#include "message.h"

//如果是定义一个常数就用#define   如果定义一个变量就用extern float
//SCI
#define PackHeadLength 4 //包头长度
#define PackHead 0x00FE  //包头
//#define TestIO PORTBbits.RB5  //用于检测
//#define CL485 PORTBbits.RB13  //CL485为1时发送使能，为0时接收使能
#define ParameterNumber 118 //定义通信变量个数
#define SortNo 0x01//站点号，本程序中未使用
//#define set485 GpioDataRegs.GPASET.bit.GPIO26= 1//发送使能
//#define clear485 GpioDataRegs.GPACLEAR.bit.GPIO26= 1 //接收使能
#define On_off 81   //开关机
#define ConfirmCode  01 //SCI确认码
//自定义变量SCI对应关系
#define speed_bldcm 1
#define I_meandc_run 2
#define Iu_adc_run 3
#define Iv_adc_run 4
#define Iw_adc_run 5
#define U_meandc_run 6
#define hallposition 7
//8、9、10作为采样显示的保留
#define overcurrentt 11
#define overloadd 12
#define overvoltagee 13
#define undervoltagee 14
#define tabanfaultt 15
#define hightemcircuitt 16
#define hightemmotorr 17
#define ocfaultt 18
#define faultoccurr 19
#define flagfault_run 20
//////////////////////////////以上是运行参数中固定参数，基本不可修改，仅可对8、9、10增加项目。
#define io_alpha 21
#define io_beta  22
#define io_d     23
#define io_q     24
#define uo_alpha 25
#define uo_beta 26
#define uo_d 27
#define uo_q 28

#define U0 311
#define w0 50*2*PI
#define m 0.000005
#define n 0.0001
#define M 500


//////////////////////////////希望通过上位机给定的值在main里有一段Paramet的语句是上位机给变量赋值的
#define P_0     44//给定的有功 无功功率
#define Q_0     45
#define kp_I_p  46//电流内环d轴PI参数
#define ki_I_p  47//
#define kp_I_n  48//电流内环q轴PI参数
#define ki_I_n  49
#define kp_u_p  50//电压外环d轴PI参数
#define ki_u_p  51
#define kp_u_n  52//电压外环q轴PI参数
#define ki_u_n  53
#define kp_pcc 54
#define ki_pcc 55
#define reduce_ratio 56
#define current_ref 57
#define stop_moder 58
#define speed_max 59
#define speed_ref 60
#define ref_source 61
#define rotatdircw 62//正反转
#define N_kp 63
#define N_ki 64
#define PI_N_max 65
#define PI_N_min 66
#define I_kp 67
#define I_ki 68
#define PI_I_max 70
#define PI_I_min 71
//开关频率为fk,则开关周期为ts,TBPRD对应的值表示开关周期的一半，switch表示开关周期值
#define W_kp 72 //
#define W_ki 73
#define PI_W_max 74
#define PI_W_min 75
#define modulation 76
#define predir_num2 77
#define Kw_start 78
#define Iq_ref_startup 79
#define debugmode 80
#define debug_predirORlcom 81
#define slope_delt_n 82
#define slope_delt_t 83
#define Slope_en_sign 84
#define current_ref_limit 85
#define rs_psm 102
#define Ld_psm 103
#define Lq_psm 104
#define flux_psm 105


#define graphNumber 400
#define PSONumber 48
        //IAD=I*0.08*4095/3
        //UDC=U*0.0025*4095/3
//#define ADC_I 0.009157509    //0.08
//#define ADC_I 0.029296875      //0.025
//#define ADC_U 0.1831502      //0.004
#define ADC_I -0.04884005      //1.5/100=0.015
#define ADC_U -0.2442002      //1.5/500=0.003
//------------------------MARS------------------------------------------------------------------------
//定义电机运行相关参数
//定义电机运行相关参数
//#define Ld 0.0157 // 0.01567//15.67//
//#define Lq 0.0183 // 0.018265//18.265//
//#define Ls 0.017 // 0.018265//18.265//

#define twopi (2*PI)//102944
#define pibytwo (PI/2)//25736
#define c2 1430  //286  //12000//7000  //4685824//286//10du

#define vn_comp  3500  //负序补偿时间

extern unsigned int Switchsystem;
extern unsigned int RCBUF[24];//RS485 接收缓存器 （系统级通讯协议的25个数据）
extern unsigned int ReciveRCOUNT;//RS485 接收计数器 0~25
extern unsigned int RC_DataCount;   //接收数据计数器
extern unsigned int TXCOUNT;//RS485 发送计数器
extern unsigned int PSOCOUNT;//RS485 发送计数器
extern unsigned int TXBUF[13];//RS485 发送缓存器
extern unsigned int PSOBUF[PSONumber];//RS485 发送缓存器
extern unsigned int flagRC;//接收数据结束标志位
extern unsigned int flagSEND;//发送数据标志位
extern unsigned int PSOSENDF;//发送数据标志位
extern unsigned int RunCommand_L;
extern unsigned int RunCommand_H;
extern unsigned int RunCommand;
extern unsigned int cyclecount;
extern unsigned int datasum;//发送数据求和
extern unsigned int datasum1;
extern unsigned int PackLength;     //数据包长
extern unsigned int RC_DataBUF[20];  //接收数据缓存器（包括站点号-序列号-命令码-数据高-数据低-校验码）
extern unsigned int RC_DataCount;   //接收数据计数器
extern unsigned int SortNumber;     //站点号
extern unsigned int SerialNumber;   //SCI序号
extern unsigned int CommandCode;     //SCI命令码
extern unsigned int CheckCode;      //校验码
extern unsigned int SendData;       //发送数据
extern unsigned int SendDataNumber;     //发送数据个数
//extern unsigned int Paramet[ParameterNumber];
extern unsigned int PSO_datainit_flag;
extern float Paramet[ParameterNumber];

extern Uint32 AC_Zero[12][8];
extern Uint32 AC_ZeroMean[12];
extern Uint16 AC_Zero_yu[12][8];
extern Uint16 AC_Zero_yuz[12];
extern Uint16 ADCresult[12] ;    //2
extern Uint16 ADCZeroCNT0;      //6采样校零用来计数，计600
extern int32 ADCresultNEW[12]; //用于存放去偏置之后的数据（可选择减或不减直流偏置）
extern Uint16 adcresult15;
extern Uint32 ADCcount;
extern Uint32 ad_count[3];
extern int32 Udc;
extern Uint32 j;
extern Uint32 i;
extern Uint16 N_stage;
extern Uint16 N_stage2;
extern Uint16 N_stage1;
//标志变量




extern Uint16 break_count;


extern float pso_t[10];
extern int n_pso;




extern Uint16 stage;
extern struct FLAG_REGS FlagRegs;
extern struct COUNTER_REGS CounterRegs;

extern float T;

extern float max_current;
extern float min_current;


extern PI_CONTROL PI_Ud;
extern PI_CONTROL PI_Uq;
extern PI_CONTROL PI_Id;
extern PI_CONTROL PI_Iq;
extern PI_CONTROL PI_Udn;
extern PI_CONTROL PI_Uqn;
extern PI_CONTROL PI_Idn;
extern PI_CONTROL PI_Iqn;
extern PI_CONTROL PI_PCC;

//坐标变换
extern ABC_PARK I_conversion;//Iabc变成iα、iβ
extern ABC_PARK U_conversion;//iα、iβ变成id、iq
//extern ABC_PARK Io_conversion;//uα、uβ变成ud、uq
extern ABC_PARK Uo_conversion;//ud、uq变成uα、uβ
extern ABC_PARK Uref_conversion; //电压反馈
extern ABC_PARK Uout_conversion;
extern ABC_PARK Uoutn_conversion;
//extern ABC_PARK test_conversion;//测试用
//extern ABC_PARK test2_conversion;//测试用

extern FILTRATE test_filtrate;//测试用
//extern FILTRATE Udp_filtrate;
//extern FILTRATE Uqp_filtrate;
//extern FILTRATE Udn_filtrate;
//extern FILTRATE Uqn_filtrate;

extern JIEOU jieou_positive;
extern JIEOU jieou_negative;

//extern DDSRF_PLL test_DDSRF_PLL;//测试用
extern DDSRF_PLL I_DDSRF_PLL;
extern DDSRF_PLL U_DDSRF_PLL;
extern DDSRF_PLL Uo_DDSRF_PLL;

extern float a_graph[graphNumber];
extern float b_graph[graphNumber];
extern float c_graph[graphNumber];
extern int n_graph;
//
//extern float test_Uout;//测试用
//extern float   ceshi;

extern float w;
extern float U;
extern float P;
extern float Q;
extern float P_last;
extern float Q_last;

extern float Ua_ref;
extern float Ub_ref;
extern float Uc_ref;


extern float Ud;
extern float Uq;
extern float Ud_ref;
extern float Uq_ref;
extern float delt_Ud;
extern float delt_Uq;
extern float Id;
extern float Iq;




extern float Idp;
extern float Iqp;
extern float Idn;
extern float Iqn;

extern float Udp;
extern float Uqp;
extern float Udn;
extern float Uqn;

extern float Uodp;
extern float Uoqp;
extern float Uodn;
extern float Uoqn;

//-------------------------输出
extern float Idpout;
extern float Iqpout;
extern float Idnout;
extern float Iqnout;

extern float Udpout;
extern float Uqpout;
extern float Udnout;
extern float Uqnout;

extern float Uodpout;
extern float Uoqpout;
extern float Uodnout;
extern float Uoqnout;

//---------------------------计算不平衡度
extern float VUFout;
extern float VUFpcc;
extern float VUFpccmean;
extern unsigned int n_count;
extern unsigned int n_count1;
extern float Udn_ref;//负序电压的参考值 由粒子群算法给出
extern float Uqn_ref;
extern float Idn_ref;
extern float Iqn_ref;
extern float delt_Udn;
extern float delt_Uqn;
extern float delt_Idn;
extern float delt_Iqn;




//电流
extern float Id_ref;
extern float Iq_ref;
extern float delt_Id;
extern float delt_Iq;

extern float theta_fan;                                                                                 //估算角度——参与坐标变换

                                                                                    //角度显示

//中间变量
extern float error;
extern float gain;

//spwm
extern float Ua;
extern float Ub;
extern float Uc;
extern Uint16 Tcmpa;
extern Uint16 Tcmpb;
extern Uint16 Tcmpc;
extern float m_sin_a;
extern float m_sin_b;
extern float m_sin_c;

//快速故障判断---用于PWM中断判断进行过流保护，过压保护，欠压保护策略
//过流保护
extern Uint16 moder_of_stop;
extern Uint16 overcurrent_flag;
extern Uint16 overcurrent_U;//U相过流标志
extern Uint16 overcurrent_W;//W相过流标志
extern Uint16 overcurrent_V;//V相过流标志

extern float P0;
extern float Q0;
//extern float kp_current_d;
//extern float ki_current_d;
//extern float kp_current_q;
//extern float ki_current_q;
//extern float kp_voltage_d;
//extern float ki_voltage_d;
//extern float kp_voltage_q;
//extern float ki_voltage_q;
extern float kp_current_dqp;
extern float ki_current_dqp;
extern float kp_current_dqn;
extern float ki_current_dqn;
extern float kp_voltage_dqp;
extern float ki_voltage_dqp;
extern float kp_voltage_dqn;
extern float ki_voltage_dqn;
extern float kp_pcc_degree;
extern float ki_pcc_degree;

extern float PIout_Ud;
extern float PIout_Uq;
extern float PIout_Id;
extern float PIout_Iq;
extern float PIout_Udn;
extern float PIout_Uqn;
extern float PIout_Idn;
extern float PIout_Iqn;

//extern int i_a;
//extern float a1_filter[100];
//extern float a_filter[FilterNumber];

extern union FLOAT_COM  Data_get;
extern union FLOAT_COMF  FData_send;
extern union FLOAT_COMF  FData_get;
extern union FLOAT_COMF  PSO_send;
extern union FLOAT_COMF  PSO_get;
extern float PSO_g[4];

#endif
