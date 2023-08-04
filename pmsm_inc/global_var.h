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

//����Ƕ���һ����������#define   �������һ����������extern float
//SCI
#define PackHeadLength 4 //��ͷ����
#define PackHead 0x00FE  //��ͷ
//#define TestIO PORTBbits.RB5  //���ڼ��
//#define CL485 PORTBbits.RB13  //CL485Ϊ1ʱ����ʹ�ܣ�Ϊ0ʱ����ʹ��
#define ParameterNumber 118 //����ͨ�ű�������
#define SortNo 0x01//վ��ţ���������δʹ��
//#define set485 GpioDataRegs.GPASET.bit.GPIO26= 1//����ʹ��
//#define clear485 GpioDataRegs.GPACLEAR.bit.GPIO26= 1 //����ʹ��
#define On_off 81   //���ػ�
#define ConfirmCode  01 //SCIȷ����
//�Զ������SCI��Ӧ��ϵ
#define speed_bldcm 1
#define I_meandc_run 2
#define Iu_adc_run 3
#define Iv_adc_run 4
#define Iw_adc_run 5
#define U_meandc_run 6
#define hallposition 7
//8��9��10��Ϊ������ʾ�ı���
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
//////////////////////////////���������в����й̶����������������޸ģ����ɶ�8��9��10������Ŀ��
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


//////////////////////////////ϣ��ͨ����λ��������ֵ��main����һ��Paramet���������λ����������ֵ��
#define P_0     44//�������й� �޹�����
#define Q_0     45
#define kp_I_p  46//�����ڻ�d��PI����
#define ki_I_p  47//
#define kp_I_n  48//�����ڻ�q��PI����
#define ki_I_n  49
#define kp_u_p  50//��ѹ�⻷d��PI����
#define ki_u_p  51
#define kp_u_n  52//��ѹ�⻷q��PI����
#define ki_u_n  53
#define kp_pcc 54
#define ki_pcc 55
#define reduce_ratio 56
#define current_ref 57
#define stop_moder 58
#define speed_max 59
#define speed_ref 60
#define ref_source 61
#define rotatdircw 62//����ת
#define N_kp 63
#define N_ki 64
#define PI_N_max 65
#define PI_N_min 66
#define I_kp 67
#define I_ki 68
#define PI_I_max 70
#define PI_I_min 71
//����Ƶ��Ϊfk,�򿪹�����Ϊts,TBPRD��Ӧ��ֵ��ʾ�������ڵ�һ�룬switch��ʾ��������ֵ
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
//������������ز���
//������������ز���
//#define Ld 0.0157 // 0.01567//15.67//
//#define Lq 0.0183 // 0.018265//18.265//
//#define Ls 0.017 // 0.018265//18.265//

#define twopi (2*PI)//102944
#define pibytwo (PI/2)//25736
#define c2 1430  //286  //12000//7000  //4685824//286//10du

#define vn_comp  3500  //���򲹳�ʱ��

extern unsigned int Switchsystem;
extern unsigned int RCBUF[24];//RS485 ���ջ����� ��ϵͳ��ͨѶЭ���25�����ݣ�
extern unsigned int ReciveRCOUNT;//RS485 ���ռ����� 0~25
extern unsigned int RC_DataCount;   //�������ݼ�����
extern unsigned int TXCOUNT;//RS485 ���ͼ�����
extern unsigned int PSOCOUNT;//RS485 ���ͼ�����
extern unsigned int TXBUF[13];//RS485 ���ͻ�����
extern unsigned int PSOBUF[PSONumber];//RS485 ���ͻ�����
extern unsigned int flagRC;//�������ݽ�����־λ
extern unsigned int flagSEND;//�������ݱ�־λ
extern unsigned int PSOSENDF;//�������ݱ�־λ
extern unsigned int RunCommand_L;
extern unsigned int RunCommand_H;
extern unsigned int RunCommand;
extern unsigned int cyclecount;
extern unsigned int datasum;//�����������
extern unsigned int datasum1;
extern unsigned int PackLength;     //���ݰ���
extern unsigned int RC_DataBUF[20];  //�������ݻ�����������վ���-���к�-������-���ݸ�-���ݵ�-У���룩
extern unsigned int RC_DataCount;   //�������ݼ�����
extern unsigned int SortNumber;     //վ���
extern unsigned int SerialNumber;   //SCI���
extern unsigned int CommandCode;     //SCI������
extern unsigned int CheckCode;      //У����
extern unsigned int SendData;       //��������
extern unsigned int SendDataNumber;     //�������ݸ���
//extern unsigned int Paramet[ParameterNumber];
extern unsigned int PSO_datainit_flag;
extern float Paramet[ParameterNumber];

extern Uint32 AC_Zero[12][8];
extern Uint32 AC_ZeroMean[12];
extern Uint16 AC_Zero_yu[12][8];
extern Uint16 AC_Zero_yuz[12];
extern Uint16 ADCresult[12] ;    //2
extern Uint16 ADCZeroCNT0;      //6����У��������������600
extern int32 ADCresultNEW[12]; //���ڴ��ȥƫ��֮������ݣ���ѡ����򲻼�ֱ��ƫ�ã�
extern Uint16 adcresult15;
extern Uint32 ADCcount;
extern Uint32 ad_count[3];
extern int32 Udc;
extern Uint32 j;
extern Uint32 i;
extern Uint16 N_stage;
extern Uint16 N_stage2;
extern Uint16 N_stage1;
//��־����




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

//����任
extern ABC_PARK I_conversion;//Iabc���i����i��
extern ABC_PARK U_conversion;//i����i�±��id��iq
//extern ABC_PARK Io_conversion;//u����u�±��ud��uq
extern ABC_PARK Uo_conversion;//ud��uq���u����u��
extern ABC_PARK Uref_conversion; //��ѹ����
extern ABC_PARK Uout_conversion;
extern ABC_PARK Uoutn_conversion;
//extern ABC_PARK test_conversion;//������
//extern ABC_PARK test2_conversion;//������

extern FILTRATE test_filtrate;//������
//extern FILTRATE Udp_filtrate;
//extern FILTRATE Uqp_filtrate;
//extern FILTRATE Udn_filtrate;
//extern FILTRATE Uqn_filtrate;

extern JIEOU jieou_positive;
extern JIEOU jieou_negative;

//extern DDSRF_PLL test_DDSRF_PLL;//������
extern DDSRF_PLL I_DDSRF_PLL;
extern DDSRF_PLL U_DDSRF_PLL;
extern DDSRF_PLL Uo_DDSRF_PLL;

extern float a_graph[graphNumber];
extern float b_graph[graphNumber];
extern float c_graph[graphNumber];
extern int n_graph;
//
//extern float test_Uout;//������
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

//-------------------------���
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

//---------------------------���㲻ƽ���
extern float VUFout;
extern float VUFpcc;
extern float VUFpccmean;
extern unsigned int n_count;
extern unsigned int n_count1;
extern float Udn_ref;//�����ѹ�Ĳο�ֵ ������Ⱥ�㷨����
extern float Uqn_ref;
extern float Idn_ref;
extern float Iqn_ref;
extern float delt_Udn;
extern float delt_Uqn;
extern float delt_Idn;
extern float delt_Iqn;




//����
extern float Id_ref;
extern float Iq_ref;
extern float delt_Id;
extern float delt_Iq;

extern float theta_fan;                                                                                 //����Ƕȡ�����������任

                                                                                    //�Ƕ���ʾ

//�м����
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

//���ٹ����ж�---����PWM�ж��жϽ��й�����������ѹ������Ƿѹ��������
//��������
extern Uint16 moder_of_stop;
extern Uint16 overcurrent_flag;
extern Uint16 overcurrent_U;//U�������־
extern Uint16 overcurrent_W;//W�������־
extern Uint16 overcurrent_V;//V�������־

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
