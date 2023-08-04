#define PackHeadLength 4 //��ͷ����
#define PackHead 0x00FE  //��ͷ

#include "DSP2833x_Device.h"        // F2837xD Headerfile Include File
#include "global_var.h"

//AD����
Uint32 AC_Zero[12][8];
Uint32 AC_ZeroMean[12];
Uint16 AC_Zero_yu[12][8];
Uint16 AC_Zero_yuz[12];

Uint16 ADCresult[12] ;    //2
Uint16 ADCZeroCNT0;      //6����У��������������600
//Uint16 Rezidue[10];       //���ڴ�Ž����������ʱ������
int32 ADCresultNEW[12]; //���ڴ��ȥƫ��֮������ݣ���ѡ����򲻼�ֱ��ƫ�ã�
//int32 ADCresultAmend[10];  //5���ڴ����������Ĳ�����
Uint16 adcresult15;
//
Uint32 ADCcount;
Uint32 ad_count[3];
int32 Udc;


//sci
unsigned int Switchsystem;
unsigned int RCBUF[24];//RS485 ���ջ����� ��ϵͳ��ͨѶЭ���24�����ݣ�
unsigned int ReciveRCOUNT=0;//RS485 ���ռ����� 0~11
unsigned int RC_DataCount=0;   //�������ݼ�����
unsigned int TXCOUNT=0;//RS485 ���ͼ�����
unsigned int PSOCOUNT=0;//RS485 ���ͼ�����
unsigned int TXBUF[13];//RS485 ���ͻ�����
unsigned int PSOBUF[PSONumber];//RS485 ���ͻ�����
unsigned int flagRC=0;//�������ݽ�����־λ
unsigned int flagSEND=0;//�������ݱ�־λ
unsigned int PSOSENDF=0;//�������ݱ�־λ
unsigned int RunCommand_L;
unsigned int RunCommand_H;
unsigned int RunCommand;
unsigned int cyclecount;
unsigned int datasum;//�����������
unsigned int datasum1;
unsigned int PackLength;     //���ݰ���
unsigned int RC_DataBUF[20];  //�������ݻ�����������վ���-���к�-������-���ݸ�-���ݵ�-У���룩
unsigned int RC_DataCount;   //�������ݼ�����
unsigned int SortNumber;     //վ���
unsigned int SerialNumber;   //SCI���
unsigned int CommandCode;     //SCI������
unsigned int CheckCode;      //У����
unsigned int SendData;       //��������
unsigned int SendDataNumber;     //�������ݸ���
//unsigned int Paramet[ParameterNumber];
unsigned int PSO_datainit_flag;
float Paramet[ParameterNumber];

//float a1_filter[100];
//float a_filter[FilterNumber];
Uint16 adcresult15;
Uint32 ADCcount;
Uint32 ad_count[3];
int32 Udc;
Uint32 j;
Uint32 i;
Uint16 N_stage=0;
Uint16 N_stage2=0;
Uint16 N_stage1=0;
//��־����




Uint16 break_count;






Uint16 stage;
struct FLAG_REGS FlagRegs={0,0};//��־
struct COUNTER_REGS CounterRegs;//����



float speed_cankao=0;
float T=0.0002;//��������5k
float sudu_max;
float kp_weak;
float ki_weak;
float min_weak;

float kp_current;
float ki_current;
float max_current;
float min_current;
float kp_speed;
float ki_speed;
float max_speed;
float min_speed;
float id_level;
float id_ratio;

float pso_t[10];
int n_pso=0;

//�ṹ�����
ADC_VOLT_CURRENT_GET Adcget=ADC_VOLT_CURRENT_GET_DEFAULTS;
//PI��������Ӧ�Ľṹ�����


PI_CONTROL PI_Ud=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Uq=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Id=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Iq=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Udn=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Uqn=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Idn=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Iqn=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_PCC=PI_CONTROL_DEFAULTS;


ABC_PARK I_conversion=CLARKE_PARK_DEFAULTS;   //��е���
ABC_PARK U_conversion=CLARKE_PARK_DEFAULTS;   //������˿ڵ�ѹ
//ABC_PARK Io_conversion=CLARKE_PARK_DEFAULTS;   //PCC����
ABC_PARK Uo_conversion=CLARKE_PARK_DEFAULTS;   //PCC��ѹ
ABC_PARK Uref_conversion=CLARKE_PARK_DEFAULTS;   //��ѹ����
ABC_PARK Uout_conversion=CLARKE_PARK_DEFAULTS;   //���Ʋ�
ABC_PARK Uoutn_conversion=CLARKE_PARK_DEFAULTS;
//ABC_PARK test_conversion=CLARKE_PARK_DEFAULTS;   //������
//ABC_PARK test2_conversion=CLARKE_PARK_DEFAULTS;

FILTRATE test_filtrate=FILTRATE_DEFAULTS_20Hz;//������
//FILTRATE Udp_filtrate=FILTRATE_DEFAULTS_20Hz;
//FILTRATE Uqp_filtrate=FILTRATE_DEFAULTS_20Hz;
//FILTRATE Udn_filtrate=FILTRATE_DEFAULTS_20Hz;
//FILTRATE Uqn_filtrate=FILTRATE_DEFAULTS_20Hz;

JIEOU jieou_positive=JIEOU_DEFAULTS;
JIEOU jieou_negative=JIEOU_DEFAULTS;

//DDSRF_PLL test_DDSRF_PLL=DDSRF_PLL_DEFAULTS;//������
DDSRF_PLL I_DDSRF_PLL=DDSRF_PLL_DEFAULTS;
DDSRF_PLL U_DDSRF_PLL=DDSRF_PLL_DEFAULTS;
DDSRF_PLL Uo_DDSRF_PLL=DDSRF_PLL_DEFAULTS;

float a_graph[graphNumber];
float b_graph[graphNumber];
float c_graph[graphNumber];
int n_graph=0;

///��ѹ
float Idp=0;
float Iqp=0;
float Idn=0;
float Iqn=0;

float Udp=0;
float Uqp=0;
float Udn=0;
float Uqn=0;

float Uodp=0;
float Uoqp=0;
float Uodn=0;
float Uoqn=0;
//--------------------------���
float Idpout=0;
float Iqpout=0;
float Idnout=0;
float Iqnout=0;

float Udpout=0;
float Uqpout=0;
float Udnout=0;
float Uqnout=0;

float Uodpout=0;
float Uoqpout=0;
float Uodnout=0;
float Uoqnout=0;

//---------------------------���㲻ƽ���
float VUFout=0;
float VUFpcc=0;
float VUFpccmean=0;
unsigned int n_count=0;
unsigned int n_count1=0;

float Udn_ref=0;//�����ѹ�Ĳο�ֵ ������Ⱥ�㷨����
float Uqn_ref=0;
float Idn_ref=0;
float Iqn_ref=0;
float delt_Udn=0;
float delt_Uqn=0;
float delt_Idn=0;
float delt_Iqn=0;

//�������㷨d���ѹ
                                                                                     //�������㷨q���ѹ
float Ud;
float Uq;
float Id;
float Iq;

float Ud_ref;
float Uq_ref;
float delt_Ud;
float delt_Uq;
//����
float Id_ref;
float Iq_ref;
float delt_Id;
float delt_Iq;
                                                                                //q��������ָ�ֵ
                                                                             //����Ƕ��м����
float theta_fan;                                                                                 //����Ƕȡ�����������任

//�м����
float error;
float gain;

float Ua;
float Ub;
float Uc;
float Ua_ref;
float Ub_ref;
float Uc_ref;


//////////////////////////////����+PI
float P0;
float Q0;
//float kp_current_d;
//float ki_current_d;
//float kp_current_q;
//float ki_current_q;
//float kp_voltage_d;
//float ki_voltage_d;
//float kp_voltage_q;
//float ki_voltage_q;
float kp_current_dqp;
float ki_current_dqp;
float kp_current_dqn;
float ki_current_dqn;
float kp_voltage_dqp;
float ki_voltage_dqp;
float kp_voltage_dqn;
float ki_voltage_dqn;
float kp_pcc_degree;
float ki_pcc_degree;

float w;
float U;
float P;
float Q;
float P_last;
float Q_last;
float PIout_Ud;
float PIout_Uq;
float PIout_Id;
float PIout_Iq;
float PIout_Udn;
float PIout_Uqn;
float PIout_Idn;
float PIout_Iqn;


//spwm

Uint16 Tcmpa;
Uint16 Tcmpb;
Uint16 Tcmpc;
float m_sin_a;
float m_sin_b;
float m_sin_c;

float test_Uout;//������

int i_a=0;
//float   ceshi;

//���ٹ����ж�---����PWM�ж��жϽ��й�����������ѹ������Ƿѹ��������
//��������
Uint16 moder_of_stop;
Uint16 overcurrent_flag;
Uint16 overcurrent_U;//U�������־
Uint16 overcurrent_W;//W�������־
Uint16 overcurrent_V;//V�������־

union FLOAT_COM  Data_get;
union FLOAT_COMF  FData_send;
union FLOAT_COMF  FData_get;
union FLOAT_COMF  PSO_send;
union FLOAT_COMF  PSO_get;

float PSO_g[4]={0,0,0,0};
