#define PackHeadLength 4 //包头长度
#define PackHead 0x00FE  //包头

#include "DSP2833x_Device.h"        // F2837xD Headerfile Include File
#include "global_var.h"

//AD采样
Uint32 AC_Zero[12][8];
Uint32 AC_ZeroMean[12];
Uint16 AC_Zero_yu[12][8];
Uint16 AC_Zero_yuz[12];

Uint16 ADCresult[12] ;    //2
Uint16 ADCZeroCNT0;      //6采样校零用来计数，计600
//Uint16 Rezidue[10];       //用于存放交流较零计算时的余数
int32 ADCresultNEW[12]; //用于存放去偏置之后的数据（可选择减或不减直流偏置）
//int32 ADCresultAmend[10];  //5用于存放修正过后的采样量
Uint16 adcresult15;
//
Uint32 ADCcount;
Uint32 ad_count[3];
int32 Udc;


//sci
unsigned int Switchsystem;
unsigned int RCBUF[24];//RS485 接收缓存器 （系统级通讯协议的24个数据）
unsigned int ReciveRCOUNT=0;//RS485 接收计数器 0~11
unsigned int RC_DataCount=0;   //接收数据计数器
unsigned int TXCOUNT=0;//RS485 发送计数器
unsigned int PSOCOUNT=0;//RS485 发送计数器
unsigned int TXBUF[13];//RS485 发送缓存器
unsigned int PSOBUF[PSONumber];//RS485 发送缓存器
unsigned int flagRC=0;//接收数据结束标志位
unsigned int flagSEND=0;//发送数据标志位
unsigned int PSOSENDF=0;//发送数据标志位
unsigned int RunCommand_L;
unsigned int RunCommand_H;
unsigned int RunCommand;
unsigned int cyclecount;
unsigned int datasum;//发送数据求和
unsigned int datasum1;
unsigned int PackLength;     //数据包长
unsigned int RC_DataBUF[20];  //接收数据缓存器（包括站点号-序列号-命令码-数据高-数据低-校验码）
unsigned int RC_DataCount;   //接收数据计数器
unsigned int SortNumber;     //站点号
unsigned int SerialNumber;   //SCI序号
unsigned int CommandCode;     //SCI命令码
unsigned int CheckCode;      //校验码
unsigned int SendData;       //发送数据
unsigned int SendDataNumber;     //发送数据个数
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
//标志变量




Uint16 break_count;






Uint16 stage;
struct FLAG_REGS FlagRegs={0,0};//标志
struct COUNTER_REGS CounterRegs;//计数



float speed_cankao=0;
float T=0.0002;//开关周期5k
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

//结构体变量
ADC_VOLT_CURRENT_GET Adcget=ADC_VOLT_CURRENT_GET_DEFAULTS;
//PI调节器对应的结构体变量


PI_CONTROL PI_Ud=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Uq=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Id=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Iq=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Udn=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Uqn=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Idn=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_Iqn=PI_CONTROL_DEFAULTS;
PI_CONTROL PI_PCC=PI_CONTROL_DEFAULTS;


ABC_PARK I_conversion=CLARKE_PARK_DEFAULTS;   //电感电流
ABC_PARK U_conversion=CLARKE_PARK_DEFAULTS;   //逆变器端口电压
//ABC_PARK Io_conversion=CLARKE_PARK_DEFAULTS;   //PCC电流
ABC_PARK Uo_conversion=CLARKE_PARK_DEFAULTS;   //PCC电压
ABC_PARK Uref_conversion=CLARKE_PARK_DEFAULTS;   //电压反馈
ABC_PARK Uout_conversion=CLARKE_PARK_DEFAULTS;   //调制波
ABC_PARK Uoutn_conversion=CLARKE_PARK_DEFAULTS;
//ABC_PARK test_conversion=CLARKE_PARK_DEFAULTS;   //测试用
//ABC_PARK test2_conversion=CLARKE_PARK_DEFAULTS;

FILTRATE test_filtrate=FILTRATE_DEFAULTS_20Hz;//测试用
//FILTRATE Udp_filtrate=FILTRATE_DEFAULTS_20Hz;
//FILTRATE Uqp_filtrate=FILTRATE_DEFAULTS_20Hz;
//FILTRATE Udn_filtrate=FILTRATE_DEFAULTS_20Hz;
//FILTRATE Uqn_filtrate=FILTRATE_DEFAULTS_20Hz;

JIEOU jieou_positive=JIEOU_DEFAULTS;
JIEOU jieou_negative=JIEOU_DEFAULTS;

//DDSRF_PLL test_DDSRF_PLL=DDSRF_PLL_DEFAULTS;//测试用
DDSRF_PLL I_DDSRF_PLL=DDSRF_PLL_DEFAULTS;
DDSRF_PLL U_DDSRF_PLL=DDSRF_PLL_DEFAULTS;
DDSRF_PLL Uo_DDSRF_PLL=DDSRF_PLL_DEFAULTS;

float a_graph[graphNumber];
float b_graph[graphNumber];
float c_graph[graphNumber];
int n_graph=0;

///电压
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
//--------------------------输出
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

//---------------------------计算不平衡度
float VUFout=0;
float VUFpcc=0;
float VUFpccmean=0;
unsigned int n_count=0;
unsigned int n_count1=0;

float Udn_ref=0;//负序电压的参考值 由粒子群算法给出
float Uqn_ref=0;
float Idn_ref=0;
float Iqn_ref=0;
float delt_Udn=0;
float delt_Uqn=0;
float delt_Idn=0;
float delt_Iqn=0;

//进估算算法d轴电压
                                                                                     //进估算算法q轴电压
float Ud;
float Uq;
float Id;
float Iq;

float Ud_ref;
float Uq_ref;
float delt_Ud;
float delt_Uq;
//电流
float Id_ref;
float Iq_ref;
float delt_Id;
float delt_Iq;
                                                                                //q轴电流积分赋值
                                                                             //估算角度中间变量
float theta_fan;                                                                                 //估算角度——参与坐标变换

//中间变量
float error;
float gain;

float Ua;
float Ub;
float Uc;
float Ua_ref;
float Ub_ref;
float Uc_ref;


//////////////////////////////功率+PI
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

float test_Uout;//测试用

int i_a=0;
//float   ceshi;

//快速故障判断---用于PWM中断判断进行过流保护，过压保护，欠压保护策略
//过流保护
Uint16 moder_of_stop;
Uint16 overcurrent_flag;
Uint16 overcurrent_U;//U相过流标志
Uint16 overcurrent_W;//W相过流标志
Uint16 overcurrent_V;//V相过流标志

union FLOAT_COM  Data_get;
union FLOAT_COMF  FData_send;
union FLOAT_COMF  FData_get;
union FLOAT_COMF  PSO_send;
union FLOAT_COMF  PSO_get;

float PSO_g[4]={0,0,0,0};
