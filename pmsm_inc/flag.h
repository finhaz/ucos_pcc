/*
 * flag.h
 *
 *  Created on: 2017-1-6
 *      Author: naiyangui
 */

#ifndef FLAG_H_
#define FLAG_H_

#ifdef __cplusplus
extern "C" {
#endif

//----------------����Ĵ���----------------------------------------
//���ϱ�־�Ĵ���---���ڱ���
struct  FLAGFAULT_BITS {

        Uint16 overcurrent:1;      /*����*/
        Uint16 overload :1 ;       //����
        Uint16 highvoltage_dc:1;   /*ֱ�����ѹ*/
        Uint16 lowvoltage_dc:1;    /*ֱ����Ƿѹ*/
        Uint16 rsvd1:1;
        //Uint16 tabanfault:1;       /*̤���źŹ���*/
        Uint16 hightemcircuit:1;   /*��·���¶ȹ���*/
        Uint16 hightemmotor:1;     /*����¶ȹ���*/
        Uint16 ocfault:1;          /*Ӳ������*/
        Uint16 rsvd2:8;

};
//
//ϵͳ���б�־�Ĵ���
struct  FLAGSYSTEM_BITS {

        Uint16 sysonoff:1;     //�����������1-���� 0-�ػ�
        Uint16 AC0CheckFinished:1;//����У�������־λ
        Uint16 rotatdircww:1;     //��ת-1 ��ת-0
        Uint16 motor_drive_medel:1;     //��ת
        Uint16 Changphase:1;    //
        Uint16 eeprom_w:1;      //дeeprom��־λ
        Uint16 faultoccur:1;       //���˹��ϱ�־λ����ʾҪ����ͣ��
        Uint16 if_start:1;//Ԥ��λ����ģʽ�еĶ�λ0����1��־
        Uint16 pre_dir_sign:1;
        Uint16 mars_sign:1;
        Uint16 rsvd1:6;
};
//



//---------------���干ͬ��--------------------------------------------
//���ϱ�־��ͬ��
union FLAGFAULT_REG {

   Uint16        all;
   struct  FLAGFAULT_BITS bit;
};
//
//ϵͳ���б�־��ͬ��
union FLAGSYSTEM_REG {

   Uint16        all;
   struct  FLAGSYSTEM_BITS bit;
};
//
/*
//������ͬ��
union ADCDEAL_REG{
	Uint16        all;
	struct ADCDEAL_BITS bit;
};*/
//----------------����ṹ��--------------------------------------------
//��־�ṹ��
struct FLAG_REGS{
	union   FLAGFAULT_REG  flagfault; //���ϱ�־
	union   FLAGSYSTEM_REG flagsystem;
};
extern  struct FLAG_REGS FlagRegs;
//extern volatile struct FLAG_REGS FlagRegs;
//
/*
//�����ṹ��
struct ADCDEALRES_REGS{
	union   ADCDEAL_REG   adcdeal;
};
extern  struct ADCDEALRES_REGS AdcdealRegs;
*/
//extern volatile struct ADCDEALRES_REGS AdcdealRegs;
//

//-------------------------------����λ-----------------------------------//
struct  COUNT_BITS {

        Uint16 ADC_ZeroCNTloop:4;     //����ѭ������
        Uint16 rsvd1:4;
        Uint16 rsvd2:1;
        Uint16 rsvd3:7;
};
union COUNT_REG {

   Uint16        all;
   struct  COUNT_BITS bit;
};
//�����Ĵ���
struct COUNTER_REGS {
	union    COUNT_REG  count; //���ϱ�־
	Uint16  resvd1;
};
extern  struct COUNTER_REGS CounterRegs;

//
#ifdef __cplusplus
}
#endif /* extern "C" */
#endif /* FLAG_H_ */
