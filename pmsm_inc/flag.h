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

//----------------定义寄存器----------------------------------------
//故障标志寄存器---用于保护
struct  FLAGFAULT_BITS {

        Uint16 overcurrent:1;      /*过流*/
        Uint16 overload :1 ;       //过载
        Uint16 highvoltage_dc:1;   /*直流侧过压*/
        Uint16 lowvoltage_dc:1;    /*直流侧欠压*/
        Uint16 rsvd1:1;
        //Uint16 tabanfault:1;       /*踏板信号故障*/
        Uint16 hightemcircuit:1;   /*电路板温度过高*/
        Uint16 hightemmotor:1;     /*电机温度过高*/
        Uint16 ocfault:1;          /*硬件故障*/
        Uint16 rsvd2:8;

};
//
//系统运行标志寄存器
struct  FLAGSYSTEM_BITS {

        Uint16 sysonoff:1;     //运行与否命令1-开机 0-关机
        Uint16 AC0CheckFinished:1;//采样校零结束标志位
        Uint16 rotatdircww:1;     //正转-1 反转-0
        Uint16 motor_drive_medel:1;     //反转
        Uint16 Changphase:1;    //
        Uint16 eeprom_w:1;      //写eeprom标志位
        Uint16 faultoccur:1;       //极端故障标志位，表示要故障停机
        Uint16 if_start:1;//预定位与起动模式中的定位0和起动1标志
        Uint16 pre_dir_sign:1;
        Uint16 mars_sign:1;
        Uint16 rsvd1:6;
};
//



//---------------定义共同体--------------------------------------------
//故障标志共同体
union FLAGFAULT_REG {

   Uint16        all;
   struct  FLAGFAULT_BITS bit;
};
//
//系统运行标志共同体
union FLAGSYSTEM_REG {

   Uint16        all;
   struct  FLAGSYSTEM_BITS bit;
};
//
/*
//采样共同体
union ADCDEAL_REG{
	Uint16        all;
	struct ADCDEAL_BITS bit;
};*/
//----------------定义结构体--------------------------------------------
//标志结构体
struct FLAG_REGS{
	union   FLAGFAULT_REG  flagfault; //故障标志
	union   FLAGSYSTEM_REG flagsystem;
};
extern  struct FLAG_REGS FlagRegs;
//extern volatile struct FLAG_REGS FlagRegs;
//
/*
//采样结构体
struct ADCDEALRES_REGS{
	union   ADCDEAL_REG   adcdeal;
};
extern  struct ADCDEALRES_REGS AdcdealRegs;
*/
//extern volatile struct ADCDEALRES_REGS AdcdealRegs;
//

//-------------------------------计数位-----------------------------------//
struct  COUNT_BITS {

        Uint16 ADC_ZeroCNTloop:4;     //采样循环计数
        Uint16 rsvd1:4;
        Uint16 rsvd2:1;
        Uint16 rsvd3:7;
};
union COUNT_REG {

   Uint16        all;
   struct  COUNT_BITS bit;
};
//计数寄存器
struct COUNTER_REGS {
	union    COUNT_REG  count; //故障标志
	Uint16  resvd1;
};
extern  struct COUNTER_REGS CounterRegs;

//
#ifdef __cplusplus
}
#endif /* extern "C" */
#endif /* FLAG_H_ */
