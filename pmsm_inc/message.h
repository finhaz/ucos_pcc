#ifndef __MESSAGE_H__
#define __MESSAGE_H__

//----------------------------------float communication
struct FLOAT_COMMUNICATION_BITS {     // bits  description
	Uint16  MEM1:8;      // 7:0
	Uint16  MEM2:8;   // 15:8

};

union FLOAT_COM{
   int16                all;
   struct FLOAT_COMMUNICATION_BITS   bit;
};

//----------------------------------float communication
struct FLOAT_COMMUNICATION_BITSF {     // bits  description
	Uint16  MEM1:8;      // 7:0
	Uint16  MEM2:8;   // 15:8
	Uint16  MEM3:8;      //23:16
	Uint16  MEM4:8;   // 31:24
};

union FLOAT_COMF {
   float                all;
   struct FLOAT_COMMUNICATION_BITSF   bit;
};


extern void SciRecieve(void);
extern void SciSend(void);
extern void TXdeal(void);
extern void Checkdata(void);
extern void ClrTxbuf(void);
extern void cltran(void);
extern void PSOsend(float U[10]);

#endif
