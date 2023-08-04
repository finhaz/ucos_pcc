#ifndef __BSP__
#define __BSP__

interrupt void scirxintab_isr(void);
interrupt void xint1_isr(void);
interrupt void adca1_interrupt_isr(void);
extern void ConfigureADC(void);
extern void InitSciGpio_a(void);
extern void ConfigureEPwm(void);
extern void ConfigureSci(void);
extern void InitSciParameter(void);
extern void InitBoardGpio(void);
extern void ConfigureXint(void);
extern void InitUseGpio(void);

#endif
