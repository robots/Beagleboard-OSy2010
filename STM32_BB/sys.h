#ifndef SYS_H_
#define SYS_H_

#define SYS_INT_CANTXIF  0x0001
#define SYS_INT_CANRX0IF 0x0002
#define SYS_INT_CANRX1IF 0x0004
#define SYS_INT_CANERRIF 0x0008

#define SYS_INT_PWRALARM 0x0100
#define SYS_INT_PWRAC    0x0200

#define SYS_RESET_MAGIC  0xBABE

extern uint16_t SYS_InterruptEnable;
extern uint16_t SYS_InterruptFlag;
extern uint16_t SYS_Identifier;
extern uint16_t SYS_Reset;

void SYS_Init();
void SYS_ChangeIntFlag(uint16_t);
void SYS_IntFlagWriteHandle(void);
void SYS_ResetHandler(void);

#endif

