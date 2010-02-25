#ifndef SYS_H_
#define SYS_H_


#define SYS_INT_CANTXIF  0x0001
#define SYS_INT_CANRX0IF 0x0002
#define SYS_INT_CANRX1IF 0x0004
#define SYS_INT_CANERRIF 0x0008

void SYS_ChangeIntFlag(uint16_t);
void SYS_IntFlagReadHandle(void);
void SYS_IntFlagWriteHandle(void);

#endif

