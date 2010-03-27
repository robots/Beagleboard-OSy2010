/*
 * External interrupt and system managenment
 *
 * 2010 Michal Demin
 *
 */

#ifndef SYS_H_
#define SYS_H_

#define SYS_INT_CANMASK  0x00ff
#define SYS_INT_CANTXIF  0x0001
#define SYS_INT_CANRX0IF 0x0002
#define SYS_INT_CANRX1IF 0x0004
#define SYS_INT_CANERRIF 0x0008

#define SYS_INT_PWRMASK  0x0f00
#define SYS_INT_PWRALARM 0x0100
#define SYS_INT_PWRAC    0x0200

#define SYS_RESET_MAGIC  0xBABE
#define SYS_ID_MAGIC     0xCAFE

// debug
extern uint32_t DEBUG_ON;

extern volatile uint16_t SYS_InterruptEnable;
extern volatile uint16_t SYS_InterruptFlag;
extern const uint16_t SYS_Identifier;
extern volatile uint16_t SYS_Reset;

void SYS_Init();
void SYS_SetIntFlag(uint16_t);
void SYS_ClrIntFlag(uint16_t);
void SYS_IntFlagWriteHandle(void);
void SYS_ResetHandler(void);

#endif

