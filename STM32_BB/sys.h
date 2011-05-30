/*
 * External interrupt and system managenment
 *
 * 2010 Michal Demin
 *
 */

#ifndef SYS_H_
#define SYS_H_

/** Can controller mask */
#define SYS_INT_CANMASK  0x00ff
/** Can controller TX interrupt */
#define SYS_INT_CANTXIF  0x0001
/** Can Controller RX interrupt */
#define SYS_INT_CANRX0IF 0x0002
/** Can Controller error interrupt */
#define SYS_INT_CANERRIF 0x0008
/** Can controller reset interrupt */
#define SYS_INT_CANRSTIF 0x0010

/** Power management mask */
#define SYS_INT_PWRMASK  0x0f00
/** Power management Alarm interrupt */
#define SYS_INT_PWRALARM 0x0100
/** Power management AC adaptor interrupt */
#define SYS_INT_PWRAC    0x0200

/** Reset magic value */
#define SYS_RESET_MAGIC  0xBABE
/** System ID magic value */
#define SYS_ID_MAGIC     0xCAFE

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
