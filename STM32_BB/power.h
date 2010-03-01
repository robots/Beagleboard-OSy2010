#ifndef POWER_H_
#define POWER_H_

#define PWR_CTRL_ADC 0x0001
#define PWR_CTRL_PWM 0x0002
#define PWR_CTRL_EN  0x0004 // charger enable
#define PWR_CTRL_ACS 0x0008 // ac select

#define PWR_STAT_ALARM 0x0001
#define PWR_STAT_ACPRE 0x0002

struct pwr_data_t {
	uint16_t i_bat;
	uint16_t i_sys;
	uint16_t v_bat;
	uint16_t v_ac;
} __attribute__ ((packet));

struct pwr_i_set_t {
	uint16_t i_ac;
	uint16_t i_bat;
} __attribute__ ((packet));

extern struct pwr_data_t PWR_Measurement_Data;
extern struct pwr_i_set_t PWR_I_Set;
extern uint16_t PWR_Control;
extern uint16_t PWR_Status;


void PWR_Init();
void PWR_ControlHandle();
void PWR_CurrentHandle();

#endif

