/*
 * Power Managenment for STM32 family processors
 *
 * 2010 Michal Demin
 *
 */

#ifndef POWER_H_
#define POWER_H_

#define PWR_CTRL_ADC 0x0001 /* Enable ADC sampling */
#define PWR_CTRL_PWM 0x0002 /* Enable PWM output for Current setting */
#define PWR_CTRL_EN  0x0004 /* Charger enable 1 = ENABLED */
#define PWR_CTRL_ACS 0x0008 /* AC select 1 = AC, 0 = BAT */

#define PWR_STAT_ALARM 0x0001 /* ALARM state */
#define PWR_STAT_ACPRE 0x0002 /* AC present */

struct pwr_data_t {
	uint16_t i_bat;
	uint16_t i_sys;
	uint16_t v_bat;
	uint16_t v_ac;
} __attribute__ ((packed));

struct pwr_i_set_t {
	uint16_t i_ac;
	uint16_t i_bat;
} __attribute__ ((packed));

extern volatile struct pwr_data_t PWR_Measurement_Data;
extern volatile struct pwr_i_set_t PWR_I_Set;
extern volatile uint16_t PWR_Control;
extern volatile uint16_t PWR_Status;


void PWR_Init();
void PWR_ControlHandle();
void PWR_CurrentHandle();

#endif
