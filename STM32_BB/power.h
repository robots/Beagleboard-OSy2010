#ifndef POWER_H_
#define POWER_H_


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

#endif

