#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "platform.h"
#include "stm32f10x.h"

#include "cancontroller.h"

void CANController_ISR(uint8_t cmd) {
	if (cmd & CAN_CMD_WRITE) { 
		switch (
	} else { // read command
		
	}
}

