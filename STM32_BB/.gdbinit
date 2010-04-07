
file FLASH_RUN/stm32_bb.elf
target remote localhost:3333

# set the target-specific info
set remote hardware-watchpoint-limit 4
set remote hardware-breakpoint-limit 6
monitor gdb_breakpoint_override hard

monitor soft_reset_halt

# enable debug mode
break main
continue
set variable DEBUG_ON = 1
del 1

define can_enable
	set variable SYS_InterruptEnable = 0x0000
	set variable CANController_Control = 1
	call CANController_ControlHandle ()
	set variable CANController_Timing = { 0x03, 0x115 }
	call CANController_TimingHandle()
	set variable CANController_Control = 0
	call CANController_ControlHandle ()
	set variable CANController_TXBuffer = {0x3, 0x80, {12,23,43 , 12,23,23,23,23}}
	set variable SYS_InterruptEnable = 0x000f
end
document can_enable
	Sets CAN timing to 1Mbit
end

define pwr_enable
	set variable PWR_Control = 0xf
	call PWR_ControlHandle ()
end
document pwr_enable
	Enables power managenment module
end
