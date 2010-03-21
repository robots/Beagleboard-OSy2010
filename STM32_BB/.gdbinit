

file FLASH_RUN/stm32_bb.elf

target remote localhost:3333

monitor soft_reset_halt

break main
continue
set variable DEBUG_ON = 1
