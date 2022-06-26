target remote :3333
monitor arm semihosting enable
load
add-symbol-file peripheral-map/ucregview_stm32f407xx.elf 0x0
