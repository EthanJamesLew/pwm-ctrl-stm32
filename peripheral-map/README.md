# Peripheral Register Map

This folder contains an `.elf` file that has symbols you can load in GDB to view register
values of *peripherals* for the STM32F4xx. For example, if you're debugging a USART interrupt,
you can run 

```
(gdb) b DMA1_STREAM1
Breakpoint 1 at 0x800077e
(gdb) c
Continuing.
Note: automatically using hardware breakpoints for read-only addresses.

Breakpoint 1, 0x0800077e in DMA1_STREAM1 ()
(gdb) p/x DMA1
$2 = {LISR = 0xc00, HISR = 0x0, LIFCR = 0x0, HIFCR = 0x0}
(gdb) p/x DMA2
$3 = {LISR = 0x0, HISR = 0x0, LIFCR = 0x0, HIFCR = 0x0}
```

In this example, we can see the low interrupt status register of the DMA1 device and its non-default status.

## Setup

Make sure to have the toolchain installed (you may not necessarily if you're using embedded rust).
```shell
sudo apt install gcc-arm-none-eabi
```

Build the [ucRegView](https://github.com/maximevince/ucregview),
```shell
git clone https://github.com/maximevince/ucregview.git
cd ucregview
make TARGET=stm32f407xx
```