# 16 Channel PWM Sinusiodal PWM Controller

![pwm waveforms viewed on an oscilloscope](./doc/img/pwm_oscope.jpg)

An experiment to make a PWM controller in embedded Rust for the STM32F407 Discovery board.

## Usage

### Build

A release build is preferred for the optimizations. Run
```shell
cargo build --release
```
Note that the build may not fit in the processor if the debug config is used.

### Development
Setup OpenOCD and run
```shell
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -l /tmp/openocd.log
```

To run with semihosting,
```shell
gdb-multiarch -q target/thumbv7em-none-eabihf/release/stm32f4-hello --command=.gdbinit
```
and run `c` to continue.

## Resources

This project is a chance for me to learn more about embedded Rust. Here are some helpful resources

### Rust Resources
* [Embedded Rust Page](https://www.rust-lang.org/what/embedded) high-level overview of embedded rust.
* [STM32F4xx HAL Docs](https://docs.rs/stm32f4xx-hal/latest/stm32f4xx_hal/) an abstracted interface to use STM32F4xx features and devices.
* [STM32F4xx HAL Examples](https://github.com/stm32-rs/stm32f4xx-hal/tree/master/examples) playing with these examples are a good place to start.
* [RTIC Book](https://rtic.rs/1/book/en/) RTIC framework for programming with hardware interrupts.

#### Embedded-Friendly Libraries
* [heapless](https://docs.rs/heapless/latest/heapless/) embedded-friendly data structures that don't require dynamic memory allocation.
* [light-cli](https://rudihorn.github.io/light-cli/light_cli/index.html) heapless CLI parser. I used this to make a simple CLI to control the device 
over serial TTL.
* [micromath](https://docs.rs/micromath/latest/micromath/) embedded-friendly math functions library. I used this for LUT generation.

### STM32F4xx Resources
* [STM32F407 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf) 
complete information of how to use the processor and its peripheral devices. I referred to it when learning how to configure
DMA for USART. I also use it for debugging, looking up what the status registers mean.
* [STM32F4 Discovery Board Manual](https://www.st.com/resource/en/user_manual/dm00039084-discovery-kit-with-stm32f407vg-mcu-stmicroelectronics.pdf)
reference for external devices that come with the development board. I referred to this when finding which GPIO blocks are connected to the LEDs. I
also discovered that one pin gave filtered results because it was connected to a bypass cap.

#### STM32F4xx Peripheral Resources
* [UART using DMA](https://stm32f4-discovery.net/2017/07/stm32-tutorial-efficiently-receive-uart-data-using-dma/) useful for understanding serial using DMA
and the tradeoffs of the various configurations.


### Notes

