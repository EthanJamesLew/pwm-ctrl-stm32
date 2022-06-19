# 16 Channel PWM Sinusiodal PWM Controller

An experiment to make a PWM controller in embedded Rust for the STM32F407 Discovery board.

## Usage

### Build

A release build is preferred for the optimizations. Run
```shell
cargo build --release
```

### Development
Setup OpenOCD and run
```shell
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -l /tmp/openocd.log
```

To run with semihosting,
```shell
gdb-multiarch -q target/thumbv7em-none-eabihf/release/stm32f4-hello --command=test.gdb
```
and run `c` to continue.