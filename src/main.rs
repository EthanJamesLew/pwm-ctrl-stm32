#![deny(unsafe_code)]
#![deny(warnings)]
#![allow(unused_imports)]
#![no_main]
#![no_std]

// our signal generator is an rtic app
#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [TIM5])]
mod app {
    /// hardware layers imports
    use embedded_hal::spi::{Mode, Phase, Polarity};
    use hal::{
        prelude::*,
    };
    use panic_semihosting as _;
    use systick_monotonic::*;
    use stm32f4xx_hal as hal;
    
    #[local]
    struct Local {}

    #[shared]
    struct Shared {}

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<1000>; // 1000 Hz / 1 ms granularity

    #[init()]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut core = cx.core;
        core.DWT.enable_cycle_counter();

        let device_peripherals: hal::pac::Peripherals = cx.device;

        let rcc = device_peripherals.RCC;
        let rcc = rcc.constrain();
        let _clocks = rcc.cfgr.sysclk(100.MHz()).pclk1(36.MHz()).freeze();

        let mono = Systick::new(core.SYST, 100_000_000); 

        (Shared {}, Local {}, init::Monotonics(mono))
    }
}