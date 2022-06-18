//! Demonstrate the use of a blocking `Delay` using the SYST (sysclock) timer.

#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _; // panic handler

use cortex_m_rt::entry;
use cortex_m_semihosting::hio;
use core::fmt::Write;
use stm32f4xx_hal as hal;
use crate::hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let mut stdout = hio::hstdout().map_err(|_| core::fmt::Error).unwrap();

        // I'm on an STM32F4 Discovery, so we need GPIOD 12, 13, 14, and 15 
        let gpiod = dp.GPIOD.split();
        let mut led0 = gpiod.pd12.into_push_pull_output();
        let mut led1 = gpiod.pd13.into_push_pull_output();
        let mut led2 = gpiod.pd14.into_push_pull_output();
        let mut led3 = gpiod.pd15.into_push_pull_output();
    
        // Set up the system clock. We want to run at 48MHz for this one.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        // Create a delay abstraction based on SysTick
        let mut delay = cp.SYST.delay(&clocks);

        loop {
            // On for 1s, off for 1s.
            write!(stdout, "hello world").unwrap();
            led0.set_high();
            delay.delay_ms(100_u32);
            led0.set_low();
            led1.set_high();
            delay.delay_ms(100_u32);
            led1.set_low();
            led2.set_high();
            delay.delay_ms(100_u32);
            led2.set_low();
            led3.set_high();
            delay.delay_ms(100_u32);
            led3.set_low();
        }
    }

    loop {}
}

