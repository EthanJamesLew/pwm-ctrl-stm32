//! Demonstrate the use of a blocking `Delay` using the SYST (sysclock) timer.

#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _; // panic handler

use cortex_m_rt::entry;
use embedded_hal::spi::{Mode, Phase, Polarity};
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
        
        // Set up the system clock. We want to run at 48MHz for this one.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.freeze();


        // I'm on an STM32F4 Discovery, so we need GPIOD 12, 13, 14, and 15 
        let gpiod = dp.GPIOD.split();
        let mut led0 = gpiod.pd12.into_push_pull_output();
        let mut led1 = gpiod.pd13.into_push_pull_output();
        let mut led2 = gpiod.pd14.into_push_pull_output();
        let mut led3 = gpiod.pd15.into_push_pull_output();

        // TODO: into_alternate resolved correctly?
        let gpioa = dp.GPIOA.split();
        let gpiosck = gpioa.pa5.into_alternate::<5_u8>();
        let gpiomiso = gpioa.pa6.into_alternate::<5_u8>();
        let gpiomosi = gpioa.pa7.into_alternate::<5_u8>().internal_pull_up(true);

        let gpioe = dp.GPIOE.split();
        let mut gpiocs = gpioe.pe3.into_push_pull_output();

        let mode = Mode {
            polarity: Polarity::IdleHigh,
            phase: Phase::CaptureOnFirstTransition,
        };

        //write!(stdout, "Starting SPI").unwrap();
        let mut spi1 = dp.SPI1.spi_bidi((gpiosck, gpiomiso, gpiomosi), mode, 1.MHz(), &clocks);
        //let mut spi1 = hal::spi::Spi::new(dp.SPI1, (gpiosck, gpiomiso, gpiomosi), mode, 5.MHz(), &clocks);
        spi1.enable(true);

        // Create a delay abstraction based on SysTick
        let mut delay = cp.SYST.delay(&clocks);
        
        gpiocs.set_high();
        delay.delay_ms(100_u32);
        write!(stdout, "Starting Main Loop").unwrap();
        let data = [0xD8, 0x00];
        loop {
            // On for 1s, off for 1s.
            gpiocs.set_low();
            delay.delay_us(1_u32);
            spi1.write(&data).unwrap();
            //let r = spi1.read().unwrap();
            //write!(stdout, "{}\n", r).unwrap();
            gpiocs.set_high();

            //led0.set_high();
            delay.delay_ms(1_u32);
            //led0.set_low();
            //led1.set_high();
            //delay.delay_ms(100_u32);
            //led1.set_low();
            //led2.set_high();
            //delay.delay_ms(100_u32);
            //led2.set_low();
            //led3.set_high();
            //delay.delay_ms(100_u32);
            //led3.set_low();
        }
    }

    loop {}
}

