#![deny(unsafe_code)]
#![no_main]
#![no_std]
/// STM32F407 16 Channel PWM Code
// Halt on panic
use panic_halt as _;

use core::f32::consts::FRAC_PI_2;
use cortex_m_rt::entry;
use micromath::F32Ext;
use stm32f4xx_hal::{pac, prelude::*, timer::Channel};

/// program constants
const PWM_FREQ: u32 = 15_360;
const TABLE_SIZE: usize = 256;
const SHIFT: usize = 0;

/// fast modulus - inside of the fast running update loop
#[inline(always)]
fn fast_mod(n: usize, d: usize) -> usize {
    n & (d - 1)
}

/// 16 Channel PWM
#[entry]
fn main() -> ! {
    if let Some(dp) = pac::Peripherals::take() {
        // Set up the system clock and timing
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();
        let uperiod = ((1.0 / (PWM_FREQ as f32)) * 1E6) as u32;

        // GPIO blocks involved
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();

        // channels (gpio) that we will use for the signal
        let channels0 = (
            gpioa.pa8.into_alternate(),
            gpioa.pa9.into_alternate(),
            gpioa.pa10.into_alternate(),
            gpioa.pa11.into_alternate(),
        );
        let channels1 = (
            gpioa.pa5.into_alternate(),
            gpioa.pa1.into_alternate(),
            gpioa.pa2.into_alternate(),
            gpioa.pa3.into_alternate(),
        );
        let channels2 = (
            gpioa.pa6.into_alternate(),
            gpioa.pa7.into_alternate(),
            gpiob.pb0.into_alternate(),
            gpiob.pb1.into_alternate(),
        );
        let channels3 = (
            gpiob.pb6.into_alternate(),
            gpiob.pb7.into_alternate(),
            gpiob.pb8.into_alternate(),
            gpiob.pb9.into_alternate(),
        );

        // build PWM HAL interfaces
        let mut pwm0 = dp.TIM1.pwm_us(channels0, uperiod.micros(), &clocks);
        let mut pwm1 = dp.TIM2.pwm_us(channels1, uperiod.micros(), &clocks);
        let mut pwm2 = dp.TIM3.pwm_us(channels2, uperiod.micros(), &clocks);
        let mut pwm3 = dp.TIM4.pwm_us(channels3, uperiod.micros(), &clocks);

        let mut counter = dp.TIM5.counter_us(&clocks);
        let max_duty = pwm0.get_max_duty();

        // build the PWM LUT
        const N: usize = TABLE_SIZE;
        let mut sin_a = [0_u16; N + 1];

        // fill sinus array
        let a = (4.0 * FRAC_PI_2) / (N as f32);
        for (i, b) in sin_a.iter_mut().enumerate() {
            let angle = a * (i as f32);
            *b = ((angle.sin() / 2.0 + 0.500) * (max_duty as f32)) as u16;
        }

        // counter will be the loop update time
        counter.start(uperiod.micros()).unwrap();

        // enable the channels
        pwm0.enable(Channel::C1);
        pwm1.enable(Channel::C1);
        pwm2.enable(Channel::C1);
        pwm3.enable(Channel::C1);

        pwm0.enable(Channel::C2);
        pwm1.enable(Channel::C2);
        pwm2.enable(Channel::C2);
        pwm3.enable(Channel::C2);

        pwm0.enable(Channel::C3);
        pwm1.enable(Channel::C3);
        pwm2.enable(Channel::C3);
        pwm3.enable(Channel::C3);

        pwm0.enable(Channel::C4);
        pwm1.enable(Channel::C4);
        pwm2.enable(Channel::C4);
        pwm3.enable(Channel::C4);

        let mut i = 0;
        loop {
            nb::block!(counter.wait()).unwrap();

            // compute indices
            let (j0, j1, j2, j3) = (
                fast_mod(i, N),
                fast_mod(i + SHIFT, N),
                fast_mod(i + 2 * SHIFT, N),
                fast_mod(i + 3 * SHIFT, N),
            );

            // set channels
            pwm0.set_duty(Channel::C1, sin_a[j0]);
            pwm0.set_duty(Channel::C2, sin_a[j1]);
            pwm0.set_duty(Channel::C3, sin_a[j2]);
            pwm0.set_duty(Channel::C4, sin_a[j3]);

            pwm1.set_duty(Channel::C1, sin_a[j0]);
            pwm1.set_duty(Channel::C2, sin_a[j1]);
            pwm1.set_duty(Channel::C3, sin_a[j2]);
            pwm1.set_duty(Channel::C4, sin_a[j3]);

            pwm2.set_duty(Channel::C1, sin_a[j0]);
            pwm2.set_duty(Channel::C2, sin_a[j1]);
            pwm2.set_duty(Channel::C3, sin_a[j2]);
            pwm2.set_duty(Channel::C4, sin_a[j3]);

            pwm3.set_duty(Channel::C1, sin_a[j0]);
            pwm3.set_duty(Channel::C2, sin_a[j1]);
            pwm3.set_duty(Channel::C3, sin_a[j2]);
            pwm3.set_duty(Channel::C4, sin_a[j3]);

            // update loop increment (and mod it)
            i += 1;
            if i == N {
                i -= N;
            }
        }
    }

    loop {
        cortex_m::asm::nop();
    }
}
