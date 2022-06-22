#![allow(unsafe_code)]
#![allow(unused_imports)]
#![no_main]
#![no_std]

// signal generator is an rtic app
#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [TIM5])]
mod app {
    /// hardware layers imports
    use embedded_hal::spi::{Mode, Phase, Polarity};
    use stm32f4xx_hal::{
        spi::{Spi},
        pac::{DMA1, SPI3, USART3},
        gpio::{gpiod::PD12, Output, PushPull},
        dma::{
            StreamsTuple, config::DmaConfig, 
            Transfer, PeripheralToMemory, 
            MemoryToPeripheral, Stream0, Stream5,
            traits::StreamISR
        },
        interrupt,
        serial::{config::Config, Tx, Rx, Event::Rxne},
        prelude::*,
        pac,
    };
    use panic_semihosting as _;
    use systick_monotonic::*;
    use stm32f4xx_hal as hal;

    /// app constants
    const ARRAY_SIZE: usize = 3;

    #[local]
    struct Local {
        tx_buffer: Option<&'static mut [u8; ARRAY_SIZE]>,
        rx_buffer: Option<&'static mut [u8; ARRAY_SIZE]>,
    }

    #[shared]
    struct Shared {
        led: PD12<Output<PushPull>>,
        led_state: bool,
        tx_transfer: Tx<USART3, u16>,
        rx_transfer: Rx<USART3, u16>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SMonotonic = Systick<1000>; // 1000 Hz / 1 ms granularity

    #[init()]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // configure data watchpoint
        let mut core = cx.core;
        core.DWT.enable_cycle_counter();

        // Initialize the clocks
        let device_peripherals: hal::pac::Peripherals = cx.device;
        let rcc = device_peripherals.RCC;
        let rcc = rcc.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();

        // RTIC monotonic
        let mono = Systick::new(core.SYST, 100_000_000);

        // define RX/TX pins
        let gpiod = device_peripherals.GPIOD.split();
        let tx_pin = gpiod.pd8.into_alternate();
        let rx_pin = gpiod.pd9.into_alternate();

        // configure serial
        let mut serial = device_peripherals 
            .USART3
            .serial(
                (tx_pin, rx_pin),
                Config::default().baudrate(9600.bps()).wordlength_9(),
                &clocks,
            )
            .unwrap()
            .with_u16_data();
  
        //serial.listen(Rxne);

        let (mut tx, mut rx) = serial.split();

        // Initialize the buffer for rx/tx
        let rx_buffer2 = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [0; ARRAY_SIZE]).unwrap();
        let tx_buffer2 = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [4,5,6]).unwrap();

        let mut led = gpiod.pd12.into_push_pull_output();

        rx.listen_idle();
        //rx.listen_idle();
        
        pac::NVIC::unpend(pac::Interrupt::USART3);
        
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::USART3);
        }

        (Shared {
            led: led,
            led_state: false,
            tx_transfer: tx,
            rx_transfer: rx
        }, Local {
            rx_buffer: Some(tx_buffer2),
            tx_buffer: Some(rx_buffer2)
        }, init::Monotonics(mono))
    }

    #[task(binds = USART3, shared = [rx_transfer, tx_transfer, led, led_state], local = [rx_buffer])]
    fn on_receiving(cx: on_receiving::Context) {
        let on_receiving::Context { mut shared, local } = cx;
        shared.led.lock(|led| led.toggle());
        shared.rx_transfer.lock(|rx| rx.clear_idle_interrupt());
    }

}