#![deny(unsafe_code)]
#![allow(unused_imports)]
#![no_main]
#![no_std]

// signal generator is an rtic app
#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [TIM5])]
mod app {
    /// hardware layers imports
    use embedded_hal::spi::{Mode, Phase, Polarity};
    use stm32f4xx_hal::{
        spi::{Spi, Tx, Rx},
        pac::{DMA1, SPI3},
        gpio::{gpiod::PD12, Output, PushPull},
        dma::{
            StreamsTuple, config::DmaConfig, 
            Transfer, PeripheralToMemory, 
            MemoryToPeripheral, Stream0, Stream5,
            traits::StreamISR
        },
        prelude::*,
    };
    use panic_semihosting as _;
    use systick_monotonic::*;
    use stm32f4xx_hal as hal;

    /// app constants
    const ARRAY_SIZE: usize = 3;

    type TxTransfer =
        Transfer<Stream5<DMA1>, 0, Tx<SPI3>, MemoryToPeripheral, &'static mut [u8; ARRAY_SIZE]>;

    type RxTransfer =
        Transfer<Stream0<DMA1>, 0, Rx<SPI3>, PeripheralToMemory, &'static mut [u8; ARRAY_SIZE]>;
    
    #[local]
    struct Local {
        tx_buffer: Option<&'static mut [u8; ARRAY_SIZE]>,
        rx_buffer: Option<&'static mut [u8; ARRAY_SIZE]>,
    }

    #[shared]
    struct Shared {
        led: PD12<Output<PushPull>>,
        tx_transfer: TxTransfer,
        rx_transfer: RxTransfer,
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
        let clocks = rcc.cfgr.sysclk(100.MHz()).pclk1(36.MHz()).freeze();

        // RTIC monotonic
        let mono = Systick::new(core.SYST, 100_000_000);
        
        // Initialize the SPI
        let gpiob = device_peripherals.GPIOB;
        let spi = device_peripherals.SPI3;
        let gpiob = gpiob.split();

        // gpio SPI pins
        let sck = gpiob.pb3.into_alternate();
        let miso = gpiob.pb4.into_alternate();
        let mosi = gpiob.pb5.into_alternate();

        // SPI structs and get SPI Tx/Rx
        let mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };

        let spi3 = Spi::new_slave(spi, (sck, miso, mosi), mode, 8.MHz(), &clocks);

        let (tx, rx) = spi3.use_dma().txrx();

        // Initialize the DMA
        let streams = StreamsTuple::new(device_peripherals.DMA1);
        let tx_stream = streams.5;
        let rx_stream = streams.0;

        let rx_buffer = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [0; ARRAY_SIZE]).unwrap();
        let tx_buffer = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [1,2,3]).unwrap();

        let mut rx_transfer = Transfer::init_peripheral_to_memory(
            rx_stream,
            rx,
            rx_buffer,
            None,
            DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );

        let mut tx_transfer = Transfer::init_memory_to_peripheral(
            tx_stream,
            tx,
            tx_buffer,
            None,
            DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );

        rx_transfer.start(|_rx| {});
        tx_transfer.start(|_tx| {});

        // Initialize the buffer for rx/tx
        let rx_buffer2 = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [0; ARRAY_SIZE]).unwrap();
        let tx_buffer2 = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [4,5,6]).unwrap();

        let gpiod = device_peripherals.GPIOD.split();
        let mut led = gpiod.pd12.into_push_pull_output();

        (Shared {
            led: led,
            tx_transfer: tx_transfer,
            rx_transfer: rx_transfer
        }, Local {
            rx_buffer: Some(tx_buffer2),
            tx_buffer: Some(rx_buffer2)
        }, init::Monotonics(mono))
    }

    #[task(binds = DMA1_STREAM0, shared = [rx_transfer, led], local = [rx_buffer])]
    fn on_receiving(cx: on_receiving::Context) {
        let on_receiving::Context { mut shared, local } = cx;
        if Stream0::<DMA1>::get_fifo_error_flag() {
            shared
                .rx_transfer
                .lock(|spi_dma| spi_dma.clear_fifo_error_interrupt());
        }
        if Stream0::<DMA1>::get_transfer_complete_flag() {
            shared
                .rx_transfer
                .lock(|spi_dma| spi_dma.clear_transfer_complete_interrupt());
            let filled_buffer = shared.rx_transfer.lock(|spi_dma| {
                let (result, _) = spi_dma
                    .next_transfer(local.rx_buffer.take().unwrap())
                    .unwrap();
                result
            });
            match filled_buffer[0] {
                1 => shared.led.lock(|led| led.set_low()),
                _ => shared.led.lock(|led| led.set_high()),
            }
            *local.rx_buffer = Some(filled_buffer);
        }
    }

}