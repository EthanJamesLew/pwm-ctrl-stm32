#![allow(unsafe_code)]
#![allow(unused_imports)]
#![no_main]
#![no_std]

// signal generator is an rtic app
#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [TIM5])]
mod app {
    // hardware layers imports
    use core::fmt::write;
    use cortex_m_semihosting::{hprint, hprintln};
    use embedded_hal::spi::{Mode, Phase, Polarity};
    use heapless::Vec;
    use panic_semihosting as _;
    use pwm_protocol::command::{PwmCommand, PwmOpCode};
    use pwm_protocol::lut;
    use pwm_protocol::serial_config;
    use stm32f4xx_hal as hal;
    use stm32f4xx_hal::{
        block,
        dma::{
            config::DmaConfig, traits::StreamISR, MemoryToPeripheral, PeripheralToMemory, Stream1,
            StreamsTuple, Transfer,
        },
        gpio::{gpiod::PD12, Alternate, Output, Pin, PushPull},
        interrupt, pac,
        pac::{DMA1, SPI3, USART3},
        prelude::*,
        rng::ErrorKind,
        serial,
        serial::{config::Config, Event::Rxne, Rx, Tx},
        spi::Spi,
        timer::{Ch, Channel, Pwm},
    };
    use systick_monotonic::{Systick, fugit};

    // app constants
    const ARRAY_SIZE: usize = 4;

    // app resources local to specific tasks
    #[local]
    struct Local {
        rx_buffer: Option<&'static mut [u8; ARRAY_SIZE]>,
        count: usize,
    }

    // app resources shared between tasks
    #[shared]
    struct Shared {
        led: PD12<Output<PushPull>>,
        tx_transfer: Tx<USART3, u8>,
        rx_transfer: Transfer<
            Stream1<DMA1>,
            4_u8,
            Rx<USART3>,
            PeripheralToMemory,
            &'static mut [u8; ARRAY_SIZE],
        >,
        pwm0: Pwm<
            pac::TIM1,
            (Ch<0_u8>, Ch<1_u8>, Ch<2_u8>, Ch<3_u8>),
            (
                Pin<'A', 8_u8, Alternate<1_u8>>,
                Pin<'A', 9_u8, Alternate<1_u8>>,
                Pin<'A', 10_u8, Alternate<1_u8>>,
                Pin<'A', 11_u8, Alternate<1_u8>>,
            ),
            1000000_u32,
        >,
        pwm1: Pwm<
            pac::TIM2,
            (Ch<0_u8>, Ch<1_u8>, Ch<2_u8>, Ch<3_u8>),
            (
                Pin<'A', 5_u8, Alternate<1_u8>>,
                Pin<'A', 1_u8, Alternate<1_u8>>,
                Pin<'A', 2_u8, Alternate<1_u8>>,
                Pin<'A', 3_u8, Alternate<1_u8>>,
            ),
            1000000_u32,
        >,
        pwm2: Pwm<
            pac::TIM3,
            (Ch<0_u8>, Ch<1_u8>, Ch<2_u8>, Ch<3_u8>),
            (
                Pin<'A', 6_u8, Alternate<2_u8>>,
                Pin<'A', 7_u8, Alternate<2_u8>>,
                Pin<'B', 0_u8, Alternate<2_u8>>,
                Pin<'B', 1_u8, Alternate<2_u8>>,
            ),
            1000000_u32,
        >,
        pwm3: Pwm<
            pac::TIM4,
            (Ch<0_u8>, Ch<1_u8>, Ch<2_u8>, Ch<3_u8>),
            (
                Pin<'B', 6_u8, Alternate<2_u8>>,
                Pin<'B', 7_u8, Alternate<2_u8>>,
                Pin<'B', 8_u8, Alternate<2_u8>>,
                Pin<'B', 9_u8, Alternate<2_u8>>,
            ),
            1000000_u32,
        >,
        signal_lut: Vec<u16, 512>,
        pwm_config: lut::SignalConfig,
    }

    // RTIC runtime timer -- timing for tasks
    #[monotonic(binds = SysTick, default = true)]
    type SMonotonic = Systick<200_000>; // 1000 Hz / 1 ms granularity

    #[init()]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // configure data watchpoint
        let mut core = cx.core;
        core.DWT.enable_cycle_counter();

        // Initialize the clocks
        let device_peripherals: hal::pac::Peripherals = cx.device;
        let rcc = device_peripherals.RCC;
        let rcc = rcc.constrain();
        let clocks = rcc.cfgr.sysclk(100.MHz()).use_hse(8.MHz()).freeze();

        // RTIC monotonic
        let mono = Systick::new(core.SYST, 100_000_000);

        // define RX/TX pins
        let gpiod = device_peripherals.GPIOD.split();
        let tx_pin = gpiod.pd8.into_alternate();
        let rx_pin = gpiod.pd9.into_alternate();

        // configure serial
        // 115200 baud rate using 8N1 words
        let serial = device_peripherals
            .USART3
            .serial(
                (tx_pin, rx_pin),
                Config::default()
                    .baudrate(serial_config::BAUDRATE.bps())
                    .dma(serial::config::DmaConfig::Rx),
                &clocks,
            )
            .unwrap();
        // get the serial Rx/Tx as their own peripherals
        let (tx, rx) = serial.split();

        // For USART3 Using DMA, use DMA1, Stream 1
        // RM0090 Rev 19, pp 307
        let streams = StreamsTuple::new(device_peripherals.DMA1);
        let rx_stream = streams.1;

        // buffer that the DMA USART3 Rx will populate
        let rx_buffer = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [0; ARRAY_SIZE]).unwrap();

        // configure the DMA and setup the transfer from the peripheral
        // RM0090 Rev 19, pp 322
        // Peripheral-to-Memory, FIFO Mode
        // Priority Medium
        // Memory Increment
        // 1/4 FIFO threshold
        // No Bursts
        // start the DMA immediately
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
        rx_transfer.start(|_rx| {});

        // Initialize the buffer for rx/tx
        let rx_buffer2 = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [0; ARRAY_SIZE]).unwrap();

        let led = gpiod.pd12.into_push_pull_output();

        // PWM Stuff
        let uperiod = ((1.0 / (lut::DEFAULT_FREQ as f32)) * 1E6) as u32;

        // GPIO blocks involved
        let gpioa = device_peripherals.GPIOA.split();
        let gpiob = device_peripherals.GPIOB.split();

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
        let mut pwm0 = device_peripherals
            .TIM1
            .pwm_us(channels0, uperiod.micros(), &clocks);
        let mut pwm1 = device_peripherals
            .TIM2
            .pwm_us(channels1, uperiod.micros(), &clocks);
        let mut pwm2 = device_peripherals
            .TIM3
            .pwm_us(channels2, uperiod.micros(), &clocks);
        let mut pwm3 = device_peripherals
            .TIM4
            .pwm_us(channels3, uperiod.micros(), &clocks);

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

        let max_duty = pwm0.get_max_duty();
        let pwm_config = lut::SignalConfig::default(max_duty);
        let signal_lut = pwm_config.generate_lut().unwrap();

        update_duty::spawn_after(fugit::Duration:: <u64,1,200_000> ::from_ticks(1)).ok();

        // setup the RTIC resources
        (
            Shared {
                led: led,
                tx_transfer: tx,
                rx_transfer: rx_transfer,
                pwm0: pwm0,
                pwm1: pwm1,
                pwm2: pwm2,
                pwm3: pwm3,
                signal_lut: signal_lut,
                pwm_config: pwm_config,
            },
            Local {
                rx_buffer: Some(rx_buffer2),
                count: 0,
            },
            init::Monotonics(mono),
        )
    }

    /// fast modulus - inside of the fast running update loop
    #[inline(always)]
    fn fast_mod(n: usize, d: usize) -> usize {
        n % d
        //n & (d - 1)
    }

    #[task(shared = [pwm0, pwm1, pwm2, pwm3, signal_lut, pwm_config], local=[count])]
    fn update_duty(cx: update_duty::Context) {
        let update_duty::Context { mut shared, local } = cx;

        shared.pwm_config.lock(|config| {
            // compute indices
            let count = *local.count;
        
            // update loop increment (and mod it)
            *local.count += 1;
            if *local.count >= config.table_size {
                *local.count = 0;
            };

            // calculate the table indices
            let (j0, j1, j2, j3, j4, j5, j6, j7, j8, j9, j10, j11, j12, j13, j14, j15) = (
                fast_mod(count + config.shifts[0], config.table_size),
                fast_mod(count + config.shifts[1], config.table_size),
                fast_mod(count + config.shifts[2], config.table_size),
                fast_mod(count + config.shifts[3], config.table_size),
                fast_mod(count + config.shifts[4], config.table_size),
                fast_mod(count + config.shifts[5], config.table_size),
                fast_mod(count + config.shifts[6], config.table_size),
                fast_mod(count + config.shifts[7], config.table_size),
                fast_mod(count + config.shifts[8], config.table_size),
                fast_mod(count + config.shifts[9], config.table_size),
                fast_mod(count + config.shifts[10], config.table_size),
                fast_mod(count + config.shifts[11], config.table_size),
                fast_mod(count + config.shifts[12], config.table_size),
                fast_mod(count + config.shifts[13], config.table_size),
                fast_mod(count + config.shifts[14], config.table_size),
                fast_mod(count + config.shifts[15], config.table_size),
            );

            // set channels
            shared.signal_lut.lock(|signal_lut| {
                shared.pwm0.lock(|pwm| {
                    pwm.set_duty(Channel::C1, signal_lut[j0]);
                    pwm.set_duty(Channel::C2, signal_lut[j1]);
                    pwm.set_duty(Channel::C3, signal_lut[j2]);
                    pwm.set_duty(Channel::C4, signal_lut[j3]);
                });

                shared.pwm1.lock(|pwm| {
                    pwm.set_duty(Channel::C1, signal_lut[j4]);
                    pwm.set_duty(Channel::C2, signal_lut[j5]);
                    pwm.set_duty(Channel::C3, signal_lut[j6]);
                    pwm.set_duty(Channel::C4, signal_lut[j7]);
                });

                shared.pwm2.lock(|pwm| {
                    pwm.set_duty(Channel::C1, signal_lut[j8]);
                    pwm.set_duty(Channel::C2, signal_lut[j9]);
                    pwm.set_duty(Channel::C3, signal_lut[j10]);
                    pwm.set_duty(Channel::C4, signal_lut[j11]);
                });

                shared.pwm3.lock(|pwm| {
                    pwm.set_duty(Channel::C1, signal_lut[j12]);
                    pwm.set_duty(Channel::C2, signal_lut[j13]);
                    pwm.set_duty(Channel::C3, signal_lut[j14]);
                    pwm.set_duty(Channel::C4, signal_lut[j15]);
                });

            });
        });

        // schedule the next update
        shared.pwm_config.lock(|config| {
            let period = ((200_000 as u64) / (config.pwm_freq as u64)) as u64;
            update_duty::spawn_after(fugit::Duration:: <u64,1,200_000> ::from_ticks(period)).ok();
        });
    }

    #[task(binds = DMA1_STREAM1, shared = [rx_transfer, tx_transfer, led, pwm_config, signal_lut], local = [rx_buffer])]
    fn on_receiving(cx: on_receiving::Context) {
        let on_receiving::Context { mut shared, local } = cx;

        // attend to the interrupt based on the status register in the LISR
        // RM0090 Rev 19, pp 325
        if Stream1::<DMA1>::get_fifo_error_flag() {
            shared
                .rx_transfer
                .lock(|serial_dma| serial_dma.clear_fifo_error_interrupt());
        }
        if Stream1::<DMA1>::get_transfer_complete_flag() {
            shared
                .rx_transfer
                .lock(|serial_dma| serial_dma.clear_transfer_complete_interrupt());
            let filled_buffer = shared.rx_transfer.lock(|serial_dma| {
                let (result, _) = serial_dma
                    .next_transfer(local.rx_buffer.take().unwrap())
                    .unwrap();
                result
            });

            // convert the filled buffer to a valid PWM command
            let code_op = PwmOpCode::try_from_u8(filled_buffer[0]);

            // process the command
            match code_op {
                Some(op) => {
                    let command = PwmCommand {
                        op: op,
                        channel: filled_buffer[1],
                        arg: ((filled_buffer[2] as u16) << 8) | filled_buffer[3] as u16,
                    };
                    match command.op {
                        PwmOpCode::Frequency => {
                            shared.pwm_config.lock(|config|{
                                config.pwm_freq = command.arg as u32; 
                            });
                        },
                        PwmOpCode::Phase => {
                            shared.pwm_config.lock(|config|{
                                let channel_idx = command.channel as usize;
                                if channel_idx >= 16 {
                                    hprintln!("invalid channel index");
                                } else {
                                    config.shifts[command.channel as usize] = command.arg as usize;
                                }
                            });
                        },
                        PwmOpCode::Wavetype => {
                            shared.pwm_config.lock(|config|{
                                config.signal_type = (command.arg as u8).into();
                                shared.signal_lut.lock(|lut|{
                                    let lut_op = config.generate_lut();
                                    match lut_op {
                                        Some(l) => *lut = l,
                                        _ => {
                                            hprintln!("invalid wavetype");
                                        }
                                    }
                                });
                            });
                        },
                        PwmOpCode::TableSize => {
                            shared.pwm_config.lock(|config|{
                                if (command.arg as usize) > lut::MAX_TABLE_SIZE {
                                    hprintln!("beyond max table size");
                                } else {
                                    config.table_size = command.arg as usize;
                                    shared.signal_lut.lock(|lut|{
                                        let lut_op = config.generate_lut();
                                        match lut_op {
                                            Some(l) => *lut = l,
                                            _ => {
                                                hprintln!("invalid table size");
                                            }
                                        }
                                    });
                                }
                            });
                        },
                        _ => (), 
                    };
                    //hprintln!("{:?}", command);
                }
                None => {
                    hprintln!("Invalid Code Received")
                }
            }

            // switch out the buffers
            *local.rx_buffer = Some(filled_buffer);
            shared.led.lock(|led| led.toggle());
        }
    }
}
