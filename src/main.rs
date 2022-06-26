#![allow(unsafe_code)]
#![allow(unused_imports)]
#![no_main]
#![no_std]

// signal generator is an rtic app
#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [TIM5])]
mod app {
    use core::fmt::write;

    use cortex_m_semihosting::{hprint, hprintln};
    /// hardware layers imports
    use embedded_hal::spi::{Mode, Phase, Polarity};
    use heapless::String;
    use panic_semihosting as _;
    use stm32f4xx_hal as hal;
    use stm32f4xx_hal::{
        block,
        dma::{
            config::DmaConfig, traits::StreamISR, MemoryToPeripheral, PeripheralToMemory, Stream1,
            Stream5, StreamsTuple, Transfer,
        },
        gpio::{gpiod::PD12, Output, PushPull},
        interrupt, pac,
        pac::{DMA1, SPI3, USART3},
        prelude::*,
        rng::ErrorKind,
        serial,
        serial::{config::Config, Event::Rxne, Rx, Tx},
        spi::Spi,
    };
    use systick_monotonic::*;

    /// app constants
    const ARRAY_SIZE: usize = 3;
    const STRING_SIZE: usize = 64;
    type HString = String<STRING_SIZE>;

    #[local]
    struct Local {
        rx_buffer: Option<&'static mut [u8; ARRAY_SIZE]>,
    }

    #[shared]
    struct Shared {
        led: PD12<Output<PushPull>>,
        led_state: bool,
        tx_transfer: Tx<USART3, u8>,
        rx_transfer: Transfer<
            Stream1<DMA1>,
            4_u8,
            Rx<USART3>,
            PeripheralToMemory,
            &'static mut [u8; ARRAY_SIZE],
        >,
        command: HString,
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
        let clocks = rcc.cfgr.sysclk(168.MHz()).use_hse(8.MHz()).freeze();

        // RTIC monotonic
        let mono = Systick::new(core.SYST, 100_000_000);

        // define RX/TX pins
        let gpiod = device_peripherals.GPIOD.split();
        let tx_pin = gpiod.pd8.into_alternate();
        let rx_pin = gpiod.pd9.into_alternate();

        // configure serial
        let serial = device_peripherals
            .USART3
            .serial(
                (tx_pin, rx_pin),
                Config::default()
                    .baudrate(115_200.bps())
                    .wordlength_9()
                    .dma(serial::config::DmaConfig::Rx),
                &clocks,
            )
            .unwrap();

        //serial.listen(Rxne);

        let (tx, mut rx) = serial.split();
        //rx.listen_idle();

        let streams = StreamsTuple::new(device_peripherals.DMA1);
        let rx_stream = streams.1;

        let rx_buffer = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [0; ARRAY_SIZE]).unwrap();

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
        let rx_buffer2 = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [4,5,6]).unwrap();

        let led = gpiod.pd12.into_push_pull_output();

        (
            Shared {
                led: led,
                led_state: false,
                tx_transfer: tx,
                rx_transfer: rx_transfer,
                command: String::<STRING_SIZE>::from(""),
            },
            Local {
                rx_buffer: Some(rx_buffer2),
            },
            init::Monotonics(mono),
        )
    }

    #[inline(always)]
    fn write_char(tx: &mut Tx<USART3, u8>, c: char) {
        let terr = block!(tx.write(c as u8));
        match terr {
            Ok(_) => (),
            Err(e) => hprintln!("UART Tx Error {:?}", e),
        }
    }

    #[inline(always)]
    fn read_char(rx: &mut Rx<USART3, u8>) -> Result<u8, serial::Error> {
        let res = block!(rx.read());
        match res {
            Ok(_) => (),
            Err(e) => hprintln!("Uart Rx Error {:?}", e),
        };
        res
    }

    #[task(binds = USART3, shared = [tx_transfer, led, led_state, command], local = [])]
    fn on_receiving(cx: on_receiving::Context) {
        let on_receiving::Context { mut shared } = cx;
        shared.led.lock(|led| led.toggle());
        shared.tx_transfer.lock(|tx| write_char(tx, 45 as char));
        //shared.rx_transfer.lock(|rx| {
        //    let res = read_char(rx);
        //    match res {
        //        Ok(c) => {
        //            let cchar = ((c & 0xFF) as u8) as char;
        //            if cchar as u8 == 13 {
        //                print_command::spawn().unwrap();
        //            } else {
        //                shared.tx_transfer.lock(|tx| write_char(tx, cchar));
        //                shared.command.lock(|comm| {
        //                    let s_res = comm.push(cchar);
        //                    match s_res {
        //                        Ok(_) => (),
        //                        Err(e) => hprintln!("String Push Error {:?}", e),
        //                    }
        //                });
        //            }
        //        }
        //        Err(_) => (),
        //    };
        //});
    }

    #[task(shared = [command, tx_transfer])]
    fn print_command(cx: print_command::Context) {
        let print_command::Context { mut shared } = cx;
        shared.command.lock(|comm| {
            shared.tx_transfer.lock(|tx| {
                write_char(tx, 10 as char);
                write_char(tx, 13 as char)
            });
            for c in comm.chars() {
                shared.tx_transfer.lock(|tx| write_char(tx, c));
            }
            shared.tx_transfer.lock(|tx| {
                write_char(tx, 10 as char);
                write_char(tx, 13 as char)
            });
            //hprintln!("Received Command: {}", s.as_str());
            comm.clear();
        });
    }

    #[task(binds=DMA1_STREAM1, shared = [command, rx_transfer, led], local=[rx_buffer])]
    fn rx_dma(cx: rx_dma::Context) {
        let rx_dma::Context { mut shared, local } = cx;
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
            match filled_buffer[0] {
                1 => shared.led.lock(|led| led.set_low()),
                _ => shared.led.lock(|led| led.set_high()),
            }
            //let mut s = String::<32>::from("");
            //for c in filled_buffer.into_iter() {
            //    let sres = s.push(*c as char);
            //    match sres {
            //        Ok(_) => (),
            //        Err(e) => hprint!("String Error {:?}", e),
            //    }
            //}
            *local.rx_buffer = Some(filled_buffer);
            //hprintln!("dma: {}", s.as_str());
        }
    }
}
