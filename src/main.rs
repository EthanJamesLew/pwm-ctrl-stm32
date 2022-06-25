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
            config::DmaConfig, traits::StreamISR, MemoryToPeripheral, PeripheralToMemory, Stream0,
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
        tx_transfer: Tx<USART3, u16>,
        rx_transfer: Rx<USART3, u16>,
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
                Config::default().baudrate(115_200.bps()).wordlength_9(),
                &clocks,
            )
            .unwrap()
            .with_u16_data();

        //serial.listen(Rxne);

        let (tx, mut rx) = serial.split();

        // Initialize the buffer for rx/tx
        let tx_buffer2 = cortex_m::singleton!(: [u8; ARRAY_SIZE] = [4,5,6]).unwrap();

        let led = gpiod.pd12.into_push_pull_output();

        rx.listen_idle();

        (
            Shared {
                led: led,
                led_state: false,
                tx_transfer: tx,
                rx_transfer: rx,
                command: String::<STRING_SIZE>::from(""),
            },
            Local {
                rx_buffer: Some(tx_buffer2),
            },
            init::Monotonics(mono),
        )
    }

    #[inline(always)]
    fn write_char(tx: &mut Tx<USART3, u16>, c: char) {
        let terr = block!(tx.write(c as u16));
        match terr {
            Ok(_) => (),
            Err(e) => hprintln!("UART Tx Error {:?}", e),
        }
    }

    #[inline(always)]
    fn read_char(rx: &mut Rx<USART3, u16>) -> Result<u16, serial::Error> {
        let res = block!(rx.read());
        match res {
            Ok(_) => (),
            Err(e) => hprintln!("Uart Rx Error {:?}", e),
        };
        res
    }

    #[task(binds = USART3, shared = [rx_transfer, tx_transfer, led, led_state, command], local = [rx_buffer])]
    fn on_receiving(cx: on_receiving::Context) {
        let on_receiving::Context {
            mut shared,
            local: _,
        } = cx;
        shared.led.lock(|led| led.toggle());
        shared.rx_transfer.lock(|rx| {
            let res = read_char(rx);
            match res {
                Ok(c) => {
                    let cchar = ((c & 0xFF) as u8) as char;
                    if cchar as u8 == 13 {
                        print_command::spawn().unwrap();
                    } else {
                        shared.tx_transfer.lock(|tx| write_char(tx, cchar));
                        shared.command.lock(|comm| {
                            let s_res = comm.push(cchar);
                            match s_res {
                                Ok(_) => (),
                                Err(e) => hprintln!("String Push Error {:?}", e),
                            }
                        });
                    }
                }
                Err(_) => (),
            };
        });
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
}
