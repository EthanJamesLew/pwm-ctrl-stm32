#![no_std]

/// Serial Requirements of device and host
pub mod serial_config {
    pub const BAUDRATE: u32 = 115_200;
}

/// Command model for serial C&C
pub mod command {

    /// PWM Commands are indentified by Op Code
    #[derive(Debug)]
    #[repr(u8)]
    pub enum PwmOpCode {
        Frequency = 0,
        Phase = 1,
        Wavetype = 2,
        TableSize = 3,
    }

    impl PwmOpCode {
        /// Convert u8 to OpCode if its valid
        pub fn try_from_u8(orig: u8) -> Option<Self> {
            match orig {
                0 => return Some(Self::Frequency),
                1 => return Some(Self::Phase),
                2 => return Some(Self::Wavetype),
                3 => return Some(Self::TableSize),
                _ => None,
            }
        }
    }

    /// command frame model sent over the wire
    #[derive(Debug)]
    pub struct PwmCommand {
        pub op: PwmOpCode,
        pub channel: u8,
        pub arg: u16,
    }
}
