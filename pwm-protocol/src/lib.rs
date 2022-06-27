#![no_std]

/// Serial Requirements of device and host
pub mod serial_config {
    pub const BAUDRATE: u32 = 115_200;
}

pub mod lut {
    use core::f32::consts::FRAC_PI_2;
    use heapless::Vec;
    use micromath::F32Ext;

    pub const MAX_TABLE_SIZE: usize = 512;
    pub const DEFAULT_FREQ: u32 = 15_360;

    pub struct SignalConfig {
        pub pwm_freq: u32,
        pub max_duty: u16,
        pub table_size: usize,
        pub shifts: [usize; 16],
    }

    impl SignalConfig {
        pub fn default(max_duty: u16) -> Self {
            Self {
                pwm_freq: DEFAULT_FREQ,
                max_duty: max_duty,
                table_size: 256,
                shifts: [0; 16],
            }
        }

        pub fn generate_lut(&self) -> Option<Vec<u16, MAX_TABLE_SIZE>> {
            let mut v = Vec::<u16, MAX_TABLE_SIZE>::new();
            let a = (4.0 * FRAC_PI_2) / (self.table_size as f32);
            for i in 0..self.table_size {
                let angle = a * (i as f32);
                let p_op = v.push(((angle.sin() / 2.0 + 0.5) * (self.max_duty as f32)) as u16);
                match p_op {
                    Ok(_) => (),
                    Err(_) => return None,
                }
            }
            Some(v)
        }
    }
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
