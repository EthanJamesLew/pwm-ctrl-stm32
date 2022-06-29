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

    #[derive(PartialEq, PartialOrd, Debug)]
    pub enum SignalType {
        SINUSOID,
        SQUARE,
        SAW,
        SPIKE,
    }

    impl From<u8> for SignalType {
        fn from(orig: u8) -> Self {
            match orig {
                0x0 => return SignalType::SINUSOID,
                0x1 => return SignalType::SQUARE,
                0x2 => return SignalType::SAW,
                0x3 => return SignalType::SPIKE,
                _ => return SignalType::SINUSOID,
            }
        }
    }
    
    impl SignalType {
        fn from_str(orig: &str) -> Self {
            match orig {
                "Sinusoid" => return SignalType::SINUSOID,
                "Square" => return SignalType::SQUARE,
                "Saw" => return SignalType::SAW,
                "Spike" => return SignalType::SPIKE,
                _ => return SignalType::SINUSOID,
            }
        }
    }

    pub struct SignalConfig {
        pub signal_type: SignalType,
        pub pwm_freq: u32,
        pub max_duty: u16,
        pub table_size: usize,
        pub shifts: [usize; 16],
    }
    

    impl SignalConfig {
        pub fn default(max_duty: u16) -> Self {
            Self {
                signal_type: SignalType::SINUSOID,
                pwm_freq: DEFAULT_FREQ,
                max_duty: max_duty,
                table_size: 256,
                shifts: [0; 16],
            }
        }

        pub fn generate_lut(&self) -> Option<Vec<u16, MAX_TABLE_SIZE>> {
            let mut v = Vec::<u16, MAX_TABLE_SIZE>::new();
            let a = (4.0 * FRAC_PI_2) / (self.table_size as f32);
            match self.signal_type {
                SignalType::SINUSOID => {
                        for i in 0..self.table_size {
                            let angle = a * (i as f32);
                            let p_op = v.push(((angle.sin() / 2.0 + 0.5) * (self.max_duty as f32)) as u16);
                            match p_op {
                                Ok(_) => (),
                                Err(_) => return None,
                            }
                        }
                    },
                    SignalType::SQUARE => {
                        for i in 0..self.table_size {
                            let p_op = v.push(if i < (self.table_size / 2 ) {self.max_duty} else {0} as u16);
                            match p_op {
                                Ok(_) => (),
                                Err(_) => return None,
                            }
                        }
                    },
                    SignalType::SPIKE => {
                        for i in 0..self.table_size {
                            let p_op = v.push(if i == 0 {self.max_duty} else {0} as u16);
                            match p_op {
                                Ok(_) => (),
                                Err(_) => return None,
                            }
                        }
                    },
                    SignalType::SAW => {
                        for i in 0..self.table_size {
                            let p_op = v.push(((((i as u16) * self.max_duty) as f32) / (self.table_size as f32 + 1.0)) as u16);
                            match p_op {
                                Ok(_) => (),
                                Err(_) => return None,
                            }
                        }
                    },
                    _ => {
                        for i in 0..self.table_size {
                            let p_op = v.push(0 as u16);
                            match p_op {
                                Ok(_) => (),
                                Err(_) => return None,
                            }
                        }
                    }
            };
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
