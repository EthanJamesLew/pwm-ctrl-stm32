use pwm_protocol::command::PwmOpCode;
use pwm_protocol::serial_config;
use serialport;
use std::error;
use std::result;
use std::time::Duration;

use clap::Parser;

/// PWM Serial CLI App
#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Name of Serial Port
    #[clap(short, long, value_parser)]
    device_name: String,

    /// PWM Command Name
    #[clap(short, long, value_parser, default_value_t=String::from("Wavetype"))]
    action: String,

    /// PWM Channel to perform action  
    #[clap(short, long, value_parser, default_value_t = 0)]
    channel: u8,

    /// Action Argument
    #[clap(short, long, value_parser, default_value_t = 0)]
    value: u16,
}

fn main() -> result::Result<(), Box<dyn error::Error>> {
    let args = Args::parse();

    // find if the device name is a valid serial device
    let dev_name = args.device_name;
    let ports = serialport::available_ports()?;
    let port_names: Vec<String> = ports
        .iter()
        .map(|s_info| s_info.port_name.clone())
        .collect();
    if !port_names.contains(&String::from(&dev_name)) {
        panic!("device name {} not found in ports", dev_name)
    }

    // build the serial port
    let mut port = serialport::new(dev_name, serial_config::BAUDRATE)
        .timeout(Duration::from_millis(10))
        .stop_bits(serialport::StopBits::One)
        .parity(serialport::Parity::None)
        .data_bits(serialport::DataBits::Eight)
        .open()?;

    // translate CLI input into a command
    let op_code: PwmOpCode = match args.action.as_str() {
        "Wavetype" => Some(PwmOpCode::Wavetype),
        "Phase" => Some(PwmOpCode::Phase),
        "Frequency" => Some(PwmOpCode::Frequency),
        "TableSize" => Some(PwmOpCode::TableSize),
        _ => None,
    }
    .expect(format!("{:?} isn't a valid action", args.action).as_str());

    // write to the serial port
    port.flush()?;
    port.write(&[
        op_code as u8,
        args.channel,
        (args.value >> 8) as u8,
        args.value as u8,
    ])?;

    Ok(())
}
