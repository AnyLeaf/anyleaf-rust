//! This example demonstrates how to measure temperature
//! using Rust on Linux

use embedded_hal::blocking::delay::DelayMs;
use linux_embedded_hal::{Delay, Pin, Spidev};
use anyleaf::{Rtd, RtdType, RtdWires};

fn main() {
    let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
    let cs = Pin::new(5);

    let mut rtd = Rtd::new(&mut spi, cs, RtdType::Pt100, RtdWires::Three);

    let mut delay = Delay {};

    loop {
        println!("Temp: {}", rtd.read(&mut spi).unwrap());

        delay.delay_ms(1_000_u16);
    }
}