//! This example demonstrates how to measure temperature
//! using Rust on Linux

use embedded_hal::blocking::delay::DelayMs;
use linux_embedded_hal::{Delay, Spidev};
use anyleaf::{Rtd, CalPtT};

fn main() {
    let mut spi = Spidev::new("/dev/spidev0.0").unwrap();
    let mut cs = Gpio::new().unwrap().get(5).unwrap().into_output();

    let mut rtd = Rtd::new(&mut spi, cs);

    let mut delay = Delay {};

    loop {
        println!("Temp: {}", rtd.read().unwrap());

        delay.delay_ms(dt as u16 * 1000);
    }
}