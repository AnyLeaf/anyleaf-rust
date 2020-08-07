//! This example demonstrates how to measure temperature
//! using Rust on Linux

use embedded_hal::blocking::delay::DelayMs;
use linux_embedded_hal::{Delay, I2cdev};
use anyleaf::{Rtd, CalPtT};

fn main() {
    let spi = I2cdev::new("/dev/i2c-1").unwrap();
    cs =
    rdy =

    let mut rtd = Rtd::new(&mut spi, cs, rdy);

    let mut delay = Delay {};

    loop {
        println!("Temp: {}", rtd.read().unwrap());

        delay.delay_ms(dt as u16 * 1000);
    }
}