//! This example demonstrates how to measure temperature
//! using Rust on Linux

use anyleaf::{Rtd, RtdType, Wires};
use embedded_hal::blocking::delay::DelayMs;
use linux_embedded_hal::{
    spidev::{SpiModeFlags, SpidevOptions},
    sysfs_gpio::Direction,
    Delay, Pin, Spidev,
};

fn main() {
    let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
    let options = SpidevOptions::new()
         .bits_per_word(8)
         .max_speed_hz(20_000)
         .mode(SpiModeFlags::SPI_MODE_3)
         .build();
    spi.configure(&options).unwrap();

    let cs = Pin::new(5);
    cs.export().unwrap();
    while !cs.is_exported() {}
    cs.set_direction(Direction::Out).unwrap();


    let mut rtd = Rtd::new(&mut spi, cs, RtdType::Pt100, Wires::Three);

    let mut delay = Delay {};

    loop {
        println!("Temp: {}°C", rtd.read(&mut spi, &mut delay).unwrap());
        println!("Resistance: {}Ω", rtd.read_resistance(&mut spi, &mut delay).unwrap());
        println!("Configruation: {:?}", rtd.read_config(&mut spi).unwrap());

        delay.delay_ms(1_000_u16);
    }
}