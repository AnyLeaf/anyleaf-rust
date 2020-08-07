[![crates.io version](https://meritbadge.herokuapp.com/anyleaf)](https://crates.io/crates/anyleaf)
[![docs.rs](https://docs.rs/anyleaf/badge.svg)](https://docs.rs/anyleaf)

# Anyleaf

## For use with the AnyLeaf pH sensor in Rust on embedded systems, and single-board computers.
[Homepage](https://anyleaf.org)

### Example for Raspberry Pi, and other Linux systems:

`Cargo.toml`:
```toml
[package]
name = "anyleaf_linux_example"
version = "0.1.0"
authors = ["Anyleaf <anyleaf@anyleaf.org>"]
edition = "2018"

[dependencies]
embedded-hal = "^0.2.3"
linux-embedded-hal = "^0.3.0"
anyleaf = "^0.1.6"
```

`main.rs`:
```rust
use embedded_hal::blocking::delay::DelayMs;
use linux_embedded_hal::{Delay, I2cdev};
use anyleaf::{PhSensor, CalPt, CalSlot, TempSource};

fn main() {
    let i2c = I2cdev::new("/dev/i2c-1").unwrap();
    let dt = 1.; // Time between measurements, in seconds
    let mut ph_sensor = PhSensor::new(i2c, dt);

    // 2 or 3 pt calibration both give acceptable results.
    // Calibrate with known values. (voltage, pH, temp in °C).
    // You can find voltage and temperature with `ph_sensor.read_voltage()` and 
    // `ph_sensor.read_temp()` respectively.
    // For 3 pt calibration, pass a third argument to `calibrate_all`.
    ph_sensor.calibrate_all(
        CalPt::new(0., 7., 25.), CalPt::new(0.18, 4., 25.), None,
    );

    // Or, call these with the sensor in the appropriate buffer solution.
    // This will automatically use voltage and temperature.
    // Voltage and Temp are returned, but calibration occurs
    // without using the return values.
    // (V, T) = ph_sensor.calibrate(CalSlot::One, 7.);
    // ph_sensor.calibrate(CalSlot::Two, 4.);

    // Store the calibration parameters somewhere, so they persist
    // between program runs.

    let mut delay = Delay {};

    loop {
        let pH = ph_sensor.read(TempSource::OnBoard).unwrap();
        println!("pH: {}", pH);

        delay.delay_ms(dt as u16 * 1000);
    }
}
```

## Example for Stm32F3:

`Cargo.toml`:
```toml
[package]
name = "anyleaf_stm32_example"
version = "0.1.0"
authors = ["Anyleaf <anyleaf@anyleaf.org>"]
edition = "2018"

[dependencies]
cortex-m = "^0.6.2"
cortex-m-rt = "^0.6.12"
stm32f3xx-hal = { version = "^0.4.3", features=["stm32f303xc", "rt"] }
f3 ="0.6.1"
embedded-hal = "^0.2.3"
cortex-m-semihosting = "0.3.5"
panic-semihosting = "0.5.3"

anyleaf = "^0.1.5"

[profile.release]
codegen-units = 1
debug = true
lto = true
```

`main.rs`:
```rust
//! This program demonstrates how to use the AnyLeaf pH module
//! with an STM32F3 microcontroller. It continuously displays
//! the pH using an output handler, and demonstrates how
//! to calibrate.

#![no_main]
#![no_std]

use cortex_m::{self, iprintln};
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f3xx_hal as hal;
use hal::{delay::Delay, i2c::I2c, prelude::*, stm32};
use embedded_hal::blocking::delay::DelayMs;

#[cfg(debug_assertions)]
extern crate panic_semihosting;

use anyleaf::{PhSensor, CalPt, CalSlot, TempSource};

#[entry]
fn main() -> ! {
    // Set up i2C.
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();

    let stim = &mut cp.ITM.stim[0];

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb); // PB GPIO pins
    let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 100.khz(), clocks, &mut rcc.apb1);

    let dt = 1.; // Time between measurements, in seconds
    let mut ph_sensor = PhSensor::new(i2c, dt);

    ph_sensor.calibrate_all(
        CalPt::new(0., 7., 25.), CalPt::new(0.17, 4., 25.), Some(CalPt::new(-0.18, 10., 25.))
    );

    ph_linux

    loop {
        let pH = ph_sensor.read(TempSource::OffBoard(20.)).unwrap();
        hprintln!("pH: {}", pH).unwrap();

        delay.delay_ms(dt as u16 * 1000);
    }
}
```