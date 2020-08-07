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
        CalPt::new(0., 7., 25.), CalPt::new(0.17, 4., 25.), Some(CalPt::new(-0.17, 10., 25.))
    );

    // See `ph_linux` example for the automatic calibration method.

    loop {
        let pH = ph_sensor.read(TempSource::OnBoard).unwrap();
        hprintln!("pH: {}", pH).unwrap();

        delay.delay_ms(dt as u16 * 1000);
    }
}