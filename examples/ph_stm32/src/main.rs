//! This program demonstrates how to use the AnyLeaf pH module
//! with an STM32F3 microcontroller. It continuously displays
//! the pH using an output handler, and demonstrates how
//! to calibrate.

//! // todo: Redo using stm32-hal.

#![no_main]
#![no_std]

use cortex_m;
use cortex_m_rt::entry;
use stm32f3xx_hal as hal;
use hal::{delay::Delay, i2c::I2c, prelude::*, pac};
use embedded_hal::blocking::delay::DelayMs;
use rtt_target::{rprintln, rtt_init_print};
use anyleaf::{PhSensor, CalPt, CalSlot, TempSource};

#[entry]
fn main() -> ! {
    rtt_init_print!();

    // Set up i2C.
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb); // PB GPIO pins
    let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 100.khz(), clocks, &mut rcc.apb1);

    let mut ph_sensor = PhSensor::new(i2c);

    ph_sensor.calibrate_all(
        CalPt::new(0., 7., 25.), CalPt::new(0.17, 4., 25.), Some(CalPt::new(-0.17, 10., 25.))
    );

    // See `ph_linux` example for the automatic calibration method.

    loop {
        let pH = ph_sensor.read(TempSource::OnBoard).unwrap();
        rprintln!("pH: {}", pH);

        delay.delay_ms(1000);
    }
}

#[panic_handler]
fn my_panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
