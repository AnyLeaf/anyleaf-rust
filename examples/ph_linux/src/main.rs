//! This example demonstrates how to measure pH
//! using Rust on Linux

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
        CalPt::new(0., 7., 25.), CalPt::new(0.17, 4., 25.), None
    );

    // Or, call these with the sensor in the appropriate buffer solution.
    // This will automatically use voltage and temperature.
    // Voltage and Temp are returned, but calibration occurs
    // without using the return values.
    // (V, T) = ph_sensor.calibrate(CalSlot::One, 7., TempSource::OnBoard);
    // ph_sensor.calibrate(CalSlot::Two, 4., TempSource::OffBoard(20.);

    // Store the calibration parameters somewhere, so they persist
    // between program runs.

    let mut delay = Delay {};

    loop {
        let pH = ph_sensor.read(TempSource::OnBoard).unwrap();
        println!("pH: {}", pH);

        delay.delay_ms(dt as u16 * 1000);
    }
}