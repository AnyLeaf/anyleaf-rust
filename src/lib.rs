//! Driver for the [AnyLeaf ph module](https://anyleaf.org/ph-module).
//! Allows you to measure pH using high-level code. See the
//! [Github repository](https://github.com/anyleaf/ph-rust) for
//! complete examples on the Rasperry Pi and stm32f3.
//!
//! Example for Rasperry Pi, and other ph_linux systems:
//! Cargo.toml:
//! ```toml
//! [package]
//! name = "anyleaf_linux_example"
//! version = "0.1.0"
//! authors = ["Anyleaf <anyleaf@anyleaf.org>"]
//! edition = "2018"
//!
//! [dependencies]
//! embedded-hal = "^0.2.3"
//! ph_linux-embedded-hal = "^0.3.0"
//! anyleaf = "^0.1.6"
//! ```
//!
//! main.rs:
//! ```rust
//!use embedded_hal::blocking::delay::DelayMs;
//!use linux_embedded_hal::{Delay, I2cdev};
//!use anyleaf::{PhSensor, CalPt, CalSlot, TempSource};
//!
//!fn main() {
//!    let i2c = I2cdev::new("/dev/i2c-1").unwrap();
//!    let mut ph_sensor = PhSensor::new(i2c);
//!
//!    // 2 or 3 pt calibration both give acceptable results.
//!    // Calibrate with known values. (voltage, pH, temp in °C).
//!    // You can find voltage and temperature with `ph_sensor.read_voltage()` and
//!    // `ph_sensor.read_temp()` respectively.
//!    // For 3 pt calibration, pass a third argument to `calibrate_all`.
//!    ph_sensor.calibrate_all(
//!        CalPt::new(0., 7., 25.), CalPt::new(0.17, 4., 25.), None,
//!    );
//!
//!    // Or, call these with the sensor in the appropriate buffer solution.
//!    // This will automatically use voltage and temperature.
//!    // Voltage and Temp are returned, but calibration occurs
//!    // without using the return values.
//!    // (V, T) = ph_sensor.calibrate(CalSlot::One, 7.);
//!    // ph_sensor.calibrate(CalSlot::Two, 4.);
//!
//!    // Store the calibration parameters somewhere, so they persist
//!    // between program runs.
//!
//!    let mut delay = Delay {};
//!
//!    loop {
//!        let pH = ph_sensor.read(TempSource::OnBoard).unwrap();
//!        println!("pH: {}", pH);
//!
//!        delay.delay_ms(1000);
//!    }
//!}
//! ```

#![no_std]
#![allow(non_snake_case, clippy::needless_doctest_main)]

use embedded_hal::blocking::i2c::{Write, WriteRead};

pub mod rtd;
mod storage;

pub use rtd::{Rtd, RtdType, Wires};

// Compensate for temperature diff between readings and calibration.
const PH_TEMP_C: f32 = -0.05694; // pH/(V*T). V is in volts, and T is in °C
const DISCRETE_PH_JUMP_THRESH: f32 = 0.2;
const DISCRETE_ORP_JUMP_THRESH: f32 = 30.;
const PH_STD: f32 = 0.1;
const ORP_STD: f32 = 10.;

const ADC_ADDR_1: u8 = 0x48;
const ADC_ADDR_2: u8 = 0x49;
const CFG_REG: u8 = 0x1;
const CONV_REG: u8 = 0x0;

// See ads1115 datasheet Section 9.6.3: Config Register. Start a differential conversion on channels
// 0 and 1, With 2.048V full scale range, one-shot mode, no alert pin activity.
const PH_ORP_CMD: u16 = 0b1000_0101_1000_0000;
// Same as `PH_ORP_CM`, but with channel A2.
const T_CMD: u16 = 0b1110_0101_1000_0000;

#[derive(Debug, Clone, Copy)]
/// Keeps our calibration organized, so we track when to overwrite.
pub enum CalSlot {
    One,
    Two,
    Three,
}

#[derive(Debug, Clone, Copy)]
/// Specify onboard or offboard temperature source.
pub enum TempSource {
    OnBoard,
    OffBoard(f32),
}

#[derive(Debug, Clone, Copy)]
/// Data for a single pH (or other ion measurement) calibration point.
pub struct CalPt {
    pub V: f32, // voltage, in Volts
    pub pH: f32,
    pub T: f32, // in Celsius
}

impl CalPt {
    pub fn new(V: f32, pH: f32, T: f32) -> Self {
        Self { V, pH, T }
    }
}

#[derive(Debug, Clone, Copy)]
/// Data for a single ORP (or other ion measurement) calibration point.
pub struct CalPtOrp {
    pub V: f32,   // voltage, in Volts
    pub ORP: f32, // in mV
}

impl CalPtOrp {
    pub fn new(V: f32, ORP: f32) -> Self {
        Self { V, ORP }
    }
}

#[derive(Debug, Clone, Copy)]
/// Data for a single temperature calibration point.
pub struct CalPtT {
    pub V: f32, // voltage, in Volts
    pub T: f32, // in Celsius
}

impl CalPtT {
    pub fn new(V: f32, T: f32) -> Self {
        Self { V, T }
    }
}

#[derive(Debug, Clone, Copy)]
/// Data for a single ORP (or other ion measurement) calibration point.
pub struct CalPtEc {
    // todo: this struct is DRY with ecfirmware.
    pub reading: f32, // reading
    pub ec: f32,      // in μS/cm
    pub T: f32,       // in Celsius
}

impl CalPtEc {
    pub fn new(reading: f32, ec: f32, T: f32) -> Self {
        Self { reading, ec, T }
    }
}

pub struct PhSensor {
    pub addr: u8,
    pub cal_1: CalPt,
    pub cal_2: CalPt,
    pub cal_3: Option<CalPt>,
}

impl PhSensor {
    pub fn new() -> Self {
        Self {
            addr: ADC_ADDR_1,
            cal_1: CalPt::new(0., 7., 23.),
            cal_2: CalPt::new(0.17, 4., 23.),
            cal_3: None,
        }
    }

    /// Create a new sensor with an ADC I2C address of 0x49.
    pub fn new_alt_addr() -> Self {
        Self {
            addr: ADC_ADDR_2,
            ..Self::new()
        }
    }

    /// Set calibration to a sensible default for nitrate, with unit mg/L
    pub fn cal_nitrate_default(&mut self) {
        self.cal_1 = CalPt::new(0.25, -2. * 62_000., 23.);
        self.cal_2 = CalPt::new(0.4, -5. * 62_000., 23.);
        self.cal_3 = None;
    }

    /// Set calibration to a sensible default for phosphate, with unit mg/L
    pub fn cal_phosphate_default(&mut self) {
        self.cal_1 = CalPt::new(0.25, -2. * 62_000., 23.);
        self.cal_2 = CalPt::new(0.4, -5. * 62_000., 23.);
        self.cal_3 = None;
    }

    /// Set calibration to a sensible default for potassium, with unit mg/L
    pub fn cal_potassium_default(&mut self) {
        self.cal_1 = CalPt::new(0.25, -2. * 62_000., 23.);
        self.cal_2 = CalPt::new(0.4, -5. * 62_000., 23.);
        self.cal_3 = None;
    }

    /// Take a pH reading
    pub fn read<I2C, E>(&mut self, t: TempSource, i2c: &mut I2C) -> f32
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        let T = match t {
            TempSource::OnBoard => {
                temp_from_voltage(voltage_from_adc(take_reading(self.addr, T_CMD, i2c)))
            }
            TempSource::OffBoard(t_) => t_,
        };

        // We don't re-use `self.read_voltage` here due to i16/f32 type issues.
        let pH = ph_from_voltage(
            // todo: This is hard crashing WM if adc is missing.
            voltage_from_adc(take_reading(self.addr, PH_ORP_CMD, i2c)),
            // 2.,
            T,
            &self.cal_1,
            &self.cal_2,
            &self.cal_3,
        );

        pH
    }

    /// Useful for getting calibration data
    pub fn read_voltage<I2C, E>(&mut self, i2c: &mut I2C) -> f32
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        voltage_from_adc(take_reading(self.addr, PH_ORP_CMD, i2c))
    }

    /// Useful for getting calibration data
    pub fn read_temp<I2C, E>(&mut self, i2c: &mut I2C) -> f32
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        temp_from_voltage(voltage_from_adc(take_reading(self.addr, T_CMD, i2c)))
    }

    /// Calibrate by measuring voltage and temp at a given pH. Set the
    /// calibration, and return (Voltage, Temp).
    pub fn calibrate<I2C, E>(
        &mut self,
        slot: CalSlot,
        pH: f32,
        t: TempSource,
        i2c: &mut I2C,
    ) -> (f32, f32)
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        let T = match t {
            TempSource::OnBoard => {
                temp_from_voltage(voltage_from_adc(take_reading(self.addr, T_CMD, i2c)))
            }
            TempSource::OffBoard(t_) => t_,
        };
        let V = voltage_from_adc(take_reading(self.addr, PH_ORP_CMD, i2c));
        let pt = CalPt::new(V, pH, T);

        match slot {
            CalSlot::One => self.cal_1 = pt,
            CalSlot::Two => self.cal_2 = pt,
            CalSlot::Three => self.cal_3 = Some(pt),
        }
        (V, T)
    }

    pub fn calibrate_all(&mut self, pt0: CalPt, pt1: CalPt, pt2: Option<CalPt>) {
        self.cal_1 = pt0;
        self.cal_2 = pt1;
        self.cal_3 = pt2;
    }

    pub fn reset_calibration(&mut self) {
        self.cal_1 = CalPt::new(0., 7., 25.);
        self.cal_2 = CalPt::new(0.17, 4., 25.);
        self.cal_3 = None;
    }
}

pub struct OrpSensor {
    // These sensors operate in a similar, minus the conversion from
    // voltage to measurement, not compensating for temp, and using only 1 cal pt.
    pub addr: u8,
    pub cal: CalPtOrp,
}

impl OrpSensor {
    pub fn new() -> Self {
        Self {
            addr: ADC_ADDR_1,
            cal: CalPtOrp::new(0.4, 400.),
        }
    }

    /// This isn't intended to be used by the standalone module, but by the water monitor,
    /// which connects the ORP sensor to the second ADC, although it still uses differential
    /// input A0, A1.
    pub fn new_alt_addr() -> Self {
        Self {
            addr: ADC_ADDR_2,
            ..Self::new()
        }
    }

    /// Take an ORP reading.
    pub fn read<I2C, E>(&mut self, i2c: &mut I2C) -> f32
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        // We don't re-use `self.read_voltage`, due to i16 vs f32 typing issues.

        let orp = orp_from_voltage(
            voltage_from_adc(take_reading(self.addr, PH_ORP_CMD, i2c)),
            &self.cal,
        );

        orp
    }

    /// Useful for getting calibration data
    pub fn read_voltage<I2C, E>(&mut self, i2c: &mut I2C) -> f32
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        voltage_from_adc(take_reading(self.addr, PH_ORP_CMD, i2c))
    }

    /// Useful for getting calibration data
    pub fn read_temp<I2C, E>(&mut self, i2c: &mut I2C) -> f32
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        temp_from_voltage(voltage_from_adc(take_reading(self.addr, T_CMD, i2c)))
    }

    /// Calibrate by measuring voltage at a given ORP. Set the
    /// calibration, and return Voltage.
    pub fn calibrate<I2C, E>(&mut self, ORP: f32, i2c: &mut I2C) -> f32
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        let V = voltage_from_adc(take_reading(self.addr, PH_ORP_CMD, i2c));
        self.cal = CalPtOrp::new(V, ORP);
        V
    }

    pub fn calibrate_all(&mut self, pt: CalPtOrp) {
        self.cal = pt;
    }

    pub fn reset_calibration(&mut self) {
        self.cal = CalPtOrp::new(0.4, 400.);
    }
}

/// We use SensorError on results from the `WaterMonitor` struct.
#[derive(Copy, Clone, Debug)]
pub enum SensorError {
    Bus,
    // eg an I2C or SPI error
    NotConnected,
    // todo
    BadMeasurement,
}

#[derive(Debug, Clone)]
pub struct Readings {
    pub T: Result<f32, SensorError>,
    pub pH: Result<f32, SensorError>,
    pub ORP: Result<f32, SensorError>,
    pub ec: Result<f32, SensorError>,
}

/// Convert a 16-bit digital value to voltage.
/// Input ranges from +- 2.048V; this is configurable.
/// Output ranges from -32_768 to +32_767.
pub fn voltage_from_adc(digi: i16) -> f32 {
    let vref = 2.048;
    (digi as f32 / 32_768.) * vref
}

/// Compute the result of a Lagrange polynomial of order 3.
/// Algorithm created from the `P(x)` eq
/// [here](https://mathworld.wolfram.com/LagrangeInterpolatingPolynomial.html).
/// todo: Figure out how to just calculate the coefficients for a more
/// todo flexible approach. More eloquent, but tough to find info on compared
/// todo to this approach.
fn lg(pt0: (f32, f32), pt1: (f32, f32), pt2: (f32, f32), X: f32) -> f32 {
    let mut result = 0.;

    let x = [pt0.0, pt1.0, pt2.0];
    let y = [pt0.1, pt1.1, pt2.1];

    for j in 0..3 {
        let mut c = 1.;
        for i in 0..3 {
            if j == i {
                continue;
            }
            c *= (X - x[i]) / (x[j] - x[i]);
        }
        result += y[j] * c;
    }

    result
}

/// Convert voltage to pH
/// We model the relationship between sensor voltage and pH linearly
/// using 2-pt calibration, or quadratically using 3-pt. Temperature
/// compensated. Input `T` is in Celsius.
fn ph_from_voltage(V: f32, T: f32, cal_0: &CalPt, cal_1: &CalPt, cal_2: &Option<CalPt>) -> f32 {
    // We infer a -.05694 pH/(V*T) sensitivity linear relationship
    // (higher temp means higher pH/V ratio)
    let T_diff = T - cal_0.T;
    let T_comp = PH_TEMP_C * T_diff; // pH / V

    // todo: Why does T_comp affect things differnetly??
    match cal_2 {
        // Model as a quadratic Lagrangian polynomial, to compensate for slight nonlinearity.
        Some(c2) => {
            let result = lg((cal_0.V, cal_0.pH), (cal_1.V, cal_1.pH), (c2.V, c2.pH), V);
            result + T_comp * V
        }
        // Model as a line
        None => {
            // a is the slope, pH / v.
            let a = (cal_1.pH - cal_0.pH) / (cal_1.V - cal_0.V);
            let b = cal_1.pH - a * cal_1.V;
            (a + T_comp) * V + b
        }
    }
}

/// Convert sensor voltage to ORP voltage
/// We model the relationship between sensor voltage and ORP linearly
/// between the calibration point, and (0., 0.). Output is in mV.
fn orp_from_voltage(V: f32, cal: &CalPtOrp) -> f32 {
    // a is the slope, ORP / V.
    let a = cal.ORP / cal.V;
    let b = cal.ORP - a * cal.V;
    a * V + b
}

/// Map voltage to temperature for the TI LM61, in °C
/// Datasheet: https://datasheet.lcsc.com/szlcsc/Texas-
/// Instruments-TI-LM61BIM3-NOPB_C132073.pdf
pub fn temp_from_voltage(V: f32) -> f32 {
    100. * V - 60.
}

/// Take a measurement from an external ADS1115 ADC, using the I2C peripheral.
fn take_reading<I2C, E>(addr: u8, cmd: u16, i2c: &mut I2C) -> i16
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    let mut result_buf: [u8; 2] = [0, 0];

    // Set up the cfg, and command a one-shot reading. Note that we
    // pass the 16-bit i2c command as 2 bytes.
    i2c.write(addr, &[CFG_REG, (cmd >> 8) as u8, cmd as u8])
        .ok();

    // Wait until the conversion is complete.
    let mut converting = true;
    let mut buf = [0, 0];
    while converting {
        i2c.write_read(addr, &[CFG_REG], &mut buf).ok();
        // First of 16 cfg reg bits is 0 while converting, 1 when ready. (when reading)
        converting = buf[0] >> 7 == 0;
    }

    // Read the result from the conversion register.
    i2c.write_read(addr, &[CONV_REG], &mut result_buf).ok();

    // Convert the 2 8-bit integers from the result into a single 16-bit signed integer.
    i16::from_be_bytes([result_buf[0], result_buf[1]])
}
