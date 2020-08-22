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
//!    let dt = 1.; // Time between measurements, in seconds
//!    let mut ph_sensor = PhSensor::new(i2c, dt);
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
//!        delay.delay_ms(dt as u16 * 1000);
//!    }
//!}
//! ```

#![no_std]
#![allow(non_snake_case)]
#![feature(unsize)] // Used by the `max31865` module.

#[macro_use(block)]
extern crate nb;

use ads1x1x::{
    self,
    channel::{DifferentialA0A1, SingleA2},
    ic::{Ads1115, Resolution16Bit},
    interface::I2cInterface,
    Ads1x1x, SlaveAddr,
};
use embedded_hal::{
    adc::OneShot,
    blocking::i2c::{Read, Write, WriteRead},
    blocking::spi,
    blocking::delay::{DelayMs, DelayUs},
    digital::v2::OutputPin,
    PwmPin,
};
use filter::kalman::kalman_filter::KalmanFilter;

use nalgebra::{
    dimension::{U1, U2},
    Vector1,
};

use num_traits::float::FloatCore; // Required to take absolute value in `no_std`.

mod ec;
mod filter_;
mod max31865;
mod mcp4921;
mod storage;

use ec::EcSensor;
use max31865::{FilterMode, Max31865};

// Compensate for temperature diff between readings and calibration.
const PH_TEMP_C: f32 = -0.05694; // pH/(V*T). V is in volts, and T is in °C
const DISCRETE_PH_JUMP_THRESH: f32 = 0.2;
const DISCRETE_ORP_JUMP_THRESH: f32 = 30.;
const PH_STD: f32 = 0.1;
const ORP_STD: f32 = 10.;

pub type Adc<I> = Ads1x1x<
    I2cInterface<I>,
    Ads1115,
    Resolution16Bit,
    ads1x1x::mode::OneShot,
    // todo: How do we do type for continuous variant?
>;

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

// #[derive(Debug, Clone, Copy)]
// /// Data for a single RTD (or thermister) calibration point.
// pub struct CalPtRtd {
//     pub R: f32, // Resistance, ohms
//     pub T: f32, // in Celsius
// }
//
// impl CalPtRtd {
//     pub fn new(R: f32, T: f32) -> Self {
//         Self { R, T }
//     }
// }

#[derive(Debug, Clone, Copy)]
/// Data for a single ORP (or other ion measurement) calibration point.
pub struct CalPtOrp {
    pub V: f32,   // voltage, in Volts
    pub ORP: f32, // in mV
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

impl CalPtOrp {
    pub fn new(V: f32, ORP: f32) -> Self {
        Self { V, ORP }
    }
}

pub struct PhSensor<I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>, E> {
    adc: Option<
        Ads1x1x<
            ads1x1x::interface::I2cInterface<I2C>,
            Ads1115,
            Resolution16Bit,
            ads1x1x::mode::OneShot,
        >, // todo continuous support
    >,
    // We store address, since `unfree` needs it to recreate the ADC.
    addr: SlaveAddr,
    pub filter: KalmanFilter<f32, U2, U1, U1>,
    dt: f32, // used for manually resetting the filter (`filterpy` has a reset method, `filter-rs` doesn't).
    last_meas: f32, // to let discrete jumps bypass the filter.
    cal_1: CalPt,
    cal_2: CalPt,
    cal_3: Option<CalPt>,
}

impl<I2C: WriteRead<Error = E> + Write<Error = E> + Read<Error = E>, E> PhSensor<I2C, E> {
    pub fn new(i2c: I2C, dt: f32) -> Self {
        // `dt` is in seconds.
        let adc = Ads1x1x::new_ads1115(i2c, SlaveAddr::default());
        // Leave the default range of 2.048V; this is overkill (and reduces precision of)
        // the pH sensor, but is needed for the temp sensor.

        // adc.set_full_scale_range(ads1x1x::FullScaleRange::Within6_144V);

        Self {
            adc: Some(adc),
            addr: SlaveAddr::default(),
            filter: filter_::create(dt, PH_STD),
            dt,
            last_meas: 7.,
            cal_1: CalPt::new(0., 7., 23.),
            cal_2: CalPt::new(0.17, 4., 23.),
            cal_3: None,
        }
    }

    pub fn new_alt_addr(i2c: I2C, dt: f32) -> Self {
        // `dt` is in seconds.
        let adc = Ads1x1x::new_ads1115(i2c, SlaveAddr::new_vdd());
        // Leave the default range of 2.048V; this is overkill (and reduces precision of)
        // the pH sensor, but is needed for the temp sensor.

        Self {
            adc: Some(adc),
            addr: SlaveAddr::new_vdd(),
            filter: filter_::create(dt, PH_STD),
            dt,
            last_meas: 7.,
            cal_1: CalPt::new(0., 7., 23.),
            cal_2: CalPt::new(0.17, 4., 23.),
            cal_3: None,
        }
    }

    /// Free the I2C device
    pub fn free(&mut self) -> I2C {
        self.adc
            .take()
            .expect("I2C already freed")
            .destroy_ads1115()
    }

    /// Re-assign the I2C device (eg after free)
    pub fn unfree(&mut self, i2c: I2C) {
        self.adc = Some(Ads1x1x::new_ads1115(i2c, self.addr));
    }

    /// Make a prediction using the Kalman filter. Not generally used directly.
    pub fn predict(&mut self) {
        self.filter.predict(None, None, None, None)
    }

    /// Update the Kalman filter with a pH reading. Not generally used directly.
    pub fn update(&mut self, t: TempSource) -> Result<(), ads1x1x::Error<E>> {
        let pH = self.read_raw(t)?;
        let z = Vector1::new(pH);

        if (pH - self.last_meas).abs() > DISCRETE_PH_JUMP_THRESH {
            self.filter = filter_::create(self.dt, PH_STD) // reset the filter.
        }

        self.filter.update(&z, None, None);

        Ok(())
    }

    /// Take a pH reading, using the Kalman filter. This reduces sensor
    /// noise, and provides a more accurate reading.
    pub fn read(&mut self, t: TempSource) -> Result<f32, ads1x1x::Error<E>> {
        self.predict();
        self.update(t)?;
        // self.filter.x is mean, variance. We only care about the mean
        Ok(self.filter.x[0])
    }

    /// Take a pH reading, without using the Kalman filter
    // todo: find the right error type for nb/ads111x
    // todo: Error type: is this right? Read:Error instead?
    pub fn read_raw(&mut self, t: TempSource) -> Result<f32, ads1x1x::Error<E>> {
        let T = match t {
            TempSource::OnBoard => temp_from_voltage(voltage_from_adc(block!(self
                .adc
                .as_mut()
                .expect("Measurement after I2C freed")
                .read(&mut SingleA2))?)),
            TempSource::OffBoard(t_) => t_,
        };

        // We don't re-use `self.read_voltage` here due to i16/f32 type issues.
        let pH = ph_from_voltage(
            voltage_from_adc(block!(self
                .adc
                .as_mut()
                .expect("Measurement after I2C freed")
                .read(&mut DifferentialA0A1))?),
            T,
            &self.cal_1,
            &self.cal_2,
            &self.cal_3,
        );

        self.last_meas = pH;
        Ok(pH)
    }

    /// Useful for getting calibration data
    pub fn read_voltage(&mut self) -> Result<f32, ads1x1x::Error<E>> {
        Ok(voltage_from_adc(block!(self
            .adc
            .as_mut()
            .expect("Measurement after I2C freed")
            .read(&mut DifferentialA0A1))?))
    }

    /// Useful for getting calibration data
    pub fn read_temp(&mut self) -> Result<f32, ads1x1x::Error<E>> {
        Ok(temp_from_voltage(voltage_from_adc(block!(self
            .adc
            .as_mut()
            .expect("Measurement after I2C freed")
            .read(&mut SingleA2))?)))
    }

    /// Calibrate by measuring voltage and temp at a given pH. Set the
    /// calibration, and return (Voltage, Temp).
    pub fn calibrate(
        &mut self,
        slot: CalSlot,
        pH: f32,
        t: TempSource,
    ) -> Result<(f32, f32), ads1x1x::Error<E>> {
        let T = match t {
            TempSource::OnBoard => temp_from_voltage(voltage_from_adc(block!(self
                .adc
                .as_mut()
                .expect("Measurement after I2C freed")
                .read(&mut SingleA2))?)),
            TempSource::OffBoard(t_) => t_,
        };
        let V = voltage_from_adc(block!(self
            .adc
            .as_mut()
            .expect("Measurement after I2C freed")
            .read(&mut DifferentialA0A1))?);
        let pt = CalPt::new(V, pH, T);

        match slot {
            CalSlot::One => self.cal_1 = pt,
            CalSlot::Two => self.cal_2 = pt,
            CalSlot::Three => self.cal_3 = Some(pt),
        }
        Ok((V, T))
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

pub struct OrpSensor<I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>, E> {
    // These sensors operate in a similar, minus the conversion from
    // voltage to measurement, not compensating for temp, and using only 1 cal pt.
    // The adc will be empty if I2C has been freed.
    pub(crate) adc: Option<
        Ads1x1x<
            ads1x1x::interface::I2cInterface<I2C>,
            Ads1115,
            Resolution16Bit,
            ads1x1x::mode::OneShot,
        >, // todo continuous support
    >,
    addr: SlaveAddr,
    pub filter: KalmanFilter<f32, U2, U1, U1>,
    dt: f32, // used for manually resetting the filter (`filterpy` has a reset method, `filter-rs` doesn't).
    last_meas: f32, // to let discrete jumps bypass the filter.
    cal: CalPtOrp,
}

impl<I2C: WriteRead<Error = E> + Write<Error = E> + Read<Error = E>, E> OrpSensor<I2C, E> {
    pub fn new(i2c: I2C, dt: f32) -> Self {
        let adc = Ads1x1x::new_ads1115(i2c, SlaveAddr::default());

        Self {
            adc: Some(adc),
            addr: SlaveAddr::default(),
            filter: filter_::create(dt, ORP_STD),
            dt,
            last_meas: 0.,
            cal: CalPtOrp::new(0.4, 400.),
        }
    }

    /// This isn't intended to be used by the standalone module, but by the water monitor,
    /// which connects the ORP sensor to the second ADC, although it still uses differential
    /// input A0, A1.
    pub fn new_alt_addr(i2c: I2C, dt: f32) -> Self {
        let adc = Ads1x1x::new_ads1115(i2c, SlaveAddr::new_vdd());

        Self {
            adc: Some(adc),
            addr: SlaveAddr::new_vdd(),
            filter: filter_::create(dt, ORP_STD),
            dt,
            last_meas: 0.,
            cal: CalPtOrp::new(0.4, 400.),
        }
    }

    /// Free the I2C device
    pub fn free(&mut self) -> I2C {
        self.adc
            .take()
            .expect("I2C already freed")
            .destroy_ads1115()
    }

    /// Re-assign the I2C device (eg after free)
    pub fn unfree(&mut self, i2c: I2C) {
        self.adc = Some(Ads1x1x::new_ads1115(i2c, self.addr));
    }

    /// Re-assign the I2C device (eg after free)
    pub fn unfree_alt_addr(&mut self, i2c: I2C) {
        self.adc = Some(Ads1x1x::new_ads1115(i2c, SlaveAddr::new_vdd()));
    }

    /// Make a prediction using the Kalman filter. Not generally used directly.
    pub fn predict(&mut self) {
        self.filter.predict(None, None, None, None)
    }

    /// Update the Kalman filter with an ORP reading. Not generally used directly.
    pub fn update(&mut self) -> Result<(), ads1x1x::Error<E>> {
        let ORP = self.read_raw()?;
        let z = Vector1::new(ORP);

        if (ORP - self.last_meas).abs() > DISCRETE_ORP_JUMP_THRESH {
            self.filter = filter_::create(self.dt, ORP_STD) // reset the filter.
        }

        self.filter.update(&z, None, None);

        Ok(())
    }

    /// Take an ORP reading, using the Kalman filter. This reduces sensor
    /// noise, and provides a more accurate reading.
    pub fn read(&mut self) -> Result<f32, ads1x1x::Error<E>> {
        self.predict();
        self.update()?;
        // self.filter.x is mean, variance. We only care about the mean
        Ok(self.filter.x[0])
    }

    /// Take an ORP reading, without using the Kalman filter
    pub fn read_raw(&mut self) -> Result<f32, ads1x1x::Error<E>> {
        // We don't re-use `self.read_voltage`, due to i16 vs f32 typing issues.
        let orp = orp_from_voltage(
            voltage_from_adc(block!(self
                .adc
                .as_mut()
                .expect("Measurement after I2C freed")
                .read(&mut DifferentialA0A1))?),
            &self.cal,
        );

        self.last_meas = orp;
        Ok(orp)
    }

    /// Useful for getting calibration data
    pub fn read_voltage(&mut self) -> Result<f32, ads1x1x::Error<E>> {
        Ok(voltage_from_adc(block!(self
            .adc
            .as_mut()
            .expect("Measurement after I2C freed")
            .read(&mut DifferentialA0A1))?))
    }

    /// Useful for getting calibration data
    pub fn read_temp(&mut self) -> Result<f32, ads1x1x::Error<E>> {
        Ok(temp_from_voltage(voltage_from_adc(block!(self
            .adc
            .as_mut()
            .expect("Measurement after I2C freed")
            .read(&mut SingleA2))?)))
    }

    //    /// Not support by the pH module. Used for reading temperature when an ec
    //    /// circuit is wired to A3.
    //    #[doc(hidden)]
    //    pub fn read_ec(&mut self, t: TempSource) -> Result<f32, ads1x1x::Error<E>> {
    //        let T = 0.; // todo
    //
    //        Ok(ec_from_voltage(
    //            voltage_from_adc(block!(self
    //                .adc
    //                .as_mut()
    //                .expect("Measurement after I2C freed")
    //                .read(&mut SingleA3))?),
    //            T,
    //        ))
    //    }

    /// Calibrate by measuring voltage at a given ORP. Set the
    /// calibration, and return Voltage.
    pub fn calibrate(&mut self, ORP: f32) -> Result<f32, ads1x1x::Error<E>> {
        let V = voltage_from_adc(block!(self
            .adc
            .as_mut()
            .expect("Measurement after I2C freed")
            .read(&mut DifferentialA0A1))?);
        self.cal = CalPtOrp::new(V, ORP);
        Ok(V)
    }

    pub fn calibrate_all(&mut self, pt: CalPtOrp) {
        self.cal = pt;
    }

    pub fn reset_calibration(&mut self) {
        self.cal = CalPtOrp::new(0.4, 400.);
    }
}

#[derive(Clone, Copy, Debug)]
pub enum RtdType {
    // todo: The lib you're using only supports PT100s. Will have to add PT1000
    // todo support in your fork.
    Pt100,
    Pt1000,
}

#[derive(Clone, Copy, Debug)]
/// Thinly wraps `max31865::SensorType`.
pub enum RtdWires {
    Two,
    Three,
    Four,
}

/// This provides a higher level interface than in the `max31865` module.
pub struct Rtd<CS: OutputPin> {
    sensor: Max31865<CS>,
    type_: RtdType,
    wires: RtdWires,
    // cal: CalPtRtd,
}

impl<CS: OutputPin> Rtd<CS> {
    pub fn new<SPI, E>(spi: &mut SPI, cs: CS, type_: RtdType, wires: RtdWires) -> Self
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        let mut sensor = Max31865::new(cs, type_, wires);

        let ref_R = match type_ {
            RtdType::Pt100 => 300,
            RtdType::Pt1000 => 3_000,
        };
        // Set cal to the circuit's reference resistance * 100.
        // sensor.set_calibration::<E>(ref_R * 100)?;

        // Note: We don't return a Result for this function to keep upstream error
        // handling cleaner; eg sensor struct creation doesn't fail, but readings do.
        // Calibration and config errors here fail silently.
        sensor.set_calibration::<E>(ref_R * 100).ok();

        sensor.configure(
            spi,
            true, // vbias voltage; must be true to perform conversion.
            true, // automatically perform conversion
            true, // One-shot mode
            // todo: Make this configurable once you add non-US markets.
            FilterMode::Filter60Hz, // mains freq, eg 50Hz in Europe, 50Hz in US.
        ).ok();

        Self {
            sensor,
            type_,
            wires,
            // cal: CalPtRtd::new(0., 100.),
        }
    }

    /// Set filter mode to 50Hz AC noise, eg in Europe. Defaults to 60Hz for US.
    pub fn set_50hz<SPI, E>(&mut self, spi: &mut SPI) -> Result<(), E>
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        self.sensor.configure(
            spi,
            true,                   // vbias voltage; must be true to perform conversion.
            true,                   // automatically perform conversion
            true,                   // One-shot mode
            FilterMode::Filter50Hz, // mains freq, eg 50Hz in Europe, 50Hz in US.
        )?;

        Ok(())
    }

    /// Measure temperature, in Celsius
    pub fn read<SPI, E>(&mut self, spi: &mut SPI) -> Result<f32, E>
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        match self.sensor.read_default_conversion(spi) {
            Ok(val) => Ok(val as f32 / 100.),
            Err(e) => Err(e),
        }
    }

    /// (From driver notes:   You can perform calibration by putting the sensor in boiling (100 degrees
    /// Celcius) water and then measuring the raw value using `read_raw`. Calculate
    /// `calib` as `(13851 << 15) / raw >> 1`.
    /// todo: Sort this out.
    pub fn calibrate<SPI, E>(&mut self, spi: &mut SPI) -> Result<(), E>
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        let raw = self.sensor.read_raw(spi)?;
        //        self.sensor.set_calibration(  // todo: Fix
        //            ((13851 << 15) / (raw >> 1)) as u32
        //        );

        Ok(())
    }

    // todo: Way to pre-set calibration value?
}

/// We use SensorError on results from the `WaterMonitor` struct.
#[derive(Clone, Debug)]
pub enum SensorError {
    Bus,  // eg an I2C or SPI error
    NotConnected, // todo
    BadMeasurement,
}

#[derive(Debug)]
pub struct Readings {
    pub pH: Result<f32, SensorError>,
    pub T: Result<f32, SensorError>,
    pub ec: Result<f32, SensorError>,
    pub ORP: Result<f32, SensorError>,
}

/// We use this to pull data from the Water Monitor to an external program over I2C.
/// It interacts directly with the ADCs, and has no interaction to the Water Monitor's MCU.
/// We own the I2C bus, and borrow SPI, currently.
/// `EI` is for I2C errors. `ES` is for SPI errors.
/// todo: For now, this is just for external connections to the Water Monitor: We don't
/// todo use it in its project code, although we could change that.
// todo: Be able to properly fail for both SPI and I2C errors. We currently
// todo: only properly handle I2C ones.
pub struct WaterMonitor<I2C, CsRtd, CsEc, P0, P1, P2, PWM0, PWM1, PWM2, EI>
where
    I2C: WriteRead<Error = EI> + Write<Error = EI> + Read<Error = EI>,
    CsRtd: OutputPin,
    CsEc: OutputPin,
    P0: OutputPin,
    P1: OutputPin,
    P2: OutputPin,
    PWM0: PwmPin,
    PWM1: PwmPin,
    PWM2: PwmPin,
{
    rtd: Rtd<CsRtd>,                    // Max31865 RTD chip.
    ph: PhSensor<I2C, EI>,              // at 0x48. Inludes the temp sensor at input A3.
    pub(crate) orp: OrpSensor<I2C, EI>, // at 0x49. Inlucdes the ec sensor at input A3.
    ec: EcSensor<CsEc, P0, P1, P2, PWM0, PWM1, PWM2>,
}

impl<I2C, CsRtd, CsEc, P0, P1, P2, PWM0, PWM1, PWM2, EI>
    WaterMonitor<I2C, CsRtd, CsEc, P0, P1, P2, PWM0, PWM1, PWM2, EI>
where
    I2C: WriteRead<Error = EI> + Write<Error = EI> + Read<Error = EI>,
    CsRtd: OutputPin,
    CsEc: OutputPin,
    P0: OutputPin,
    P1: OutputPin,
    P2: OutputPin,
    PWM0: PwmPin,
    PWM1: PwmPin,
    PWM2: PwmPin,
{
    pub fn new<SPI, ES>(
        spi: &mut SPI,
        i2c: I2C,
        cs_rtd: CsRtd,
        cs_ec: CsEc,
        switch_pins: (P0, P1, P2),
        pwm: (PWM0, PWM1, PWM2),
        dt: f32,
    ) -> Self
    where
        SPI: spi::Write<u8, Error = ES> + spi::Transfer<u8, Error = ES>,
    {
        let rtd = Rtd::new(spi, cs_rtd, RtdType::Pt100, RtdWires::Three);

        let mut ph = PhSensor::new(i2c, dt);
        let i2c = ph.free();

        let mut orp = OrpSensor::new_alt_addr(i2c, dt);
        ph.unfree(orp.free());

        let ec = EcSensor::new(cs_ec, switch_pins, pwm);

        // todo: You should perhaps have these as options or results, so hardware failures like for
        // todo the RTD don't crash the program.
        Self {
            ph,
            orp,
            rtd,
            ec,
        }
    }

    // Read all sensors.
    pub fn read_all<SPI, ES>(&mut self, spi: &mut SPI) -> Readings
    where
        SPI: spi::Write<u8, Error = ES> + spi::Transfer<u8, Error = ES>,
    {
        self.ph_take();

        let T = self.read_temp(spi);
        // Don't invalidate the temperature-compensated readings just because
        // we have a problem reading temperature. The user will have to note
        // that they may have errors due to this, but still take the readings.
        let T2 = T.unwrap_or(20.);
        // todo: TEMP TS until you get max316 into WM
        // let T2 = 20.;

        let pH = match self.ph.read(TempSource::OffBoard(T2)) {
            Ok(v) => Ok(v),
            Err(_) => Err(SensorError::Bus),
        };

        self.orp.unfree(self.ph.free());

        let ORP = match self.orp.read() {
            Ok(v) => Ok(v),
            Err(_) => Err(SensorError::Bus),
        };
        //        let ec = self.orp_ec.read_ec(TempSource::OffBoard(T2));
        let ec = Ok(0.); // todo impl once you get new ec circuit working.

        // Assume the ph_temp ADC has I2C by default.
        self.ph.unfree(self.orp.free());

        // todo: temp Ok(T2)! You ran into an issue with unwrap taking ownership.
        // todo: Make it throw the error instead of your dummy value of 20. if there's an error.
        Readings {
            pH,
            T: Ok(T2),
            ORP,
            ec,
        }
    }

    /// Read temperature from the MAX31865 RTD IC.
    pub fn read_temp<SPI, ES>(&mut self, spi: &mut SPI) -> Result<f32, SensorError>
    where
        SPI: spi::Write<u8, Error = ES> + spi::Transfer<u8, Error = ES>,
    {
        match self.rtd.read(spi) {
            Ok(v) => Ok(v),
            Err(_) => Err(SensorError::Bus),
        }
    }

    /// Read pH from the `orp_ph` ADC.
    pub fn read_ph<SPI, ES>(&mut self, spi: &mut SPI) -> Result<f32, SensorError>
    where
        SPI: spi::Write<u8, Error = ES> + spi::Transfer<u8, Error = ES>,
    {
        let t = TempSource::OffBoard(self.read_temp(spi)?);

        self.ph_take();
        match self.ph.read(t) {
            Ok(v) => Ok(v),
            Err(_) => Err(SensorError::Bus),
        }
    }

    /// Read ORP from the `orp_ph` ADC.
    pub fn read_orp(&mut self) -> Result<f32, SensorError> {
        self.orp_take();
        match self.orp.read() {
            Ok(v) => Ok(v),
            Err(_) => Err(SensorError::Bus),
        }
    }

    /// Read electrical conductivity.
    pub fn read_ec<SPI, ES, D>(&mut self, spi: &mut SPI, delay: &mut D) -> Result<f32, SensorError>
    where
        SPI: spi::Write<u8, Error = ES> + spi::Transfer<u8, Error = ES>,
        D: DelayUs<u16> + DelayMs<u16>,
    {
        let T = self.read_temp(spi)?;

        self.orp_take();
        match self.ec.read(spi, &mut self.orp.adc.as_mut().unwrap(), delay, T) {
            Ok(v) => Ok(v),
            Err(_) => Err(SensorError::Bus),
        }
    }

    /// Read raw voltage from the pH probe.
    pub fn read_ph_voltage(&mut self) -> Result<f32, SensorError> {
        self.ph_take();

        match self.ph.read_voltage() {
            Ok(v) => Ok(v),
            Err(_) => Err(SensorError::Bus),
        }
    }

    /// Read raw voltage from the ORP probe.
    pub fn read_orp_voltage(&mut self) -> Result<f32, SensorError> {
        self.orp_take();

        match self.orp.read_voltage() {
            Ok(v) => Ok(v),
            Err(_) => Err(SensorError::Bus),
        }
    }

    // pub fn read_ec_voltage(&mut self) -> Result<(f32, f32), SensorError> {
    //     self.orp_take();
    //
    //     match self.ec.read_voltage(&mut self.orp.adc) {
    //         Ok(v) => Ok(v),
    //         Err(e) => Err(SensorError::Bus),
    //     }
    // }

    pub fn calibrate_all_ph(&mut self, pt0: CalPt, pt1: CalPt, pt2: Option<CalPt>) {
        self.ph.calibrate_all(pt0, pt1, pt2);
    }

    //    pub fn calibrate_all_temp(&mut self, pt0: CalPtT, pt1: CalPtT) {
    //        self.cal_temp_1 = pt0;
    //        self.cal_temp_2 = pt1;
    //    }

    pub fn calibrate_all_orp(&mut self, pt: CalPtOrp) {
        self.orp.calibrate_all(pt);
    }

    // pub fn calibrate_all_ec(&mut self, pt: CalPtOrp) {
    //     self.orp_ec.calibrate_all(pt);
    // }

    /// Helper function to take the I2C peripheral.
    pub fn ph_take(&mut self) {
        if self.ph.adc.is_none() {
            self.ph.unfree(self.orp.free());
        }
    }

    /// Helper function to take the I2C peripheral.
    pub fn orp_take(&mut self) {
        if self.orp.adc.is_none() {
            self.orp.unfree(self.ph.free());
        }
    }
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
fn ph_from_voltage(V: f32, temp: f32, cal_0: &CalPt, cal_1: &CalPt, cal_2: &Option<CalPt>) -> f32 {
    // We infer a -.05694 pH/(V*T) sensitivity linear relationship
    // (higher temp means higher pH/V ratio)
    let T_diff = temp - cal_0.T;
    let T_comp = PH_TEMP_C * T_diff; // pH / V

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

///// Convert PT100-circuit voltage to Temperature, in °C
///// We model the relationship between sensor voltage and temperature linearly
///// between 2 calibration points.
//fn temp_pt100_from_voltage(V: f32, cal_0: CalPtT, cal_1: CalPtT) -> f32 {
//    // a is the slope, T / V.
//    let a = (cal_1.T - cal_0.T) / (cal_1.V - cal_0.V);
//    let b = cal_1.T - a * cal_1.V;
//    a * V + b
//
//    // todo: Evaluate if a LUT is what you want. Quadratic polynomial, 3-deg poly?
//    // todo: Lagrange poly?
//
//    // https://www.digikey.com/en/articles/rtds-ptcs-and-ntcs-how-to
//    // -effectively-decipher-this-alphabet-soup-of-temperature-sensors
//
//}
