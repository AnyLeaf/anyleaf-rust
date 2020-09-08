//! Supports the Max31865. Based on [rudihorn's max31865 lib](https://github.com/rudihorn/max31865),
//! with modifications like support for Pt1000, and borrowing SPI instead of owning the bus.

use core::marker::Unsize;
use core::mem;

use embedded_hal::{
    blocking::spi,
    digital::v2::{InputPin, OutputPin},
};

#[allow(non_camel_case_types)]
#[allow(dead_code)]
#[derive(Clone, Copy)]
enum Register {
    // Register addresses and POR state. From Datasheet, Table 1.
    CONFIG = 0x00,
    CONFIG_W = 0x80,
    RTD_MSB = 0x01,
    RTD_LSB = 0x02,
    HIGH_FAULT_THRESHOLD_MSB = 0x03,
    HIGH_FAULT_THRESHOLD_LSB = 0x04,
    LOW_FAULT_THRESHOLD_MSB = 0x05,
    LOW_FAULT_THRESHOLD_LSB = 0x06,
    HIGH_FAULT_THRESHOLD_MSB_W = 0x83,
    HIGH_FAULT_THRESHOLD_LSB_W = 0x84,
    LOW_FAULT_THRESHOLD_MSB_W = 0x85,
    LOW_FAULT_THRESHOLD_LSB_W = 0x86,
    FAULT_STATUS = 0x07,
}

// Configuration register definition. From Datasheet, Table 2.
const _VBIAS_OFF: u8 = 0x00;
const _VBIAS_ON: u8 = 0x01;
const _CONVERSION_MODE_OFF: u8 = 0x00;
const _CONVERSION_MODE_AUTO: u8 = 0x01;
const _ONE_SHOT: u8 = 0x01; // auto-clear
const TWO_OR_FOUR_WIRE: u8 = 0x00;
const THREE_WIRE: u8 = 0x01;
const _FAULT_STATUS_CLEAR: u8 = 0x01; // auto-clear
const FILTER_60HZ: u8 = 0x00;
const FILTER_50_HZ: u8 = 0x01;

// Fault detection cycle control. From Datasheet, Table 3.
// Naming convention: FAULT_D3_D2.
// todo: Fix these, and impl fault detection.
// const FAULT_ZERO_ZERO = XXXX00XXb
// const FAULT_ZERO_ONE = 100X010Xb
// const FAULT_ONE_ZERO = 100X100Xb
// const FAULT_ONE_ONE = 100X110Xb

const R: u8 = 0 << 7;
const W: u8 = 1 << 7;

impl Register {
    fn read_address(&self) -> u8 {
        *self as u8 | R
    }

    fn write_address(&self) -> u8 {
        *self as u8 | W
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
pub enum Wires {
    Two,
    Three,
    Four,
}

#[repr(u8)]
pub enum FilterMode {
    Filter60Hz = FILTER_60HZ,
    Filter50Hz = FILTER_50_HZ,
}

/// This provides a higher level interface than in the `max31865` module.
pub struct Rtd<CS: OutputPin> {
    cs: CS,
    calibration: u32,
    type_: RtdType,
    wires: Wires,
    // cal: CalPtRtd,
}

impl<CS: OutputPin> Rtd<CS> {
    pub fn new<SPI, E>(spi: &mut SPI, mut cs: CS, type_: RtdType, wires: Wires) -> Self
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        cs.set_high().ok();

        let ref_R = match type_ {
            RtdType::Pt100 => 300,
            RtdType::Pt1000 => 3_000,
        };

        let mut result = Self {
            cs,
            // Set cal to the circuit's reference resistance * 100.
            calibration: ref_R * 100,
            type_,
            wires,
            // cal: CalPtRtd::new(0., 100.),
        };

        result
            .configure(
                spi,
                true, // vbias voltage; must be true to perform conversion.
                true, // automatically perform conversion
                true, // One-shot mode
                // todo: Make this configurable once you add non-US markets.
                FilterMode::Filter60Hz, // mains freq, eg 50Hz in Europe, 50Hz in US.
            )
            .ok();

        result
    }

    /// Updates the devices configuration.
    ///
    /// # Arguments
    /// * `vbias` - Set to `true` to enable V_BIAS voltage, which is required to correctly perform conversion.Clone
    /// * `conversion_mode` - `true` to automatically perform conversion, otherwise normally off.
    /// * `one_shot` - Only perform detection once if set to `true`, otherwise repeats conversion.
    /// * `sensor_type` - Define whether a two, three or four wire sensor is used.Clone
    /// * `filter_mode` - Specify the mains frequency that should be used to filter out noise, e.g. 50Hz in Europe.
    ///
    /// # Remarks
    ///
    /// This will update the configuration register of the MAX31865 register. If the device doesn't properly react
    /// to this, add a delay after calling `new` to increase the time that the chip select line is set high.
    ///
    /// *Note*: The correct sensor configuration also requires changes to the PCB! Make sure to read the datasheet
    /// concerning this.
    pub fn configure<SPI, E>(
        &mut self,
        spi: &mut SPI,
        vbias: bool,
        conversion_mode: bool,
        one_shot: bool,
        filter_mode: FilterMode,
    ) -> Result<(), E>
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        let wires = match self.wires {
            Wires::Two => TWO_OR_FOUR_WIRE,
            Wires::Three => THREE_WIRE,
            Wires::Four => TWO_OR_FOUR_WIRE,
        };

        let conf: u8 = ((vbias as u8) << 7)
            | ((conversion_mode as u8) << 6)
            | ((one_shot as u8) << 5)
            | (wires << 4)
            | (filter_mode as u8);

        self.write(spi, Register::CONFIG, conf)?;

        Ok(())
    }

    fn write<SPI, E>(&mut self, spi: &mut SPI, reg: Register, val: u8) -> Result<(), E>
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        self.cs.set_low().ok();
        spi.write(&[reg.write_address(), val])?;
        self.cs.set_high().ok();
        Ok(())
    }

    /// Set filter mode to 50Hz AC noise, eg in Europe. Defaults to 60Hz for US.
    pub fn set_50hz<SPI, E>(&mut self, spi: &mut SPI) -> Result<(), E>
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        self.configure(
            spi,
            true,                   // vbias voltage; must be true to perform conversion.
            true,                   // automatically perform conversion
            true,                   // One-shot mode
            FilterMode::Filter50Hz, // mains freq, eg 50Hz in Europe, 50Hz in US.
        )?;

        Ok(())
    }

    fn read_data<SPI, E>(&mut self, spi: &mut SPI, reg: Register) -> Result<u8, E>
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        let buffer: [u8; 2] = self.read_many(spi, reg)?;
        Ok(buffer[1])
    }

    fn read_many<B, SPI, E>(&mut self, spi: &mut SPI, reg: Register) -> Result<B, E>
    where
        B: Unsize<[u8]>,
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        let mut buffer: B = unsafe { mem::zeroed() };
        {
            let slice: &mut [u8] = &mut buffer;
            slice[0] = reg.read_address();
            self.cs.set_low().ok();
            spi.transfer(slice)?;
            self.cs.set_high().ok();
        }

        Ok(buffer)
    }

    /// Read the raw RTD value.
    ///
    /// # Remarks
    ///
    /// The raw value is the value of the combined MSB and LSB registers.
    /// The first 15 bits specify the ohmic value in relation to the reference
    /// resistor (i.e. 2^15 - 1 would be the exact same resistance as the reference
    /// resistor). See manual for further information.
    /// The last bit specifies if the conversion was successful.
    pub fn read_raw<SPI, E>(&mut self, spi: &mut SPI) -> Result<u16, E>
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        let msb: u16 = self.read_data(spi, Register::RTD_MSB)? as u16;
        let lsb: u16 = self.read_data(spi, Register::RTD_LSB)? as u16;

        Ok((msb << 8) | lsb)
    }

    /// Measure RTD resistance, in Ohms.
    pub fn read_resistance<SPI, E>(&mut self, spi: &mut SPI) -> Result<f32, E>
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        let raw = self.read_raw(spi)?;
        Ok((((raw >> 1) as u32 * self.calibration) >> 15) as f32)
    }

    /// Measure temperature, in Celsius
    pub fn read<SPI, E>(&mut self, spi: &mut SPI) -> Result<f32, E>
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        let resistance = self.read_resistance(spi)?;
        let temp = lookup_temperature(resistance as u16, self.type_);

        Ok(temp as f32 / 100.)
    }

    /// Find the fault status
    pub fn fault_status<SPI, E>(&mut self, spi: &mut SPI) -> Result<[bool; 6], E>
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        // todo: REturn a string description, or enum.
        let status = self.read_data(spi, Register::FAULT_STATUS)?;

        let overv = status & (1 << (3 - 1)) > 0;
        let rtdin = status & (1 << (4 - 1)) > 0;
        let refin = status & (1 << (5 - 1)) > 0;
        let refin2 = status & (1 << (6 - 1)) > 0;
        let rtd_low = status & (1 << (7 - 1)) > 0;
        let rtd_high = status & (1 << (8 - 1)) > 0;

        Ok([overv, rtdin, refin, refin2, rtd_low, rtd_high])
    }

    /// (From driver notes:   You can perform calibration by putting the sensor in boiling (100 degrees
    /// Celcius) water and then measuring the raw value using `read_raw`. Calculate
    /// `calib` as `(13851 << 15) / raw >> 1`.
    /// todo: Sort this out.
    ///
    /// Set the calibration reference resistance.
    /// This can be used to calibrate inaccuracies of both the reference resistor
    /// and the PT100 element.
    ///
    /// # Arguments
    ///
    /// * `calib` - A 32 bit integer specifying the reference resistance in ohms
    ///             multiplied by 100, e.g. `40000` for 400 Ohms
    ///
    /// # Remarks
    ///
    /// You can perform calibration by putting the sensor in boiling (100 degrees
    /// Celcius) water and then measuring the raw value using `read_raw`. Calculate
    /// `calib` as `(13851 << 15) / raw >> 1`.
    pub fn calibrate<SPI, E>(&mut self, spi: &mut SPI) -> Result<(), E>
    where
        SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    {
        // todo
        let raw = self.read_raw(spi)?;
        self.calibration = ((13851 << 15) / (raw >> 1)) as u32;

        Ok(())
    }

    /// Determine if a new conversion is available
    ///
    /// # Remarks
    ///
    /// When the module is finished converting the temperature it sets the
    /// ready pin to low. It is automatically returned to high upon reading the
    /// RTD registers.
    pub fn _is_ready<I: InputPin<Error = E>, E>(&self, rdy: I) -> Result<bool, E> {
        rdy.is_low()
    }

    // todo: Way to pre-set calibration value?
}

type TempPair = (u16, u16);

// this table contains a pair of temperatures and their
// corresponding resistance values for a PT100 thermometer
// The first entry of each pair is the temperature multiplied by 10,
// while the second entry contains the temperature at that resistance
// value multiplied by 10, i.e. at 0 deg C, the probe should have a resistance
// of 100 ohms

// todo: QC this, set it up for pt1000, and make sure it's what you want.
// todo: Try checking the DS.
static LOOKUP_TABLE: &[TempPair] = &[
    (0, 10000),
    (1000, 10390),
    (2000, 10779),
    (3000, 11167),
    (4000, 11554),
    (5000, 11940),
    (6000, 12324),
    (7000, 12708),
    (8000, 13090),
    (9000, 13471),
    (10000, 13851),
    (11000, 14229),
    (12000, 14607),
    (13000, 14983),
];

/// Convert the specified PT100 resistance value into a temperature.
///
/// # Arguments
///
/// * `val` - A 16 bit unsigned integer specifying the resistance in Ohms multiplied by 100, e.g.
///           13851 would indicate 138.51 Ohms and convert to 100 degrees Celcius.
///
/// # Remarks
///
/// The output temperature will be in degrees Celcius multiplied by 100, e.g. 10000 would signify 100.00
/// degrees Celcius.
///
/// *Note*: This won't handle edge cases very well.
fn lookup_temperature(val: u16, type_: RtdType) -> u32 {
    // todo: Take into account nominal resistance here, eg 100 or 1000ohm.

    let mut first = &(0, 10000);
    let mut second = &(1000, 10390);
    let mut iterator = LOOKUP_TABLE.iter();
    while let Some(a) = iterator.next() {
        first = second;
        second = &a;
        if a.1 > val {
            break;
        }
    }
    let second = iterator.next();

    if let Some(second) = second {
        // todo: This panics if no device is connected.
        let temp = (second.0 - first.0) as u32 * (val - first.1) as u32
            / (second.1 - first.1) as u32
            + first.0 as u32;
        temp
    } else {
        0
    }
}
