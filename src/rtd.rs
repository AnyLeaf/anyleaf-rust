//! Supports the Max31865. Based on [rudihorn's max31865 lib](https://github.com/rudihorn/max31865),
//! with modifications like support for Pt1000, and borrowing SPI instead of owning the bus.

use core::mem;

use embedded_hal::{
    blocking::{
        delay::DelayMs,
        spi::{Transfer, Write},
    },
    digital::v2::{InputPin, OutputPin},
};

#[allow(non_camel_case_types)]
#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy)]
enum Register {
    // Register addresses and POR state. From Datasheet, Table 1.
    Config = 0x00,
    ConfigW = 0x80,
    RtdMsb = 0x01,
    RtdLsb = 0x02,
    HighFaultThresholdMsb = 0x03,
    HighFaultThresholdLsb = 0x04,
    LowFaultThresholdMsb = 0x05,
    LowFaultThresholdLsb = 0x06,
    HighFaultThresholdMsbW = 0x83,
    HighFaultThresholdLsbW = 0x84,
    LowFaultThresholdMsbW = 0x85,
    LowFaultThresholdLsbW = 0x86,
    FaultStatus = 0x07,
}

// See Table2. Configuration Register Definition for info on these enums.

#[repr(u8)]
#[derive(Clone, Copy, Debug)]
/// vbias voltage; must be On to perform conversion.
pub enum Vbias {
    Off = 0,
    On = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, Debug)]
/// Set `Auto` to automatically perform conversion.
pub enum ConversionMode {
    NormallyOff = 0,
    Auto = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, Debug)]
/// RM: When the conversion mode is set to “Normally Off”, write 1  to  this  bit
/// to  start  a  conversion.  This  causes  a  single  resistance  conversion
/// to  take  place.  The  conversion  is  triggered  when  CS  goes  high  after
/// writing  a  1  to  this  bit.
pub enum OneShot {
    Cleared = 0,
    OneShot = 1, // auto-clear
}

#[derive(Clone, Copy, Debug)]
/// Thinly wraps `max31865::SensorType`.
pub enum Wires {
    Two,
    Three,
    Four,
}

#[repr(u8)]
/// Mains freq to filter out, eg 50Hz in Europe, 50Hz in US.
pub enum FilterMode {
    Filter60Hz = 0,
    Filter50Hz = 1,
}

#[derive(Clone, Copy, Debug)]
pub enum RtdType {
    Pt100,
    Pt1000,
}

/// A struct used to describe the RTD. Owns the cs pin.
pub struct Rtd<CS: OutputPin> {
    cs: CS,
    calibration: u32,
    type_: RtdType,
    wires: Wires,
}

impl<CS: OutputPin> Rtd<CS> {
    pub fn new<SPI, E>(spi: &mut SPI, mut cs: CS, type_: RtdType, wires: Wires) -> Self
    where
        SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    {
        cs.set_high().ok();

        // todo: This assumes specific hardware ref resistors
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
        };

        // Set up with vbias off, and in one-shot mode, to save power.
        result
            .configure(
                spi,
                Vbias::Off,
                ConversionMode::NormallyOff,
                OneShot::Cleared,
                FilterMode::Filter60Hz,
            )
            .ok();

        result
    }

    // /// Appears to be required after a power cycle, or it will read 0.
    // pub fn re_init<SPI, E>(&mut self, spi: &mut SPI)
    // where
    //     SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    // {
    //     self.configure(
    //         spi,
    //         Vbias::On,
    //         ConversionMode::NormallyOff,
    //         OneShot::Cleared,
    //         FilterMode::Filter60Hz,
    //     )
    //     .ok();
    // }

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
        vbias: Vbias,
        conversion_mode: ConversionMode,
        one_shot: OneShot,
        filter_mode: FilterMode,
    ) -> Result<(), E>
    where
        SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    {
        let wires = match self.wires {
            Wires::Two => 0,
            Wires::Three => 1,
            Wires::Four => 0,
        };

        let conf: u8 = ((vbias as u8) << 7)
            | ((conversion_mode as u8) << 6)
            | ((one_shot as u8) << 5)
            | (wires << 4)
            | (filter_mode as u8);

        self.write(spi, Register::ConfigW, conf)?;
        Ok(())
    }

    fn write<SPI, E>(&mut self, spi: &mut SPI, reg: Register, val: u8) -> Result<(), E>
    where
        SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    {
        self.cs.set_low().ok();
        spi.write(&[reg as u8, val])?;
        self.cs.set_high().ok();
        Ok(())
    }

    fn read_data<SPI, E>(&mut self, spi: &mut SPI, reg: Register) -> Result<u8, E>
    where
        SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    {
        let buffer: [u8; 2] = self.read_many(spi, reg)?;
        Ok(buffer[1])
    }

    fn read_many<SPI, E>(&mut self, spi: &mut SPI, reg: Register) -> Result<[u8; 2], E>
    where
        SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    {
        let mut buffer: [u8; 2] = unsafe { mem::zeroed() };
        {
            let slice: &mut [u8] = &mut buffer;
            slice[0] = reg as u8;
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
    pub fn read_raw<SPI, E, D: DelayMs<u8>>(
        &mut self,
        spi: &mut SPI,
        delay: &mut D,
    ) -> Result<u16, E>
    where
        SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    {
        // Set up a one-shot conversion by enabling Vbias and OneShot.
        // See the `1-Shot (D5)` section of the datasheet for details.
        let existing_config = self.read_data(spi, Register::Config)?;

        let conf = existing_config | (1 << 7); // Enable VBias
        self.write(spi, Register::ConfigW, conf)?;

        // . (datasheet): Enable VBIAS and wait at least 10.5
        // time constants of the input RC network plus an additional
        // 1ms before initiating the conversion. Note that a single
        // conversion requires approximately 52ms in 60Hz filter
        // mode or 62.5ms in 50Hz filter mode to complete. 1-Shot
        // is a self-clearing bit.
        // todo: is this delay causing trouble?
        delay.delay_ms(60);

        // `When the conversion mode is set to “Normally Off”, write
        // 1 to this bit to start a conversion.`
        // Trigger a one-shot conversion.
        self.write(spi, Register::ConfigW, conf | (1 << 5))?;

        let msb = self.read_data(spi, Register::RtdMsb)? as u16;
        let lsb = self.read_data(spi, Register::RtdLsb)? as u16;

        // Turn off Vbias by writing the original config.
        self.write(spi, Register::ConfigW, existing_config)?;

        Ok((msb << 8) | lsb)
    }

    /// Measure RTD resistance, in Ohms.
    pub fn read_resistance<SPI, E, D: DelayMs<u8>>(
        &mut self,
        spi: &mut SPI,
        delay: &mut D,
    ) -> Result<f32, E>
    where
        SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    {
        let raw = self.read_raw(spi, delay)?;

        Ok((((raw >> 1) as u32 * self.calibration) >> 15) as f32)
    }

    /// Measure temperature, in Celsius
    pub fn read<SPI, E, D: DelayMs<u8>>(&mut self, spi: &mut SPI, delay: &mut D) -> Result<f32, E>
    where
        SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    {
        let resistance = self.read_resistance(spi, delay)?;
        let temp = lookup_temperature(resistance as u16, self.type_);

        Ok(temp as f32 / 100.)
    }

    /// Return the configuration register data.
    pub fn read_config<SPI, E>(&mut self, spi: &mut SPI) -> Result<[bool; 8], E>
    where
        SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    {
        let status = self.read_data(spi, Register::Config)?;

        let vbias = status & (1 << 7) > 0;
        let conv_mode = status & (1 << 6) > 0;
        let one_shot = status & (1 << 5) > 0;
        let wire = status & (1 << 4) > 0;
        let fault_deta = status & (1 << 3) > 0;
        let fault_detb = status & (1 << 2) > 0;
        let fault_status = status & (1 << 1) > 0;
        let filter = status & (1 << 0) > 0;

        Ok([
            vbias,
            conv_mode,
            one_shot,
            wire,
            fault_deta,
            fault_detb,
            fault_status,
            filter,
        ])
    }

    /// Find the fault status. See Table 7.
    pub fn fault_status<SPI, E>(&mut self, spi: &mut SPI) -> Result<[bool; 6], E>
    where
        SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    {
        let status = self.read_data(spi, Register::FaultStatus)?;

        let rtd_high = status & (1 << 7) > 0;
        let rtd_low = status & (1 << 6) > 0;
        let refin1 = status & (1 << 5) > 0;
        let refin2 = status & (1 << 4) > 0;
        let rtdin = status & (1 << 3) > 0;
        let overv = status & (1 << 2) > 0;

        Ok([rtd_high, rtd_low, refin1, refin2, rtdin, overv])
    }

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
    pub fn calibrate<SPI, E, D: DelayMs<u8>>(
        &mut self,
        spi: &mut SPI,
        delay: &mut D,
    ) -> Result<(), E>
    where
        SPI: Write<u8, Error = E> + Transfer<u8, Error = E>,
    {
        // todo
        let raw = self.read_raw(spi, delay)?;
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
fn lookup_temperature(val: u16, _type: RtdType) -> u32 {
    // todo: Take into account nominal resistance here, eg 100 or 1000ohm.

    let mut first = &(0, 10000);
    let mut second = &(1000, 10390);
    let mut iterator = LOOKUP_TABLE.iter();
    for a in &mut iterator {
        first = second;
        second = &a;
        if a.1 > val {
            break;
        }
    }
    let second = iterator.next();

    if let Some(second) = second {
        // Don't crash the program with a divide-by-0 error
        // if no device is connected.
        if (second.1 - first.1) == 0 {
            return 0;
        }
        (second.0 - first.0) as u32 * (val - first.1) as u32 / (second.1 - first.1) as u32
            + first.0 as u32
    } else {
        0
    }
}
