//! Driver for the MCP4921 DAC.
//! Based on [this package](https://github.com/eldruin/mcp49xx-rs),
//! but modified for this specific variant, borrows SPI, and eschews the
//! traits and macros that complicate that crate.

//! This is a platform-agnostic Rust driver for the MCP49xx and MCP48xx SPI
//! digital-to-analog converters (DAC), based on the [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//!
//! This driver allows you to:
//! - Set a channel to a value.
//! - Shutdown a channel.
//! - Use buffering on commands.
//! - Select gain.
//!
//! ## The devices
//! The Microchip Technology Inc. MCP49xx and MCP48xx devices are single/dual
//! channel 8-bit, 10-bit and 12-bit buffered voltage output Digital-to-Analog
//! Converters (DACs). The devices operate from a single 2.7V to 5.5V supply
//! with an SPI compatible Serial Peripheral Interface. The user can configure
//! the full-scale range of the device to be Vref or 2*Vref by setting the gain
//! selection option bit (gain of 1 of 2).
//!
//! The user can shut down the device by setting the Configuration Register bit.
//! In Shutdown mode, most of the internal circuits are turned off for power
//! savings, and the output amplifier is configured to present a known high
//! resistance output load (500 kΩ, typical).
//!
//! The devices include double-buffered registers, allowing synchronous updates
//! of the DAC output using the LDAC pin. These devices also incorporate a
//! Power-on Reset (POR) circuit to ensure reliable power-up.
//!
//! The devices utilize a resistive string architecture, with its inherent
//! advantages of low Differential Non-Linearity (DNL) error and fast settling
//! time. These devices are specified over the extended temperature range (+125°C).
//!
//! The devices provide high accuracy and low noise performance for consumer
//! and industrial applications where calibration or compensation of signals
//! (such as temperature, pressure and humidity) are required.
//!
//! This driver is compatible with these devices:
//!
//! | Device  | Resolution | Channels | Buffering     |
//! |---------|------------|----------|---------------|
//! | MCP4801 | 8-bit      | 1        | Not supported |
//! | MCP4802 | 8-bit      | 2        | Not supported |
//! | MCP4811 | 10-bit     | 1        | Not supported |
//! | MCP4812 | 10-bit     | 2        | Not supported |
//! | MCP4821 | 12-bit     | 1        | Not supported |
//! | MCP4822 | 12-bit     | 2        | Not supported |
//! | MCP4901 | 8-bit      | 1        | Supported     |
//! | MCP4902 | 8-bit      | 2        | Supported     |
//! | MCP4911 | 10-bit     | 1        | Supported     |
//! | MCP4912 | 10-bit     | 2        | Supported     |
//! | MCP4921 | 12-bit     | 1        | Supported     |
//! | MCP4922 | 12-bit     | 2        | Supported     |
//!
//! Datasheets:
//! - [MCP48x1](http://ww1.microchip.com/downloads/en/DeviceDoc/22244B.pdf)
//! - [MCP48x2](http://ww1.microchip.com/downloads/en/DeviceDoc/20002249B.pdf)
//! - [MCP49x1](http://ww1.microchip.com/downloads/en/DeviceDoc/22248a.pdf)
//! - [MCP49x2](http://ww1.microchip.com/downloads/en/DeviceDoc/22250A.pdf)
//!
//! ## The interface
//!
//! These devices support changing all configuration flags in each command
//! sent. In order to keep this flexibility, this driver does not provide
//! individual methods to set the settings but provides a `Command` struct
//! which can be used to specify all settings.
//! Then commands can be sent to the device through the `send()` method.
//!
//! ## Usage examples (see also examples folder)
//!
//! To use this driver, import this crate and an `embedded_hal` implementation,
//! then instantiate the appropriate device.
//! In the following examples an instance of the device MCP4921 will be created
//! as an example. Other devices can be created with similar methods like:
//! `Mcp49xx::new_mcp4822(...)`.
//!
//! Please find additional examples using hardware in this repository: [driver-examples]
//!
//! [driver-examples]: https://github.com/eldruin/driver-examples
//!
//! ### Set channel 0 to position 1024 in a MCP4921 device
//!
//! ```no_run
//! extern crate embedded_hal;
//! extern crate linux_embedded_hal;
//! extern crate mcp49xx;
//! use mcp49xx::{Channel, Command, Mcp49xx};
//! use linux_embedded_hal::{Pin, Spidev};
//!
//! # fn main() {
//! let spi = Spidev::open("/dev/spidev0.0").unwrap();
//! let chip_select = Pin::new(25);
//!
//! let mut dac = Mcp49xx::new_mcp4921(spi, chip_select);
//!
//! let cmd = Command::default();
//! let cmd = cmd.channel(Channel::Ch0).value(1024);
//! dac.send(cmd).unwrap();
//!
//! // Get SPI device and CS pin back
//! let (_spi, _chip_select) = dac.destroy();
//! # }
//! ```
//!
//! ### Set position and shutdown channels in a MCP4822 device
//!
//! ```no_run
//! extern crate embedded_hal;
//! extern crate linux_embedded_hal;
//! extern crate mcp49xx;
//! use mcp49xx::{Channel, Command, Mcp49xx};
//! use linux_embedded_hal::{Pin, Spidev};
//!
//! # fn main() {
//! let spi = Spidev::open("/dev/spidev0.0").unwrap();
//! let chip_select = Pin::new(25);
//!
//! let mut dac = Mcp49xx::new_mcp4822(spi, chip_select);
//!
//! let cmd = Command::default();
//! let cmd = cmd.channel(Channel::Ch1).value(1024);
//! dac.send(cmd).unwrap();
//!
//! let cmd = Command::default();
//! let cmd = cmd.channel(Channel::Ch0).shutdown();
//! dac.send(cmd).unwrap();
//!
//! // Get SPI device and CS pin back
//! let (_spi, _chip_select) = dac.destroy();
//! # }
//! ```
//!
//! ### Set position and activate buffering and double gain in a MCP4911 device
//!
//! ```no_run
//! extern crate embedded_hal;
//! extern crate linux_embedded_hal;
//! extern crate mcp49xx;
//! use mcp49xx::{Channel, Command, Mcp49xx};
//! use linux_embedded_hal::{Pin, Spidev};
//!
//! # fn main() {
//! let spi = Spidev::open("/dev/spidev0.0").unwrap();
//! let chip_select = Pin::new(25);
//!
//! let mut dac = Mcp49xx::new_mcp4911(spi, chip_select);
//!
//! let cmd = Command::default();
//! let cmd = cmd.channel(Channel::Ch0).buffered().double_gain().value(511);
//! dac.send(cmd).unwrap();
//!
//! // Get SPI device and CS pin back
//! let (_spi, _chip_select) = dac.destroy();
//! # }
//! ```

use embedded_hal::{
    blocking::spi,
    digital::v2::OutputPin,
    spi::{Mode, Phase, Polarity},
};

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// Communication error
    Comm(E),
    /// The channel provided is not available in the current device (MCP4xx1)
    InvalidChannel,
    /// The value provided does not fit the bitness of the current device
    InvalidValue,
    /// Buffering is not available in the current device (MCP48xx)
    BufferingNotSupported,
}

/// SPI mode (CPOL = 0, CPHA = 0)
pub const MODE0: Mode = Mode {
    phase: Phase::CaptureOnFirstTransition,
    polarity: Polarity::IdleLow,
};

/// SPI mode (CPOL = 1, CPHA = 1)
pub const MODE1: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

/// MCP49xx digital potentiometer driver
#[derive(Debug, Default)]
pub struct Mcp4921<CS> {
    cs: CS,
}

impl<CS: OutputPin> Mcp4921<CS> {
    pub fn new(cs: CS) -> Self {
        Self { cs }
    }

    /// Send command to device.
    ///
    /// This will return an error if the command is not appropriate for the current device:
    /// - If the channel is not available it will return `Error::InvalidChannel`.
    /// - If the value is too big it will return `Error::InvalidValue`.
    /// - If buffering is not supported it will return `Error::BufferingNotSupported`.
    ///
    /// Otherwise if a communication error happened it will return `Error::Comm`.
    pub fn send<SPI, E>(&mut self, spi: &mut SPI, command: Command) -> Result<(), Error<E>>
    where
        SPI: spi::Write<u8, Error = E>,
    {
        check_channel_is_appropriate(command.channel)?;
        check_value_is_appropriate(command.value)?;
        check_buffering_is_appropriate(command.buffered)?;
        let value = get_value_for_spi(command.value);

        let command_ = command.get_config_bits() | value[0];
        let data = value[1];

        self.cs.set_low().ok();

        let payload: [u8; 2] = [command_, data];
        let result = spi.write(&payload).map_err(Error::Comm);

        self.cs.set_high().ok();
        result
    }
}

fn check_value_is_appropriate<E>(value: u16) -> Result<(), Error<E>> {
    if value >= 1 << 12 {
        Err(Error::InvalidValue)
    } else {
        Ok(())
    }
}
fn get_value_for_spi(value: u16) -> [u8; 2] {
    [(value >> 8) as u8, (value & 0xff) as u8]
}

fn check_channel_is_appropriate<E>(channel: Channel) -> Result<(), Error<E>> {
    if channel != Channel::Ch0 {
        Err(Error::InvalidChannel)
    } else {
        Ok(())
    }
}

fn check_buffering_is_appropriate<E>(_buffered: bool) -> Result<(), Error<E>> {
    Ok(())
}

/// Channel selector
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Channel {
    /// Channel 0
    Ch0,
    /// Channel 1 (only valid for dual devices. i.e. MCP4xx2)
    ///
    /// Sending a command on this channel to a single channel device will
    /// return an `Error::InvalidChannel`.
    Ch1,
}

/// Configurable command that can be sent to the device
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Command {
    pub(crate) channel: Channel,
    pub(crate) buffered: bool,
    double_gain: bool,
    shutdown: bool,
    pub(crate) value: u16,
}

impl Default for Command {
    /// Create new command instance.
    ///
    /// Per default the command is on channel 0, unbuffered, with single gain,
    /// enabled and with value 0.
    fn default() -> Self {
        Command {
            channel: Channel::Ch0,
            buffered: false,
            double_gain: false,
            shutdown: false,
            value: 0,
        }
    }
}

impl Command {
    /// Select the channel
    pub fn channel(self, channel: Channel) -> Self {
        let mut cmd = self;
        cmd.channel = channel;
        cmd
    }

    /// Shutdown the channel
    pub fn shutdown(self) -> Self {
        let mut cmd = self;
        cmd.shutdown = true;
        cmd
    }

    /// Enable the channel (undo a shutdown)
    pub fn enable(self) -> Self {
        let mut cmd = self;
        cmd.shutdown = false;
        cmd
    }

    /// Send the value buffered
    pub fn buffered(self) -> Self {
        let mut cmd = self;
        cmd.buffered = true;
        cmd
    }

    /// Send the value unbuffered
    pub fn unbuffered(self) -> Self {
        let mut cmd = self;
        cmd.buffered = false;
        cmd
    }

    /// Send the value with double gain (2x)
    pub fn double_gain(self) -> Self {
        let mut cmd = self;
        cmd.double_gain = true;
        cmd
    }

    /// Send the value with single gain (1x)
    pub fn single_gain(self) -> Self {
        let mut cmd = self;
        cmd.double_gain = false;
        cmd
    }

    /// Set the value
    pub fn value(self, value: u16) -> Self {
        let mut cmd = self;
        cmd.value = value;
        cmd
    }

    // get the config bits at the beginning of the command
    pub(crate) fn get_config_bits(self) -> u8 {
        let mut value = 0b0011_0000;
        if self.channel != Channel::Ch0 {
            value |= 0b1000_0000;
        }
        if self.buffered {
            value |= 0b0100_0000;
        }
        if self.double_gain {
            value &= 0b1101_1111;
        }
        if self.shutdown {
            value &= 0b1110_1111;
        }
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn check_cmd(
        cmd: Command,
        channel: Channel,
        buffered: bool,
        double_gain: bool,
        shutdown: bool,
        value: u16,
    ) {
        assert_eq!(cmd.channel, channel);
        assert_eq!(cmd.buffered, buffered);
        assert_eq!(cmd.double_gain, double_gain);
        assert_eq!(cmd.shutdown, shutdown);
        assert_eq!(cmd.value, value);
    }

    #[test]
    fn default_command_all_off() {
        let cmd = Command::default();
        check_cmd(cmd, Channel::Ch0, false, false, false, 0);
    }

    #[test]
    fn can_set_double_gain() {
        let cmd = Command::default().double_gain();
        check_cmd(cmd, Channel::Ch0, false, true, false, 0);
    }

    #[test]
    fn can_set_single_gain() {
        let cmd = Command::default().double_gain().single_gain();
        check_cmd(cmd, Channel::Ch0, false, false, false, 0);
    }

    #[test]
    fn can_set_shutdown() {
        let cmd = Command::default().shutdown();
        check_cmd(cmd, Channel::Ch0, false, false, true, 0);
    }

    #[test]
    fn can_set_enable() {
        let cmd = Command::default().shutdown().enable();
        check_cmd(cmd, Channel::Ch0, false, false, false, 0);
    }

    #[test]
    fn can_set_buffered() {
        let cmd = Command::default().buffered();
        check_cmd(cmd, Channel::Ch0, true, false, false, 0);
    }

    #[test]
    fn can_set_unbuffered() {
        let cmd = Command::default().buffered().unbuffered();
        check_cmd(cmd, Channel::Ch0, false, false, false, 0);
    }

    #[test]
    fn can_set_value() {
        let cmd = Command::default().value(1024);
        check_cmd(cmd, Channel::Ch0, false, false, false, 1024);
    }

    #[test]
    fn operations_leave_original_command_unchanged() {
        let original = Command::default();
        check_cmd(original.shutdown(), Channel::Ch0, false, false, true, 0);
        check_cmd(original, Channel::Ch0, false, false, false, 0);
    }
}
