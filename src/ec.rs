//! Code that triggers parts of the ec circuit: Analog Devices CN-0411.

#![allow(non_snake_case)]

// todo: temp
use rtt_target::rprintln;

use embedded_hal::{
    adc::OneShot,
    blocking::{
        delay::{DelayMs, DelayUs},
        i2c,
    },
    digital::v2::OutputPin,
    // PwmPin,
};

use ads1x1x::{
    self,
    channel::{SingleA2, SingleA3},
};

// todo: Once you make `SingleChannelDac` more abstract, remove this.
use stm32f3xx_hal::{
    dac::SingleChannelDac,
    pac::TIM2,
    rcc::APB1,
    timer::{Channel, Timer},
};

// Frequencies for the PWM channels.
// const F_LOW: u16 = 94; // uS range
// const F_HIGH: u16 = 2_400; // mS range

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EcGain {
    // Gain 1 isn't used. We set it up this way
    // so the gains match with the S values.
    Two,   // S2
    Three, // S3
    Four,  // S4
    Five,  // S5
    Six,   // S6
    Seven, // S7
    Eight, // S8
}

impl EcGain {
    /// Raise by one level.
    fn raise(&self) -> Self {
        match self {
            Self::Two => Self::Three,
            Self::Three => Self::Four,
            Self::Four => Self::Five,
            Self::Five => Self::Six,
            Self::Six => Self::Seven,
            Self::Seven => Self::Eight,
            Self::Eight => panic!("Gain is already at maximum"),
        }
    }
    /// Drop by one level.
    fn drop(&self) -> Self {
        match self {
            Self::Two => panic!("Gain is already at minimum"),
            Self::Three => Self::Two,
            Self::Four => Self::Three,
            Self::Five => Self::Four,
            Self::Six => Self::Five,
            Self::Seven => Self::Six,
            Self::Eight => Self::Seven,
        }
    }

    /// Display the resistance associated with gain, in Ω.
    fn resistance(&self) -> u32 {
        match self {
            Self::Two => 20,
            Self::Three => 200,
            Self::Four => 2_000,
            Self::Five => 20_000,
            Self::Six => 200_000,
            Self::Seven => 2_000_000,
            Self::Eight => 20_000_000,
        }
    }
}

/// Use the ADG1608 to select the right resistor.
#[derive(Debug)]
pub struct ADG1608<P0: OutputPin, P1: OutputPin, P2: OutputPin> {
    // todo pub is temp
    pin0: P0,
    pin1: P1,
    pin2: P2,
}

impl<P0: OutputPin, P1: OutputPin, P2: OutputPin> ADG1608<P0, P1, P2> {
    pub fn new(pin0: P0, pin1: P1, pin2: P2) -> Self {
        Self { pin0, pin1, pin2 }
    }

    pub fn set(&mut self, gain: EcGain) {
        // Enable pin must be pulled high for this to work.
        match gain {
            EcGain::Eight => {
                self.pin0.set_high().ok();
                self.pin1.set_low().ok();
                self.pin2.set_low().ok();
            }
            EcGain::Seven => {
                self.pin0.set_low().ok();
                self.pin1.set_high().ok();
                self.pin2.set_low().ok();
            }
            EcGain::Six => {
                self.pin0.set_high().ok();
                self.pin1.set_high().ok();
                self.pin2.set_low().ok();
            }
            EcGain::Five => {
                self.pin0.set_low().ok();
                self.pin1.set_low().ok();
                self.pin2.set_high().ok();
            }
            EcGain::Four => {
                self.pin0.set_high().ok();
                self.pin1.set_low().ok();
                self.pin2.set_high().ok();
            }
            EcGain::Three => {
                self.pin0.set_low().ok();
                self.pin1.set_high().ok();
                self.pin2.set_high().ok();
            }
            EcGain::Two => {
                self.pin0.set_high().ok();
                self.pin1.set_high().ok();
                self.pin2.set_high().ok();
            }
        }
    }
}

// pub fn start_pwm<PWM0, PWM1, PWM2, D>(p0: &mut PWM0, p1: &mut PWM1, p2: &mut PWM2, delay: &mut D)
pub fn start_pwm(timer: &mut Timer<TIM2>) {
    timer.enable(Channel::One);
    timer.enable(Channel::Two);
    timer.enable(Channel::Three);
}

// pub fn stop_pwm<PWM0, PWM1, PWM2>(p0: &mut PWM0, p1: &mut PWM1, p2: &mut PWM2)
// pub fn stop_pwm(p0: &mut PWM0, p1: &mut PWM1, p2: &mut PWM2)
/// Stop all 3 PWM channels
pub fn stop_pwm(timer: &mut Timer<TIM2>) {
    timer.disable(Channel::One);
    timer.disable(Channel::Two);
    timer.disable(Channel::Three);
}

/// The high-level struct representing the EC circuit.
#[derive(Debug)]
pub struct EcSensor<DAC, P0, P1, P2>
where
    // todo: There has ato be a way to keep these trait bounds local.
    DAC: SingleChannelDac,
    P0: OutputPin,
    P1: OutputPin,
    P2: OutputPin,
{
    pub dac: DAC,                         // todo pub temp
    pub gain_switch: ADG1608<P0, P1, P2>, // todo pub temp.
    K_cell: f32, // constant of the conductivity probe used.
}

impl<DAC, P0, P1, P2> EcSensor<DAC, P0, P1, P2>
where
    DAC: SingleChannelDac,
    P0: OutputPin,
    P1: OutputPin,
    P2: OutputPin,
{
    pub fn new(dac: DAC, switch_pins: (P0, P1, P2), K_cell: f32) -> Self {
        Self {
            dac,
            gain_switch: ADG1608::new(switch_pins.0, switch_pins.1, switch_pins.2),
            K_cell,
        }
    }

    /// Set gain and excitation voltage, as an auto-ranging procedure.
    /// Return the values, for use in the computation. See CN-0411: Table 12.
    fn set_range<E, I2C>(&mut self, adc: &mut crate::Adc<I2C>) -> Result<(f32, EcGain), E>
    where
        I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E> + i2c::Read<Error = E>,
    {
        // todo: Experimenting with fixed gain as temp measure.
        self.dac.try_set_voltage(0.4).ok();
        // self.gain_switch.set(EcGain::Five);
        // return Ok((0.8, EcGain::Five));

        // Set multiplexer to highest gain resistance
        let mut gain = EcGain::Eight;
        self.gain_switch.set(gain);

        // Set DAC to Output V_EXC = 400mV
        let mut v_exc = 0.4;
        self.dac.try_set_voltage(v_exc).ok();

        // Read ADC Input V+ and V-
        let (mut v_p, mut v_m) = self.read_voltage(adc)?;

        // `v_def` is the desired applied voltage across the conductivity electrodes.
        // todo: How do we set it?
        let v_def = 1.5;

        // todo: Remove the 3 requirement. It's only for the lack of 20Mohm resistor for now.
        while v_p + v_m <= 0.3 * 2. * v_exc as f32 && gain != EcGain::Two && gain != EcGain::Three {
            gain = gain.drop();
            self.gain_switch.set(gain);

            // todo: DRY!
            // Read ADC Input V+ and V-
            let readings = self.read_voltage(adc)?;
            v_p = readings.0;
            v_m = readings.1;
        }

        v_exc = v_def * (v_exc as f32) / (v_p + v_m);
        self.dac.try_set_voltage(v_exc).ok();

        Ok((v_exc, gain))
    }

    /// Read the two voltages associated with ec measurement from the ADC.
    /// Assumes the measurement process has already been set up.
    pub(crate) fn read_voltage<I2C, E>(
        &self,
        adc: &mut crate::Adc<I2C>,
        // ) -> Result<(f32, f32), ads1x1x::Error<E>>
    ) -> Result<(f32, f32), E>
    where
        I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E> + i2c::Read<Error = E>,
    {
        // We use two additional pins on the same ADC as the ORP sensor.
        let v_p = crate::voltage_from_adc(
            block!(adc
                // .as_mut()
                // .expect("Measurement after I2C freed")
                .read(&mut SingleA2))
            .unwrap_or(45),
        ); // todo temp unwrap due to Error type mismatches.

        let v_m = crate::voltage_from_adc(
            block!(adc
                // .as_mut()
                // .expect("Measurement after I2C freed")
                .read(&mut SingleA3))
            .unwrap_or(45),
        ); // todo temp unwrap

        Ok((v_p, v_m))
    }

    /// Take a conductivity measurement. Result is in uS/cm
    /// todo: Way to read TDS (sep fn, or perhaps an enum)
    /// todo: Return result or option.
    pub fn read<D, I2C, E>(
        &mut self,
        adc: &mut crate::Adc<I2C>,
        delay: &mut D,
        T: f32,
        apb1: &mut APB1,
        timer: &mut Timer<TIM2>,
    ) -> Result<f32, E>
    where
        D: DelayUs<u16> + DelayMs<u16>,
        I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E> + i2c::Read<Error = E>,
    {
        // [dis]enabling the dac each reading should improve battery usage.
        self.dac.try_enable(apb1).ok(); // todo: Put back

        // TODO: pUT BACK
        // start_pwm(timer);
        let (v_exc, gain) = self.set_range(adc)?;

        delay.delay_ms(500); // todo experiment

        let (v_p, v_m) = self.read_voltage(adc)?;

        delay.delay_ms(500); // todo experiment

        // todo: put back
        // stop_pwm(timer);
        // todo: Put back
        // self.dac.try_disable(apb1).ok();

        // `pp` means peak-to-peak
        let V_cond_pp = 0.1 * v_p + 0.1 * v_m; // CN0411, Eq 6
        let I_cond_pp = (2. * v_exc - V_cond_pp) / (gain.resistance() as f32); // CN0411, Eq 7
        let Y_sol = self.K_cell * I_cond_pp / V_cond_pp; // CN0411, Eq 8

        // todo: Temp compensation:

        Ok(Y_sol) // todo
    }
}
