//! Code that triggers parts of the ec circuit: Analog Devices CN-0411.

use embedded_hal::{
    adc::OneShot,
    blocking::{delay::DelayMs, i2c},
    digital::v2::OutputPin,
};

use ads1x1x::{
    self,
    channel::{SingleA2, SingleA3},
};

// todo: Once you make `SingleChannelDac` more abstract, remove this.
use stm32f3xx_hal::{
    dac::Dac,
    pac::{self, TIM2},
    rcc::APB1,
    timer::{Channel, Timer},
};

use rtt_target::rprintln;

use crate::CalPtEc;

// Frequencies for the PWM channels. Higher frequencies reduce polarization.
// Lower frequencies reduce capacitance effects. The frequencies we choose are a compromise
// between the two.
// todo: finer resolution
// 94Hz for uS, and 2.4khz for mS range,
const PWM_THRESH_HIGH: f32 = 900.;
const PWM_THRESH_LOW: f32 = 400.;
// const PSC_LOW_FREQ: u16 = 532;  // 94 Hz. mS
// const PSC_MED_FREQ: u16 = 200;  // ? Hz. mS
// const PSC_HIGH_FREQ: u16 = 20;  // 2.4kHz. uS.

#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq)]
/// 94Hz for uS, and 2.4khz for mS range. The values here corresopnd to STM32
/// timer PSC registers, with ARR set at 240.
enum PwmFreq {
    Low = 532, // 94 Hz. mS
    Med = 80,  // ? Hz. mid
    High = 20, // 2.4kHz. uS
}

// `V_DEF` is the desired applied voltage across the conductivity electrodes.
// Loose requirement from AD engineers: "100mV is good enough", and don't
// overvolt the probe.
const V_DEF: f32 = 0.2;
// const F_LOW: u16 = 94; // uS range
// const F_HIGH: u16 = 2_400; // mS range

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EcGain {
    // Gain 1 isn't used. We set it up this way
    // so the gains match with the S values.
    One,   // S8. Highest resistance
    Two,   // S7
    Three, // S6
    Four,  // S5
    Five,  // S4
    Six,   // S3
    Seven, // S2. Lowest resistance
}

impl EcGain {
    /// Raise by one level.
    fn raise(&self) -> Self {
        // todo temp until you fix the mux!! Evens are broke due to broken pad
        match self {
            Self::One => Self::Three,
            Self::Two => Self::Three,
            Self::Three => Self::Five,
            Self::Four => Self::Five,
            Self::Five => Self::Seven,
            Self::Six => Self::Seven,
            Self::Seven => panic!("Gain is already at maximum"),
        }

        // match self {
        //     Self::One => Self::Two,
        //     Self::Two => Self::Three,
        //     Self::Three => Self::Four,
        //     Self::Four => Self::Five,
        //     Self::Five => Self::Six,
        //     Self::Six => Self::Seven,
        //     Self::Seven => panic!("Gain is already at maximum"),
        // }
    }
    /// Drop by one level.
    fn _drop(&self) -> Self {
        match self {
            Self::One => panic!("Gain is already at minimum"),
            Self::Two => Self::One,
            Self::Three => Self::Two,
            Self::Four => Self::Three,
            Self::Five => Self::Four,
            Self::Six => Self::Five,
            Self::Seven => Self::Six,
        }
    }

    /// Display the resistance associated with gain, in Ω.
    /// For gains Six and Seven, we must take into account resistance of the multiplexer circuit,
    /// since it's significant compared to the resistor value. This is why they deviate
    /// from the pattern. These values are taken from measurement.
    ///
    /// todo: Re-evaluate cal values on the final board for gains 6/7.
    fn resistance(&self) -> u32 {
        match self {
            Self::Seven => 31,  // Off 11 from ideal.
            Self::Six => 209, // Calibration value is 9 off from ideal.
            Self::Five => 2_000,
            Self::Four => 20_000,
            Self::Three => 200_000,
            Self::Two => 2_000_000,
            Self::One => 20_000_000,
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
        //
        match gain {
            EcGain::Seven => {
                self.pin0.set_high().ok();
                self.pin1.set_low().ok();
                self.pin2.set_low().ok();
            }
            EcGain::Six => {
                self.pin0.set_low().ok();
                self.pin1.set_high().ok();
                self.pin2.set_low().ok();
            }
            EcGain::Five => {
                self.pin0.set_high().ok();
                self.pin1.set_high().ok();
                self.pin2.set_low().ok();
            }
            EcGain::Four => {
                self.pin0.set_low().ok();
                self.pin1.set_low().ok();
                self.pin2.set_high().ok();
            }
            EcGain::Three => {
                self.pin0.set_high().ok();
                self.pin1.set_low().ok();
                self.pin2.set_high().ok();
            }
            EcGain::Two => {
                self.pin0.set_low().ok();
                self.pin1.set_high().ok();
                self.pin2.set_high().ok();
            }
            EcGain::One => {
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
pub struct EcSensor<P0, P1, P2>
where
    P0: OutputPin,
    P1: OutputPin,
    P2: OutputPin,
{
    pub dac: Dac,                         // todo pub temp
    pub gain_switch: ADG1608<P0, P1, P2>, // todo pub temp.
    K_cell: f32,                          // constant of the conductivity probe used.
    cal_1: CalPtEc,
    cal_2: Option<CalPtEc>,
    // Temp-compensated, in uS/cm2; post-processing. Used for determining
    // how to set the PWM freq for the next cycle.
    last_meas: PwmFreq,
}

impl<P0, P1, P2> EcSensor<P0, P1, P2>
where
    P0: OutputPin,
    P1: OutputPin,
    P2: OutputPin,
{
    /// Prior to taking measurements, set up the timer at the
    /// appropriate frequency and resolution.
    pub fn new(dac: Dac, switch_pins: (P0, P1, P2), K_cell: f32) -> Self {
        Self {
            dac,
            gain_switch: ADG1608::new(switch_pins.0, switch_pins.1, switch_pins.2),
            K_cell,
            cal_1: CalPtEc::new(100., 1000., 23.),
            cal_2: Some(CalPtEc::new(1_413., 1_413., 23.)),
            last_meas: PwmFreq::High,
        }
    }


    /// Set gain and excitation voltage, as an auto-ranging procedure.
    /// Return the values, for use in the computation. See CN-0411: Table 12.
    fn set_range<E, I2C, D: DelayMs<u16>>(
        &mut self,
        adc: &mut crate::Adc<I2C>,
        delay: &mut D,
    ) -> Result<(f32, EcGain), E>
    where
        I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E> + i2c::Read<Error = E>,
    {
        // Set multiplexer to highest gain resistance
        let mut gain = EcGain::One;
        self.gain_switch.set(gain);

        // temp fail r check:
        // One: reading 2mh. should be 20m.
        // Three: reading 20k. Should be 200k
        // Five: Reading 200. Should read 2k.
        // Seven: Reading 0. Should read 20.

        // S7 is correct: 011
        // S5 is correct: 001
        // S3 is correct: 010

        // s2 tries to set 100. Result is like 000. So perhaps A0 is bad.
        // s4 tries to set 110. Result is like 010
        // S6 tries to set 101. Result is like 001.
        // Conclusion: A0 is bbeing pulled to ground, or is floating.

        // Gain Six Calibration: Reading 209. Ideal is 200.

        // Latest:
        // One: 20M
        // Two: Wrong @ 20M
        // Three: 200k
        // Four: Wrong @ 200k
        // Five: 2k.
        // Six: Assume wrong
        // Seven: Wrigbh @ 31.2


        // Set DAC to Output V_EXC = 400mV, to start.
        let mut v_exc = 0.4;
        self.dac.set_voltage(v_exc);

        delay.delay_ms(100);
        // Read ADC Input V+ and V-
        let (mut v_p, mut v_m) = self.read_voltage(adc)?;

        // Note that higher conductivities result in higher gain, due to the nature of
        // our 2-part voltage divider. Resistance one is from the gain resistor; resistance
        // 2 is from the ec probe.
        while v_p + v_m <= (0.3 * 2. * v_exc as f32) && gain != EcGain::Seven {
            gain = gain.raise();
            self.gain_switch.set(gain);

            // todo: DRY!
            // Read ADC Input V+ and V-
            delay.delay_ms(100);
            let readings = self.read_voltage(adc)?;
            v_p = readings.0;
            v_m = readings.1;
            rprintln!("Vp: {:?}, Vm: {:?}", &v_p, &v_m);
        }

        v_exc = V_DEF * (v_exc as f32) / (v_p + v_m);
        self.dac.set_voltage(v_exc);

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

    /// Misleading name compared to other sensors. This is our uncalibrated reading, used
    /// for calibrating. Similar to others sensors' `read_voltage()`.
    pub fn read_direct<D, I2C, E>(
        &mut self,
        adc: &mut crate::Adc<I2C>,
        delay: &mut D,
        apb1: &mut APB1,
        timer: &mut Timer<TIM2>,
    ) -> Result<f32, E>
    where
        D: DelayMs<u16>,
        I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E> + i2c::Read<Error = E>,
    {
        // [dis]enabling the dac each reading should improve battery usage.
        self.dac.enable(apb1);
        start_pwm(timer);

        // Delay to charge the sample and hold before reading.
        let (v_exc, gain) = self.set_range(adc, delay)?;

        rprintln!("V: {:?}, Gain: {:?}", &v_exc, &gain);

        // the lowest freq we use is 94hz. Period = ~11ms.
        delay.delay_ms(100); // todo: What should this be?
        let (v_p, v_m) = self.read_voltage(adc)?;

        // rprintln!("+: {:?}, -: {:?}", &v_p, v_m);

        // stop_pwm(timer);
        // self.dac.disable(apb1);

        // See also
        // https://wiki.analog.com/resources/eval/user-guides/eval-adicup360/reference_designs/demo_cn0411
        // https://github.com/analogdevicesinc/EVAL-ADICUP360/blob/master/projects/ADuCM360_demo_cn0411/src/CN0411.c

        // todo: Experimenting with non-PWM, no sample+hold approach.
        // // Set high.
        // unsafe { (*pac::GPIOA::ptr()).bsrr.write(|w| w.bits(1 << 0)) };
        // while unsafe { (*pac::GPIOA::ptr()).idr.read().idr0.is_low() } {}
        // self.dac.enable(apb1);
        // delay.delay_ms(5);
        // let v_p = crate::voltage_from_adc(
        //     block!(adc
        //         // .as_mut()
        //         // .expect("Measurement after I2C freed")
        //         .read(&mut SingleA2))
        //     .unwrap_or(45),
        // ); // todo temp unwrap due to Error type mismatches.

        // Set low
        // unsafe { (*pac::GPIOA::ptr()).bsrr.write(|w| w.bits(1 << (16 + 0))) };
        // while unsafe { (*pac::GPIOA::ptr()).idr.read().idr0.is_high() } {}
        // delay.delay_ms(5);
        // let v_m = crate::voltage_from_adc(
        //     block!(adc
        //         // .as_mut()
        //         // .expect("Measurement after I2C freed")
        //         .read(&mut SingleA2))
        //     .unwrap_or(45),
        // ); // todo temp unwrap due to Error type mismatches.
        // self.dac.disable(apb1);

        //Res k:
        // Dry: inf
        // 100us: 4.35M
        // 1413us:


        // `pp` means peak-to-peak
        let V_cond_pp = 0.1 * v_p + 0.1 * v_m; // CN0411, Eq 6
        let I_cond_pp = (2. * v_exc - V_cond_pp) / (gain.resistance() as f32); // CN0411, Eq 7
        let Y_sol = self.K_cell * I_cond_pp / V_cond_pp; // CN0411, Eq 8

        Ok(Y_sol)
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
        D: DelayMs<u16>,
        I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E> + i2c::Read<Error = E>,
    {
        let Y_sol = self.read_direct(adc, delay, apb1, timer)?;

        let result = ec_from_reading(Y_sol, &self.cal_1, &self.cal_2, T);

        // if result > PWM_THRESH_HIGH && self.last_meas != PwmFreq::High {
        //     unsafe {
        //         (*TIM2::ptr())
        //             .psc
        //             .write(|w| w.psc().bits(PwmFreq::High as u16));
        //     }
        //     self.last_meas = PwmFreq::High;
        // } else if result > PWM_THRESH_LOW && self.last_meas != PwmFreq::Med {
        //     unsafe {
        //         (*TIM2::ptr())
        //             .psc
        //             .write(|w| w.psc().bits(PwmFreq::Med as u16));
        //     }
        //     self.last_meas = PwmFreq::Med;
        // } else if result < PWM_THRESH_LOW && self.last_meas != PwmFreq::Low {
        //     unsafe {
        //         (*TIM2::ptr())
        //             .psc
        //             .write(|w| w.psc().bits(PwmFreq::Low as u16));
        //     }
        //     self.last_meas = PwmFreq::Low;
        // }

        Ok(result)
    }

    pub fn calibrate_all(&mut self, pt0: CalPtEc, pt1: Option<CalPtEc>) {
        self.cal_1 = pt0;
        self.cal_2 = pt1;
    }
}

/// Convert sensor voltage to ORP voltage
/// We model the relationship between sensor output and ec linearly.
fn ec_from_reading(reading: f32, cal_0: &CalPtEc, cal_1: &Option<CalPtEc>, T: f32) -> f32 {
    // a is the slope, ec / reading.
    rprintln!("RAW: {:?}", &reading);
    let T_diff = T - cal_0.T;
    // let T_comp = PH_TEMP_C * T_diff; // pH / V
    let T_comp = 0.;

    // todo: Dry with `ph_from_voltage`
    let Y_sol = match cal_1 {
        // Model as a quadratic Lagrangian polynomia
        // 3 pt polynomial calibration, using a 3rd point at 0, 0.
        Some(c1) => {
            let result = crate::lg(
                (0., 0.),
                (cal_0.reading, cal_0.ec),
                (c1.reading, c1.ec),
                reading,
            );
            // todo: Eval this and in teh pH fn!
            (result + T_comp) * reading
        },
        // Model as a line, using (0., 0.) as the other pt.
        None => {
            let a = cal_0.ec / cal_0.reading;
            let b = cal_0.ec - a * cal_0.reading;
            (a + T_comp) * reading + b
        }
    };

    // Temperature compensation. Reference Analog Devices CN0411, equation 10.
    // We're picking cal point 1 as a temperature reference arbitrarily.

    // The electrical conductivity (EC) of an aqueous solution increases with temperature
    // significantly: about 2 per degree Celsius. (https://www.aqion.de/site/112)

    // let a = 2.14;  // Temperature coefficient. For KCl. Use 2.14 for NaCl.
    // Y_sol / (1. + a * (T - cal_0.T))
    Y_sol
}
