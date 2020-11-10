//! Code that triggers parts of the ec circuit: Analog Devices CN-0411.
//! References:
//! https://www.analog.com/en/design-center/reference-designs/circuits-from-the-lab/cn0411.html
//! https://wiki.analog.com/resources/eval/user-guides/eval-adicup360/reference_designs/demo_cn0411
//! https://github.com/analogdevicesinc/EVAL-ADICUP360/blob/master/projects/ADuCM360_demo_cn0411/src/CN0411.c

use embedded_hal::{
    adc::OneShot,
    blocking::{delay::DelayMs, i2c},
    digital::v2::OutputPin,
};

use ads1x1x::{self, channel::SingleA2};

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
const PWM_THRESH_HIGH: f32 = 1_200.;
const PWM_THRESH_LOW: f32 = 400.;
// const PSC_LOW_FREQ: u16 = 532;  // 94 Hz. mS
// const PSC_MED_FREQ: u16 = 200;  // 617 Hz. mS
// const PSC_HIGH_FREQ: u16 = 20;  // 2.4kHz. uS.

// `V_PROBE_TGT` is the desired applied voltage across the conductivity electrodes.
// Loose requirement from AD engineers: "100mV is good enough", and don't
// overvolt the probe.
// const V_PROBE_TGT: f32 = 0.1;
const V_PROBE_TGT: f32 = 0.15;

// `V_EXC_INIT` is our initial excitation voltage, set by the DAC.
const V_EXC_INIT: f32 = 0.4;
const AMP_GAIN: f32 = 10.; // Multiplication factor of the instrumentation amp.

// Take the average of several readings, to produce smoother results.
// A higher value of `N_SAMPLES` is more accurate, but takes longer.
const N_SAMPLES: u8 = 10;

#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq)]
/// 94Hz for uS, and 2.4khz for mS range. The values here corresopnd to STM32
/// timer PSC registers, with ARR set at 240.
enum PwmFreq {
    Low = 532, // 94 Hz. mS
    Med = 80,  // ? Hz. mid
    High = 20, // 2.4kHz. uS
}

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

    /// Display the resistance associated with gain, in Ω.
    /// For gains Six and Seven, we must take into account resistance of the multiplexer circuit,
    /// since it's significant compared to the resistor value. This is why they deviate
    /// from the pattern. These values are taken from measurement.
    ///
    /// todo: Re-evaluate cal values on the final board for gains 6/7.
    fn resistance(&self) -> u32 {
        match self {
            Self::Seven => 31, // Off 11 from ideal.
            Self::Six => 209,  // Calibration value is 9 off from ideal.
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
pub fn _start_pwm(timer: &mut Timer<TIM2>) {
    timer.enable(Channel::One);
    timer.enable(Channel::Two);
    timer.enable(Channel::Three);
}

/// Stop all 3 PWM channels
pub fn _stop_pwm(timer: &mut Timer<TIM2>) {
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

        // Set DAC to Output V_EXC; eg = 400mV, to start.
        // let mut V_exc = V_EXC_INIT;
        let mut V_exc = 0.4;
        self.dac.set_voltage(V_exc);

        delay.delay_ms(40);
        // Read ADC Input V+ and V-
        let mut readings = self.read_voltage(adc)?;

        // todo: Dry with the read circuit. If you end up using it here, make it its
        // todo own fn.
        let mut V_probe = (readings.0 + readings.1) / 2.;
        let mut I = (V_exc - V_probe) / gain.resistance() as f32;

        // Note that higher conductivities result in higher gain, due to the nature of
        // our 2-part voltage divider. Resistance one is from the gain resistor; resistance
        // 2 is from the ec probe.

        // We adjust gain until probe resistance and gain circuit resistance are
        // at the same order of magnitude. We'd like the gain resistance
        // to be higher than probe.

        // Note: You exceed full-scale ADC range when probe resistance is
        // higher than gain res
        rprintln!("V: {:?} I: {:?}, R: {:?}", V_probe, I, gain.resistance());

        // todo: Maintain voltage and mayb egain for next reading, or don't
        // todo set range every time.

        // We attempt to get probe and gain-circuit resistances to be on the same
        // order of magnitude. We keep gain resistance higher than probe resistance,
        // so we don't exceed the ADC fullscale range (currently set to +-2.048V). Note
        // that this assumes a 10x voltage amplification by the instrumentation amp.
        // If V_probe is set to 0.1, and probe resistance <= gain resistance,
        // we should stay within that range.
        // while ((gain.resistance() as f32) >= R_probe) && gain != EcGain::Seven {
        // todo: Make this 0.3 a constant and find out what it means.
        // while (R_sol < gain.resistance() as f32 * 10.) && gain != EcGain::Seven {
        // while ((v_p + v_m) / 2.) <= (0.3 * v_exc as f32) && gain != EcGain::Seven {

        // Initially, it's likely the gain resistance will be much higher than probe resistance;
        // Very little voltage will be read at the ADC. Increase gain until we get a voltage
        // on the order of magnitude of v_exc. This also means gain resistance and probe
        // resistance are on the same order of magnitude.
        while V_probe <= (0.3 * V_exc as f32) && gain != EcGain::Seven {
            // todo: Dry setting gain and v_exc between here, and before the loop
            gain = gain.raise();
            self.gain_switch.set(gain);

            // .1V reading. <= 1.2V limit

            // Read ADC Input V+ and V-
            delay.delay_ms(50);
            readings = self.read_voltage(adc)?;

            V_probe = (readings.0 + readings.1) / 2.;
            I = (V_exc - V_probe) / gain.resistance() as f32;
            rprintln!("V: {:?} I: {:?}, R: {:?}", V_probe, I, gain.resistance());
        }

        // Set excitation voltage to deliver a target voltage across the probe. This prevents
        // overvolting both the probe, and ADC. (Since ADC input is v_probe, amplified.)
        // See `measure` for how we
        V_exc = I * gain.resistance() as f32 + V_PROBE_TGT;
        self.dac.set_voltage(V_exc);
        rprintln!("FINAL vexc: {:?}", &V_exc);

        delay.delay_ms(40);

        Ok((V_exc, gain))
    }

    /// We use the PWM timer's channels 2 and 3 to trigger readings at the appropriate times in
    /// the cycle. They're configured in `util::setup_pwm`, and trigger in the middle of
    /// each of the 2 polarities.
    ///
    /// We divide the read voltage by the AMP; we're looking for a pre-amp voltage,
    /// but reading post-amp.
    pub(crate) fn read_voltage<I2C, E>(
        &mut self,
        adc: &mut crate::Adc<I2C>,
    ) -> Result<(f32, f32), E>
    where
        I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E> + i2c::Read<Error = E>,
    {
        // loop {
        //     unsafe { (*pac::TIM2::ptr()).sr.modify(|_, w| w.cc2if().clear_bit()) }
        //     unsafe { (*pac::TIM2::ptr()).sr.modify(|_, w| w.cc3if().clear_bit()) }
        //     while unsafe { !(*pac::TIM2::ptr()).sr.read().cc2if().bit() && !(*pac::TIM2::ptr()).sr.read().cc3if().bit() } {}
        //     if unsafe { (*pac::TIM2::ptr()).sr.read().cc2if().bit() } {
        //         rprintln!("2");
        //     } else if unsafe { (*pac::TIM2::ptr()).sr.read().cc3if().bit() } {
        //        rprintln!("3");
        //     }
        // }

        unsafe { (*pac::TIM2::ptr()).sr.modify(|_, w| w.cc2if().clear_bit()) }
        // while unsafe { !(*pac::TIM2::ptr()).sr.read().cc2if().bit() } {}
        let v_p = crate::voltage_from_adc(block!(adc.read(&mut SingleA2)).unwrap_or(0));

        // todo: PUt the waits back!!
        unsafe { (*pac::TIM2::ptr()).sr.modify(|_, w| w.cc3if().clear_bit()) }
        // while unsafe { !(*pac::TIM2::ptr()).sr.read().cc3if().bit() } {}
        let v_m = crate::voltage_from_adc(block!(adc.read(&mut SingleA2)).unwrap_or(0));

        Ok((v_p / AMP_GAIN, v_m / AMP_GAIN))
    }

    /// Take an uncalibrated reading, without adjusting PWM.
    /// Similar in use to others sensors' `read_voltage()`.
    pub fn measure<D, I2C, E>(
        &mut self,
        adc: &mut crate::Adc<I2C>,
        delay: &mut D,
        apb1: &mut APB1,
    ) -> Result<f32, E>
    where
        D: DelayMs<u16>,
        I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E> + i2c::Read<Error = E>,
    {
        // [dis]enabling the dac each reading should improve battery usage.
        self.dac.enable(apb1);
        // todo: Set ADC sample speed here?
        delay.delay_ms(50); // Delay to let current flow from the DAC. // todo: Customize this.

        // Delay to charge the sample and hold before reading.
        let (V_exc, gain) = self.set_range(adc, delay)?;
        rprintln!("Vexc: {:?}, Gain: {:?}", &V_exc, &gain);

        // todo: Put back. We took thsi out to test non-sample-hold/pwm approach
        let mut v_p_cum = 0.;
        let mut v_m_cum = 0.;

        for _ in 0..N_SAMPLES {
            // the lowest freq we use is 94hz. Period = ~11ms.
            let (v_p, v_m) = self.read_voltage(adc)?;
            // rprintln!("Vp: {:?}, vm: {:?}", &v_p, &v_m);
            delay.delay_ms(10);
            v_p_cum += v_p;
            v_m_cum += v_m;
        }

        // self.dac.disable(apb1); // todo: temp; put back

        let v_p = v_p_cum / N_SAMPLES as f32;
        let v_m = v_m_cum / N_SAMPLES as f32;

        // We calculate conductivity across the probe using a voltage-divider model.
        // We know the following:
        // V_exc (set by DAC)
        // R_gain (set by multiplexer)
        // V_out (measuring by ADC), is the same as V_probe
        //
        // We solve for current, and use it to calculate conductivity:
        //
        // I = V_exc / (R_gain + R_probe) = V_exc / (R_gain + V_probe / I)
        // I(R_gain + V_probe / I) = V_exc
        // I x R_gain + V_probe = V_exc
        // I = (V_exc - V_probe) / R_gain

        let V_probe = (v_p + v_m) / 2.;
        // `I` is the current flowing through the gain circuitry, then probe.
        let I = (V_exc - V_probe) / gain.resistance() as f32;
        // K is in 1/cm. eg: K=1.0 could mean 2 1cm square plates 1cm apart.
        let Y_sol = self.K_cell * I / V_probe;

        rprintln!("V: {:?} I: {:?}, R: {:?}", V_probe, I, gain.resistance());

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
        let Y_sol = self.measure(adc, delay, apb1)?;

        let result = ec_from_reading(Y_sol, &self.cal_1, &self.cal_2, T);

        // todo: Put back
        // if result > PWM_THRESH_HIGH && self.last_meas != PwmFreq::High {
        //     unsafe {
        //         (*TIM2::ptr())
        //             .psc
        //             .write(|w| w.psc().bits(PwmFreq::High as u16));
        //     }
        //     self.last_meas = PwmFreq::High;
        // } else if result > PWM_THRESH_LOW
        //     && result < PWM_THRESH_HIGH
        //     && self.last_meas != PwmFreq::Med
        // {
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

/// Apply 1 point calibration to the readings.
fn ec_from_reading(reading: f32, cal_0: &CalPtEc, cal_1: &Option<CalPtEc>, T: f32) -> f32 {
    // a is the slope, ec / reading.
    rprintln!("RAW uS/cm: {:?}", reading * 1_000_000.);
    let T_diff = T - cal_0.T;
    // let T_comp = PH_TEMP_C * T_diff; // pH / V
    let T_comp = 0.;

    // todo: Dry with `ph_from_voltage`
    let Y_sol = match cal_1 {
        // Model as a quadratic Lagrangian polynomia
        // 3 pt polynomial calibration, using a 3rd point at 0, 0.
        Some(c1) => {
            // a is the slope, pH / v.

            // todo: DRY.
            // Model as two lines, connecting 0, the lower point, and the higher point.

            let (l, h) = if cal_0.reading < c1.reading {
                (cal_0, c1)
            } else {
                (c1, cal_0)
            };

            let l = if cal_0.reading < c1.reading {
                cal_0.clone()
            } else {
                c1.clone()
            };
            if reading < l.reading {
                let a = l.ec / l.reading;
                let b = l.ec - a * l.reading;
                a * reading + b
            } else {
                let a = (h.ec - cal_0.ec) / (c1.reading - cal_0.reading);
                let b = c1.ec - a * c1.reading;
                let result = (a + T_comp) * reading + b;
                result
            }

            // todo: Polynomial isn't given good results.
            // let result = crate::lg(
            //     (0., 0.),
            //     (cal_0.reading, cal_0.ec),
            //     (c1.reading, c1.ec),
            //     reading,
            // );
            // todo: Eval this and in teh pH fn!
            // (result + T_comp) * reading
        }
        // Model as a line, using (0., 0.) as the other pt.
        None => {
            let a = cal_0.ec / cal_0.reading;
            let b = cal_0.ec - a * cal_0.reading;
            (a + T_comp) * reading + b
        }
    };

    // todo: Model as 2 lines instead? Polynomial is not giving good results
    // todo outside the range.

    // Temperature compensation. Reference Analog Devices CN0411, equation 10.
    // We're picking cal point 1 as a temperature reference arbitrarily.

    // The electrical conductivity (EC) of an aqueous solution increases with temperature
    // significantly: about 2 per degree Celsius. (https://www.aqion.de/site/112)

    // let a = 2.14;  // Temperature coefficient. For KCl. Use 2.14 for NaCl.
    // Y_sol / (1. + a * (T - cal_0.T))
    Y_sol
}

// todo: Consider only allowing one conductivity pt!
