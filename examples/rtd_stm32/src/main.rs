//! This program demonstrates how to use the AnyLeaf pH module
//! with an STM32F3 microcontroller. It continuously displays
//! the pH using an output handler, and demonstrates how
//! to calibrate.

#![no_main]
#![no_std]

use anyleaf::{Rtd, RtdType, Wires};
use cortex_m;
use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use hal::{
    delay::Delay,
    prelude::*,
    spi::{Mode, Phase, Polarity, Spi},
    stm32,
};
use rtt_target::{rprintln, rtt_init_print};
use stm32f3xx_hal as hal;

#[entry]
fn main() -> ! {
    rtt_init_print!();

    // Set up i2C.
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    // let clocks = rcc.cfgr.freeze(&mut flash.acr);

  let clocks = rcc
        .cfgr
        // .use_hse(8.mhz())
        // .sysclk(48.mhz())
        // .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    let mut delay = Delay::new(cp.SYST, clocks);

    // Set up SPI, where SCK is on PA5, MISO is on PA6, MOSI is on PA7, CS is on PA8,
    // and RDY is on PA9.
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);

    let cs = gpioa
        .pa1
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    let spi_mode = Mode {
        // Must use SPI mode 1 or 3.
        polarity: Polarity::IdleHigh,
        phase: Phase::CaptureOnSecondTransition,
    };

    let mut spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        spi_mode,
        2.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let mut rtd = Rtd::new(&mut spi, cs, RtdType::Pt100, Wires::Three);

    loop {
        // rprintln!("Temp: {}°C", rtd.read(&mut spi).unwrap());
        // rprintln!("Resistance: {}Ω", rtd.read_resistance(&mut spi).unwrap());
        // rprintln!("Faults: {:?}", rtd.fault_status(&mut spi).unwrap());
        rprintln!("T: {:?}", rtd.test(&mut spi).unwrap());

        delay.delay_ms(2_000_u16);
    }
}

// This handler will cause a crash if present in Debug, and one if not present
// in Release mode. We should be only building in release mode, since it offers
// a large performance boost. So much so, that any increase in compile time
// is offset by the faster init speed.
#[panic_handler]
fn my_panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
