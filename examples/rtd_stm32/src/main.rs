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
    pac,
};
use rtt_target::{rprintln, rtt_init_print};
use stm32f3xx_hal as hal;

#[entry]
fn main() -> ! {
    rtt_init_print!();

    // Set up i2C.
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut delay = Delay::new(cp.SYST, clocks);

    // Set up SPI, where SCK is on PA5, MISO is on PA6, MOSI is on PA7, CS is on PA8,
    // and RDY is on PB11.
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

    let sck = gpioa.pa5.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let miso = gpioa.pa6.into_af5(&mut gpioa.moder, &mut gpioa.afrl);
    let mosi = gpioa.pa7.into_af5(&mut gpioa.moder, &mut gpioa.afrl);

    let cs = gpiob
        .pb11
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    let spi_mode = Mode {
        // Must use SPI mode 1 or 3.
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnSecondTransition,
    };

    let mut spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        spi_mode,
        4.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let mut rtd = Rtd::new(&mut spi, cs, RtdType::Pt100, Wires::Three);

    loop {
        rprintln!("Temp: {}°C", rtd.read(&mut spi));
        rprintln!("Resistance: {}Ω", rtd.read_resistance(&mut spi));
        rprintln!("Faults: {:?}", rtd.fault_status(&mut spi));

        delay.delay_ms(2_000_u16);
    }
}

#[panic_handler]
fn my_panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
