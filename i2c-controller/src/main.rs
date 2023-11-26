#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::gpio::{FunctionI2C, Pin, PullNone},
};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::*;
use panic_probe as _;

use fugit::RateExtU32;
use rp_pico as bsp;

use bsp::hal::{
    self as hal,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let mut led_pin = pins.led.into_push_pull_output();

    // Configure two pins as being I²C, not GPIO
    let sda_pin: Pin<_, FunctionI2C, PullNone> = pins.gpio0.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, PullNone> = pins.gpio1.reconfigure();

    // Create the I²C driver, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let mut i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    loop {
        delay.delay_ms(500);
        led_pin.set_high().unwrap();
        let r = i2c.write(0x43, &[1, 2, 3]);
        info!("write: r = {}", defmt::Debug2Format(&r));

        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        let mut buf = [0u8; 3];
        let r = i2c.read(0x43, &mut buf);
        info!(" read: r = {}, buf = {}", defmt::Debug2Format(&r), unsafe {
            core::str::from_utf8_unchecked(&buf)
        });

        delay.delay_ms(1);
        led_pin.set_low().unwrap();
        let mut buf = [0u8; 8];
        let r = i2c.read(0x43, &mut buf);
        info!(" read: r = {}, buf = {}", defmt::Debug2Format(&r), unsafe {
            core::str::from_utf8_unchecked(&buf)
        });
    }
}

// End of file
