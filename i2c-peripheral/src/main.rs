//! I2C Peripheral example
//!
//! This example implements a simple i2c-event handler.
//! It prints on its rtt trace buffer (via defmt).
//!
//! - On read request it sends back as many `Hey` sequences as necessary.
//! - On write requests it prints the received bytes.
//!
//! Note: The peripheral cannot `NAK` read requests so it has to send bytes as long as the
//! controller requests for more. Whether it repeats the data or sends a predefined pattern is
//! application defined.
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::ToggleableOutputPin;
use panic_probe as _;

#[cfg(feature = "slow")]
use embedded_hal::blocking::delay::DelayMs;
#[cfg(not(feature = "slow"))]
use embedded_hal::blocking::delay::DelayUs;

use rp_pico as bsp;

use bsp::{
    entry,
    hal::{
        clocks::init_clocks_and_plls,
        gpio::{FunctionI2C, Pin, PullNone},
        i2c::peripheral::I2CEvent,
        pac,
        sio::Sio,
        watchdog::Watchdog,
    },
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
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

    let mut delay = bsp::hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    // Configure two pins as being I²C, not GPIO
    let sda_pin: Pin<_, FunctionI2C, PullNone> = pins.gpio0.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, PullNone> = pins.gpio1.reconfigure();

    // Create the I²C driver, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let mut i2c = bsp::hal::I2C::new_peripheral_event_iterator(
        pac.I2C0,
        sda_pin,
        scl_pin,
        &mut pac.RESETS,
        0x43,
    );

    let mut log_throttle = 0;
    loop {
        if let Some(evt) = i2c.next() {
            match evt {
                I2CEvent::Start => info!(" start"),
                I2CEvent::Restart => info!("restart"),
                I2CEvent::Stop => {
                    info!("  stop");
                    info!("------")
                }
                I2CEvent::TransferRead => {
                    let n = i2c.write(b"Hey");
                    info!("  sent: {} byte(s)", n);
                }
                I2CEvent::TransferWrite => {
                    let mut buf = [0; 64];
                    let n = i2c.read(&mut buf);
                    info!(" recvd: {}", buf[..n])
                }
            }
            log_throttle = 0;
            led_pin.toggle().unwrap();
        } else {
            log_throttle += 1;
            if log_throttle == 500_000 {
                info!("None");
                log_throttle = 0;
            }
            led_pin.toggle().unwrap();
            #[cfg(not(feature = "slow"))]
            delay.delay_us(1);
            #[cfg(feature = "slow")]
            delay.delay_ms(1);
        }
    }
}
