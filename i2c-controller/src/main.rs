//! I2C controller example
//!
//! This example implements a simple application using i2c as a bus controller.
//! It prints on its rtt trace buffer (via defmt).
//!
//! It runs an infinite loop doing:
//! - wait 500ms
//! - toggle the LED
//! - write `[1, 2, 3u8]` to the peripheral at address `0x43`.
//! - wait 500ms
//! - toggle the LED
//! - read 3 bytes from the peripheral at address `0x43` and prints them as a string
//! - wait 500ms
//! - toggle the LED
//! - read 8 bytes from the peripheral at address `0x43` and prints them as a string
//!
#![no_std]
#![no_main]

use panic_probe as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use defmt::*;
    use defmt_rtt as _;
    use embedded_hal::digital::v2::ToggleableOutputPin;
    use embedded_hal::prelude::*;
    use fugit::RateExtU32;

    use rp_pico as bsp;

    use bsp::hal::{
        clocks::init_clocks_and_plls,
        gpio::{
            bank0::{Gpio0, Gpio1, Gpio25},
            FunctionI2C, FunctionSioOutput, Pin, PullDown, PullNone,
        },
        i2c::I2C,
        pac,
        sio::Sio,
        watchdog::Watchdog,
    };

    type SDA = Pin<Gpio0, FunctionI2C, PullNone>;
    type SCL = Pin<Gpio1, FunctionI2C, PullNone>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        delay: bsp::hal::Timer,
        i2c: I2C<pac::I2C0, (SDA, SCL)>,
        led: Pin<Gpio25, FunctionSioOutput, PullDown>,
    }

    #[init]
    fn init(mut c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            bsp::hal::sio::spinlock_reset();
        }

        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let sio = Sio::new(c.device.SIO);

        // External high-speed crystal on the pico board is 12Mhz
        let external_xtal_freq_hz = 12_000_000u32;
        let clocks = init_clocks_and_plls(
            external_xtal_freq_hz,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut c.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let delay = bsp::hal::Timer::new(c.device.TIMER, &mut c.device.RESETS, &clocks);

        let pins = bsp::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut c.device.RESETS,
        );

        let led = pins.led.into_push_pull_output();

        // Configure two pins as being I²C, not GPIO
        let sda_pin: Pin<_, FunctionI2C, PullNone> = pins.gpio0.reconfigure();
        let scl_pin: Pin<_, FunctionI2C, PullNone> = pins.gpio1.reconfigure();

        // Create the I²C driver, using the two pre-configured pins. This will fail
        // at compile time if the pins are in the wrong mode, or if this I²C
        // peripheral isn't available on these pins!
        let i2c = bsp::hal::I2C::i2c0(
            c.device.I2C0,
            sda_pin,
            scl_pin,
            400.kHz(),
            &mut c.device.RESETS,
            &clocks.peripheral_clock,
        );

        (Shared {}, Local { delay, i2c, led }, init::Monotonics())
    }

    #[task(local = [delay, i2c, led])]
    async fn idle(cx: idle::Context) -> ! {
        loop {
            cx.local.delay.delay_ms(500);
            cx.local.led.toggle().unwrap();
            let r = cx.local.i2c.write(0x43, &[1, 2, 3]);
            info!("write: r = {}", defmt::Debug2Format(&r));

            cx.local.delay.delay_ms(500);
            cx.local.led.toggle().unwrap();
            let mut buf = [0u8; 3];
            let r = cx.local.i2c.read(0x43, &mut buf);
            info!(" read: r = {}, buf = {}", defmt::Debug2Format(&r), unsafe {
                core::str::from_utf8_unchecked(&buf)
            });

            cx.local.delay.delay_ms(500);
            cx.local.led.toggle().unwrap();
            let mut buf = [0u8; 8];
            let r = cx.local.i2c.read(0x43, &mut buf);
            info!(" read: r = {}, buf = {}", defmt::Debug2Format(&r), unsafe {
                core::str::from_utf8_unchecked(&buf)
            });
        }
    }
}
