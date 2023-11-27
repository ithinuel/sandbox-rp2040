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
#![feature(type_alias_impl_trait)]

use panic_probe as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use defmt::*;
    use defmt_rtt as _;
    use embedded_hal::digital::v2::ToggleableOutputPin;
    use rtic_monotonics::systick::*;

    use rp_pico as bsp;

    use bsp::hal::{
        clocks::init_clocks_and_plls,
        gpio::{
            bank0::{Gpio0, Gpio1, Gpio25},
            FunctionI2C, FunctionSioOutput, Pin, PullDown, PullNone,
        },
        i2c::peripheral::{I2CEvent, I2CPeripheralEventIterator},
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
        i2c: I2CPeripheralEventIterator<pac::I2C0, (SDA, SCL)>,
        led: Pin<Gpio25, FunctionSioOutput, PullDown>,
    }

    #[init]
    fn init(mut c: init::Context) -> (Shared, Local) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            bsp::hal::sio::spinlock_reset();
        }

        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let sio = Sio::new(c.device.SIO);

        // External high-speed crystal on the pico board is 12Mhz
        let external_xtal_freq_hz = 12_000_000u32;
        let _clocks = init_clocks_and_plls(
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

        let systic_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(c.core.SYST, 125_000_000, systic_mono_token);

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
        let i2c = bsp::hal::I2C::new_peripheral_event_iterator(
            c.device.I2C0,
            sda_pin,
            scl_pin,
            &mut c.device.RESETS,
            0x43,
        );

        let _ = work::spawn();
        (Shared {}, Local { i2c, led })
    }

    #[task(local = [i2c, led])]
    async fn work(cx: work::Context) {
        let mut log_throttle = 0;
        loop {
            if let Some(evt) = cx.local.i2c.next() {
                match evt {
                    I2CEvent::Start => info!(" start"),
                    I2CEvent::Restart => info!("restart"),
                    I2CEvent::Stop => {
                        info!("  stop");
                        info!("------")
                    }
                    I2CEvent::TransferRead => {
                        let n = cx.local.i2c.write(b"Hey");
                        info!("  sent: {} byte(s)", n);
                    }
                    I2CEvent::TransferWrite => {
                        let mut buf = [0; 64];
                        let n = cx.local.i2c.read(&mut buf);
                        info!(" recvd: {}", buf[..n])
                    }
                }
                log_throttle = 0;
                cx.local.led.toggle().unwrap();
            } else {
                log_throttle += 1;
                if log_throttle == 500_000 {
                    info!("None");
                    log_throttle = 0;
                }
                cx.local.led.toggle().unwrap();
                #[cfg(not(feature = "slow"))]
                Systick::delay(1.micros()).await;
                #[cfg(feature = "slow")]
                Systick::delay(1.millis()).await;
            }
        }
    }
}
