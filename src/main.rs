#![no_std]
#![no_main]

use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;
use tm1637::TM1637;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = rp2040_hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let hal_sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        hal_sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut clock_pin = pins.gpio6.into_push_pull_output();
    let mut data_pin = pins.gpio7.into_push_pull_output();

    let mut tm = TM1637::new(&mut clock_pin, &mut data_pin, &mut delay);

    tm.init().expect("1");
    tm.set_brightness(255).expect("2");
    tm.clear().expect("3");
    loop {
        tm.print_raw(0, &[0x77]).expect("4");
    }
}
