#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal::multicore::{Multicore, Stack};
use defmt_rtt as _;
use panic_probe as _;

use bsp::hal;
use bsp::hal::pac;
use bsp::XOSC_CRYSTAL_FREQ as XTAL_FREQ_HZ;
use rp_pico as bsp;

static mut CORE1_STACK: Stack<4096> = Stack::new();

#[entry]
fn main() -> ! {
    // the entry attributes turns `static mut`s at the start of the function to be `&'static mut T`
    // arguments passed to the function so that it is safe to mutate although being static;
    // At the start, we don't have a timer to put in so keep it as None.
    #[allow(non_upper_case_globals)]
    static mut timer: Option<hal::Timer> = None;

    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Initialize the timer and store it in the static storage we have for it.
    *timer = Some(hal::Timer::new(pac.TIMER, &mut pac.RESETS));
    // We take a &'static ref to it. We can safely unwrap because we know we just set it.
    let timer_ref = timer.as_ref().unwrap();

    let mut sio = hal::Sio::new(pac.SIO);
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _ = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || loop {
        let _ = timer_ref.get_counter().ticks();
    });
    loop {
        let _ = timer_ref.get_counter().ticks();
    }
}

// End of file
