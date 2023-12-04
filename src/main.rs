#![no_std]
#![no_main]

use panic_halt as _;

#[rp_pico::entry]
fn main() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}
