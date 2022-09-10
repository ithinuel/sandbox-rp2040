#![no_std]

pub fn get_program() -> pio::Program<32> {
    // pio_proc uses itertools with std on host at build time
    pio_proc::pio_asm!(
        ".side_set 1"

        ".wrap_target"
        "  out pins 1   side 0 [1]"
        "  nop          side 1 [1]"
        ".wrap"
    )
    .program
}
