#![no_std]
pub fn demo(i: impl IntoIterator<Item = u16>) {
    // the app uses itertool without std at run time
    use itertools::Itertools;
    i.into_iter()
        .map(|v| {
            let mut s = arrayvec::ArrayString::<10>::new();
            use core::fmt::Write;
            let _ = write!(&mut s, "{:x}", v);
            s
        })
        .interleave(Some(arrayvec::ArrayString::from("instr").unwrap()))
        .for_each(|msg| {
            defmt::info!("{}", *msg);
        });
}
