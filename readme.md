# I2C controller/peripheral test

## Requirement

- two rp2040
- two resistors in the range 2kOhm-3kOhm
- Some wires
- two SWD probes

## Setup

1. Setup your probes according to their documentation.
1. Connect two rp2040 using `gpio0` for `sda` and `gpio1` for `scl`.
1. Add a pull-up on each line (tested with 2.2kOhm).

## How to run

Identify the VID, PID and Serial for each probe with `probe-rs list`

1. Open two terminal
1. Then:
    - Terminal 1: run `cargo run --release -p i2c-controller -- --probe <VID>:<PID>:<SERIAL> --log-format '{L}{s}'`
    - Terminal 2: run `cargo run --release -p i2c-peripheral -- --probe <VID>:<PID>:<SERIAL> --log-format '{L}{s}'`

## Features

The i2c-peripheral crate has a `slow` feature increasing the event loop's latency from 1μs to 1ms.  
Enable with `cargo run --release -p i2c-peripheral --features slow -- --probe <VID>:<PID>:<SERIAL> --log-format '{L}{s}'`
