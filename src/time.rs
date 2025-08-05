use pyo3::prelude::*;
use std::thread::sleep;
use std::time::Duration;

#[pyfunction]
pub fn sleep_ms(ms: u64) {
    sleep(Duration::from_millis(ms));
}

#[pyfunction]
pub fn sleep_us(us: u64) {
    sleep(Duration::from_micros(us));
}