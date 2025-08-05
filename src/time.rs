use pyo3::prelude::*;
use std::thread::sleep;
use std::time::Duration;
use std::time::{SystemTime, UNIX_EPOCH};
use pyo3::exceptions::PyOSError;

#[pyfunction]
/// Sleeps for given seconds
pub fn sleep_s(s: f64) {
    sleep(Duration::from_secs_f64(s));
}

#[pyfunction]
/// Sleeps for given milliseconds
pub fn sleep_ms(ms: f64) {
    sleep(Duration::from_secs_f64(ms / 1_000.0));
}

#[pyfunction]
/// Sleeps for given microseconds
pub fn sleep_us(us: f64) {
    sleep(Duration::from_secs_f64(us / 1_000_000.0));
}

#[pyfunction]
/// Return current time as float seconds since UNIX epoch
pub fn time_time() -> PyResult<f64> {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|e| PyOSError::new_err(format!("SystemTime error: {}", e)))?;
    Ok(now.as_secs_f64())
}