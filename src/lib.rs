mod gpio;
mod spi;
mod serial;
mod time;

use pyo3::prelude::*;

#[pymodule]
fn sysio(_py: Python, m: &PyModule) -> PyResult<()> {
    // GPIO
    m.add_function(wrap_pyfunction!(gpio::setmode, m)?)?;
    m.add_function(wrap_pyfunction!(gpio::setup, m)?)?;
    m.add_function(wrap_pyfunction!(gpio::output, m)?)?;
    m.add_function(wrap_pyfunction!(gpio::input, m)?)?;

    // SPI MCP3008
    m.add_class::<spi::MCP3008>()?;

    // Serial RS-485
    m.add_class::<serial::Serial485>()?;

    m.add_class::<serial::DataBits>()?;
    m.add_class::<serial::Parity>()?;
    m.add_class::<serial::StopBits>()?;
    m.add_class::<serial::Serial485>()?;
    
    // Timing helpers
    m.add_function(wrap_pyfunction!(time::sleep_s, m)?)?;
    m.add_function(wrap_pyfunction!(time::sleep_ms, m)?)?;
    m.add_function(wrap_pyfunction!(time::sleep_us, m)?)?;
    m.add_function(wrap_pyfunction!(time::time_time, m)?)?;
    Ok(())
}
