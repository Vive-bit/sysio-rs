mod gpio;
mod spi;
mod serial;
mod time;

use pyo3::prelude::{PyModule, PyResult, Python};
use pyo3::pymodule;
use pyo3::wrap_pyfunction;


#[pymodule]
fn sysio(_py: Python<'_>, m: &PyModule) -> PyResult<()> {
    // GPIO
    m.add_class::<gpio::Mode>()?;
    m.add_class::<gpio::GPIO>()?;

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
