mod gpio;
mod spi;
mod serial;

use pyo3::prelude::*;

#[pymodule]
fn sysio(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(gpio::setmode, m)?)?;
    m.add_function(wrap_pyfunction!(gpio::setup, m)?)?;
    m.add_function(wrap_pyfunction!(gpio::output, m)?)?;
    m.add_function(wrap_pyfunction!(gpio::input, m)?)?;
    m.add_class::<spi::MCP3008>()?;
    m.add_class::<serial::Serial485>()?;
    Ok(())
}
