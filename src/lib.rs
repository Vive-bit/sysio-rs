use pyo3::prelude::*;
use pyo3::exceptions::PyOSError;
use std::fs::File;
use std::os::unix::io::{AsRawFd, RawFd};
use std::ptr;
use libc::{open, mmap, munmap, close, O_RDWR, PROT_READ, PROT_WRITE, MAP_SHARED};

const GPIO_BASE_OFFSET: usize = 0x200000; // for Raspberry Pi 1; adjust for Pi2/3 (0x3F000000) or Pi4 (0xFE000000)
const GPIO_LEN: usize = 0xB4;

static mut GPIO_MEM: *mut u32 = ptr::null_mut();

fn init_gpio() -> Result<(), PyOSError> {
    unsafe {
        if !GPIO_MEM.is_null() {
            return Ok(());
        }
        // open /dev/gpiomem
        let fd = open("/dev/gpiomem\0".as_ptr() as *const i8, O_RDWR);
        if fd < 0 {
            return Err(PyOSError::new_err("Failed to open /dev/gpiomem"));
        }
        // mmap GPIO registers
        let map = mmap(
            ptr::null_mut(),
            GPIO_LEN,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            fd,
            0,
        );
        if map == libc::MAP_FAILED {
            close(fd);
            return Err(PyOSError::new_err("GPIO mmap failed"));
        }
        GPIO_MEM = map as *mut u32;
        close(fd);
        Ok(())
    }
}

#[pyfunction]
fn setmode(_mode: &str) -> PyResult<()> {
    init_gpio()?;
    Ok(())
}

fn gpio_register(index: usize) -> *mut u32 {
    unsafe { GPIO_MEM.add(index) }
}

#[pyfunction]
fn setup(pin: u8, direction: &str) -> PyResult<()> {
    init_gpio()?;
    let fsel = (pin as usize) / 10;
    let shift = ((pin % 10) * 3) as usize;
    unsafe {
        let reg = gpio_register(fsel);
        let val = ptr::read_volatile(reg);
        let mask = !(0b111 << shift);
        let new = (val & mask) |
                  ((if direction == "OUT" { 0b001 } else { 0b000 }) << shift);
        ptr::write_volatile(reg, new);
    }
    Ok(())
}

#[pyfunction]
fn output(pin: u8, value: u8) -> PyResult<()> {
    init_gpio()?;
    let reg_index = if value == 0 { 10 + ((pin as usize) / 32) } else { 7 + ((pin as usize) / 32) };
    let shift = (pin % 32) as usize;
    unsafe {
        let reg = gpio_register(reg_index);
        ptr::write_volatile(reg, 1 << shift);
    }
    Ok(())
}

#[pyfunction]
fn input(pin: u8) -> PyResult<u8> {
    init_gpio()?;
    let reg = gpio_register(13 + ((pin as usize) / 32));
    let shift = (pin % 32) as usize;
    unsafe {
        let val = ptr::read_volatile(reg);
        Ok(((val >> shift) & 1) as u8)
    }
}

#[pymodule]
fn sysio(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(setmode, m)?)?;
    m.add_function(wrap_pyfunction!(setup, m)?)?;
    m.add_function(wrap_pyfunction!(output, m)?)?;
    m.add_function(wrap_pyfunction!(input, m)?)?;
    Ok(())
}
