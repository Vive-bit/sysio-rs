use pyo3::prelude::*;
use pyo3::exceptions::PyOSError;
use std::ptr;
use libc::{ open, mmap, close, PROT_READ, PROT_WRITE, MAP_SHARED, O_RDWR, c_char };
use std::sync::{OnceLock, Mutex};

const GPIO_LEN: usize = 0xB4;
static GPIO_MEM: OnceLock<Mutex<*mut u32>> = OnceLock::new();

fn init_gpio() -> PyResult<*mut u32> {
    let lock = GPIO_MEM.get_or_init(|| Mutex::new(ptr::null_mut()));
    let mut guard = lock.lock().unwrap();
    if guard.is_null() {
        let fd = unsafe { open(b"/dev/gpiomem\0".as_ptr() as *const c_char, O_RDWR) };
        if fd < 0 {
            return Err(PyOSError::new_err("Failed to open /dev/gpiomem"));
        }
        let map = unsafe { mmap(ptr::null_mut(), GPIO_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0) };
        unsafe { close(fd) };
        if map == libc::MAP_FAILED {
            return Err(PyOSError::new_err("GPIO mmap failed"));
        }
        *guard = map as *mut u32;
    }
    Ok(*guard)
}

fn gpio_reg(base: *mut u32, idx: usize) -> *mut u32 {
    unsafe { base.add(idx) }
}

#[pyfunction]
pub fn setmode(_mode: &str) -> PyResult<()> {
    init_gpio()?;
    Ok(())
}

#[pyfunction]
pub fn setup(pin: u8, direction: &str) -> PyResult<()> {
    let base = init_gpio()?;
    let fsel = (pin as usize) / 10;
    let shift = ((pin % 10) * 3) as usize;
    unsafe {
        let reg = gpio_reg(base, fsel);
        let val = ptr::read_volatile(reg);
        let mask = !(0b111 << shift);
        let bits = if direction == "OUT" { 0b001 } else { 0b000 };
        ptr::write_volatile(reg, (val & mask) | (bits << shift));
    }
    Ok(())
}

#[pyfunction]
pub fn output(pin: u8, value: u8) -> PyResult<()> {
    let base = init_gpio()?;
    let idx = if value == 0 { 10 + (pin as usize) / 32 } else { 7 + (pin as usize) / 32 };
    let shift = (pin % 32) as usize;
    unsafe {
        let reg = gpio_reg(base, idx);
        ptr::write_volatile(reg, 1 << shift);
    }
    Ok(())
}

#[pyfunction]
pub fn input(pin: u8) -> PyResult<u8> {
    let base = init_gpio()?;
    let idx = 13 + (pin as usize) / 32;
    let shift = (pin % 32) as usize;
    let val = unsafe { ptr::read_volatile(gpio_reg(base, idx)) };
    Ok(((val >> shift) & 1) as u8)
}