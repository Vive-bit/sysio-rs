use pyo3::prelude::*;
use pyo3::exceptions::PyOSError;
use pyo3::types::PyType;

use std::ptr;
use std::sync::{Mutex, OnceLock};
use std::collections::HashMap;

use libc::{
    open, mmap, close, PROT_READ, PROT_WRITE, MAP_SHARED, O_RDWR, c_char, MAP_FAILED,
};

#[pyclass]
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Mode {
    BCM,
    BOARD,
}

#[pyclass]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Direction {
    IN,
    OUT,
}

static PIN_MODE: OnceLock<Mutex<Mode>> = OnceLock::new();
static GPIO_MEM: OnceLock<Mutex<usize>> = OnceLock::new();
static PIN_DIRECTIONS: OnceLock<Mutex<HashMap<u8, Direction>>> = OnceLock::new();

const GPIO_LEN: usize = 0xB4;

// 1..40 (0 unused). -1 = kein GPIO
const BOARD_TO_BCM: [i8; 41] = [
    -1, -1, -1,  2, -1,  3, -1,  4, 14, -1,
    15, 17, 18, 27, -1, 22, 23, -1, 24, 10,
    -1,  9, 25, 11,  8, -1,  7, -1, -1,  5,
    -1,  6, 12, 13, -1, 19, 16, 26, 20, -1,
     21,
];

fn init_gpio() -> PyResult<*mut u32> {
    let lock = GPIO_MEM.get_or_init(|| Mutex::new(0));
    let mut guard = lock.lock().unwrap();
    if *guard == 0 {
        let fd = unsafe {
            open(b"/dev/gpiomem\0".as_ptr() as *const c_char, O_RDWR)
        };
        if fd < 0 {
            return Err(PyOSError::new_err("Failed to open /dev/gpiomem"));
        }
        let map = unsafe {
            mmap(
                ptr::null_mut(),
                GPIO_LEN,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                fd,
                0,
            )
        };
        unsafe { close(fd) };
        if map == MAP_FAILED {
            return Err(PyOSError::new_err("GPIO mmap failed"));
        }
        *guard = map as usize;
    }
    Ok(*guard as *mut u32)
}

fn get_mode() -> Mode {
    *PIN_MODE
        .get_or_init(|| Mutex::new(Mode::BCM))
        .lock()
        .unwrap()
}

fn map_pin(pin: u8) -> PyResult<u8> {
    match get_mode() {
        Mode::BCM => Ok(pin),
        Mode::BOARD => {
            let idx = pin as usize;
            if idx == 0 || idx > 40 {
                return Err(PyOSError::new_err("Board pin out of range 1–40"));
            }
            let bcm = BOARD_TO_BCM[idx];
            if bcm < 0 {
                return Err(PyOSError::new_err("Board pin is not a GPIO"));
            }
            Ok(bcm as u8)
        }
    }
}

#[inline]
fn gpio_reg(base: *mut u32, idx: usize) -> *mut u32 {
    unsafe { base.add(idx) }
}

fn is_direction_set(base: *mut u32, bcm: u8, want: Direction) -> bool {
    let fsel = (bcm as usize) / 10;
    let shift = ((bcm % 10) * 3) as usize;
    unsafe {
        let reg = gpio_reg(base, fsel);
        let val = ptr::read_volatile(reg);
        let bits = (val >> shift) & 0b111;
        match want {
            Direction::IN  => bits == 0b000,
            Direction::OUT => bits == 0b001,
        }
    }
}

#[pyclass]
pub struct GPIO;

#[pymethods]
impl GPIO {
    #[classattr]
    #[allow(non_snake_case)]
    fn Mode(py: Python<'_>) -> &PyType { py.get_type::<Mode>() }

    #[classattr]
    #[allow(non_snake_case)]
    fn Direction(py: Python<'_>) -> &PyType { py.get_type::<Direction>() }

    #[staticmethod]
    pub fn setmode(mode: Mode) -> PyResult<()> {
        let lock = PIN_MODE.get_or_init(|| Mutex::new(Mode::BCM));
        *lock.lock().unwrap() = mode;
        init_gpio()?; 
        Ok(())
    }

    /// Idempotent + Conflict check
    #[staticmethod]
    pub fn setup(pin: u8, direction: Direction) -> PyResult<()> {
        let bcm = map_pin(pin)?;
        let base = init_gpio()?;

        let cache_lock = PIN_DIRECTIONS.get_or_init(|| Mutex::new(HashMap::new()));
        {
            let mut cache = cache_lock.lock().unwrap();
            if let Some(prev) = cache.get(&bcm) {
                if *prev == direction {
                    if is_direction_set(base, bcm, direction) {
                        return Ok(());
                    }
                } else {
                    return Err(PyOSError::new_err(format!(
                        "GPIO {} bereits als {:?} konfiguriert", bcm, prev
                    )));
                }
            } else if is_direction_set(base, bcm, direction) {
                cache.insert(bcm, direction);
                return Ok(());
            }
        }

        let fsel = (bcm as usize) / 10;
        let shift = ((bcm % 10) * 3) as usize;
        unsafe {
            let reg = gpio_reg(base, fsel);
            let val = ptr::read_volatile(reg);
            let mask = !(0b111 << shift);
            let bits = match direction { Direction::OUT => 0b001, Direction::IN => 0b000 };
            ptr::write_volatile(reg, (val & mask) | (bits << shift));
        }

        PIN_DIRECTIONS
            .get_or_init(|| Mutex::new(HashMap::new()))
            .lock().unwrap()
            .insert(bcm, direction);

        Ok(())
    }

    #[staticmethod]
    pub fn is_setup(pin: u8, direction: Direction) -> PyResult<bool> {
        let bcm = map_pin(pin)?;
        let base = init_gpio()?;
        let cache = PIN_DIRECTIONS
            .get_or_init(|| Mutex::new(HashMap::new()))
            .lock().unwrap();
        let cached = cache.get(&bcm).copied();
        let reg_ok = is_direction_set(base, bcm, direction);
        Ok(match (cached, reg_ok) {
            (Some(d), true) if d == direction => true,
            _ => reg_ok,
        })
    }

    #[staticmethod]
    pub fn output(pin: u8, value: u8) -> PyResult<()> {
        let bcm = map_pin(pin)?;
        let base = init_gpio()?;
        let idx = if value == 0 {
            10 + (bcm as usize) / 32 // GPCLR
        } else {
            7 + (bcm as usize) / 32  // GPSET
        };
        let shift = (bcm % 32) as usize;
        unsafe {
            let reg = gpio_reg(base, idx);
            ptr::write_volatile(reg, 1 << shift);
        }
        Ok(())
    }

    #[staticmethod]
    pub fn input(pin: u8) -> PyResult<u8> {
        let bcm = map_pin(pin)?;
        let base = init_gpio()?;
        let idx = 13 + (bcm as usize) / 32; // GPLEV
        let shift = (bcm % 32) as usize;
        let val = unsafe { ptr::read_volatile(gpio_reg(base, idx)) };
        Ok(((val >> shift) & 1) as u8)
    }
}
