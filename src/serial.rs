use pyo3::prelude::*;
use pyo3::exceptions::PyOSError;
use std::ffi::CString;
use std::collections::HashSet;
use std::sync::{Mutex, OnceLock};
use libc::{
    open, read, write, ioctl,
    tcgetattr, tcsetattr, cfmakeraw, cfsetispeed, cfsetospeed,
    tcdrain, tcflush, termios, c_int, FIONREAD, TCIFLUSH, TCOFLUSH,
    O_RDWR, O_NOCTTY, O_NONBLOCK,
    B9600, B38400, B115200,
    TCSANOW, VMIN, VTIME, EAGAIN, EWOULDBLOCK,
    CS5, CS6, CS7, CS8, PARENB, PARODD, CSTOPB,
};
use crate::gpio::{GPIO, Direction};

static CONFIGURED_PINS: OnceLock<Mutex<HashSet<u8>>> = OnceLock::new();

/// DATA BITS
#[pyclass]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum DataBits {
    Five  = 5,
    Six   = 6,
    Seven = 7,
    Eight = 8,
}

/// PARITY
#[pyclass]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Parity {
    N, // None
    E, // Even
    O, // Odd
}

/// STOP BITS
#[pyclass]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum StopBits {
    One = 1,
    Two = 2,
}

fn config_serial(
    fd: c_int,
    baud: u32,
    timeout: f64,
    data_bits: DataBits,
    parity: Parity,
    stop_bits: StopBits,
) -> Result<(), String> {
    unsafe {
        let mut tio: termios = std::mem::zeroed();
        if tcgetattr(fd, &mut tio) != 0 {
            return Err("tcgetattr failed".into());
        }
        cfmakeraw(&mut tio);

        tio.c_cflag &= !libc::CSIZE;
        tio.c_cflag |= match data_bits {
            DataBits::Five  => CS5,
            DataBits::Six   => CS6,
            DataBits::Seven => CS7,
            DataBits::Eight => CS8,
        };

        match parity {
            Parity::N => tio.c_cflag &= !PARENB,
            Parity::E => { tio.c_cflag |= PARENB; tio.c_cflag &= !PARODD; },
            Parity::O => { tio.c_cflag |= PARENB; tio.c_cflag |= PARODD; },
        }

        if let StopBits::Two = stop_bits {
            tio.c_cflag |= CSTOPB;
        } else {
            tio.c_cflag &= !CSTOPB;
        }

        let speed = match baud {
            9600   => B9600,
            38400  => B38400,
            115200 => B115200,
            _      => return Err("Unsupported baud".into()),
        };
        cfsetispeed(&mut tio, speed);
        cfsetospeed(&mut tio, speed);

        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = (timeout * 10.0) as u8;

        if tcsetattr(fd, TCSANOW, &tio) != 0 {
            return Err("tcsetattr failed".into());
        }
    }
    Ok(())
}

#[pyclass(name = "Serial")]
pub struct Serial485 {
    fd: Mutex<c_int>,
    de_pin: u8,
}

#[pymethods]
impl Serial485 {
    #[new]
    pub fn new(
        path: String,
        baud: u32,
        timeout: f64,
        de_pin: u8,
        data_bits: DataBits,
        parity: Parity,
        stop_bits: StopBits
    ) -> PyResult<Self> {
        let mut set = CONFIGURED_PINS
            .get_or_init(|| Mutex::new(HashSet::new()))
            .lock()
            .unwrap();
        if set.insert(de_pin) {
            GPIO::setup(de_pin, Direction::OUT)?;
        }
        GPIO::output(de_pin, 0)?;

        // Open device
        let cpath = CString::new(path)
            .map_err(|e| PyOSError::new_err(e.to_string()))?;
        let raw_fd = unsafe { open(cpath.as_ptr(), O_RDWR | O_NOCTTY | O_NONBLOCK) };
        if raw_fd < 0 {
            return Err(PyOSError::new_err("Serial open failed"));
        }

        // Params
        config_serial(raw_fd, baud, timeout, data_bits, parity, stop_bits)
            .map_err(PyOSError::new_err)?;

        Ok(Serial485 { fd: Mutex::new(raw_fd), de_pin })
    }

    /// Fully send
    pub fn write(&self, data: &[u8]) -> PyResult<usize> {
        let fd_guard = self.fd.lock().unwrap();
        let fd = *fd_guard;

        GPIO::output(self.de_pin, 1)?;
        let mut sent = 0usize;
        while sent < data.len() {
            let n = unsafe { write(fd, data[sent..].as_ptr() as *const _, data.len() - sent) };
            if n < 0 {
                GPIO::output(self.de_pin, 0)?;
                return Err(PyOSError::new_err("Serial write failed"));
            }
            sent += n as usize;
        }
        unsafe { tcdrain(fd); }
        GPIO::output(self.de_pin, 0)?;
        Ok(sent)
    }

    /// Non-blocking Read (VTIME/VMIN controls Blocking-Verhalten)
    pub fn read(&self, size: usize) -> PyResult<Vec<u8>> {
        let mut buf = vec![0u8; size];
        let fd_guard = self.fd.lock().unwrap();
        let fd = *fd_guard;

        let n = unsafe { read(fd, buf.as_mut_ptr() as *mut _, size) };
        if n < 0 {
            let err = unsafe { *libc::__errno_location() };
            if err == EAGAIN || err == EWOULDBLOCK {
                return Ok(Vec::new());
            }
            return Err(PyOSError::new_err("Serial read failed"));
        }
        buf.truncate(n as usize);
        Ok(buf)
    }

    pub fn in_waiting(&self) -> PyResult<usize> {
        let mut bytes: c_int = 0;
        let fd_guard = self.fd.lock().unwrap();
        let fd = *fd_guard;

        let rc = unsafe { ioctl(fd, FIONREAD, &mut bytes) };
        if rc < 0 {
            return Err(PyOSError::new_err("ioctl(FIONREAD) failed"));
        }
        Ok(bytes as usize)
    }

    pub fn flush(&self) -> PyResult<()> {
        let fd_guard = self.fd.lock().unwrap();
        let fd = *fd_guard;
        unsafe { tcdrain(fd); }
        Ok(())
    }

    pub fn reset_input_buffer(&self) -> PyResult<()> {
        let fd_guard = self.fd.lock().unwrap();
        let fd = *fd_guard;
        unsafe { tcflush(fd, TCIFLUSH); }
        Ok(())
    }

    pub fn reset_output_buffer(&self) -> PyResult<()> {
        let fd_guard = self.fd.lock().unwrap();
        let fd = *fd_guard;
        unsafe { tcflush(fd, TCOFLUSH); }
        Ok(())
    }

    pub fn close(&mut self) -> PyResult<()> {
        let mut fd_guard = self.fd.lock().unwrap();
        if *fd_guard >= 0 {
            unsafe { libc::close(*fd_guard) };
            *fd_guard = -1;
        }
        Ok(())
    }

    fn __del__(&mut self) {
        let _ = self.close();
    }
}

impl Drop for Serial485 {
    fn drop(&mut self) {
        let _ = self.close();
    }
}
