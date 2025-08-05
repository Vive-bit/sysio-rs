use pyo3::prelude::*;
use pyo3::exceptions::PyOSError;
use std::ffi::CString;
use std::sync::Mutex;
use libc::{ open, read, write, ioctl, tcgetattr, tcsetattr, cfmakeraw, cfsetispeed,
            cfsetospeed, tcdrain, tcflush, termios, c_int, FIONREAD, TCIFLUSH,
            TCOFLUSH, O_RDWR, O_NOCTTY, O_NONBLOCK, B9600, B38400, B115200,
            TCSANOW, VMIN, VTIME, EAGAIN, EWOULDBLOCK };
use crate::gpio::{setup, output};

fn config_serial(fd: c_int, baud: u32) -> Result<(), String> {
    unsafe {
        let mut tio: termios = std::mem::zeroed();
        if tcgetattr(fd, &mut tio) != 0 {
            return Err("tcgetattr failed".into());
        }
        cfmakeraw(&mut tio);
        let speed = match baud {
            9600   => B9600,
            38400  => B38400,
            115200 => B115200,
            _      => return Err("Unsupported baud".into()),
        };
        cfsetispeed(&mut tio, speed);
        cfsetospeed(&mut tio, speed);
        if tcsetattr(fd, TCSANOW, &tio) != 0 {
            return Err("tcsetattr failed".into());
        }
    }
    Ok(())
}

#[pyclass]
pub struct Serial485 {
    fd: Mutex<c_int>,
    de_pin: u8,
}

#[pymethods]
impl Serial485 {
    #[new]
    pub fn new(path: String, baud: u32, timeout: f64, de_pin: u8) -> PyResult<Self> {
        setup(de_pin, "OUT")?;
        output(de_pin, 0)?;
        let cpath = CString::new(path).unwrap();
        let raw_fd = unsafe { open(cpath.as_ptr(), O_RDWR | O_NOCTTY | O_NONBLOCK) };
        if raw_fd < 0 {
            return Err(PyOSError::new_err("Serial open failed"));
        }
        config_serial(raw_fd, baud).map_err(PyOSError::new_err)?;
        unsafe {
            let mut tio: termios = std::mem::zeroed();
            tcgetattr(raw_fd, &mut tio);
            tio.c_cc[VMIN]  = 0;
            tio.c_cc[VTIME] = (timeout * 10.0) as u8;
            tcsetattr(raw_fd, TCSANOW, &tio);
        }
        Ok(Serial485 { fd: Mutex::new(raw_fd), de_pin })
    }

    pub fn write(&self, data: &[u8]) -> PyResult<usize> {
        output(self.de_pin, 1)?;
        let fd = *self.fd.lock().unwrap();
        let n = unsafe { write(fd, data.as_ptr() as *const _, data.len()) };
        if n < 0 {
            return Err(PyOSError::new_err("Serial write failed"));
        }
        unsafe { tcdrain(fd); }
        output(self.de_pin, 0)?;
        Ok(n as usize)
    }

    pub fn read(&self, size: usize) -> PyResult<Vec<u8>> {
        let mut buf = vec![0u8; size];
        let fd = *self.fd.lock().unwrap();
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
        let fd = *self.fd.lock().unwrap();
        unsafe { ioctl(fd, FIONREAD, &mut bytes) };
        Ok(bytes as usize)
    }

    pub fn flush(&self) -> PyResult<()> {
        let fd = *self.fd.lock().unwrap();
        unsafe { tcdrain(fd); }
        Ok(())
    }

    pub fn reset_input_buffer(&self) -> PyResult<()> {
        let fd = *self.fd.lock().unwrap();
        unsafe { tcflush(fd, TCIFLUSH); }
        Ok(())
    }

    pub fn reset_output_buffer(&self) -> PyResult<()> {
        let fd = *self.fd.lock().unwrap();
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
        let fd = self.fd.get_mut().unwrap();
        let _ = unsafe { libc::close(*fd) };
    }
}

impl Drop for Serial485 {
    fn drop(&mut self) {
        if let Ok(fd) = self.fd.get_mut() {
            unsafe { libc::close(*fd); }
        }
    }
}
