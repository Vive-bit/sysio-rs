use pyo3::prelude::*;
use pyo3::exceptions::PyOSError;
use std::ffi::CString;
use libc::{ open, ioctl, c_int, c_char };
// SpiTr, ioc(), SPI_MAGIC, SPI_WR_* consts...

#[repr(C)]
struct SpiTr {
    tx_buf: u64, rx_buf: u64, len: u32,
    speed_hz: u32, delay_us: u16,
    bits_per_word: u8, cs_change: u8, pad: u32,
}

const SPI_MAGIC: u8 = b'k';
const fn ioc(dir: u32, t: u8, n: u8, s: u32) -> u64 {
    const NR: u32 = 0;
    const TS: u32 = NR + 8;
    const SS: u32 = TS + 8;
    const DS: u32 = SS + 14;
    ((dir << DS) | ((t as u32) << TS) | ((n as u32) << NR) | (s << SS)) as u64
}
const SPI_WR_MODE:  u64 = ioc(1, SPI_MAGIC, 1, std::mem::size_of::<u8>()  as u32);
const SPI_WR_BITS:  u64 = ioc(1, SPI_MAGIC, 3, std::mem::size_of::<u8>()  as u32);
const SPI_WR_SPEED: u64 = ioc(1, SPI_MAGIC, 4, std::mem::size_of::<u32>() as u32);


#[pyclass]
struct MCP3008 { fd: c_int, channel: u8 }

#[pymethods]
impl MCP3008 {
    #[new]
    fn new(bus: u8, cs: u8, channel: u8, speed_khz: u32) -> PyResult<Self> {
        let dev = format!("/dev/spidev{}.{}", bus, cs);
        let cdev = CString::new(dev).unwrap();
        let fd = unsafe { open(cdev.as_ptr(), O_RDWR) };
        if fd < 0 {
            return Err(PyOSError::new_err("SPI open failed"));
        }
        unsafe {
            let m: u8 = 0;
            if libc::ioctl(fd, crate::SPI_WR_MODE as _, &m) < 0 {
                return Err(PyOSError::new_err("SPI mode set failed"));
            }
            let b: u8 = 8;
            if libc::ioctl(fd, crate::SPI_WR_BITS as _, &b) < 0 {
                return Err(PyOSError::new_err("SPI bits set failed"));
            }
            let s = speed_khz * 1_000;
            if libc::ioctl(fd, crate::SPI_WR_SPEED as _, &s) < 0 {
                return Err(PyOSError::new_err("SPI speed set failed"));
            }
        }
        Ok(MCP3008 { fd, channel })
    }

    fn read_raw(&self) -> PyResult<u16> {
        let tx = [1, ((8 | self.channel) << 4) as u8, 0];
        let mut rx = [0u8; 3];
        let mut tr = crate::SpiTr {
            tx_buf: tx.as_ptr() as u64,
            rx_buf: rx.as_mut_ptr() as u64,
            len:    3,
            speed_hz: 0,
            delay_us: 0,
            bits_per_word: 8,
            cs_change: 0,
            pad: 0,
        };
        let ret = unsafe {
            libc::ioctl(
                self.fd,
                crate::ioc(1, crate::SPI_MAGIC, 0, std::mem::size_of::<crate::SpiTr>() as u32) as _,
                &mut tr,
            )
        };
        if ret < 1 {
            return Err(PyOSError::new_err("SPI transfer failed"));
        }
        Ok((((rx[1] & 3) as u16) << 8) | rx[2] as u16)
    }

    #[getter]
    fn raw_value(&self) -> PyResult<u16> {
        self.read_raw()
    }

    #[getter]
    fn value(&self) -> PyResult<f64> {
        Ok(self.read_raw()? as f64 / 1023.0)
    }

    #[getter]
    fn bits(&self) -> PyResult<u8> {
        Ok(10)
    }

    fn read(&self) -> PyResult<(u16, f64)> {
        let r = self.read_raw()?;
        Ok((r, r as f64 / 1023.0))
    }

    fn value(&self) -> PyResult<f64> {
        // old!
        self.value()
    }
}