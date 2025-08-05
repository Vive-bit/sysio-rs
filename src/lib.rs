use pyo3::prelude::*;
use pyo3::exceptions::PyOSError;
use std::{ptr, ffi::CString};
use std::os::unix::io::AsRawFd;
use std::fs::OpenOptions;
use libc::{
    open, mmap, close, read, write, ioctl,
    O_RDWR, O_NOCTTY, O_NONBLOCK,
    PROT_READ, PROT_WRITE, MAP_SHARED, c_char, c_int,
    termios, tcgetattr, tcsetattr, cfmakeraw,
    cfsetispeed, cfsetospeed, tcdrain, tcflush,
    B9600, B38400, B115200, TCSANOW, VMIN, VTIME, FIONREAD,
    TCIFLUSH, TCOFLUSH, EAGAIN, EWOULDBLOCK
};

const GPIO_LEN: usize = 0xB4;
static mut GPIO_MEM: *mut u32 = ptr::null_mut();

fn init_gpio() -> PyResult<()> {
    unsafe {
        if !GPIO_MEM.is_null() {
            return Ok(());
        }
        let fd = open(b"/dev/gpiomem\0".as_ptr() as *const c_char, O_RDWR);
        if fd < 0 {
            return Err(PyOSError::new_err("Failed to open /dev/gpiomem"));
        }
        let map = mmap(
            ptr::null_mut(),
            GPIO_LEN,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            fd,
            0,
        );
        close(fd);
        if map == libc::MAP_FAILED {
            return Err(PyOSError::new_err("GPIO mmap failed"));
        }
        GPIO_MEM = map as *mut u32;
        Ok(())
    }
}

fn gpio_reg(idx: usize) -> *mut u32 {
    unsafe { GPIO_MEM.add(idx) }
}

#[pyfunction]
fn setmode(_mode: &str) -> PyResult<()> {
    init_gpio()?;
    Ok(())
}

#[pyfunction]
fn setup(pin: u8, direction: &str) -> PyResult<()> {
    init_gpio()?;
    let fsel = (pin as usize) / 10;
    let shift = ((pin % 10) * 3) as usize;
    unsafe {
        let reg = gpio_reg(fsel);
        let val = ptr::read_volatile(reg);
        let mask = !(0b111 << shift);
        let bits = if direction == "OUT" { 0b001 } else { 0b000 };
        ptr::write_volatile(reg, (val & mask) | (bits << shift));
    }
    Ok(())
}

#[pyfunction]
fn output(pin: u8, value: u8) -> PyResult<()> {
    init_gpio()?;
    let idx = if value == 0 {
        10 + (pin as usize) / 32
    } else {
        7 + (pin as usize) / 32
    };
    let shift = (pin % 32) as usize;
    unsafe {
        let reg = gpio_reg(idx);
        ptr::write_volatile(reg, 1 << shift);
    }
    Ok(())
}

#[pyfunction]
fn input(pin: u8) -> PyResult<u8> {
    init_gpio()?;
    let idx = 13 + (pin as usize) / 32;
    let shift = (pin % 32) as usize;
    unsafe {
        let val = ptr::read_volatile(gpio_reg(idx));
        Ok(((val >> shift) & 1) as u8)
    }
}

// MCP3008 ADC

#[repr(C)]
struct SpiTr {
    tx_buf:       u64,
    rx_buf:       u64,
    len:          u32,
    speed_hz:     u32,
    delay_us:     u16,
    bits_per_word:u8,
    cs_change:    u8,
    pad:          u32,
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
            if libc::ioctl(fd, SPI_WR_MODE as _, &m) < 0 {
                return Err(PyOSError::new_err("SPI mode set failed"));
            }
            let b: u8 = 8;
            if libc::ioctl(fd, SPI_WR_BITS as _, &b) < 0 {
                return Err(PyOSError::new_err("SPI bits set failed"));
            }
            let s = speed_khz * 1000;
            if libc::ioctl(fd, SPI_WR_SPEED as _, &s) < 0 {
                return Err(PyOSError::new_err("SPI speed set failed"));
            }
        }
        Ok(MCP3008 { fd, channel })
    }

    fn read_raw(&self) -> PyResult<u16> {
        let tx = [1, ((8 | self.channel) << 4) as u8, 0];
        let mut rx = [0u8; 3];
        let mut tr = SpiTr {
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
                ioc(1, SPI_MAGIC, 0, std::mem::size_of::<SpiTr>() as u32) as _,
                &mut tr,
            )
        };
        if ret < 1 {
            return Err(PyOSError::new_err("SPI transfer failed"));
        }
        Ok((((rx[1] & 0x03) as u16) << 8) | (rx[2] as u16))
    }

    fn value(&self) -> PyResult<f64> {
        Ok(self.read_raw()? as f64 / 1023.0)
    }

    fn read(&self) -> PyResult<(u16, f64)> {
        let r = self.read_raw()?;
        Ok((r, r as f64 / 1023.0))
    }
}

// RS485 Serial

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
struct Serial485 { fd: c_int, de_pin: u8 }

#[pymethods]
impl Serial485 {
    #[new]
    fn new(path: String, baud: u32, timeout: f64, de_pin: u8) -> PyResult<Self> {
        setup(de_pin, "OUT")?;
        output(de_pin, 0)?;
        let cpath = CString::new(path).unwrap();
        let fd = unsafe { open(cpath.as_ptr(), O_RDWR | O_NOCTTY | O_NONBLOCK) };
        if fd < 0 {
            return Err(PyOSError::new_err("Serial open failed"));
        }
        config_serial(fd, baud).map_err(PyOSError::new_err)?;
        unsafe {
            let mut tio: termios = std::mem::zeroed();
            tcgetattr(fd, &mut tio);
            tio.c_cc[VMIN]  = 0;
            tio.c_cc[VTIME] = (timeout * 10.0) as u8;
            tcsetattr(fd, TCSANOW, &tio);
        }
        Ok(Serial485 { fd, de_pin })
    }

    fn read(&self, size: usize) -> PyResult<Vec<u8>> {
        let mut buf = vec![0u8; size];
        let n = unsafe { read(self.fd, buf.as_mut_ptr() as *mut _, size) };
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

    fn read(&self, size: usize) -> PyResult<Vec<u8>> {
        let mut buf = vec![0u8; size];
        let n = unsafe { read(self.fd, buf.as_mut_ptr() as *mut _, size) };
        if n < 0 {
            return Err(PyOSError::new_err("Serial read failed"));
        }
        buf.truncate(n as usize);
        Ok(buf)
    }

    fn in_waiting(&self) -> PyResult<usize> {
        let mut bytes: c_int = 0;
        unsafe { ioctl(self.fd, FIONREAD, &mut bytes) };
        Ok(bytes as usize)
    }

    fn flush(&self) -> PyResult<()> {
        unsafe { tcdrain(self.fd); }
        Ok(())
    }

    fn reset_input_buffer(&self) -> PyResult<()> {
        unsafe { tcflush(self.fd, TCIFLUSH); }
        Ok(())
    }

    fn reset_output_buffer(&self) -> PyResult<()> {
        unsafe { tcflush(self.fd, TCOFLUSH); }
        Ok(())
    }
}

#[pymodule]
fn sysio(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(setmode, m)?)?;
    m.add_function(wrap_pyfunction!(setup, m)?)?;
    m.add_function(wrap_pyfunction!(output, m)?)?;
    m.add_function(wrap_pyfunction!(input, m)?)?;
    m.add_class::<MCP3008>()?;
    m.add_class::<Serial485>()?;
    Ok(())
}
