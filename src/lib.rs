use pyo3::prelude::*;
use pyo3::exceptions::PyOSError;
use std::ptr;
use std::os::unix::io::AsRawFd;
use std::fs::OpenOptions;
use libc::{
    open, mmap, close, O_RDWR, PROT_READ, PROT_WRITE, MAP_SHARED, c_char,
    termios, tcgetattr, tcsetattr, cfmakeraw, cfsetispeed, cfsetospeed, tcflush,
    tcdrain, B9600, B38400, B115200, TCSANOW, VMIN, VTIME, FIONREAD, ioctl, read, write, c_int,
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

fn gpio_register(idx: usize) -> *mut u32 {
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
        let reg = gpio_register(fsel);
        let val = ptr::read_volatile(reg);
        let mask = !(0b111 << shift);
        let mode_bits = if direction == "OUT" { 0b001 } else { 0b000 };
        ptr::write_volatile(reg, (val & mask) | (mode_bits << shift));
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
        let reg = gpio_register(idx);
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
        let val = ptr::read_volatile(gpio_register(idx));
        Ok(((val >> shift) & 1) as u8)
    }
}

#[repr(C)]
struct SpiIocTransfer {
    tx_buf: u64,
    rx_buf: u64,
    len: u32,
    speed_hz: u32,
    delay_us: u16,
    bits_per_word: u8,
    cs_change: u8,
    pad: u32,
}
fn _ioc(dir: u32, type_: u8, nr: u8, size: u32) -> u64 {
    const IOC_NRSHIFT: u32 = 0;
    const IOC_TYPESHIFT: u32 = IOC_NRSHIFT + 8;
    const IOC_SIZESHIFT: u32 = IOC_TYPESHIFT + 8;
    const IOC_DIRSHIFT: u32 = IOC_SIZESHIFT + 14;
    ((dir << IOC_DIRSHIFT)
     | ((type_ as u32) << IOC_TYPESHIFT)
     | ((nr as u32) << IOC_NRSHIFT)
     | (size << IOC_SIZESHIFT)) as u64
}
const SPI_IOC_MAGIC: u8 = b'k';
const SPI_IOC_WR_MODE: u64 = _ioc(1, SPI_IOC_MAGIC, 1, std::mem::size_of::<u8>() as u32);
const SPI_IOC_WR_BITS: u64 = _ioc(1, SPI_IOC_MAGIC, 3, std::mem::size_of::<u8>() as u32);
const SPI_IOC_WR_SPEED: u64 = _ioc(1, SPI_IOC_MAGIC, 4, std::mem::size_of::<u32>() as u32);

#[pyclass]
struct MCP3008 { fd: c_int, channel: u8 }

#[pymethods]
impl MCP3008 {
    #[new]
    fn new(bus: u8, cs: u8, channel: u8, speed_khz: u32) -> PyResult<Self> {
        let dev = format!("/dev/spidev{}.{}", bus, cs);
        let file = OpenOptions::new().read(true).write(true)
            .open(&dev).map_err(|e| PyOSError::new_err(e.to_string()))?;
        let fd = file.as_raw_fd();
        unsafe {
            let mode: u8 = 0;
            if libc::ioctl(fd, SPI_IOC_WR_MODE as _, &mode) < 0 {
                return Err(PyOSError::new_err("SPI mode set failed"));
            }
            let bits: u8 = 8;
            if libc::ioctl(fd, SPI_IOC_WR_BITS as _, &bits) < 0 {
                return Err(PyOSError::new_err("SPI bits set failed"));
            }
            let speed = speed_khz * 1000;
            if libc::ioctl(fd, SPI_IOC_WR_SPEED as _, &speed) < 0 {
                return Err(PyOSError::new_err("SPI speed set failed"));
            }
        }
        Ok(MCP3008 { fd, channel })
    }
    fn read_raw(&self) -> PyResult<u16> {
        let tx = [1, ((8 | self.channel) << 4) as u8, 0];
        let mut rx = [0u8;3];
        let mut tr = SpiIocTransfer {
            tx_buf: tx.as_ptr() as u64,
            rx_buf: rx.as_mut_ptr() as u64,
            len: 3, speed_hz: 0, delay_us: 0,
            bits_per_word:8, cs_change:0, pad:0,
        };
        let ret = unsafe { libc::ioctl(self.fd, _ioc(1, SPI_IOC_MAGIC, 0, std::mem::size_of::<SpiIocTransfer>() as u32) as _, &mut tr) };
        if ret < 1 {
            return Err(PyOSError::new_err("SPI transfer failed"));
        }
        Ok((((rx[1] & 3) as u16) << 8) | (rx[2] as u16))
    }
    fn value(&self) -> PyResult<f64> {
        Ok(self.read_raw()? as f64 / 1023.0)
    }
    fn read(&self) -> PyResult<(u16,f64)> {
        let raw = self.read_raw()?;
        Ok((raw, raw as f64/1023.0))
    }
}

fn configure_serial(fd: c_int, baud: u32) -> Result<(), String> {
    unsafe {
        let mut tio: termios = std::mem::zeroed();
        if tcgetattr(fd, &mut tio) != 0 {
            return Err("tcgetattr failed".into());
        }
        cfmakeraw(&mut tio);
        let speed = match baud {
            9600 => B9600,
            38400 => B38400,
            115200 => B115200,
            _ => return Err("Unsupported baud".into()),
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
        let file = OpenOptions::new().read(true).write(true)
            .open(&path).map_err(|e| PyOSError::new_err(e.to_string()))?;
        let fd = file.as_raw_fd();
        configure_serial(fd, baud).map_err(|e| PyOSError::new_err(e))?;
        unsafe {
            let mut tio: termios = std::mem::zeroed();
            tcgetattr(fd, &mut tio);
            tio.c_cc[VMIN] = 0;
            tio.c_cc[VTIME] = (timeout * 10.0) as u8;
            tcsetattr(fd, TCSANOW, &tio);
        }
        Ok(Serial485 { fd, de_pin })
    }
    fn write(&self, data: &[u8]) -> PyResult<usize> {
        output(self.de_pin, 1)?;
        let n = unsafe { write(self.fd, data.as_ptr() as *const _, data.len()) };
        if n < 0 {
            return Err(PyOSError::new_err("Serial write failed"));
        }
        unsafe { tcdrain(self.fd); }
        output(self.de_pin, 0)?;
        Ok(n as usize)
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
        unsafe {
            ioctl(self.fd, FIONREAD, &mut bytes);
        }
        Ok(bytes as usize)
    }
    fn flush(&self) -> PyResult<()> {
        unsafe { tcdrain(self.fd); }
        Ok(())
    }
    fn reset_input_buffer(&self) -> PyResult<()> {
        unsafe { tcflush(self.fd, libc::TCIFLUSH); }
        Ok(())
    }
    fn reset_output_buffer(&self) -> PyResult<()> {
        unsafe { tcflush(self.fd, libc::TCOFLUSH); }
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
