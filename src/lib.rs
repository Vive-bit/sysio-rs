use pyo3::prelude::*;
use pyo3::exceptions::PyOSError;
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::os::raw::c_ulong;
use std::os::unix::io::{AsRawFd, RawFd};
use libc;

const SPI_IOC_MAGIC: u8 = b'k';
const IOC_NRSHIFT: u32 = 0;
const IOC_TYPESHIFT: u32 = IOC_NRSHIFT + 8;
const IOC_SIZESHIFT: u32 = IOC_TYPESHIFT + 8;
const IOC_DIRSHIFT: u32 = IOC_SIZESHIFT + 14;
const IOC_WRITE: u32 = 1;

const fn _ioc(dir: u32, type_: u8, nr: u8, size: u32) -> u64 {
    ((dir << IOC_DIRSHIFT)
     | ((type_ as u32) << IOC_TYPESHIFT)
     | ((nr as u32) << IOC_NRSHIFT)
     | (size << IOC_SIZESHIFT)) as u64
}

const SPI_IOC_WR_MODE: c_ulong = _ioc(IOC_WRITE, SPI_IOC_MAGIC, 1, std::mem::size_of::<u8>() as u32) as c_ulong;
const SPI_IOC_WR_BITS_PER_WORD: c_ulong = _ioc(IOC_WRITE, SPI_IOC_MAGIC, 3, std::mem::size_of::<u8>() as u32) as c_ulong;
const SPI_IOC_WR_MAX_SPEED_HZ: c_ulong = _ioc(IOC_WRITE, SPI_IOC_MAGIC, 4, std::mem::size_of::<u32>() as u32) as c_ulong;

static mut PIN_MODE: Option<Mode> = None;
static BOARD_TO_BCM: [i32; 40] = [
     2, -1,  3,  4, -1,  0,  1, // 1-7
    14, -1, 15, 17, 18, 27, -1, // 8-14
    22, 23, -1, 24, 10,  9, 25,  // 15-21
    11,  8, -1,  7,  0,  1,  5,  // 22-28
    -1,  6, 12, 13, -1, 19, 16,  // 29-35
    26, 20, -1, 21              // 36-40
];

enum Mode { BCM, BOARD }

fn map_pin(pin: u8) -> PyResult<u8> {
    unsafe {
        match PIN_MODE {
            Some(Mode::BCM) => Ok(pin),
            Some(Mode::BOARD) => {
                let idx = pin as usize;
                if idx == 0 || idx > 40 {
                    return Err(PyOSError::new_err("Board pin out of range"));
                }
                let bcm = BOARD_TO_BCM[idx - 1];
                if bcm < 0 {
                    return Err(PyOSError::new_err("Invalid board pin"));
                }
                Ok(bcm as u8)
            }
            None => Err(PyOSError::new_err("Mode not set")),
        }
    }
}

fn write_sysfs(path: &str, value: &str) -> std::io::Result<()> {
    let mut file = OpenOptions::new().write(true).open(path)?;
    file.write_all(value.as_bytes())?;
    file.write_all(b"\n")
}

fn export_pin(pin: u8) -> PyResult<()> {
    let dir = format!("/sys/class/gpio/gpio{}", pin);
    if std::path::Path::new(&dir).exists() {
        return Ok(());
    }
    match write_sysfs("/sys/class/gpio/export", &pin.to_string()) {
        Ok(_) => Ok(()),
        Err(e) if matches!(e.raw_os_error(), Some(libc::EBUSY) | Some(libc::EINVAL)) => Ok(()),
        Err(e) => Err(PyOSError::new_err(e.to_string())),
    }
}

fn set_direction(pin: u8, dir: &str) -> PyResult<()> {
    let path = format!("/sys/class/gpio/gpio{}/direction", pin);
    write_sysfs(&path, dir).map_err(|e| PyOSError::new_err(e.to_string()))
}

fn write_value(pin: u8, val: u8) -> PyResult<()> {
    let path = format!("/sys/class/gpio/gpio{}/value", pin);
    write_sysfs(&path, if val == 0 { "0" } else { "1" })
        .map_err(|e| PyOSError::new_err(e.to_string()))
}

fn read_value(pin: u8) -> PyResult<u8> {
    let path = format!("/sys/class/gpio/gpio{}/value", pin);
    let mut buf = String::new();
    File::open(&path).and_then(|mut f| f.read_to_string(&mut buf))
        .map_err(|e| PyOSError::new_err(e.to_string()))?;
    buf.trim().parse::<u8>().map_err(|e| PyOSError::new_err(e.to_string()))
}

#[pyfunction]
fn setmode(mode: &str) -> PyResult<()> {
    unsafe {
        PIN_MODE = match mode {
            "BCM" => Some(Mode::BCM),
            "BOARD" => Some(Mode::BOARD),
            _ => return Err(PyOSError::new_err("Invalid mode")),
        };
    }
    Ok(())
}

#[pyfunction]
fn setup(pin: u8, direction: &str) -> PyResult<()> {
    let bcm = map_pin(pin)?;
    export_pin(bcm)?;
    let dir = match direction {
        "IN" => "in",
        "OUT" => "out",
        _ => return Err(PyOSError::new_err("Invalid direction")),
    };
    set_direction(bcm, dir)
}

#[pyfunction]
fn output(pin: u8, value: u8) -> PyResult<()> {
    let bcm = map_pin(pin)?;
    write_value(bcm, value)
}

#[pyfunction]
fn input(pin: u8) -> PyResult<u8> {
    let bcm = map_pin(pin)?;
    read_value(bcm)
}

#[repr(C)]
struct SpiIocTransfer { tx_buf: u64, rx_buf: u64, len: u32, speed_hz: u32, delay_us: u16, bits_per_word: u8, cs_change: u8, pad: u32 }

fn spi_ioc_message(n: u32) -> u64 { _ioc(IOC_WRITE, SPI_IOC_MAGIC, 0, std::mem::size_of::<SpiIocTransfer>() as u32 * n) }

#[pyclass]
struct MCP3008 { fd: RawFd, channel: u8 }

#[pymethods]
impl MCP3008 {
    #[new]
    fn new(bus: u8, cs: u8, channel: u8, speed_khz: u32) -> PyResult<Self> {
        let path = format!("/dev/spidev{}.{}", bus, cs);
        let file = OpenOptions::new().read(true).write(true).open(&path)
            .map_err(|e| PyOSError::new_err(e.to_string()))?;
        let fd = file.as_raw_fd();
        unsafe {
            if libc::ioctl(fd, SPI_IOC_WR_MODE, &0u8) < 0 { return Err(PyOSError::new_err("SPI mode set failed")); }
            if libc::ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &8u8) < 0 { return Err(PyOSError::new_err("SPI bits set failed")); }
            let speed = speed_khz * 1000;
            if libc::ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0 { return Err(PyOSError::new_err("SPI speed set failed")); }
        }
        Ok(MCP3008 { fd, channel })
    }
    fn read_raw(&self) -> PyResult<u16> {
        let tx = [1, ((8 | self.channel) << 4) as u8, 0];
        let mut rx = [0u8; 3];
        let mut tr = SpiIocTransfer { tx_buf: tx.as_ptr() as u64, rx_buf: rx.as_mut_ptr() as u64, len: 3, speed_hz: 0, delay_us: 0, bits_per_word: 8, cs_change: 0, pad: 0 };
        let ret = unsafe { libc::ioctl(self.fd, spi_ioc_message(1) as c_ulong, &mut tr as *mut _) };
        if ret < 1 { return Err(PyOSError::new_err("SPI transfer failed")); }
        Ok((((rx[1] & 3) as u16) << 8) | (rx[2] as u16))
    }
    fn value(&self) -> PyResult<f64> { Ok(self.read_raw()? as f64 / 1023.0) }
}

fn configure_serial(fd: RawFd, baud: u32) -> Result<(), String> {
    unsafe {
        let mut tio: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut tio) != 0 { return Err("termios get failed".into()); }
        libc::cfmakeraw(&mut tio);
        let speed = match baud { 9600 => libc::B9600, 38400 => libc::B38400, 115200 => libc::B115200, _ => return Err("Unsupported baud".into()) };
        libc::cfsetispeed(&mut tio, speed);
        libc::cfsetospeed(&mut tio, speed);
        if libc::tcsetattr(fd, libc::TCSANOW, &tio) != 0 { return Err("termios set failed".into()); }
    }
    Ok(())
}

#[pyclass]
struct Serial485 { fd: RawFd, de_pin: u8 }

#[pymethods]
impl Serial485 {
    #[new]
    fn new(path: String, baud: u32, timeout: f64, de: u8) -> PyResult<Self> {
        unsafe { setup(de, "OUT")?; write_value(de, 0)?; }
        let file = OpenOptions::new().read(true).write(true).open(&path)
            .map_err(|e| PyOSError::new_err(e.to_string()))?;
        let fd = file.as_raw_fd();
        configure_serial(fd, baud).map_err(|e| PyOSError::new_err(e))?;
        unsafe {
            let mut tio: libc::termios = std::mem::zeroed();
            libc::tcgetattr(fd, &mut tio);
            tio.c_cc[libc::VMIN] = 0;
            tio.c_cc[libc::VTIME] = (timeout * 10.0) as u8;
            libc::tcsetattr(fd, libc::TCSANOW, &tio);
        }
        Ok(Serial485 { fd, de_pin: de })
    }
    fn write(&self, data: &[u8]) -> PyResult<usize> {
        write_value(self.de_pin, 1)?;
        let n = unsafe { libc::write(self.fd, data.as_ptr() as *const _, data.len()) };
        if n < 0 { return Err(PyOSError::new_err("write failed")); }
        unsafe { libc::tcdrain(self.fd); }
        write_value(self.de_pin, 0)?;
        Ok(n as usize)
    }
    fn read(&self, size: usize) -> PyResult<Vec<u8>> {
        let mut buf = vec![0; size];
        let n = unsafe { libc::read(self.fd, buf.as_mut_ptr() as *mut _, size) };
        if n < 0 { return Err(PyOSError::new_err("read failed")); }
        buf.truncate(n as usize);
        Ok(buf)
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
