use pyo3::prelude::*;
use pyo3::exceptions::PyOSError;
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::os::raw::c_ulong;
use std::os::unix::io::{AsRawFd, RawFd};
use std::thread::sleep;

static mut PIN_MODE: Option<Mode> = None;
static BOARD_TO_BCM: [i32; 40] = [
    -1, -1, -1, 2, -1, 3, 4, //  1–7
    -1, 14, -1, 15, 17, 18, 27, //  8–14
    -1, 22, 23, -1, 24, 10, 9,  // 15–21
    25, 11, 8, -1, 7, 0, 1,     // 22–28
    5, -1, 6, 12, 13, -1, 19,   // 29–35
    16, 26, 20, -1, 21          // 36–40
];

#[derive(Copy, Clone)]
enum Mode { BCM, BOARD }

fn map_pin(pin: u8) -> PyResult<u8> {
    unsafe {
        match PIN_MODE {
            Some(Mode::BCM) => Ok(pin),
            Some(Mode::BOARD) => {
                let idx = pin as usize;
                if idx==0 || idx>40 { return Err(PyOSError::new_err("Board-Pin außerhalb 1–40")); }
                let bcm = BOARD_TO_BCM[idx];
                if bcm<0 { return Err(PyOSError::new_err("Board-Pin ist kein GPIO")); }
                Ok(bcm as u8)
            }
            None => Err(PyOSError::new_err("setmode() zuerst aufrufen")),
        }
    }
}

fn sysfs_write(path: &str, val: &str) -> std::io::Result<()> {
    let mut f = OpenOptions::new().write(true).open(path)?;
    f.write_all(val.as_bytes())
}

fn export_pin(pin: u8) -> PyResult<()> {
    sysfs_write("/sys/class/gpio/export", &format!("{}", pin))
        .or_else(|e| if e.kind().to_string().contains("Device or resource busy") { Ok(()) } else { Err(e) })
        .map_err(|e| PyOSError::new_err(e.to_string()))
}

fn set_direction(pin: u8, dir: &str) -> PyResult<()> {
    let path = format!("/sys/class/gpio/gpio{}/direction", pin);
    sysfs_write(&path, dir).map_err(|e| PyOSError::new_err(e.to_string()))
}

fn write_value(pin: u8, v: u8) -> PyResult<()> {
    let path = format!("/sys/class/gpio/gpio{}/value", pin);
    sysfs_write(&path, if v==0 { "0" } else { "1" })
        .map_err(|e| PyOSError::new_err(e.to_string()))
}

fn read_value(pin: u8) -> PyResult<u8> {
    let path = format!("/sys/class/gpio/gpio{}/value", pin);
    let mut buf = String::new();
    File::open(&path)
        .and_then(|mut f| f.read_to_string(&mut buf))
        .map_err(|e| PyOSError::new_err(e.to_string()))?;
    Ok(buf.trim().parse::<u8>()
        .map_err(|e| PyOSError::new_err(e.to_string()))?)
}

#[pyfunction]
fn setmode(mode: &str) -> PyResult<()> {
    unsafe {
        PIN_MODE = match mode {
            "BCM"   => Some(Mode::BCM),
            "BOARD" => Some(Mode::BOARD),
            _       => return Err(PyOSError::new_err("Mode muss BCM oder BOARD sein")),
        };
    }
    Ok(())
}

#[pyfunction]
fn setup(pin: u8, direction: &str) -> PyResult<()> {
    let bcm = map_pin(pin)?;
    export_pin(bcm)?;
    let dir = match direction {
        "IN"  => "in",
        "OUT" => "out",
        _     => return Err(PyOSError::new_err("Direction muss IN oder OUT sein")),
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

// MCP3008 using /dev/spidev0.*

#[repr(C)]
struct SpiIocTransfer {
    tx_buf:    u64,
    rx_buf:    u64,
    len:       u32,
    speed_hz:  u32,
    delay_us:  u16,
    bits_per_word: u8,
    cs_change: u8,
    pad:       u32,
}

const SPI_IOC_MAGIC: u8 = b'k';
const IOC_NRSHIFT:  u32 = 0;
const IOC_TYPESHIFT: u32 = IOC_NRSHIFT + 8;
const IOC_SIZESHIFT: u32 = IOC_TYPESHIFT + 8;
const IOC_DIRSHIFT:  u32 = IOC_SIZESHIFT + 14;

const IOC_WRITE: u32 = 1;

const fn _ioc(dir: u32, type_: u8, nr: u8, size: u32) -> u64 {
    ((dir << IOC_DIRSHIFT)
        | ((type_ as u32) << IOC_TYPESHIFT)
        | ((nr as u32) << IOC_NRSHIFT)
        | (size << IOC_SIZESHIFT)) as u64
}

const fn spi_ioc_message(n: u32) -> u64 {
    // _IOW(SPI_IOC_MAGIC, 0, struct spi_ioc_transfer[n])
    _ioc(IOC_WRITE, SPI_IOC_MAGIC, 0, std::mem::size_of::<SpiIocTransfer>() as u32 * n)
}

#[pyclass]
struct MCP3008 {
    fd: RawFd,
    channel: u8,
}

#[pymethods]
impl MCP3008 {
    #[new]
    fn new(bus: u8, cs: u8, channel: u8, speed_khz: u32) -> PyResult<Self> {
        let dev = format!("/dev/spidev{}.{}", bus, cs);
        let file = OpenOptions::new().read(true).write(true).open(&dev)
            .map_err(|e| PyOSError::new_err(e.to_string()))?;
        let fd = file.as_raw_fd();

        // mode = 0, bits = 8, max_speed_hz = speed_khz*1000
        unsafe {
            let mode: u8 = 0;
            if libc::ioctl(fd, libc::SPI_IOC_WR_MODE as c_ulong, &mode) < 0 {
                return Err(PyOSError::new_err("SPI set mode failed"));
            }
            let bits: u8 = 8;
            if libc::ioctl(fd, libc::SPI_IOC_WR_BITS_PER_WORD as c_ulong, &bits) < 0 {
                return Err(PyOSError::new_err("SPI set bits failed"));
            }
            let speed = speed_khz * 1000;
            if libc::ioctl(fd, libc::SPI_IOC_WR_MAX_SPEED_HZ as c_ulong, &speed) < 0 {
                return Err(PyOSError::new_err("SPI set speed failed"));
            }
        }

        Ok(MCP3008 { fd, channel })
    }

    fn read_raw(&self) -> PyResult<u16> {
        let tx = [
            0x01,
            ((0x08 | self.channel) << 4) as u8,
            0x00,
        ];
        let mut rx = [0u8; 3];
        let mut tr = SpiIocTransfer {
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
            libc::ioctl(self.fd,
                        spi_ioc_message(1) as c_ulong,
                        &mut tr as *mut SpiIocTransfer)
        };
        if ret < 1 {
            return Err(PyOSError::new_err("SPI transfer failed"));
        }

        // 10-bit val: rx[1] & 0x03 << 8 | rx[2]
        let value = (((rx[1] & 0x03) as u16) << 8) | (rx[2] as u16);
        Ok(value)
    }

    fn value(&self) -> PyResult<f64> {
        let raw = self.read_raw()?;
        Ok(raw as f64 / 1023.0)
    }
}

// Serial RS-485 w. termios + DE-Pin Toggling

fn configure_serial(fd: RawFd, baud: u32) -> Result<(), String> {
    unsafe {
        let mut tio: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut tio) != 0 {
            return Err("tcgetattr failed".into());
        }
        libc::cfmakeraw(&mut tio);

        let speed = match baud {
            9600   => libc::B9600,
            38400  => libc::B38400,
            115200 => libc::B115200,
            _      => return Err("Unsupported baudrate".into()),
        };
        libc::cfsetispeed(&mut tio, speed);
        libc::cfsetospeed(&mut tio, speed);
        if libc::tcsetattr(fd, libc::TCSANOW, &tio) != 0 {
            return Err("tcsetattr failed".into());
        }
    }
    Ok(())
}

#[pyclass]
struct Serial485 {
    fd: RawFd,
    de_pin: u8,
}

#[pymethods]
impl Serial485 {
    #[new]
    fn new(path: String, baud: u32, timeout_s: f64, de_pin: u8) -> PyResult<Self> {
        unsafe { setup(de_pin, "OUT")?; write_value(de_pin, 0)?; }

        let file = OpenOptions::new()
            .read(true).write(true).open(&path)
            .map_err(|e| PyOSError::new_err(e.to_string()))?;
        let fd = file.as_raw_fd();
        configure_serial(fd, baud)
            .map_err(|e| PyOSError::new_err(e))?;

        unsafe {
            let mut tio: libc::termios = std::mem::zeroed();
            libc::tcgetattr(fd, &mut tio);
            tio.c_cc[libc::VMIN]  = 0;
            tio.c_cc[libc::VTIME] = (timeout_s * 10.0) as u8;
            libc::tcsetattr(fd, libc::TCSANOW, &tio);
        }
        Ok(Serial485 { fd, de_pin })
    }

    fn write(&self, data: &[u8]) -> PyResult<usize> {
        write_value(self.de_pin, 1)?;
        let n = unsafe { libc::write(self.fd, data.as_ptr() as *const _, data.len()) };
        if n < 0 {
            return Err(PyOSError::new_err("Serial write failed"));
        }
        unsafe { libc::tcdrain(self.fd); }
        // DE LOW
        write_value(self.de_pin, 0)?;
        Ok(n as usize)
    }

    fn read(&self, size: usize) -> PyResult<Vec<u8>> {
        let mut buf = vec![0u8; size];
        let n = unsafe { libc::read(self.fd, buf.as_mut_ptr() as *mut _, size) };
        if n < 0 {
            return Err(PyOSError::new_err("Serial read failed"));
        }
        buf.truncate(n as usize);
        Ok(buf)
    }
}

#[pymodule]
fn rustgpio(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(setmode, m)?)?;
    m.add_function(wrap_pyfunction!(setup, m)?)?;
    m.add_function(wrap_pyfunction!(output, m)?)?;
    m.add_function(wrap_pyfunction!(input, m)?)?;
    m.add_class::<MCP3008>()?;
    m.add_class::<Serial485>()?;
    Ok(())
}
