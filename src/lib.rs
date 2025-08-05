use pyo3::prelude::*;
use pyo3::exceptions::PyOSError;
use std::{ptr, ffi::CString};
use libc::{
    open, mmap, close, read, write, ioctl,
    O_RDWR, O_NOCTTY, O_NONBLOCK,
    PROT_READ, PROT_WRITE, MAP_SHARED, c_char, c_int,
    termios, tcgetattr, tcsetattr, cfmakeraw,
    cfsetispeed, cfsetospeed, tcdrain, tcflush,
    B9600, B38400, B115200, TCSANOW, VMIN, VTIME, FIONREAD
};

const GPIO_LEN: usize = 0xB4;
static mut GPIO_MEM: *mut u32 = ptr::null_mut();

fn init_gpio() -> PyResult<()> {
    unsafe {
        if !GPIO_MEM.is_null() { return Ok(()); }
        let fd = open(b"/dev/gpiomem\0".as_ptr() as *const c_char, O_RDWR);
        if fd < 0 { return Err(PyOSError::new_err("Failed to open /dev/gpiomem")); }
        let m = mmap(ptr::null_mut(), GPIO_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
        close(fd);
        if m == libc::MAP_FAILED { return Err(PyOSError::new_err("GPIO mmap failed")); }
        GPIO_MEM = m as *mut u32;
        Ok(())
    }
}

fn gpio_reg(idx: usize) -> *mut u32 {
    unsafe { GPIO_MEM.add(idx) }
}

#[pyfunction]
fn setmode(_mode: &str) -> PyResult<()> {
    init_gpio()?; Ok(())
}

#[pyfunction]
fn setup(pin: u8, direction: &str) -> PyResult<()> {
    init_gpio()?;
    let fsel = (pin as usize) / 10;
    let shift = ((pin % 10) * 3) as usize;
    unsafe {
        let r = gpio_reg(fsel);
        let v = ptr::read_volatile(r);
        let m = !(0b111 << shift);
        let bits = if direction=="OUT" {0b001} else {0b000};
        ptr::write_volatile(r, (v & m) | (bits<<shift));
    }
    Ok(())
}

#[pyfunction]
fn output(pin: u8, value: u8) -> PyResult<()> {
    init_gpio()?;
    let idx = if value==0 { 10 + (pin as usize)/32 } else { 7 + (pin as usize)/32 };
    let shift = (pin%32) as usize;
    unsafe { ptr::write_volatile(gpio_reg(idx), 1<<shift); }
    Ok(())
}

#[pyfunction]
fn input(pin: u8) -> PyResult<u8> {
    init_gpio()?;
    let idx = 13 + (pin as usize)/32;
    let shift = (pin%32) as usize;
    unsafe {
        let v = ptr::read_volatile(gpio_reg(idx));
        Ok(((v>>shift)&1) as u8)
    }
}

// MCP3008

#[repr(C)]
struct SpiTr { tx_buf:u64, rx_buf:u64, len:u32, speed_hz:u32, delay_us:u16, bits_per_word:u8, cs_change:u8, pad:u32 }

const SPI_MAGIC: u8 = b'k';
const fn ioc(dir:u32, t:u8, n:u8, s:u32)->u64 {
    const NR: u32 = 0;
    const TS: u32 = NR+8;
    const SS: u32 = TS+8;
    const DS: u32 = SS+14;
    ((dir<<DS)|((t as u32)<<TS)|((n as u32)<<NR)|(s<<SS)) as u64
}
const SPI_WR_MODE: u64 = ioc(1,SPI_MAGIC,1,std::mem::size_of::<u8>() as u32);
const SPI_WR_BITS: u64 = ioc(1,SPI_MAGIC,3,std::mem::size_of::<u8>() as u32);
const SPI_WR_SPEED: u64 = ioc(1,SPI_MAGIC,4,std::mem::size_of::<u32>() as u32);

#[pyclass]
struct MCP3008 { fd: c_int, channel:u8 }

#[pymethods]
impl MCP3008 {
    #[new]
    fn new(bus:u8, cs:u8, channel:u8, speed_khz:u32)->PyResult<Self> {
        let dev = format!("/dev/spidev{}.{}",bus,cs);
        let cd = CString::new(dev.clone()).unwrap();
        let fd = unsafe { open(cd.as_ptr(), O_RDWR) };
        if fd<0 { return Err(PyOSError::new_err("SPI open failed")); }
        unsafe{
            let m:u8=0;
            if libc::ioctl(fd, SPI_WR_MODE as _, &m)<0 {return Err(PyOSError::new_err("SPI mode"))}
            let b:u8=8;
            if libc::ioctl(fd, SPI_WR_BITS as _, &b)<0 {return Err(PyOSError::new_err("SPI bits"))}
            let s = speed_khz*1000;
            if libc::ioctl(fd, SPI_WR_SPEED as _, &s)<0 {return Err(PyOSError::new_err("SPI speed"))}
        }
        Ok(MCP3008{fd,channel})
    }
    fn read_raw(&self)->PyResult<u16>{
        let tx=[1,((8|self.channel)<<4) as u8,0];
        let mut rx=[0u8;3];
        let mut tr=SpiTr{tx_buf:tx.as_ptr() as u64, rx_buf:rx.as_mut_ptr() as u64, len:3,speed_hz:0,delay_us:0,bits_per_word:8,cs_change:0,pad:0};
        let r=unsafe{ libc::ioctl(self.fd, ioc(1,SPI_MAGIC,0,std::mem::size_of::<SpiTr>() as u32) as _, &mut tr) };
        if r<1 { return Err(PyOSError::new_err("SPI xfer"))}
        Ok((((rx[1]&3) as u16)<<8)|(rx[2] as u16))
    }
    fn value(&self)->PyResult<f64> { Ok(self.read_raw()? as f64/1023.0) }
    fn read(&self)->PyResult<(u16,f64)>{ let r=self.read_raw()?; Ok((r,r as f64/1023.0)) }
}

// RS485

fn config_serial(fd:c_int,baud:u32)->Result<(),String>{
    unsafe {
        let mut tio:termios=std::mem::zeroed();
        if tcgetattr(fd,&mut tio)!=0 {return Err("tcgetattr".into())}
        cfmakeraw(&mut tio);
        let sp=match baud{9600=>B9600,38400=>B38400,115200=>B115200,_=>return Err("baud".into())};
        cfsetispeed(&mut tio,sp); cfsetospeed(&mut tio,sp);
        if tcsetattr(fd,TCSANOW,&tio)!=0 {return Err("tcsetattr".into())}
    }
    Ok(())
}

#[pyclass]
struct Serial485{ fd:c_int, de_pin:u8 }

#[pymethods]
impl Serial485 {
    #[new]
    fn new(path:String, baud:u32, timeout:f64, de_pin:u8)->PyResult<Self>{
        setup(de_pin,"OUT")?; output(de_pin,0)?;
        let cd=CString::new(path).unwrap();
        let fd=unsafe{ open(cd.as_ptr(), O_RDWR|O_NOCTTY|O_NONBLOCK) };
        if fd<0 {return Err(PyOSError::new_err("open serial"))}
        config_serial(fd,baud).map_err(PyOSError::new_err)?;
        unsafe{
            let mut tio:termios=std::mem::zeroed();
            tcgetattr(fd,&mut tio);
            tio.c_cc[VMIN]=0;
            tio.c_cc[VTIME]=(timeout*10.0) as u8;
            tcsetattr(fd,TCSANOW,&tio);
        }
        Ok(Serial485{fd,de_pin})
    }

    fn write(&self,data:&[u8])->PyResult<usize>{
        output(self.de_pin,1)?;
        let n=unsafe{ write(self.fd,data.as_ptr() as *const _,data.len()) };
        if n<0 {return Err(PyOSError::new_err("Serial write failed"))}
        unsafe{ tcdrain(self.fd) }
        output(self.de_pin,0)?;
        Ok(n as usize)
    }

    fn read(&self,size:usize)->PyResult<Vec<u8>>{
        let mut buf=vec![0;size];
        let n=unsafe{ read(self.fd,buf.as_mut_ptr() as *mut _,size) };
        if n<0 {return Err(PyOSError::new_err("Serial read failed"))}
        buf.truncate(n as usize);
        Ok(buf)
    }

    fn in_waiting(&self)->PyResult<usize>{
        let mut b=0;
        unsafe{ ioctl(self.fd,FIONREAD,&mut b) };
        Ok(b as usize)
    }

    fn flush(&self)->PyResult<()>{ unsafe{tcdrain(self.fd)}; Ok(()) }
    fn reset_input_buffer(&self)->PyResult<()>{ unsafe{tcflush(self.fd,libc::TCIFLUSH)}; Ok(()) }
    fn reset_output_buffer(&self)->PyResult<()>{ unsafe{tcflush(self.fd,libc::TCOFLUSH)}; Ok(()) }
}

#[pymodule]
fn sysio(_py:Python,m:&PyModule)->PyResult<()>{
    m.add_function(wrap_pyfunction!(setmode,m)?)?;
    m.add_function(wrap_pyfunction!(setup,m)?)?;
    m.add_function(wrap_pyfunction!(output,m)?)?;
    m.add_function(wrap_pyfunction!(input,m)?)?;
    m.add_class::<MCP3008>()?;
    m.add_class::<Serial485>()?;
    Ok(())
}
