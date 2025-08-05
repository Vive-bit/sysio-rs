# sysio-rs
A native Rust extension for Python providing direct `GPIO, SPI (MCP3008) and RS-485 serial access on Linux`, with no external dependencies beyond `pyo3` and `libc`.
Currently used for super performance critical components of my py projects.

# Features
## GPIO via /dev/gpiomem + mmap

- setmode(mode: "BOARD" | "BCM")
- setup(pin: int, direction: "IN" | "OUT")
- output(pin: int, value: 0 | 1)
- input(pin: int) -> int

## SPI

- MCP3008(bus: int, cs: int, channel: int, speed_khz: int)
  - .read_raw() `-> int (0–1023)`
  - .value `-> float (0.0–1.0)`
  - .bits() `-> int (const. 10)`
  - .read() `-> (raw: int, normalized: float)`

## RS-485 Serial via termios + direction-pin toggling

- Serial485(path: str, baud: int, timeout_s: float, de_pin: int)
  - .write(data: bytes) `-> int`
  - .read(size: int) `-> bytes`
  - .in_waiting() `-> int`
  - .flush()
  - .reset_input_buffer()
  - .reset_output_buffer()

## Utility

- sleep_s(s: float)
- sleep_ms(ms: float)
- sleep_us(us: float)
- time_time()

# Modules
## GPIO
- Manages memory-mapped I/O for Raspberry Pi
- Lazy init of /dev/gpiomem
- Thread-safe via OnceLock<Mutex>

## SPI
- Raw SPI ioctl calls to /dev/spidevX.Y
- Custom _IOC helper to calculate request codes 

## Serial
- Opens serial port non-blocking, toggles DE-pin for RS-485
- Configures termios for raw mode and timeouts
- Methods mirror Python serial package