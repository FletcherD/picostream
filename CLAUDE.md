# pico-bitstream

USB device for streaming arbitrary digital bitstreams out of an RP2040 GPIO pin.

## Project Structure

```
pico-bitstream/
├── Cargo.toml           # Workspace definition
├── shared/              # no_std crate with common definitions
│   └── src/lib.rs
├── firmware/            # RP2040 Embassy firmware
│   ├── .cargo/config.toml
│   ├── build.rs
│   ├── memory.x
│   └── src/main.rs
└── client/              # Desktop CLI using nusb
    └── src/main.rs
```

## Crates

### shared (`pico-bitstream-shared`)
Common definitions shared between firmware and client:
- USB VID/PID (0x1209/0x0001)
- Endpoint addresses (0x01 OUT, 0x81 IN)
- Control request codes (SET_SAMPLE_RATE, START, STOP, GET_STATUS)
- `BufferStatus` struct with to_bytes/from_bytes serialization
- Constants (32KB ring buffer, GPIO 0, 125MHz clock)

### firmware (`pico-bitstream-firmware`)
RP2040 firmware using Embassy 0.9:
- USB device with vendor-specific interface (class 0xFF)
- Bulk OUT endpoint receives data into 32KB ring buffer
- PIO state machine outputs bits at configurable sample rate
- Single PIO instruction: `out pins, 1` (MSB first, autopull)
- DMA transfers from ring buffer to PIO TX FIFO
- Underrun detection and counting

Key components:
- `DeviceState`: Shared state protected by critical-section mutex
- `ControlHandler`: Handles vendor USB control requests (sync callbacks)
- `bulk_handler`: Async task reading USB bulk OUT, feeding PIO via DMA

### client (`pico-bitstream`)
Desktop CLI using nusb 0.2:
- Device discovery by VID/PID
- Control requests for configuration
- Bulk OUT streaming with backpressure (8 pending transfers)
- Status display on stderr (~10 Hz updates)
- Ctrl-C and EOF handling

## Building

### Firmware
```bash
cd firmware
cargo build --release --target thumbv6m-none-eabi
```

Flash with picotool (device in BOOTSEL mode):
```bash
picotool load -x target/thumbv6m-none-eabi/release/pico-bitstream-firmware
```

### Client
```bash
cargo build --release -p pico-bitstream
```

## Usage

```bash
# Stream 1MHz bitstream
echo -ne '\xAA\xAA\xAA\xAA' | pico-bitstream -s 1000000

# Continuous streaming from file
cat data.bin | pico-bitstream -s 100000

# Test 500Hz square wave (0xAA = 10101010 at 1kHz = 500Hz fundamental)
echo -ne '\xAA\xAA' | pico-bitstream -s 1000
```

## USB Protocol

| Request | Code | Direction | Data | Description |
|---------|------|-----------|------|-------------|
| SET_SAMPLE_RATE | 0x01 | OUT | u32 LE | Set output rate in Hz |
| START | 0x02 | OUT | none | Start streaming, clear buffer and underrun count |
| STOP | 0x03 | OUT | none | Stop streaming |
| GET_STATUS | 0x10 | IN | BufferStatus | Get buffer status |

BufferStatus (16 bytes):
- bytes_used: u32 - bytes currently in buffer
- buffer_size: u32 - total buffer capacity (32768)
- underrun_count: u32 - underruns since last START
- is_running: u8 - streaming active flag
- _reserved: [u8; 3]

## Hardware

- Output: GPIO 0
- System clock: 125 MHz
- Sample rate: clock_divider = 125MHz / sample_rate

## Dependencies

Firmware uses Embassy 0.9.x ecosystem:
- embassy-executor, embassy-rp, embassy-usb, embassy-sync, embassy-time
- defmt/defmt-rtt for logging
- fixed for clock divider calculations

Client uses:
- nusb 0.2 for USB communication
- clap for CLI argument parsing
- ctrlc for signal handling
