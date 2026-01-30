# pico-bitstream

USB device for streaming arbitrary digital bitstreams out of an RP2040/RP235x GPIO pin.

## Project Structure

```
pico-bitstream/
├── Cargo.toml           # Workspace definition
├── .cargo/config.toml   # Workspace-level runner config
├── shared/              # no_std crate with common definitions
│   └── src/lib.rs
├── firmware/            # RP2040/RP235x Embassy firmware
│   ├── .cargo/config.toml  # Default target (thumbv6m-none-eabi)
│   ├── build.rs            # Selects memory.x, sets linker args
│   ├── memory-rp2040.x     # RP2040 linker script (boot2)
│   ├── memory-rp235x.x     # RP235x linker script (start_block/end_block)
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
RP2040/RP235x firmware using Embassy 0.9:
- USB device with vendor-specific interface (class 0xFF)
- Bulk OUT endpoint receives data into 32KB ring buffer
- PIO state machine outputs bits at configurable sample rate
- Single PIO instruction: `out pins, 1` (MSB first, autopull)
- DMA transfers from ring buffer to PIO TX FIFO
- Underrun detection and counting

Cargo features:
- `rp2040` (default) - Build for RP2040 (Pico, Pico W)
- `rp235x` - Build for RP235x (Pico 2)

### client (`pico-bitstream`)
Desktop CLI using nusb 0.2:
- Device discovery by VID/PID
- Control requests for configuration
- Bulk OUT streaming with backpressure (8 pending transfers)
- Status display on stderr (~10 Hz updates)
- Ctrl-C and EOF handling

## Building & Flashing

### Firmware for RP2040 (default)
```bash
# Build and flash with probe-rs, streams defmt logs
cargo run --release -p pico-bitstream-firmware

# Or just build
cargo build --release -p pico-bitstream-firmware
```

### Firmware for RP235x (Pico 2)
```bash
cargo run --release -p pico-bitstream-firmware \
  --target thumbv8m.main-none-eabihf \
  --no-default-features --features rp235x
```

### Client
```bash
cargo build --release -p pico-bitstream
```

## Usage

```bash
# Stream 1MHz bitstream
echo -ne '\xAA\xAA\xAA\xAA' | ./target/release/pico-bitstream -s 1000000

# Continuous streaming from file
cat data.bin | ./target/release/pico-bitstream -s 100000

# Test 500Hz square wave (0xAA = 10101010 at 1kHz = 500Hz fundamental)
echo -ne '\xAA\xAA' | ./target/release/pico-bitstream -s 1000
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
- System clock: 125 MHz (RP2040), 150 MHz (RP235x default)
- Sample rate: clock_divider = system_clock / sample_rate

## Target Differences

| | RP2040 | RP235x |
|---|--------|--------|
| Target | thumbv6m-none-eabi | thumbv8m.main-none-eabihf |
| Feature | rp2040 | rp235x |
| RAM | 256K | 512K |
| Boards | Pico, Pico W | Pico 2 |

## Development Notes

### Reference Examples
Embassy examples are at `/home/fletcher/RustroverProjects/embassy/examples/`:
- `rp/` - RP2040 examples (usb_raw_bulk.rs, pio_dma.rs useful)
- `rp235x/` - RP235x examples

nusb examples at `/home/fletcher/RustroverProjects/nusb/examples/`:
- `bulk.rs`, `control.rs`, `bulk_io.rs` - USB transfer patterns

### Key Implementation Details

**Firmware architecture:**
- `DeviceState` in `Mutex<CriticalSectionRawMutex, RefCell<...>>` for shared state
- `ControlHandler` implements `embassy_usb::Handler` for vendor requests (sync callbacks)
- `bulk_handler` async task: USB bulk OUT → ring buffer → DMA → PIO

**PIO configuration:**
- Program: `set pindirs, 1` (runs once) then `out pins, 1` in wrap loop
- `ShiftConfig`: auto_fill=true, threshold=32, direction=Left (MSB first)
- `FifoJoin::TxOnly` doubles TX FIFO to 8 words
- Clock divider: `(clk_sys_freq() / sample_rate).to_fixed()` - uses actual system clock

**Client nusb patterns:**
- `interface.endpoint::<Bulk, Out>(addr)?.writer(buf_size).with_num_transfers(n)`
- Control: `interface.control_out(ControlOut{...}, timeout).wait()?`
- `MaybeFuture` trait provides `.wait()` for blocking

### Build System

**build.rs responsibilities:**
1. Copy correct memory.x based on `CARGO_FEATURE_RP235X`
2. Set linker search path
3. Add linker args: `--nmagic`, `-Tlink.x`, `-Tlink-rp.x` (RP2040 only), `-Tdefmt.x`

**Memory layout differences:**
- RP2040: BOOT2 section at 0x10000000, then FLASH
- RP235x: start_block/end_block sections, bi_entries for picotool

### Common Issues

**USB permission denied on Linux (error code 13):**
- By default, USB devices are owned by root
- Create udev rule: `/etc/udev/rules.d/99-pico-bitstream.rules`
```
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0001", MODE="0666", GROUP="plugdev"
```
- Reload rules: `sudo udevadm control --reload-rules && sudo udevadm trigger`

**Cargo runner not applied for RP235x target:**
- Target triples with dots (like `thumbv8m.main-none-eabihf`) must NOT be quoted in `.cargo/config.toml`
- Wrong: `[target.'thumbv8m.main-none-eabihf']` or `[target."thumbv8m.main-none-eabihf"]`
- Correct: `[target.thumbv8m.main-none-eabihf]` (no quotes, TOML nested table)
- Cargo splits target triples at dots during config lookup, so quoted keys don't match

**Linker errors about undefined symbols (`__bi_entries_start`, etc.):**
- Ensure build.rs includes proper `-Tlink.x` and memory.x has required sections

**critical-section conflicts:**
- Use `embassy-rp/critical-section-impl`, NOT `cortex-m/critical-section-single-core`
- Use `cortex-m` with `inline-asm` feature only

**USB not enumerating:**
- Check VID/PID match between firmware and client
- Ensure `Builder::new()` has all required buffers (config, bos, msos, control)

### Dependencies (firmware)
- embassy-executor 0.9, embassy-rp 0.9, embassy-usb 0.5
- defmt 1.x, defmt-rtt 1.x for logging
- fixed for clock divider math

## Testing

### test-firmware.sh

A test script to: 
- flash and run the firmware while capturing debug logs in the background. 
- run the client and send it a block of data.
- when the send finishes, kill the probe-rs process. 
- save the logs from the firmware and client (both stdout and stderr) to separate files in /tmp and print their locations. 
Use this script any time you want to run the entire system end to end to test or capture logs.