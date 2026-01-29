# pico-bitstream: RP2040 USB GPIO Bitstream Device

## Overview

A simple tool for streaming arbitrary digital signals out of an RP2040 GPIO pin at configurable rates. Consists of:

1. **Firmware** — Runs on RP2040, presents a custom USB device, receives data over bulk endpoint, outputs via PIO
2. **Client** — Desktop CLI tool, configures device, pipes stdin to device with backpressure, shows buffer status on stderr
3. **Shared crate** — Common definitions for USB descriptors, protocol messages, constants

## Hardware

- **Target board:** Raspberry Pi Pico or any RP2040 board
- **Output pin:** GPIO 0 (directly accessible on Pico, directly accessible on Pico)
- **Data format:** Packed bits, MSB first (bit 7 of first byte is first output)

## USB Interface

### Device Identity

| Field | Value |
|-------|-------|
| Vendor ID | `0x1209` (pid.codes) |
| Product ID | `0x0001` (obtain proper sub-pid for real use) |
| Device Class | `0xFF` (Vendor Specific) |
| Product String | `"pico-bitstream"` |

### Endpoints

| Endpoint | Direction | Type | Max Packet Size | Purpose |
|----------|-----------|------|-----------------|---------|
| `0x00` | Both | Control | 64 | Configuration |
| `0x01` | OUT | Bulk | 64 | Data stream |
| `0x81` | IN | Bulk | 64 | Status polling |

### Control Requests (bmRequestType = 0x40 vendor, host-to-device)

| bRequest | wValue | wIndex | Data | Description |
|----------|--------|--------|------|-------------|
| `0x01` | — | — | `u32 LE` | Set sample rate in Hz |
| `0x02` | — | — | — | Start output |
| `0x03` | — | — | — | Stop output |

### Control Requests (bmRequestType = 0xC0 vendor, device-to-host)

| bRequest | wValue | wIndex | Length | Returns | Description |
|----------|--------|--------|--------|---------|-------------|
| `0x10` | — | — | 8 | `BufferStatus` | Get buffer status |

### Data Types

```rust
#[repr(C, packed)]
struct BufferStatus {
    fill_level: u16,   // Current bytes in buffer (LE)
    capacity: u16,     // Total buffer capacity (LE)
    underruns: u32,    // Underrun count since start (LE)
}
```

## Firmware Behavior

### Startup
1. Initialize USB device, wait for enumeration
2. Configure PIO program for serial bit output on GPIO 0
3. Configure DMA in circular mode to feed PIO from ring buffer
4. Output idles LOW until first data received and Start command issued

### Data Flow
1. Bulk OUT transfers are only armed when buffer has space ≥ max packet size
2. Received bytes are pushed into ring buffer
3. DMA continuously drains buffer to PIO TX FIFO
4. PIO shifts out bits MSB first at configured sample rate

### Underrun Handling
- If buffer empties, PIO holds last output state (no glitch to low)
- Underrun counter increments
- Output resumes automatically when buffer refills

### Stop Behavior
- Stop command halts PIO immediately
- Buffer is cleared
- GPIO returns to LOW

## Client Behavior

### Usage

```
pico-bitstream -s <sample_rate_hz>

Options:
  -s, --sample-rate <HZ>    Output sample rate in Hz (required)
  -h, --help                Print help
```

### Operation

1. Find and open USB device by VID/PID
2. Send SetSampleRate control request
3. Send Start control request
4. Loop:
   - Read chunk from stdin (e.g., 4KB at a time)
   - Write to bulk OUT endpoint (blocks when device buffer full)
   - Poll status endpoint periodically, update stderr display
5. On stdin EOF:
   - Continue polling status until buffer drains to zero
   - Send Stop command
   - Exit cleanly
6. On Ctrl-C (SIGINT):
   - Send Stop command immediately
   - Exit

### Status Display (stderr)

Simple single-line updating display:

```
[####------] 42% | 13.2 KB/32 KB | 0 underruns
```

Update ~10 times per second, use `\r` to overwrite line.

## Project Structure

```
pico-bitstream/
├── Cargo.toml              # Workspace definition
├── shared/
│   ├── Cargo.toml
│   └── src/
│       └── lib.rs          # VID, PID, endpoints, messages, BufferStatus
├── firmware/
│   ├── Cargo.toml          # embassy-rp, embassy-usb, etc.
│   ├── build.rs
│   ├── memory.x
│   └── src/
│       └── main.rs         # USB device, PIO program, DMA, ring buffer
└── client/
    ├── Cargo.toml          # rusb/nusb, clap
    └── src/
        └── main.rs         # CLI, USB communication, stdin piping
```

## Dependencies

### Firmware
- `embassy-rp` — RP2040 HAL with async USB, PIO, DMA
- `embassy-usb` — USB device stack
- `embassy-executor` — Async runtime
- `embassy-sync` — Async primitives (channels, mutexes)
- `defmt` + `defmt-rtt` — Logging (optional, for debug)

### Client
- `nusb` — Modern pure-Rust USB library (preferred) OR `rusb` (libusb wrapper)
- `clap` — Argument parsing
- `ctrlc` — Signal handling

### Shared
- `#![no_std]` compatible
- No dependencies (just type definitions and constants)

## PIO Program (pseudocode)

```
; Autopull threshold = 32 (pull word when OSR empty)
; Shift right = false (MSB first)
; Each bit: set pin, delay for sample period

.wrap_target
    out pins, 1      ; Shift 1 bit from OSR to pin
.wrap
```

Clock divider set to achieve desired sample rate:
`div = 125_000_000 / sample_rate`

## Sample Rates

| Sample Rate | Clock Divider | Max Sustainable (USB limited) |
|-------------|---------------|-------------------------------|
| 1 kHz | 125000 | ✓ |
| 100 kHz | 1250 | ✓ |
| 1 MHz | 125 | ✓ |
| 8 MHz | 15.625 | ~✓ (marginal, USB ~1 MB/s = 8 Mbps) |
| 10 MHz+ | <12.5 | ✗ USB can't keep up |

Practical max is around 8 MHz with perfect USB throughput. Buffer smooths short bursts.

## Future Enhancements (out of scope for v1)

- Configurable GPIO pin
- LSB-first option
- One-bit-per-byte mode (easier to generate)
- Multi-pin parallel output
- Trigger/sync input
- Looping mode (repeat buffer)
- Read-back of what was actually output
