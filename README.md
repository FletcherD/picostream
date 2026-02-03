# picostream

Stream arbitrary digital bitstreams out of an RP2040/RP235x GPIO pin over USB.

## What it does

picostream turns a Raspberry Pi Pico (or Pico 2) into a USB-controlled digital signal generator. Send bit patterns from your computer and they'll be output on GPIO 0 at your specified sample rate.

**Use cases:**
- Testing digital interfaces (SPI, I2C, custom protocols)
- Generating arbitrary waveforms
- Debugging tool for digital signals

## Hardware

- **Supported boards:** Raspberry Pi Pico (RP2040), Pico 2 (RP2350), or other development boards with one of these chips
- **Output pin:** GPIO 0 (directly accessible on the Pico's pin header)
- **Sample rate:** Up to ~8 MHz 

## Quick Start

### 1. Flash the firmware

Download the appropriate `.uf2` file from [releases](https://github.com/FletcherD/picostream/releases), hold BOOTSEL while plugging in the Pico, and drag the file to the mounted drive.

### 2. Download the client

Get the appropriate client binary for your platform from [releases](https://github.com/FletcherD/picostream/releases).

### 3. Stream some bits

```bash
# Output a simple pattern at 1 MHz
./target/release/picostream -s 1000000 -d "01010101"

# Loop a pattern continuously (Ctrl-C to stop)
./target/release/picostream -s 1000000 -d "11110000" --loop

# Stream from a file
./target/release/picostream -s 500000 -i pattern.txt

# Pipe data from stdin (use -f bytes for binary data)
cat data.bin | ./target/release/picostream -s 100000 -f bytes
```

## Client Usage

```
picostream [OPTIONS] --sample-rate <SAMPLE_RATE>

Options:
  -s, --sample-rate <HZ>   Output sample rate in Hz (required)
  -f, --format <FORMAT>    Input format: bits (default) or bytes
  -i, --file <FILE>        Read input from file
  -d, --data <DATA>        Use string as input data
  -l, --loop               Loop input continuously (requires --file or --data)
      --quiet              Skip status display
  -h, --help               Print help
```

**Input formats:**
- `bits` (default): ASCII `0` and `1` characters. Whitespace is ignored, so you can format for readability.
- `bytes`: Raw packed bytes. Each byte provides 8 output samples, MSB first.

**Input sources (mutually exclusive):**
- stdin (default)
- `--file`: Read from a file
- `--data`: Use a command-line string

## How it works

1. The firmware presents a custom USB device
2. The client sends data over USB bulk transfers into a 32KB ring buffer
3. A PIO state machine shifts out bits at the configured sample rate
4. DMA keeps the PIO fed from the ring buffer
5. Backpressure prevents buffer overruns; underruns are counted and reported

## Building from source

**Prerequisites:**
- Rust toolchain: https://rustup.rs/
- For firmware: appropriate target and [probe-rs](https://probe.rs/) for flashing
  ```bash
  rustup target add thumbv6m-none-eabi        # For RP2040
  rustup target add thumbv8m.main-none-eabihf # For RP235x
  ```

**Build the client:**
```bash
cargo build --release -p picostream
# Binary will be at target/release/picostream
```

**Build and flash firmware (requires debug probe):**
```bash
# For Pico (RP2040):
cargo run --release -p picostream-firmware \
  --target thumbv6m-none-eabi --features rp2040

# For Pico 2 (RP235x):
cargo run --release -p picostream-firmware \
  --target thumbv8m.main-none-eabihf --features rp235x
```

**Build firmware without flashing (to get .uf2 file):**
```bash
cargo install elf2uf2-rs
cargo build --release -p picostream-firmware \
  --target thumbv6m-none-eabi --features rp2040
elf2uf2-rs target/thumbv6m-none-eabi/release/picostream-firmware firmware.uf2
```

## Linux USB permissions

If you get permission errors, install the included udev rules:

```bash
sudo cp 99-picostream.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then unplug and replug the device (or reboot).

## Changing the output pin

The output pin is configured in `firmware/src/main.rs`. Look for the `OUTPUT_PIN` constant near the top of the file and the corresponding `p.PIN_X` reference in `main()`.
