use std::fs::File;
use std::io::{self, BufReader, Read, Write};
use std::path::PathBuf;

#[cfg(unix)]
use std::os::unix::io::AsRawFd;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use clap::{Parser, ValueEnum};
use nusb::transfer::{Bulk, ControlIn, ControlOut, ControlType, Out, Recipient};
use nusb::MaybeFuture;

use picostream_shared::*;

#[derive(Debug, Clone, Copy, Default, ValueEnum)]
enum InputFormat {
    /// Packed bytes
    Bytes,
    /// ASCII '0' and '1' characters, whitespace ignored (default)
    #[default]
    Bits,
}

#[derive(Parser, Debug)]
#[command(name = "picostream")]
#[command(about = "Stream digital bitstream to RP2040/RP235x GPIO")]
#[command(after_help = "By default, input is read from stdin.")]
struct Args {
    /// Sample rate in Hz
    #[arg(short = 's', long = "sample-rate")]
    sample_rate: u32,

    /// Input format
    #[arg(short = 'f', long = "format", default_value = "bits")]
    format: InputFormat,

    /// Read input from file
    #[arg(short = 'i', long = "file", conflicts_with = "data")]
    file: Option<PathBuf>,

    /// Use string as input data
    #[arg(short = 'd', long = "data", conflicts_with = "file")]
    data: Option<String>,

    /// Loop input continuously (requires --file or --data)
    #[arg(short = 'l', long = "loop")]
    loop_input: bool,

    /// Skip status display
    #[arg(long)]
    quiet: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    // Validate: --loop requires --file or --data
    if args.loop_input && args.file.is_none() && args.data.is_none() {
        return Err("--loop requires --file or --data".into());
    }

    // Setup Ctrl-C handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
        eprintln!("\nInterrupted, stopping...");
    })?;

    // Find device
    let device = find_device()?;
    eprintln!("Found picostream device");

    let interface = device.claim_interface(0).wait()?;

    // Set sample rate
    set_sample_rate(&interface, args.sample_rate)?;
    eprintln!("Sample rate set to {} Hz", args.sample_rate);

    // Start streaming
    start_streaming(&interface)?;
    eprintln!("Streaming started");

    // Stream data from the appropriate source
    let (result, eof_reached, total_bytes_sent) = if let Some(ref data) = args.data {
        stream_from_data(&interface, running.clone(), !args.quiet, args.format, data, args.loop_input)
    } else if let Some(ref path) = args.file {
        stream_from_file(&interface, running.clone(), !args.quiet, args.format, path, args.loop_input)
    } else {
        stream_from_stdin(&interface, running.clone(), !args.quiet, args.format)
    };

    // Stop streaming - use drain_and_stop for clean EOF, immediate stop for interrupt
    if eof_reached && running.load(Ordering::SeqCst) {
        drain_and_stop(&interface, !args.quiet, total_bytes_sent)?;
    } else {
        stop_streaming(&interface)?;
    }
    eprintln!("\nStreaming stopped");

    // Print final status
    if let Ok(status) = get_status(&interface) {
        eprintln!(
            "Final: {} samples remaining, {} underruns",
            format_samples(status.bytes_used as u64 * 8),
            status.underrun_count
        );
    }

    result
}

fn find_device() -> Result<nusb::Device, Box<dyn std::error::Error>> {
    for dev_info in nusb::list_devices().wait()? {
        if dev_info.vendor_id() == USB_VID && dev_info.product_id() == USB_PID {
            return Ok(dev_info.open().wait()?);
        }
    }
    Err("Device not found (VID=0x1209, PID=0x0001)".into())
}

fn set_sample_rate(
    interface: &nusb::Interface,
    rate: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    let data = rate.to_le_bytes();
    interface
        .control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Interface,
                request: REQ_SET_SAMPLE_RATE,
                value: 0,
                index: 0,
                data: &data,
            },
            Duration::from_secs(1),
        )
        .wait()?;
    Ok(())
}

fn start_streaming(interface: &nusb::Interface) -> Result<(), Box<dyn std::error::Error>> {
    interface
        .control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Interface,
                request: REQ_START_WHEN_FULL,
                value: 0,
                index: 0,
                data: &[],
            },
            Duration::from_secs(1),
        )
        .wait()?;
    Ok(())
}

fn stop_streaming(interface: &nusb::Interface) -> Result<(), Box<dyn std::error::Error>> {
    interface
        .control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Interface,
                request: REQ_STOP,
                value: 0,
                index: 0,
                data: &[],
            },
            Duration::from_secs(1),
        )
        .wait()?;
    Ok(())
}

fn drain_and_stop(
    interface: &nusb::Interface,
    show_status: bool,
    total_sent: u64,
) -> Result<(), Box<dyn std::error::Error>> {
    // Tell firmware to drain buffer then stop
    interface
        .control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Interface,
                request: REQ_DRAIN_AND_STOP,
                value: 0,
                index: 0,
                data: &[],
            },
            Duration::from_secs(1),
        )
        .wait()?;

    // Wait for firmware to finish draining (is_running becomes false)
    loop {
        let status = get_status(interface)?;
        if status.is_running == 0 {
            break;
        }
        if show_status {
            display_status(&status, total_sent);
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    Ok(())
}

fn get_status(interface: &nusb::Interface) -> Result<BufferStatus, Box<dyn std::error::Error>> {
    let data = interface
        .control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Interface,
                request: REQ_GET_STATUS,
                value: 0,
                index: 0,
                length: BufferStatus::SIZE as u16,
            },
            Duration::from_secs(1),
        )
        .wait()?;
    BufferStatus::from_bytes(&data).ok_or("Invalid status response".into())
}

/// Convert input bytes to packed bytes based on format
fn process_input(input: &[u8], format: InputFormat, bit_state: &mut BitState) -> Result<Vec<u8>, String> {
    match format {
        InputFormat::Bytes => Ok(input.to_vec()),
        InputFormat::Bits => {
            let mut packed = Vec::new();
            for &byte in input {
                let ch = byte as char;
                match ch {
                    '0' => {
                        bit_state.accumulator = (bit_state.accumulator << 1) | 0;
                        bit_state.count += 1;
                    }
                    '1' => {
                        bit_state.accumulator = (bit_state.accumulator << 1) | 1;
                        bit_state.count += 1;
                    }
                    ' ' | '\t' | '\n' | '\r' => {}
                    _ => return Err(format!("Invalid character in bits mode: {:?}", ch)),
                }
                if bit_state.count == 8 {
                    packed.push(bit_state.accumulator);
                    bit_state.accumulator = 0;
                    bit_state.count = 0;
                }
            }
            Ok(packed)
        }
    }
}

/// Flush remaining bits (pad with zeros)
fn flush_bits(bit_state: &mut BitState) -> Option<u8> {
    if bit_state.count > 0 {
        let byte = bit_state.accumulator << (8 - bit_state.count);
        bit_state.accumulator = 0;
        bit_state.count = 0;
        Some(byte)
    } else {
        None
    }
}

#[derive(Default)]
struct BitState {
    accumulator: u8,
    count: u8,
}

fn stream_from_stdin(
    interface: &nusb::Interface,
    running: Arc<AtomicBool>,
    show_status: bool,
    format: InputFormat,
) -> (Result<(), Box<dyn std::error::Error>>, bool, u64) {
    let stdin = io::stdin();

    // Set stdin to non-blocking (Unix only)
    #[cfg(unix)]
    unsafe {
        let stdin_fd = stdin.as_raw_fd();
        let flags = libc::fcntl(stdin_fd, libc::F_GETFL);
        libc::fcntl(stdin_fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
    }

    let mut stdin = stdin.lock();

    let mut writer = match interface.endpoint::<Bulk, Out>(EP_BULK_OUT) {
        Ok(ep) => ep.writer(4096).with_num_transfers(8),
        Err(e) => return (Err(e.into()), false, 0),
    };

    let mut read_buf = [0u8; 4096];
    let mut last_status_time = Instant::now();
    let status_interval = Duration::from_millis(100);
    let mut eof_reached = false;
    let mut total_bytes_sent: u64 = 0;
    let mut bit_state = BitState::default();

    while running.load(Ordering::SeqCst) && !eof_reached {
        match stdin.read(&mut read_buf) {
            Ok(0) => {
                eof_reached = true;
                if let Some(byte) = flush_bits(&mut bit_state) {
                    if let Err(e) = writer.write_all(&[byte]) {
                        return (Err(e.into()), eof_reached, total_bytes_sent);
                    }
                    total_bytes_sent += 1;
                }
                eprintln!("\nEOF reached, draining buffer...");
            }
            Ok(n) => {
                match process_input(&read_buf[..n], format, &mut bit_state) {
                    Ok(packed) => {
                        if !packed.is_empty() {
                            if let Err(e) = writer.write_all(&packed) {
                                return (Err(e.into()), eof_reached, total_bytes_sent);
                            }
                            total_bytes_sent += packed.len() as u64;
                        }
                    }
                    Err(e) => return (Err(e.into()), eof_reached, total_bytes_sent),
                }
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                std::thread::sleep(Duration::from_millis(10));
            }
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => {
                eprintln!("\nStdin read error: {}", e);
                break;
            }
        }

        if show_status && last_status_time.elapsed() >= status_interval {
            if let Ok(status) = get_status(interface) {
                display_status(&status, total_bytes_sent);
            }
            last_status_time = Instant::now();
        }
    }

    if running.load(Ordering::SeqCst) {
        if let Err(e) = writer.flush() {
            return (Err(e.into()), eof_reached, total_bytes_sent);
        }
    }

    (Ok(()), eof_reached, total_bytes_sent)
}

fn stream_from_data(
    interface: &nusb::Interface,
    running: Arc<AtomicBool>,
    show_status: bool,
    format: InputFormat,
    data: &str,
    loop_input: bool,
) -> (Result<(), Box<dyn std::error::Error>>, bool, u64) {
    let mut writer = match interface.endpoint::<Bulk, Out>(EP_BULK_OUT) {
        Ok(ep) => ep.writer(4096).with_num_transfers(8),
        Err(e) => return (Err(e.into()), false, 0),
    };

    let mut last_status_time = Instant::now();
    let status_interval = Duration::from_millis(100);
    let mut total_bytes_sent: u64 = 0;

    loop {
        let mut bit_state = BitState::default();

        match process_input(data.as_bytes(), format, &mut bit_state) {
            Ok(packed) => {
                if !packed.is_empty() {
                    if let Err(e) = writer.write_all(&packed) {
                        return (Err(e.into()), !loop_input, total_bytes_sent);
                    }
                    total_bytes_sent += packed.len() as u64;
                }
            }
            Err(e) => return (Err(e.into()), false, total_bytes_sent),
        }

        // Flush remaining bits
        if let Some(byte) = flush_bits(&mut bit_state) {
            if let Err(e) = writer.write_all(&[byte]) {
                return (Err(e.into()), !loop_input, total_bytes_sent);
            }
            total_bytes_sent += 1;
        }

        if !loop_input || !running.load(Ordering::SeqCst) {
            break;
        }

        // Update status display periodically
        if show_status && last_status_time.elapsed() >= status_interval {
            if let Ok(status) = get_status(interface) {
                display_status(&status, total_bytes_sent);
            }
            last_status_time = Instant::now();
        }
    }

    if running.load(Ordering::SeqCst) {
        if let Err(e) = writer.flush() {
            return (Err(e.into()), !loop_input, total_bytes_sent);
        }
        if !loop_input {
            eprintln!("\nData sent, draining buffer...");
        }
    }

    (Ok(()), !loop_input, total_bytes_sent)
}

fn stream_from_file(
    interface: &nusb::Interface,
    running: Arc<AtomicBool>,
    show_status: bool,
    format: InputFormat,
    path: &PathBuf,
    loop_input: bool,
) -> (Result<(), Box<dyn std::error::Error>>, bool, u64) {
    let mut writer = match interface.endpoint::<Bulk, Out>(EP_BULK_OUT) {
        Ok(ep) => ep.writer(4096).with_num_transfers(8),
        Err(e) => return (Err(e.into()), false, 0),
    };

    let mut last_status_time = Instant::now();
    let status_interval = Duration::from_millis(100);
    let mut total_bytes_sent: u64 = 0;
    let mut read_buf = [0u8; 4096];

    loop {
        let file = match File::open(path) {
            Ok(f) => f,
            Err(e) => return (Err(e.into()), false, total_bytes_sent),
        };
        let mut reader = BufReader::new(file);
        let mut bit_state = BitState::default();

        loop {
            if !running.load(Ordering::SeqCst) {
                return (Ok(()), false, total_bytes_sent);
            }

            match reader.read(&mut read_buf) {
                Ok(0) => {
                    // EOF - flush remaining bits
                    if let Some(byte) = flush_bits(&mut bit_state) {
                        if let Err(e) = writer.write_all(&[byte]) {
                            return (Err(e.into()), !loop_input, total_bytes_sent);
                        }
                        total_bytes_sent += 1;
                    }
                    break;
                }
                Ok(n) => {
                    match process_input(&read_buf[..n], format, &mut bit_state) {
                        Ok(packed) => {
                            if !packed.is_empty() {
                                if let Err(e) = writer.write_all(&packed) {
                                    return (Err(e.into()), !loop_input, total_bytes_sent);
                                }
                                total_bytes_sent += packed.len() as u64;
                            }
                        }
                        Err(e) => return (Err(e.into()), false, total_bytes_sent),
                    }
                }
                Err(e) => return (Err(e.into()), false, total_bytes_sent),
            }

            if show_status && last_status_time.elapsed() >= status_interval {
                if let Ok(status) = get_status(interface) {
                    display_status(&status, total_bytes_sent);
                }
                last_status_time = Instant::now();
            }
        }

        if !loop_input {
            break;
        }
    }

    if running.load(Ordering::SeqCst) {
        if let Err(e) = writer.flush() {
            return (Err(e.into()), !loop_input, total_bytes_sent);
        }
        if !loop_input {
            eprintln!("\nFile sent, draining buffer...");
        }
    }

    (Ok(()), !loop_input, total_bytes_sent)
}

fn display_status(status: &BufferStatus, total_sent: u64) {
    let pct = status.fill_percentage();
    let filled = (pct / 10) as usize;
    let empty = 10 - filled;

    let bar: String = "#".repeat(filled) + &"-".repeat(empty);

    let used_samples = status.bytes_used as u64 * 8;
    let total_samples = status.buffer_size as u64 * 8;
    let streamed_bytes = total_sent.saturating_sub(status.bytes_used as u64);
    let streamed_samples = streamed_bytes * 8;

    eprint!(
        "\r[{}] {:3}% | {}/{} samples | {} underruns | streamed: {}   ",
        bar,
        pct,
        format_samples(used_samples),
        format_samples(total_samples),
        status.underrun_count,
        format_samples(streamed_samples),
    );
    let _ = io::stderr().flush();
}

fn format_samples(samples: u64) -> String {
    if samples >= 1_000_000_000 {
        format!("{:.1}G", samples as f64 / 1_000_000_000.0)
    } else if samples >= 1_000_000 {
        format!("{:.1}M", samples as f64 / 1_000_000.0)
    } else if samples >= 1_000 {
        format!("{:.1}K", samples as f64 / 1_000.0)
    } else {
        format!("{}", samples)
    }
}
