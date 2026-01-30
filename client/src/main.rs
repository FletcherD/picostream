use std::io::{self, Read, Write};
use std::os::unix::io::AsRawFd;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use clap::Parser;
use nusb::transfer::{Bulk, ControlIn, ControlOut, ControlType, Out, Recipient};
use nusb::MaybeFuture;

use pico_bitstream_shared::*;

#[derive(Parser, Debug)]
#[command(name = "pico-bitstream")]
#[command(about = "Stream digital bitstream to RP2040 GPIO")]
struct Args {
    /// Sample rate in Hz
    #[arg(short = 's', long = "sample-rate")]
    sample_rate: u32,

    /// Skip status display
    #[arg(long)]
    quiet: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    // Setup Ctrl-C handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
        eprintln!("\nInterrupted, stopping...");
    })?;

    // Find device
    let device = find_device()?;
    eprintln!("Found pico-bitstream device");

    let interface = device.claim_interface(0).wait()?;

    // Set sample rate
    set_sample_rate(&interface, args.sample_rate)?;
    eprintln!("Sample rate set to {} Hz", args.sample_rate);

    // Start streaming
    start_streaming(&interface)?;
    eprintln!("Streaming started");

    // Stream data from stdin
    let (result, eof_reached, total_bytes_sent) =
        stream_data(&interface, running.clone(), !args.quiet);

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
                request: REQ_START,
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

fn stream_data(
    interface: &nusb::Interface,
    running: Arc<AtomicBool>,
    show_status: bool,
) -> (Result<(), Box<dyn std::error::Error>>, bool, u64) {
    let stdin = io::stdin();
    let stdin_fd = stdin.as_raw_fd();

    // Set stdin to non-blocking
    unsafe {
        let flags = libc::fcntl(stdin_fd, libc::F_GETFL);
        libc::fcntl(stdin_fd, libc::F_SETFL, flags | libc::O_NONBLOCK);
    }

    let mut stdin = stdin.lock();

    // Get bulk OUT endpoint with writer interface
    let mut writer = match interface.endpoint::<Bulk, Out>(EP_BULK_OUT) {
        Ok(ep) => ep.writer(4096).with_num_transfers(8),
        Err(e) => return (Err(e.into()), false, 0),
    };

    // Buffer for reading from stdin
    let mut read_buf = [0u8; 4096];

    let mut last_status_time = Instant::now();
    let status_interval = Duration::from_millis(100);

    let mut eof_reached = false;
    let mut total_bytes_sent: u64 = 0;

    while running.load(Ordering::SeqCst) && !eof_reached {
        // Read from stdin (non-blocking)
        match stdin.read(&mut read_buf) {
            Ok(0) => {
                eof_reached = true;
                eprintln!("\nEOF reached, draining buffer...");
            }
            Ok(n) => {
                // Write to USB
                if let Err(e) = writer.write_all(&read_buf[..n]) {
                    return (Err(e.into()), eof_reached, total_bytes_sent);
                }
                total_bytes_sent += n as u64;
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                // No data available, sleep briefly and check running flag
                std::thread::sleep(Duration::from_millis(10));
            }
            Err(e) if e.kind() == io::ErrorKind::Interrupted => {
                continue;
            }
            Err(e) => {
                eprintln!("\nStdin read error: {}", e);
                break;
            }
        }

        // Update status display periodically
        if show_status && last_status_time.elapsed() >= status_interval {
            if let Ok(status) = get_status(interface) {
                display_status(&status, total_bytes_sent);
            }
            last_status_time = Instant::now();
        }
    }

    // Flush remaining data if not interrupted
    if running.load(Ordering::SeqCst) {
        if let Err(e) = writer.flush() {
            return (Err(e.into()), eof_reached, total_bytes_sent);
        }
    }

    (Ok(()), eof_reached, total_bytes_sent)
}

fn display_status(status: &BufferStatus, total_sent: u64) {
    let pct = status.fill_percentage();
    let filled = (pct / 10) as usize;
    let empty = 10 - filled;

    let bar: String = "#".repeat(filled) + &"-".repeat(empty);

    let used_samples = status.bytes_used as u64 * 8;
    let total_samples = status.buffer_size as u64 * 8;
    let sent_samples = total_sent * 8;

    eprint!(
        "\r[{}] {:3}% | {}/{} samples | {} underruns | sent: {}   ",
        bar,
        pct,
        format_samples(used_samples),
        format_samples(total_samples),
        status.underrun_count,
        format_samples(sent_samples),
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
