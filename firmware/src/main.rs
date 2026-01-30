#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::with as cs_with;

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_futures::select::select;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::Peri;
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::pio::{Config, InterruptHandler as PioInterruptHandler, Pio, ShiftConfig, ShiftDirection, StateMachine};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::EndpointOut;
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler};
use fixed::traits::ToFixed;
use static_cell::StaticCell;

use picostream_shared::*;

use {defmt_rtt as _, panic_probe as _};

// ============================================================================
// CONFIGURATION - Change this to use a different GPIO pin for output
// Note: Also update the `p.PIN_X` reference in main() to match
// ============================================================================
const OUTPUT_PIN: u8 = 0;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

/// Device state shared between control handler and async tasks
struct DeviceState {
    sample_rate: u32,
    is_running: bool,
    is_draining: bool,
    waiting_to_start: bool,
    underrun_count: u32,
    ring_buffer: RingBuffer,
}

impl DeviceState {
    const fn new() -> Self {
        Self {
            sample_rate: 1_000_000,
            is_running: false,
            is_draining: false,
            waiting_to_start: false,
            underrun_count: 0,
            ring_buffer: RingBuffer::new(),
        }
    }
}

/// Simple ring buffer for streaming data
struct RingBuffer {
    buffer: [u8; RING_BUFFER_SIZE],
    read_pos: usize,
    write_pos: usize,
    count: usize,
}

impl RingBuffer {
    const fn new() -> Self {
        Self {
            buffer: [0u8; RING_BUFFER_SIZE],
            read_pos: 0,
            write_pos: 0,
            count: 0,
        }
    }

    fn available_space(&self) -> usize {
        RING_BUFFER_SIZE - self.count
    }

    fn available_data(&self) -> usize {
        self.count
    }

    fn push(&mut self, data: &[u8]) -> usize {
        let to_write = data.len().min(self.available_space());
        for &byte in &data[..to_write] {
            self.buffer[self.write_pos] = byte;
            self.write_pos = (self.write_pos + 1) % RING_BUFFER_SIZE;
        }
        self.count += to_write;
        to_write
    }

    fn pop(&mut self, out: &mut [u8]) -> usize {
        let to_read = out.len().min(self.available_data());
        for byte in &mut out[..to_read] {
            *byte = self.buffer[self.read_pos];
            self.read_pos = (self.read_pos + 1) % RING_BUFFER_SIZE;
        }
        self.count -= to_read;
        to_read
    }

    fn clear(&mut self) {
        self.read_pos = 0;
        self.write_pos = 0;
        self.count = 0;
    }
}

/// Global device state
static STATE: Mutex<CriticalSectionRawMutex, RefCell<DeviceState>> =
    Mutex::new(RefCell::new(DeviceState::new()));

static CONFIG_CHANGED: AtomicBool = AtomicBool::new(false);

/// Signal to wake PIO feeder when new data is available
static DATA_AVAILABLE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signal to wake USB receiver when buffer space is available
static SPACE_AVAILABLE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Control handler for vendor-specific USB requests
struct ControlHandler {
    if_num: InterfaceNumber,
}

impl Handler for ControlHandler {
    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        if req.request_type != RequestType::Vendor {
            return None;
        }
        if req.recipient != Recipient::Interface {
            return None;
        }
        if req.index != self.if_num.0 as u16 {
            return None;
        }

        match req.request {
            REQ_SET_SAMPLE_RATE => {
                if data.len() >= 4 {
                    let rate = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                    info!("Set sample rate: {} Hz", rate);
                    STATE.lock(|s| {
                        s.borrow_mut().sample_rate = rate;
                    });
                    CONFIG_CHANGED.store(true, Ordering::Release);
                    Some(OutResponse::Accepted)
                } else {
                    warn!("SET_SAMPLE_RATE: insufficient data");
                    Some(OutResponse::Rejected)
                }
            }
            REQ_START_WHEN_FULL => {
                info!("Start when full requested");
                STATE.lock(|s| {
                    let mut state = s.borrow_mut();
                    state.waiting_to_start = true;
                    state.is_running = false;
                    state.is_draining = false;
                    state.underrun_count = 0;
                    state.ring_buffer.clear();
                });
                CONFIG_CHANGED.store(true, Ordering::Release);
                // Wake USB receiver (buffer now has space after clear)
                SPACE_AVAILABLE.signal(());
                Some(OutResponse::Accepted)
            }
            REQ_STOP => {
                info!("Stop streaming");
                STATE.lock(|s| {
                    let mut state = s.borrow_mut();
                    state.is_running = false;
                    state.is_draining = false;
                    state.waiting_to_start = false;
                    state.ring_buffer.clear();
                });
                CONFIG_CHANGED.store(true, Ordering::Release);
                // Wake USB receiver (buffer cleared, can accept new data for next start)
                SPACE_AVAILABLE.signal(());
                Some(OutResponse::Accepted)
            }
            REQ_DRAIN_AND_STOP => {
                info!("Drain and stop");
                STATE.lock(|s| {
                    let mut state = s.borrow_mut();
                    // If waiting to start (small transfer case), start now
                    if state.waiting_to_start {
                        info!("Starting PIO (drain triggered)");
                        state.waiting_to_start = false;
                        state.is_running = true;
                    }
                    state.is_draining = true;
                });
                CONFIG_CHANGED.store(true, Ordering::Release);
                Some(OutResponse::Accepted)
            }
            _ => {
                warn!("Unknown vendor OUT request: {}", req.request);
                None
            }
        }
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.request_type != RequestType::Vendor {
            return None;
        }
        if req.recipient != Recipient::Interface {
            return None;
        }
        if req.index != self.if_num.0 as u16 {
            return None;
        }

        match req.request {
            REQ_GET_STATUS => {
                let status = STATE.lock(|s| {
                    let state = s.borrow();
                    BufferStatus {
                        bytes_used: state.ring_buffer.available_data() as u32,
                        buffer_size: RING_BUFFER_SIZE as u32,
                        underrun_count: state.underrun_count,
                        is_running: if state.is_running { 1 } else { 0 },
                        _reserved: [0; 3],
                    }
                });
                let bytes = status.to_bytes();
                let len = bytes.len().min(buf.len());
                buf[..len].copy_from_slice(&bytes[..len]);
                Some(InResponse::Accepted(&buf[..len]))
            }
            _ => {
                warn!("Unknown vendor IN request: {}", req.request);
                None
            }
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("picostream starting (output on GPIO {})", OUTPUT_PIN);

    let p = embassy_rp::init(Default::default());

    // Create USB driver
    let driver = Driver::new(p.USB, Irqs);

    // USB configuration buffers
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut config = embassy_usb::Config::new(USB_VID, USB_PID);
    config.manufacturer = Some("picostream");
    config.product = Some("Bitstream Generator");
    config.serial_number = Some("001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Build USB device
    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Create vendor-specific interface
    let mut function = builder.function(0xFF, 0x00, 0x00);
    let mut interface = function.interface();
    let if_num = interface.interface_number();
    let mut alt = interface.alt_setting(0xFF, 0x00, 0x00, None);
    let mut ep_out = alt.endpoint_bulk_out(None, 64);
    let mut _ep_in = alt.endpoint_bulk_in(None, 64);
    drop(function);

    // Create and register control handler
    static HANDLER: StaticCell<ControlHandler> = StaticCell::new();
    let handler = HANDLER.init(ControlHandler { if_num });
    builder.handler(handler);

    // Build USB device
    let mut usb = builder.build();

    // PIO setup
    let Pio {
        mut common,
        sm0,
        ..
    } = Pio::new(p.PIO0, Irqs);

    // PIO program: output 1 bit at a time
    // set pindirs runs once at startup to configure GPIO as output
    let prg = pio_asm!(
        "set pindirs, 1",
        ".wrap_target",
        "out pins, 1",
        ".wrap"
    );

    let out_pin = common.make_pio_pin(p.PIN_0);

    let mut cfg = Config::default();
    cfg.set_out_pins(&[&out_pin]);
    cfg.set_set_pins(&[&out_pin]);
    cfg.use_program(&common.load_program(&prg.program), &[]);

    // Initial clock divider for default sample rate
    let initial_rate = STATE.lock(|s| s.borrow().sample_rate);
    let sys_freq = clk_sys_freq();
    info!("System clock: {} Hz, sample rate: {} Hz", sys_freq, initial_rate);
    cfg.clock_divider = (sys_freq as f32 / initial_rate as f32).to_fixed();

    // Shift out MSB first, autopull at 32 bits
    cfg.shift_out = ShiftConfig {
        auto_fill: true,
        threshold: 32,
        direction: ShiftDirection::Left,
    };

    cfg.fifo_join = embassy_rp::pio::FifoJoin::TxOnly;

    // Store config in static cell for PIO feeder task
    static SM_CONFIG: StaticCell<Config<'static, PIO0>> = StaticCell::new();
    let sm_config = SM_CONFIG.init(cfg);

    info!("PIO configured");

    // LED on GPIO 25 (active high on Pico/Pico 2)
    let led = Output::new(p.PIN_25, Level::Low);
    info!("LED configured on GPIO 25");

    info!("USB device ready");

    // Run USB device, USB receiver, and PIO feeder concurrently
    let usb_fut = usb.run();
    let usb_receiver_fut = usb_receiver(&mut ep_out);
    let pio_feeder_fut = pio_feeder(sm0, sm_config, p.DMA_CH0, out_pin, led);

    join3(usb_fut, usb_receiver_fut, pio_feeder_fut).await;
}

/// Handle bulk OUT transfers - receive USB data into ring buffer
async fn usb_receiver(ep: &mut impl EndpointOut) {
    let mut buf = [0u8; 64];

    loop {
        // Wait for USB endpoint to be ready
        ep.wait_enabled().await;

        // Wait for buffer space before reading from USB (backpressure)
        loop {
            let space = STATE.lock(|s| s.borrow().ring_buffer.available_space());
            if space >= buf.len() {
                break;
            }
            // Buffer full, wait for PIO feeder to drain some data
            SPACE_AVAILABLE.wait().await;
        }

        // Read bulk data
        match ep.read(&mut buf).await {
            Ok(n) => {
                if n > 0 {
                    // Push data to ring buffer and check if we should start
                    let should_start = STATE.lock(|s| {
                        let mut state = s.borrow_mut();
                        state.ring_buffer.push(&buf[..n]);
                        // Start if waiting and buffer is now full
                        if state.waiting_to_start && state.ring_buffer.available_space() == 0 {
                            state.waiting_to_start = false;
                            state.is_running = true;
                            true
                        } else {
                            false
                        }
                    });
                    if should_start {
                        info!("Starting PIO (buffer full)");
                        CONFIG_CHANGED.store(true, Ordering::Release);
                    }
                    // Signal that new data is available
                    DATA_AVAILABLE.signal(());
                }
            }
            Err(_) => {
                // Endpoint disconnected, continue waiting
                continue;
            }
        }
    }
}

/// Feed PIO from ring buffer using DMA - runs independently of USB reception
async fn pio_feeder(
    mut sm: StateMachine<'static, PIO0, 0>,
    cfg: &'static Config<'static, PIO0>,
    mut dma: Peri<'static, DMA_CH0>,
    out_pin: embassy_rp::pio::Pin<'static, PIO0>,
    mut led: Output<'static>,
) {
    // Apply config and set pin direction
    sm.set_config(cfg);
    sm.set_pin_dirs(embassy_rp::pio::Direction::Out, &[&out_pin]);

    let sys_freq = clk_sys_freq();
    let mut current_sample_rate = STATE.lock(|s| s.borrow().sample_rate);
    let mut pio_running = false;
    let mut dma_buf = [0u32; 16]; // Buffer for DMA transfers

    loop {
        // Check for config changes (sample rate)
        // Use critical section for atomic swap since thumbv6m lacks native swap
        let config_changed = cs_with(|_| {
            let val = CONFIG_CHANGED.load(Ordering::Relaxed);
            if val {
                CONFIG_CHANGED.store(false, Ordering::Relaxed);
            }
            val
        });
        if config_changed {
            let new_rate = STATE.lock(|s| s.borrow().sample_rate);
            if new_rate != current_sample_rate {
                current_sample_rate = new_rate;
                let divider: fixed::FixedU32<fixed::types::extra::U8> =
                    (sys_freq as f32 / current_sample_rate as f32).to_fixed();
                sm.set_clock_divider(divider);
                info!("PIO clock divider updated for {} Hz", current_sample_rate);
            }
        }

        // Check if we need to update running state
        let is_running = STATE.lock(|s| s.borrow().is_running);

        if is_running && !pio_running {
            sm.set_enable(true);
            led.set_high();
            pio_running = true;
            info!("PIO enabled, LED on");
        } else if !is_running && pio_running {
            sm.set_enable(false);
            led.set_low();
            pio_running = false;
            info!("PIO disabled, LED off");
        }

        if !pio_running {
            // Wait for data signal or timeout to check running state
            let _ = select(
                DATA_AVAILABLE.wait(),
                Timer::after(Duration::from_millis(100)),
            ).await;
            continue;
        }

        // Check how much data is available and draining state
        let (available, is_draining) = STATE.lock(|s| {
            let state = s.borrow();
            (state.ring_buffer.available_data(), state.is_draining)
        });

        if available < 4 {
            // Not enough data for even one word
            let (_, tx) = sm.rx_tx();

            if is_draining {
                // Draining mode: wait for TX FIFO to empty, then stop
                if tx.empty() {
                    info!("Drain complete");
                    STATE.lock(|s| {
                        let mut state = s.borrow_mut();
                        state.is_running = false;
                        state.is_draining = false;
                    });
                    CONFIG_CHANGED.store(true, Ordering::Release);
                    SPACE_AVAILABLE.signal(());
                    continue;
                }
                // Still draining TX FIFO, wait a bit
            } else {
                // Normal mode: check for actual underrun (PIO stalled)
                if tx.stalled() {
                    STATE.lock(|s| {
                        s.borrow_mut().underrun_count += 1;
                    });
                    debug!("Underrun detected");
                }
            }

            // Wait for more data (with short timeout to stay responsive)
            let _ = select(
                DATA_AVAILABLE.wait(),
                Timer::after(Duration::from_micros(100)),
            ).await;
            continue;
        }

        // Fill DMA buffer with words from ring buffer
        let mut word_count = 0;
        let mut word_buf = [0u8; 4];

        while word_count < dma_buf.len() {
            let read = STATE.lock(|s| {
                s.borrow_mut().ring_buffer.pop(&mut word_buf)
            });

            if read < 4 {
                break;
            }

            // Convert to u32 (MSB first for our shift direction)
            dma_buf[word_count] = u32::from_be_bytes(word_buf);
            word_count += 1;
        }

        // DMA push to PIO
        if word_count > 0 {
            let (_, tx) = sm.rx_tx();
            tx.dma_push(dma.reborrow(), &dma_buf[..word_count], false).await;
            // Signal that buffer space is now available
            SPACE_AVAILABLE.signal(());
        }
    }
}
