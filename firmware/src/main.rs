#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_futures::select::select;
use embassy_rp::bind_interrupts;
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

use pico_bitstream_shared::*;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

/// Device state shared between control handler and async tasks
struct DeviceState {
    sample_rate: u32,
    is_running: bool,
    underrun_count: u32,
    ring_buffer: RingBuffer,
}

impl DeviceState {
    const fn new() -> Self {
        Self {
            sample_rate: 1_000_000,
            is_running: false,
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
            REQ_START => {
                info!("Start streaming");
                STATE.lock(|s| {
                    let mut state = s.borrow_mut();
                    state.is_running = true;
                    state.underrun_count = 0;
                    state.ring_buffer.clear();
                });
                CONFIG_CHANGED.store(true, Ordering::Release);
                Some(OutResponse::Accepted)
            }
            REQ_STOP => {
                info!("Stop streaming");
                STATE.lock(|s| {
                    s.borrow_mut().is_running = false;
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
    info!("pico-bitstream starting");

    let p = embassy_rp::init(Default::default());

    // Create USB driver
    let driver = Driver::new(p.USB, Irqs);

    // USB configuration buffers
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut config = embassy_usb::Config::new(USB_VID, USB_PID);
    config.manufacturer = Some("pico-bitstream");
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
    cfg.clock_divider = 1500.to_fixed();

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
    info!("USB device ready");

    // Run USB device, USB receiver, and PIO feeder concurrently
    let usb_fut = usb.run();
    let usb_receiver_fut = usb_receiver(&mut ep_out);
    let pio_feeder_fut = pio_feeder(sm0, sm_config, p.DMA_CH0, out_pin);

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
                    // Push data to ring buffer (guaranteed to fit since we checked space)
                    STATE.lock(|s| {
                        s.borrow_mut().ring_buffer.push(&buf[..n]);
                    });
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
) {
    // Apply config and set pin direction
    sm.set_config(cfg);
    sm.set_pin_dirs(embassy_rp::pio::Direction::Out, &[&out_pin]);

    let mut pio_running = false;
    let mut dma_buf = [0u32; 16]; // Buffer for DMA transfers

    loop {
        // Check if we need to update running state
        let is_running = STATE.lock(|s| s.borrow().is_running);

        if is_running && !pio_running {
            sm.set_enable(true);
            pio_running = true;
            info!("PIO enabled");
        } else if !is_running && pio_running {
            sm.set_enable(false);
            pio_running = false;
            info!("PIO disabled");
        }

        if !pio_running {
            // Wait for data signal or timeout to check running state
            let _ = select(
                DATA_AVAILABLE.wait(),
                Timer::after(Duration::from_millis(100)),
            ).await;
            continue;
        }

        // Check how much data is available
        let available = STATE.lock(|s| s.borrow().ring_buffer.available_data());

        if available < 4 {
            // Not enough data for even one word
            // Check for underrun
            let (_, tx) = sm.rx_tx();
            if tx.empty() {
                STATE.lock(|s| {
                    s.borrow_mut().underrun_count += 1;
                });
                debug!("Underrun detected");
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
