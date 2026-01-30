#![no_std]

/// USB Vendor ID (pid.codes test VID)
pub const USB_VID: u16 = 0x1209;

/// USB Product ID
pub const USB_PID: u16 = 0x0001;

/// Bulk OUT endpoint address (host to device)
pub const EP_BULK_OUT: u8 = 0x01;

/// Bulk IN endpoint address (device to host, for status)
pub const EP_BULK_IN: u8 = 0x81;

/// Vendor control request: Set sample rate (value = rate in Hz, as u32 in data)
pub const REQ_SET_SAMPLE_RATE: u8 = 0x01;

/// Vendor control request: Start streaming
pub const REQ_START: u8 = 0x02;

/// Vendor control request: Stop streaming immediately
pub const REQ_STOP: u8 = 0x03;

/// Vendor control request: Drain buffer then stop (for clean EOF)
pub const REQ_DRAIN_AND_STOP: u8 = 0x04;

/// Vendor control request: Get status (returns BufferStatus)
pub const REQ_GET_STATUS: u8 = 0x10;

/// Ring buffer size in bytes (32KB)
pub const RING_BUFFER_SIZE: usize = 32 * 1024;

/// GPIO pin for bitstream output
pub const OUTPUT_GPIO: u8 = 0;

/// System clock frequency in Hz
pub const SYSTEM_CLOCK_HZ: u32 = 125_000_000;

/// USB bulk transfer size
pub const BULK_PACKET_SIZE: usize = 64;

/// Status information returned by GET_STATUS request
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct BufferStatus {
    /// Number of bytes currently in the buffer
    pub bytes_used: u32,
    /// Total buffer capacity
    pub buffer_size: u32,
    /// Number of underrun events since last start
    pub underrun_count: u32,
    /// Whether streaming is currently active
    pub is_running: u8,
    /// Padding for alignment
    pub _reserved: [u8; 3],
}

impl BufferStatus {
    pub const SIZE: usize = 16;

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..4].copy_from_slice(&self.bytes_used.to_le_bytes());
        buf[4..8].copy_from_slice(&self.buffer_size.to_le_bytes());
        buf[8..12].copy_from_slice(&self.underrun_count.to_le_bytes());
        buf[12] = self.is_running;
        buf
    }

    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            bytes_used: u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
            buffer_size: u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
            underrun_count: u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
            is_running: bytes[12],
            _reserved: [0; 3],
        })
    }

    pub fn fill_percentage(&self) -> u8 {
        if self.buffer_size == 0 {
            return 0;
        }
        ((self.bytes_used as u64 * 100) / self.buffer_size as u64) as u8
    }
}
