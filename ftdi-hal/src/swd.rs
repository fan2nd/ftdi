use std::{
    ops::{Deref, DerefMut},
    sync::{Arc, Mutex, MutexGuard},
};

use crate::{
    FtMpsse, Pin, PinUse,
    ftdaye::FtdiError,
    mpsse::{ClockBitsIn, ClockBitsOut, ClockBytesIn, ClockBytesOut, MpsseCmdBuilder},
};

/// SCK bitmask
const SCK: u8 = 1 << 0;
/// DIO bitmask
const DIO: u8 = 1 << 1;

struct SwdCmdBuilder(MpsseCmdBuilder);
impl Deref for SwdCmdBuilder {
    type Target = MpsseCmdBuilder;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for SwdCmdBuilder {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl SwdCmdBuilder {
    const BITS_IN: ClockBitsIn = ClockBitsIn::LsbNeg;
    const BITS_OUT: ClockBitsOut = ClockBitsOut::LsbPos;
    const BYTES_IN: ClockBytesIn = ClockBytesIn::LsbNeg;
    const BYTES_OUT: ClockBytesOut = ClockBytesOut::LsbPos;
    pub fn new() -> Self {
        SwdCmdBuilder(MpsseCmdBuilder::new())
    }
    pub fn swd_enable(&mut self, lock: &MutexGuard<FtMpsse>) -> &mut Self {
        const ONES: [u8; 8] = [0xff; 8]; // 64 ones
        const SEQUENCE: [u8; 2] = [0x79, 0xe7]; // Activation pattern

        self.swd_out(lock)
            .clock_bytes_out(ClockBytesOut::LsbPos, &ONES) // >50 ones (LSB first)
            .clock_bytes_out(ClockBytesOut::MsbPos, &SEQUENCE) // Activation pattern (MSB first)
            .clock_bytes_out(ClockBytesOut::LsbPos, &ONES) // >50 ones (LSB first)
            .send_immediate();
        self
    }
    pub fn swd_out(&mut self, lock: &MutexGuard<FtMpsse>) -> &mut Self {
        self.set_gpio_lower(lock.lower.value, lock.lower.direction | SCK | DIO);
        self
    }
    pub fn swd_in(&mut self, lock: &MutexGuard<FtMpsse>) -> &mut Self {
        self.set_gpio_lower(lock.lower.value, lock.lower.direction | SCK);
        self
    }
    pub fn swd_trn(&mut self) -> &mut Self {
        self.clock_bits_out(Self::BITS_OUT, 0xff, 1);
        self
    }
    pub fn swd_send_request(&mut self, request: u8) -> &mut Self {
        self.clock_bytes_out(Self::BYTES_OUT, &[request]); // // Send request
        self
    }
    pub fn swd_read_response(&mut self) -> &mut Self {
        self.clock_bits_in(Self::BITS_IN, 3);
        self
    }
    pub fn swd_read_data(&mut self) -> &mut Self {
        const DATA_BYTES: usize = 4;
        const PARITY_BITS: usize = 1;
        self.clock_bytes_in(Self::BYTES_IN, DATA_BYTES)
            .clock_bits_in(Self::BITS_IN, PARITY_BITS);
        self
    }
    pub fn swd_write_data(&mut self, value: u32) -> &mut Self {
        const PARITY_BITS: usize = 1;
        let bytes = value.to_le_bytes();
        let parity = (value.count_ones() & 0x01) as u8;
        self.clock_bytes_out(Self::BYTES_OUT, &bytes)
            .clock_bits_out(Self::BITS_OUT, parity, PARITY_BITS);
        self
    }
}

#[derive(Debug, Clone, Copy)]
pub enum SwdAddr {
    Dp(u8),
    Ap(u8),
}
impl From<SwdAddr> for u8 {
    fn from(value: SwdAddr) -> Self {
        // Timing Sequence: [Start(1), APnDP, RnW, A[2:3], Parity, Stop(0), Park(1)]
        // LSB Format: [Park(1), Stop(0), Parity, A[3:2], RnW, APnDP, Start(1)]
        const PORT_MASK: u8 = 1 << 1;
        const ADDR_MASK: u8 = 0b11 << 2;
        match value {
            SwdAddr::Dp(addr) => addr << 1 & ADDR_MASK,
            SwdAddr::Ap(addr) => addr << 1 & ADDR_MASK | PORT_MASK,
        }
    }
}
pub struct Swd {
    /// Parent FTDI device.
    mtx: Arc<Mutex<FtMpsse>>,
}
impl Drop for Swd {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.free_pin(Pin::Lower(0));
        lock.free_pin(Pin::Lower(1));
        lock.free_pin(Pin::Lower(2));
    }
}
impl Swd {
    // Swd ACK (3 bits)
    // 0..=2 - 001:失败 010:等待 100:成功
    const REPONSE_SUCCESS: u8 = 0b001;
    const REPONSE_WAIT: u8 = 0b010;
    const REPONSE_FAILED: u8 = 0b100;
    /// Initialize SWD interface
    /// Allocates and configures GPIO pins:
    ///   Pin0 (SCK)        - Output
    ///   Pin1 (DIO_OUTPUT) - Output
    ///   Pin2 (DIO_INPUT)  - Input
    pub fn new(mtx: Arc<Mutex<FtMpsse>>) -> Result<Self, FtdiError> {
        {
            log::warn!("Swd module has not been tested yet!");
            log::warn!("Swd module need connect AD1 and AD2!");
            let mut lock = mtx.lock().expect("Failed to aquire FTDI mutex");
            lock.alloc_pin(Pin::Lower(0), PinUse::Swd);
            lock.alloc_pin(Pin::Lower(1), PinUse::Swd);
            lock.alloc_pin(Pin::Lower(2), PinUse::Swd);
            // clear direction and value of first 3 pins
            // set to input and value 0
            lock.lower.direction &= !0x07;
            lock.lower.value &= !0x07;
            // set GPIO pins to new state
            let mut cmd = MpsseCmdBuilder::new();
            cmd.set_gpio_lower(lock.lower.value, lock.lower.direction)
                .disable_adaptive_data_clocking()
                .disable_loopback()
                .enable_3phase_data_clocking()
                .send_immediate();
            lock.write_read(cmd.as_slice(), &mut [])?;
        }
        Ok(Self { mtx })
    }
    /// Send SWD activation sequence
    /// Sequence: >50 ones + 0x79E7 (MSB first) + >50 ones
    pub fn enable(&self) -> Result<(), FtdiError> {
        let lock = self.mtx.lock().expect("Failed to acquire FTDI mutex");
        let mut cmd = SwdCmdBuilder::new();
        cmd.swd_enable(&lock);

        lock.write_read(cmd.as_slice(), &mut [])?;
        Ok(())
    }
    /// Build SWD request packet (lsb 8 bits)
    /// Timing Sequence: [Start(1), APnDP, RnW, A[2:3], Parity, Stop(0), Park(1)]
    /// LSB Format: [Park(1), Stop(0), Parity, A[3:2], RnW, APnDP, Start(1)]
    fn build_request(is_read: bool, addr: SwdAddr) -> u8 {
        const START_MASK: u8 = 1 << 0;
        const PARK_MASK: u8 = 1 << 7;
        const READ_MASK: u8 = 1 << 2;
        let mut request = START_MASK | PARK_MASK; // Start(1) + Park(1) with Stop(0)
        request |= if is_read { READ_MASK } else { 0 }; // Set RnW bit (position 2)
        request |= u8::from(addr);

        // The parity check is made over the APnDP, RnW and A[2:3] bits. If, of these four bits:
        // • the number of bits set to 1 is odd, then the parity bit is set to 1
        // • the number of bits set to 1 is even, then the parity bit is set to 0.
        let parity = ((request >> 1) & 0x0F).count_ones() as u8 & 0x01;
        request |= parity << 5; // Set parity bit (position 5)

        request
    }
    /// Perform SWD read operation
    pub fn read(&self, addr: SwdAddr) -> Result<u32, FtdiError> {
        let request = Self::build_request(true, addr);
        let response: &mut [u8] = &mut [0];
        let lock = self.mtx.lock().expect("Failed to acquire FTDI mutex");
        // Send request (8 bits)
        let mut cmd = SwdCmdBuilder::new();
        cmd.swd_out(&lock)
            .swd_send_request(request)
            .swd_in(&lock)
            .swd_trn()
            .swd_read_response()
            .send_immediate();
        lock.write_read(cmd.as_slice(), response)?;

        // Read ACK (3 bits)
        // 0..=2 - 001:失败 010:等待 100:成功
        let ack = response[0] >> 5;
        if ack != Self::REPONSE_SUCCESS {
            let mut cmd = SwdCmdBuilder::new();
            cmd.swd_trn().send_immediate();
            lock.write_read(cmd.as_slice(), &mut [])?;
            match ack {
                Self::REPONSE_WAIT => return Err(FtdiError::Other("Swd ack wait".into())),
                Self::REPONSE_FAILED => return Err(FtdiError::Other("Swd ack fail".into())),
                x => return Err(FtdiError::Other(format!("Unknown ack {x:3b}"))),
            }
        }

        // Read data (32 bits) + parity (1 bit) = 33 bits
        let mut data = [0u8; 5]; // 33 bits = 5 bytes
        let mut cmd = SwdCmdBuilder::new();
        cmd.swd_read_data().swd_trn().send_immediate();
        lock.write_read(cmd.as_slice(), &mut data)?;

        // Parse the data (LSB first)
        let value = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let parity = (data[4] >> 7) & 0x01;
        let calc_parity = value.count_ones() as u8 & 0x01;

        if parity != calc_parity {
            return Err(FtdiError::Other("Swd data parity error".to_string()));
        }
        Ok(value)
    }

    pub fn write(&self, addr: SwdAddr, value: u32) -> Result<(), FtdiError> {
        let request = Self::build_request(false, addr);
        let response: &mut [u8] = &mut [0];
        let lock = self.mtx.lock().expect("Failed to acquire FTDI mutex");
        let mut cmd = SwdCmdBuilder::new();
        cmd.swd_out(&lock)
            .swd_send_request(request)
            .swd_in(&lock)
            .swd_trn()
            .swd_read_response()
            .swd_trn()
            .send_immediate();
        lock.write_read(cmd.as_slice(), response)?;

        // Read ACK (3 bits)
        // 0..=2 - 001:失败 010:等待 100:成功
        let ack = response[0] >> 5;
        if ack != Self::REPONSE_SUCCESS {
            match ack {
                Self::REPONSE_WAIT => return Err(FtdiError::Other("Swd ack wait".into())),
                Self::REPONSE_FAILED => return Err(FtdiError::Other("Swd ack fail".into())),
                x => return Err(FtdiError::Other(format!("Unknown ack {x:3b}"))),
            }
        }
        // Send data (33 bits)
        let mut cmd = SwdCmdBuilder::new();
        cmd.swd_out(&lock).swd_write_data(value).send_immediate();
        lock.write_read(cmd.as_slice(), &mut [])?;
        Ok(())
    }
}
