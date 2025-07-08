use std::sync::{Arc, Mutex};

use crate::{
    FtMpsse, Pin, PinUse,
    ftdaye::FtdiError,
    mpsse::{ClockBitsIn, ClockBitsOut, ClockBytesIn, ClockBytesOut, MpsseCmdBuilder},
};

/// SCK bitmask
const SCK: u8 = 1 << 0;
/// DIO bitmask
const DIO: u8 = 1 << 1;

#[derive(Debug, Clone, Copy)]
pub enum SwdPort {
    Dp = 0,
    Ap = 1,
}
#[derive(Debug, Clone, Copy)]
pub enum SwdOp {
    Read = 1,
    Write = 0,
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
    /// Initialize SWD interface
    /// Allocates and configures GPIO pins:
    ///   Pin0 (SCK)        - Output
    ///   Pin1 (DIO_OUTPUT) - Output
    ///   Pin2 (DIO_INPUT)  - Input
    pub fn new(mtx: Arc<Mutex<FtMpsse>>) -> Result<Self, FtdiError> {
        {
            log::warn!("Swd module has not been tested yet!");
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
        const ONES: [u8; 8] = [0xff; 8]; // 64 ones
        const SEQUENCE: [u8; 2] = [0x79, 0xe7]; // Activation pattern

        let lock = self.mtx.lock().expect("Failed to acquire FTDI mutex");
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction | SCK | DIO)
            .clock_data_out(ClockBytesOut::LsbPos, &ONES) // >50 ones (LSB first)
            .clock_data_out(ClockBytesOut::MsbPos, &SEQUENCE) // Activation pattern (MSB first)
            .clock_data_out(ClockBytesOut::LsbPos, &ONES) // >50 ones (LSB first)
            .send_immediate();

        lock.write_read(cmd.as_slice(), &mut [])?;
        Ok(())
    }
    /// Build SWD request packet (lsb 8 bits)
    /// Format: low[Start(1), APnDP, RnW, A[2:3], Parity, Stop(0), Park(1)]high
    fn build_request(port: SwdPort, op: SwdOp, addr: u8) -> u8 {
        let addr = (addr >> 2) & 0x03; // Extract A[3:2]
        let mut request = 0x81; // Start(1) + Park(1) with Stop(0)

        request |= (port as u8) << 1; // Set APnDP bit (position 1)
        request |= (op as u8) << 2; // Set RnW bit (position 2)
        request |= addr << 3; // Set address bits (positions 3-4)

        // The parity check is made over the APnDP, RnW and A[2:3] bits. If, of these four bits:
        // • the number of bits set to 1 is odd, then the parity bit is set to 1
        // • the number of bits set to 1 is even, then the parity bit is set to 0.
        let parity = ((request >> 1) & 0x0F).count_ones() as u8 & 1;
        request |= parity << 5; // Set parity bit (position 5)

        request
    }
    /// Perform SWD read operation
    pub fn read(&self, port: SwdPort, addr: u8) -> Result<u32, FtdiError> {
        let request = Self::build_request(port, SwdOp::Read, addr);
        let response: &mut [u8] = &mut [0];
        let lock = self.mtx.lock().expect("Failed to acquire FTDI mutex");
        // Send request (8 bits)
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction | SCK | DIO) // DIO as output
            .clock_data_out(ClockBytesOut::LsbPos, &[request]) // // Send request
            .set_gpio_lower(lock.lower.value, lock.lower.direction | SCK) // DIO as input
            .clock_bits_out(ClockBitsOut::LsbPos, 0xff, 1) // TRN cycle Output2Input
            .clock_bits_in(ClockBitsIn::LsbNeg, 3) // Read ACK (3 bits)
            .send_immediate();
        lock.write_read(cmd.as_slice(), response)?;

        // Read ACK (3 bits)
        // 0..2	- 001:失败 010:等待 100:成功
        let ack = response[0] >> 5;
        if ack != 0b001 {
            let mut cmd = MpsseCmdBuilder::new();
            cmd.clock_bits_out(ClockBitsOut::LsbPos, 0xff, 1) // TRN cycle Input2Output
                .send_immediate();
            lock.write_read(cmd.as_slice(), &mut [])?;
            match ack {
                0b010 => return Err(FtdiError::Other("Swd ack wait".into())),
                0b100 => return Err(FtdiError::Other("Swd ack fail".into())),
                x => return Err(FtdiError::Other(format!("Unknown ack {x:3b}"))),
            }
        }

        // Read data (32 bits) + parity (1 bit) = 33 bits
        let mut data = [0u8; 5]; // 33 bits = 5 bytes
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_data_in(ClockBytesIn::LsbNeg, 4) // 32-bit data
            .clock_bits_in(ClockBitsIn::LsbNeg, 1) // 1-bit parity
            .clock_bits_out(ClockBitsOut::LsbPos, 0xff, 1) // TRN cycle Input2Output
            .send_immediate();
        lock.write_read(cmd.as_slice(), &mut data)?;

        // Parse the data (LSB first)
        let value = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let parity = (data[4] >> 7) & 0x01;
        let calc_parity = value.count_ones() as u8 & 1;

        if parity != calc_parity {
            return Err(FtdiError::Other("Swd data parity error".to_string()));
        }
        Ok(value)
    }

    pub fn write(&self, port: SwdPort, addr: u8, value: u32) -> Result<(), FtdiError> {
        let request = Self::build_request(port, SwdOp::Write, addr);
        let response: &mut [u8] = &mut [0];
        let lock = self.mtx.lock().expect("Failed to acquire FTDI mutex");
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction | SCK | DIO) // DIO as output
            .clock_data_out(ClockBytesOut::LsbPos, &[request]) // Send request
            .set_gpio_lower(lock.lower.value, lock.lower.direction | SCK) // DIO as input
            .clock_bits_out(ClockBitsOut::LsbPos, 0xff, 1) // TRN cycle Output2Input
            .clock_bits_in(ClockBitsIn::LsbNeg, 3) // Read ACK (3 bits)
            .clock_bits_out(ClockBitsOut::LsbPos, 0xff, 1) // TRN cycle Input2Output
            .send_immediate();
        lock.write_read(cmd.as_slice(), response)?;

        // Read ACK (3 bits)
        // 0..2	- 001:失败 010:等待 100:成功
        let ack = response[0] >> 5;
        if ack != 0b001 {
            match ack {
                0b010 => return Err(FtdiError::Other("Swd ack wait".into())),
                0b100 => return Err(FtdiError::Other("Swd ack fail".into())),
                x => return Err(FtdiError::Other(format!("Unknown ack {x:3b}"))),
            }
        }

        // Prepare data: 32 bits value + 1 parity bit
        let data = value.to_le_bytes();
        let parity = value.count_ones() as u8 & 1;

        // Send data (33 bits)
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction | SCK | DIO) // DIO as output
            .clock_data_out(ClockBytesOut::LsbPos, &data)
            .clock_bits_out(ClockBitsOut::LsbPos, parity, 1)
            .send_immediate();
        lock.write_read(cmd.as_slice(), &mut [])?;
        Ok(())
    }
}
