use std::time::Duration;

use embedded_hal::spi::{
    ErrorKind, ErrorType, Mode, Operation, Polarity, SpiDevice, MODE_0, MODE_1, MODE_2, MODE_3,
};

use crate::{command::Command, error::FtdiError, interface::FtdiInterface, request::BitMode};

pub struct FtdiSpi {
    interface: FtdiInterface,
    mode: Mode,

    bytes_should_read: usize,
    read_buffer: Vec<u8>,
}
impl FtdiSpi {
    const DIRECTION: u8 = 0x0b;
    const IDLE_HIGH: u8 = 0x09;
    const IDLE_LOW: u8 = 0x08;

    pub fn new(mut interface: FtdiInterface, mode: Mode) -> Result<Self, FtdiError> {
        if interface.bitmode != BitMode::Mpsse {
            interface.set_bitmode(BitMode::Mpsse)?;
            interface.reset_rx()?;
            interface.reset_tx()?;
        }
        match mode.polarity {
            Polarity::IdleHigh => interface.schedule_write(&[
                Command::SetBitsLow.into(),
                Self::IDLE_HIGH,
                Self::DIRECTION,
            ]),
            Polarity::IdleLow => interface.schedule_write(&[
                Command::SetBitsLow.into(),
                Self::IDLE_LOW,
                Self::DIRECTION,
            ]),
        }
        interface.common_setting();
        interface.read_result()?;
        Ok(FtdiSpi {
            interface,
            mode,
            read_buffer: Vec::new(),
            bytes_should_read: 0,
        })
    }
}

impl FtdiSpi {
    pub const LSB_BYTES_OUT_P: u8 = 0x18;
    pub const LSB_BYTES_OUT_N: u8 = 0x19;
    pub const LSB_BYTES_IN_P: u8 = 0x28;
    pub const LSB_BYTES_IN_N: u8 = 0x2c;
    pub const LSB_BYTES_OUT_N_IN_P: u8 = 0x39;
    pub const LSB_BYTES_OUT_P_IN_N: u8 = 0x3c;
    fn chip_select(&mut self, en: bool) -> Result<(), FtdiError> {
        let value = (en as u8) * 0x08;
        match self.mode.polarity {
            Polarity::IdleHigh => self.interface.schedule_write(&[
                Command::SetBitsLow.into(),
                value + 1,
                Self::DIRECTION,
            ]),
            Polarity::IdleLow => {
                self.interface
                    .schedule_write(&[Command::SetBitsLow.into(), value, Self::DIRECTION])
            }
        }
        Ok(())
    }
    fn schedule_write(&mut self, data: &[u8]) -> Result<(), FtdiError> {
        debug_assert!(data.len() <= u16::MAX as usize);
        let mut cmd = vec![0; data.len() + 3];
        cmd[0] = match self.mode {
            MODE_0 => Self::LSB_BYTES_OUT_N,
            MODE_1 => Self::LSB_BYTES_OUT_P,
            MODE_2 => Self::LSB_BYTES_OUT_P,
            MODE_3 => Self::LSB_BYTES_OUT_N,
        };
        cmd[1] = (data.len() - 1) as u8;
        cmd[2] = ((data.len() - 1) >> 8) as u8;
        cmd[3..].copy_from_slice(data);
        self.interface.schedule_write(&cmd);
        Ok(())
    }
    fn schedule_read(&mut self, data: &[u8]) -> Result<(), FtdiError> {
        let mut cmd = [0; 3];
        cmd[0] = match self.mode {
            MODE_0 => Self::LSB_BYTES_IN_P,
            MODE_1 => Self::LSB_BYTES_IN_N,
            MODE_2 => Self::LSB_BYTES_IN_N,
            MODE_3 => Self::LSB_BYTES_IN_P,
        };
        cmd[1] = (data.len() - 1) as u8;
        cmd[2] = ((data.len() - 1) >> 8) as u8;
        self.interface.schedule_write(&cmd);
        self.bytes_should_read += data.len();
        Ok(())
    }
    fn schedule_write_read(&mut self, data: &[u8]) -> Result<(), FtdiError> {
        debug_assert!(data.len() <= u16::MAX as usize);
        let mut cmd = vec![0; data.len() + 3];
        cmd[0] = match self.mode {
            MODE_0 => Self::LSB_BYTES_OUT_N_IN_P,
            MODE_1 => Self::LSB_BYTES_OUT_P_IN_N,
            MODE_2 => Self::LSB_BYTES_OUT_P_IN_N,
            MODE_3 => Self::LSB_BYTES_OUT_N_IN_P,
        };
        cmd[1] = (data.len() - 1) as u8;
        cmd[2] = ((data.len() - 1) >> 8) as u8;
        cmd[3..].copy_from_slice(data);
        self.interface.schedule_write(&cmd);
        self.bytes_should_read += data.len();
        Ok(())
    }
    fn flush(&mut self) -> Result<(), FtdiError> {
        let result = self.interface.read_result()?;
        self.read_buffer.extend(result);
        Ok(())
    }
}

impl From<FtdiError> for ErrorKind {
    fn from(_value: FtdiError) -> Self {
        ErrorKind::Other
    }
}

impl ErrorType for FtdiSpi {
    type Error = ErrorKind;
}

impl SpiDevice for FtdiSpi {
    fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.chip_select(true)?;
        for op in operations.iter() {
            match op {
                Operation::Write(write) => self.schedule_write(write)?,
                Operation::Read(read) => self.schedule_read(read)?,
                Operation::Transfer(_, write) => self.schedule_write_read(write)?,
                Operation::TransferInPlace(write_read) => self.schedule_write_read(write_read)?,
                Operation::DelayNs(ns) => {
                    self.flush()?;
                    std::thread::sleep(Duration::from_nanos(*ns as u64));
                }
            }
        }
        self.chip_select(false)?;
        self.flush()?;
        debug_assert_eq!(self.read_buffer.len(), self.bytes_should_read);
        let mut result: &[u8] = &self.read_buffer;
        for op in operations.iter_mut() {
            match op {
                Operation::Read(write) => {
                    write.copy_from_slice(&result[..write.len()]);
                    result = &result[write.len()..]
                }
                Operation::Transfer(read, _) => {
                    read.copy_from_slice(&result[..read.len()]);
                    result = &result[read.len()..]
                }
                Operation::TransferInPlace(write_read) => {
                    write_read.copy_from_slice(&result[..write_read.len()]);
                    result = &result[write_read.len()..]
                }
                _ => continue,
            }
        }
        Ok(())
    }
}
