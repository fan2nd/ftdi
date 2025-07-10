use crate::ftdaye::FtdiError;
use crate::mpsse_cmd::{ClockBytes, ClockBytesIn, ClockBytesOut, MpsseCmdBuilder};
use crate::{FtdiMpsse, Pin, PinUse};
use eh1::spi::{Error, ErrorKind, ErrorType, SpiBus};
use std::sync::{Arc, Mutex};

// Spi only support mode0 and mode2
// TDI(AD1) can only can output on second edge.
// TDO(AD2) can only can sample on first edge.
// according to AN108-2.2.
// https://ftdichip.com/Support/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
#[derive(Debug, Clone, Copy)]
pub enum SpiMode {
    MsbMode0,
    LsbMode0,
    MsbMode2,
    LsbMode2,
}
#[derive(Debug, Clone, Copy)]
struct SpiCommond {
    write_read: ClockBytes,
    read: ClockBytesIn,
    write: ClockBytesOut,
}
const MSB_MODE0: SpiCommond = SpiCommond {
    write_read: ClockBytes::Tck0Msb,
    read: ClockBytesIn::Tck0Msb,
    write: ClockBytesOut::Tck0Msb,
};
const LSB_MODE0: SpiCommond = SpiCommond {
    write_read: ClockBytes::Tck0Lsb,
    read: ClockBytesIn::Tck0Lsb,
    write: ClockBytesOut::Tck0Lsb,
};
const MSB_MODE2: SpiCommond = SpiCommond {
    write_read: ClockBytes::Tck1Msb,
    read: ClockBytesIn::Tck1Msb,
    write: ClockBytesOut::Tck1Msb,
};
const LSB_MODE2: SpiCommond = SpiCommond {
    write_read: ClockBytes::Tck1Lsb,
    read: ClockBytesIn::Tck1Lsb,
    write: ClockBytesOut::Tck1Lsb,
};
impl From<SpiMode> for SpiCommond {
    fn from(value: SpiMode) -> Self {
        match value {
            SpiMode::MsbMode0 => MSB_MODE0,
            SpiMode::LsbMode0 => LSB_MODE0,
            SpiMode::MsbMode2 => MSB_MODE2,
            SpiMode::LsbMode2 => LSB_MODE2,
        }
    }
}

/// FTDI SPI bus.
///
/// In embedded-hal version 1 this represents an exclusive SPI bus.
pub struct Spi {
    /// Parent FTDI device.
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// SPI polarity
    cmd: SpiCommond,
}

impl Drop for Spi {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().unwrap();
        lock.free_pin(Pin::Lower(0));
        lock.free_pin(Pin::Lower(1));
        lock.free_pin(Pin::Lower(2));
    }
}

impl Spi {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<Spi, FtdiError> {
        {
            log::warn!("Spi module has not been tested yet!");
            let mut lock = mtx.lock().unwrap();
            lock.alloc_pin(Pin::Lower(0), PinUse::Spi);
            lock.alloc_pin(Pin::Lower(1), PinUse::Spi);
            lock.alloc_pin(Pin::Lower(2), PinUse::Spi);

            // default MODE0, SCK(AD0) default 0
            // set SCK(AD0) and MOSI (AD1) as output pins
            lock.lower.direction |= 0x03;

            let mut cmd = MpsseCmdBuilder::new();
            cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
            lock.write_read(cmd.as_slice(), &mut [])?;
        }
        Ok(Spi {
            mtx,
            cmd: MSB_MODE0,
        })
    }
    /// set spi mode and bitorder
    pub fn set_mode(&mut self, mode: SpiMode) -> Result<(), FtdiError> {
        let mut lock = self.mtx.lock().unwrap();
        // set SCK polarity
        match mode {
            SpiMode::MsbMode0 | SpiMode::LsbMode0 => lock.lower.value &= 0xfe, // set SCK(AD0) to 0
            SpiMode::MsbMode2 | SpiMode::LsbMode2 => lock.lower.value |= 0x01, // set SCK(AD0) to 1
        }
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.write_read(cmd.as_slice(), &mut [])?;
        self.cmd = mode.into();
        Ok(())
    }
}

impl Error for FtdiError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl ErrorType for Spi {
    type Error = FtdiError;
}

impl SpiBus<u8> for Spi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_bytes_in(self.cmd.read, words.len());

        let lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), words)?;

        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_bytes_out(self.cmd.write, words);

        let lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), &mut [])?;

        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_bytes(self.cmd.write_read, words);

        let lock = self.mtx.lock().unwrap();

        lock.write_read(cmd.as_slice(), words)?;

        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_bytes(self.cmd.write_read, write);

        let lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), read)?;

        Ok(())
    }
}
