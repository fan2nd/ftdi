use crate::ftdaye::FtdiError;
use crate::mpsse::{ClockData, ClockDataIn, ClockDataOut, MpsseCmdBuilder};
use crate::{FtMpsse, Pin, PinUse};
use eh1::spi::{Error, ErrorKind, ErrorType, SpiBus};
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Copy)]
pub enum SpiMode {
    MsbMode0,
    LsbMode0,
    MsbMode2,
    LsbMode2,
}
#[derive(Debug, Clone, Copy)]
struct SpiCommond {
    write_read: ClockData,
    read: ClockDataIn,
    write: ClockDataOut,
}
const MSB_MODE0: SpiCommond = SpiCommond {
    write_read: ClockData::MsbPosIn,
    read: ClockDataIn::MsbPos,
    write: ClockDataOut::MsbNeg,
};
const LSB_MODE0: SpiCommond = SpiCommond {
    write_read: ClockData::LsbPosIn,
    read: ClockDataIn::LsbPos,
    write: ClockDataOut::LsbNeg,
};
const MSB_MODE2: SpiCommond = SpiCommond {
    write_read: ClockData::MsbNegIn,
    read: ClockDataIn::MsbNeg,
    write: ClockDataOut::MsbPos,
};
const LSB_MODE2: SpiCommond = SpiCommond {
    write_read: ClockData::LsbNegIn,
    read: ClockDataIn::LsbNeg,
    write: ClockDataOut::LsbPos,
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
    mtx: Arc<Mutex<FtMpsse>>,
    /// SPI polarity
    cmd: SpiCommond,
}

impl Drop for Spi {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.free_pin(Pin::Lower(0));
        lock.free_pin(Pin::Lower(1));
        lock.free_pin(Pin::Lower(2));
    }
}

impl Spi {
    pub fn new(mtx: Arc<Mutex<FtMpsse>>) -> Result<Spi, FtdiError> {
        {
            log::warn!("Spi module has not been tested yet!");
            let mut lock = mtx.lock().expect("Failed to aquire FTDI mutex");
            lock.alloc_pin(Pin::Lower(0), PinUse::Spi);
            lock.alloc_pin(Pin::Lower(1), PinUse::Spi);
            lock.alloc_pin(Pin::Lower(2), PinUse::Spi);

            // default MODE0, SCK(AD0) default 0
            // set SCK(AD0) and MOSI (AD1) as output pins
            lock.lower.direction |= 0x03;

            let mut cmd = MpsseCmdBuilder::new();
            cmd.set_gpio_lower(lock.lower.value, lock.lower.direction)
                .disable_3phase_data_clocking()
                .send_immediate();
            lock.write_read(cmd.as_slice(), &mut [])?;
        }
        Ok(Spi {
            mtx,
            cmd: MSB_MODE0,
        })
    }
    /// set spi mode and bitorder
    pub fn set_mode(&mut self, mode: SpiMode) -> Result<(), FtdiError> {
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        // set SCK polarity
        match mode {
            SpiMode::MsbMode0 | SpiMode::LsbMode0 => lock.lower.value &= 0xfe, // set SCK(AD0) to 0
            SpiMode::MsbMode2 | SpiMode::LsbMode2 => lock.lower.value |= 0x01, // set SCK(AD0) to 1
        }
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction)
            .send_immediate();
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
        cmd.clock_data_in(self.cmd.read, words.len())
            .send_immediate();

        let lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.write_read(cmd.as_slice(), words)?;

        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_data_out(self.cmd.write, words).send_immediate();

        let lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.write_read(cmd.as_slice(), &mut [])?;

        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_data(self.cmd.write_read, words).send_immediate();

        let lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");

        lock.write_read(cmd.as_slice(), words)?;

        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.clock_data(self.cmd.write_read, write).send_immediate();

        let lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.write_read(cmd.as_slice(), read)?;

        Ok(())
    }
}
