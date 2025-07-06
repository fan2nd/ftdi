use crate::ftdaye::FtdiError;
use crate::mpsse::{ClockData, ClockDataIn, ClockDataOut, MpsseCmdBuilder};
use crate::{BitOrder, FtMpsse, Pin, PinUse};
use eh1::spi::{Error, ErrorKind, ErrorType, SpiBus};
use std::sync::{Arc, Mutex};

/// FTDI SPI polarity.
///
/// This is a helper type to support multiple embedded-hal versions simultaneously.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpiCommond {
    write_read: ClockData,
    read: ClockDataIn,
    write: ClockDataOut,
}

/// FTDI SPI bus.
///
/// In embedded-hal version 1 this represents an exclusive SPI bus.
///
/// This is created by calling [`FtHal::spi`].
///
/// [`FtHal::spi`]: crate::FtHal::spi
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
            let mut lock = mtx.lock().expect("Failed to aquire FTDI mutex");
            lock.alloc_pin(Pin::Lower(0), PinUse::Spi);
            lock.alloc_pin(Pin::Lower(1), PinUse::Spi);
            lock.alloc_pin(Pin::Lower(2), PinUse::Spi);

            // clear direction of first 3 pins
            lock.lower.direction &= !0x07;
            // set SCK(AD0) and MOSI (AD1) as output pins
            lock.lower.direction |= 0x03;
            // set SCK(AD0) to 1
            lock.lower.value |= 0x01;
            let cmd = MpsseCmdBuilder::new()
                .set_gpio_lower(lock.lower.value, lock.lower.direction)
                .disable_adaptive_data_clocking()
                .disable_loopback()
                .disable_3phase_data_clocking()
                .send_immediate();
            lock.ft.write_read(cmd.as_slice(), &mut [])?;
        }
        let cmd = SpiCommond {
            write_read: ClockData::MsbPosIn,
            read: ClockDataIn::MsbPos,
            write: ClockDataOut::MsbNeg,
        };
        Ok(Spi { mtx, cmd })
    }
    /// set spi mode and bitorder
    pub fn set_mode(&mut self, mode: eh1::spi::Mode, order: BitOrder) -> Result<(), FtdiError> {
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        // set SCK polarity
        match mode.polarity {
            eh1::spi::Polarity::IdleLow => lock.lower.value &= 0xFE, // set SCK(AD0) to 0
            eh1::spi::Polarity::IdleHigh => lock.lower.value |= 0x01, // set SCK(AD0) to 1
        }
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .set_gpio_lower(lock.lower.value, lock.lower.direction)
            .send_immediate();
        lock.ft.write_read(cmd.as_slice(), &mut [])?;

        self.cmd = match (mode, order) {
            (eh1::spi::MODE_0 | eh1::spi::MODE_3, BitOrder::Lsb) => SpiCommond {
                write_read: ClockData::LsbPosIn,
                read: ClockDataIn::LsbPos,
                write: ClockDataOut::LsbNeg,
            },
            (eh1::spi::MODE_0 | eh1::spi::MODE_3, BitOrder::Msb) => SpiCommond {
                write_read: ClockData::MsbPosIn,
                read: ClockDataIn::MsbPos,
                write: ClockDataOut::MsbNeg,
            },
            (eh1::spi::MODE_1 | eh1::spi::MODE_2, BitOrder::Lsb) => SpiCommond {
                write_read: ClockData::LsbNegIn,
                read: ClockDataIn::LsbNeg,
                write: ClockDataOut::LsbPos,
            },
            (eh1::spi::MODE_1 | eh1::spi::MODE_2, BitOrder::Msb) => SpiCommond {
                write_read: ClockData::MsbNegIn,
                read: ClockDataIn::MsbNeg,
                write: ClockDataOut::MsbPos,
            },
        };
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
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data_in(self.cmd.read, words.len())
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.ft.write_read(cmd.as_slice(), words)?;

        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data_out(self.cmd.write, words)
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.ft.write_read(cmd.as_slice(), &mut [])?;

        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data(self.cmd.write_read, words)
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");

        lock.ft.write_read(cmd.as_slice(), words)?;

        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data(self.cmd.write_read, write)
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.ft.write_read(cmd.as_slice(), read)?;

        Ok(())
    }
}
