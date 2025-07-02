use crate::error::Error;
use crate::gpio::Pin;
use crate::{BitOrder, FtInner, PinUse};
use ftdi_mpsse::{ClockData, ClockDataIn, ClockDataOut, MpsseCmdBuilder, MpsseCmdExecutor};
use std::sync::{Arc, Mutex};

/// FTDI SPI polarity.
///
/// This is a helper type to support multiple embedded-hal versions simultaneously.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpiCommond {
    /// MPSSE command used to clock data in and out simultaneously.
    ///
    /// This is set by [`Spi::set_clock_polarity`].
    read_write: ClockData,
    /// MPSSE command used to clock data out.
    ///
    /// This is set by [`Spi::set_clock_polarity`].
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
#[derive(Debug)]
pub struct Spi<Device: MpsseCmdExecutor> {
    /// Parent FTDI device.
    mtx: Arc<Mutex<FtInner<Device>>>,
    /// SPI polarity
    cmd: SpiCommond,
}

impl<Device: MpsseCmdExecutor> Drop for Spi<Device> {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.free_pin(Pin::Lower(0));
        lock.free_pin(Pin::Lower(1));
        lock.free_pin(Pin::Lower(2));
    }
}

impl<Device, E> Spi<Device>
where
    Device: MpsseCmdExecutor<Error = E>,
    E: std::error::Error,
    Error<E>: From<E>,
{
    pub(crate) fn new(mtx: Arc<Mutex<FtInner<Device>>>) -> Result<Spi<Device>, Error<E>> {
        {
            let mut lock = mtx.lock().expect("Failed to aquire FTDI mutex");
            lock.alloc_pin(Pin::Lower(0), PinUse::Spi);
            lock.alloc_pin(Pin::Lower(1), PinUse::Spi);
            lock.alloc_pin(Pin::Lower(2), PinUse::Spi);

            // clear direction of first 3 pins
            lock.lower.direction &= !0x07;
            // set SCK (AD0) and MOSI (AD1) as output pins
            lock.lower.direction |= 0x03;
            // set GPIO pins to new state
            lock.lower.value |= 0x01;
            let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
                .set_gpio_lower(lock.lower.value, lock.lower.direction)
                .send_immediate();
            lock.ft.send(cmd.as_slice())?;
        }
        let cmd = SpiCommond {
            read_write: ClockData::MsbPosIn,
            read: ClockDataIn::MsbPos,
            write: ClockDataOut::MsbNeg,
        };
        Ok(Spi { mtx, cmd })
    }
    /// set spi mode and bitorder
    pub fn set_mode(&mut self, mode: eh1::spi::Mode, order: BitOrder) -> Result<(), Error<E>> {
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        // set SCK polarity
        match mode.polarity {
            eh1::spi::Polarity::IdleLow => lock.lower.value &= 0xFE,
            eh1::spi::Polarity::IdleHigh => lock.lower.value |= 0x01,
        }
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .set_gpio_lower(lock.lower.value, lock.lower.direction)
            .send_immediate();
        lock.ft.send(cmd.as_slice())?;

        self.cmd = match (mode, order) {
            (eh1::spi::MODE_0 | eh1::spi::MODE_3, BitOrder::Lsb) => SpiCommond {
                read_write: ClockData::LsbPosIn,
                read: ClockDataIn::LsbPos,
                write: ClockDataOut::LsbNeg,
            },
            (eh1::spi::MODE_0 | eh1::spi::MODE_3, BitOrder::Msb) => SpiCommond {
                read_write: ClockData::MsbPosIn,
                read: ClockDataIn::MsbPos,
                write: ClockDataOut::MsbNeg,
            },
            (eh1::spi::MODE_1 | eh1::spi::MODE_2, BitOrder::Lsb) => SpiCommond {
                read_write: ClockData::LsbNegIn,
                read: ClockDataIn::LsbNeg,
                write: ClockDataOut::LsbPos,
            },
            (eh1::spi::MODE_1 | eh1::spi::MODE_2, BitOrder::Msb) => SpiCommond {
                read_write: ClockData::MsbNegIn,
                read: ClockDataIn::MsbNeg,
                write: ClockDataOut::MsbPos,
            },
        };
        Ok(())
    }
}

impl<E> eh1::spi::Error for Error<E>
where
    E: std::error::Error,
    Error<E>: From<E>,
{
    fn kind(&self) -> eh1::spi::ErrorKind {
        eh1::spi::ErrorKind::Other
    }
}

impl<Device, E> eh1::spi::ErrorType for Spi<Device>
where
    Device: MpsseCmdExecutor<Error = E>,
    E: std::error::Error,
    Error<E>: From<E>,
{
    type Error = Error<E>;
}

impl<Device, E> eh1::spi::SpiBus<u8> for Spi<Device>
where
    Device: MpsseCmdExecutor<Error = E>,
    E: std::error::Error,
    Error<E>: From<E>,
{
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data_in(self.cmd.read, words.len())
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.ft.send(cmd.as_slice())?;
        lock.ft.recv(words)?;

        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Error<E>> {
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data_out(self.cmd.write, words)
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.ft.send(cmd.as_slice())?;

        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Error<E>> {
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data(self.cmd.read_write, words)
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");

        lock.ft.send(cmd.as_slice())?;
        lock.ft.recv(words)?;

        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error<E>> {
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data(self.cmd.read_write, write)
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.ft.send(cmd.as_slice())?;
        lock.ft.recv(read)?;

        let remain: usize = write.len().saturating_sub(read.len());
        if remain != 0 {
            let mut remain_buf: Vec<u8> = vec![0; remain];
            lock.ft.recv(&mut remain_buf)?;
        }

        Ok(())
    }
}
