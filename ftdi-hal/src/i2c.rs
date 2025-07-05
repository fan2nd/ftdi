use crate::ftdaye::FtdiError;
use crate::mpsse::{ClockBitsIn, ClockBitsOut, MpsseCmdBuilder};
use crate::{FtMpsse, Pin, PinUse};
use eh1::i2c::{ErrorKind, NoAcknowledgeSource, Operation, SevenBitAddress};
use std::sync::{Arc, Mutex};

/// SCL bitmask
const SCL: u8 = 1 << 0;
/// SDA bitmask
const SDA: u8 = 1 << 1;

const BITS_IN: ClockBitsIn = ClockBitsIn::MsbPos;
const BITS_OUT: ClockBitsOut = ClockBitsOut::MsbNeg;

/// FTDI I2C interface.
///
/// This is created by calling [`FtHal::i2c`].
///
/// [`FtHal::i2c`]: crate::FtHal::i2c
pub struct I2c {
    /// Parent FTDI device.
    mtx: Arc<Mutex<FtMpsse>>,
    /// Length of the start, repeated start, and stop conditions.
    ///
    /// The units for these are dimensionless number of MPSSE commands.
    /// More MPSSE commands roughly correlates to more time.
    start_stop_cmds: usize,
}

impl Drop for I2c {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.free_pin(Pin::Lower(0));
        lock.free_pin(Pin::Lower(1));
        lock.free_pin(Pin::Lower(2));
    }
}

impl I2c {
    pub fn new(mtx: Arc<Mutex<FtMpsse>>) -> Result<I2c, FtdiError> {
        {
            let mut lock = mtx.lock().expect("Failed to aquire FTDI mutex");

            lock.alloc_pin(Pin::Lower(0), PinUse::I2c);
            lock.alloc_pin(Pin::Lower(1), PinUse::I2c);
            lock.alloc_pin(Pin::Lower(2), PinUse::I2c);

            // clear direction and value of first 3 pins
            // set to input and value 0
            lock.lower.direction &= !0x07;
            lock.lower.value &= !0x07;
            // AD0: SCL
            // AD1: SDA (master out)
            // AD2: SDA (master in)
            // pins are set as input (tri-stated) in idle mode

            // set GPIO pins to new state
            let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
                .set_gpio_lower(lock.lower.value, lock.lower.direction)
                .disable_adaptive_data_clocking()
                .disable_loopback()
                .enable_3phase_data_clocking()
                .send_immediate();
            lock.ft.write_read(cmd.as_slice(), &mut [])?;
        }

        Ok(I2c {
            mtx,
            start_stop_cmds: 3,
        })
    }

    /// Set the length of start and stop conditions.
    ///
    /// This is an advanced feature that most people will not need to touch.
    /// I2C start and stop conditions are generated with a number of MPSSE
    /// commands.  This sets the number of MPSSE command generated for each
    /// stop and start condition.  An increase in the number of MPSSE commands
    /// roughtly correlates to an increase in the duration.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use ftdi_embedded_hal as hal;
    ///
    /// # #[cfg(feature = "libftd2xx")]
    /// # {
    /// let device = libftd2xx::Ft2232h::with_description("Dual RS232-HS A")?;
    /// let hal = hal::FtHal::init_freq(device, 3_000_000)?;
    /// let mut i2c = hal.i2c()?;
    /// i2c.set_stop_start_len(10);
    /// # }
    /// # Ok::<(), std::boxed::Box<dyn std::error::Error>>(())
    /// ```
    pub fn set_stop_start_len(&mut self, start_stop_cmds: usize) {
        self.start_stop_cmds = start_stop_cmds
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), ErrorKind> {
        // lock at the start to prevent GPIO from being modified while we build
        // the MPSSE command
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");

        // ST
        let mut mpsse_cmd: MpsseCmdBuilder = MpsseCmdBuilder::new();
        for _ in 0..self.start_stop_cmds {
            mpsse_cmd = mpsse_cmd.set_gpio_lower(
                lock.lower.value | SCL | SDA,
                SCL | SDA | lock.lower.direction,
            )
        }
        for _ in 0..self.start_stop_cmds {
            mpsse_cmd =
                mpsse_cmd.set_gpio_lower(lock.lower.value | SCL, SCL | SDA | lock.lower.direction)
        }
        lock.ft.write_read(mpsse_cmd.as_slice(), &mut [])?;

        let mut prev_op_was_a_read: bool = false;
        for (idx, operation) in operations.iter_mut().enumerate() {
            match operation {
                Operation::Read(buffer) => {
                    if idx == 0 || !prev_op_was_a_read {
                        let mut mpsse_cmd: MpsseCmdBuilder = MpsseCmdBuilder::new();
                        if idx != 0 {
                            // SR
                            for _ in 0..self.start_stop_cmds {
                                mpsse_cmd = mpsse_cmd.set_gpio_lower(
                                    lock.lower.value | SCL | SDA,
                                    SCL | SDA | lock.lower.direction,
                                )
                            }
                            for _ in 0..self.start_stop_cmds {
                                mpsse_cmd = mpsse_cmd.set_gpio_lower(
                                    lock.lower.value | SCL,
                                    SCL | SDA | lock.lower.direction,
                                )
                            }
                            for _ in 0..self.start_stop_cmds {
                                mpsse_cmd = mpsse_cmd.set_gpio_lower(
                                    lock.lower.value,
                                    SCL | SDA | lock.lower.direction,
                                )
                            }
                        }

                        mpsse_cmd = mpsse_cmd
                            // SAD + R
                            .set_gpio_lower(lock.lower.value, SCL | SDA | lock.lower.direction)
                            .clock_bits_out(BITS_OUT, (address << 1) | 1, 8)
                            // SAK
                            .set_gpio_lower(lock.lower.value, SCL | lock.lower.direction)
                            .clock_bits_in(BITS_IN, 1)
                            .send_immediate();

                        let mut ack_buf: [u8; 1] = [0; 1];
                        lock.ft.write_read(mpsse_cmd.as_slice(), &mut ack_buf)?;
                        if (ack_buf[0] & 0b1) != 0x00 {
                            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
                        }
                    }

                    let mut mpsse_cmd: MpsseCmdBuilder = MpsseCmdBuilder::new();
                    for idx in 0..buffer.len() {
                        mpsse_cmd = mpsse_cmd
                            .set_gpio_lower(lock.lower.value, SCL | lock.lower.direction)
                            .clock_bits_in(BITS_IN, 8);
                        if idx == buffer.len() - 1 {
                            // NMAK
                            mpsse_cmd = mpsse_cmd
                                .set_gpio_lower(lock.lower.value, SCL | SDA | lock.lower.direction)
                                .clock_bits_out(BITS_OUT, 0x80, 1)
                        } else {
                            // MAK
                            mpsse_cmd = mpsse_cmd
                                .set_gpio_lower(lock.lower.value, SCL | SDA | lock.lower.direction)
                                .clock_bits_out(BITS_OUT, 0x00, 1)
                        }
                    }
                    lock.ft.write_read(mpsse_cmd.as_slice(), &mut [])?;

                    prev_op_was_a_read = true;
                }
                Operation::Write(bytes) => {
                    if idx == 0 || prev_op_was_a_read {
                        let mut mpsse_cmd: MpsseCmdBuilder = MpsseCmdBuilder::new();
                        if idx != 0 {
                            // SR
                            for _ in 0..self.start_stop_cmds {
                                mpsse_cmd = mpsse_cmd.set_gpio_lower(
                                    lock.lower.value | SCL | SDA,
                                    SCL | SDA | lock.lower.direction,
                                )
                            }
                            for _ in 0..self.start_stop_cmds {
                                mpsse_cmd = mpsse_cmd.set_gpio_lower(
                                    lock.lower.value | SCL,
                                    SCL | SDA | lock.lower.direction,
                                )
                            }
                            for _ in 0..self.start_stop_cmds {
                                mpsse_cmd = mpsse_cmd.set_gpio_lower(
                                    lock.lower.value,
                                    SCL | SDA | lock.lower.direction,
                                )
                            }
                        }

                        mpsse_cmd = mpsse_cmd
                            // SAD + W
                            .set_gpio_lower(lock.lower.value, SCL | SDA | lock.lower.direction)
                            .clock_bits_out(BITS_OUT, address << 1, 8)
                            // SAK
                            .set_gpio_lower(lock.lower.value, SCL | lock.lower.direction)
                            .clock_bits_in(BITS_IN, 1)
                            .send_immediate();

                        let mut ack_buf: [u8; 1] = [0; 1];
                        lock.ft.write_read(mpsse_cmd.as_slice(), &mut ack_buf)?;
                        if (ack_buf[0] & 0b1) != 0x00 {
                            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
                        }
                    }

                    for byte in *bytes {
                        let mut mpsse_cmd: MpsseCmdBuilder = MpsseCmdBuilder::new();
                        mpsse_cmd = mpsse_cmd
                            // Oi
                            .set_gpio_lower(lock.lower.value, SCL | SDA | lock.lower.direction)
                            .clock_bits_out(BITS_OUT, *byte, 8)
                            // SAK
                            .set_gpio_lower(lock.lower.value, SCL | lock.lower.direction)
                            .clock_bits_in(BITS_IN, 1)
                            .send_immediate();

                        let mut ack_buf: [u8; 1] = [0; 1];
                        lock.ft.write_read(mpsse_cmd.as_slice(), &mut ack_buf)?;
                        if (ack_buf[0] & 0b1) != 0x00 {
                            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
                        }
                    }

                    prev_op_was_a_read = false;
                }
            }
        }

        let mut mpsse_cmd: MpsseCmdBuilder = MpsseCmdBuilder::new();
        // SP
        for _ in 0..self.start_stop_cmds {
            mpsse_cmd = mpsse_cmd.set_gpio_lower(lock.lower.value, SCL | SDA | lock.lower.direction)
        }
        for _ in 0..self.start_stop_cmds {
            mpsse_cmd =
                mpsse_cmd.set_gpio_lower(lock.lower.value | SCL, SCL | SDA | lock.lower.direction)
        }
        for _ in 0..self.start_stop_cmds {
            mpsse_cmd = mpsse_cmd.set_gpio_lower(
                lock.lower.value | SCL | SDA,
                SCL | SDA | lock.lower.direction,
            )
        }

        // Idle
        mpsse_cmd = mpsse_cmd
            .set_gpio_lower(lock.lower.value, lock.lower.direction)
            .send_immediate();
        lock.ft.write_read(mpsse_cmd.as_slice(), &mut [])?;

        Ok(())
    }
}

impl From<FtdiError> for ErrorKind {
    fn from(_value: FtdiError) -> Self {
        ErrorKind::Other
    }
}

impl eh1::i2c::ErrorType for I2c {
    type Error = eh1::i2c::ErrorKind;
}

impl eh1::i2c::I2c for I2c {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction(address, operations)
    }
}
