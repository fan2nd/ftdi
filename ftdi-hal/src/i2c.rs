use crate::ftdaye::FtdiError;
use crate::mpsse_cmd::MpsseCmdBuilder;
use crate::{FtdiMpsse, Pin, PinUse};
use eh1::i2c::{ErrorKind, NoAcknowledgeSource, Operation, SevenBitAddress};
use std::sync::{Arc, Mutex};

/// SCL bitmask
const SCL: u8 = 1 << 0;
/// SDA bitmask
const SDA: u8 = 1 << 1;

const TCK_INIT_VALUE: bool = false;
const IS_LSB: bool = false;

/// FTDI I2C interface.
pub struct I2c {
    /// Parent FTDI device.
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// Length of the start, repeated start, and stop conditions.
    ///
    /// The units for these are dimensionless number of MPSSE commands.
    /// More MPSSE commands roughly correlates to more time.
    start_stop_cmds: usize,
}

impl Drop for I2c {
    fn drop(&mut self) {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.enable_3phase_data_clocking(false);
        let mut lock = self.mtx.lock().unwrap();
        lock.write_read(cmd.as_slice(), &mut []).unwrap();
        lock.free_pin(Pin::Lower(0));
        lock.free_pin(Pin::Lower(1));
        lock.free_pin(Pin::Lower(2));
    }
}

impl I2c {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<I2c, FtdiError> {
        {
            log::warn!("IIC module has not been tested yet!");
            let mut lock = mtx.lock().unwrap();

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
            let mut cmd = MpsseCmdBuilder::new();
            cmd.set_gpio_lower(lock.lower.value, lock.lower.direction)
                .enable_3phase_data_clocking(true);
            lock.write_read(cmd.as_slice(), &mut [])?;
        }
        let this = I2c {
            mtx,
            start_stop_cmds: 3,
        };
        log::info!("IIC default 100Khz");
        this.set_frequency(100_000)?;
        Ok(this)
    }

    /// Set the length of start and stop conditions.
    ///
    /// This is an advanced feature that most people will not need to touch.
    /// I2C start and stop conditions are generated with a number of MPSSE
    /// commands.  This sets the number of MPSSE command generated for each
    /// stop and start condition.  An increase in the number of MPSSE commands
    /// roughtly correlates to an increase in the duration.
    pub fn set_stop_start_len(&mut self, start_stop_cmds: usize) {
        self.start_stop_cmds = start_stop_cmds
    }

    pub fn set_frequency(&self, frequency_hz: usize) -> Result<(), FtdiError> {
        let lock = self.mtx.lock().unwrap();
        lock.set_frequency(frequency_hz * 3 / 2)?;
        Ok(())
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), ErrorKind> {
        // lock at the start to prevent GPIO from being modified while we build
        // the MPSSE command
        let lock = self.mtx.lock().unwrap();

        // ST
        let mut mpsse_cmd = MpsseCmdBuilder::new();
        for _ in 0..self.start_stop_cmds {
            mpsse_cmd.set_gpio_lower(
                lock.lower.value | SCL | SDA,
                SCL | SDA | lock.lower.direction,
            );
        }
        for _ in 0..self.start_stop_cmds {
            mpsse_cmd.set_gpio_lower(lock.lower.value | SCL, SCL | SDA | lock.lower.direction);
        }
        lock.write_read(mpsse_cmd.as_slice(), &mut [])?;

        let mut prev_op_was_a_read: bool = false;
        for (idx, operation) in operations.iter_mut().enumerate() {
            match operation {
                Operation::Read(buffer) => {
                    if idx == 0 || !prev_op_was_a_read {
                        let mut mpsse_cmd = MpsseCmdBuilder::new();
                        if idx != 0 {
                            // SR
                            for _ in 0..self.start_stop_cmds {
                                mpsse_cmd.set_gpio_lower(
                                    lock.lower.value | SCL | SDA,
                                    SCL | SDA | lock.lower.direction,
                                );
                            }
                            for _ in 0..self.start_stop_cmds {
                                mpsse_cmd.set_gpio_lower(
                                    lock.lower.value | SCL,
                                    SCL | SDA | lock.lower.direction,
                                );
                            }
                            for _ in 0..self.start_stop_cmds {
                                mpsse_cmd.set_gpio_lower(
                                    lock.lower.value,
                                    SCL | SDA | lock.lower.direction,
                                );
                            }
                        }

                        mpsse_cmd
                            // SAD + R
                            .set_gpio_lower(lock.lower.value, SCL | SDA | lock.lower.direction)
                            .clock_bits_out(TCK_INIT_VALUE, IS_LSB, (address << 1) | 1, 8)
                            // SAK
                            .set_gpio_lower(lock.lower.value, SCL | lock.lower.direction)
                            .clock_bits_in(TCK_INIT_VALUE, IS_LSB, 1);

                        let mut ack_buf: [u8; 1] = [0; 1];
                        lock.write_read(mpsse_cmd.as_slice(), &mut ack_buf)?;
                        if (ack_buf[0] & 0b1) != 0x00 {
                            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
                        }
                    }

                    let mut mpsse_cmd = MpsseCmdBuilder::new();
                    for idx in 0..buffer.len() {
                        mpsse_cmd
                            .set_gpio_lower(lock.lower.value, SCL | lock.lower.direction)
                            .clock_bits_in(TCK_INIT_VALUE, IS_LSB, 8);
                        if idx == buffer.len() - 1 {
                            // NMAK
                            mpsse_cmd
                                .set_gpio_lower(lock.lower.value, SCL | SDA | lock.lower.direction)
                                .clock_bits_out(TCK_INIT_VALUE, IS_LSB, 0x80, 1);
                        } else {
                            // MAK
                            mpsse_cmd
                                .set_gpio_lower(lock.lower.value, SCL | SDA | lock.lower.direction)
                                .clock_bits_out(TCK_INIT_VALUE, IS_LSB, 0x00, 1);
                        }
                    }
                    lock.write_read(mpsse_cmd.as_slice(), &mut [])?;

                    prev_op_was_a_read = true;
                }
                Operation::Write(bytes) => {
                    if idx == 0 || prev_op_was_a_read {
                        let mut mpsse_cmd = MpsseCmdBuilder::new();
                        if idx != 0 {
                            // SR
                            for _ in 0..self.start_stop_cmds {
                                mpsse_cmd.set_gpio_lower(
                                    lock.lower.value | SCL | SDA,
                                    SCL | SDA | lock.lower.direction,
                                );
                            }
                            for _ in 0..self.start_stop_cmds {
                                mpsse_cmd.set_gpio_lower(
                                    lock.lower.value | SCL,
                                    SCL | SDA | lock.lower.direction,
                                );
                            }
                            for _ in 0..self.start_stop_cmds {
                                mpsse_cmd.set_gpio_lower(
                                    lock.lower.value,
                                    SCL | SDA | lock.lower.direction,
                                );
                            }
                        }

                        mpsse_cmd
                            // SAD + W
                            .set_gpio_lower(lock.lower.value, SCL | SDA | lock.lower.direction)
                            .clock_bits_out(TCK_INIT_VALUE, IS_LSB, address << 1, 8)
                            // SAK
                            .set_gpio_lower(lock.lower.value, SCL | lock.lower.direction)
                            .clock_bits_in(TCK_INIT_VALUE, IS_LSB, 1);

                        let mut ack_buf: [u8; 1] = [0; 1];
                        lock.write_read(mpsse_cmd.as_slice(), &mut ack_buf)?;
                        if (ack_buf[0] & 0b1) != 0x00 {
                            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
                        }
                    }

                    for byte in *bytes {
                        let mut mpsse_cmd = MpsseCmdBuilder::new();
                        mpsse_cmd
                            // Oi
                            .set_gpio_lower(lock.lower.value, SCL | SDA | lock.lower.direction)
                            .clock_bits_out(TCK_INIT_VALUE, IS_LSB, *byte, 8)
                            // SAK
                            .set_gpio_lower(lock.lower.value, SCL | lock.lower.direction)
                            .clock_bits_in(TCK_INIT_VALUE, IS_LSB, 1);

                        let mut ack_buf: [u8; 1] = [0; 1];
                        lock.write_read(mpsse_cmd.as_slice(), &mut ack_buf)?;
                        if (ack_buf[0] & 0b1) != 0x00 {
                            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
                        }
                    }

                    prev_op_was_a_read = false;
                }
            }
        }

        let mut mpsse_cmd = MpsseCmdBuilder::new();
        // SP
        for _ in 0..self.start_stop_cmds {
            mpsse_cmd.set_gpio_lower(lock.lower.value, SCL | SDA | lock.lower.direction);
        }
        for _ in 0..self.start_stop_cmds {
            mpsse_cmd.set_gpio_lower(lock.lower.value | SCL, SCL | SDA | lock.lower.direction);
        }
        for _ in 0..self.start_stop_cmds {
            mpsse_cmd.set_gpio_lower(
                lock.lower.value | SCL | SDA,
                SCL | SDA | lock.lower.direction,
            );
        }

        // Idle
        mpsse_cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.write_read(mpsse_cmd.as_slice(), &mut [])?;

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
