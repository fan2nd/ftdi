//! Copy from ftdi-mpsse crate
//! Multi-protocol synchronous serial engine utilities for FTDI devices.

/// MPSSE opcodes.
///
/// Exported for use by [`mpsse`] macro. May also be used for manual command array construction.
///
/// Data clocking MPSSE commands are broken out into separate enums for API ergonomics:
/// * [`ClockBytesOut`]
/// * [`ClockBitsOut`]
/// * [`ClockBytesIn`]
/// * [`ClockBitsIn`]
/// * [`ClockBytes`]
/// * [`ClockBits`]
#[repr(u8)]
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum MpsseCmd {
    /// Used by [`set_gpio_lower`][`MpsseCmdBuilder::set_gpio_lower`].
    SetDataBitsLowbyte = 0x80,
    /// Used by [`gpio_lower`][`MpsseCmdBuilder::gpio_lower`].
    GetDataBitsLowbyte = 0x81,
    /// Used by [`set_gpio_upper`][`MpsseCmdBuilder::set_gpio_upper`].
    SetDataBitsHighbyte = 0x82,
    /// Used by [`gpio_upper`][`MpsseCmdBuilder::gpio_upper`].
    GetDataBitsHighbyte = 0x83,
    /// Used by [`enable_loopback`][`MpsseCmdBuilder::enable_loopback`].
    EnableLoopback = 0x84,
    /// Used by [`disable_loopback`][`MpsseCmdBuilder::disable_loopback`].
    DisableLoopback = 0x85,
    /// Used by [`set_clock`][`MpsseCmdBuilder::set_clock`].
    SetClockFrequency = 0x86,
    /// Used by [`send_immediate`][`MpsseCmdBuilder::send_immediate`].
    SendImmediate = 0x87,
    /// Used by [`wait_on_io_high`][`MpsseCmdBuilder::wait_on_io_high`].
    WaitOnIOHigh = 0x88,
    /// Used by [`wait_on_io_low`][`MpsseCmdBuilder::wait_on_io_low`].
    WaitOnIOLow = 0x89,
    /// Used by [`set_clock`][`MpsseCmdBuilder::set_clock`].
    DisableClockDivide = 0x8A,
    /// Used by [`set_clock`][`MpsseCmdBuilder::set_clock`].
    EnableClockDivide = 0x8B,
    /// Used by [`enable_3phase_data_clocking`][`MpsseCmdBuilder::enable_3phase_data_clocking`].
    Enable3PhaseClocking = 0x8C,
    /// Used by [`disable_3phase_data_clocking`][`MpsseCmdBuilder::disable_3phase_data_clocking`].
    Disable3PhaseClocking = 0x8D,
    /// Used by [`disable_adaptive_data_clocking`][`MpsseCmdBuilder::disable_adaptive_data_clocking`].
    EnableAdaptiveClocking = 0x96,
    /// Used by [`enable_adaptive_data_clocking`][`MpsseCmdBuilder::enable_adaptive_data_clocking`].
    DisableAdaptiveClocking = 0x97,
    /// This command is only available to FT232
    EnableDriveOnlyZero = 0x9E,
}

/// Modes for clocking data out of the FTDI device.
///
/// This is an argument to the [`clock_bytes_out`] method.
///
/// [`clock_bytes_out`]: MpsseCmdBuilder::clock_bytes_out
#[repr(u8)]
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum ClockBytesOut {
    /// Positive clock edge MSB first.
    ///
    /// The data is sent MSB first.
    ///
    /// The data will change to the next bit on the rising edge of the CLK pin.
    MsbPos = 0x10,
    /// Negative clock edge MSB first.
    ///
    /// The data is sent MSB first.
    ///
    /// The data will change to the next bit on the falling edge of the CLK pin.
    MsbNeg = 0x11,
    /// Positive clock edge LSB first.
    ///
    /// The first bit in will be the LSB of the first byte and so on.
    ///
    /// The data will change to the next bit on the rising edge of the CLK pin.
    LsbPos = 0x18,
    /// Negative clock edge LSB first.
    ///
    /// The first bit in will be the LSB of the first byte and so on.
    ///
    /// The data will change to the next bit on the falling edge of the CLK pin.
    LsbNeg = 0x19,
}

/// Modes for clocking bits out of the FTDI device.
///
/// This is an argument to the [`clock_bits_out`] method.
///
/// [`clock_bits_out`]: MpsseCmdBuilder::clock_bits_out
#[repr(u8)]
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum ClockBitsOut {
    /// Positive clock edge MSB first.
    ///
    /// The data is sent MSB first (bit 7 first).
    ///
    /// The data will change to the next bit on the rising edge of the CLK pin.
    MsbPos = 0x12,
    /// Negative clock edge MSB first.
    ///
    /// The data is sent MSB first (bit 7 first).
    ///
    /// The data will change to the next bit on the falling edge of the CLK pin.
    MsbNeg = 0x13,
    /// Positive clock edge LSB first (bit 0 first).
    ///
    /// The first bit in will be the LSB of the first byte and so on.
    ///
    /// The data will change to the next bit on the rising edge of the CLK pin.
    LsbPos = 0x1A,
    /// Negative clock edge LSB first (bit 0 first).
    ///
    /// The first bit in will be the LSB of the first byte and so on.
    ///
    /// The data will change to the next bit on the falling edge of the CLK pin.
    LsbNeg = 0x1B,
}

/// Modes for clocking data into the FTDI device.
///
/// This is an argument to the [`clock_bytes_in`] method.
///
/// [`clock_bytes_in`]: MpsseCmdBuilder::clock_bytes_in
#[repr(u8)]
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum ClockBytesIn {
    /// Positive clock edge MSB first.
    ///
    /// The first bit in will be the MSB of the first byte and so on.
    ///
    /// The data will be sampled on the rising edge of the CLK pin.
    MsbPos = 0x20,
    /// Negative clock edge MSB first.
    ///
    /// The first bit in will be the MSB of the first byte and so on.
    ///
    /// The data will be sampled on the falling edge of the CLK pin.
    MsbNeg = 0x24,
    /// Positive clock edge LSB first.
    ///
    /// The first bit in will be the LSB of the first byte and so on.
    ///
    /// The data will be sampled on the rising edge of the CLK pin.
    LsbPos = 0x28,
    /// Negative clock edge LSB first.
    ///
    /// The first bit in will be the LSB of the first byte and so on.
    ///
    /// The data will be sampled on the falling edge of the CLK pin.
    LsbNeg = 0x2C,
}

/// Modes for clocking data bits into the FTDI device.
///
/// This is an argument to the [`clock_bits_in`] method.
///
/// [`clock_bits_in`]: MpsseCmdBuilder::clock_bits_in
#[repr(u8)]
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum ClockBitsIn {
    /// Positive clock edge MSB first.
    ///
    /// The data will be shifted up so that the first bit in may not be in bit 7
    /// but from 6 downwards depending on the number of bits to shift
    /// (i.e. a length of 1 bit will have the data bit sampled in bit 0 of the
    /// byte sent back to the PC).
    ///
    /// The data will be sampled on the rising edge of the CLK pin.
    MsbPos = 0x22,
    /// Negative clock edge MSB first.
    ///
    /// The data will be shifted up so that the first bit in may not be in bit 7
    /// but from 6 downwards depending on the number of bits to shift
    /// (i.e. a length of 1 bit will have the data bit sampled in bit 0 of the
    /// byte sent back to the PC).
    ///
    /// The data will be sampled on the falling edge of the CLK pin.
    MsbNeg = 0x26,
    /// Positive clock edge LSB first.
    ///
    /// The data will be shifted down so that the first bit in may not be in bit
    /// 0 but from 1 upwards depending on the number of bits to shift
    /// (i.e. a length of 1 bit will have the data bit sampled in bit 7 of the
    /// byte sent back to the PC).
    ///
    /// The data will be sampled on the rising edge of the CLK pin.
    LsbPos = 0x2A,
    /// Negative clock edge LSB first.
    ///
    /// The data will be shifted down so that the first bit in may not be in bit
    /// 0 but from 1 upwards depending on the number of bits to shift
    /// (i.e. a length of 1 bit will have the data bit sampled in bit 7 of the
    /// byte sent back to the PC).
    ///
    /// The data will be sampled on the falling edge of the CLK pin.
    LsbNeg = 0x2E,
}

/// Modes for clocking data in and out of the FTDI device.
///
/// This is an argument to the [`clock_bytes`] method.
///
/// [`clock_bytes`]: MpsseCmdBuilder::clock_bytes
#[repr(u8)]
#[allow(dead_code)]
#[allow(clippy::enum_variant_names)]
#[derive(Debug, Copy, Clone)]
pub enum ClockBytes {
    /// MSB first, data in on positive edge, data out on negative edge.
    MsbPosIn = 0x31,
    /// MSB first, data in on negative edge, data out on positive edge.
    MsbNegIn = 0x34,
    /// LSB first, data in on positive edge, data out on negative edge.
    LsbPosIn = 0x39,
    /// LSB first, data in on negative edge, data out on positive edge.
    LsbNegIn = 0x3C,
}

/// Modes for clocking data bits in and out of the FTDI device.
///
/// This is an argument to the [`clock_bits`] method.
///
/// [`clock_bits`]: MpsseCmdBuilder::clock_bits
#[repr(u8)]
#[allow(dead_code)]
#[allow(clippy::enum_variant_names)]
#[derive(Debug, Copy, Clone)]
pub enum ClockBits {
    /// MSB first, data in on positive edge, data out on negative edge.
    MsbPosIn = 0x33,
    /// MSB first, data in on negative edge, data out on positive edge.
    MsbNegIn = 0x36,
    /// LSB first, data in on positive edge, data out on negative edge.
    LsbPosIn = 0x3B,
    /// LSB first, data in on negative edge, data out on positive edge.
    LsbNegIn = 0x3E,
}

/// Modes for clocking bits out on TMS for JTAG mode.
///
/// This is an argument to the [`clock_tms_out`] method.
///
/// [`clock_tms_out`]: MpsseCmdBuilder::clock_tms_out
#[repr(u8)]
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum ClockTMSOut {
    /// LSB first, TMS out on positive edge
    PosEdge = 0x4A,
    /// LSB first, TMS out on negative edge
    NegEdge = 0x4B,
}

/// Modes for clocking bits out on TMS for JTAG mode while reading TDO.
///
/// This is an argument to the [`clock_tms`] method.
///
/// [`clock_tms`]: MpsseCmdBuilder::clock_tms
#[repr(u8)]
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum ClockTMS {
    /// LSB first, TMS out on positive edge, TDO in on positive edge.
    PosTMSPosTDO = 0x6A,
    /// LSB first, TMS out on positive edge, TDO in on negative edge.
    PosTMSNegTDO = 0x6E,
    /// LSB first, TMS out on negative edge, TDO in on positive edge.
    NegTMSPosTDO = 0x6B,
    /// LSB first, TMS out on negative edge, TDO in on negative edge.
    NegTMSNegTDO = 0x6F,
}

/// FTDI Multi-Protocol Synchronous Serial Engine (MPSSE) command builder.
///
/// For details about the MPSSE read the [FTDI MPSSE Basics].
///
/// This structure is a `Vec<u8>` that the methods push bytewise commands onto.
/// These commands can then be written to the device with the appropriate
/// implementations of [`send`] and [`xfer`] methods.
///
/// This is useful for creating commands that need to do multiple operations
/// quickly, since individual write calls can be expensive. For example,
/// this can be used to set a GPIO low and clock data out for SPI operations.
///
/// If dynamic command layout is not required, the [`mpsse`] macro can build
/// command `[u8; N]` arrays at compile-time.
///
/// [FTDI MPSSE Basics]: https://www.ftdichip.com/Support/Documents/AppNotes/AN_135_MPSSE_Basics.pdf
/// [`send`]: MpsseCmdExecutor::send
/// [`xfer`]: MpsseCmdExecutor::xfer
#[derive(Default)]
pub struct MpsseCmdBuilder {
    cmd: Vec<u8>,
    read_len: usize,
}
#[allow(unused)]
impl MpsseCmdBuilder {
    /// Create a new command builder.
    pub const fn new() -> MpsseCmdBuilder {
        MpsseCmdBuilder {
            cmd: Vec::new(),
            read_len: 0,
        }
    }

    /// Get the MPSSE command as a slice.
    pub fn as_slice(&self) -> &[u8] {
        self.cmd.as_slice()
    }

    /// Set the MPSSE clock frequency using provided
    /// divisor value and clock divider configuration.
    /// Both parameters are device dependent.
    pub fn set_clock(&mut self, divisor: u16, clkdiv: Option<bool>) -> &mut Self {
        match clkdiv {
            Some(true) => self.cmd.push(MpsseCmd::EnableClockDivide as u8),
            Some(false) => self.cmd.push(MpsseCmd::DisableClockDivide as u8),
            None => {}
        };

        self.cmd.push(MpsseCmd::SetClockFrequency as u8);
        self.cmd.push((divisor & 0xFF) as u8);
        self.cmd.push(((divisor >> 8) & 0xFF) as u8);

        self
    }

    /// Enable the MPSSE loopback state.
    pub fn enable_loopback(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::EnableLoopback as u8);
        self
    }

    /// Disable the MPSSE loopback state.
    pub fn disable_loopback(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::DisableLoopback as u8);
        self
    }
    /// Disable 3 phase data clocking.
    ///
    /// This is only available on FTx232H devices.
    ///
    /// This will give a 2 stage data shift which is the default state.
    ///
    /// It will appears as:
    ///
    /// 1. Data setup for 1/2 clock period
    /// 2. Pulse clock for 1/2 clock period
    pub fn disable_3phase_data_clocking(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::Disable3PhaseClocking as u8);
        self
    }

    /// Enable 3 phase data clocking.
    ///
    /// This is only available on FTx232H devices.
    ///
    /// This will give a 3 stage data shift for the purposes of supporting
    /// interfaces such as I2C which need the data to be valid on both edges of
    /// the clock.
    ///
    /// It will appears as:
    ///
    /// 1. Data setup for 1/2 clock period
    /// 2. Pulse clock for 1/2 clock period
    /// 3. Data hold for 1/2 clock period
    pub fn enable_3phase_data_clocking(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::Enable3PhaseClocking as u8);
        self
    }

    /// Enable adaptive data clocking.
    ///
    /// This is only available on FTx232H devices.
    pub fn enable_adaptive_data_clocking(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::EnableAdaptiveClocking as u8);
        self
    }

    /// Enable adaptive data clocking.
    ///
    /// This is only available on FTx232H devices.
    pub fn disable_adaptive_data_clocking(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::DisableAdaptiveClocking as u8);
        self
    }

    /// Set the pin direction and state of the lower byte (0-7) GPIO pins on the
    /// MPSSE interface.
    ///
    /// The pins that this controls depends on the device.
    ///
    /// * On the FT232H this will control the AD0-AD7 pins.
    ///
    /// # Arguments
    ///
    /// * `state` - GPIO state mask, `0` is low (or input pin), `1` is high.
    /// * `direction` - GPIO direction mask, `0` is input, `1` is output.
    pub fn set_gpio_lower(&mut self, state: u8, direction: u8) -> &mut Self {
        self.cmd
            .extend_from_slice(&[MpsseCmd::SetDataBitsLowbyte as u8, state, direction]);
        self
    }

    /// Set the pin direction and state of the upper byte (8-15) GPIO pins on
    /// the MPSSE interface.
    ///
    /// The pins that this controls depends on the device.
    /// This method may do nothing for some devices, such as the FT4232H that
    /// only have 8 pins per port.
    ///
    /// # Arguments
    ///
    /// * `state` - GPIO state mask, `0` is low (or input pin), `1` is high.
    /// * `direction` - GPIO direction mask, `0` is input, `1` is output.
    ///
    /// # FT232H Corner Case
    ///
    /// On the FT232H only CBUS5, CBUS6, CBUS8, and CBUS9 can be controlled.
    /// These pins confusingly map to the first four bits in the direction and
    /// state masks.
    pub fn set_gpio_upper(&mut self, state: u8, direction: u8) -> &mut Self {
        self.cmd
            .extend_from_slice(&[MpsseCmd::SetDataBitsHighbyte as u8, state, direction]);
        self
    }

    /// Get the pin state state of the lower byte (0-7) GPIO pins on the MPSSE
    /// interface.
    pub fn gpio_lower(&mut self) -> &mut Self {
        self.read_len += 1;
        self.cmd.push(MpsseCmd::GetDataBitsLowbyte as u8);
        self
    }

    /// Get the pin state state of the upper byte (8-15) GPIO pins on the MPSSE
    /// interface.
    ///
    /// See [`set_gpio_upper`] for additional information about physical pin
    /// mappings.
    ///
    /// [`set_gpio_upper`]: MpsseCmdBuilder::set_gpio_upper
    pub fn gpio_upper(&mut self) -> &mut Self {
        self.read_len += 1;
        self.cmd.push(MpsseCmd::GetDataBitsHighbyte as u8);
        self
    }

    /// Send the preceding commands immediately.
    pub fn send_immediate(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::SendImmediate as u8);
        self
    }

    /// Make controller wait until GPIOL1 or I/O1 is high before running further commands.
    /// use crate::mpsse::{ClockBytes, MpsseCmdBuilder};
    ///
    /// // Assume a "chip ready" signal is connected to GPIOL1. This signal is pulled high
    /// // shortly after AD3 (chip select) is pulled low. Data will not be clocked out until
    /// // the chip is ready.
    pub fn wait_on_io_high(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::WaitOnIOHigh as u8);
        self
    }

    /// Make controller wait until GPIOL1 or I/O1 is low before running further commands.
    /// use crate::mpsse::{ClockBytes, MpsseCmdBuilder};
    ///
    /// // Assume a "chip ready" signal is connected to GPIOL1. This signal is pulled low
    /// // shortly after AD3 (chip select) is pulled low. Data will not be clocked out until
    /// // the chip is ready.
    pub fn wait_on_io_low(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::WaitOnIOLow as u8);
        self
    }

    /// Clock data out.
    ///
    /// This will clock out bytes on TDI/DO.
    /// No data is clocked into the device on TDO/DI.
    ///
    /// This will panic for data lengths greater than `u16::MAX + 1`.
    pub fn clock_bytes_out(&mut self, mode: ClockBytesOut, data: &[u8]) -> &mut Self {
        let mut len = data.len();
        assert!(
            len > 0 && len <= 65536,
            "data length should be in 1..=u16::MAX+1"
        );
        len -= 1;
        self.cmd
            .extend_from_slice(&[mode as u8, (len & 0xFF) as u8, ((len >> 8) & 0xFF) as u8]);
        self.cmd.extend_from_slice(data);
        self
    }

    /// Clock data in.
    ///
    /// This will clock in bytes on TDO/DI.
    /// No data is clocked out of the device on TDI/DO.
    ///
    /// # Arguments
    ///
    /// * `mode` - Data clocking mode.
    /// * `len` - Number of bytes to clock in.
    ///   This will panic for values greater than `u16::MAX + 1`.
    pub fn clock_bytes_in(&mut self, mode: ClockBytesIn, mut len: usize) -> &mut Self {
        assert!(
            len > 0 && len <= 65536,
            "data length should be in 1..=u16::MAX+1"
        );
        self.read_len += len;
        len -= 1;
        self.cmd
            .extend_from_slice(&[mode as u8, (len & 0xFF) as u8, ((len >> 8) & 0xFF) as u8]);
        self
    }

    /// Clock data in and out simultaneously.
    ///
    /// This will panic for data lengths greater than `u16::MAX + 1`.
    pub fn clock_bytes(&mut self, mode: ClockBytes, data: &[u8]) -> &mut Self {
        let mut len = data.len();
        assert!(
            len > 0 && len <= 65536,
            "data length should be in 1..=u16::MAX+1"
        );
        self.read_len += len;
        len -= 1;
        self.cmd
            .extend_from_slice(&[mode as u8, (len & 0xFF) as u8, ((len >> 8) & 0xFF) as u8]);
        self.cmd.extend_from_slice(data);
        self
    }

    /// Clock data bits out.
    ///
    /// # Arguments
    ///
    /// * `mode` - Bit clocking mode.
    /// * `data` - Data bits.
    /// * `len` - Number of bits to clock out.
    ///   This will panic for values greater than 8.
    pub fn clock_bits_out(&mut self, mode: ClockBitsOut, data: u8, len: usize) -> &mut Self {
        assert!(len > 0 && len <= 8, "data length should be in 1..=8");
        self.cmd
            .extend_from_slice(&[mode as u8, (len - 1) as u8, data]);
        self
    }

    /// Clock data bits in.
    ///
    /// # Arguments
    ///
    /// * `mode` - Bit clocking mode.
    /// * `len` - Number of bits to clock in.
    ///   This will panic for values greater than 8.
    pub fn clock_bits_in(&mut self, mode: ClockBitsIn, len: usize) -> &mut Self {
        assert!(len > 0 && len <= 8, "data length should be in 1..=8");
        self.read_len += 1;
        self.cmd.extend_from_slice(&[mode as u8, (len - 1) as u8]);
        self
    }

    /// Clock data bits in and out simultaneously.
    ///
    /// # Arguments
    ///
    /// * `mode` - Bit clocking mode.
    /// * `len` - Number of bits to clock in.
    ///   This will panic for values greater than 8.
    pub fn clock_bits(&mut self, mode: ClockBits, data: u8, len: usize) -> &mut Self {
        assert!(len > 0 && len <= 8, "data length should be in 1..=8");
        self.read_len += 1;
        self.cmd
            .extend_from_slice(&[mode as u8, (len - 1) as u8, data]);
        self
    }

    /// Clock TMS bits out.
    ///
    /// # Arguments
    ///
    /// * `mode` - TMS clocking mode.
    /// * `data` - TMS bits.
    /// * `tdi` - Value to place on TDI while clocking.
    /// * `len` - Number of bits to clock out.
    ///   This will panic for values greater than 7.
    pub fn clock_tms_out(
        &mut self,
        mode: ClockTMSOut,
        mut data: u8,
        tdi: bool,
        len: usize,
    ) -> &mut Self {
        assert!(len > 0 && len <= 7, "data length should be in 1..=7");
        if tdi {
            data |= 0x80;
        }
        self.cmd
            .extend_from_slice(&[mode as u8, (len - 1) as u8, data]);
        self
    }

    /// Clock TMS bits out while clocking TDO bits in.
    ///
    /// # Arguments
    ///
    /// * `mode` - TMS clocking mode.
    /// * `data` - TMS bits.
    /// * `tdi` - Value to place on TDI while clocking.
    /// * `len` - Number of bits to clock out.
    ///   This will panic for values greater than 7.
    pub fn clock_tms(&mut self, mode: ClockTMS, mut data: u8, tdi: bool, len: usize) -> &mut Self {
        assert!(len > 0 && len <= 7, "data length should be in 1..=7");
        self.read_len += 1;
        if tdi {
            data |= 0x80;
        }
        self.cmd
            .extend_from_slice(&[mode as u8, (len - 1) as u8, data]);
        self
    }
}
