use crate::ftdaye::FtdiError;
use crate::mpsse::{ClockBits, ClockBytes, ClockTMS, ClockTMSOut, MpsseCmdBuilder};
use crate::{FtMpsse, OutputPin, Pin, PinUse};
use std::ops::{Deref, DerefMut};
use std::sync::{Arc, Mutex};

// const BYTES_WRITE: ClockBytesOut = ClockBytesOut::LsbNeg;
// const BYTES_READ: ClockBytesIn = ClockBytesIn::LsbPos;
const BYTES_WRITE_READ: ClockBytes = ClockBytes::LsbPosIn;

// const BITS_WRITE: ClockBitsOut = ClockBitsOut::LsbNeg;
// const BITS_READ: ClockBitsIn = ClockBitsIn::LsbPos;
const BITS_WRITE_READ: ClockBits = ClockBits::LsbPosIn;

const TMS_WRITE: ClockTMSOut = ClockTMSOut::NegEdge;
const TMS_WRITE_READ: ClockTMS = ClockTMS::NegTMSPosTDO;

pub struct JtagCmdBuilder(MpsseCmdBuilder);
impl JtagCmdBuilder {
    fn new() -> Self {
        JtagCmdBuilder(MpsseCmdBuilder::new())
    }
    fn jtag_any2idle(&mut self) -> &mut Self {
        self.0.clock_tms_out(TMS_WRITE, 0b0001_1111, true, 6);
        self
    }
    fn jtag_idle_cycle(&mut self) -> &mut Self {
        self.0.clock_tms_out(TMS_WRITE, 0, true, 7);
        self
    }
    fn jtag_idle2ir(&mut self) -> &mut Self {
        self.0.clock_tms_out(TMS_WRITE, 0b0000_0011, true, 4);
        self
    }
    fn jtag_ir_exit2dr(&mut self) -> &mut Self {
        self.0.clock_tms_out(TMS_WRITE, 0b0000_0011, true, 4);
        self
    }
    fn jtag_idle2dr(&mut self) -> &mut Self {
        self.0.clock_tms_out(TMS_WRITE, 0b0000_0001, true, 3);
        self
    }
    fn dr_exit2idle(&mut self) -> &mut Self {
        self.0.clock_tms_out(TMS_WRITE, 0b0000_0001, true, 2);
        self
    }
    fn jtag_shift(&mut self, data: &[u8], bits_count: usize) -> &mut Self {
        let last_bit_is_bit7 = (bits_count & 0b111) == 0;
        let bytes_count = if last_bit_is_bit7 {
            (bits_count >> 3) - 1
        } else {
            bits_count >> 3
        };
        let remain_bits = (bits_count & 0b111) - 1; // not include full bytes and last bit
        let last_bit = data[bytes_count] >> data[bytes_count] >> remain_bits == 1;
        self.clock_bytes(BYTES_WRITE_READ, &data[0..bytes_count])
            .clock_bits(BITS_WRITE_READ, data[bytes_count], remain_bits)
            .clock_tms(TMS_WRITE_READ, 0b0000_0001, last_bit, 1);
        self
    }
    fn jtag_parse_single_shift(response: &mut [u8], bits_count: usize) -> usize {
        let last_bit_is_bit7 = (bits_count & 0b111) == 0;
        let bytes_count = if last_bit_is_bit7 {
            (bits_count >> 3) - 1
        } else {
            bits_count >> 3
        };
        let remain_bits = (bits_count & 0b111) - 1; // not include full bytes and last bit
        if remain_bits == 0 {
            response[bytes_count] >>= 7
        } else {
            response[bytes_count] >>= 8 - remain_bits;
            response[bytes_count] |= (response[bytes_count + 1] & 0b1000_0000) >> (7 - remain_bits);
        }
        bytes_count + 1
    }
}
impl Deref for JtagCmdBuilder {
    type Target = MpsseCmdBuilder;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for JtagCmdBuilder {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
pub struct Jtag {
    /// Parent FTDI device.
    mtx: Arc<Mutex<FtMpsse>>,
    is_ilde: bool,
    direction: Option<[OutputPin; 4]>,
}
impl Drop for Jtag {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.free_pin(Pin::Lower(0));
        lock.free_pin(Pin::Lower(1));
        lock.free_pin(Pin::Lower(2));
        lock.free_pin(Pin::Lower(3));
    }
}
impl Jtag {
    pub fn new(mtx: Arc<Mutex<FtMpsse>>) -> Result<Self, FtdiError> {
        {
            let mut lock = mtx.lock().expect("Failed to aquire FTDI mutex");
            lock.alloc_pin(Pin::Lower(0), PinUse::Jtag);
            lock.alloc_pin(Pin::Lower(1), PinUse::Jtag);
            lock.alloc_pin(Pin::Lower(2), PinUse::Jtag);
            lock.alloc_pin(Pin::Lower(3), PinUse::Jtag);
            // set TCK(AD0) TDI(AD1) TMS(AD3) as output pins
            lock.lower.direction |= 0x0b;
            // TCK(AD0) must be init with value 0
            let mut cmd = MpsseCmdBuilder::new();
            cmd.set_gpio_lower(lock.lower.value, lock.lower.direction)
                .disable_3phase_data_clocking()
                .send_immediate();
            lock.write_read(cmd.as_slice(), &mut [])?;
        }
        Ok(Self {
            mtx,
            is_ilde: false,
            direction: Default::default(),
        })
    }
    pub fn with_direction(
        &mut self,
        tck: Pin,
        tdi: Pin,
        tdo: Pin,
        tms: Pin,
    ) -> Result<(), FtdiError> {
        let tck = OutputPin::new(self.mtx.clone(), tck)?;
        let tdi = OutputPin::new(self.mtx.clone(), tdi)?;
        let tdo = OutputPin::new(self.mtx.clone(), tdo)?;
        let tms = OutputPin::new(self.mtx.clone(), tms)?;
        tck.set(true)?;
        tdi.set(true)?;
        tdo.set(false)?;
        tms.set(true)?;
        self.direction = Some([tck, tdi, tdo, tms]);
        Ok(())
    }
    pub fn goto_idle(&mut self) -> Result<(), FtdiError> {
        let mut cmd = JtagCmdBuilder::new();
        cmd.jtag_any2idle().send_immediate();
        let lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.write_read(cmd.as_slice(), &mut [])?;
        self.is_ilde = true;
        Ok(())
    }
    pub fn scan_with(&mut self, tdi: bool) -> Result<Vec<Option<u32>>, FtdiError> {
        const ID_LEN: usize = 32;
        let mut cmd = JtagCmdBuilder::new();
        cmd.jtag_any2idle().jtag_idle2dr();
        let lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.write_read(cmd.as_slice(), &mut [])?;
        let tdi = if tdi { vec![0xff; 4] } else { vec![0; 4] };
        // 移入0并读取TDO，持续直到检测到连续32个0
        let mut idcodes = Vec::new();
        let mut current_id = 0u32;
        let mut bit_count = 0;
        let mut consecutive_zeros = 0;

        'outer: loop {
            let mut cmd = MpsseCmdBuilder::new();
            cmd.clock_bytes(BYTES_WRITE_READ, &tdi).send_immediate();
            let read_buf = &mut [0; 4];
            lock.write_read(cmd.as_slice(), read_buf)?;
            let tdos: Vec<_> = read_buf
                .iter()
                .flat_map(|&byte| (0..8).map(move |i| (byte >> i) & 1 == 1))
                .collect();
            for tdo_val in tdos {
                // bypass
                if bit_count == 0 && !tdo_val {
                    idcodes.push(None);
                    consecutive_zeros += 1;
                } else {
                    current_id = (current_id >> 1) | if tdo_val { 0x8000_0000 } else { 0 };
                    bit_count += 1;
                    consecutive_zeros = 0;
                }
                // 连续32个0退出
                if consecutive_zeros == ID_LEN {
                    break 'outer;
                }
                // 每32位保存一个IDCODE
                if bit_count == ID_LEN {
                    // 连续32个1退出
                    if current_id == u32::MAX {
                        break 'outer;
                    }
                    idcodes.push(Some(current_id));
                    bit_count = 0;
                }
            }
        }

        // 退出Shift-DR状态
        drop(lock);
        self.goto_idle()?;
        Ok(idcodes)
    }
    pub fn write_read(
        &self,
        ir: &[u8],
        irlen: usize,
        dr: &[u8],
        drlen: usize,
    ) -> Result<Vec<u8>, FtdiError> {
        log::warn!("Not test");
        let mut cmd = JtagCmdBuilder::new();
        if !self.is_ilde {
            cmd.jtag_any2idle();
        }
        cmd.jtag_idle2ir()
            .jtag_shift(ir, irlen)
            .jtag_ir_exit2dr()
            .jtag_shift(dr, drlen)
            .dr_exit2idle()
            .jtag_idle_cycle()
            .send_immediate();
        let lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        let mut read_buf = vec![0; cmd.read_len()];
        lock.write_read(cmd.as_slice(), &mut read_buf)?;
        let len = JtagCmdBuilder::jtag_parse_single_shift(&mut read_buf, drlen);

        if read_buf.len() > len {
            read_buf.pop();
        }
        Ok(read_buf)
    }
}
