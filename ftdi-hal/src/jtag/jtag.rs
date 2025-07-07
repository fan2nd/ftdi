use crate::ftdaye::FtdiError;
use crate::mpsse::{ClockData, ClockDataIn, ClockDataOut, ClockTMS, ClockTMSOut, MpsseCmdBuilder};
use crate::{BitOrder, FtMpsse, OutputPin, Pin, PinUse};
use std::fs::read;
use std::ops::Deref;
use std::sync::{Arc, Mutex};

pub struct JtagCmdBuilder(MpsseCmdBuilder);
impl JtagCmdBuilder {
    fn new() -> Self {
        JtagCmdBuilder(MpsseCmdBuilder::new())
    }
    fn as_slice(&self) -> &[u8] {
        self.0.as_slice()
    }
    fn any2idle(self) -> Self {
        JtagCmdBuilder(
            self.0
                .clock_tms_out(ClockTMSOut::NegEdge, 0b0001_1111, true, 6),
        )
    }
    fn idle_cycle(self) -> Self {
        JtagCmdBuilder(self.0.clock_tms_out(ClockTMSOut::NegEdge, 0, true, 7))
    }
    fn idle2ir(self) -> Self {
        JtagCmdBuilder(
            self.0
                .clock_tms_out(ClockTMSOut::NegEdge, 0b0000_0011, true, 4),
        )
    }
    fn ir_last(self, tdi: bool) -> Self {
        JtagCmdBuilder(
            self.0
                .clock_tms(ClockTMS::NegTMSPosTDO, 0b0000_0001, tdi, 1),
        )
    }
    fn ir_exit2dr(self) -> Self {
        JtagCmdBuilder(
            self.0
                .clock_tms_out(ClockTMSOut::NegEdge, 0b0000_0011, true, 4),
        )
    }
    fn idle2dr(self) -> Self {
        JtagCmdBuilder(
            self.0
                .clock_tms_out(ClockTMSOut::NegEdge, 0b0000_0001, true, 3),
        )
    }
    fn dr_last(self, tdi: bool) -> Self {
        JtagCmdBuilder(
            self.0
                .clock_tms(ClockTMS::NegTMSPosTDO, 0b0000_0001, tdi, 1),
        )
    }
    fn dr_exit2idle(self) -> Self {
        JtagCmdBuilder(
            self.0
                .clock_tms_out(ClockTMSOut::NegEdge, 0b0000_0001, true, 2),
        )
    }
}

pub struct Jtag {
    /// Parent FTDI device.
    mtx: Arc<Mutex<FtMpsse>>,
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
            let cmd = MpsseCmdBuilder::new()
                .set_gpio_lower(lock.lower.value, lock.lower.direction)
                .disable_adaptive_data_clocking()
                .set_clock(0, Some(false))
                .disable_loopback()
                .disable_3phase_data_clocking()
                .send_immediate();
            lock.write_read(cmd.as_slice(), &mut [])?;
        }
        Ok(Self {
            mtx: mtx,
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
    pub fn goto_idle(&self) -> Result<(), FtdiError> {
        let cmd = JtagCmdBuilder::new().any2idle();
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.write_read(cmd.as_slice(), &mut [])?;
        Ok(())
    }
    pub fn scan_with(&self, tdi: bool) -> Result<Vec<Option<u32>>, FtdiError> {
        const ID_LEN: usize = 32;
        let cmd = JtagCmdBuilder::new().any2idle().idle2dr();
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.write_read(cmd.as_slice(), &mut [])?;
        let tdi = if tdi { vec![0xff; 4] } else { vec![0; 4] };
        // 移入0并读取TDO，持续直到检测到连续32个0
        let mut idcodes = Vec::new();
        let mut current_id = 0u32;
        let mut bit_count = 0;
        let mut consecutive_zeros = 0;

        'outer: loop {
            let cmd = MpsseCmdBuilder::new()
                .clock_data(ClockData::LsbPosIn, &tdi)
                .send_immediate();
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
        let cmd = JtagCmdBuilder::new().any2idle();
        lock.write_read(cmd.as_slice(), &mut [])?;
        Ok(idcodes)
    }
}
