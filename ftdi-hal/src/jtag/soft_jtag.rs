use crate::{FtMpsse, OutputPin, Pin, PinUse, ftdaye::FtdiError, mpsse::MpsseCmdBuilder};
use std::{
    sync::{Arc, Mutex},
    u32,
};

pub struct SoftJtag {
    mtx: Arc<Mutex<FtMpsse>>,
    tck: usize,
    tdi: usize,
    tdo: usize,
    tms: usize,
    direction: Option<[OutputPin; 4]>,
}
impl Drop for SoftJtag {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.free_pin(Pin::Lower(self.tck));
        lock.free_pin(Pin::Lower(self.tdi));
        lock.free_pin(Pin::Lower(self.tdo));
        lock.free_pin(Pin::Lower(self.tms));
    }
}
impl SoftJtag {
    /// Only can use lower pins
    pub fn new(
        mtx: Arc<Mutex<FtMpsse>>,
        tck: usize,
        tdi: usize,
        tdo: usize,
        tms: usize,
    ) -> Result<Self, FtdiError> {
        let mut lock = mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.alloc_pin(Pin::Lower(tck), PinUse::Jtag);
        lock.alloc_pin(Pin::Lower(tdi), PinUse::Jtag);
        lock.alloc_pin(Pin::Lower(tdo), PinUse::Jtag);
        lock.alloc_pin(Pin::Lower(tms), PinUse::Jtag);
        // all pins default set to low
        lock.lower.direction |= 1 << tck | 1 << tdi | 1 << tms; // all pins default input, set tck/tdi/tms to output
        Ok(Self {
            mtx: mtx.clone(),
            tck,
            tdi,
            tdo,
            tms,
            direction: None,
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

    // 辅助函数：产生时钟边沿并读取TDO
    fn clock_tck(&self, tms_val: u8, tdi_val: u8) -> Result<bool, FtdiError> {
        assert!(tms_val == 0 || tms_val == 1);
        assert!(tdi_val == 0 || tdi_val == 1);
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        let cmd = MpsseCmdBuilder::new()
            .set_gpio_lower(
                lock.lower.value | tdi_val << self.tdi | tms_val << self.tms,
                lock.lower.direction,
            )
            .set_gpio_lower(
                lock.lower.value | 1 << self.tck | tdi_val << self.tdi | tms_val << self.tms,
                lock.lower.direction,
            )
            .gpio_lower()
            .set_gpio_lower(
                lock.lower.value | tdi_val << self.tdi | tms_val << self.tms,
                lock.lower.direction,
            )
            .send_immediate();
        let read = &mut [0];
        lock.write_read(cmd.as_slice(), read)?;
        Ok(read[0] & 1 << self.tdo != 0)
    }

    // 辅助函数：产生时钟边沿并读取TDO
    fn clock_tcks(&self, tms_val: u8, tdi_val: u8, count: usize) -> Result<Vec<bool>, FtdiError> {
        assert!(tms_val == 0 || tms_val == 1);
        assert!(tdi_val == 0 || tdi_val == 1);
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        let mut cmd = MpsseCmdBuilder::new().set_gpio_lower(
            lock.lower.value | tdi_val << self.tdi | tms_val << self.tms,
            lock.lower.direction,
        );
        for _ in 0..count {
            cmd = cmd
                .set_gpio_lower(
                    lock.lower.value | 1 << self.tck | tdi_val << self.tdi | tms_val << self.tms,
                    lock.lower.direction,
                ) // set tck to high
                .gpio_lower()
                .set_gpio_lower(
                    lock.lower.value | tdi_val << self.tdi | tms_val << self.tms,
                    lock.lower.direction,
                ); // set tck to low
        }
        cmd = cmd.send_immediate();
        let mut read_buf = vec![0; count];
        lock.write_read(cmd.as_slice(), &mut read_buf)?;
        Ok(read_buf
            .into_iter()
            .map(|x| x & 1 << self.tdo != 0)
            .collect())
    }

    // 将JTAG状态机复位到Run-Test/Idle状态
    fn goto_idle(&self) -> Result<(), FtdiError> {
        // 发送5个TMS=1复位状态机 (Test-Logic-Reset)
        self.clock_tcks(1, 1, 5)?;
        // 进入Run-Test/Idle: TMS=0 -> Run-Test/Idle
        self.clock_tck(0, 1)?;
        // 保持Run-Test/Idle (TMS=0)
        self.clock_tck(0, 1)?;
        Ok(())
    }

    // 使用指定TDI值扫描JTAG链上的设备IDCODE
    pub fn scan_with(&self, tdi_val: u8) -> Result<Vec<Option<u32>>, FtdiError> {
        const ID_LEN: usize = 32;
        self.goto_idle()?;

        // 进入Shift-DR状态
        self.clock_tck(1, 1)?; // Select-DR-Scan
        self.clock_tck(0, 1)?; // Capture-DR
        self.clock_tck(0, 1)?; // Shift-DR

        // 移入0并读取TDO，持续直到检测到连续32个0
        let mut idcodes = Vec::new();
        let mut current_id = 0u32;
        let mut bit_count = 0;
        let mut consecutive_zeros = 0;

        'outer: loop {
            let tdos = self.clock_tcks(0, tdi_val, ID_LEN)?; // 移入tdi_val
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
        self.clock_tck(1, 0)?; // Exit1-DR
        self.clock_tck(1, 0)?; // Update-DR
        self.clock_tck(0, 0)?; // 返回Run-Test/Idle

        Ok(idcodes)
    }
}
