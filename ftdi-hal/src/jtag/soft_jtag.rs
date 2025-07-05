use crate::{FtMpsse, InputPin, OutputPin, Pin, ftdaye::FtdiError};
use std::sync::{Arc, Mutex};

pub struct SoftJtag {
    mtx: Arc<Mutex<FtMpsse>>,
    tck: OutputPin,
    tdi: OutputPin,
    tdo: InputPin,
    tms: OutputPin,
    direction: Option<[OutputPin; 4]>,
}

impl SoftJtag {
    pub fn new(
        mtx: Arc<Mutex<FtMpsse>>,
        tck: Pin,
        tdi: Pin,
        tdo: Pin,
        tms: Pin,
    ) -> Result<Self, FtdiError> {
        let tck = OutputPin::new(mtx.clone(), tck)?;
        tck.set(false)?; // tck闲置状态为低
        Ok(Self {
            mtx: mtx.clone(),
            tck,
            tdi: OutputPin::new(mtx.clone(), tdi)?,
            tdo: InputPin::new(mtx.clone(), tdo)?,
            tms: OutputPin::new(mtx.clone(), tms)?,
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
    fn clock_tck(&self, tms_val: bool, tdi_val: bool) -> Result<bool, FtdiError> {
        self.tms.set(tms_val)?;
        self.tdi.set(tdi_val)?;
        // self.tck.set(false)?;
        self.tck.set(true)?; // 上升沿采样
        let tdo_val = self.tdo.get()?;
        self.tck.set(false)?; // 回到低电平
        Ok(tdo_val)
    }

    // 辅助函数：移位数据位
    fn shift_bits(&self, bits: &[bool], last: bool) -> Result<Vec<bool>, FtdiError> {
        let mut result = Vec::new();
        for (i, &bit) in bits.iter().enumerate() {
            let tms = last && (i == bits.len() - 1);
            result.push(self.clock_tck(tms, bit)?);
        }
        Ok(result)
    }

    // 将JTAG状态机复位到Run-Test/Idle状态
    fn goto_idle(&self) -> Result<(), FtdiError> {
        // 发送5个TMS=1复位状态机 (Test-Logic-Reset)
        for _ in 0..5 {
            self.clock_tck(true, true)?;
        }
        // 进入Run-Test/Idle: TMS=0 -> Run-Test/Idle
        self.clock_tck(false, true)?;
        // 保持Run-Test/Idle (TMS=0)
        self.clock_tck(false, true)?;
        Ok(())
    }

    // 写入JTAG寄存器
    pub fn write_reg(
        &self,
        ir: &[u8],
        irlen: usize,
        dr: &[u8],
        drlen: usize,
    ) -> Result<(), FtdiError> {
        self.goto_idle()?;

        // 进入Shift-IR状态
        self.clock_tck(true, true)?; // Select-DR-Scan
        self.clock_tck(true, true)?; // Select-IR-Scan
        self.clock_tck(false, true)?; // Capture-IR
        self.clock_tck(false, true)?; // Shift-IR

        // 转换IR数据为bool切片并移位
        let ir_bits: Vec<bool> = ir
            .iter()
            .flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1 != 0))
            .take(irlen)
            .collect();
        self.shift_bits(&ir_bits, true)?; // 移位IR（最后一位TMS=1）

        // 退出IR移位状态
        self.clock_tck(true, true)?; // Update-IR
        self.clock_tck(false, true)?; // 返回Run-Test/Idle

        // 进入Shift-DR状态
        self.clock_tck(true, true)?; // Select-DR-Scan
        self.clock_tck(false, true)?; // Capture-DR
        self.clock_tck(false, true)?; // Shift-DR

        // 转换DR数据为bool切片并移位
        let dr_bits: Vec<bool> = dr
            .iter()
            .flat_map(|&b| (0..8).map(move |i| (b >> i) & 1 != 0))
            .take(drlen)
            .collect();
        self.shift_bits(&dr_bits, true)?; // 移位DR（最后一位TMS=1）

        // 退出DR移位状态
        self.clock_tck(true, true)?; // Update-DR
        self.clock_tck(false, true)?; // 返回Run-Test/Idle

        Ok(())
    }

    // 读取JTAG寄存器
    pub fn read_reg(&self, ir: &[u8], irlen: usize, drlen: usize) -> Result<Vec<u8>, FtdiError> {
        self.goto_idle()?;

        // 进入Shift-IR状态
        self.clock_tck(true, true)?; // Select-DR-Scan
        self.clock_tck(true, true)?; // Select-IR-Scan
        self.clock_tck(false, true)?; // Capture-IR
        self.clock_tck(false, true)?; // Shift-IR

        // 移位IR指令
        let ir_bits: Vec<bool> = ir
            .iter()
            .flat_map(|&b| (0..8).map(move |i| (b >> i) & 1 != 0))
            .take(irlen)
            .collect();
        self.shift_bits(&ir_bits, true)?;

        // 退出IR移位状态
        self.clock_tck(true, true)?; // Update-IR
        self.clock_tck(false, true)?; // 返回Run-Test/Idle

        // 进入Shift-DR状态
        self.clock_tck(true, true)?; // Select-DR-Scan
        self.clock_tck(false, true)?; // Capture-DR
        self.clock_tck(false, true)?; // Shift-DR

        // 移位DR（移入0，读取数据）
        let zeros = vec![false; drlen];
        let dr_bits = self.shift_bits(&zeros, true)?;

        // 退出DR移位状态
        self.clock_tck(true, true)?; // Update-DR
        self.clock_tck(false, true)?; // 返回Run-Test/Idle

        // 将读取的位转换为字节
        let mut result = Vec::new();
        for chunk in dr_bits.chunks(8) {
            let mut byte = 0u8;
            for (i, &bit) in chunk.iter().enumerate() {
                if bit {
                    byte |= 1 << (7 - i);
                }
            }
            result.push(byte);
        }
        Ok(result)
    }

    // 使用指定TDI值扫描JTAG链上的设备IDCODE
    pub fn scan_with(&self, tdi_val: bool) -> Result<Vec<Option<u32>>, FtdiError> {
        const ID_LEN: usize = 32;
        self.goto_idle()?;

        // 进入Shift-DR状态
        self.clock_tck(true, true)?; // Select-DR-Scan
        self.clock_tck(false, true)?; // Capture-DR
        self.clock_tck(false, true)?; // Shift-DR

        // 移入0并读取TDO，持续直到检测到连续32个0
        let mut idcodes = Vec::new();
        let mut current_id = 0u32;
        let mut bit_count = 0;
        let mut consecutive_zeros = 0;
        let mut consecutive_ones = 0;

        while consecutive_ones < ID_LEN && consecutive_zeros < ID_LEN {
            // 每32位保存一个IDCODE
            if bit_count == ID_LEN {
                idcodes.push(Some(current_id));
                bit_count = 0;
            }

            let tdo_val = self.clock_tck(false, tdi_val)?; // 移入tdi_val
            // bypass
            if bit_count == 0 && !tdo_val {
                idcodes.push(None);
            } else {
                current_id = (current_id >> 1) | if tdo_val { 0x8000_0000 } else { 0 };
                bit_count += 1;
            }

            if tdo_val {
                consecutive_ones += 1;
                consecutive_zeros = 0;
            } else {
                consecutive_zeros += 1;
                consecutive_ones = 0;
            }
        }

        // 退出Shift-DR状态
        self.clock_tck(true, false)?; // Exit1-DR
        self.clock_tck(true, false)?; // Update-DR
        self.clock_tck(false, false)?; // 返回Run-Test/Idle

        Ok(idcodes)
    }
    // 扫描JTAG链上的设备IDCODE
    pub fn scan(&self) -> Result<Vec<Option<u32>>, FtdiError> {
        let scan0 = self.scan_with(false)?;
        let scan1 = self.scan_with(true)?;
        Ok(scan0
            .into_iter()
            .zip(scan1.into_iter())
            .take_while(|(x, y)| x == y)
            .map(|(x, _)| x)
            .collect())
    }
}
