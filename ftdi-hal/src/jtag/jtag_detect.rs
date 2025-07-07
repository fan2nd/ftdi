use crate::{FtMpsse, Pin, PinUse, ftdaye::FtdiError, mpsse::MpsseCmdBuilder};
use std::sync::{Arc, Mutex};

pub struct JtagDetect {
    mtx: Arc<Mutex<FtMpsse>>,
    tck: usize,
    tms: usize,
    has_direction: bool,
}
impl Drop for JtagDetect {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        for i in 0..8 {
            lock.free_pin(Pin::Lower(i));
            if self.has_direction {
                lock.free_pin(Pin::Upper(i));
            }
        }
    }
}
impl JtagDetect {
    /// Only can use lower pins
    pub fn new(mtx: Arc<Mutex<FtMpsse>>, tck: usize, tms: usize) -> Result<Self, FtdiError> {
        let mut lock = mtx.lock().expect("Failed to aquire FTDI mutex");
        for i in 0..8 {
            lock.alloc_pin(Pin::Lower(i), PinUse::JtagDetect);
        }
        // all pins default set to low
        Ok(Self {
            mtx: mtx.clone(),
            tck,
            tms,
            has_direction: false,
        })
    }

    pub fn with_direction(&mut self) -> Result<(), FtdiError> {
        self.has_direction = true;
        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        for i in 0..8 {
            lock.alloc_pin(Pin::Upper(i), PinUse::JtagDetect);
        }
        lock.upper.value |= 1 << self.tck | 1 << self.tms;
        lock.upper.direction = 0xff;
        let cmd = MpsseCmdBuilder::new().set_gpio_upper(lock.upper.value, lock.upper.direction);
        lock.write_read(cmd.as_slice(), &mut [])?;
        Ok(())
    }

    // 将JTAG状态机复位到Run-Test/Idle状态
    fn reset2dr(&self) -> Result<(), FtdiError> {
        let direction = 1 << self.tck | 1 << self.tms;
        let mut cmd = MpsseCmdBuilder::new().set_gpio_lower(1 << self.tck, direction);
        for _ in 0..5 {
            cmd = cmd
                // TMS1
                .set_gpio_lower(1 << self.tms, direction) // TCK to low
                .set_gpio_lower(1 << self.tck | 1 << self.tms, direction); // TCK to high
        }
        cmd = cmd
            // TMS0
            .set_gpio_lower(0, direction) // TCK to low
            .set_gpio_lower(1 << self.tck, direction) // TCK to high
            // TMS1
            .set_gpio_lower(1 << self.tms, direction) // TCK to low
            .set_gpio_lower(1 << self.tck | 1 << self.tms, direction) // TCK to high
            // TMS0
            .set_gpio_lower(0, direction) // TCK to low
            .set_gpio_lower(1 << self.tck, direction) // TCK to high
            .set_gpio_lower(0, direction) // TCK to low
            .set_gpio_lower(1 << self.tck, direction); // TCK to high
        let lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.write_read(cmd.as_slice(), &mut [])?;
        Ok(())
    }

    fn shift_dr(&self, len: usize) -> Result<Vec<u8>, FtdiError> {
        let direction = 1 << self.tck | 1 << self.tms;
        let mut read_buf = vec![0; len];
        let mut cmd = MpsseCmdBuilder::new();
        for _ in 0..len {
            cmd = cmd
                .set_gpio_lower(0, direction) // TCK to low
                .set_gpio_lower(1 << self.tck, direction) // TCK to high
                .gpio_lower()
        }
        let lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.write_read(cmd.as_slice(), &mut read_buf)?;
        Ok(read_buf)
    }

    // 使用指定TDI值扫描JTAG链上的设备IDCODE
    pub fn scan(&self) -> Result<Vec<Vec<Option<u32>>>, FtdiError> {
        const ID_LEN: usize = 32;
        self.reset2dr()?;

        // 移入0并读取TDO，持续直到检测到连续32个0
        let mut idcodes = vec![Vec::new(); 8];
        let mut current_id = [0u32; 8];
        let mut bit_count = [0; 8];
        let mut consecutive_zeros = [0; 8];

        let read = self.shift_dr(ID_LEN * 2)?;
        // println!("read_buf{read:?}");
        for i in 0..8 {
            if i == self.tck || i == self.tms {
                continue;
            }
            let tdos: Vec<_> = read.iter().map(|&x| (x >> i) & 1 == 1).collect();

            for tdo_val in tdos {
                // bypass
                if bit_count[i] == 0 && !tdo_val {
                    idcodes[i].push(None);
                    consecutive_zeros[i] += 1;
                } else {
                    current_id[i] = (current_id[i] >> 1) | if tdo_val { 0x8000_0000 } else { 0 };
                    bit_count[i] += 1;
                    consecutive_zeros[i] = 0;
                }
                // 连续32个0退出
                if consecutive_zeros[i] == ID_LEN {
                    break;
                }
                // 每32位保存一个IDCODE
                if bit_count[i] == ID_LEN {
                    // 连续32个1退出
                    if current_id[i] == u32::MAX {
                        break;
                    }
                    idcodes[i].push(Some(current_id[i]));
                    bit_count[i] = 0;
                }
            }
        }

        Ok(idcodes)
    }
}
