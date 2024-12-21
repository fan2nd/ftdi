use nusb::DeviceInfo;

use crate::{
    error::FtdiError,
    interface::{FtdiInterface, FtdiInterfaceEnum},
    request::BitMode,
};

#[derive(Debug, Clone, Copy)]
pub enum ChipType {
    FT232H,
    FT2232H,
    FT4232H,
    Unknown,
}
#[derive(Debug, Clone, Copy)]
pub struct ChipInfo {
    #[allow(unused)]
    chip_type: ChipType,
    vid: u16,
    pid: u16,
    fifo_size: usize,
}

pub struct FtdiDevice {
    device: DeviceInfo,
    chip_info: &'static ChipInfo,
}
impl FtdiDevice {
    const KNOWN_CHIPS: &'static [ChipInfo] = &[
        ChipInfo {
            chip_type: ChipType::FT232H,
            vid: 0x0403,
            pid: 0x6014,
            fifo_size: 1024,
        },
        ChipInfo {
            chip_type: ChipType::FT2232H,
            vid: 0x0403,
            pid: 0x6010,
            fifo_size: 4096,
        },
    ];

    pub fn list_all() -> Vec<FtdiDevice> {
        nusb::list_devices()
            .unwrap()
            .into_iter()
            .filter_map(|x| {
                for info in Self::KNOWN_CHIPS.iter() {
                    if x.vendor_id() == info.vid && x.product_id() == info.pid {
                        return Some(FtdiDevice {
                            device: x,
                            chip_info: info,
                        });
                    }
                }
                return None;
            })
            .collect()
    }
    pub fn open(
        self,
        interface: FtdiInterfaceEnum,
        bitmode: BitMode,
    ) -> Result<FtdiInterface, FtdiError> {
        let device = self
            .device
            .open()
            .map_err(|_| FtdiError::DeviceOpenFailed)?;
        let usb_interface = device
            .detach_and_claim_interface(interface.interface_number())
            .map_err(|_| FtdiError::InterfaceOpenFailed)?;
        FtdiInterface::new(
            usb_interface,
            interface,
            bitmode,
            512,
            self.chip_info.fifo_size,
        )
    }
}
