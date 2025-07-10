use std::sync::{Arc, Mutex};

use ftdi_hal::{FtdiMpsse, Swd, SwdAddr, list_all_device};

fn main() {
    env_logger::init();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0], 0).unwrap();
    let mtx = Arc::new(Mutex::new(mpsse));
    let swd = Swd::new(mtx).unwrap();
    swd.enable().unwrap();
    let idcode = swd.read(SwdAddr::Dp(0)).unwrap();
    println!("idcode:{idcode:#?}")
}
