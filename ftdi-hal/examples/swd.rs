use std::sync::{Arc, Mutex};

use ftdi_hal::{FtMpsse, Interface, Swd, SwdPort, list_all_device};

fn main() {
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtMpsse::open(&devices[0], Interface::A, 0).unwrap();
    let mtx = Arc::new(Mutex::new(mpsse));
    let swd = Swd::new(mtx).unwrap();
    swd.enable().unwrap();
    let idcode = swd.read(SwdPort::Dp, 0x00).unwrap();
    println!("idcode:{idcode:#?}")
}
