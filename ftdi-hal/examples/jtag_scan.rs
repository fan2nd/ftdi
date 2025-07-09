use std::{
    sync::{Arc, Mutex},
    time::Instant,
};

use ftdi_hal::{FtMpsse, Jtag, list_all_device};

fn main() {
    env_logger::init();
    let now = Instant::now();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtMpsse::open(&devices[0].usb_device, devices[0].interface[0], 0).unwrap();
    mpsse.set_frequency(30_000_000).unwrap();
    let mtx = Arc::new(Mutex::new(mpsse));
    let mut jtag = Jtag::new(mtx).unwrap();
    let ids = jtag.scan_with(true).unwrap();
    println!("Scan Result:{ids:x?}");
    println!("Finish Scan Using {:?}", now.elapsed());
}
