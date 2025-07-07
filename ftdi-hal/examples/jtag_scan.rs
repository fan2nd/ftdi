use std::{
    sync::{Arc, Mutex},
    time::Instant,
};

use ftdi_hal::{FtMpsse, Interface, Jtag, list_all_device};

fn main() {
    let now = Instant::now();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtMpsse::open(devices[0].clone(), Interface::A, 0).unwrap();
    let mtx = Arc::new(Mutex::new(mpsse));
    let jtag = Jtag::new(mtx).unwrap();
    let ids = jtag.scan_with(true).unwrap();
    println!("Scan Result:{:x?}", ids);
    println!("Finish Scan Using {:?}", now.elapsed());
}
