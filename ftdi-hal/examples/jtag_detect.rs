use std::sync::{Arc, Mutex};

use ftdi_hal::{FtMpsse, Interface, Pin, SoftJtag, list_all_device};
use itertools::Itertools;

fn main() {
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtMpsse::open(devices[0].clone(), Interface::A, 0).unwrap();
    let mtx = Arc::new(Mutex::new(mpsse));
    let pins = [Pin::Lower(0), Pin::Lower(1), Pin::Lower(2), Pin::Lower(3)];
    for couple in pins.into_iter().permutations(4) {
        println!(
            "testing: tck:{:?},tdi:{:?},tdo:{:?},tms:{:?}",
            couple[0], couple[1], couple[2], couple[3],
        );
        let softjtag =
            SoftJtag::new(mtx.clone(), couple[0], couple[1], couple[2], couple[3]).unwrap();
        let ids = softjtag.scan_with(true).unwrap();
        if ids.iter().any(|x| x.is_some()) {
            println!("!!!!!! Found Devices:{ids:#x?}");
        }
    }
}
