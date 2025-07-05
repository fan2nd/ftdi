use std::sync::{Arc, Mutex};

use ftdi_hal::{FtMpsse, Interface, Pin, SoftJtag, list_all_device};
use itertools::Itertools;

fn main() {
    let devices = list_all_device();
    let mpsse = FtMpsse::open(devices[0].clone(), Interface::A, 0).unwrap();
    let mtx = Arc::new(Mutex::new(mpsse));
    let pins = [Pin::Lower(2), Pin::Lower(3), Pin::Lower(0), Pin::Lower(1)];
    for couple in pins.into_iter().permutations(4) {
        println!(
            "testing: tck:{:?},tdi:{:?},tdo:{:?},tms:{:?}",
            couple[0], couple[1], couple[2], couple[3],
        );
        let mut softjtag =
            SoftJtag::new(mtx.clone(), couple[0], couple[1], couple[2], couple[3]).unwrap();
        let ids = softjtag.scan().unwrap();
        if !ids.is_empty() {
            println!("!!!!!! Find Devices:{ids:x?}");
        }
    }
}
