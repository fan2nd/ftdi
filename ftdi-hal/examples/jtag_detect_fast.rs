use std::{
    sync::{Arc, Mutex},
    time::Instant,
};

use ftdi_hal::{FtMpsse, JtagDetectTdi, JtagDetectTdo, list_all_device};
use itertools::Itertools;

fn main() {
    let now = Instant::now();
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtMpsse::open(&devices[0].usb_device, devices[0].interface[0], 0).unwrap();
    let mtx = Arc::new(Mutex::new(mpsse));
    let pins = [0, 1, 2, 3, 4, 5, 6, 7];
    let mut notdi = Vec::new();
    // find tdo
    for couple in pins.into_iter().permutations(2) {
        let tck = couple[0];
        let tms = couple[1];
        let jtag = JtagDetectTdo::new(mtx.clone(), tck, tms).unwrap();
        let ids_scan = jtag.scan().unwrap();
        for &tdo in pins.iter() {
            if tdo == tck || tdo == tms {
                continue;
            }
            if ids_scan[tdo].iter().any(Option::is_some) {
                println!("Pins:tck[{tck}],tdo[{tdo}],tms[{tms}] maybe true");
                notdi.push((tck, tms, tdo));
            }
        }
    }
    // find tdi
    for (tck, tms, tdo) in notdi {
        for &tdi in pins.iter() {
            if tdi == tck || tdi == tms || tdi == tdo {
                continue;
            }
            let jtag = JtagDetectTdi::new(mtx.clone(), tck, tdi, tdo, tms).unwrap();
            let ids_scan1 = jtag.scan_with(1).unwrap();
            let ids_scan0 = jtag.scan_with(0).unwrap();
            if ids_scan0.len() > ids_scan1.len() {
                println!("!!!!!! Pins:tck[{tck}],tdi[{tdi}],tdo[{tdo}],tms[{tms}]");
                println!("!!!!!! Found Devices:{ids_scan1:x?}");
            }
        }
    }
    println!("Finish Detect Using {:?}", now.elapsed());
}
