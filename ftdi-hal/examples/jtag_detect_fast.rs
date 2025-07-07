use std::{
    sync::{Arc, Mutex},
    time::Instant,
};

use ftdi_hal::{FtMpsse, Interface, JtagDetect, JtagScan, list_all_device};
use itertools::Itertools;

fn main() {
    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");
    let mpsse = FtMpsse::open(devices[0].clone(), Interface::A, 0).unwrap();
    let mtx = Arc::new(Mutex::new(mpsse));
    let pins = [0, 1, 2, 3, 4, 5, 6, 7];
    let mut notdi = Vec::new();
    let now = Instant::now();
    for couple in pins.into_iter().permutations(2) {
        let tck = couple[0];
        let tms = couple[1];
        let jtag = JtagDetect::new(mtx.clone(), tck, tms).unwrap();
        let ids_scan = jtag.scan().unwrap();
        for tdo in 0..8 {
            if tdo == tck | tms {
                continue;
            }
            if ids_scan[tdo].iter().any(Option::is_some) {
                println!("tck:{},tms:{},tdo:{} maybe true", tck, tms, tdo);
                notdi.push((tck, tms, tdo));
            }
        }
    }
    for (tck, tms, tdo) in notdi {
        for tdi in 0..8 {
            if tdi == tck || tdi == tms || tdi == tdo {
                continue;
            }
            let jtag = JtagScan::new(mtx.clone(), tck, tdi, tdo, tms).unwrap();
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
