use embedded_hal::spi::{SpiDevice, MODE_0};
use ftdi::{device::FtdiDevice, interface::FtdiInterfaceEnum, request::BitMode, spi::FtdiSpi};

fn main() {
    let devices = FtdiDevice::list_all();
    let mpsse = devices[0]
        .open(FtdiInterfaceEnum::A, BitMode::Mpsse)
        .unwrap();
    mpsse.set_speed(15_000_000).unwrap();
    let mut spi = FtdiSpi::new(mpsse, MODE_0).unwrap();

    const DATA_COUNT: usize = 65535;

    let mut write_buffer = [0u8; DATA_COUNT];
    for a in write_buffer.iter_mut() {
        *a = rand::random();
    }

    let mut read_buffer = [0u8; DATA_COUNT];
    spi.transfer(&mut read_buffer, &write_buffer).unwrap();
    println!("{:x?}", &read_buffer[0..10]);

    // verify
    for index in 0..DATA_COUNT {
        if write_buffer[index] != read_buffer[index] {
            println!(
                "index-{}: write{} != read:{}",
                index, write_buffer[index], read_buffer[index]
            );
            break;
        }
    }
}
