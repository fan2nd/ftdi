use embedded_hal::spi::{SpiDevice, MODE_0};
use ftdi::{
    command::Command, device::FtdiDevice, interface::FtdiInterfaceEnum, request::BitMode,
    spi::FtdiSpi,
};

fn main() {
    let devices = FtdiDevice::list_all();
    let mut mpsse = devices[0]
        .open(FtdiInterfaceEnum::A, BitMode::Mpsse)
        .unwrap();
    mpsse.set_speed(15_000_000).unwrap();
    let mut spi = FtdiSpi::new(mpsse, MODE_0).unwrap();
    spi.interface
        .immidiate_write(&[Command::EnableLoopBack.into()], 0);
    const DATA_COUNT: usize = 65535;

    let mut write_buffer = [0u8; DATA_COUNT];
    for a in write_buffer.iter_mut() {
        *a = rand::random();
    }
    let now = std::time::Instant::now();
    let mut read_buffer = [0u8; DATA_COUNT];
    spi.transfer(&mut read_buffer, &write_buffer).unwrap();
    println!("{:?}", now.elapsed());
    println!("write_buffer {:x?}", &write_buffer[0..10]);
    println!("read_buffer  {:x?}", &read_buffer[0..10]);

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
