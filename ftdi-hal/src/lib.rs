//! This is an [embedded-hal] implementation for the FTDI chips
//! that use the [libftd2xx] or [ftdi-rs] drivers.
//!
//! This enables development of embedded device drivers without the use of
//! a microcontroller. The FTDI devices interface with a PC via USB, and
//! provide a multi-protocol synchronous serial engine to interface
//! with most GPIO, SPI, and I2C embedded devices.
//!
//! **Note:**
//! This is strictly a development tool.
//! The crate contains runtime borrow checks and explicit panics to adapt the
//! FTDI device into the [embedded-hal] traits.
//!
//! # Quickstart
//!
//! * Enable the "libftd2xx-static" feature flag to use static linking with libftd2xx driver.
//! * Linux users only: Add [udev rules].
//!
//! ```toml
//! [dependencies.ftdi-embedded-hal]
//! version = "0.23.2"
//! features = ["libftd2xx", "libftd2xx-static"]
//! ```
//!
//! # Limitations
//!
//! * Limited trait support: SPI, I2C, Delay, InputPin, and OutputPin traits are implemented.
//! * Limited device support: FT232H, FT2232H, FT4232H.
//! * Limited SPI modes support: MODE0, MODE2.
//!
//! # Examples
//!
//! ## SPI
//!
//! Pin setup:
//!
//! * D0 - SCK
//! * D1 - SDO (MOSI)
//! * D2 - SDI (MISO)
//! * D3..D7 - Available for CS
//!
//! Communicate with SPI devices using [ftdi-rs] driver:
//! ```no_run
//! use ftdi_embedded_hal as hal;
//!
//! # #[cfg(feature = "ftdi")]
//! # {
//! let device = ftdi::find_by_vid_pid(0x0403, 0x6010)
//!     .interface(ftdi::Interface::A)
//!     .open()?;
//!
//! let hal = hal::FtHal::init_freq(device, 3_000_000)?;
//! let spi = hal.spi()?;
//! # }
//! # Ok::<(), std::boxed::Box<dyn std::error::Error>>(())
//! ```
//!
//! Communicate with SPI devices using [libftd2xx] driver:
//! ```no_run
//! use ftdi_embedded_hal as hal;
//!
//! # #[cfg(feature = "libftd2xx")]
//! # {
//! let device = libftd2xx::Ft2232h::with_description("Dual RS232-HS A")?;
//!
//! let hal = hal::FtHal::init_freq(device, 3_000_000)?;
//! let spi = hal.spi()?;
//! # }
//! # Ok::<(), std::boxed::Box<dyn std::error::Error>>(())
//! ```
//!
//! ## I2C
//!
//! Communicate with I2C devices using [ftdi-rs] driver:
//! ```no_run
//! use ftdi_embedded_hal as hal;
//!
//! # #[cfg(feature = "ftdi")]
//! # {
//! let device = ftdi::find_by_vid_pid(0x0403, 0x6010)
//!     .interface(ftdi::Interface::A)
//!     .open()?;
//!
//! let hal = hal::FtHal::init_freq(device, 400_000)?;
//! let i2c = hal.i2c()?;
//! # }
//! # Ok::<(), std::boxed::Box<dyn std::error::Error>>(())
//! ```
//!
//! Communicate with I2C devices using [libftd2xx] driver:
//! ```no_run
//! use ftdi_embedded_hal as hal;
//!
//! # #[cfg(feature = "libftd2xx")]
//! # {
//! let device = libftd2xx::Ft232h::with_description("Single RS232-HS")?;
//!
//! let hal = hal::FtHal::init_freq(device, 400_000)?;
//! let i2c = hal.i2c()?;
//! # }
//! # Ok::<(), std::boxed::Box<dyn std::error::Error>>(())
//! ```
//!
//! ## GPIO
//!
//! Control GPIO pins using [libftd2xx] driver:
//! ```no_run
//! use ftdi_embedded_hal as hal;
//!
//! # #[cfg(feature = "libftd2xx")]
//! # {
//! let device = libftd2xx::Ft232h::with_description("Single RS232-HS")?;
//!
//! let hal = hal::FtHal::init_default(device)?;
//! let gpio = hal.d6();
//! # }
//! # Ok::<(), std::boxed::Box<dyn std::error::Error>>(())
//! ```
//!
//! Control GPIO pins using [ftdi-rs] driver:
//! ```no_run
//! use ftdi_embedded_hal as hal;
//!
//! # #[cfg(feature = "ftdi")]
//! # {
//! let device = ftdi::find_by_vid_pid(0x0403, 0x6010)
//!     .interface(ftdi::Interface::A)
//!     .open()?;
//!
//! let hal = hal::FtHal::init_default(device)?;
//! let gpio = hal.d6();
//! # }
//! # Ok::<(), std::boxed::Box<dyn std::error::Error>>(())
//! ```
//!
//! ## More examples
//!
//! * [newAM/eeprom25aa02e48-rs]: read data from Microchip 25AA02E48 SPI EEPROM
//! * [newAM/bme280-rs]: read samples from Bosch BME280 sensor via I2C protocol
//!
//! [embedded-hal]: https://github.com/rust-embedded/embedded-hal
//! [ftdi-rs]: https://github.com/tanriol/ftdi-rs
//! [libftd2xx crate]: https://github.com/ftdi-rs/libftd2xx-rs/
//! [libftd2xx]: https://github.com/ftdi-rs/libftd2xx-rs
//! [newAM/eeprom25aa02e48-rs]: https://github.com/newAM/eeprom25aa02e48-rs/blob/main/examples/ftdi.rs
//! [newAM/bme280-rs]: https://github.com/newAM/bme280-rs/blob/main/examples/ftdi-i2c.rs
//! [udev rules]: https://github.com/ftdi-rs/libftd2xx-rs/#udev-rules
//! [setup executable]: https://www.ftdichip.com/Drivers/CDM/CDM21228_Setup.zip
#![forbid(unsafe_code)]

pub use eh1;

mod ftdaye;
mod gpio;
mod i2c;
mod jtag;
mod list;
mod mpsse;
mod spi;
mod swd;

pub use crate::ftdaye::Interface;
use crate::{
    ftdaye::{ChipType, FtdiContext, FtdiError},
    mpsse::MpsseCmdBuilder,
};
pub use gpio::{InputPin, OutputPin};
pub use i2c::I2c;
pub use jtag::SoftJtag;
pub use list::list_all_device;
pub use spi::Spi;
pub use swd::{Swd, SwdOp, SwdPort};

/// Order
#[derive(Debug, Clone, Copy)]
pub enum BitOrder {
    /// Bit0 first
    Lsb,
    /// Bit7 first
    Msb,
}
/// Pin number
#[derive(Debug, Copy, Clone)]
pub enum Pin {
    Lower(usize),
    Upper(usize),
}
/// State tracker for each pin on the FTDI chip.
#[derive(Debug, Clone, Copy)]
enum PinUse {
    Output,
    Input,
    I2c,
    Spi,
    Jtag,
    Swd,
}
/// State tracker for each pin on the FTDI chip.
#[derive(Debug, Clone, Copy)]
enum PinDirection {
    Input = 0,
    Output = 1,
}
#[derive(Debug, Default)]
struct GpioByte {
    /// GPIO direction. 0 for input and 1 for output
    direction: u8,
    /// GPIO value.
    value: u8,
    /// Pin allocation.
    pins: [Option<PinUse>; 8],
}

pub struct FtMpsse {
    /// FTDI device.
    ft: FtdiContext,
    chip_type: ChipType,
    lower: GpioByte,
    upper: GpioByte,
}

impl FtMpsse {
    pub fn open(
        usb_device: nusb::DeviceInfo,
        interface: Interface,
        mask: u8,
    ) -> Result<Self, FtdiError> {
        let handle = usb_device.open()?;
        let max_packet_size = {
            let interface_alt_settings: Vec<_> = handle
                .active_configuration()?
                .interface_alt_settings()
                .collect();
            let endpoints: Vec<_> = interface_alt_settings[interface as usize - 1]
                .endpoints()
                .collect();
            endpoints[0].max_packet_size()
        };
        let chip_type = match (
            usb_device.device_version(),
            usb_device.serial_number().unwrap_or(""),
        ) {
            (0x400, _) | (0x200, "") => ChipType::Bm,
            (0x200, _) => ChipType::Am,
            (0x500, _) => ChipType::FT2232C,
            (0x600, _) => ChipType::R,
            (0x700, _) => ChipType::FT2232H,
            (0x800, _) => ChipType::FT4232H,
            (0x900, _) => ChipType::FT232H,
            (0x1000, _) => ChipType::FT230X,

            (version, _) => {
                return Err(FtdiError::Other(format!(
                    "Unkonwn ChipType version:0x{version:x}",
                )));
            }
        };

        let handle = handle.detach_and_claim_interface(interface as u8 - 1)?;

        let mut this = Self {
            ft: FtdiContext::new(handle, interface, max_packet_size).into_mpsse(mask)?,
            chip_type,
            lower: Default::default(),
            upper: Default::default(),
        };
        let cmd = MpsseCmdBuilder::new()
            .set_gpio_lower(this.lower.value, this.lower.direction)
            .set_gpio_upper(this.upper.value, this.upper.direction)
            .send_immediate();
        this.write_read(cmd.as_slice(), &mut [])?;
        Ok(this)
    }
    /// Write mpsse command and read response
    fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), FtdiError> {
        self.ft.write_read(write, read)
    }
    /// Allocate a pin for a specific use.
    fn alloc_pin(&mut self, pin: Pin, purpose: PinUse) {
        let (byte, idx) = match pin {
            Pin::Lower(idx) => (&mut self.lower, idx),
            Pin::Upper(idx) => (&mut self.upper, idx),
        };
        assert!(idx < 8, "Pin index {idx} is out of range 0 - 7");

        if let Some(current) = byte.pins[idx] {
            panic!(
                "Unable to allocate pin {pin:?} for {purpose:?}, pin is already allocated for {current:?}"
            );
        } else {
            byte.pins[idx] = Some(purpose)
        }
    }
    /// Allocate a pin for a specific use.
    fn free_pin(&mut self, pin: Pin) {
        match pin {
            Pin::Lower(idx) => {
                assert!(idx < 8, "Pin index {idx} is out of range 0 - 7");
                self.lower.pins[idx] = None;
                self.lower.value &= !(1 << idx);
                self.lower.direction &= !(1 << idx);
                let cmd = MpsseCmdBuilder::new()
                    .set_gpio_lower(self.lower.value, self.lower.direction)
                    .send_immediate();
                self.write_read(cmd.as_slice(), &mut []).unwrap();
            }
            Pin::Upper(idx) => {
                assert!(idx < 8, "Pin index {idx} is out of range 0 - 7");
                self.lower.pins[idx] = None;
                self.upper.value &= !(1 << idx);
                self.upper.direction &= !(1 << idx);
                let cmd = MpsseCmdBuilder::new()
                    .set_gpio_lower(self.upper.value, self.upper.direction)
                    .send_immediate();
                self.write_read(cmd.as_slice(), &mut []).unwrap();
            }
        };
    }
}
