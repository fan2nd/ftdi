use crate::{command::Command, request::Request};
#[derive(Debug, thiserror::Error)]
pub enum FtdiError {
    #[error("Unkonwn chip type")]
    UnknownChipType,
    #[error("Device open failed")]
    DeviceOpenFailed,
    #[error("Interface open failed")]
    InterfaceOpenFailed,
    #[error("Request:{0:?} failed")]
    RequestError(Request),
    #[error("Command:{0:?} failed")]
    CommandError(Command),
    #[error("Usb Transfer:{0} failed")]
    TransferError(nusb::transfer::TransferError),
}
