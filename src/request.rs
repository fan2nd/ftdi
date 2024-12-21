#[derive(Debug)]
pub enum Request {
    Reset = 0,
    ModemCtrl = 0x01,
    FlowCtrl = 0x02,
    BaudRate = 0x03,
    SetData = 0x04,
    PollModemStatus = 0x05,
    SetEventChar = 0x06,
    SetErrorChar = 0x07,
    SetLatencyTimer = 0x09,
    GetLatencyTimer = 0x0a,
    SetBitMode = 0x0b,
    // eeprom opreation
    ReadEeprom = 0x90,
    WriteEeprom = 0x91,
    EraseEeprom = 0x92,
}
// 0x00
#[derive(Debug, Clone, Copy)]
pub enum RequestReset {
    Reset = 0,
    PurgeRxBuffer = 1,
    PurgeTxBuffer = 2,
}
/// 0x0b
/// mode<<8 | mask
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BitMode {
    Reset = 0x00,
    AsyncBitBang = 0x01,
    Mpsse = 0x02,
    SyncBitBang = 0x04,
    Mcu = 0x08,
    Opto = 0x10,
    Cbus = 0x20,
    SyncFifo = 0x40,
    Ft1284 = 0x80,
}
