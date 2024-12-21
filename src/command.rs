use bitfield_struct::bitfield;

#[bitfield(u8,order = Lsb)]
pub struct CommandShift {
    pub ne_clk_write: bool,
    pub bit_mode: bool,
    pub ne_clk_read: bool,
    pub lsb_first: bool,
    pub write_tdi: bool,
    pub read_tdo: bool,
    pub write_tms: bool,
    __: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum Command {
    Shift(CommandShift),
    // bits value&direction
    SetBitsLow,  // 0x80
    SetBitsHigh, // 0x82
    GetBitsLow,  // 0x81
    GetBitsHigh, // 0x83
    // TDI&TDO loopback
    EnableLoopBack, // 0x84
    DisLoopBack,    // 0x85
    // Clk disvior
    SetClkDivisor, // 0x86
    // CPU mode
    ReadShortAddr,   // 0x90
    ReadExtendAddr,  // 0x91
    WriteShortAddr,  // 0x92
    WriteExtendAddr, // 0x93
    // Both mpsse and mcuhost
    SendImmediate, // 0x87
    WaitOnIOHigh,  // 0x88
    WaitOnIoLow,   // 0x89
    // FT232H,FT2232H,FT4232H
    DisableClkDivide5,                                   // 0x8a
    EnableClkDivide5,                                    // 0x8b
    Enable3PhaseDataClock,                               // 0x8c
    Disable3PhaseDataClock,                              // 0x8d
    ClockForNBitsWithNoDataTransfer,                     // 0x8e
    ClockForNBytesWithNoDataTransfer,                    // 0x8f
    ClkContinuouslyWaitOnIoHigh,                         // 0x94
    ClkContinuouslyWaitOnIoLow,                          // 0x95
    EnableAdaptiveClocking,                              // 0x96
    DisableAdaptiveClocking,                             // 0x97
    ClockForNBytesWithNoDataTransferOrUntilGpioL1IsHigh, // 0x9c
    ClockForNBytesWithNoDataTransferOrUntilGpioL1IsLow,  // 0x9d
    // FT232
    SetIoOnlyDriveOn0, // 0x9e
    // Not Known
    Unknown(u8),
}

impl Into<u8> for Command {
    fn into(self) -> u8 {
        match self {
            Self::Shift(x) => x.into_bits(),
            Self::SetBitsLow => 0x80,
            Self::GetBitsLow => 0x81,
            Self::SetBitsHigh => 0x82,
            Self::GetBitsHigh => 0x83,
            Self::EnableLoopBack => 0x84,
            Self::DisLoopBack => 0x85,
            Self::SetClkDivisor => 0x86,
            Self::SendImmediate => 0x87,
            Self::WaitOnIOHigh => 0x88,
            Self::WaitOnIoLow => 0x89,
            Self::ReadShortAddr => 0x90,
            Self::ReadExtendAddr => 0x91,
            Self::WriteShortAddr => 0x92,
            Self::WriteExtendAddr => 0x93,
            Self::DisableClkDivide5 => 0x8a,
            Self::EnableClkDivide5 => 0x8b,
            Self::Enable3PhaseDataClock => 0x8c,
            Self::Disable3PhaseDataClock => 0x8d,
            Self::ClockForNBitsWithNoDataTransfer => 0x8e,
            Self::ClockForNBytesWithNoDataTransfer => 0x8f,
            Self::ClkContinuouslyWaitOnIoHigh => 0x94,
            Self::ClkContinuouslyWaitOnIoLow => 0x95,
            Self::EnableAdaptiveClocking => 0x96,
            Self::DisableAdaptiveClocking => 0x97,
            Self::ClockForNBytesWithNoDataTransferOrUntilGpioL1IsHigh => 0x9c,
            Self::ClockForNBytesWithNoDataTransferOrUntilGpioL1IsLow => 0x9d,
            Self::SetIoOnlyDriveOn0 => 0x9e,
            Self::Unknown(_) => 0xff,
        }
    }
}

impl From<u8> for Command {
    fn from(value: u8) -> Self {
        match value {
            x if x < 0x80 => Self::Shift(CommandShift(x)),
            0x80 => Self::SetBitsLow,
            0x81 => Self::GetBitsLow,
            0x82 => Self::SetBitsHigh,
            0x83 => Self::GetBitsHigh,
            0x84 => Self::EnableLoopBack,
            0x85 => Self::DisLoopBack,
            0x86 => Self::SetClkDivisor,
            0x87 => Self::SendImmediate,
            0x88 => Self::WaitOnIOHigh,
            0x89 => Self::WaitOnIoLow,
            0x90 => Self::ReadShortAddr,
            0x91 => Self::ReadExtendAddr,
            0x92 => Self::WriteShortAddr,
            0x93 => Self::WriteExtendAddr,
            0x8a => Self::DisableClkDivide5,
            0x8b => Self::EnableClkDivide5,
            0x8c => Self::Enable3PhaseDataClock,
            0x8d => Self::Disable3PhaseDataClock,
            0x8e => Self::ClockForNBitsWithNoDataTransfer,
            0x8f => Self::ClockForNBytesWithNoDataTransfer,
            0x94 => Self::ClkContinuouslyWaitOnIoHigh,
            0x95 => Self::ClkContinuouslyWaitOnIoLow,
            0x96 => Self::EnableAdaptiveClocking,
            0x97 => Self::DisableAdaptiveClocking,
            0x9c => Self::ClockForNBytesWithNoDataTransferOrUntilGpioL1IsHigh,
            0x9d => Self::ClockForNBytesWithNoDataTransferOrUntilGpioL1IsLow,
            0x9e => Self::SetIoOnlyDriveOn0,
            x => Self::Unknown(x),
        }
    }
}

// pub const MsbBytesOutP:u8 = 0x10;
// pub const MsbBytesOutN:u8 = 0x11;
// pub const MsbBitsOutP:u8 = 0x12;
// pub const MsbBitsOutN:u8 = 0x13;
// pub const MsbBytesInP:u8 = 0x20;
// pub const MsbBytesInN:u8 = 0x24;
// pub const MsbBitsInP:u8 = 0x22;
// pub const MsbBitsInN:u8 = 0x26;
// pub const MsbBytesOutNInP:u8 = 0x31;
// pub const MsbBytesOutPInN:u8 = 0x34;
// pub const MsbBitsOutNInP:u8 = 0x33;
// pub const MsbBitsOutPInN:u8 = 0x36;
// // // LSB shift
// pub const LsbBytesOutP:u8 = 0x18;
// pub const LsbBytesOutN:u8 = 0x19;
// pub const LsbBitsOutP:u8 = 0x1a;
// pub const LsbBitsOutN:u8 = 0x1b;
// pub const LsbBytesInP:u8 = 0x28;
// pub const LsbBytesInN:u8 = 0x2c;
// pub const LsbBitsInP:u8 = 0x2a;
// pub const LsbBitsInN:u8 = 0x2e;
// pub const LsbBytesOutNInP:u8 = 0x39;
// pub const LsbBytesOutPInN:u8 = 0x3c;
// pub const LsbBitsOutNInP:u8 = 0x3b;
// pub const LsbBitsOutPInN:u8 = 0x3e;
// // // TMS shift
// pub const LsbTmsOutP:u8 = 0x4a;
// pub const LsbTmsOutN:u8 = 0x4b;
// pub const LsbTmsOutPInP:u8 = 0x6a;
// pub const LsbTmsOutPInN:u8 = 0x6b;
// pub const LsbTmsOutNInP:u8 = 0x6e;
// pub const LsbTmsOutNInN:u8 = 0x6f;
