use crate::{
    command::{Command, CommandShift},
    error::FtdiError,
    request::{BitMode, Request, RequestReset},
};
use futures_lite::future::block_on;
use nusb::{
    transfer::{Control, ControlType, Recipient, RequestBuffer},
    Interface,
};
use std::time::Duration;

#[derive(Debug, Clone, Copy)]
pub enum FtdiInterfaceEnum {
    A = 1,
    B = 2,
    C = 3,
    D = 4,
}
impl FtdiInterfaceEnum {
    const fn index(&self) -> u16 {
        *self as u16
    }
    pub const fn interface_number(&self) -> u8 {
        (self.index() - 1) as u8
    }
    const fn read_ep(&self) -> u8 {
        (self.index() + 0x80) as u8
    }
    const fn write_ep(&self) -> u8 {
        (self.index() * 2) as u8
    }
}

pub struct FtdiInterface {
    interface: Interface,
    index: FtdiInterfaceEnum,
    pub bitmode: BitMode,
    max_packet_size: usize,
    fifo_size: usize,

    tx_buffer: Vec<u8>,
    response_info: Vec<usize>,
}

impl FtdiInterface {
    pub fn new(
        interface: Interface,
        index: FtdiInterfaceEnum,
        bitmode: BitMode,
        max_packet_size: usize,
        fifo_size: usize,
    ) -> Result<FtdiInterface, FtdiError> {
        let this = Self {
            interface,
            index,
            bitmode,
            max_packet_size,
            fifo_size,
            tx_buffer: Default::default(),
            response_info: Default::default(),
        };
        this.reset()?;
        this.set_latency_timer(0xff)?;
        this.set_bitmode(BitMode::Mpsse)?;
        this.reset_rx()?;
        this.reset_tx()?;
        Ok(this)
    }

    pub fn write_request(&self, request: u8, value: u16, data: &[u8]) -> Result<usize, FtdiError> {
        println!("0x{request:x}-0x{value:x}");
        Ok(self
            .interface
            .control_out_blocking(
                Control {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: request,
                    value: value,
                    index: self.index.index() as u16,
                },
                data,
                Duration::from_millis(50),
            )
            .map_err(|e| FtdiError::TransferError(e))?)
    }
    pub fn write(&self, data: &[u8]) -> Result<(), FtdiError> {
        // println!("write ep[{:x}],data:{:02x?}", self.index.write_ep(), data);
        block_on(self.interface.bulk_out(self.index.write_ep(), data.into()))
            .into_result()
            .map_err(|e| FtdiError::TransferError(e))?;
        Ok(())
    }
    pub fn read(&self) -> Result<Vec<u8>, FtdiError> {
        let result = block_on(
            self.interface
                .bulk_in(self.index.read_ep(), RequestBuffer::new(512)),
        )
        .into_result()
        .map_err(|e| FtdiError::TransferError(e))?;
        // println!("read ep[{:x}],data:{:02x?}", self.index.read_ep(), result);
        Ok(result)
    }
}

impl FtdiInterface {
    pub fn reset(&self) -> Result<(), FtdiError> {
        self.write_request(Request::Reset as u8, RequestReset::Reset as u16, &[])?;
        Ok(())
    }
    pub fn reset_rx(&self) -> Result<(), FtdiError> {
        self.write_request(
            Request::Reset as u8,
            RequestReset::PurgeRxBuffer as u16,
            &[],
        )?;
        Ok(())
    }
    pub fn reset_tx(&self) -> Result<(), FtdiError> {
        self.write_request(
            Request::Reset as u8,
            RequestReset::PurgeTxBuffer as u16,
            &[],
        )?;
        Ok(())
    }
    pub fn set_latency_timer(&self, time: u8) -> Result<(), FtdiError> {
        self.write_request(Request::SetLatencyTimer as u8, time as u16, &[])?;
        Ok(())
    }
    pub fn set_bitmode(&self, mode: BitMode) -> Result<(), FtdiError> {
        self.write_request(Request::SetBitMode as u8, (mode as u16) << 8, &[])?;
        Ok(())
    }
}

impl FtdiInterface {
    pub fn set_speed(&self, speed_hz: usize) -> Result<(), FtdiError> {
        let mut cmd = Vec::new();
        let mut base_speed = 30_000_000usize;
        if speed_hz < 6_000_000 {
            cmd.push(Command::EnableClkDivide5.into());
            base_speed = 6_000_000;
        } else {
            cmd.push(Command::DisableClkDivide5.into());
        }
        let mut div: u16 = 0;
        while speed_hz < base_speed {
            base_speed /= 2;
            div += 1;
        }
        cmd.push(Command::SetClkDivisor.into());
        cmd.push(div as u8);
        cmd.push((div >> 8) as u8);
        self.write(&cmd)?;
        Ok(())
    }
    pub fn common_setting(&mut self) {
        // 85,97,8d
        // disable loopback
        // TurnOffnAdaptiveClocking
        // disable 3 phase
        self.schedule_write(&[Command::DisLoopBack.into()]);
        self.schedule_write(&[Command::DisableAdaptiveClocking.into()]);
        self.schedule_write(&[Command::Disable3PhaseDataClock.into()]);
    }
    pub fn schedule_write(&mut self, command: &[u8]) {
        // println!("cmd:{:x?}", command);
        let mut will_receive = if let Some(len) = self.response_info.last() {
            *len
        } else {
            0
        };
        if command[0] & 0x80 == 0 {
            self.tx_buffer.push(command[0]);
            self.response_info.push(will_receive);
            // shift command
            let shift_cmd = CommandShift::from(command[0]);
            if shift_cmd.bit_mode() {
                self.tx_buffer.push(command[1]);
                if shift_cmd.write_tdi() || shift_cmd.write_tms() {
                    debug_assert_eq!(command.len(), 3);
                    self.response_info.push(will_receive);
                    self.tx_buffer.push(command[2]);
                    let read_on_write = shift_cmd.read_tdo() as usize;
                    self.response_info.push(will_receive + read_on_write);
                } else {
                    // only read
                    debug_assert_eq!(command.len(), 2);
                    self.response_info.push(will_receive + 1);
                }
            } else {
                let rw_len: usize = command[1] as usize + ((command[2] as usize) << 8) + 1;
                self.tx_buffer.push(command[1]);
                self.tx_buffer.push(command[2]);
                self.response_info.push(will_receive);
                if shift_cmd.write_tdi() {
                    debug_assert_eq!(command.len(), rw_len + 3);
                    self.response_info.push(will_receive);
                    let read_on_write = shift_cmd.read_tdo() as usize;
                    for i in 0..rw_len {
                        self.tx_buffer.push(command[3 + i]);
                        will_receive += read_on_write;
                        self.response_info.push(will_receive);
                    }
                } else {
                    debug_assert_eq!(command.len(), 3);
                    self.response_info.push(will_receive + rw_len);
                }
            }
        } else if command[0] == Command::SetClkDivisor.into() {
            // set clk
            debug_assert_eq!(command.len(), 3);
            self.tx_buffer.extend_from_slice(command);
            self.response_info.extend_from_slice(&[will_receive; 3]);
        } else if command[0] & 0b11111100 == 0x80 {
            // get set bits
            if command[0] & 0x01 == 0 {
                // set
                debug_assert_eq!(command.len(), 3);
                self.tx_buffer.extend_from_slice(command);
                self.response_info.extend_from_slice(&[will_receive; 3]);
            } else {
                // get
                debug_assert_eq!(command.len(), 2);
                self.tx_buffer.extend_from_slice(command);
                self.response_info
                    .extend_from_slice(&[will_receive, will_receive + 1]);
            }
        } else if command[0] & 0b11111100 == 0x90 {
            // cpu mode
            todo!()
        } else {
            // only command
            debug_assert_eq!(command.len(), 1);
            self.tx_buffer.push(command[0]);
            self.response_info.push(will_receive);
        }
    }
    pub fn read_result(&mut self) -> Result<Vec<u8>, FtdiError> {
        debug_assert_eq!(self.tx_buffer.len(), self.response_info.len());
        // println!("response_info: {:?}",self.response_info);
        let response_len = if let Some(len) = self.response_info.last() {
            *len
        } else {
            0
        };
        let fifo_len = self.fifo_size;
        let mut tx_fifo_used = 0;
        let mut cmd_slice: &[u8] = &self.tx_buffer;
        let mut response = Vec::new();
        let mut response_len_index = 0;

        while response.len() < response_len {
            // fill txfifo
            println!("fill txfifo");
            while tx_fifo_used < fifo_len && cmd_slice.len() > 0 {
                let mut this_packet_size = if self.max_packet_size > (fifo_len - tx_fifo_used) {
                    fifo_len - tx_fifo_used
                } else {
                    self.max_packet_size
                };
                this_packet_size = if this_packet_size > cmd_slice.len() {
                    cmd_slice.len()
                } else {
                    this_packet_size
                };
                self.write(&cmd_slice[0..this_packet_size])?;
                cmd_slice = &cmd_slice[this_packet_size..];
                tx_fifo_used += this_packet_size;
                print!("tx_fifo_used:{tx_fifo_used}-");
                println!("cmd_slice_len:{}", cmd_slice.len());
            }
            // read response
            println!("read response");
            let this_response = self.read()?;
            if this_response[0] == 0xff {
                return Err(FtdiError::CommandError(Command::from(this_response[1])));
            }
            response.extend_from_slice(&this_response[2..]);
            // cacluraue tx fifo status
            println!("cacluraue tx fifo status");
            while self.response_info[response_len_index] < response.len() {
                response_len_index += 1;
                if tx_fifo_used > 0 {
                    tx_fifo_used -= 1;
                }
            }
        }
        self.tx_buffer.clear();
        self.response_info.clear();
        Ok(response)
    }
}
